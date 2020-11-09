#include <linux/init.h>
#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include <linux/gpio/consumer.h>
#include <linux/of_platform.h>
#include <linux/err.h>
#include <linux/of.h>
#include <linux/delay.h>
#include <asm/irq.h>
#include <linux/cdev.h>
#include <linux/miscdevice.h>

#define DEVICE_NAME "rht03"
#define IOCTL_TRIG 7000
#define IOCTL_READ 7001 

#define SHORT_PULSE_NS 80000
#define LONG_PULSE_NS 120000
#define TOLERANCE 15000
#define TOLERANCE_HIGH 30000
#define PATTERN_LENGTH 32 

struct rht03_gpio_platform_data; 

static int rht03_gpio_probe(struct platform_device *pdev); 
static int rht03_gpio_remove(struct platform_device *pdev); 
static long rht03_ioctl(struct file*, unsigned int, unsigned long);

static int init_device(struct platform_device *,
		struct rht03_gpio_platform_data **);
static void trig_reading(struct rht03_gpio_platform_data *pdata);
static irqreturn_t gpio_irq_handler(int, void *);

struct rht03_gpio_platform_data {
	struct gpio_desc *gpiod;
	struct gpio_desc *pullup_gpiod;
	void (*enable_external_pullup)(int enable);
	unsigned int pullup_duration;
	int index;
	u32 values;
	u64 previous_timestamp;
	u32 current_value;
	int current_valid;
	int irq;
        struct miscdevice mdev;
        struct platform_device *pdev;
};

struct file_operations rht03_fops =
{
	.unlocked_ioctl = rht03_ioctl
};

#if defined(CONFIG_OF)
static const struct of_device_id rht03_gpio_dt_ids[] = {
	{ .compatible = "rht03-gpio" },
	{}
};
MODULE_DEVICE_TABLE(of, rht03_gpio_dt_ids);
#endif

static struct platform_driver rht03_gpio_driver = {
	.driver = {
		.name	= "rht03-gpio",
		.of_match_table = of_match_ptr(rht03_gpio_dt_ids),
	},
	.probe = rht03_gpio_probe,
	.remove = rht03_gpio_remove,
};

static int next_node;
static dev_t device_number;
struct class *rht03_class;
struct cdev rht03_cdev;

static int rht03_gpio_probe(struct platform_device *pdev)
{
	int result, ret;
	struct rht03_gpio_platform_data *pdata;

	result = init_device(pdev, &pdata);
	if (result != 0) {
		return result;
	}
	platform_set_drvdata(pdev, pdata);
        pdata->pdev = pdev;

        pdata->mdev.minor = MISC_DYNAMIC_MINOR;
        pdata->mdev.name = "rht03";
        pdata->mdev.fops = &rht03_fops;
        pdata->mdev.parent = NULL; 

        ret = misc_register(&pdata->mdev);
        if (ret) {
                dev_err(&pdev->dev, "Failed to register rht03 miscdev\n");
        }

        dev_info(&pdev->dev, "Registered\n");

	return 0;
}

static int rht03_gpio_remove(struct platform_device *pdev)
{
	struct rht03_gpio_platform_data *pdata = dev_get_platdata(&pdev->dev);

        misc_deregister(&pdata->mdev);

	if (pdata->enable_external_pullup)
		pdata->enable_external_pullup(0);

	if (pdata->pullup_gpiod)
		gpiod_set_value(pdata->pullup_gpiod, 0);


	return 0;
} 

static int init_device(struct platform_device *pdev,
		struct rht03_gpio_platform_data **pdata_out)
{
	struct rht03_gpio_platform_data *pdata;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;

	/* Enforce open drain mode by default */
	enum gpiod_flags gflags = GPIOD_OUT_HIGH_OPEN_DRAIN;


	if (of_have_populated_dt()) {
		pdata = devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
		if (!pdata)
			return -ENOMEM;

		/*
		 * This parameter means that something else than the gpiolib has
		 * already set the line into open drain mode, so we should just
		 * driver it high/low like we are in full control of the line and
		 * open drain will happen transparently.
		 */
		if (of_get_property(np, "linux,open-drain", NULL))
			gflags = GPIOD_OUT_HIGH;

		pdev->dev.platform_data = pdata;
	}
	pdata = dev_get_platdata(dev);

	if (!pdata) {
		dev_err(dev, "No configuration data\n");
		return -ENXIO;
	}

	*pdata_out = pdata;


	pdata->gpiod = devm_gpiod_get_index(dev, NULL, 0, gflags);
	if (IS_ERR(pdata->gpiod)) {
		dev_err(dev, "gpio_request (pin) failed\n");
		return PTR_ERR(pdata->gpiod);
	}

	pdata->pullup_gpiod =
		devm_gpiod_get_index_optional(dev, NULL, 1, GPIOD_OUT_LOW);
	if (IS_ERR(pdata->pullup_gpiod)) {
		dev_err(dev, "gpio_request_one "
			"(ext_pullup_enable_pin) failed\n");
		return PTR_ERR(pdata->pullup_gpiod);
	}

	pdata->irq = gpiod_to_irq(pdata->gpiod);

	return devm_request_irq(&pdev->dev,
					pdata->irq, gpio_irq_handler,
					IRQF_TRIGGER_RISING, pdev->name,
					pdata);
}

static void trig_reading(struct rht03_gpio_platform_data *pdata)
{
	int high, i;
	pdata->index = -3;
	pdata->previous_timestamp = 0;
	pdata->values = 0;
	pdata->current_valid = 0;

	gpiod_direction_output(pdata->gpiod, 1);

	high = 0;
	for (i = 0; i < 2; ++i)
	{
		gpiod_set_value(pdata->gpiod, high);
		high = !high;
		udelay(1100);
	}

	gpiod_direction_input(pdata->gpiod);

}
static irqreturn_t gpio_irq_handler(int irq, void *dev_id)
{
	u64 timestamp = ktime_get_ns();
	u64 timediff;
	struct rht03_gpio_platform_data *pdata = dev_id;

	if (dev_id == NULL) {
		printk("IRQ dev_id==NULL\n");
		goto out;
	}

	if (pdata->index >= 0 && pdata->index < PATTERN_LENGTH-1) {
		timediff = timestamp - pdata->previous_timestamp;

		if (SHORT_PULSE_NS - TOLERANCE < timediff && 
			timediff < SHORT_PULSE_NS + TOLERANCE) {
			// Do nothing
		}
		else if (LONG_PULSE_NS - TOLERANCE < timediff &&
			timediff < LONG_PULSE_NS + TOLERANCE_HIGH) {
			pdata->values |= (1<<(PATTERN_LENGTH - 1 - pdata->index));
		}
		else {
			printk("? diff=%llu\n", timediff);
			printk("Invalid pulse length\n");
			pdata->current_valid = -1;
		}
	}
	else if (pdata->index == PATTERN_LENGTH-1) {
		if (pdata->current_valid == 0) {
			pdata->current_value = pdata->values;
			pdata->current_valid = 1;
			printk("\nFINAL VALUE: 0x%x\n", pdata->values);
		}
		else {
			printk("\nInvalid value: 0x%x\n", pdata->values);
		}
	}

	pdata->index++;
	pdata->previous_timestamp = timestamp;
out:
	return IRQ_HANDLED;
}

static long rht03_ioctl(struct file *filp,
	unsigned int ioctl_num,
	unsigned long ioctl_param)
{
        struct miscdevice *miscdev = filp->private_data;
        struct rht03_gpio_platform_data *pdata = container_of(miscdev, struct rht03_gpio_platform_data, mdev);
        
        printk("ioctl_num: %x\n", ioctl_num);

        if (ioctl_num == IOCTL_TRIG) {
                trig_reading(pdata);
                return 0;
        } else if (ioctl_num  == IOCTL_READ) {
                printk("current_valid: %x\n", pdata->current_valid);
                if (pdata->current_valid == 1) {
                        printk("current_value: %x\n", pdata->current_value);
                        return pdata->current_value;
                } else {
                        return -EIO;
                }
        }

        return -EINVAL;

}

static int rht03_init(void)
{
	next_node = 0;
	alloc_chrdev_region(&device_number, 0, 1, "gpio-rht03");
	rht03_class = class_create(THIS_MODULE, "rht03_class");

	cdev_init(&rht03_cdev, &rht03_fops);
	rht03_cdev.owner = THIS_MODULE;
	cdev_add(&rht03_cdev, device_number, 1);

	platform_driver_register(&rht03_gpio_driver);
	return 0;
}

static void rht03_exit(void)
{
        class_destroy(rht03_class);
	unregister_chrdev_region(device_number, 1); 
	platform_driver_unregister(&rht03_gpio_driver);
}

module_init(rht03_init);
module_exit(rht03_exit);

MODULE_DESCRIPTION("RHT03 GPIO driver");
MODULE_AUTHOR("William Arvidsson");
MODULE_LICENSE("GPL");

