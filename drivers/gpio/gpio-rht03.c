
#include <linux/init.h>
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

#define DEVICE_NAME "rht03"

#define SHORT_PULSE_NS 80000
#define LONG_PULSE_NS 120000
#define TOLERANCE 15000
#define TOLERANCE_HIGH 30000
#define PATTERN_LENGTH 32 

static int rht03_gpio_probe(struct platform_device *pdev); 
static int rht03_gpio_remove(struct platform_device *pdev); 

struct rht03_gpio_platform_data {
	struct gpio_desc *gpiod;
	struct gpio_desc *pullup_gpiod;
	void (*enable_external_pullup)(int enable);
	unsigned int pullup_duration;
	int index;
	u32 values;
	u64 previous_timestamp;
	int irq;
};

static irqreturn_t gpio_irq_handler(int irq, void *dev_id)
{
	u64 timestamp = ktime_get_ns();
	struct rht03_gpio_platform_data *pdata = dev_id;
	u64 timediff;

	//printk("IRQ %d fired at %llu, index=%d", irq, timestamp, pdata->index);

	if (dev_id == NULL)
	{
		printk("IRQ dev_id==NULL\n");
		goto out;
	}

	if (pdata->index >= 0 && pdata->index < PATTERN_LENGTH-1)
	{
		timediff = timestamp - pdata->previous_timestamp;

		if (SHORT_PULSE_NS - TOLERANCE < timediff && timediff < SHORT_PULSE_NS + TOLERANCE)
		{
			printk("0, diff=%llu\n", timediff);
		}
		else if (LONG_PULSE_NS - TOLERANCE < timediff && timediff < LONG_PULSE_NS + TOLERANCE_HIGH)
		{
			pdata->values |= (1<<(PATTERN_LENGTH - 1 - pdata->index));
			printk("1, diff=%llu\n", timediff);
			printk("VALUE: 0x%x\n", pdata->values);
		}
		else
		{
			printk("? diff=%llu\n", timediff);
			printk("Invalid pulse length\n");
		}
	}
	else if (pdata->index == PATTERN_LENGTH-1)
	{
		printk("\nFINAL SVALUE: 0x%x\n", pdata->values);
	}

	pdata->index++;
	pdata->previous_timestamp = timestamp;
out:
	return IRQ_HANDLED;
}

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

static int rht03_gpio_probe(struct platform_device *pdev)
{
	struct rht03_gpio_platform_data *pdata;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;

	/* Enforce open drain mode by default */
	enum gpiod_flags gflags = GPIOD_OUT_HIGH_OPEN_DRAIN;
	int i, high, ret;

	printk("rht03_gpio_probe()\n");

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

	pdata->index = -3;
	pdata->previous_timestamp = 0;
	pdata->values = 0x0000;

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

	gpiod_direction_output(pdata->gpiod, 1);

	pdata->irq = gpiod_to_irq(pdata->gpiod);

	ret = devm_request_irq(&pdev->dev,
					pdata->irq, gpio_irq_handler,
					IRQF_TRIGGER_RISING, pdev->name,
					pdata);

	high = 0;
	for (i = 0; i < 2; ++i)
	{
		gpiod_set_value(pdata->gpiod, high);
		high = !high;
		udelay(1100);
	}
	gpiod_direction_input(pdata->gpiod);

	printk("IRQ: %d\n", pdata->irq);

	if (ret == 0)
	{
		printk("IRQ enabled\n");
	}
	else
	{
		printk("Could not request irq, ret: %d\n", ret);
		return -1;
	}

	platform_set_drvdata(pdev, pdata);

	return 0;
}
static int rht03_gpio_remove(struct platform_device *pdev)
{
	struct rht03_gpio_platform_data *pdata = dev_get_platdata(&pdev->dev);

	printk("rht03_gpio_remove()\n");

	if (pdata->enable_external_pullup)
		pdata->enable_external_pullup(0);

	if (pdata->pullup_gpiod)
		gpiod_set_value(pdata->pullup_gpiod, 0);

	return 0; } 
static int rht03_init(void)
{
	printk("rht03 module_init()\n");
	platform_driver_register(&rht03_gpio_driver);
	return 0;
}

static void rht03_exit(void)
{
	printk("rht03 module_exit()\n");
	platform_driver_unregister(&rht03_gpio_driver);
}
/*
// Anpassning w1/masters/w1-gpio.c
static void w1_gpio_write_bit(struct w1_gpio_platform_data *pdata, u8 bit)
{
	gpiod_set_value(pdata->gpiod, bit);
}

// Anpassning w1/masters/w1-gpio.c
static u8 w1_gpio_read_bit(struct w1_gpio_platform_data *pdata)
{
	return gpiod_get_value(pdata->gpiod) ? 1 : 0;
}
*/



module_init(rht03_init);
module_exit(rht03_exit);


MODULE_DESCRIPTION("RHT03 GPIO driver");
MODULE_AUTHOR("Wille");
MODULE_LICENSE("GPL");
