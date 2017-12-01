GPIO驱动架构

初始化流程:

MACHINE_START(MX51_TRICYCLE, "tricycle board tokenwireless")
	...
	.init_irq = mx5_init_irq,
	...
MACHINE_END
//====>
void __init mx5_init_irq(void) {
	unsigned long tzic_addr;

	tzic_addr = MX51_TZIC_BASE_ADDR;
	mxc_tzic_init_irq(tzic_addr);
}
//====>
void __init mxc_tzic_init_irq(unsigned long base)
{
	mxc_register_gpios();
}
//====>
//根据iMX51 spec chapter 2 Memory Map p63，知道iMX51 共有4组gpio
//mxc_gpio_ports这个结构体数组中就包含了这4组gpio
int __init mxc_register_gpios(void)
{
	return mxc_gpio_init(mxc_gpio_ports, 4);//传入这个4，就表示4组gpio
}

struct mxc_gpio_port mxc_gpio_ports[] = {
	{ 	.chip.label = "gpio-0",
		.base = IO_ADDRESS(GPIO1_BASE_ADDR),//0x73F84000
		.irq = MXC_INT_GPIO1_LOW,
		.irq_high = MXC_INT_GPIO1_HIGH,
		.virtual_irq_start = MXC_GPIO_IRQ_START //128
	},
	{	.chip.label = "gpio-1",
		.base = IO_ADDRESS(GPIO2_BASE_ADDR),//0x73F88000
		...
	},
	{	.chip.label = "gpio-2",
		.base = IO_ADDRESS(GPIO3_BASE_ADDR),//0x73F8C000
		...
	},
	{	.chip.label = "gpio-3",
		.base = IO_ADDRESS(GPIO4_BASE_ADDR),//0x73F90000
		...
	},
};

//====>
int __init mxc_gpio_init(struct mxc_gpio_port *port, int cnt)
{
	int i, j;
	int ret = 0;

	for (i = 0; i < cnt; i++) {
		__raw_writel(0, port[i].base + GPIO_IMR); //iMX51 spec p1036: disable interrupt
		__raw_writel(~0, port[i].base + GPIO_ISR);//p1040,clear all interrupt status bits
		//分别设置每组gpio的virtual irq,
		//gpio1 virtual irq start :128
		//gpio2                    160
		//gpio3                    192
		//gpio4                    224
		//
		//Q: 为何每组gpio相隔32?
		//A: iMx51 spec p1031 p1042
		//   即每组gpio最多支持32个中断，下面循环中就是对每个中断作相应处理，
		//   如设置中断处理函数，设置flags,设置中断的desc
		for (j = port[i].virtual_irq_start;
			j < port[i].virtual_irq_start + 32; j++) {
			//下面函数是为 gpio_irq_chip挂接相关函数指针。
			set_irq_chip(j, &gpio_irq_chip);
			set_irq_handler(j, handle_level_irq);
			set_irq_flags(j, IRQF_VALID);
		}

		//为chip挂接相应函数指针，gpio_set_value(),gpio_direction_input()等函数
		//
		port[i].chip.direction_input = mxc_gpio_direction_input;
		port[i].chip.direction_output = mxc_gpio_direction_output;
		port[i].chip.get = mxc_gpio_get;
		port[i].chip.set = mxc_gpio_set;

		port[i].chip.base = i * 32;
		port[i].chip.ngpio = 32;
		//关键：
		//Q：为何在定义gpio的时候， #define TRICYCLE_GPIO2_0	(1*32 + 0)	/* GPIO_2_0 */
		//   只需写成1*32 + 0这样设置gpio时就可以找到相应的gpio？
		//A：此处就是关键所在了。
		//   上面chip.base = i*32表示一组gpio,imx51一共有四组
		//   chip.ngpio=32; 表示每组gpio又有32个
		//   下面这个gpiochip_add()函数就是将每个gpio依次添加gpio_desc[]数组中，
		//   设置时再根据这个宏的值，从数组中取出相应的gpio.即可对其进行相关设置
		gpiochip_add(&port[i].chip);

		if (!cpu_is_mx2() || cpu_is_mx25()) {
			/* setup one handler for each entry */
			set_irq_chained_handler(port[i].irq, mx3_gpio_irq_handler);
			set_irq_data(port[i].irq, &port[i]);
			if (port[i].irq_high) {
				set_irq_chained_handler(port[i].irq_high, mx3_gpio_irq_handler);
				set_irq_data(port[i].irq_high, &port[i]);
			}
		}
	}

	return ret;
}

int gpiochip_add(struct gpio_chip *chip)
{	
	//获取gpio的基地址,基地址的初始化是在:
	int		base = chip->base;
	int		status = 0;
	unsigned	id;

	//chip->ngpio在mxc_gpio_init()中赋值的
	//port[i].chip.ngpio = 32;
	//那么这个循环其实就是判断结构体数组gpio_desc[]是否为空，主要是用来防止重复操作
	for (id = base; id < base + chip->ngpio; id++) {
		if (gpio_desc[id].chip != NULL) {
			status = -EBUSY;
			break;
		}
	}

	//这里就是依次将每个gpio添加到gpio_desc[]数组中
	//当调用gpio_request(),gpio_direction_output(),gpio_set_value()等函数
	//时都是通过从gpio_desc[]数组中来先获取gpio_chip结构体，并调用mxc_gpio_init()函数中
	//为chip挂接的相关函数指针，从而实现设置gpio的目的。
	if (status == 0) {
		for (id = base; id < base + chip->ngpio; id++) {
			gpio_desc[id].chip = chip;
			gpio_desc[id].flags = !chip->direction_input
				? (1 << FLAG_IS_OUT) : 0;
		}
	}

	if (status == 0)
		status = gpiochip_export(chip);
}

static struct class_attribute gpio_class_attrs[] = {
	__ATTR(export, 0200, NULL, export_store),
	__ATTR(unexport, 0200, NULL, unexport_store),
	__ATTR_NULL,
};

static struct class gpio_class = {
	.name =		"gpio",
	.class_attrs =	gpio_class_attrs,
};

static int gpiochip_export(struct gpio_chip *chip)
{
	//这里会在/sys/devices/virtual/下创建名为gpiochip0/32/64/96的设备
	dev = device_create(&gpio_class, chip->dev, MKDEV(0, 0), chip,
				"gpiochip%d", chip->base);
	if (!IS_ERR(dev)) {
		//在指定目录下生成相应的sys文件:
		//e.g:/sys/class/gpio目录下会有gpiochip0,gpiochip32,gpiochip64,gpiochip96
		status = sysfs_create_group(&dev->kobj,
				&gpiochip_attr_group);
	}
}


相应函数的功能及实现：

//用来检查某个gpio是否已经有使用了，若已经使用了，则返回BUSY,
//没有则将&desc->flags的FLAG_REQUESTED位设为1,并返回0
//此外申请使用完gpio之后，要调用gpio_free()来释放
=========================================
gpio_request(TRICYCLE_3G_PW, "3g_power");
=========================================
int gpio_request(unsigned gpio, const char *label)
{
	struct gpio_desc	*desc;
	struct gpio_chip	*chip;

	//判断传入的gpio的值是否在范围内：最大不能超过256
	if (!gpio_is_valid(gpio))
		goto done;
	desc = &gpio_desc[gpio];

	//Q: 此处的desc->chip在哪里赋值的?
	//A: mxc_gpio_init()  ==>  gpiochip_add()
	chip = desc->chip;

	//Q: 下面这个函数是什么用?
	//A: 这个函数是用来将&desc->flags的第FLAG——REQUESTED位设为1，并返回该位原来的值。
	//注意:FLAG_REQUESTED 是表示需要设置的是哪一位，
	//     而不是将&desc->flags的值设置成FLAG_REQUESTED
	if (test_and_set_bit(FLAG_REQUESTED, &desc->flags) == 0) {
		//该函数内部的宏开关没开，所以调用gpio_request()时，其第二个参数可以随意
		//即使宏开了也无所谓，其实它只是将gpio_request()传入的第二个参数赋值给
		//desc->label
		desc_set_label(desc, label ? : "?");
		status = 0;
	} else {
		status = -EBUSY;
		module_put(chip->owner);
		goto done;
	}

	//mxc_gpio_init()中并没有挂接此函数指针，所以这里不会进。
	if (chip->request) {
		status = chip->request(chip, gpio - chip->base);
	}
}

==========================
gpio_free(TRICYCLE_3G_PW);
==========================
void gpio_free(unsigned gpio)
{
	struct gpio_desc	*desc;
	struct gpio_chip	*chip;

	if (!gpio_is_valid(gpio))
		return;

	desc = &gpio_desc[gpio];
	chip = desc->chip;

	//若desc->flags的第FLAG_REQUESTED位为1，则
	if (chip && test_bit(FLAG_REQUESTED, &desc->flags)) {
		desc_set_label(desc, NULL);
		clear_bit(FLAG_REQUESTED, &desc->flags);
	}
}
	
=========================================
gpio_direction_output(TRICYCLE_3G_PW, 1);
=========================================
int gpio_direction_input(unsigned gpio)
{
	struct gpio_chip	*chip;
	struct gpio_desc	*desc = &gpio_desc[gpio];

	chip = desc->chip;
	//Q：这里gpio为何又要减一个chip->base
	//A: 传入的gpio值是表示哪一组gpio的哪一位，
	//   这里chip->base的初始化是在mxc_gpio_init(),其值表示哪一组，通常值为0，32，64，96
	//   实际上这儿是得到了一个gpio在某组gpio中的位置，如TRICYCLE_3G_PW是(1*32 + 4)，那
	//   么这里就得到4.
	gpio -= chip->base;

	//检查在此之前是否已经request了gpio.
	status = gpio_ensure_requested(desc, gpio);
	//这里就会调用在mxc_gpio_init()中初始化chip时挂接的函数指针
	// port[i].chip.direction_input = mxc_gpio_direction_input;
	status = chip->direction_input(chip, gpio);
}

static int mxc_gpio_direction_input(struct gpio_chip *chip, unsigned offset)
{
	_set_gpio_direction(chip, offset, 0);
	return 0;
}
//这里就是设置具体的寄存器了
//iMX51 spec p1036 set GPIO Direction Register
//                 0 : gpio is configured as input
//                 1 : gpio is configured as output
static void _set_gpio_direction(struct gpio_chip *chip, unsigned offset,
				int dir)
{
	struct mxc_gpio_port *port =
		container_of(chip, struct mxc_gpio_port, chip);
	u32 l;
	//现在可以知道前面在gpio_direction_input()中通过 gpio -= chip->base;
	//来某个gpio在一组gpio中的位置，真正的用途是用在这里设置寄存器的时候，
	//作为偏移。前面知道一组GPIO有32个，每个gpio就对应了GPIO Direction Register
	//中的一位，所以要设置某gpio是input/output，要先计算出它在GDIR 寄存器的中
	//偏移，再写入相应的值。
	l = __raw_readl(port->base + GPIO_GDIR);
	if (dir)
		l |= 1 << offset;
	else
		l &= ~(1 << offset);
	__raw_writel(l, port->base + GPIO_GDIR);
}
===========================================
gpio_direction_output(TRICYCLE_GPIO3_0, 1);
===========================================
从传入的参数来看，设gpio为output比设为input多了一个参数（1／0）.
同样gpio_direction_output()也会调用mxc_gpio_init()函数中为chip挂接的函数指针
port[i].chip.direction_output = mxc_gpio_direction_output;
====>
static int mxc_gpio_direction_output(struct gpio_chip *chip,
				     unsigned offset, int value)
{
	//与mxc_gpio_direction_input()稍有不同的是：设置为output会先set gpio value.
	mxc_gpio_set(chip, offset, value);
	_set_gpio_direction(chip, offset, 1);
	return 0;
}

//这里也就是设置iMX51 spec p1035 set GPIO DATA Register
static void mxc_gpio_set(struct gpio_chip *chip, unsigned offset, int value)
{
	struct mxc_gpio_port *port =
		container_of(chip, struct mxc_gpio_port, chip);
	void __iomem *reg = port->base + GPIO_DR;
	u32 l;

	l = (__raw_readl(reg) & (~(1 << offset))) | (value << offset);
	__raw_writel(l, reg);
}

==================================
gpio_set_value(TRICYCLE_3G_PW, 1);
==================================
同样gpio_set_value()也会调用mxc_gpio_init()函数中为chip挂接的函数指针
====>
     port[i].chip.set = mxc_gpio_set;

gpio_get_value()的情况同set value，故在此不再说明。
