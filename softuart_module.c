#include <linux/module.h>
#include <linux/delay.h>
#include <linux/tty.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/tty_driver.h>
#include <linux/gpio.h>
#include <linux/version.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>

#include "softuart.h"

#define SOFT_GPIO_DRIVER_NAME "Uart-soft"

#define SOFT_UART_MAJOR            0
#define N_PORTS                    1
#define NONE                       0
#define TX_BUFFER_FLUSH_TIMEOUT 4000  // milliseconds

// Module prototypes.
static int  soft_uart_open(struct tty_struct*, struct file*);
static void soft_uart_close(struct tty_struct*, struct file*);
static int  soft_uart_write(struct tty_struct*, const unsigned char*, int);
static int  soft_uart_write_room(struct tty_struct*);
static void soft_uart_flush_buffer(struct tty_struct*);
static int  soft_uart_chars_in_buffer(struct tty_struct*);
static void soft_uart_set_termios(struct tty_struct*, struct ktermios*);
static void soft_uart_stop(struct tty_struct*);
static void soft_uart_start(struct tty_struct*);
static void soft_uart_hangup(struct tty_struct*);
static int  soft_uart_tiocmget(struct tty_struct*);
static int  soft_uart_tiocmset(struct tty_struct*, unsigned int, unsigned int);
static int  soft_uart_ioctl(struct tty_struct*, unsigned int, unsigned int long);
static void soft_uart_throttle(struct tty_struct*);
static void soft_uart_unthrottle(struct tty_struct*);

// Module operations.
static const struct tty_operations soft_uart_operations = {
    .open            = soft_uart_open,
    .close           = soft_uart_close,
    .write           = soft_uart_write,
    .write_room      = soft_uart_write_room,
    .flush_buffer    = soft_uart_flush_buffer,
    .chars_in_buffer = soft_uart_chars_in_buffer,
    .ioctl           = soft_uart_ioctl,
    .set_termios     = soft_uart_set_termios,
    .stop            = soft_uart_stop,
    .start           = soft_uart_start,
    .hangup          = soft_uart_hangup,
    .tiocmget        = soft_uart_tiocmget,
    .tiocmset        = soft_uart_tiocmset,
    .throttle        = soft_uart_throttle,
    .unthrottle      = soft_uart_unthrottle
};

/**
 * Opens a given TTY device.
 * @param tty given TTY device
 * @param file
 * @return error code.
 */
static int soft_uart_open(struct tty_struct* tty, struct file* file)
{
    int error = NONE;

    if (gpio_soft_uart_open(tty))
    {
        printk(KERN_INFO "soft_uart: Device opened.\n");
    }
    else
    {
        printk(KERN_ALERT "soft_uart: Device busy.\n");
        error = -ENODEV;
    }

    return error;
}

/**
 * Closes a given TTY device.
 * @param tty
 * @param file
 */
static void soft_uart_close(struct tty_struct* tty, struct file* file)
{
    // Waits for the TX buffer to be empty before closing the UART.
    int wait_time = 0;
    while ((gpio_soft_uart_get_tx_queue_size() > 0)
           && (wait_time < TX_BUFFER_FLUSH_TIMEOUT))
    {
        msleep(100);
        wait_time += 100;
    }

    if (gpio_soft_uart_close())
    {
        printk(KERN_INFO "soft_uart: Device closed.\n");
    }
    else
    {
        printk(KERN_ALERT "soft_uart: Could not close the device.\n");
    }
}

/**
 * Writes the contents of a given buffer into a given TTY device.
 * @param tty given TTY device
 * @param buffer given buffer
 * @param buffer_size number of bytes contained in the given buffer
 * @return number of bytes successfuly written into the TTY device
 */
static int soft_uart_write(struct tty_struct* tty, const unsigned char* buffer, int buffer_size)
{
    return gpio_soft_uart_send_string(buffer, buffer_size);
}

/**
 * Tells the kernel the number of bytes that can be written to a given TTY.
 * @param tty given TTY
 * @return number of bytes
 */
static int soft_uart_write_room(struct tty_struct* tty)
{
    return gpio_soft_uart_get_tx_queue_room();
}

/**
 * Does nothing.
 * @param tty
 */
static void soft_uart_flush_buffer(struct tty_struct* tty)
{
}

/**
 * Tells the kernel the number of bytes contained in the buffer of a given TTY.
 * @param tty given TTY
 * @return number of bytes
 */
static int soft_uart_chars_in_buffer(struct tty_struct* tty)
{
    return gpio_soft_uart_get_tx_queue_size();
}

/**
 * Sets the UART parameters for a given TTY (only the baudrate is taken into account).
 * @param tty given TTY
 * @param termios parameters
 */
static void soft_uart_set_termios(struct tty_struct* tty, struct ktermios* termios)
{
    int cflag = 0;
    //ty->driver_data;
    speed_t baudrate = tty_get_baud_rate(tty);
    printk(KERN_INFO "soft_uart: soft_uart_set_termios: baudrate = %d.\n", baudrate);

    // Gets the cflag.
    cflag = tty->termios.c_cflag;
//#if LINUX_VERSION_CODE < KERNEL_VERSION(3,10,0)
//    cflag = tty->termios->c_cflag;
//#endif

    // Verifies the number of data bits (it must be 8).
    if ((cflag & CSIZE) != CS8)
    {
        printk(KERN_ALERT "soft_uart: Invalid number of data bits.\n");
    }

    // Verifies the number of stop bits (it must be 1).
    if (cflag & CSTOPB)
    {
        printk(KERN_ALERT "soft_uart: Invalid number of stop bits.\n");
    }

    // Verifies the parity (it must be none).
    if (cflag & PARENB)
    {
        printk(KERN_ALERT "soft_uart: Invalid parity.\n");
    }

    // Configure the baudrate.
    if (!gpio_soft_uart_set_baudrate(baudrate))
    {
        printk(KERN_ALERT "soft_uart: Invalid baudrate.\n");
    }
}

/**
 * Does nothing.
 * @param tty
 */
static void soft_uart_stop(struct tty_struct* tty)
{
    printk(KERN_DEBUG "soft_uart: soft_uart_stop.\n");
}

/**
 * Does nothing.
 * @param tty
 */
static void soft_uart_start(struct tty_struct* tty)
{
    printk(KERN_DEBUG "soft_uart: soft_uart_start.\n");
}

/**
 * Does nothing.
 * @param tty
 */
static void soft_uart_hangup(struct tty_struct* tty)
{
    printk(KERN_DEBUG "soft_uart: soft_uart_hangup.\n");
}

/**
 * Does nothing.
 * @param tty
 */
static int soft_uart_tiocmget(struct tty_struct* tty)
{
    return 0;
}

/**
 * Does nothing.
 * @param tty
 * @param set
 * @param clear
 */
static int soft_uart_tiocmset(struct tty_struct* tty, unsigned int set, unsigned int clear)
{
    return 0;
}

/**
 * Does nothing.
 * @param tty
 * @param command
 * @param parameter
 */
static int soft_uart_ioctl(struct tty_struct* tty, unsigned int command, unsigned int long parameter)
{
    int error = NONE;

    switch (command)
    {
    case TIOCMSET:
        error = NONE;
        break;

    case TIOCMGET:
        error = NONE;
        break;

    default:
        error = -ENOIOCTLCMD;
        break;
    }

    return error;
}

/**
 * Does nothing.
 * @param tty
 */
static void soft_uart_throttle(struct tty_struct* tty)
{
    printk(KERN_DEBUG "soft_uart: soft_uart_throttle.\n");
}

/**
 * Does nothing.
 * @param tty
 */
static void soft_uart_unthrottle(struct tty_struct* tty)
{
    printk(KERN_DEBUG "soft_uart: soft_uart_unthrottle.\n");
}

static int uart_gpio_probe(struct platform_device *pdev)
{
    struct uart_gpio_device_data *data;
    struct tty_driver* soft_uart_driver;
    struct tty_port *port;
    int ret;
    struct device_node *np = pdev->dev.of_node;
    unsigned long irq_flags;

    dev_err(&pdev->dev, "uart_gpio_probe schips\n");
    if(!np)
    {
        dev_err(&pdev->dev, "need as dtsi\n");
        return -ENOMEM;
    }

    /* allocate space for device info */
    data = devm_kzalloc(&pdev->dev, sizeof(struct uart_gpio_device_data),
                        GFP_KERNEL);
    if (!data)
        return -ENOMEM;
    /* Initializes the Soft UART infrastructure. */
    mutex_init(&data->current_tty_mutex);

    // Initializes the TX timer.
    hrtimer_init(&data->timer_tx, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    data->timer_tx.function = &handle_tx;

    // Initializes the RX timer.
    hrtimer_init(&data->timer_rx, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
    data->timer_rx.function = &handle_rx;

    // Initializes the GPIO pins.
    // 1. RX, input and irq.
    dev_info(&pdev->dev, "Doing for gpio_rx\n");
    data->gpio_rx = of_get_named_gpio_flags(np, "gpio-rx", 0, NULL);
    ret = devm_gpio_request(&pdev->dev, data->gpio_rx, "soft_uart_tx");
    if (ret) {
        dev_err(&pdev->dev, "failed to request GPIO %u for gpio_rx\n",
                data->gpio_rx);
        return ret;
    }

    ret = gpio_direction_input(data->gpio_rx);
    if (ret) {
        dev_err(&pdev->dev, "failed to set gpio_rx direction\n");
        return -EINVAL;
    }

    ret = gpio_to_irq(data->gpio_rx);
    if (ret < 0) {
        dev_err(&pdev->dev, "failed to map gpio_rx to IRQ: %d\n", ret);
        return -EINVAL;
    }
    data->irq_rx = ret;
    /* register IRQ interrupt handler */
    ret = devm_request_irq(&pdev->dev, data->irq_rx,
                           (irq_handler_t) handle_rx_start,
                            IRQF_TRIGGER_FALLING ,
                            "soft_uart_irq_handler", data);
    if (ret) {
        dev_err(&pdev->dev, "failed to acquire IRQ for gpio_rx %d\n", data->gpio_rx);
        return -EINVAL;
    }
    disable_irq(data->irq_rx);
    dev_err(&pdev->dev, "Schips Registered IRQ %d as soft uart source\n", data->irq_rx);

    dev_err(&pdev->dev, "Done for gpio_rx\n");

    // 2. TX for output
    dev_err(&pdev->dev, "Doing for gpio_tx\n");
    data->gpio_tx = of_get_named_gpio_flags(np, "gpio-tx", 0, (enum of_gpio_flags *)&irq_flags);
    ret = devm_gpio_request(&pdev->dev, data->gpio_tx, "soft_uart_tx");
    if (ret) {
        dev_err(&pdev->dev, "failed to request GPIO %u for gpio_tx\n",
                data->gpio_tx);
        return ret;
    }

    ret = gpio_direction_output(data->gpio_tx, 1);
    if (ret) {
        dev_err(&pdev->dev, "failed to set gpio_tx output direction\n");
        return -EINVAL;
    }
    dev_err(&pdev->dev, "Done for gpio_tx\n");

    // Set up uart
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0)
    dev_info(&pdev->dev, "soft_uart: LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0).\n");

    // Initializes the port.
    port = &data->port;
    tty_port_init(port);
    port->low_latency = 0;

    // Allocates the driver.
    soft_uart_driver = tty_alloc_driver(N_PORTS, TTY_DRIVER_REAL_RAW);

    // Returns if the allocation fails.
    if (IS_ERR(soft_uart_driver))
    {
        dev_err(&pdev->dev, "soft_uart: Failed to allocate the driver.\n");
        return -ENOMEM;
    }
#else
    printk(KERN_INFO "soft_uart: LINUX_VERSION_CODE < KERNEL_VERSION(3,10,0).\n");

    // Allocates the driver.
    soft_uart_driver = alloc_tty_driver(N_PORTS);

    // Returns if the allocation fails.
    if (!soft_uart_driver)
    {
        dev_err(&pdev->dev, "soft_uart: Failed to allocate the driver.\n");
        return -ENOMEM;
    }
#endif

    // Initializes the driver.
    soft_uart_driver->owner                 = THIS_MODULE;
    soft_uart_driver->driver_name           = "soft_uart";
    soft_uart_driver->name                  = "ttySOFT";
    soft_uart_driver->major                 = SOFT_UART_MAJOR;
    soft_uart_driver->minor_start           = 0;
    soft_uart_driver->flags                 = TTY_DRIVER_REAL_RAW;
    soft_uart_driver->type                  = TTY_DRIVER_TYPE_SERIAL;
    soft_uart_driver->subtype               = SERIAL_TYPE_NORMAL;
    soft_uart_driver->init_termios          = tty_std_termios;
    soft_uart_driver->init_termios.c_ispeed = 4800;
    soft_uart_driver->init_termios.c_ospeed = 4800;
    soft_uart_driver->init_termios.c_cflag  = B4800 | CREAD | CS8 | CLOCAL;

    // Sets the callbacks for the driver.
    tty_set_operations(soft_uart_driver, &soft_uart_operations);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0)
    // Link the port with the driver.
    tty_port_link_device(port, soft_uart_driver, 0);
#endif

    // Registers the TTY driver.
    if (tty_register_driver(soft_uart_driver))
    {
        printk(KERN_ALERT "soft_uart: Failed to register the driver.\n");
        put_tty_driver(soft_uart_driver);
        return -1; // return if registration fails
    }
    data->soft_uart_driver = soft_uart_driver;

    platform_set_drvdata(pdev, data);
    gpio_soft_uart_initialize(data);
    return 0;
}

static int uart_gpio_remove(struct platform_device *pdev)
{
    struct uart_gpio_device_data *data = platform_get_drvdata(pdev);

    dev_info(&pdev->dev, "Finalizing the module...\n");

    // Finalizes the soft UART.
    free_irq(gpio_to_irq(data->gpio_rx), NULL);
    gpio_set_value(data->gpio_tx, 0);
    gpio_free(data->gpio_tx);
    gpio_free(data->gpio_rx);

    // Unregisters the driver.
    if (tty_unregister_driver(data->soft_uart_driver))
    {
        printk(KERN_ALERT "soft_uart: Failed to unregister the driver.\n");
    }

    put_tty_driver(data->soft_uart_driver);
    printk(KERN_INFO "soft_uart: Module finalized.\n");
    return 0;
}

static const struct of_device_id uart_gpio_dt_ids[] = {
    { .compatible = "schips,soft_uart", },
    { /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, uart_gpio_dt_ids);

static struct platform_driver uart_gpio_driver = {
    .driver		= {
        .owner = THIS_MODULE,
        .name	= SOFT_GPIO_DRIVER_NAME,
        .of_match_table	= uart_gpio_dt_ids,
    },
    .probe		= uart_gpio_probe,
    .remove		= uart_gpio_remove,
};

//need LINUX_VERSION_CODE >= KERNEL_VERSION(3,10,0)
module_platform_driver(uart_gpio_driver);
MODULE_DESCRIPTION("A Linux driver using GPIO pin as tty device.");
MODULE_AUTHOR("Schips");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0");
