#ifndef SOFT_UART_H
#define SOFT_UART_H

/* Info for each registered platform device */
struct uart_gpio_device_data {
    struct tty_port port;
    struct mutex current_tty_mutex;
    struct hrtimer timer_tx;
    struct hrtimer timer_rx;

    struct tty_driver* soft_uart_driver;
    int irq_rx;			/* IRQ used as FPGA-PPS source */
    unsigned int gpio_tx;
    unsigned int gpio_rx;
};

void gpio_soft_uart_initialize(struct uart_gpio_device_data * uart_gpio_device);
void gpio_soft_uart_finalize(void);
int gpio_soft_uart_open(struct tty_struct* tty);
int gpio_soft_uart_close(void);
int gpio_soft_uart_set_baudrate(const int baudrate);
int gpio_soft_uart_send_string(const unsigned char* string, int string_size);
int gpio_soft_uart_get_tx_queue_room(void);
int gpio_soft_uart_get_tx_queue_size(void);

enum hrtimer_restart handle_tx(struct hrtimer* timer);
enum hrtimer_restart handle_rx(struct hrtimer* timer);
irq_handler_t handle_rx_start(unsigned int irq, void* device, struct pt_regs* registers);

#endif
