# GPIO模拟Uart 通信 (soft uart/serial)
## 介绍
基于gpio的软uart，可用于通用的Linux设备。

> 在Uart不够用的时候可以通过GPIO 来模拟，但是GPIO 模拟有一个缺点就是时钟可能不准，Uart是异步的，我们可以设置两个定时器来模拟其对应的输出。

需要通过dts进行驱动

ref :
1. https://github.com/adrianomarto/soft_uart
2. https://github.com/nishuaidai/soft_uart


此模块创建一个gpio可配置的软件uart，节点类似为：`/dev/ttySOFTx`(x = 0..n)

## 特性

* 兼容应用层, e.g. `cat`, `echo`, `minicom`.
* 波特率可配置
* TX buffer of 256 bytes.
* RX buffer managed by the kernel.

## 用法

### 加载

Module parameters:

* gpio_tx: int [default = 17]
* gpio_rx: int [default = 27]

Loading the module with default parameters:
```
insmod soft_uart.ko
```

Loading module with custom parameters:

```
insmod soft_uart.ko gpio_tx=10 gpio_rx=11
```

### 应用层使用

The device will appear as `/dev/ttySOFT0`. Use it as any usual TTY device.

Usage examples:
```bash
# config baud rate
minicom -b 4800 -D /dev/ttySOFT0
## or : stty -F /dev/ttySOFT0 speed 4800

# read
cat /dev/ttySOFT0
# write
echo "hello" > /dev/ttySOFT0
```

## 波特率

When choosing the baud rate, take into account that:
* You will probably not be running a real-time operating system.
* There will be other processes competing for CPU time.

As a result, you can expect communication errors when using fast baud rates.
* The Raspberry Pi : not try to go any faster than 4800 bps..
* The Rockchip 3568 : 9600 bps is work-well..

## 设计说明

linux下的GPIO模拟Uart涉及到如下几个内容

1、GPIO初始化、设定输入输出、以及输入中断设置

2、初始化定时器，建议是高精度定时器

3、中断处理数据接收处理和发送数据

4、fifo管理数据

5、注册tty 驱动管理
