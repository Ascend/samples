# gpio python sample 
Sample program to demonstrate driving a GPIO port on the Atlas 200 DK board using the `python-periphery` python library. The python program will turn on and off the GPIO0 pin every 2 seconds.

## Prerequisites
- Atlas 200 DK board, PCB type IT21DMDA

## Setup
- No setup circuit required, can test with LED
- This sample uses GPIO0 (pin 7), which has the file descriptor of GPIO504
- On the Atlas board, as the `root` user, edit the /etc/rc.local file and insert the following commands before exit0. Then restart the board. This will set the appropriate permissions need to run the samples.
```
echo 504 >/sys/class/gpio/export
echo 444 >/sys/class/gpio/export
chown -R HwHiAiUser /sys/class/gpio/gpio444
chown -R HwHiAiUser /sys/class/gpio/gpio504
chown -R HwHiAiUser /sys/class/gpio/gpio444/direction
chown -R HwHiAiUser /sys/class/gpio/gpio504/direction
chown -R HwHiAiUser /sys/class/gpio/gpio444/value
chown -R HwHiAiUser /sys/class/gpio/gpio504/value
chown -R HwHiAiUser /dev/i2c-1
chown -R HwHiAiUser /dev/i2c-2
chown -R HwHiAiUser /dev/ttyAMA0
chown -R HwHiAiUser /dev/ttyAMA1
usermod -aG HwHiAiUser HwHiAiUser
```

## Run
1. On the Atlas board, run `python3 gpio.py`. If the UART connection works, you will see the following output on the Atlas board terminal:

```
gpio connection test
Set gpio504 to True
Set gpio504 to False
Set gpio504 to True
Set gpio504 to False
...
```

2. To close the program, ctrl+c. 



