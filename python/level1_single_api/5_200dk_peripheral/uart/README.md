# uart python 
Sample program to demonstrate UART connection between Arduino Uno and Atlas 200 DK board using the `python-periphery` python library.

## Prerequisites
- Atlas 200 DK board, PCB type IT21DMDA
- Arduino Uno R3

## Setup
- Connect Atlas board Tx (pin 16) to Arduino Rx. This sets up the connection to send from Atlas to Arduino.
- Connect Atlas board Rx (pin 18) to Arduino Tx. This requires a voltage divider circuit to step down 5V Arduino Tx to 3.5V Atlas board Rx. 
- Connect Atlas GND to Arduino GND. 
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
1. Upload sketch.ino to Arduino Uno board. Open the serial monitor. 
2. On the Atlas board, run `python3 uart.py`. If the UART connection works, you will see the following output on the Atlas board terminal:

```
uart connection test
Write to UART
Reply: Arduino: Hello from Arduino
```



