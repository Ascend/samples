# i2c python sample
Sample program to demonstrate I2C connection between Arduino Uno and Atlas 200 DK board using the `python-periphery` python library.

## Prerequisites
- Atlas 200 DK board, PCB type IT21DMDA
- Arduino Uno R3

## Setup
- The Atlas board is the master and connects to the Arduino slave
- Connect Atlas board I2C2-SDA (pin 3) to Arduino SDA. 
- Connect Atlas board I2C2-SCL (pin 5) to Arduino SCL. 
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
2. On the Atlas board, run `python3 i2c.py`. If the connection works, you will see "Hello from Atlas" on the Arduino serial monitor, and the following output on the Atlas board terminal:

```
i2c connection test
Write to I2C
Read from I2C: Hello from Arduino
```

