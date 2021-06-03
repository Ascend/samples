from periphery import SysfsGPIO
import time

def main():
    print("gpio connection test")

    # Open GPIO0, pin 7 (GPIO504)
    gpio1 = SysfsGPIO(504, "out")
    #gpio2 = SysfsGPIO(444, "out")

    # Turn off and on GPIO every two seconds
    while True:
        value = gpio1.read()
        print(f'Set gpio504 to {not value}')
        gpio1.write(not value)
        time.sleep(2)
        
if __name__ == "__main__":
    main()
