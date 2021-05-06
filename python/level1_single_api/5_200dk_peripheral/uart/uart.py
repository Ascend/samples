from periphery import Serial

def main():
    print("uart connection test")

    # Open /dev/ttyAMA1 with baudrate 115200
    ser = Serial("/dev/ttyAMA1", 115200)

    print("Write to UART")
    ser.write(b"Hello from Atlas 200 DK\n")
        
    # Read up to 32 bytes, with timeout of 2 seconds
    readdata = ser.read(32, 2).decode('utf-8')
    print(f'Received reply: {readdata}')


if __name__ == "__main__":
    main()
