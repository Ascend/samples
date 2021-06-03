from periphery import I2C

def main():
    print("i2c connection test")

    # Open i2c-2 controller
    i2c = I2C("/dev/i2c-2")

    print("Write to I2C")
   
    readbuffer = bytearray(19)
    # Create a list of messages to send to slave
    # First message is example of writing string 
    # Second message reads bytes up to size of readbuffer
    msg = [ I2C.Message("Hello from Atlas\n".encode("utf-8")),
            I2C.Message(readbuffer, read=True) ]
    # Send messages to address 4
    i2c.transfer(4, msg)

    print(f'Read from I2C: {msg[1].data.decode("utf-8")}')

    i2c.close() 

if __name__ == "__main__":
    main()
