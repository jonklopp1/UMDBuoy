import smbus2

bus = smbus2.SMBus(1)

readData = bus.read_byte_data(77,1)
print (readData)
