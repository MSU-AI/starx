from smbus2 import SMBus
import time
import struct

# I2C address of the Arduino
ARDUINO_I2C_ADDRESS = 0x08

# Total number of bytes to read (24 floats * 4 bytes per float)
TOTAL_BYTES = 96

# Maximum bytes per I2C transaction (due to limitations)
CHUNK_SIZE = 32

def read_i2c_data(bus, addr, total_bytes, chunk_size):
    data = []
    chunks = (total_bytes + chunk_size - 1) // chunk_size
    for chunk_index in range(chunks):
        # Use read_i2c_block_data which sends a command byte before reading data
        to_read = min(chunk_size, total_bytes - (chunk_index * chunk_size))
        chunk_data = bus.read_i2c_block_data(addr, chunk_index, to_read)
        data.extend(chunk_data)
    return data

#dont forget to change bus (either 1 or 7)
with SMBus(7) as bus:  # Use the correct I2C bus number
    while True:
        try:
            # Read data in chunks
            data = read_i2c_data(bus, ARDUINO_I2C_ADDRESS, TOTAL_BYTES, CHUNK_SIZE)

            # Convert byte data to floats
            floats = []
            for i in range(0, TOTAL_BYTES, 4):
                # Combine 4 bytes into a float
                float_bytes = bytes(data[i:i+4])
                value = struct.unpack('<f', float_bytes)[0]  # '<f' for little-endian float
                floats.append(value)

            # Split into master and slave data
            masterData = floats[:12]
            slaveData = floats[12:]

            # Print the data
            print("master_data:", masterData)
            print("slave_data:", slaveData)
            # quit()
            # Wait before the next request
            # time.sleep(0.1)  # Adjust as needed

        except Exception as e:
            print("Error:", e)
            time.sleep(1)
