import serial
import time

# Define serial port configurations for the sensor
SERIAL_PORT = '/dev/ttyUSB0'  # Adjust to your system's serial port
BAUDRATE = 115200
TIMEOUT = 1  # Timeout for serial reading

# Command to request data from the sensor (adjust if necessary)
REQUEST_COMMAND = bytes([0xFF])  

ser_1 = serial.Serial(SERIAL_PORT, BAUDRATE, timeout=TIMEOUT)

def read_range():
    ser_1.reset_input_buffer()  # Clear any residual data from the buffer
    ser_1.write(REQUEST_COMMAND)
    buffer = bytearray()  # To accumulate incoming bytes

    while len(buffer) < 4:
        if ser_1.in_waiting > 0:
            buffer.extend(ser_1.read(ser_1.in_waiting))  # Read all available bytes and add them to buffer

    if len(buffer) >= 4:
        response = buffer[:4]  # Get the first 4 bytes
       
        calculated_checksum = (response[0] + response[1] + response[2]) & 0xFF
        if calculated_checksum == response[3]:
            distance = (response[1] << 8) + response[2]
            print(int(time.time() * 1000), "       " ,f"Distance: {distance} mm")
            
while True:
    read_range()