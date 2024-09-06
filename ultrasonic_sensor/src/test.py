import serial
import time

# Define serial port configurations for the sensor
SERIAL_PORT = '/dev/ttyUSB0'  # Adjust to your system's serial port
BAUDRATE = 115200
TIMEOUT = 1  # Timeout for serial reading

# Command to request data from the sensor (adjust if necessary)
REQUEST_COMMAND = bytes([0xFF])  

# Function to initialize serial communication
def initialize_serial(port):
    return serial.Serial(port, BAUDRATE, timeout=TIMEOUT)

# Function to read range from the sensor
def read_range(ser):
    ser.write(REQUEST_COMMAND)
    # time.sleep(0.002)  # Wait for the sensor to respond (adjust as necessary)

    response = None

    while response is None:
        ser.write(REQUEST_COMMAND)
        if ser.in_waiting > 0:
            response = ser.read(5)  # Read 5 bytes as per the example format
            if len(response) == 5:
                return parse_response(response)
        
    # print("No data available from the sensor")
    # return None

# Helper function to parse the response and calculate the distance
def parse_response(response):
    # Extract data from the response
    frame_header = response[0]
    data_h = response[1]
    data_l = response[2]
    checksum = response[3]

    # Verify the checksum
    calculated_checksum = (frame_header + data_h + data_l) & 0x00FF
    if calculated_checksum != checksum:
        print(f"Checksum error: Expected {checksum}, got {calculated_checksum}")
        # return None
        return read_range(initialize_serial(SERIAL_PORT))

    # Calculate the distance
    distance = (data_h << 8) + data_l

    # Print raw response data for debugging
    print(f"Raw response: {[hex(b) for b in response]}")
    print(f"Calculated checksum: 0x{calculated_checksum:02X}")

    # Assuming the modbus register 0x0209 is 0x00 (unit: mm)
    print(f"Distance: {distance} mm")

    # If the unit is in microseconds (modbus register 0x0209 is 0x01)
    # Uncomment the following lines to convert to mm
    # distance_mm = distance / 5.75
    # print(f"Converted Distance: {distance_mm:.2f} mm")

    return distance

def main():
    # Initialize serial communication
    serial_port = initialize_serial(SERIAL_PORT)

    while True:
        # Read distance from the sensor
        distance = read_range(serial_port)

        if distance is not None:
            print(f"Measured Distance: {distance} mm")
        else:
            print("Failed to read distance from the sensor")

        # time.sleep(1.2)  # Delay for readability

if __name__ == "__main__":
    main()