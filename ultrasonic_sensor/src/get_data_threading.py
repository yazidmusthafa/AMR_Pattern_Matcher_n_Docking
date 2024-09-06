import serial
import threading
from datetime import datetime

# Define serial port configurations for the sensor
SERIAL_PORT0 = '/dev/ttyCH9344USB0'  # Adjust to your system's serial port
SERIAL_PORT1 = '/dev/ttyCH9344USB1'  # Adjust to your system's serial port
SERIAL_PORT2 = '/dev/ttyCH9344USB2'  # Adjust to your system's serial port
SERIAL_PORT3 = '/dev/ttyCH9344USB3'  # Adjust to your system's serial port
SERIAL_PORT4 = '/dev/ttyCH9344USB4'  # Adjust to your system's serial port
SERIAL_PORT5 = '/dev/ttyCH9344USB5'  # Adjust to your system's serial port
SERIAL_PORT6 = '/dev/ttyCH9344USB6'  # Adjust to your system's serial port
SERIAL_PORT7 = '/dev/ttyCH9344USB7'  # Adjust to your system's serial port

BAUDRATE = 115200
TIMEOUT = 1  # Timeout for serial reading

# Command to request data from the sensor (adjust if necessary)
REQUEST_COMMAND = bytes([0xFF])  

# Initialize serial ports
ser_0 = serial.Serial(SERIAL_PORT0, BAUDRATE, timeout=TIMEOUT)
ser_1 = serial.Serial(SERIAL_PORT1, BAUDRATE, timeout=TIMEOUT)
ser_2 = serial.Serial(SERIAL_PORT2, BAUDRATE, timeout=TIMEOUT)
ser_3 = serial.Serial(SERIAL_PORT3, BAUDRATE, timeout=TIMEOUT)
ser_4 = serial.Serial(SERIAL_PORT4, BAUDRATE, timeout=TIMEOUT)
ser_5 = serial.Serial(SERIAL_PORT5, BAUDRATE, timeout=TIMEOUT)
ser_6 = serial.Serial(SERIAL_PORT6, BAUDRATE, timeout=TIMEOUT)
ser_7 = serial.Serial(SERIAL_PORT7, BAUDRATE, timeout=TIMEOUT)

sensor_data = {f'sensor{i}': None for i in range(8)}
data_ready = threading.Condition()

def read_range_sensor(ser, sensor_id):
    while True:
        ser.reset_input_buffer()  # Clear any residual data from the buffer
        ser.write(REQUEST_COMMAND)
        buffer = bytearray()  # To accumulate incoming bytes

        while len(buffer) < 4:
            if ser.in_waiting > 0:
                buffer.extend(ser.read(ser.in_waiting))  # Read all available bytes and add them to buffer

        if len(buffer) >= 4:
            response = buffer[:4]  # Get the first 4 bytes

            calculated_checksum = (response[0] + response[1] + response[2]) & 0xFF
            if calculated_checksum == response[3]:
                distance = (response[1] << 8) + response[2]
                with data_ready:
                    sensor_data[f'sensor{sensor_id}'] = distance
                    # print(sensor_data)
                    if all(value is not None for value in sensor_data.values()):
                        data_ready.notify_all()

def print_sensor_data():
    while True:
        with data_ready:
            data_ready.wait_for(lambda: all(value is not None for value in sensor_data.values()))
            timestamp = datetime.now().strftime('%Y-%m-%d %H:%M:%S')
            print(f"{timestamp} - {sensor_data}")
            for key in sensor_data.keys():
                sensor_data[key] = None  # Reset sensor data for the next cycle

if __name__ == "__main__":
    threads = []
    serial_ports = [ser_0, ser_1, ser_2, ser_3, ser_4, ser_5, ser_6, ser_7]

    for i, ser in enumerate(serial_ports):
        thread = threading.Thread(target=read_range_sensor, args=(ser, i), name=f"SensorThread-{i}")
        threads.append(thread)
        thread.start()

    # Start a thread to print sensor data periodically
    print_thread = threading.Thread(target=print_sensor_data, name="PrintThread")
    print_thread.start()

    for thread in threads:
        thread.join()