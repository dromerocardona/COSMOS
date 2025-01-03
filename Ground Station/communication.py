import threading
import serial
import csv

class Communication:

    def __init__(self, serial_port, baud_rate=9600, timeout = 4, csv_filename = 'Flight_3195.csv'):
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.timeout = timeout
        self.data_list = []
        self.reading = False
        self.csv_filename = csv_filename
        self.receivedPacketCount = 0

        with open(self.csv_filename, mode='a', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['TEAM_ID', 'MISSION_TIME', 'PACKET_COUNT', 'MODE', 'STATE',
                             'ALTITUDE', 'TEMPERATURE', 'PRESSURE', 'VOLTAGE', 'GYRO_R',
                             'GYRO_P', 'GYRO_Y', 'ACCEL_R', 'ACCEL_P', 'ACCEL_Y', 'MAG_R',
                             'MAG_P', 'MAG_Y', 'AUTO_GYRO_ROTATION RATE', 'GPS_TIME', 'GPS_ALTITUDE',
                             'GPS_LATITUDE', 'GPS_LONGITUDE', 'GPS_SATS', 'CMD_ECHO', 'TEAM_NAME'])

    def read(self, signal_emitter):
        self.reading = True
        with serial.Serial(self.serial_port, self.baud_rate, timeout=self.timeout) as ser:
            print(f"Serial port {self.serial_port} opened successfully.")
            with open(self.csv_filename, mode='a', newline='') as file:
                writer = csv.writer(file)
                while self.reading:
                    try:
                        line = ser.read_until(b'COSMOS').decode('utf-8').strip()
                        if line:
                            print(f"Received: {line}")
                            self.receivedPacketCount += 1
                            self.parse_csv_data(line)
                            signal_emitter.emit_signal()
                            writer.writerow(line.split(','))
                    except Exception as e:
                        print(f"Error: {e}")

    def send_command(self, command):
        try:
            with serial.Serial(self.serial_port, self.baud_rate, timeout=self.timeout) as ser:
                command_to_send = f"{command}\n"
                ser.write(command_to_send.encode('utf-8'))
                print(f"Command sent: {command}")
        except serial.SerialException as e:
            print(f"Failed to send command: {e}")

    def stop_reading(self):
        self.reading = False
        print("Reading stopped.")

    def parse_csv_data(self, data):
        csv_data = data.split(',')
        self.data_list.append(csv_data)

    def get_data(self):
        return self.data_list

    def get_TEAM_ID(self):
        if self.data_list:
            try:
                return self.data_list[-1][0]
            except (IndexError, ValueError):
                return None
        return None

    def get_MISSION_TIME(self):
        if self.data_list:
            try:
                return self.data_list[-1][1]
            except (IndexError, ValueError):
                return None
        return None

    def get_PACKET_COUNT(self):
        if self.data_list:
            try:
                return self.data_list[-1][2]
            except (IndexError, ValueError):
                return None
        return None
    
    def get_MODE(self):
        if self.data_list:
            try:
                return self.data_list[-1][3]
            except (IndexError, ValueError):
                return None
        return None

    def get_STATE(self):
        if self.data_list:
            try:
                return self.data_list[-1][4]
            except (IndexError, ValueError):
                return None
        return None

    def get_ALTITUDE(self):
        if self.data_list:
            try:
                return self.data_list[-1][5]
            except (IndexError, ValueError):
                return None
        return None

    def get_TEMPERATURE(self):
        if self.data_list:
            try:
                return self.data_list[-1][6]
            except (IndexError, ValueError):
                return None
        return None

    def get_PRESSURE(self):
        if self.data_list:
            try:
                return self.data_list[-1][7]
            except (IndexError, ValueError):
                return None
        return None

    def get_VOLTAGE(self):
        if self.data_list:
            try:
                return self.data_list[-1][8]
            except (IndexError, ValueError):
                return None
        return None

    def get_GYRO_R(self):
        if self.data_list:
            try:
                return self.data_list[-1][9]
            except (IndexError, ValueError):
                return None
        return None

    def get_GYRO_P(self):
        if self.data_list:
            try:
                return self.data_list[-1][10]
            except (IndexError, ValueError):
                return None
        return None

    def get_GYRO_Y(self):
        if self.data_list:
            try:
                return self.data_list[-1][11]
            except (IndexError, ValueError):
                return None
        return None

    def get_ACCEL_R(self):
        if self.data_list:
            try:
                return self.data_list[-1][12]
            except (IndexError, ValueError):
                return None
        return None

    def get_ACCEL_P(self):
        if self.data_list:
            try:
                return self.data_list[-1][13]
            except (IndexError, ValueError):
                return None
        return None

    def get_ACCEL_Y(self):
        if self.data_list:
            try:
                return self.data_list[-1][14]
            except (IndexError, ValueError):
                return None
        return None

    def get_MAG_R(self):
        if self.data_list:
            try:
                return self.data_list[-1][15]
            except (IndexError, ValueError):
                return None
        return None

    def get_MAG_P(self):
        if self.data_list:
            try:
                return self.data_list[-1][16]
            except (IndexError, ValueError):
                return None
        return None

    def get_MAG_Y(self):
        if self.data_list:
            try:
                return self.data_list[-1][17]
            except (IndexError, ValueError):
                return None
        return None

    def get_AUTO_GYRO_ROTATION_RATE(self):
        if self.data_list:
            try:
                return self.data_list[-1][18]
            except (IndexError, ValueError):
                return None
        return None

    def get_GPS_TIME(self):
        if self.data_list:
            try:
                return self.data_list[-1][19]
            except (IndexError, ValueError):
                return None
        return None

    def get_GPS_ALTITUDE(self):
        if self.data_list:
            try:
                return self.data_list[-1][20]
            except (IndexError, ValueError):
                return None

    def get_GPS_LATITUDE(self):
        if self.data_list:
            try:
                return self.data_list[-1][21]
            except (IndexError, ValueError):
                return None

    def get_GPS_LONGITUDE(self):
        if self.data_list:
            try:
                return self.data_list[-1][22]
            except (IndexError, ValueError):
                return None

    def get_GPS_SATS(self):
        if self.data_list:
            try:
                return self.data_list[-1][23]
            except (IndexError, ValueError):
                return None

    def get_CMD_ECHO(self):
        if self.data_list:
            try:
                return self.data_list[-1][24]
            except (IndexError, ValueError):
                return None
