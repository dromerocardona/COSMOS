import serial
import csv
import time
import threading

class Communication:

    def __init__(self, serial_port, baud_rate=9600, timeout = 4, csv_filename = 'Flight_3195.csv'):
        # Initialize communication parameters
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.timeout = timeout
        self.data_list = []
        self.reading = False
        self.csv_filename = csv_filename
        self.receivedPacketCount = 0
        self.lastPacket = ""
        self.command_queue = []
        self.command_lock = threading.Lock()
        self.read_thread = None
        self.send_thread = None
        self.simulation = False
        self.simEnabled = False

        #Create a CSV file to store telemetry data
        with open(self.csv_filename, mode='a', newline='') as file:
            writer = csv.writer(file)
            if not file.tell():
                writer.writerow(['TEAM_ID', 'MISSION_TIME', 'PACKET_COUNT', 'MODE', 'STATE',
                                 'ALTITUDE', 'TEMPERATURE', 'PRESSURE', 'VOLTAGE', 'GYRO_R',
                                 'GYRO_P', 'GYRO_Y', 'ACCEL_R', 'ACCEL_P', 'ACCEL_Y', 'MAG_R',
                                 'MAG_P', 'MAG_Y', 'AUTO_GYRO_ROTATION RATE', 'GPS_TIME', 'GPS_ALTITUDE',
                                 'GPS_LATITUDE', 'GPS_LONGITUDE', 'GPS_SATS', 'CMD_ECHO', 'TEAM_NAME'])

    def start_communication(self, signal_emitter):
        self.reading = True
        self.read_thread = threading.Thread(target=self.read, args=(signal_emitter,))
        self.send_thread = threading.Thread(target=self.send_commands)
        self.read_thread.start()
        self.send_thread.start()

    def read(self, signal_emitter):
        with serial.Serial(self.serial_port, self.baud_rate, timeout=self.timeout) as ser:
            print(f"Serial port {self.serial_port} opened successfully.")
            with open(self.csv_filename, mode='a', newline='') as file:
                writer = csv.writer(file)
                while self.reading:
                    try:
                        # Reads line until 'COSMOS' is found
                        line = ser.read_until(b'COSMOS').decode('utf-8').strip()
                        self.lastPacket = line
                        if line:
                            self.receivedPacketCount += 1
                            self.parse_csv_data(line)
                            signal_emitter.emit_signal()
                            writer.writerow(line.split(','))
                    except Exception as e:
                        print(f"Error: {e}")

    def send_commands(self):
        while self.reading:
            with self.command_lock:
                if self.command_queue:
                    command = self.command_queue.pop(0)
                    try:
                        with serial.Serial(self.serial_port, self.baud_rate, timeout=self.timeout) as ser:
                            command_to_send = f"{command}\n"
                            ser.write(command_to_send.encode('utf-8'))
                            print(f"Command sent: {command}")
                    except serial.SerialException as e:
                        print(f"Failed to send command: {e}")
            time.sleep(0.1)  # Adjust the sleep time as needed

    def send_command(self, command):
        with self.command_lock:
            self.command_queue.append(command)

    def stop_communication(self):
        self.reading = False
        if self.read_thread:
            self.read_thread.join()
        if self.send_thread:
            self.send_thread.join()
        print("Communication stopped.")

    #Simulation mode, reads data from a CSV file and sends it to the serial port
    def simulation_mode(self, csv_filename):
        self.simulation = True
        with open(csv_filename, mode='r') as file:
            csv_reader = csv.reader(file)
            for line in csv_reader:
                if not self.simulation:
                    break
                self.send_command(f'CMD,3195,SIMP,{line}')
                time.sleep(1)  # Add a delay to simulate real-time data sending

    #Stops reading data from serial port
    def stop_reading(self):
        self.reading = False
        print("Reading stopped.")

    #Splits lines with commas and appends them to a list
    def parse_csv_data(self, data):
        csv_data = data.split(',')
        self.data_list.append(csv_data)

    #Returns data list
    def get_data(self):
        return self.data_list

    #Telemetry data getters
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
                return float(self.data_list[-1][5])
            except (IndexError, ValueError):
                return None
        return None

    def get_TEMPERATURE(self):
        if self.data_list:
            try:
                return float(self.data_list[-1][6])
            except (IndexError, ValueError):
                return None
        return None

    def get_PRESSURE(self):
        if self.data_list:
            try:
                return float(self.data_list[-1][7])
            except (IndexError, ValueError):
                return None
        return None

    def get_VOLTAGE(self):
        if self.data_list:
            try:
                return float(self.data_list[-1][8])
            except (IndexError, ValueError):
                return None
        return None

    def get_GYRO_R(self):
        if self.data_list:
            try:
                return float(self.data_list[-1][9])
            except (IndexError, ValueError):
                return None
        return None

    def get_GYRO_P(self):
        if self.data_list:
            try:
                return float(self.data_list[-1][10])
            except (IndexError, ValueError):
                return None
        return None

    def get_GYRO_Y(self):
        if self.data_list:
            try:
                return float(self.data_list[-1][11])
            except (IndexError, ValueError):
                return None
        return None

    def get_ACCEL_R(self):
        if self.data_list:
            try:
                return float(self.data_list[-1][12])
            except (IndexError, ValueError):
                return None
        return None

    def get_ACCEL_P(self):
        if self.data_list:
            try:
                return float(self.data_list[-1][13])
            except (IndexError, ValueError):
                return None
        return None

    def get_ACCEL_Y(self):
        if self.data_list:
            try:
                return float(self.data_list[-1][14])
            except (IndexError, ValueError):
                return None
        return None

    def get_MAG_R(self):
        if self.data_list:
            try:
                return float(self.data_list[-1][15])
            except (IndexError, ValueError):
                return None
        return None

    def get_MAG_P(self):
        if self.data_list:
            try:
                return float(self.data_list[-1][16])
            except (IndexError, ValueError):
                return None
        return None

    def get_MAG_Y(self):
        if self.data_list:
            try:
                return float(self.data_list[-1][17])
            except (IndexError, ValueError):
                return None
        return None

    def get_AUTO_GYRO_ROTATION_RATE(self):
        if self.data_list:
            try:
                return float(self.data_list[-1][18])
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
                return float(self.data_list[-1][21])
            except (IndexError, ValueError):
                return None

    def get_GPS_LONGITUDE(self):
        if self.data_list:
            try:
                return float(self.data_list[-1][22])
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
