import serial
import csv
import time
import threading

class Communication:

    def __init__(self, serial_port, baud_rate=115200, timeout=4, csv_filename='Flight_3195.csv'):
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

        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=self.timeout)
        except serial.SerialException as e:
            print(f"Failed to open serial port {self.serial_port}: {e}")
            self.ser = None

        with open(self.csv_filename, mode='a', newline='') as file:
            writer = csv.writer(file)
            if not file.tell():
                writer.writerow(['TEAM_ID', 'MISSION_TIME', 'PACKET_COUNT', 'MODE', 'STATE',
                                 'ALTITUDE', 'TEMPERATURE', 'PRESSURE', 'VOLTAGE', 'GYRO_R',
                                 'GYRO_P', 'GYRO_Y', 'ACCEL_R', 'ACCEL_P', 'ACCEL_Y', 'MAG_R',
                                 'MAG_P', 'MAG_Y', 'AUTO_GYRO_ROTATION_RATE', 'GPS_TIME', 'GPS_ALTITUDE',
                                 'GPS_LATITUDE', 'GPS_LONGITUDE', 'GPS_SATS', 'CMD_ECHO', 'TEAM_NAME'])

    def start_communication(self, signal_emitter):
        if self.ser is None:
            print("Serial port is not available.")
            return
        self.reading = True
        self.read_thread = threading.Thread(target=self.read, args=(signal_emitter,))
        self.send_thread = threading.Thread(target=self.send_commands)
        self.read_thread.start()
        self.send_thread.start()

    def read(self, signal_emitter):
        print(f"Serial port {self.serial_port} opened successfully.")
        with open(self.csv_filename, mode='a', newline='') as file:
            writer = csv.writer(file)
            while self.reading:
                try:
                    line = self.ser.read_until(b'COSMOS').decode('utf-8').strip()
                    self.lastPacket = line
                    if line:
                        self.receivedPacketCount += 1
                        self.parse_csv_data(line)
                        signal_emitter.emit_signal()
                        writer.writerow(line.split(','))
                except serial.SerialException as e:
                    print(f"Serial error: {e}")
                except Exception as e:
                    print(f"Error: {e}")

    def send_commands(self):
                while self.reading:
                    with self.command_lock:
                        if self.command_queue:
                            command = self.command_queue.pop(0)
                            try:
                                command_to_send = f"{command}\n"
                                for _ in range(5):  # Send the command 5 times
                                    self.ser.write(command_to_send.encode('utf-8'))
                                    print(f"Command sent: {command}")
                                    time.sleep(0.1)  # Small delay between sends
                            except serial.SerialException as e:
                                print(f"Failed to send command: {e}")
                    time.sleep(0.1)

    def send_command(self, command):
        with self.command_lock:
            self.command_queue.append(command)

    def stop_communication(self):
        self.reading = False
        if self.read_thread:
            self.read_thread.join()
        if self.send_thread:
            self.send_thread.join()
        if self.ser:
            self.ser.close()
        print("Communication stopped.")

    def change_baud_rate(self, new_baud_rate):
        """Change the baud rate dynamically."""
        if self.ser and self.ser.is_open:
            self.ser.close()  # Close the serial port
        self.baud_rate = new_baud_rate
        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=self.timeout)
            print(f"Baud rate changed to {self.baud_rate}")
        except serial.SerialException as e:
            print(f"Failed to reopen serial port with new baud rate: {e}")
            self.ser = None

    def simulation_mode(self, csv_filename):
        self.simulation = True
        threading.Thread(target=self._run_simulation, args=(csv_filename,)).start()

    def _run_simulation(self, csv_filename):
        with open(csv_filename, mode='r') as file:
            csv_reader = csv.reader(file)
            for line in csv_reader:
                if not self.simulation:
                    break
                if line and line[0] == 'CMD':
                    line[1] = "3195"
                    line_str = ','.join(line)
                    self.send_command(line_str)
                    print(f"Command sent: {line_str}")
                    time.sleep(1)

    def stop_reading(self):
        self.reading = False
        print("Reading stopped.")

    def parse_csv_data(self, data):
        csv_data = data.split(',')
        self.data_list.append(csv_data)
        self.data_list = self.data_list[-20:]

    def get_data(self):
        return self.data_list

    def reset_csv(self):
        with open(self.csv_filename, mode='w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(['TEAM_ID', 'MISSION_TIME', 'PACKET_COUNT', 'MODE', 'STATE',
                             'ALTITUDE', 'TEMPERATURE', 'PRESSURE', 'VOLTAGE', 'GYRO_R',
                             'GYRO_P', 'GYRO_Y', 'ACCEL_R', 'ACCEL_P', 'ACCEL_Y', 'MAG_R',
                             'MAG_P', 'MAG_Y', 'AUTO_GYRO_ROTATION_RATE', 'GPS_TIME', 'GPS_ALTITUDE',
                             'GPS_LATITUDE', 'GPS_LONGITUDE', 'GPS_SATS', 'CMD_ECHO', 'TEAM_NAME'])

    # Telemetry data getters
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
        return None

    def get_GPS_LATITUDE(self):
        if self.data_list:
            try:
                return float(self.data_list[-1][21])
            except (IndexError, ValueError):
                return None
        return None

    def get_GPS_LONGITUDE(self):
        if self.data_list:
            try:
                return float(self.data_list[-1][22])
            except (IndexError, ValueError):
                return None
        return None

    def get_GPS_SATS(self):
        if self.data_list:
            try:
                return self.data_list[-1][23]
            except (IndexError, ValueError):
                return None
        return None

    def get_CMD_ECHO(self):
        if self.data_list:
            try:
                return self.data_list[-1][24]
            except (IndexError, ValueError):
                return None
        return None