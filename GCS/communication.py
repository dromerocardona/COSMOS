import serial
import csv
import time
import threading
import queue
import logging

# Configure logging
logging.basicConfig(level=logging.DEBUG, format='%(asctime)s - %(levelname)s - %(message)s')

class Communication:

    def __init__(self, serial_port, baud_rate=9600, timeout=4, csv_filename='Flight_3195.csv'):
        self.sim_thread = None
        self.serial_port = serial_port
        self.baud_rate = baud_rate
        self.timeout = timeout
        self.data_list = []
        self.reading = False
        self.csv_filename = csv_filename
        self.receivedPacketCount = 0
        self.lastPacket = ""
        self.command_queue = queue.Queue(maxsize=100)
        self.read_thread = None
        self.send_thread = None
        self.simulation = False
        self.simEnabled = False

        try:
            self.ser = serial.Serial(self.serial_port, self.baud_rate, timeout=self.timeout)
            # Set a short write timeout to prevent blocking
            self.ser.write_timeout = 0.1
        except serial.SerialException as e:
            logging.error(f"Failed to open serial port {self.serial_port}: {e}")
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
        """Process commands from the queue and send them at 1-second intervals."""
        last_send_time = time.time()
        while self.reading:
            try:
                # Try to get a command without blocking
                command = self.command_queue.get_nowait()
                current_time = time.time()
                # Ensure 1-second interval between sends
                if current_time - last_send_time >= 1.0:
                    start_write_time = time.time()
                    self._write_serial(command)
                    write_duration = time.time() - start_write_time
                    last_send_time = current_time
                    self.command_queue.task_done()
                    logging.debug(f"Command sent, queue size: {self.command_queue.qsize()}, write took: {write_duration:.3f}s")
                else:
                    # Put the command back if it's too soon to send
                    self.command_queue.put(command)
                    time.sleep(0.01)  # Brief sleep to prevent busy-waiting
            except queue.Empty:
                time.sleep(0.01)  # Brief sleep to prevent busy-waiting
            except Exception as e:
                logging.error(f"Error in send_commands: {e}")
                time.sleep(0.01)

    def _write_serial(self, command):
        """Helper method to write to serial port with error handling."""
        if self.ser is None or not self.ser.is_open:
            logging.error("Serial port is not available or closed")
            return
        try:
            command_to_send = f"{command}\n"
            bytes_written = self.ser.write(command_to_send.encode('utf-8'))
            if bytes_written == len(command_to_send):
                self.ser.flush()  # Ensure the command is sent
                logging.info(f"Command sent: {command}")
            else:
                logging.warning(f"Partial write for command: {command}, only {bytes_written} bytes written")
        except serial.SerialTimeoutException:
            logging.warning(f"Write timeout for command: {command}")
        except serial.SerialException as e:
            logging.error(f"Failed to send command: {e}")
        except Exception as e:
            logging.error(f"Unexpected error in _write_serial: {e}")

    def send_command(self, command):
        """Add a command to the queue."""
        try:
            self.command_queue.put(command, timeout=0.1)
            logging.debug(f"Command queued: {command}, queue size: {self.command_queue.qsize()}")
        except queue.Full:
            logging.warning("Command queue is full, dropping command")

    def flush_csv(self):
        """Flush and close the CSV file temporarily to ensure all data is written."""
        with open(self.csv_filename, mode='a', newline='') as file:
            pass

    def stop_communication(self):
        self.reading = False
        if self.read_thread:
            self.read_thread.join()
        if self.send_thread:
            self.send_thread.join()
        if self.ser:
            self.ser.close()
        self.flush_csv()
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
        self.sim_thread = threading.Thread(target=self._run_simulation, args=(csv_filename,), daemon=True)
        self.sim_thread.start()

    def _run_simulation(self, csv_filename):
        """Send commands from the CSV file at 1-second intervals."""
        try:
            with open(csv_filename, mode='r') as file:
                csv_reader = csv.reader(file)
                next(csv_reader, None)  # Skip header if present
                for line in csv_reader:
                    if not self.simulation:  # Stop simulation if disabled
                        break
                    if line and line[0] == 'CMD':
                        line[1] = '3195'
                        command = ','.join(line)
                        start_time = time.time()
                        self.send_command(command)  # Add to the command queue
                        logging.debug(f"Simulation command prepared: {command}")
                        # Sleep to maintain 1-second interval, accounting for processing time
                        elapsed = time.time() - start_time
                        remaining = max(0, 1.0 - elapsed)
                        time.sleep(remaining)
        except FileNotFoundError:
            logging.error(f"Simulation CSV file {csv_filename} not found")
        except Exception as e:
            logging.error(f"Error in simulation: {e}")
        finally:
            self.simulation = False
            logging.info("Simulation stopped")

    def stop_simulation(self):
        """Stop the simulation process."""
        self.simulation = False
        if hasattr(self, 'sim_thread') and self.sim_thread.is_alive():
            self.sim_thread.join()

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