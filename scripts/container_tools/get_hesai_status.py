import argparse
import socket
import sys
import time

from enum import Enum
from dataclasses import dataclass
from typing import Dict

class Hesai(Enum):
    OT128 = 1
    QT128 = 2

@dataclass(frozen=True)
class CommandData:
    payload: bytes
    expected_payload_size: Dict[Hesai, int]

class Command:
    # Get Lidar Status (0x09)
    STATUS = CommandData(
        payload=bytes([0x47, 0x74, 0x09, 0x01, 0x00, 0x00, 0x00, 0x00]),
        expected_payload_size={
            Hesai.OT128: 54,
            Hesai.QT128: 58,
        }
    )
    # Get PTP Diagnostics (0x06)
    PTP_STATUS = CommandData(
        payload=bytes([0x47, 0x74, 0x06, 0x01, 0x00, 0x00, 0x00, 0x01, 0x01]),
        expected_payload_size={
            Hesai.OT128: 16,
            Hesai.QT128: 24,
        }
    )
    # Restart (0x10)
    RESTART = CommandData(
        payload=bytes([0x47, 0x74, 0x10, 0x01, 0x00, 0x00, 0x00, 0x00]),
        expected_payload_size={
            Hesai.OT128: 0,
            Hesai.QT128: 1,
        }
    )


class HesaiLidar():
    def __init__(self, name: str, ip: str, port: int, lidar_model: str):
        self.name = name
        self.ip = ip
        self.port = port
        
        if lidar_model == 'qt':
            self.lidar_model = Hesai.QT128
        elif lidar_model == 'ot':
            self.lidar_model = Hesai.OT128
        else:
            raise ValueError(f"Invalid lidar model: {lidar_model}")
        
        # For reference
        self.HEADER_SIZE = 8
        
        # Intisliase socket
        self.s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self.s.connect((self.ip, self.port))

    def __del__(self):
        print(f"[{self.name}] Closing socket")
        self.s.close()

    def send_command(self, command: CommandData) -> tuple[bytes, bytes]:
        cmd  = command.payload
        expected_response_size = self.HEADER_SIZE + command.expected_payload_size[self.lidar_model]
        try:
            self.s.send(cmd)
            
            resp = b""
            while len(resp) < expected_response_size:
                chunk = self.s.recv(2048)
                if not chunk:
                    raise RuntimeError(f"[{self.name}] socket closed unexpectedly")
                resp += chunk
            header = resp[0:self.HEADER_SIZE]
            payload = resp[self.HEADER_SIZE: -1]
            
            return header, payload
        except socket.error as e:
            raise RuntimeError(f"[{self.name}] socket error: {e}")
        
    def get_payload_size(self, response_header) -> int:
        return int.from_bytes(response_header[4:8], byteorder= 'big')


    def print_data_meta(self, name: str, data):
        bytes_list = [f"{b:02X}" for b in data]
        print(f'{name}:\n'
            f'\tBytes Num: {len(data)}\n'
            f'\tBytes: {bytes_list}')

    def what_ptp_status(self, number: int) -> str:
        ptp_status = None 
        if number == 0:
            ptp_status = 'Free run'
        elif number == 1:
            ptp_status = 'Tracking'
        elif number == 2:
            ptp_status = 'Locked'
        elif number == 3:
            ptp_status = 'Frozen'
        return ptp_status
    
    def get_ptp_offset(self, ptp_status_payload) -> int:
        # Based on QT128C2x_TCP_API_1.4.pdf
        return int.from_bytes(ptp_status_payload[0:8], byteorder='big', signed=True)
    
    def get_ptp_state(self, ptp_status_payload) -> str:
        state = None
        if self.lidar_model == Hesai.OT128:
            state = int.from_bytes(ptp_status_payload[8: 8 + 4], byteorder= 'big')
        elif self.lidar_model == Hesai.QT128:
            state = int.from_bytes(ptp_status_payload[16: 16 + 4], byteorder= 'big')

        ptp_states = {
            0: 'NONE',
            1: 'INITIALIZING',
            2: 'FAULTY',
            3: 'DISABLED',
            4: 'LISTENING',
            5: 'PRE_MASTER',
            6: 'MASTER',
            7: 'PASSIVE',
            8: 'UNCALIBRATED',
            9: 'SLAVE',
            10: 'GRAND_MASTER'
        }

        return ptp_states.get(state, 'UNKNOWN-STATE')

    def get_ptp_elapsed_time(self, ptp_status_payload) -> int:
        
        if self.lidar_model == Hesai.OT128:
            ptp_elapsed_time = int.from_bytes(ptp_status_payload[12: 12 + 4], byteorder= 'big')
        elif self.lidar_model == Hesai.QT128:
            ptp_elapsed_time = int.from_bytes(ptp_status_payload[20: 20 + 4], byteorder= 'big')

        return ptp_elapsed_time
   
    def get_ptp_status_offest_avg(self, ptp_status_payload) -> int:
        ptp_status_offset_avg = None
        if self.lidar_model == Hesai.OT128:
            # This is not supported in the OT128
            print('PTP status average offset is not supported in the OT128')
        elif self.lidar_model == Hesai.QT128:
            ptp_status_offest_avg = int.from_bytes(ptp_status_payload[8: 8 + 8], byteorder= 'big')

        return ptp_status_offest_avg

    def get_motor_speed(self, status_payload) -> int:
        return int.from_bytes(status_payload[4: 4 + 2], byteorder= 'big')
        
    def get_ptp_status(self, status_payload) -> str:
        ptp_status = None
        if self.lidar_model == Hesai.OT128:
            ptp_status = int.from_bytes(status_payload[48: 48 + 1], byteorder= 'big')
        elif self.lidar_model == Hesai.QT128:
            ptp_status = int.from_bytes(status_payload[52: 52 + 1], byteorder= 'big')
            
        return self.what_ptp_status(ptp_status)
        
    def get_lidar_status(self) -> str:
        
        header, status_payload = self.send_command(Command.STATUS)
        ptp_header, ptp_payload = self.send_command(Command.PTP_STATUS)

        motor_speed = self.get_motor_speed(status_payload)
        ptp_status = self.get_ptp_status(status_payload)
        ptp_offset_ns = self.get_ptp_offset(ptp_payload)
        ptp_status_offset_avg = None
        if self.lidar_model == Hesai.QT128:
            ptp_status_offset_avg = self.get_ptp_status_offest_avg(ptp_payload)
        
        ptp_sate = self.get_ptp_state(ptp_payload)
        ptp_elapsed_time_ms = self.get_ptp_elapsed_time(ptp_payload)
    
        status = (f'{self.name} ({self.ip}) status:\n'
                f'\tMotor speed (RPM)  : {motor_speed}\n'
                f'\tPTP status         : {ptp_status}\n'
                f'\t  PTP state             :  {ptp_sate}\n'
                f'\t  PTP elapsed time (ms) :  {ptp_elapsed_time_ms}\n'
                f'\t  PTP offset (ns)       :  {ptp_offset_ns}\n'
                )
        if ptp_status_offset_avg is not None:
            status += f'\t  PTP offset avg (ns)   :  {ptp_status_offset_avg}\n'

        return status
    
    def restart(self):
        print(f"[{self.name}]: Restarting")
        header, payload = self.send_command(Command.RESTART)
        # if self.lidar_model == Hesai.QT128:
        #     if payload[0] == 0x00:
        #         print(f"[{self.name}] Restart command sent successfully")
        #     else:
        #         print(f"[{self.name}] Restart command failed")
        # else:
        #     print(f"[{self.name}] Restart command not supported for this model")
    

if __name__ == '__main__':
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description="Hesai Lidar Status Script")
    parser.add_argument('--ot', action='store_true', help="Monitor OT128")
    parser.add_argument('--qt1', action='store_true', help="Monitor QT128 #1")
    parser.add_argument('--qt2', action='store_true', help="Monitor QT128 #2")
    parser.add_argument('--qt3', action='store_true', help="Monitor QT128 #3")

    args = parser.parse_args()

    ot_flag = args.ot
    qt1_flag = args.qt1
    qt2_flag = args.qt2
    qt3_flag = args.qt3
    data_port = 9347

    if ot_flag:
        print('Connecting to OT128')
        ot = HesaiLidar('OT','172.31.5.50', data_port, 'ot')
    if qt1_flag:
        print('Connecting to QT128 #1')
        qt1 = HesaiLidar('QT #1','172.31.6.51', data_port, 'qt')
    if qt2_flag:
        print('Connecting to QT128 #2')
        qt2 = HesaiLidar('QT #2','172.31.7.52', data_port, 'qt')
    if qt3_flag:
        print('Connecting to QT128 #3')
        qt3 = HesaiLidar('QT #3','172.31.8.53', data_port, 'qt')

    # Check if the --restart flag is set
    #restart_flag = args.restart

    #if restart_flag:
    #    qt2.restart()

    counter = 1
    try:
        while True:
            lidar_status = f'Check {counter}\n\n'
            if ot_flag:
                lidar_status += ot.get_lidar_status()
            if qt1_flag:
                lidar_status += qt1.get_lidar_status()
            if qt2_flag:
                lidar_status += qt2.get_lidar_status()
            if qt3_flag:
                lidar_status += qt3.get_lidar_status()
            # Move cursor to top and clear screen
            sys.stdout.write('\033[2J\033[H')
            print(lidar_status)

            time.sleep(1)
            counter += 1

    except KeyboardInterrupt:
        print('Stopped')
