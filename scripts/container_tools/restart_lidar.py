import argparse
import socket

def restart_lidar(ip: str, port: int):
    # Restart command payload
    RESTART_PAYLOAD = bytes([0x47, 0x74, 0x10, 0x01, 0x00, 0x00, 0x00, 0x00])
    
    try:
        # Create a socket and connect to the Lidar
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
            s.connect((ip, port))
            print(f"Connected to {ip}:{port}")
            
            # Send the restart command
            s.send(RESTART_PAYLOAD)
            print(f"Restart command sent to {ip}:{port}")
    except socket.error as e:
        print(f"Failed to connect or send command to {ip}:{port}: {e}")

if __name__ == "__main__":
    # Parse command-line arguments
    parser = argparse.ArgumentParser(description="Restart a Hesai Lidar device")
    parser.add_argument("model", type=str, help="hesai id to restart")
    args = parser.parse_args()

    data_port = 9347
    model = args.model
    ip = ''
    if model == 'OT':
        ip = '172.31.0.50'
    elif model == 'QT_1':
        ip = '172.31.0.51'
    elif model == 'QT_2':
        ip = '172.31.0.52'
        
    # Restart the Lidar
    restart_lidar(ip, data_port)