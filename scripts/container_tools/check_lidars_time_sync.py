#!/usr/bin/env python3

import requests
import time
from datetime import datetime

def check_ouster_time_sync(ip_address):
    try:
        # Get the current host time in Unix epoch format
        current_time = time.time()

        # Make the GET request to the sensor's time API
        url = f'http://{ip_address}/api/v1/time'
        start_time = time.perf_counter()
        response = requests.get(url)
        end_time = time.perf_counter()

        # Measure duration the request took
        request_duration = (end_time - start_time)

        # Compensate for the request duration to align the host time with the sensor's reported time.
        # Assumption: The round-trip time (D1 + D2) is symmetric, so D1 ≈ D2.
        #  
        #  Timeline for the request:
        #  (t_start) Host  -------- D1 -------> Ouster (ouster_time)
        #  (t_end)        <-------  D2 -------
        #
        # Where:
        #   request_duration = D1 + D2
        #   If D1 == D2, then D1 = request_duration / 2
        #
        # To fairly compare the host time with the sensor's time, we adjust the current
        # host time by adding D1:
        #   current_time + D1 <--> ouster_time
        current_time += request_duration/2.0

        if response.status_code == 200:
            data = response.json()

            # Check for PPS lock in 'sync_pulse_in'
            sync_pulse_locked = data['sensor']['sync_pulse_in'].get('locked', 0)
            sync_pulse_polarity = data['sensor']['sync_pulse_in'].get('polarity', 'N/A')

            # Check for NMEA lock in 'nmea'
            nmea_locked = data['sensor']['multipurpose_io']['nmea'].get('locked', 0)
            nmea_baud_rate = data['sensor']['multipurpose_io']['nmea'].get('baud_rate', 'N/A')
            last_nmea_message = data['sensor']['multipurpose_io']['nmea']['diagnostics']['decoding'].get('last_read_message', 'N/A')

            # Check GPS timestamp in 'timestamp'
            timestamp_time = data['sensor']['timestamp'].get('time', "0.0")

            # Display synchronization status
            print("Ouster: ")
            if sync_pulse_locked == 1:
                print("\tPPS signal is OK.")
            else:
                print(f"\tPPS signal is NOT locked. Check polarity (Current polarity: {sync_pulse_polarity}).")

            if nmea_locked == 1:
                print("\tNMEA signal is OK.")
            else:
                print(f"\tNMEA signal is NOT locked. Check polarity and baud rate (Current baud rate: {nmea_baud_rate}).")

            # Verify the GPRMC sentence format for NMEA
            if last_nmea_message.startswith("GPRMC"):
                print(f"\tLast GPRMC message received: {last_nmea_message}")
            else:
                print(f"\tInvalid or missing GPRMC message: {last_nmea_message}")

            # Extract seconds and nanoseconds from the timestamp_time
            if timestamp_time:
                sensor_time_parts = str(timestamp_time).split('.')
                sensor_seconds = int(sensor_time_parts[0])  # Whole seconds part
                sensor_nanoseconds = int(sensor_time_parts[1]) if len(sensor_time_parts) > 1 else 0  # Nanoseconds part

                # Convert nanoseconds to milliseconds
                sensor_milliseconds = sensor_nanoseconds / 1e6

                # Calculate the total sensor time as a floating-point value (seconds + milliseconds)
                sensor_total_time = sensor_seconds + sensor_milliseconds / 1000.0

                # Calculate the difference between sensor and host time
                time_diff = abs(current_time - sensor_total_time)

                # Format sensor and host times to a human-readable format
                sensor_time_formatted = datetime.utcfromtimestamp(sensor_seconds).strftime("%A, %d %B %Y %H:%M:%S")
                host_time_formatted = datetime.utcfromtimestamp(current_time).strftime("%A, %d %B %Y %H:%M:%S")

                # Display formatted times
                print(f"\tSensor time (UTC): {sensor_time_formatted}")
                print(f"\tHost time (UTC): {host_time_formatted}")
                print(f"\tSensor-Host latency: {request_duration/2.0:.4f} seconds")

                if time_diff < 1:
                    print(f"\tTimestamp is valid and synchronized. Difference: {time_diff:.6f} seconds.")
                else:
                    print(f"\tTimestamp difference is too large. Difference: {time_diff:.6f} seconds.")
            else:
                print("\tTimestamp is not updated correctly.")
        
        else:
            print(f"Failed to get data from Ouster sensor. Status code: {response.status_code}")
    
    except Exception as e:
        print(f"An error occurred: {e}")



def check_velodyne_time_sync(ip,side):
        
    try:
        # Velodyne status URL
        status_url = f'http://{ip}/cgi/status.json'
        status_response = requests.get(status_url)
        data = status_response.json()

        pps_state = data['gps'].get('pps_state', 'N/A')
        position = data['gps'].get('position', 'N/A')


        print(f"Velodyne {side}")

        if pps_state == "Locked":
            print("\tPPS is OK")
        
        else:
            print("\tPPS signal is NOT locked.")
        
        if position:
            print(f"\tPosition: {position}")
        else:
            print("Position field is empty, verify NMEA connection")

    except Exception as e:
        print(f"An error occurred while checking {ip}: {e}")

# Get Lidars info
check_velodyne_time_sync("172.31.2.2","left")
check_velodyne_time_sync("172.31.2.3","right")
check_ouster_time_sync('172.31.2.10')
