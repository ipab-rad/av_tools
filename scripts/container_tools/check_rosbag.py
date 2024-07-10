#!/usr/bin/env python3

"""This script reads MCAP files to summarize sensor data message losses."""

import argparse
import os
from collections import OrderedDict

from colorama import Fore, Style

from mcap.reader import make_reader

# Sensors frequencies
cameras_freq = 20
velodynes_freq = 9.92
ouster_freq = 10

# Topics frequencies dict
topic_frequencies = {
    '/sensor/camera/fsp_l/image_raw': cameras_freq,
    '/sensor/camera/fsp_l/image_rect_color': cameras_freq,
    '/sensor/camera/lspr_l/image_raw': cameras_freq,
    '/sensor/camera/lspr_l/image_rect_color': cameras_freq,
    '/sensor/camera/lspf_r/image_raw': cameras_freq,
    '/sensor/camera/lspf_r/image_rect_color': cameras_freq,
    '/sensor/camera/rsp_l/image_raw': cameras_freq,
    '/sensor/camera/rsp_l/image_rect_color': cameras_freq,
    '/sensor/camera/rspf_l/image_raw': cameras_freq,
    '/sensor/camera/rspf_l/image_rect_color': cameras_freq,
    '/sensor/camera/rspr_r/image_raw': cameras_freq,
    '/sensor/camera/rspr_r/image_rect_color': cameras_freq,
    '/sensor/lidar/left/points': velodynes_freq,
    '/sensor/lidar/right/points': velodynes_freq,
    '/sensor/lidar/top/points': ouster_freq,
}

topics_summary_dict = OrderedDict()


def get_mcap_summary(mcap_file_path):
    """
    Read an MCAP file and check msgs loss.

    Parameters:
    mcap_file_path (str): The path to the MCAP file.
    """
    with open(mcap_file_path, 'rb') as f:
        # Read mcap
        reader = make_reader(f)
        # Extract summary
        mcap = reader.get_summary()

        # Extract message start and end times in nanoseconds
        start_time_ns = mcap.statistics.message_start_time
        end_time_ns = mcap.statistics.message_end_time
        # Calculate duration in seconds
        duration_sec = (end_time_ns - start_time_ns) / 1e9

        topic_summaries = {}
        for idx, channel in mcap.channels.items():
            topic_name = channel.topic
            if idx in mcap.statistics.channel_message_counts:
                msg_count = mcap.statistics.channel_message_counts[idx]

            if topic_name in topic_frequencies:
                # Calculate loss data
                expected_msg_count = int(
                    topic_frequencies[topic_name] * duration_sec
                )
                msg_count_loss = msg_count - expected_msg_count
                msg_time_loss = (
                    msg_count_loss / topic_frequencies[topic_name] * 1000
                )

                color = Fore.WHITE
                if msg_count_loss == 0:
                    # No msgs were lost
                    color = Fore.LIGHTGREEN_EX
                elif msg_count_loss > 0:
                    # We are missing msgs
                    color = Fore.CYAN
                else:
                    # We have more msgs
                    color = Fore.LIGHTYELLOW_EX

                color_highlight = Fore.LIGHTWHITE_EX

                msg_time_loss = f'{msg_time_loss:.2f}'

                topic_summary = (
                    f'{color_highlight}{topic_name:40}{Style.RESET_ALL}'
                    f'count: {msg_count:4}\t'
                    f'expected_count: {expected_msg_count:4}\t'
                    f'msgs_lost: {color}{msg_count_loss} '
                    f'({msg_time_loss:3} ms){Style.RESET_ALL}'
                )

                topic_summaries[topic_name] = topic_summary
        print(f'{os.path.basename(mcap_file_path)}:')
        print(
            f'\t{Fore.LIGHTWHITE_EX}Duration: '
            f'{Fore.MAGENTA}{duration_sec} s{Style.RESET_ALL}'
        )
        for topic_name in sorted(topic_summaries.keys()):
            print(f'\t{topic_summaries[topic_name]}')

    return True


def main():
    """Parse arguments."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        'input', help='input bag path (folder or filepath) to read from'
    )

    args = parser.parse_args()
    input_path = args.input

    if os.path.isdir(input_path):
        # Get all .mcap files in the directory
        for file_name in sorted(os.listdir(input_path)):
            if file_name.endswith('.mcap'):
                file_path = os.path.join(input_path, file_name)
                get_mcap_summary(file_path)
    elif os.path.isfile(input_path) and input_path.endswith('.mcap'):
        get_mcap_summary(input_path)
    else:
        print('The input path is not a valid file or directory')


if __name__ == '__main__':
    main()
