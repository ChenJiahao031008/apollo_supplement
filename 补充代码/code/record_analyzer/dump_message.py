#!/usr/bin/env python3

###############################################################################
# Copyright 2018 The Apollo Authors. All Rights Reserved.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
###############################################################################

import argparse
import sys
import os

from cyber.python.cyber_py3.record import RecordReader
from modules.canbus.proto import chassis_pb2
from modules.control.proto import control_cmd_pb2
from modules.drivers.proto import pointcloud_pb2
from modules.drivers.proto import conti_radar_pb2
# from modules.drivers.proto import sensor_image_pb2
from modules.perception.proto import perception_obstacle_pb2
from modules.planning.proto import planning_pb2
from modules.localization.proto import localization_pb2


def create_folder(filename):
    filename = filename.strip()
    filename = filename.rstrip("\\")
    is_exists = os.path.exists(filename)

    if not is_exists:
        os.makedirs(filename)
        print("[INFO] Build " + filename + " success!")
        return  True
    else:
        print("[INFO] Folder already exists.")
        return False


if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Recode Analyzer is a tool to analyze record files.",
        prog="main.py")

    parser.add_argument(
        "-f", "--file", action="store", type=str, required=True,
        help="Specify the record file for message dumping.")

    parser.add_argument(
        "-m", "--message", action="store", type=str, required=True,
        help="Specify the message topic for dumping.")

    parser.add_argument(
        "-o", "--output", action="store", type=str, required=True,
        help="which folder to output.")

    args = parser.parse_args()

    record_file = args.file
    reader = RecordReader(record_file)
    create_folder(args.output)

    for msg in reader.read_messages():
        timestamp = msg.timestamp / float(1e9)
        if args.message == "/apollo/localization/pose" and msg.topic == args.message:
            localization = localization_pb2.LocalizationEstimate()
            localization.ParseFromString(msg.message)
            file_name = args.output + "/" + str(timestamp) + ".txt"
            with open(file_name, 'w') as f:
                f.write(str(localization))
            continue
        if args.message == "/apollo/sensor/radar/front" and msg.topic == args.message:
            radar = conti_radar_pb2.ContiRadar()
            radar.ParseFromString(msg.message)
            file_name = args.output + "/" + str(timestamp) + ".txt"
            with open(file_name, 'w') as f:
                f.write(str(radar))
            continue
        if msg.topic == args.message:
            perception_obstacles = \
                perception_obstacle_pb2.PerceptionObstacles()
            perception_obstacles.ParseFromString(msg.message)
            file_name = args.output + "/" + str(timestamp) + ".txt"
            with open(file_name, 'w') as f:
                f.write(str(perception_obstacles))
