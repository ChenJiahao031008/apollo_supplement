# -*- coding: utf-8 -*-
# @Time : 2022/8/5 13:54
# @Author : Zhangke Liang
# @FileName: RawRader2CSV.py
# @Email : zhangke_liang@outlook.com

import pandas as pd
import glob
import natsort
import numpy as np


def data2CSV(inputPath, outputPath='front_radar.csv'):
    """
    将原始的Radar数据转换为OpenCalib 毫米波雷达与激光雷达标定外参的格式
    :param inputPath: 原始Radar数据的文件夹
    :param outputPath: 输出转换后csv数据格式的存放地址，默认是当前文件夹下
    :return:
    """
    # rawTitleName = ['timestamp_sec', 'obstacle_id', 'longitude_vel', 'lateral_vel', 'longitude_dist', 'lateral_dist']
    pathLists = natsort.natsorted(glob.glob(inputPath + '/*.txt'))  # 获取文件夹下所有的Radar txt数据
    # 定义几个list保存数据
    all_time_ns = []
    all_track_id = []
    all_velocity_x = []
    all_velocity_y = []
    all_position_x = []
    all_position_y = []
    for path in pathLists:
        with open(path, 'r') as file:
            data = file.readlines()
            for i in range(4, len(data)):  # 跳过最前的5行header信息
                line = data[i].strip('\n').strip()  # 遍历每一行数据，去掉首尾的空格和换行符
                if 'obstacle_id' in line:  # 先定位obstacle_id的位置，根据id的位置来查找当前id的velocity x/y和position x/y以及时间戳time_ns
                    time_ns = np.double(data[i - 5].strip('\n').strip().split(':')[1].strip())
                    track_id = int(data[i].strip('\n').strip().split(':')[1])
                    velocity_x = np.double(data[i + 3].strip('\n').strip().split(':')[1])
                    velocity_y = np.double(data[i + 4].strip('\n').strip().split(':')[1])
                    position_x = np.double(data[i + 1].strip('\n').strip().split(':')[1])
                    position_y = np.double(data[i + 2].strip('\n').strip().split(':')[1])

                    all_time_ns.append(time_ns)
                    all_track_id.append(track_id)
                    all_velocity_x.append(velocity_x)
                    all_velocity_y.append(velocity_y)
                    all_position_x.append(position_x)
                    all_position_y.append(position_y)
    data = {'time_ns': all_time_ns, 'track_id': all_track_id, 'velocity_x': all_velocity_x,
            'velocity_y': all_velocity_y, 'position_x': all_position_x, 'position_y': all_position_y}
    csvTitleName = ['time_ns', 'track_id', 'velocity_x', 'velocity_y', 'position_x', 'position_y']
    csvFile = pd.DataFrame(data=data, columns=csvTitleName)
    csvFile.to_csv(outputPath, index=False)


if __name__ == '__main__':
    inputPath_ = './rawRadarData'
    data2CSV(inputPath_)
