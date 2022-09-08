import sys
import numpy as np
import matplotlib.pyplot as plt



def search_end_line(lines, line_start_num):
    """search frame end line"""
    for i in range(line_start_num+1, len(lines)):
        if 'new frame:' in lines[i]:
            return i - 1
    print('End frame flag not found, may be last frame')
    exit(-1)


def search_start_line(lines, frame_num):
    """search frame start line"""
    start_str = "new frame:" + frame_num
    for i in range(len(lines)):
        if start_str in lines[i]:
            return i
    return -1


def plot_frame(ax, lines, line_st_num, line_ed_num):
    """plot frame"""
    print('line start num: ' + str(line_st_num))
    print('line end num: ' + str(line_ed_num))

    plotter = dict()
    for i in range(line_st_num, line_ed_num):
        if "plot_" in lines[i]:
            data = lines[i].split("plot_")[1]
            data = data.split(":")
            tag = data[0]
            if plotter.get(tag) is None:
                plotter[tag] = ([], [])
            data[1].replace("\n", "")
            pt =data[1].split(",")
            if not len(pt) == 2:
                print(data[1], ":not a point")
                continue
            plotter[tag][0].append(float(pt[0]))
            plotter[tag][1].append(float(pt[1]))
       
    for tag in plotter.keys():
        if len(plotter[tag][0]) > 0:
            print(tag,plotter[tag])
            ax.plot(plotter[tag][0],
                plotter[tag][1], label=tag)
   

if __name__ == '__main__':
    if len(sys.argv) != 3:
        print('Please input log_analyze.py log_path frame_num')
    file_path = sys.argv[1]
    start_frame_num = sys.argv[2]
    fin = open(file_path, 'rU')
    lines = fin.readlines()
    line_search_num = search_start_line(lines, start_frame_num)
    if line_search_num == -1:
        print('no such frame!')
        sys.exit(0)
    line_st_num = line_search_num
    line_ed_num = search_end_line(lines, line_st_num)
    fig, ax = plt.subplots(1, 1)
    plot_frame(ax, lines, line_st_num, line_ed_num)

    ax.legend()
    plt.title('sl_path')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.axis('equal')
    plt.grid(True)
    plt.show()
