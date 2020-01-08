import numpy as np
import pandas as pd
import socket

if __name__ == '__main__':
    data = pd.read_csv('D:/github/kinect_tracker/kinect_tracker/demo/save/2020-1-8-11-36-13/data.csv')
    print(data.head())
    print(data.columns)
    data0 = data[data['device_id'] == 0]
    data1 = data[data['device_id'] == 1]
    # Index(['device_timestamp', 'device_id', 'frame_id', 'node_id', 'x', 'y', 'z',
    #        'qua_w', 'qua_y', 'qua_z', 'qua_x'],
    #       dtype='object')

    client = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    frame = 200

    device_id = 1

    for many_times in range(10):
        if device_id == 0:
            # 设备 0
            for i in range(frame):
                ss = ""
                for j in range(32*i, 32*(i+1)):
                    # ss += str(data0.iloc[j][4]) + " " + str(data0.iloc[j][5]) + " " + str(data0.iloc[j][6]) + ","  # x, y, z
                    ss += "0 " + "0 " + "0,"  # x, y, z
                ss += "|"

                # ss += "0.0469443 " + "-0.69991 " + "-0.0742749 " + "-0.708792,"
                for j in range(32*i, 32*(i+1)):
                    ss += str(data0.iloc[j][7]) + " " + str(data0.iloc[j][8]) + " " + str(data0.iloc[j][9]) + " " + str(data0.iloc[j][10]) + ","  # qua_w, qua_x, qua_y, qua_z

                ss = ss.encode()
                client.sendto(ss, ('127.0.0.1', 8999))
                print(f'frame: {i}')
        else:
            # 设备 1
            for i in range(frame):
                ss = ""
                for j in range(32 * i, 32 * (i + 1)):
                    # ss += str(data1.iloc[j][4]) + " " + str(data1.iloc[j][5]) + " " + str(data1.iloc[j][6]) + ","  # x, y, z
                    ss += "0 " + "0 " + "0,"  # x, y, z
                ss += "|"

                for j in range(32 * i, 32 * (i + 1)):
                    ss += str(data1.iloc[j][7]) + " " + str(data1.iloc[j][8]) + " " + str(data1.iloc[j][9]) + " " + str(
                        data1.iloc[j][10]) + ","  # qua_w, qua_x, qua_y, qua_z

                ss = ss.encode()
                client.sendto(ss, ('127.0.0.1', 8999))
                print(f'frame: {i}')
    # for i in range(min(len(data0), len(data1))):
    #     # one frame
    #     frame_id = int(data0.iloc[i][2])
    #     node_id = int(data0.iloc[i][3])
    #     x, y, z = data0.iloc[i][4], data0.iloc[i][5], data0.iloc[i][6]
    #     qua_w, qua_x, qua_y, qua_z = data0.iloc[i][7], data0.iloc[i][10], data0.iloc[i][8], data0.iloc[i][9]
    #     print(qua_w, qua_x, qua_y, qua_z)
    #     for j in range(32):



