from MirrorError import MirrorError

import os

import numpy as np
import pandas as pd
import matplotlib.pyplot as plt


def calculate_y_mirror(df, save_path="./", sample_number_in_one_position=50, timestamp_str=""):
    # data = np.array(df.iloc[0:])
    data = df
    data0 = data[:, 0]
    data1 = data[:, 1]
    data2 = data[:, 2]
    data3 = data[:, 3]

    num = data.shape[0]

    gap = sample_number_in_one_position
    n = int(num / gap)

    x11 = np.zeros((n, 1))
    x22 = np.zeros((n, 1))
    y11 = np.zeros((n, 1))
    y22 = np.zeros((n, 1))

    for i in np.arange(0, n, 1):
        x11[i] = np.sum(data0[(i) * gap + 0:(i + 1) * gap]) / gap
        x22[i] = np.sum(data1[(i) * gap + 0:(i + 1) * gap]) / gap
        y11[i] = np.sum(data2[(i) * gap + 0:(i + 1) * gap]) / gap
        y22[i] = np.sum(data3[(i) * gap + 0:(i + 1) * gap]) / gap

    lambda0 = 632.991354e-9
    n0 = 2.879266537e-9 / (1 + 0.00367 * (294.1 - 273.15)) - 3.36528e-7 + 1
    lambda_ = lambda0 / n0
    lambda_ = lambda_ / (4 * 64)

    x1 = np.zeros((n, 1))
    x2 = np.zeros((n, 1))
    y1 = np.zeros((n, 1))
    y2 = np.zeros((n, 1))

    for i in np.arange(1, n, 1):
        x1 = x11 * lambda_
        x2 = x22 * lambda_
        y1 = y11 * lambda_
        y2 = y22 * lambda_

    rasx = np.hstack((x1, x2, y1, y2))

    # # plt.figure(dpi=300)
    # plt.plot(x1, 'r+')
    # plt.title("x1 x2")
    # plt.plot(x2, 'b+')
    # # plt.savefig('D:/123.svg', dpi=300)
    # plt.show()
    #
    # # plt.figure(dpi=300)
    # plt.plot(y1, 'r+')
    # plt.title("y1 y2")
    # plt.plot(y2, 'b+')
    # # plt.savefig('D:/123.svg', dpi=300)
    # plt.show()

    IfmWidth = 0.026
    Nmax = 26
    BkpIn = np.zeros((n, 1))
    RzDelta = np.zeros((n, 1))

    for j in np.arange(0, n, 1):
        BkpIn[j] = (x1[j] + x2[j]) / 2
        RzDelta[j] = ((x2[j] - x1[j]) - (y2[j] - y1[j])) / IfmWidth

    IdName = 2
    [MirrorRZ, MirrorTXY] = MirrorError(IfmWidth, Nmax, BkpIn, RzDelta, IdName, -1, n=n)
    OriginData = np.hstack((-x1, -x2, y1, y2, -(x1 + x2) / 2, (y1 + y2) / 2))

    MirrorTXY = MirrorTXY.T
    MirrorRZ = MirrorRZ.T

    result = np.hstack((MirrorTXY, MirrorRZ))

    df_MirrorTXY = pd.DataFrame(MirrorTXY)
    df_MirrorRZ = pd.DataFrame(MirrorRZ)

    df_MirrorTXY.reset_index(drop=True)
    df_MirrorRZ.reset_index(drop=True)

    df_MirrorTXY.to_csv(os.path.join(save_path, f"YMirrorTXY_{timestamp_str}.csv"), index=False, header=False)
    df_MirrorRZ.to_csv(os.path.join(save_path, f"YMirrorRz_{timestamp_str}.csv"), index=False, header=False)

    return MirrorTXY, MirrorRZ


if __name__ == '__main__':
    # df = pd.read_csv(r"D:\jianming\1935\现场数据\20250217\干涉仪面形处理\验证\infer_mirror_SetY0_50_12141413_cycle_0.csv")
    # df = pd.read_csv(r"D:\App_Data\MetrologyScripts\data\infer_mirror_SetY0_10_10301734_cycle_{0}.csv")
    # df = pd.read_csv(r"D:\App_Data\MetrologyScripts\data\infer_mirror_SetX0_10_10301617_cycle_{0}.csv")
    # calculate_y_mirror(df)
    df = pd.read_csv(r"D:\jianming\3000\test_data\infer_mirror_SetY(5mm)_10_03031948_cycle_0.csv")
    df_split = df[['hx1', 'hx2', 'hy1', 'hy2']]
    data_np = df_split.to_numpy()
    calculate_y_mirror(data_np, sample_number_in_one_position=10)
