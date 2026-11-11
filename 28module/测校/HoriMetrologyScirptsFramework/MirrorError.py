import numpy as np
import matplotlib.pyplot as plt
from scipy.interpolate import interp1d, UnivariateSpline, CubicSpline
from scipy.optimize import curve_fit

def MirrorError(IfmWidth=None, Nmax=None, BkpIn=None, RzDelta=None, IdName=None, isPostive=None, n=1):


    # warning off
    # IfmWidth=26 干涉仪双轴之间距离
    # Nmax:在干涉仪双轴之间的步进，即序列数
    # BkpIn:步进，如果是X向面型，就是Y向位置，如果是Y向面型，就是X向位置
    # Bkp:BreakPoint横轴
    # RzDelta：Rzy-Rzx或者Rzx-Rzy

    Bkp = BkpIn
    RzDelta_Smooth = RzDelta
    MirrorRZ = np.hstack((Bkp.T, RzDelta_Smooth.T)).reshape(2, n)
    BkpStart = Bkp[0]
    BkpEnd = Bkp[-1]
    Mmax = int((BkpEnd + IfmWidth - BkpStart)[0] / IfmWidth) + 1
    Nwidth = IfmWidth / Nmax
    # print(Bkp)
    # print(Mmax)

    BkpMatrix = np.zeros((Mmax, Nmax))
    BkpMatrixAvialable = np.zeros((Mmax, Nmax))

    BkpList = np.zeros(((Nmax)*(Mmax), 1))
    MirrorList = np.zeros(((Nmax)*(Mmax), 1))
    # print(BkpList.shape)
    for Mindex in np.arange(0, Mmax, 1):
        for Nindex in np.arange(0, Nmax, 1):
            BkpMatrix[Mindex, Nindex] = (BkpStart - IfmWidth / 2)[0] + (Mindex) * IfmWidth + (Nindex) * Nwidth
            if BkpMatrix[Mindex, Nindex] <= (BkpEnd + IfmWidth / 2):
                BkpMatrixAvialable[Mindex, Nindex] = 1
    MirrorDeltaMatrix = np.zeros((Mmax, Nmax))
    for Mindex in np.arange(0, Mmax, 1):
        for Nindex in np.arange(0, Nmax, 1):
            if Mindex == 0:
                MirrorDeltaMatrix[Mindex, Nindex] = 0
            else:
                BkpForInterp = (BkpMatrix[Mindex, Nindex] - IfmWidth / 2)
                sp3=interp1d(MirrorRZ[0, :], MirrorRZ[1, :], kind='cubic', fill_value="extrapolate")
                # RzFromInterp = np.interp(BkpForInterp, MirrorRZ[0, :], MirrorRZ[1, :])  # inter1D
                RzFromInterp=sp3(BkpForInterp)
                MirrorDeltaMatrix[Mindex, Nindex] = MirrorDeltaMatrix[Mindex-1, Nindex] + IfmWidth * RzFromInterp
                if BkpMatrixAvialable[Mindex, Nindex] == 0:
                    MirrorDeltaMatrix[Mindex, Nindex] = np.nan

    # axis([BkpStart-IfmWidth/2 BkpEnd+IfmWidth/2 -inf inf]);
    # hold on;
    # for PlotIndex in np.arange(0, Nmax, 1):
    #     plt.plot(BkpMatrix[:, PlotIndex], MirrorDeltaMatrix[:, PlotIndex], "*-")
    # plt.title('MirrorDelta(offset=0)')
    # plt.show()
    # hold off;

    MirrorDeltaOffset = np.zeros(Nmax)
    BkpBase = BkpMatrix[:, 0]
    MirrorDeltaBase = MirrorDeltaMatrix[:, 0]

    # print(MirrorDeltaMatrix)
    #BkpForSplineNow=np.zeros((Mmax, Nmax-1))
    AddPerM_Factor3=np.zeros((Mmax, Nmax-1))
    # print('1',MirrorDeltaMatrix)

    # UI = []
    def cubic_polynomial(x, a, b, c, d):
        return a * x ** 3 + b * x ** 2 + c * x + d

    def replace_nans_with_cubic_fit(x, y):
        # 获取有效的x y值
        mask = np.isfinite(y)
        try:
            params, _ = curve_fit(cubic_polynomial, x[mask], y[mask])
            nan_mask = ~ mask
            y[nan_mask] = cubic_polynomial(x[nan_mask], *params)
        except RuntimeError:
            print('Warning : The fit did not converge successful.')
        return y


    for Nindex in np.arange(1, Nmax, 1):
        SumTemp = 0
        Msum = 0
        BkpNow = BkpMatrix[:, Nindex]
        MirrorDeltaNow = MirrorDeltaMatrix.copy()[:, Nindex]

        # print(MirrorDeltaNow)
        MirrorDeltaNow = replace_nans_with_cubic_fit(BkpNow, MirrorDeltaNow)

        # print(MirrorDeltaNow)


        for Mindex in np.arange(0, Mmax, 1):
            if BkpMatrixAvialable[Mindex, Nindex] == 0:
                AddPerM_Factor1 = 0
                AddPerM_Factor2 = 0
            else:
                BkpForSplineNow = BkpMatrix[Mindex, Nindex]
                BkpForSplineBase = BkpMatrix[Mindex, 0]

                # MirrorDeltaNow = np.nan_to_num(MirrorDeltaNow)
                # print(MirrorDeltaNow)
                spl_1 = interp1d(BkpBase, MirrorDeltaBase, kind='cubic', bounds_error=False, fill_value="extrapolate")
                spl_2 = interp1d(BkpNow, MirrorDeltaNow, kind='cubic', bounds_error=False, fill_value="extrapolate")
                # print(BkpBase.shape)
                # print(MirrorDeltaBase.shape)
                AddPerM_Factor1 = np.array(MirrorDeltaNow)[Mindex] - spl_1(BkpForSplineNow)
                AddPerM_Factor2 = spl_2(BkpForSplineBase) - np.array(MirrorDeltaBase)[Mindex]
                AddPerM_Factor3[Mindex, Nindex-1] = AddPerM_Factor2

                Msum = Msum + 1
                # print(AddPerM_Factor3)
                # UI.append(AddPerM_Factor3)
            SumTemp = SumTemp + AddPerM_Factor1 + AddPerM_Factor2
            SumTemp = np.nan_to_num(SumTemp)
        MirrorDeltaOffset[Nindex] = SumTemp / 2 / Msum



    # UI = np.array(UI)

    MirrorMatrix = np.zeros((Mmax, Nmax))
    for Mindex in np.arange(0, Mmax, 1):
        for Nindex in np.arange(0, Nmax, 1):
            MirrorMatrix[Mindex, Nindex] = MirrorDeltaMatrix[Mindex, Nindex] - MirrorDeltaOffset[Nindex]


    # axis([BkpStart-IfmWidth/2 BkpEnd+IfmWidth/2 -inf inf]);

    # for PlotIndex in np.arange(0, Nmax, 1):
    #     plt.plot(BkpMatrix[:, PlotIndex], MirrorMatrix[:, PlotIndex], '*-')
    # plt.title('Mirror(ddoffset)')
    # plt.show()

    MNindex = -1
    for Mindex in np.arange(0, Mmax, 1):
        for Nindex in np.arange(0, Nmax, 1):
            if BkpMatrixAvialable[Mindex, Nindex] == 1:
                MNindex = MNindex + 1
                BkpList[MNindex] = BkpMatrix[Mindex, Nindex]
                MirrorList[MNindex] = MirrorMatrix[Mindex, Nindex]
    # print(BkpList)

    # if MirrorList[-1][0]==0:
    #     MirrorList=MirrorList[:-1]
    #     BkpList = BkpList[:-1]
    while MirrorList[-1]==0:
        MirrorList=MirrorList[:-1]
        BkpList = BkpList[:-1]
    print(MirrorList.size)
    # axis([BkpStart-IfmWidth/2 BkpEnd+IfmWidth/2 -inf inf]);
    # plt.plot(BkpList, MirrorList)
    # plt.title('Form error of a surface when all sequences are aligned into a line')
    # plt.show()
    MirrorTXY = np.zeros((2, Bkp.shape[0]))

    MirrorTXY[0, :] = Bkp.reshape(Bkp.shape[0], )
    # print(len(BkpList))
    # print(len(MirrorList))
    BkpMinusHalfIfmWidth = Bkp - IfmWidth / 2

    # xi1 = interp1d(BkpList, MirrorList, kind='linear', fill_value="extrapolate")
    BkpList=BkpList.flatten()
    MirrorList=MirrorList.flatten()
    MirrorSingleBefore = np.interp(BkpMinusHalfIfmWidth, BkpList, MirrorList)

    BkpAddHalfIfWidth = Bkp + IfmWidth / 2
    xi2 = interp1d(BkpList, MirrorList, kind='linear', fill_value="extrapolate")
    MirrorSingleAfter = xi2(BkpAddHalfIfWidth)
    MirrorTXY[1, :] = isPostive * (MirrorSingleBefore + MirrorSingleAfter).T / 2

    # print(MirrorTXY[1, :])
    # clear MatrixA;
    # clear MatrixB;
    MirrorClosNmMax = MirrorTXY.shape[1]
    MatrixA = []
    MatrixB = []
    for Index in np.arange(0, MirrorClosNmMax, 1):
        # MatrixA[Index, :] = np.hstack((MirrorTXY[1, Index], 1))
        # MatrixB[Index, 1] = MirrorTXY[2, Index]
        MatrixA.append(np.hstack((MirrorTXY[0, Index], 1)))
        MatrixB.append(MirrorTXY[1, Index])

    MatrixA = np.array(MatrixA).reshape(n, 2)
    MatrixB = np.array(MatrixB).reshape(n, 1)
    MatrixC = np.linalg.pinv(MatrixA) @ MatrixB
    # print(np.array(MatrixC).shape)
    MirrorTXY[1, :] = -(np.array(MatrixB).T - np.array(MatrixA @ MatrixC).T)
    MirrorRZ[1, :] = - MirrorRZ[1, :]

    # figure('NumberTitle','off','Name',[IdName ':滤波后面型误差']);
    # axis([BkpStart-IfmWidth/2 BkpEnd+IfmWidth/2 -inf inf]);
    # hold on;
    # plt.plot(Bkp, MirrorTXY[1, :])
    # plt.title('Filtered form error')
    # plt.show()
    return MirrorRZ, MirrorTXY
