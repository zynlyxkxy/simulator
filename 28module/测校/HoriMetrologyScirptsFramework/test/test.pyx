# poly_iter.pyx
import numpy as np
cimport numpy as np

def iterate_poly_save(double x0,
                      np.ndarray[double, ndim=1] coeffs,
                      int n,
                      np.ndarray[double, ndim=1] out):
    """
    迭代计算: x_{i+1} = x_i + poly(x_i)，并保存每一步结果

    参数
    ----------
    x0 : float
        初始值
    coeffs : 1D NumPy 数组, dtype=float64
        多项式系数，从高次项到低次项（与 np.poly1d.coeffs 一致）
    n : int
        迭代次数（结果长度）
    out : 1D NumPy 数组, dtype=float64, 长度 >= n
        用于存储迭代结果的输出数组，第 i 个位置存放 x_i
    """
    cdef int i, j, deg = coeffs.shape[0] - 1
    cdef double x = x0, poly_val

    # 内存视图：允许 C 级别快速访问
    cdef double[:] c = coeffs
    cdef double[:] o = out

    o[0] = x0
    for i in range(1, n):
        # Horner 方法计算多项式值
        poly_val = c[0]
        for j in range(1, deg + 1):
            poly_val = poly_val * x + c[j]
        x = x + poly_val
        o[i] = x

    # 不需要显式返回，因为 out 已经被原地修改