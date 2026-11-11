"""
多点旋转轴标定算法

该模块提供了三种方法来标定空间中的旋转轴：
1. 原始两点法（你现有的方法）
2. 多点中垂面拟合法
3. 多点非线性优化法（最高精度）

Author: Claude
"""

import numpy as np
from scipy.optimize import minimize
from scipy.linalg import svd
from typing import List, Tuple, Optional
import warnings


# ============================================================================
# 方法1: 原始两点法 (保留原有实现作为对比)
# ============================================================================

def calculate_axis_two_points(p_a_before: list, p_b_before: list,
                              p_a_after: list, p_b_after: list) -> tuple:
    """
    原始两点法：使用两个点的中垂面交线确定旋转轴
    """
    pa_pre = np.array(p_a_before)
    pb_pre = np.array(p_b_before)
    pa_post = np.array(p_a_after)
    pb_post = np.array(p_b_after)

    n_a = pa_post - pa_pre
    m_a = (pa_post + pa_pre) / 2
    A1, B1, C1 = n_a
    D1 = -np.dot(n_a, m_a)

    n_b = pb_post - pb_pre
    m_b = (pb_post + pb_pre) / 2
    A2, B2, C2 = n_b
    D2 = -np.dot(n_b, m_b)

    axis_direction = np.cross(n_a, n_b)
    norm = np.linalg.norm(axis_direction)
    if norm < 1e-9:
        raise ValueError("无法定义旋转轴：两个中垂面平行")
    
    axis_direction = axis_direction / norm
    A3, B3, C3 = axis_direction

    M = np.array([
        [A1, B1, C1],
        [A2, B2, C2],
        [A3, B3, C3]
    ])
    V = np.array([-D1, -D2, 0])

    axis_point = np.linalg.solve(M, V)
    return axis_point, axis_direction


# ============================================================================
# 方法2: 多点中垂面拟合法 (线性最小二乘)
# ============================================================================

def calculate_axis_multi_points_linear(points_before: List[List[float]], 
                                       points_after: List[List[float]]) -> Tuple[np.ndarray, np.ndarray, dict]:
    """
    多点中垂面拟合法：使用SVD分解找到最佳拟合的旋转轴方向
    
    原理：
    - 每个点对 (P, P') 定义一个中垂面，旋转轴位于该平面上
    - 所有中垂面的法向量都垂直于旋转轴
    - 使用SVD找到与所有法向量最正交的方向（即旋转轴方向）
    
    Args:
        points_before: 旋转前各点的坐标列表 [[x1,y1,z1], [x2,y2,z2], ...]
        points_after: 旋转后各点的坐标列表
    
    Returns:
        axis_point: 旋转轴上距原点最近的点
        axis_direction: 旋转轴的单位方向向量
        stats: 统计信息字典
    """
    n_points = len(points_before)
    if n_points < 2:
        raise ValueError("至少需要2个点对")
    if len(points_after) != n_points:
        raise ValueError("旋转前后的点数必须相同")
    
    P_before = np.array(points_before)
    P_after = np.array(points_after)
    
    # 计算所有中垂面的法向量和中点
    normals = []
    midpoints = []
    
    for i in range(n_points):
        n = P_after[i] - P_before[i]
        norm = np.linalg.norm(n)
        if norm < 1e-9:
            warnings.warn(f"第 {i} 个点旋转前后位置几乎相同，已跳过")
            continue
        normals.append(n / norm)  # 归一化法向量
        midpoints.append((P_after[i] + P_before[i]) / 2)
    
    if len(normals) < 2:
        raise ValueError("有效点对不足")
    
    N = np.array(normals)  # shape: (n, 3)
    M = np.array(midpoints)
    
    # 使用SVD找到旋转轴方向
    # N 的最小奇异值对应的右奇异向量就是与所有法向量最正交的方向
    U, S, Vt = svd(N)
    axis_direction = Vt[-1]  # 最小奇异值对应的向量
    
    # 确保方向向量的主分量为正（保持一致性）
    max_idx = np.argmax(np.abs(axis_direction))
    if axis_direction[max_idx] < 0:
        axis_direction = -axis_direction
    
    # 计算轴上的一点（所有中垂面的最小二乘交点）
    # 每个中垂面方程: n_i · (x - m_i) = 0
    # 即: n_i · x = n_i · m_i
    # 加上约束: axis_direction · x = 0 (距原点最近)
    
    A = np.vstack([N, axis_direction])
    b = np.array([np.dot(normals[i], midpoints[i]) for i in range(len(normals))] + [0])
    
    # 最小二乘解
    axis_point, residuals, rank, s = np.linalg.lstsq(A, b, rcond=None)
    
    # 计算拟合质量指标
    # 1. 各点到轴的距离的标准差
    distances = []
    for i in range(n_points):
        p_mid = (P_before[i] + P_after[i]) / 2
        v = p_mid - axis_point
        dist = np.linalg.norm(v - np.dot(v, axis_direction) * axis_direction)
        distances.append(dist)
    
    # 2. 奇异值比率（越小说明轴方向越确定）
    singular_ratio = S[-1] / S[0] if S[0] > 0 else 0
    
    stats = {
        'n_points': n_points,
        'n_valid_points': len(normals),
        'singular_values': S,
        'singular_ratio': singular_ratio,
        'distance_std': np.std(distances),
        'distance_mean': np.mean(distances),
        'residual_norm': np.sqrt(np.sum(residuals)) if len(residuals) > 0 else 0
    }
    
    return axis_point, axis_direction, stats


# ============================================================================
# 方法3: 多点非线性优化法 (最高精度)
# ============================================================================

def calculate_axis_multi_points_nonlinear(points_before: List[List[float]], 
                                          points_after: List[List[float]],
                                          initial_guess: Optional[Tuple[np.ndarray, np.ndarray]] = None,
                                          optimize_angle: bool = True) -> Tuple[np.ndarray, np.ndarray, float, dict]:
    """
    多点非线性优化法：通过最小化旋转重建误差来精确标定旋转轴
    
    原理：
    - 旋转轴由轴上一点 P0 和方向向量 d 定义
    - 绕轴旋转角度 θ
    - 对每个点，使用罗德里格斯公式计算理论旋转后位置
    - 最小化所有点的理论位置与实测位置之间的误差
    
    优点：
    - 利用了旋转的刚体约束（所有点绕同一轴旋转相同角度）
    - 可以同时优化轴的位置、方向和旋转角度
    - 对测量噪声更鲁棒
    
    Args:
        points_before: 旋转前各点的坐标列表
        points_after: 旋转后各点的坐标列表
        initial_guess: 初始猜测 (axis_point, axis_direction)，如果为None则自动估计
        optimize_angle: 是否同时优化旋转角度
    
    Returns:
        axis_point: 旋转轴上距原点最近的点
        axis_direction: 旋转轴的单位方向向量
        angle: 旋转角度（弧度）
        stats: 统计信息字典
    """
    n_points = len(points_before)
    P_before = np.array(points_before)
    P_after = np.array(points_after)
    
    # 获取初始估计
    if initial_guess is None:
        # 使用线性方法获取初始估计
        try:
            p0_init, d_init, _ = calculate_axis_multi_points_linear(points_before, points_after)
        except:
            # 如果线性方法失败，使用简单估计
            p0_init = np.mean(P_before, axis=0)
            d_init = np.array([1, 0, 0])
    else:
        p0_init, d_init = initial_guess
    
    # 估计初始旋转角度
    angle_init = estimate_rotation_angle(P_before, P_after, p0_init, d_init)
    
    def rodrigues_rotation(point, axis_point, axis_dir, angle):
        """罗德里格斯旋转公式"""
        v = point - axis_point
        v_parallel = np.dot(v, axis_dir) * axis_dir
        v_perp = v - v_parallel
        
        v_rotated = (v_parallel + 
                     np.cos(angle) * v_perp + 
                     np.sin(angle) * np.cross(axis_dir, v_perp))
        
        return axis_point + v_rotated
    
    def objective_with_angle(params):
        """目标函数：最小化旋转重建误差（同时优化角度）"""
        p0 = params[:3]
        # 使用球坐标参数化方向向量，确保单位长度
        theta, phi = params[3], params[4]
        d = np.array([
            np.sin(theta) * np.cos(phi),
            np.sin(theta) * np.sin(phi),
            np.cos(theta)
        ])
        angle = params[5]
        
        total_error = 0
        for i in range(n_points):
            p_rotated = rodrigues_rotation(P_before[i], p0, d, angle)
            total_error += np.sum((p_rotated - P_after[i])**2)
        
        return total_error
    
    def objective_fixed_angle(params, angle):
        """目标函数：固定角度，只优化轴参数"""
        p0 = params[:3]
        theta, phi = params[3], params[4]
        d = np.array([
            np.sin(theta) * np.cos(phi),
            np.sin(theta) * np.sin(phi),
            np.cos(theta)
        ])
        
        total_error = 0
        for i in range(n_points):
            p_rotated = rodrigues_rotation(P_before[i], p0, d, angle)
            total_error += np.sum((p_rotated - P_after[i])**2)
        
        return total_error
    
    # 将方向向量转换为球坐标
    d_norm = d_init / np.linalg.norm(d_init)
    theta_init = np.arccos(np.clip(d_norm[2], -1, 1))
    phi_init = np.arctan2(d_norm[1], d_norm[0])
    
    if optimize_angle:
        # 完整优化
        x0 = np.concatenate([p0_init, [theta_init, phi_init, angle_init]])
        
        result = minimize(
            objective_with_angle, 
            x0, 
            method='L-BFGS-B',
            options={'maxiter': 1000, 'ftol': 1e-12}
        )
        
        p0_opt = result.x[:3]
        theta_opt, phi_opt = result.x[3], result.x[4]
        angle_opt = result.x[5]
    else:
        # 固定角度优化
        x0 = np.concatenate([p0_init, [theta_init, phi_init]])
        
        result = minimize(
            lambda params: objective_fixed_angle(params, angle_init),
            x0,
            method='L-BFGS-B',
            options={'maxiter': 1000, 'ftol': 1e-12}
        )
        
        p0_opt = result.x[:3]
        theta_opt, phi_opt = result.x[3], result.x[4]
        angle_opt = angle_init
    
    # 转换回笛卡尔坐标
    d_opt = np.array([
        np.sin(theta_opt) * np.cos(phi_opt),
        np.sin(theta_opt) * np.sin(phi_opt),
        np.cos(theta_opt)
    ])
    
    # 确保方向向量的主分量为正
    max_idx = np.argmax(np.abs(d_opt))
    if d_opt[max_idx] < 0:
        d_opt = -d_opt
        angle_opt = -angle_opt
    
    # 投影到距原点最近的点
    axis_point = p0_opt - np.dot(p0_opt, d_opt) * d_opt
    
    # 计算各点的残差
    residuals = []
    for i in range(n_points):
        p_rotated = rodrigues_rotation(P_before[i], axis_point, d_opt, angle_opt)
        residuals.append(np.linalg.norm(p_rotated - P_after[i]))
    
    stats = {
        'n_points': n_points,
        'optimization_success': result.success,
        'optimization_message': result.message,
        'final_cost': result.fun,
        'residual_mean': np.mean(residuals),
        'residual_std': np.std(residuals),
        'residual_max': np.max(residuals),
        'residuals': residuals,
        'angle_degrees': np.degrees(angle_opt)
    }
    
    return axis_point, d_opt, angle_opt, stats


def estimate_rotation_angle(points_before, points_after, axis_point, axis_dir):
    """估计旋转角度"""
    angles = []
    for i in range(len(points_before)):
        v1 = points_before[i] - axis_point
        v2 = points_after[i] - axis_point
        
        # 投影到垂直于轴的平面
        v1_perp = v1 - np.dot(v1, axis_dir) * axis_dir
        v2_perp = v2 - np.dot(v2, axis_dir) * axis_dir
        
        norm1 = np.linalg.norm(v1_perp)
        norm2 = np.linalg.norm(v2_perp)
        
        if norm1 > 1e-9 and norm2 > 1e-9:
            cos_angle = np.clip(np.dot(v1_perp, v2_perp) / (norm1 * norm2), -1, 1)
            cross = np.cross(v1_perp, v2_perp)
            sign = np.sign(np.dot(cross, axis_dir))
            angle = sign * np.arccos(cos_angle)
            angles.append(angle)
    
    return np.mean(angles) if angles else 0


# ============================================================================
# 方法4: RANSAC鲁棒估计 (处理异常点)
# ============================================================================

def calculate_axis_ransac(points_before: List[List[float]], 
                          points_after: List[List[float]],
                          n_iterations: int = 100,
                          inlier_threshold: float = 0.1) -> Tuple[np.ndarray, np.ndarray, dict]:
    """
    RANSAC方法：对异常测量点具有鲁棒性
    
    原理：
    - 随机采样点对来估计旋转轴
    - 验证其他点是否符合该轴
    - 选择内点最多的模型
    - 最后用所有内点重新拟合
    
    Args:
        points_before: 旋转前各点的坐标列表
        points_after: 旋转后各点的坐标列表
        n_iterations: RANSAC迭代次数
        inlier_threshold: 内点判断阈值（残差小于此值认为是内点）
    
    Returns:
        axis_point: 旋转轴上距原点最近的点
        axis_direction: 旋转轴的单位方向向量
        stats: 统计信息字典（包含内点信息）
    """
    n_points = len(points_before)
    P_before = np.array(points_before)
    P_after = np.array(points_after)
    
    best_inliers = []
    best_axis_point = None
    best_axis_dir = None
    
    for _ in range(n_iterations):
        # 随机选择2个点
        indices = np.random.choice(n_points, 2, replace=False)
        
        try:
            # 用两点法估计轴
            axis_point, axis_dir = calculate_axis_two_points(
                P_before[indices[0]].tolist(),
                P_before[indices[1]].tolist(),
                P_after[indices[0]].tolist(),
                P_after[indices[1]].tolist()
            )
        except:
            continue
        
        # 估计旋转角度
        angle = estimate_rotation_angle(P_before, P_after, axis_point, axis_dir)
        
        # 计算所有点的残差
        inliers = []
        for i in range(n_points):
            # 罗德里格斯旋转
            v = P_before[i] - axis_point
            v_parallel = np.dot(v, axis_dir) * axis_dir
            v_perp = v - v_parallel
            v_rotated = (v_parallel + 
                        np.cos(angle) * v_perp + 
                        np.sin(angle) * np.cross(axis_dir, v_perp))
            p_rotated = axis_point + v_rotated
            
            residual = np.linalg.norm(p_rotated - P_after[i])
            if residual < inlier_threshold:
                inliers.append(i)
        
        if len(inliers) > len(best_inliers):
            best_inliers = inliers
            best_axis_point = axis_point
            best_axis_dir = axis_dir
    
    # 使用所有内点重新拟合
    if len(best_inliers) >= 2:
        inlier_before = [points_before[i] for i in best_inliers]
        inlier_after = [points_after[i] for i in best_inliers]
        
        try:
            final_point, final_dir, angle, opt_stats = calculate_axis_multi_points_nonlinear(
                inlier_before, inlier_after
            )
        except:
            final_point = best_axis_point
            final_dir = best_axis_dir
            opt_stats = {}
    else:
        final_point = best_axis_point
        final_dir = best_axis_dir
        opt_stats = {}
    
    stats = {
        'n_points': n_points,
        'n_inliers': len(best_inliers),
        'inlier_indices': best_inliers,
        'inlier_ratio': len(best_inliers) / n_points,
        **opt_stats
    }
    
    return final_point, final_dir, stats


# ============================================================================
# 综合标定函数
# ============================================================================

def calibrate_rotation_axis(points_before: List[List[float]], 
                           points_after: List[List[float]],
                           method: str = 'nonlinear',
                           verbose: bool = True) -> Tuple[np.ndarray, np.ndarray, dict]:
    """
    综合旋转轴标定函数
    
    Args:
        points_before: 旋转前各点的坐标列表 [[x1,y1,z1], [x2,y2,z2], ...]
        points_after: 旋转后各点的坐标列表
        method: 使用的方法
            - 'two_points': 原始两点法（需要正好2个点）
            - 'linear': 多点线性最小二乘法
            - 'nonlinear': 多点非线性优化法（推荐）
            - 'ransac': RANSAC鲁棒估计法
        verbose: 是否打印详细信息
    
    Returns:
        axis_point: 旋转轴上距原点最近的点
        axis_direction: 旋转轴的单位方向向量
        stats: 统计信息字典
    """
    n_points = len(points_before)
    
    if verbose:
        print(f"\n{'='*60}")
        print(f"旋转轴标定 - 使用方法: {method}")
        print(f"点对数量: {n_points}")
        print(f"{'='*60}")
    
    if method == 'two_points':
        if n_points != 2:
            raise ValueError("两点法需要正好2个点对")
        axis_point, axis_dir = calculate_axis_two_points(
            points_before[0], points_before[1],
            points_after[0], points_after[1]
        )
        stats = {'method': 'two_points', 'n_points': 2}
        
    elif method == 'linear':
        axis_point, axis_dir, stats = calculate_axis_multi_points_linear(
            points_before, points_after
        )
        stats['method'] = 'linear'
        
    elif method == 'nonlinear':
        axis_point, axis_dir, angle, stats = calculate_axis_multi_points_nonlinear(
            points_before, points_after
        )
        stats['method'] = 'nonlinear'
        
    elif method == 'ransac':
        axis_point, axis_dir, stats = calculate_axis_ransac(
            points_before, points_after
        )
        stats['method'] = 'ransac'
        
    else:
        raise ValueError(f"未知方法: {method}")
    
    if verbose:
        print(f"\n标定结果:")
        print(f"  轴方向向量: [{axis_dir[0]:.8f}, {axis_dir[1]:.8f}, {axis_dir[2]:.8f}]")
        print(f"  轴上一点:   [{axis_point[0]:.6f}, {axis_point[1]:.6f}, {axis_point[2]:.6f}]")
        
        if 'residual_mean' in stats:
            print(f"\n拟合质量:")
            print(f"  平均残差: {stats['residual_mean']:.6f}")
            print(f"  残差标准差: {stats.get('residual_std', 'N/A')}")
        
        if 'angle_degrees' in stats:
            print(f"  旋转角度: {stats['angle_degrees']:.4f}°")
    
    return axis_point, axis_dir, stats


# ============================================================================
# 交叉验证和精度评估
# ============================================================================

def cross_validate_calibration(points_before: List[List[float]], 
                               points_after: List[List[float]],
                               n_folds: int = None) -> dict:
    """
    使用留一交叉验证评估标定精度
    
    通过留出一个点，用其他点标定，然后预测留出点的位置来评估精度
    """
    n_points = len(points_before)
    if n_folds is None:
        n_folds = n_points  # 留一法
    
    P_before = np.array(points_before)
    P_after = np.array(points_after)
    
    prediction_errors = []
    
    for i in range(n_points):
        # 留出第i个点
        train_before = [points_before[j] for j in range(n_points) if j != i]
        train_after = [points_after[j] for j in range(n_points) if j != i]
        
        if len(train_before) < 2:
            continue
            
        try:
            # 用剩余点标定
            axis_point, axis_dir, angle, _ = calculate_axis_multi_points_nonlinear(
                train_before, train_after
            )
            
            # 预测留出点的位置
            v = P_before[i] - axis_point
            v_parallel = np.dot(v, axis_dir) * axis_dir
            v_perp = v - v_parallel
            v_rotated = (v_parallel + 
                        np.cos(angle) * v_perp + 
                        np.sin(angle) * np.cross(axis_dir, v_perp))
            p_predicted = axis_point + v_rotated
            
            # 计算预测误差
            error = np.linalg.norm(p_predicted - P_after[i])
            prediction_errors.append(error)
            
        except Exception as e:
            warnings.warn(f"第{i}折验证失败: {e}")
            continue
    
    return {
        'n_folds': len(prediction_errors),
        'mean_error': np.mean(prediction_errors),
        'std_error': np.std(prediction_errors),
        'max_error': np.max(prediction_errors),
        'min_error': np.min(prediction_errors),
        'errors': prediction_errors
    }


def compare_methods(points_before: List[List[float]], 
                   points_after: List[List[float]]) -> dict:
    """比较不同方法的标定结果"""
    results = {}
    
    methods = ['linear', 'nonlinear']
    if len(points_before) >= 4:
        methods.append('ransac')
    
    for method in methods:
        try:
            axis_point, axis_dir, stats = calibrate_rotation_axis(
                points_before, points_after, method=method, verbose=False
            )
            results[method] = {
                'axis_point': axis_point,
                'axis_direction': axis_dir,
                'stats': stats
            }
        except Exception as e:
            results[method] = {'error': str(e)}
    
    # 如果有足够的点，进行交叉验证
    if len(points_before) >= 4:
        cv_results = cross_validate_calibration(points_before, points_after)
        results['cross_validation'] = cv_results
    
    return results


# ============================================================================
# 测试和演示
# ============================================================================

if __name__ == "__main__":
    np.set_printoptions(precision=6, suppress=True)
    
    print("="*70)
    print("多点旋转轴标定算法测试")
    print("="*70)
    
    # 生成测试数据
    # 定义真实的旋转轴
    true_axis_point = np.array([0.0, 0.0, -10.0])  # 轴过这个点
    true_axis_dir = np.array([0.98, 0.02, 0.0])    # 轴方向（接近X轴但有偏差）
    true_axis_dir = true_axis_dir / np.linalg.norm(true_axis_dir)
    true_angle = np.radians(5)  # 旋转5度
    
    print(f"\n真实旋转轴参数:")
    print(f"  方向向量: {true_axis_dir}")
    print(f"  轴上一点: {true_axis_point}")
    print(f"  旋转角度: {np.degrees(true_angle):.2f}°")
    
    # 定义多个测试点（比如芯片四角的标记点）
    test_points = [
        [-10.0, 10.0, 5.0],   # 点1
        [-10.0, -10.0, 5.0],  # 点2
        [10.0, 10.0, 5.0],    # 点3
        [10.0, -10.0, 5.0],   # 点4
        [0.0, 0.0, 5.0],      # 点5 (中心)
        [-5.0, 5.0, 5.0],     # 点6
    ]
    
    # 计算旋转后的位置（加入测量噪声）
    noise_level = 0.01  # 测量噪声（毫米级）
    
    def rotate_point(point, axis_point, axis_dir, angle):
        v = np.array(point) - axis_point
        v_parallel = np.dot(v, axis_dir) * axis_dir
        v_perp = v - v_parallel
        v_rotated = (v_parallel + 
                    np.cos(angle) * v_perp + 
                    np.sin(angle) * np.cross(axis_dir, v_perp))
        return axis_point + v_rotated
    
    rotated_points = []
    for p in test_points:
        p_rot = rotate_point(p, true_axis_point, true_axis_dir, true_angle)
        # 添加噪声
        p_rot += np.random.randn(3) * noise_level
        rotated_points.append(p_rot.tolist())
    
    print(f"\n测试点数量: {len(test_points)}")
    print(f"测量噪声水平: ±{noise_level} (标准差)")
    
    # 测试1: 使用两点法（原始方法）
    print("\n" + "="*70)
    print("方法1: 两点法 (原始方法)")
    print("="*70)
    
    axis_point, axis_dir = calculate_axis_two_points(
        test_points[0], test_points[1],
        rotated_points[0], rotated_points[1]
    )
    print(f"标定结果:")
    print(f"  轴方向向量: {axis_dir}")
    print(f"  轴上一点:   {axis_point}")
    print(f"方向误差: {np.degrees(np.arccos(np.clip(np.dot(axis_dir, true_axis_dir), -1, 1))):.4f}°")
    
    # 测试2: 线性多点法
    print("\n" + "="*70)
    print("方法2: 多点线性最小二乘法")
    print("="*70)
    
    axis_point, axis_dir, stats = calibrate_rotation_axis(
        test_points, rotated_points, method='linear', verbose=True
    )
    print(f"方向误差: {np.degrees(np.arccos(np.clip(np.dot(axis_dir, true_axis_dir), -1, 1))):.4f}°")
    
    # 测试3: 非线性优化法
    print("\n" + "="*70)
    print("方法3: 多点非线性优化法")
    print("="*70)
    
    axis_point, axis_dir, stats = calibrate_rotation_axis(
        test_points, rotated_points, method='nonlinear', verbose=True
    )
    print(f"方向误差: {np.degrees(np.arccos(np.clip(np.dot(axis_dir, true_axis_dir), -1, 1))):.4f}°")
    
    # 测试4: 交叉验证
    print("\n" + "="*70)
    print("交叉验证评估")
    print("="*70)
    
    cv_results = cross_validate_calibration(test_points, rotated_points)
    print(f"留一交叉验证结果:")
    print(f"  折数: {cv_results['n_folds']}")
    print(f"  平均预测误差: {cv_results['mean_error']:.6f}")
    print(f"  误差标准差: {cv_results['std_error']:.6f}")
    print(f"  最大误差: {cv_results['max_error']:.6f}")
    
    # 实际应用示例
    print("\n" + "="*70)
    print("实际应用示例 - 使用芯片四角标记点")
    print("="*70)
    
    # 零位姿态下的四个标记点
    markers_zero = [
        [-10.0, 10.0, 5.0],   # 标记1
        [-10.0, -10.0, 5.0],  # 标记2
        [10.0, 10.0, 5.0],    # 标记3
        [10.0, -10.0, 5.0],   # 标记4
    ]
    
    # 旋转Rx后的四个标记点（模拟测量值）
    markers_after_rx = [
        [-10.01, 10.12, 4.88],
        [-10.02, -9.95, 5.08],
        [9.99, 10.08, 4.92],
        [10.01, -9.98, 5.05],
    ]
    
    print("\nRx轴标定 (使用4个标记点):")
    rx_point, rx_dir, rx_stats = calibrate_rotation_axis(
        markers_zero, markers_after_rx, method='nonlinear', verbose=True
    )
    
    # 旋转Ry后的四个标记点（模拟测量值）
    markers_after_ry = [
        [-9.88, 10.01, 4.92],
        [-9.92, -10.02, 4.95],
        [10.15, 9.98, 5.12],
        [10.12, -9.99, 5.08],
    ]
    
    print("\nRy轴标定 (使用4个标记点):")
    ry_point, ry_dir, ry_stats = calibrate_rotation_axis(
        markers_zero, markers_after_ry, method='nonlinear', verbose=True
    )
