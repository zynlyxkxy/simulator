import cv2
import numpy as np
import os
import time


# 6 种焦点测量函数
def compute_local_variance(image, ksize=5):
    # gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray = image
    mean = cv2.blur(gray, (ksize, ksize))
    squared_mean = cv2.blur((gray.astype(np.float64) ** 2), (ksize, ksize))
    variance = squared_mean - (mean.astype(np.float64) ** 2)
    return np.mean(variance)


def compute_tenengrad(image):
    # gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray = image
    sobel_x = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
    sobel_y = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)
    return np.mean(np.sqrt(sobel_x ** 2 + sobel_y ** 2))


def compute_brenner_gradient(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    shifted = np.roll(gray, -2, axis=1)
    return np.sum((gray.astype(np.float64) - shifted.astype(np.float64)) ** 2)


def compute_sobel_variance(image):
    # gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray = image
    sobel_x = cv2.Sobel(gray, cv2.CV_64F, 1, 0, ksize=3)
    sobel_y = cv2.Sobel(gray, cv2.CV_64F, 0, 1, ksize=3)
    sobel_mag = np.sqrt(sobel_x ** 2 + sobel_y ** 2)
    return np.mean(sobel_mag) + np.var(gray.astype(np.float64))


def compute_laplacian(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    laplacian = cv2.Laplacian(gray, cv2.CV_64F)
    return np.var(laplacian)


if __name__ == '__main__':

    methods = {
        'LocalVariance': compute_local_variance,
        'Tenengrad': compute_tenengrad,
        'Brenner': compute_brenner_gradient,
        'SobelVariance': compute_sobel_variance,
        'Laplacian': compute_laplacian
    }

    # 设置图像目录
    image_dir = 'focus_test'  # 替换为你的图像目录路径
    output_dir = 'best_frames'
    os.makedirs(output_dir, exist_ok=True)

    # 获取目录中的所有图像文件
    image_files = [f for f in os.listdir(image_dir) if f.lower().endswith(('.png', '.jpg', '.jpeg', '.bmp', '.tiff'))]
    if not image_files:
        print("未找到图像文件！")
        exit()

    # 初始化最佳结果存储
    best_results = {name: {'score': -float('inf'), 'filename': None} for name in methods}

    # 初始化时间统计字典
    method_times = {name: [] for name in methods}  # 存储每种方法处理每张图像的时间
    total_images = len(image_files)

    # 处理每个图像文件
    for filename in image_files:
        filepath = os.path.join(image_dir, filename)
        image = cv2.imread(filepath)
        if image is None:
            print(f"无法读取图像: {filename}")
            continue

        for name, func in methods.items():
            try:
                # 记录方法开始时间
                start_time = time.perf_counter()

                # 计算焦点得分
                score = func(image)

                # 记录方法结束时间并计算耗时
                elapsed = time.perf_counter() - start_time
                method_times[name].append(elapsed)

                # 更新最佳结果
                if score > best_results[name]['score']:
                    best_results[name]['score'] = score
                    best_results[name]['filename'] = filename
            except Exception as e:
                print(f"处理 {filename} 时 {name} 方法出错: {str(e)}")

    # 输出结果并保存最佳图像
    print("\n焦点测量结果:")
    print("-" * 60)
    for name, result in best_results.items():
        if result['filename']:
            # 计算该方法的时间统计
            times = method_times[name]
            avg_time = np.mean(times) * 1000  # 转换为毫秒
            min_time = np.min(times) * 1000
            max_time = np.max(times) * 1000

            print(f"[{name}]")
            print(f"  最清晰图像: {result['filename']}")
            print(f"  分数: {result['score']:.4f}")
            print(f"  处理时间: 平均 {avg_time:.2f} ms | 最小 {min_time:.2f} ms | 最大 {max_time:.2f} ms")
            print(f"  总处理时间: {np.sum(times):.4f} 秒 (共 {len(times)} 张图像)")
            print("-" * 60)

            # 保存最佳图像
            best_image = cv2.imread(os.path.join(image_dir, result['filename']))
            out_path = os.path.join(output_dir, f"{name}_best_frame.png")
            cv2.imwrite(out_path, best_image)
        else:
            print(f"[{name}] 未找到最佳图像")

    # 输出总结信息
    print("\n时间性能总结:")
    print("=" * 60)
    print(f"处理图像总数: {total_images}")
    print("-" * 60)
    for name in methods:
        if method_times[name]:
            total_time = np.sum(method_times[name])
            avg_time = np.mean(method_times[name]) * 1000
            print(f"{name:<15}: 总时间 {total_time:.4f} 秒 | 平均 {avg_time:.2f} ms/图像")
    print("=" * 60)
