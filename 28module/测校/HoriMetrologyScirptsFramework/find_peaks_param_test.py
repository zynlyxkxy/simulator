import sys
import os
import cv2
import matplotlib
import numpy as np
import scipy.signal as signal
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
                             QLabel, QLineEdit, QPushButton, QSlider, QFileDialog, QGroupBox,
                             QCheckBox)
from PyQt5.QtCore import Qt
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.backends.backend_qt5agg import NavigationToolbar2QT as NavigationToolbar
from scipy.ndimage import percentile_filter, gaussian_filter
from matplotlib.figure import Figure
from sklearn.decomposition import PCA

matplotlib.rcParams['font.sans-serif'] = ['Microsoft YaHei']
matplotlib.rcParams['axes.unicode_minus'] = False


def extract_pca(image, n=1, isCol=True):
    if isCol:
        image = image.T
    imageCentered = image - np.mean(image, axis=0)
    pca = PCA(n)
    if n == 1:
        imagepca = pca.fit_transform(imageCentered).reshape(-1)
    else:
        imagepca = pca.fit_transform(imageCentered)[:, n - 1].reshape(-1)
    return imagepca


class FindPeaksTestWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("scipy.signal.find_peaks 测试界面")
        self.setGeometry(100, 100, 1100, 750)
        
        self.data = np.array([])
        self.peaks = np.array([])
        self.invert_data = False
        
        self.init_ui()
    
    def init_ui(self):
        central_widget = QWidget()
        central_widget.setStyleSheet("""
            QWidget {
                background-color: #f8f9fa;
                font-family: 'Microsoft YaHei', sans-serif;
            }
            QGroupBox {
                font-weight: bold;
                font-size: 14px;
                color: #2c3e50;
                border: 2px solid #3498db;
                border-radius: 8px;
                margin-top: 10px;
                padding-top: 10px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px 0 5px;
                color: #3498db;
            }
            QLabel {
                color: #34495e;
                font-size: 13px;
            }
            QLineEdit {
                border: 1px solid #bdc3c7;
                border-radius: 4px;
                padding: 5px;
                background-color: white;
                font-size: 12px;
            }
            QLineEdit:focus {
                border-color: #3498db;
                outline: none;
            }
            QPushButton {
                background-color: #3498db;
                color: white;
                border: none;
                border-radius: 4px;
                padding: 6px 15px;
                font-size: 12px;
                min-width: 80px;
            }
            QPushButton:hover {
                background-color: #2980b9;
            }
            QPushButton:pressed {
                background-color: #1f618d;
            }
            QSlider::groove:horizontal {
                height: 8px;
                background: #ecf0f1;
                border-radius: 4px;
            }
            QSlider::handle:horizontal {
                background: #3498db;
                border: 2px solid #2980b9;
                width: 16px;
                margin: -4px 0;
                border-radius: 8px;
            }
            QSlider::handle:horizontal:hover {
                background: #2980b9;
            }
            QCheckBox {
                color: #34495e;
                font-size: 13px;
                spacing: 8px;
            }
            QCheckBox::indicator {
                width: 18px;
                height: 18px;
                border-radius: 3px;
                border: 2px solid #bdc3c7;
            }
            QCheckBox::indicator:checked {
                background-color: #3498db;
                border-color: #2980b9;
            }
        """)
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)
        main_layout.setSpacing(15)
        main_layout.setContentsMargins(20, 20, 20, 20)
        
        self.figure = Figure(figsize=(10, 6), dpi=100, facecolor='#f8f9fa')
        self.canvas = FigureCanvas(self.figure)
        self.canvas.setStyleSheet("border-radius: 8px; box-shadow: 0 2px 10px rgba(0,0,0,0.1);")
        
        self.toolbar = NavigationToolbar(self.canvas, self)
        
        plot_layout = QVBoxLayout()
        plot_layout.addWidget(self.toolbar)
        plot_layout.addWidget(self.canvas)
        
        plot_widget = QWidget()
        plot_widget.setLayout(plot_layout)
        main_layout.addWidget(plot_widget)
        
        param_group = QGroupBox("参数设置")
        param_layout = QVBoxLayout(param_group)
        param_layout.setSpacing(12)
        param_layout.setContentsMargins(20, 15, 20, 15)
        
        folder_layout = QHBoxLayout()
        folder_layout.setSpacing(10)
        self.folder_edit = QLineEdit()
        self.folder_edit.setPlaceholderText("请选择包含图像文件的文件夹")
        self.folder_edit.setMinimumWidth(400)
        browse_btn = QPushButton("浏览")
        browse_btn.clicked.connect(self.browse_folder)
        folder_layout.addWidget(self.folder_edit)
        folder_layout.addWidget(browse_btn)
        param_layout.addLayout(folder_layout)
        
        invert_layout = QHBoxLayout()
        self.invert_checkbox = QCheckBox("反转数据（取负号）")
        self.invert_checkbox.stateChanged.connect(self.on_param_changed)
        invert_layout.addWidget(self.invert_checkbox)
        invert_layout.addStretch()
        param_layout.addLayout(invert_layout)
        
        self.clip_threshold_slider = self.create_slider(param_layout, "clip_threshold", 0, 100, 0)
        self.prominence_slider = self.create_slider(param_layout, "prominence", 0, 100, 1)
        self.distance_slider = self.create_slider(param_layout, "distance", 1, 200, 5)
        self.threshold_slider = self.create_slider(param_layout, "threshold", 0, 100, 0)
        self.height_min_slider, self.height_max_slider = self.create_range_sliders(param_layout, "height", 0, 100, 0, 100)
        self.width_min_slider, self.width_max_slider = self.create_range_sliders(param_layout, "width", 1, 100, 1, 10)
        
        main_layout.addWidget(param_group)

        self.on_param_changed()
    
    def create_slider(self, parent_layout, name, min_val, max_val, default_val):
        layout = QHBoxLayout()
        layout.setSpacing(10)
        
        label = QLabel(f"{name}:")
        label.setFixedWidth(80)
        
        slider = QSlider(Qt.Horizontal)
        slider.setRange(min_val, max_val)
        slider.setValue(default_val)
        slider.valueChanged.connect(self.on_param_changed)
        slider.setStyleSheet("QSlider { min-width: 300px; }")
        
        value_label = QLabel(str(default_val))
        value_label.setFixedWidth(60)
        value_label.setStyleSheet("""
            QLabel {
                background-color: #ecf0f1;
                border-radius: 4px;
                padding: 3px 8px;
                text-align: center;
                font-weight: bold;
                color: #2c3e50;
            }
        """)
        
        layout.addWidget(label)
        layout.addWidget(slider)
        layout.addWidget(value_label)
        parent_layout.addLayout(layout)
        
        slider.value_label = value_label
        slider.min_val = min_val
        slider.max_val = max_val
        return slider
    
    def create_range_sliders(self, parent_layout, name, min_val, max_val, default_min, default_max):
        layout = QHBoxLayout()
        layout.setSpacing(10)
        
        label = QLabel(f"{name}:")
        label.setFixedWidth(80)
        
        min_slider = QSlider(Qt.Horizontal)
        min_slider.setRange(min_val, max_val)
        min_slider.setValue(default_min)
        min_slider.valueChanged.connect(self.on_param_changed)
        min_slider.setStyleSheet("QSlider { min-width: 140px; }")
        
        min_value_label = QLabel(str(default_min))
        min_value_label.setFixedWidth(50)
        min_value_label.setStyleSheet("""
            QLabel {
                background-color: #ecf0f1;
                border-radius: 4px;
                padding: 3px 8px;
                text-align: center;
                font-weight: bold;
                color: #2c3e50;
            }
        """)
        
        range_label = QLabel("~")
        
        max_slider = QSlider(Qt.Horizontal)
        max_slider.setRange(min_val, max_val)
        max_slider.setValue(default_max)
        max_slider.valueChanged.connect(self.on_param_changed)
        max_slider.setStyleSheet("QSlider { min-width: 140px; }")
        
        max_value_label = QLabel(str(default_max))
        max_value_label.setFixedWidth(50)
        max_value_label.setStyleSheet("""
            QLabel {
                background-color: #ecf0f1;
                border-radius: 4px;
                padding: 3px 8px;
                text-align: center;
                font-weight: bold;
                color: #2c3e50;
            }
        """)
        
        layout.addWidget(label)
        layout.addWidget(min_slider)
        layout.addWidget(min_value_label)
        layout.addWidget(range_label)
        layout.addWidget(max_slider)
        layout.addWidget(max_value_label)
        parent_layout.addLayout(layout)
        
        min_slider.value_label = min_value_label
        max_slider.value_label = max_value_label
        
        return min_slider, max_slider
    
    def browse_folder(self):
        file_path = QFileDialog.getOpenFileName(self, "选择文件夹", f"D:/jianming/3000/3000-A/up_waferII=")
        file_path = file_path[0]
        if file_path:
            self.folder_edit.setText(file_path)
            self.load_data(file_path)
    
    def load_data(self, file_path):
        if file_path:
            try:
                image = cv2.imread(file_path, cv2.IMREAD_GRAYSCALE)
                imageColPca = extract_pca(image, 1, True)

                baseline = percentile_filter(imageColPca, percentile=10, size=75)
                baseline = gaussian_filter(baseline, sigma=3)

                x = np.array([i for i in range(baseline.shape[0])])
                y = baseline

                coeffs = np.polyfit(x, y, deg=3)
                y_fit = np.polyval(coeffs, x)

                profile_1d = imageColPca - y_fit
                profile_1d -= np.min(profile_1d)
                profile_1d = profile_1d / np.max(profile_1d)
                profile_1d = gaussian_filter(profile_1d, sigma=5)
                self.data = profile_1d

                self.on_param_changed()
            except Exception as e:
                print(f"加载数据失败: {e}")
    
    def on_param_changed(self):
        self.invert_data = self.invert_checkbox.isChecked()
        
        self.clip_threshold_slider.value_label.setText(f"{self.clip_threshold_slider.value() / 100.0:.2f}")
        self.prominence_slider.value_label.setText(f"{self.prominence_slider.value() / 100.0:.2f}")
        self.distance_slider.value_label.setText(str(self.distance_slider.value()))
        self.threshold_slider.value_label.setText(f"{self.threshold_slider.value() / 100.0:.2f}")
        self.height_min_slider.value_label.setText(f"{self.height_min_slider.value() / 100.0:.2f}")
        self.height_max_slider.value_label.setText(f"{self.height_max_slider.value() / 100.0:.2f}")
        self.width_min_slider.value_label.setText(str(self.width_min_slider.value()))
        self.width_max_slider.value_label.setText(str(self.width_max_slider.value()))
        
        if len(self.data) > 0:
            self.find_peaks_and_plot()
    
    def find_peaks_and_plot(self):
        data_to_use = -self.data + 1 if self.invert_data else self.data
        
        clip_threshold = self.clip_threshold_slider.value() / 100.0
        if clip_threshold > 0:
            data_to_use = np.maximum(data_to_use, clip_threshold)
        
        prominence = self.prominence_slider.value() / 100.0
        distance = self.distance_slider.value()
        threshold = self.threshold_slider.value() / 100.0
        height_min = self.height_min_slider.value() / 100.0
        height_max = self.height_max_slider.value() / 100.0
        width_min = self.width_min_slider.value()
        width_max = self.width_max_slider.value()
        
        prominence = prominence if prominence > 0 else None
        threshold = threshold if threshold > 0 else None
        
        if height_min > 0 or height_max < 1.0:
            height = (height_min, height_max)
        else:
            height = None
        
        if width_min > 0 or width_max < 100:
            width = (width_min, width_max)
        else:
            width = None
        
        self.peaks, _ = signal.find_peaks(data_to_use, prominence=prominence, 
                                          distance=distance, threshold=threshold,
                                          height=height, width=width)
        
        self.plot_data(data_to_use)
    
    def plot_data(self, data_to_use):
        self.figure.clear()
        ax = self.figure.add_subplot(111)
        ax.set_facecolor('#f8f9fa')
        
        ax.plot(data_to_use, color='#3498db', linewidth=2, label='数据曲线')
        
        if len(self.peaks) > 0:
            ax.plot(self.peaks, data_to_use[self.peaks], 'ro', markersize=8, 
                    markeredgecolor='#c0392b', markeredgewidth=2, label='检测峰值')
            ax.vlines(self.peaks, ymin=np.min(data_to_use), ymax=data_to_use[self.peaks], 
                      color='#e74c3c', linestyle='--', alpha=0.6)
        
        ax.legend(fontsize=12)
        ax.set_xlabel('索引', fontsize=12, color='#2c3e50')
        ax.set_ylabel('数值', fontsize=12, color='#2c3e50')
        ax.set_title('数据曲线与峰值检测结果', fontsize=14, fontweight='bold', color='#2c3e50')
        ax.grid(True, linestyle='--', alpha=0.6, color='#bdc3c7')
        ax.tick_params(axis='both', labelsize=10, color='#7f8c8d')
        
        self.canvas.draw()


if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = FindPeaksTestWindow()
    window.show()
    sys.exit(app.exec_())