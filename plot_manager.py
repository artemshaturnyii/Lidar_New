"""
Менеджер графиков — управление matplotlib визуализацией LiDAR данных.

Отвечает за:
- Создание figure/axis/canvas
- Отрисовку фона, сканов, точек касания, углов
- Обновление графика в реальном времени
"""

from typing import List, Optional, Dict
from lidar_sdk import Point
import numpy as np

import matplotlib
matplotlib.use('TkAgg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.figure import Figure


class PlotManager:
    """Управляет полярным графиком LiDAR данных."""

    def __init__(self, parent_frame, figsize=(8, 8), dpi=100):
        """
        Args:
            parent_frame: Tkinter frame для размещения графика
            figsize: Размер figure в дюймах
            dpi: Разрешение
        """
        self.parent_frame = parent_frame
        self.figsize = figsize
        self.dpi = dpi

        self.fig: Optional[Figure] = None
        self.ax = None
        self.canvas: Optional[FigureCanvasTkAgg] = None

        self._setup_plot()

    def _setup_plot(self):
        """Создаёт figure, axis и canvas."""
        self.fig = Figure(figsize=self.figsize, dpi=self.dpi)
        self.ax = self.fig.add_subplot(111, projection='polar')
        self.ax.set_theta_zero_location('N')
        self.ax.set_theta_direction(-1)
        self.ax.set_title("LiDAR Scan (Polar Coordinates)")

        self.canvas = FigureCanvasTkAgg(self.fig, master=self.parent_frame)
        self.canvas.get_tk_widget().grid(row=0, column=0, sticky="nsew")

    def get_canvas(self) -> FigureCanvasTkAgg:
        """Возвращает canvas для размещения в GUI."""
        return self.canvas

    def update_plot(self,
                    background_map: Optional[Dict[float, float]] = None,
                    scan: Optional[List[Point]] = None,
                    touch_points: Optional[List[Point]] = None,
                    corners: Optional[List[dict]] = None):
        """
        Обновляет график с новыми данными.

        Args:
            background_map: Словарь {угол: расстояние} фона
            scan: Текущий скан (список точек)
            touch_points: Точки касания для отображения
            corners: Углы проекции
        """
        try:
            self.ax.clear()
            self.ax.set_theta_zero_location('N')
            self.ax.set_theta_direction(-1)
            self.ax.set_title("LiDAR Scan (Polar Coordinates)")

            # Фон
            if background_map:
                angles = list(background_map.keys())
                distances = list(background_map.values())
                self.ax.plot(np.radians(angles), distances, 'b-',
                             linewidth=1, label='Background')

            # Текущий скан
            if scan:
                angles = [p.angle for p in scan]
                distances = [p.distance for p in scan]
                self.ax.scatter(np.radians(angles), distances,
                                c='blue', s=10, alpha=0.6, label='Scan Points')

            # Точки касания
            if touch_points and len(touch_points) > 0:
                touch_angles = [p.angle for p in touch_points]
                touch_distances = [p.distance for p in touch_points]
                self.ax.scatter(np.radians(touch_angles), touch_distances,
                                c='red', s=50, alpha=0.9, marker='o',
                                label='Touch Points')

            # Углы проекции
            if corners:
                corner_angles = [c['angle'] for c in corners]
                corner_distances = [c['distance'] for c in corners]
                self.ax.scatter(np.radians(corner_angles), corner_distances,
                                c='orange', s=100, alpha=0.9, marker='s',
                                label='Projection Corners',
                                edgecolors='black', linewidth=2)

            self.ax.legend()
            self.canvas.draw()

        except Exception as e:
            print(f"Plot update error: {e}")

    def clear_plot(self):
        """Очищает график."""
        try:
            self.ax.clear()
            self.ax.set_theta_zero_location('N')
            self.ax.set_theta_direction(-1)
            self.ax.set_title("LiDAR Scan (Polar Coordinates)")
            self.canvas.draw()
        except Exception:
            pass
