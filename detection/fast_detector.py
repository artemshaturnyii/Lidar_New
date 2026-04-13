"""
Быстрый детектор касаний для управления мышью в реальном времени.

Lookup-таблица для O(1) доступа, векторизованные операции.
"""

import numpy as np
from typing import List, Optional, Dict
from lidar_sdk import Point


class FastTouchDetector:
    """
    Быстрая детекция касаний — lookup-таблица + векторизация.

    Логика:
    1. Lookup-таблица 0-359° для O(1) доступа к фону
    2. Векторизованный поиск кандидатов (все точки сразу)
    3. Группировка по угловому окну
    4. Центр массы лучшего кластера
    """

    def __init__(self, background_map: Dict[float, float], threshold_mm: float = 50.0,
                 angle_window: float = 1.0):
        """
        Args:
            background_map: Словарь {угол: расстояние} фоновой карты
            threshold_mm: Минимальное отклонение (мм) для считывания касания
            angle_window: Угловое окно группировки кандидатов (градусы)
        """
        self.threshold_mm = threshold_mm
        self.angle_window = angle_window

        # Lookup-таблица 0-359° — O(1) доступ вместо np.interp
        self._lut = np.full(360, 0.0, dtype=np.float64)
        angles = sorted(background_map.keys())
        distances = [background_map[a] for a in angles]
        # np.interp один раз при инициализации
        idx = np.arange(360)
        self._lut = np.interp(idx, angles, distances, period=360)

    def detect_single_point(self, scan: List[Point]) -> Optional[Point]:
        """
        Возвращает ЦЕНТР МАССЫ самого большого кластера касания.
        Векторизованная версия.
        """
        if not scan:
            return None

        # Векторизованная конвертация
        n = len(scan)
        scan_angles = np.empty(n, dtype=np.float64)
        scan_distances = np.empty(n, dtype=np.float64)
        scan_intensities = np.empty(n, dtype=np.float64)

        for i, p in enumerate(scan):
            scan_angles[i] = p.angle
            scan_distances[i] = p.distance
            scan_intensities[i] = p.intensity

        # Lookup фона — O(1) на точку, без np.interp
        idx = np.round(scan_angles).astype(np.int32) % 360
        expected = self._lut[idx]

        # Векторизованная фильтрация
        mask = (expected <= 3000) & ((expected - scan_distances) >= self.threshold_mm)

        candidates_angles = scan_angles[mask]
        candidates_distances = scan_distances[mask]
        candidates_intensities = scan_intensities[mask]

        if len(candidates_angles) == 0:
            return None

        # Группировка по углу (работаем с numpy массивами)
        clusters = self._cluster_by_angle_np(candidates_angles)

        if not clusters:
            return None

        # Лучший кластер — самый большой
        best_cluster = max(clusters, key=len)

        if len(best_cluster) == 1:
            i = best_cluster[0]
            return Point(angle=float(scan_angles[i]),
                         distance=float(scan_distances[i]),
                         intensity=float(scan_intensities[i]))

        # Центр массы кластера
        idx_c = np.array(best_cluster)
        c_angles = candidates_angles[idx_c]
        c_distances = candidates_distances[idx_c]

        # Циркулярное среднее
        rad = np.radians(c_angles)
        sin_sum = np.sum(np.sin(rad))
        cos_sum = np.sum(np.cos(rad))
        avg_angle = float(np.degrees(np.arctan2(sin_sum, cos_sum)) % 360)
        avg_distance = float(np.mean(c_distances))
        avg_intensity = float(np.mean(candidates_intensities[idx_c]))

        return Point(angle=avg_angle, distance=avg_distance, intensity=avg_intensity)

    def _cluster_by_angle_np(self, angles: np.ndarray) -> List[List[int]]:
        """
        Группирует индексы точек по угловой близости.
        angles: numpy array углов (отсортированных)
        Возвращает список списков индексов.
        """
        if len(angles) == 0:
            return []

        clusters = []
        current = [0]

        for i in range(1, len(angles)):
            diff = abs(angles[i] - angles[i - 1])
            if diff > 180:
                diff = 360 - diff
            if diff <= self.angle_window:
                current.append(i)
            else:
                clusters.append(current)
                current = [i]

        clusters.append(current)
        return clusters
