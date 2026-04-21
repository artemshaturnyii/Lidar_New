"""
Быстрый детектор касаний для управления мышью в реальном времени.
"""

import numpy as np
from typing import List, Optional, Dict
from lidar_sdk import Point


class FastTouchDetector:
    """Быстрая детекция касаний с балансом скорости и надёжности."""

    def __init__(self, background_map: Dict[float, float], threshold_mm: float = 30.0,
                 angle_window: float = 2.0):
        """
        Args:
            background_map: Словарь {угол: расстояние} фоновой карты
            threshold_mm: Минимальное отклонение (мм) для считывания касания
            angle_window: Угловое окно группировки кандидатов (градусы)
        """
        self.threshold_mm = threshold_mm
        self.angle_window = angle_window

        # Lookup-таблица 0-359°
        self._lut = np.full(360, 0.0, dtype=np.float64)
        angles = sorted(background_map.keys())
        distances = [background_map[a] for a in angles]
        idx = np.arange(360)
        self._lut = np.interp(idx, angles, distances, period=360)

        # ✅ БАЛАНС: Быстрое первое касание + удержание при движении
        self._last_touch_point: Optional[Point] = None
        self._persisted_point: Optional[Point] = None
        self._missed_frames: int = 0
        self._max_missed_frames: int = 5  # ✅ ~50мс (было 15 = 150мс)
        self._consecutive_detections: int = 0
        self._min_consecutive: int = 1

    def detect_single_point(self, scan: List[Point]) -> Optional[Point]:
        """Возвращает точку касания с персистентностью."""
        if not scan:
            self._missed_frames += 1
            if self._missed_frames > self._max_missed_frames:
                self._persisted_point = None
                self._last_touch_point = None
            return self._persisted_point

        n = len(scan)
        scan_angles = np.empty(n, dtype=np.float64)
        scan_distances = np.empty(n, dtype=np.float64)
        scan_intensities = np.empty(n, dtype=np.float64)

        for i, p in enumerate(scan):
            scan_angles[i] = p.angle
            scan_distances[i] = p.distance
            scan_intensities[i] = p.intensity

        idx = np.round(scan_angles).astype(np.int32) % 360
        expected = self._lut[idx]

        # ✅ Более чувствительный порог (30мм)
        mask = (expected <= 3000) & ((expected - scan_distances) >= self.threshold_mm)

        candidates_angles = scan_angles[mask]
        candidates_distances = scan_distances[mask]
        candidates_intensities = scan_intensities[mask]

        if len(candidates_angles) == 0:
            self._missed_frames += 1
            if self._missed_frames > self._max_missed_frames:
                self._persisted_point = None
                self._last_touch_point = None
            return self._persisted_point

        # ✅ Широкое окно группировки (2.0°)
        clusters = self._cluster_by_angle_np(candidates_angles)

        if not clusters:
            self._missed_frames += 1
            if self._missed_frames > self._max_missed_frames:
                self._persisted_point = None
                self._last_touch_point = None
            return self._persisted_point

        best_cluster = max(clusters, key=len)

        if len(best_cluster) == 1:
            i = best_cluster[0]
            current_point = Point(angle=float(scan_angles[i]),
                                  distance=float(scan_distances[i]),
                                  intensity=float(scan_intensities[i]))
        else:
            idx_c = np.array(best_cluster)
            c_angles = candidates_angles[idx_c]
            c_distances = candidates_distances[idx_c]

            rad = np.radians(c_angles)
            sin_sum = np.sum(np.sin(rad))
            cos_sum = np.sum(np.cos(rad))
            avg_angle = float(np.degrees(np.arctan2(sin_sum, cos_sum)) % 360)
            avg_distance = float(np.mean(c_distances))
            avg_intensity = float(np.mean(candidates_intensities[idx_c]))

            current_point = Point(angle=avg_angle, distance=avg_distance, intensity=avg_intensity)

        # ✅ Касание обнаружено — сбрасываем счётчик пропусков
        self._missed_frames = 0
        self._consecutive_detections += 1
        self._last_touch_point = current_point
        self._persisted_point = current_point

        return current_point

    def _cluster_by_angle_np(self, angles: np.ndarray) -> List[List[int]]:
        """Группирует индексы точек по угловой близости."""
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

    def reset(self):
        """Сбрасывает состояние детектора."""
        self._last_touch_point = None
        self._persisted_point = None
        self._missed_frames = 0
        self._consecutive_detections = 0

    def set_persistence(self, max_missed_frames: int = 5):
        """Настраивает персистентность касания."""
        self._max_missed_frames = max_missed_frames
