
import numpy as np
from typing import List, Tuple, Callable
from lidar_sdk import Point

def build_background_map(collected_scans: List[List[Point]], step_deg: float = 1.0) -> Tuple[dict, Callable]:
    """
    Строит фоновую карту, интерполируя все собранные сканы на равномерную сетку углов.
    Для каждого угла сетки берём среднее (или медиану) расстояний из всех сканов,
    используя ближайшие точки или интерполяцию.
    
    Returns:
        tuple: (background_map_dict, interpolation_function)
    """
    if not collected_scans:
        return {}, None

    # Определяем сетку углов (0, step, 2*step, ..., 360-step)
    grid_angles = np.arange(0, 360, step_deg)
    grid_distances = np.full_like(grid_angles, np.nan, dtype=float)

    # Для каждого угла сетки соберём все измерения из всех сканов,
    # которые попадают в окрестность этого угла (например, ± step_deg/2)
    half_step = step_deg / 2.0
    for i, target_angle in enumerate(grid_angles):
        distances = []
        for scan in collected_scans:
            # Ищем точки, угол которых близок к target_angle
            # (учитываем переход через 0)
            for p in scan:
                # Нормализуем разницу углов к [-180, 180]
                diff = (p.angle - target_angle + 180) % 360 - 180
                if abs(diff) <= half_step:
                    distances.append(p.distance)
        if distances:
            # Используем медиану для устойчивости к выбросам
            grid_distances[i] = np.median(distances)

    # Интерполируем пропущенные углы (если есть)
    # Заполняем линейной интерполяцией
    valid = ~np.isnan(grid_distances)
    if np.any(valid):
        grid_distances = np.interp(
            grid_angles,
            grid_angles[valid],
            grid_distances[valid],
            left=grid_distances[valid][0],
            right=grid_distances[valid][-1]
        )
    else:
        # Если ни одного измерения не нашлось — используем максимальное значение
        all_distances = [p.distance for scan in collected_scans for p in scan]
        if all_distances:
            max_dist = np.nanmax(all_distances)
            grid_distances.fill(max_dist)

    # Создаем карту как словарь для быстрого доступа по углу
    background_map = dict(zip(grid_angles, grid_distances))
    
    # Создаем функцию интерполяции на будущее
    def background_interp(angle):
        return np.interp(angle, grid_angles, grid_distances, period=360)
    
    return background_map, background_interp

