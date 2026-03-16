import numpy as np
from typing import List, Dict, Callable, Tuple
from .data_handlers import build_background_map
from .file_io import save_background_to_file, load_background_from_file
from config import config

class LidarCalibrator:
    """Класс для управления процессом калибровки (только orchestration)"""
    
    def __init__(self):
        self._background_interp = None
    
    def collect_scans_with_callback(self, lidar, num_scans: int = 50) -> List[List]:
        """
        Собирает сканы через callback (app layer управляет этим процессом)
        Возвращает список собранных сканов
        """
        collected_scans = []
        current_count = 0
        
        def scan_callback(scan):
            nonlocal current_count
            collected_scans.append(scan)
            current_count += 1
            print(f"  Собран скан {current_count}/{num_scans}")
        
        # Регистрируем callback (app layer потом отменит его)
        lidar.on_scan_complete = scan_callback
        
        # Возвращаем пустой список - app layer сам будет собирать данные
        return collected_scans
    
    def process_scans_to_background(self, scans: List[List], step_deg: float = None) -> Dict[float, float]:
        """
        Обрабатывает собранные сканы и строит фоновую карту
        Чистая функция обработки данных
        """
        if step_deg is None:
            step_deg = config.processing['background_grid_step']
            
        background_map, interp_func = build_background_map(scans, step_deg)
        self._background_interp = interp_func
        return background_map
    
    def get_background_distance(self, angle: float) -> float:
        """Возвращает фоновое расстояние для заданного угла"""
        if self._background_interp is None:
            raise RuntimeError("Калибровка ещё не выполнена")
        return self._background_interp(angle)
    
    def save_background(self, filename: str, background_map: Dict[float, float]):
        """Сохраняет фоновую карту в файл"""
        save_background_to_file(filename, background_map)
    
    def load_background(self, filename: str) -> Dict[float, float]:
        """Загружает фоновую карту из файла"""
        background_map, interp_func = load_background_from_file(filename)
        self._background_interp = interp_func
        return background_map
