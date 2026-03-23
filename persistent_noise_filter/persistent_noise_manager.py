# persistent_noise_filter/persistent_noise_manager.py

import json
import numpy as np
from typing import List, Dict, Tuple
from lidar_sdk import Point
from pathlib import Path

class PersistentNoiseManager:
    """Менеджер для хранения и управления постоянным шумом"""
    
    def __init__(self, storage_file: str = "persistent_noise_points.json"):
        self.storage_file = storage_file
        self.persistent_noise_points = {}  # {(angle_bin, distance_bin): count}
        self.angle_tolerance = 2.0  # градусов
        self.distance_tolerance = 50.0  # мм
        self.activation_threshold = 5  # сколько раз точка должна появиться, чтобы стать "постоянной"
        self.load_persistent_noise()
    
    def load_persistent_noise(self):
        """Загружает постоянный шум из файла"""
        try:
            if Path(self.storage_file).exists():
                with open(self.storage_file, 'r') as f:
                    data = json.load(f)
                    # Конвертируем ключи обратно в tuple
                    self.persistent_noise_points = {
                        tuple(map(float, k.strip('()').split(', '))): v 
                        for k, v in data.items()
                    }
                print(f"📂 Загружено {len(self.persistent_noise_points)} постоянных шумовых точек")
            else:
                print("📝 Файл постоянного шума не найден, будет создан новый")
        except Exception as e:
            print(f"⚠️  Ошибка загрузки постоянного шума: {e}")
            self.persistent_noise_points = {}
    
    def save_persistent_noise(self):
        """Сохраняет постоянный шум в файл"""
        try:
            # Конвертируем tuple ключи в строки для JSON
            serializable_data = {
                str(k): v for k, v in self.persistent_noise_points.items()
            }
            with open(self.storage_file, 'w') as f:
                json.dump(serializable_data, f, indent=2)
        except Exception as e:
            print(f"⚠️  Ошибка сохранения постоянного шума: {e}")
    
    def register_noise_point(self, point: Point):
        """Регистрирует точку шума для отслеживания"""
        # Определяем "бин" для этой точки (группируем близкие точки)
        angle_bin = round(point.angle / self.angle_tolerance) * self.angle_tolerance
        distance_bin = round(point.distance / self.distance_tolerance) * self.distance_tolerance
        key = (angle_bin, distance_bin)
        
        # Увеличиваем счетчик для этого бина
        current_count = self.persistent_noise_points.get(key, 0)
        self.persistent_noise_points[key] = current_count + 1
        
        # Если точка стала "постоянной", сохраняем
        if current_count + 1 == self.activation_threshold:
            print(f"📌 Постоянный шум обнаружен: угол {angle_bin}°, расстояние {distance_bin}мм")
            self.save_persistent_noise()
    
    def is_persistent_noise(self, point: Point) -> bool:
        """Проверяет, является ли точка постоянным шумом"""
        angle_bin = round(point.angle / self.angle_tolerance) * self.angle_tolerance
        distance_bin = round(point.distance / self.distance_tolerance) * self.distance_tolerance
        key = (angle_bin, distance_bin)
        
        # Если точка появлялась достаточно часто - считаем её постоянным шумом
        return self.persistent_noise_points.get(key, 0) >= self.activation_threshold
    
    def get_persistent_noise_stats(self) -> Dict:
        """Возвращает статистику по постоянному шуму"""
        active_points = {
            k: v for k, v in self.persistent_noise_points.items() 
            if v >= self.activation_threshold
        }
        
        return {
            'total_tracked_points': len(self.persistent_noise_points),
            'active_persistent_points': len(active_points),
            'most_problematic_areas': sorted(
                active_points.items(), 
                key=lambda x: x[1], 
                reverse=True
            )[:10]  # Топ-10 самых "шумных" мест
        }
    
    def clear_persistent_noise(self):
        """Очищает все данные о постоянном шуме"""
        self.persistent_noise_points = {}
        self.save_persistent_noise()
        print("🗑️  Данные о постоянном шуме очищены")

