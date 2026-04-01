"""Конфигурационный файл для приложения"""

# Параметры подключения к лидару
LIDAR_CONFIG = {
    'host': '192.168.0.7',
    'port': 25168,
    'connection_timeout': 5.0,  # секунды
}

# Параметры калибровки
CALIBRATION_CONFIG = {
    'default_scan_count': 1,
    'min_scans': 1,
    'max_scans': 100,
}

# Параметры обработки данных
PROCESSING_CONFIG = {
    'background_grid_step': 1.0,      # градусы
    'angle_tolerance': 0.5,            # градусы для поиска точек
    'use_median_filter': True,         # использовать медиану вместо среднего
}

# Параметры визуализации
DISPLAY_CONFIG = {
    'enable_plot': True,
    'plot_size': (8, 8),
    'plot_title': "Фоновая карта (полярное представление)",
}

# Параметры производительности
PERFORMANCE_CONFIG = {
    'target_fps': 60,
    'max_touch_points': 5,
    'scan_point_limit': 50,
    'update_interval': 0.016,  # 1/60 секунды
    'fast_mode': True,
}

# Параметры точности для детекции касаний
PRECISION_CONFIG = {
    'sync_tolerance': 3.0,          # Точность синхронизации скана (градусы)
    'smoothing_window': 5,          # Размер окна сглаживания
    'min_touch_intensity': 30,      # Минимальная интенсивность касания
    'max_detection_distance': 2500, # Максимальное расстояние касания (мм)
    'mouse_update_rate': 0.016,     # Частота обновления мыши (секунды)
    'touch_stability_frames': 3,    # Количество кадров для стабильности касания
    'angle_window': 5.0,            # Угловое окно для группировки точек
    'min_touch_points': 2,          # Минимальное количество точек для подтверждения касания
    'adaptive_threshold_multiplier': 0.05,  # Множитель для адаптивного порога
    'min_distance_deviation': 50.0, # Минимальное отклонение расстояния (мм)
    'max_background_distance': 3000.0,  # Максимальное расстояние фона (мм)
}

class Config:
    """Центральный класс конфигурации"""
    
    def __init__(self):
        self.lidar = LIDAR_CONFIG.copy()
        self.calibration = CALIBRATION_CONFIG.copy()
        self.processing = PROCESSING_CONFIG.copy()
        self.display = DISPLAY_CONFIG.copy()
        self.precision = PRECISION_CONFIG.copy()
    
    def update_lidar_config(self, host=None, port=None, timeout=None):
        """Обновление параметров подключения"""
        if host is not None:
            self.lidar['host'] = host
        if port is not None:
            self.lidar['port'] = port
        if timeout is not None:
            self.lidar['connection_timeout'] = timeout
    
    def get_background_processing_params(self):
        """Получение параметров для обработки фона"""
        return {
            'step_deg': self.processing['background_grid_step'],
            'angle_tolerance': self.processing['angle_tolerance'],
            'use_median': self.processing['use_median_filter']
        }

    def get_performance_params(self):
        """Получение параметров производительности"""
        return PERFORMANCE_CONFIG.copy()
    
    def get_precision_params(self):
        """Получение параметров точности"""
        return self.precision.copy()
    
    def update_precision_config(self, **kwargs):
        """Обновление параметров точности"""
        for key, value in kwargs.items():
            if key in self.precision:
                self.precision[key] = value

# Глобальная конфигурация
config = Config()
