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

# Параметры управления мышью
MOUSE_CONFIG = {
    'stability_threshold': 5.0,      # Порог стабильности в пикселях
    'min_stable_frames': 3,          # Минимум кадров для стабильности
    'click_delay': 0.02,              # Задержка после клика в секундах
    'position_smoothing': 0.3,       # Фактор сглаживания позиции (0.0-1.0)
    'max_touch_points': 10,          # Максимум обрабатываемых точек касания
    'touch_tracking_timeout': 2.0,   # Таймаут отслеживания касания в секундах
}

class Config:
    """Центральный класс конфигурации"""
    
    def __init__(self):
        self.lidar = LIDAR_CONFIG.copy()
        self.calibration = CALIBRATION_CONFIG.copy()
        self.processing = PROCESSING_CONFIG.copy()
        self.display = DISPLAY_CONFIG.copy()
        self.performance = PERFORMANCE_CONFIG.copy()
        self.mouse = MOUSE_CONFIG.copy()
    
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
        return self.performance.copy()
    
    def get_mouse_params(self):
        """Получение параметров управления мышью"""
        return self.mouse.copy()
    
    def update_mouse_config(self, **kwargs):
        """Обновление параметров управления мышью"""
        for key, value in kwargs.items():
            if key in self.mouse:
                self.mouse[key] = value

# Глобальная конфигурация
config = Config()
