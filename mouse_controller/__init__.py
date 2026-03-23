"""Основной модуль приложения LiDAR Touch Controller"""

__version__ = "1.0.0"
__author__ = "Your Name"

# Экспортируем основные компоненты для удобства импорта
from .mouse_controller import MouseController, initialize_mouse_control, get_mouse_controller

__all__ = ['MouseController', 'initialize_mouse_control', 'get_mouse_controller']
