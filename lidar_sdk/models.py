from dataclasses import dataclass
from typing import List

@dataclass
class Point:
    """Точка сканирования"""
    angle: float      # градусы
    distance: float   # миллиметры
    intensity: float  # 0..255

@dataclass
class Node:
    """Узел данных в пакете"""
    distance: int
    confidence: int

@dataclass
class DataPackM1:
    """Пакет данных M1"""
    header: int
    node_num: int
    speed: int
    start_angle: int
    nodes: List[Node]
    end_angle: int
    timestamp: int
    crc8: int
