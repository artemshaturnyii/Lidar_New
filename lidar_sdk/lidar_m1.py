import threading
import time
import struct
import math
from typing import List, Optional, Callable
from .connection import LidarConnection
from .protocol import PACKET_FMT, PACKET_SIZE, M1_HEADER, NODES_PER_PACK, crc8
from .models import Point

class LidarM1:
    """Основной класс для работы с лидаром M1"""
    
    def __init__(self):
        """Инициализация драйвера лидара"""
        self.connection = LidarConnection()
        self._running = False
        self._thread: Optional[threading.Thread] = None
        self._current_scan: List[Point] = []
        self._last_scan: List[Point] = []
        self._scan_counter: int = 0
        self._last_angle: Optional[float] = None
        
        self.lock = threading.Lock()
        self.on_scan_complete: Optional[Callable[[List[Point]], None]] = None
        self.on_point_received: Optional[Callable[[Point], None]] = None

    def connect(self) -> bool:
        """Подключается к лидару"""
        return self.connection.connect()

    def start(self) -> None:
        """Запускает фоновый поток чтения"""
        if self._running:
            return
        self._running = True
        self._thread = threading.Thread(target=self._reader, daemon=True)
        self._thread.start()

    def stop(self) -> None:
        """Останавливает поток и закрывает соединение"""
        self._running = False
        if self._thread:
            self._thread.join(timeout=2)
        self.connection.disconnect()

    def _reader(self) -> None:
        """Фоновый поток: читает пакеты и обрабатывает их"""
        print("🔄 Начинаю чтение данных...")
        buffer = bytearray()
    
        while self._running:
            try:
                chunk = self.connection.recv_exact(256)
                if not chunk:
                    print("❌ Получены пустые данные")
                    break
                
                buffer.extend(chunk)
            
                i = 0
                while i < len(buffer) - 2:
                    if buffer[i] == 0x54 and buffer[i+1] == 0x2C:
                        j = i + 2
                        found_next = False
                        while j < len(buffer) - 1:
                            if buffer[j] == 0x54 and buffer[j+1] == 0x2C:
                                packet_data = buffer[i:j]
                                if len(packet_data) >= 10:
                                    self._process_packet(bytes(packet_data))
                                i = j - 1
                                found_next = True
                                break
                            j += 1
                    
                        if not found_next:
                            buffer = buffer[i:]
                            break
                    i += 1
                
                if len(buffer) > 2048:
                    buffer = buffer[-1024:]
                
            except Exception as e:
                if self._running:
                    print(f"❌ Reader error: {e}")
                    import traceback
                    traceback.print_exc()
                break
    
        print("⏹️  Чтение остановлено.")

    def _parse_packet(self, data):
        """Распаковывает и проверяет пакет. Возвращает None при ошибке."""
        if len(data) < 10:
            return None
        
        header = data[0]
        if header != M1_HEADER:
            return None
        
        for node_count in [0x0C, 0x2C]:
            try:
                fmt = '<BBHH' + 'HB' * node_count + 'HHB'
                expected_size = struct.calcsize(fmt)
            
                if len(data) >= expected_size:
                    packet_to_check = data[:expected_size]
                
                    calculated_crc = crc8(packet_to_check[:-1])
                    packet_crc = packet_to_check[-1]
                
                    if calculated_crc == packet_crc:
                        fields = struct.unpack(fmt, packet_to_check)
                        speed = fields[2]
                        start_angle = fields[3] * 0.01
                        end_angle = fields[4 + 2*node_count] * 0.01
                    
                        nodes = []
                        for i in range(node_count):
                            dist = fields[4 + i*2]
                            conf = fields[4 + i*2 + 1]
                            nodes.append((dist, conf))
                        
                        return start_angle, end_angle, nodes
                    
            except struct.error:
                pass

        return None

    def _process_packet(self, data):
        """Обрабатывает один пакет, разбирает точки и определяет завершение оборота."""
        parsed = self._parse_packet(data)
        if parsed is None:
            return
        
        start_angle, end_angle, nodes = parsed

        if end_angle < start_angle:
            step = (end_angle + 360 - start_angle) / (len(nodes) - 1)
        else:
            step = (end_angle - start_angle) / (len(nodes) - 1)

        if abs(step) > 5.0:
            return

        for i, (dist, conf) in enumerate(nodes):
            raw_angle = start_angle + step * i
            angle = raw_angle % 360
            pt = Point(angle=angle, distance=float(dist), intensity=float(conf))

            with self.lock:
                # ✅ ПРОСТОЕ ОБНАРУЖЕНИЕ ГРАНИЦЫ: переход 350°→10°
                if self._last_angle is not None and self._last_angle > 350 and angle < 10:
                    # Scan complete!
                    if len(self._current_scan) >= 10:
                        normalized_scan = self._normalize_scan_start(self._current_scan)
                        self._last_scan = normalized_scan
                        self._scan_counter += 1
                        if self.on_scan_complete:
                            self.on_scan_complete(self._last_scan)
                    
                    self._current_scan = [pt]
                else:
                    self._current_scan.append(pt)

                self._last_angle = angle

                if self.on_point_received:
                    self.on_point_received(pt)

    def _normalize_scan_start(self, scan: List[Point]) -> List[Point]:
        """Перемещает первую точку ближе всего к 0°."""
        if not scan:
            return scan
        zero_idx = min(range(len(scan)), key=lambda i: abs(scan[i].angle))
        return scan[zero_idx:] + scan[:zero_idx]

    def get_last_scan(self) -> List[Point]:
        """Возвращает последний завершённый скан (копию)."""
        with self.lock:
            return self._last_scan.copy()

    def get_scan_counter(self) -> int:
        """Возвращает номер последнего завершённого скана."""
        with self.lock:
            return self._scan_counter
