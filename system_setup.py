"""
Модуль автоматической настройки системных зависимостей.

Проверяет и устанавливает:
- Python пакеты (через pip)
- Системные утилиты (через apt, требует sudo)
- Права доступа к устройствам (/dev/uinput)
"""

import subprocess
import sys
import os
import platform
from pathlib import Path
from typing import List, Tuple


class SystemSetup:
    """Автоматическая настройка системных зависимостей."""

    def __init__(self):
        self.is_root = os.geteuid() == 0 if platform.system() != "Windows" else True
        self.system = platform.system()
        self.setup_marker = Path(".setup_complete")

    # ─── Проверка Python пакетов ───────────────────────────────────────

    def check_python_packages(self, packages: List[str]) -> Tuple[List[str], List[str]]:
        """
        Проверяет наличие Python пакетов.
        
        Returns:
            (installed, missing) — списки пакетов
        """
        import importlib
        installed = []
        missing = []

        for pkg in packages:
            try:
                importlib.import_module(pkg)
                installed.append(pkg)
            except ImportError:
                missing.append(pkg)

        return installed, missing

    def install_python_packages(self, packages: List[str]) -> bool:
        """Устанавливает Python пакеты через pip."""
        if not packages:
            return True

        print(f"📦 Установка Python пакетов: {', '.join(packages)}")
        try:
            subprocess.check_call([
                sys.executable, "-m", "pip", "install", "--quiet", *packages
            ])
            print("✅ Python пакеты установлены")
            return True
        except subprocess.CalledProcessError as e:
            print(f"❌ Ошибка установки Python пакетов: {e}")
            return False

    # ─── Проверка системных пакетов ────────────────────────────────────

    def check_system_command(self, command: str) -> bool:
        """Проверяет наличие команды в PATH."""
        return subprocess.run(
            ['which', command],
            capture_output=True,
            text=True
        ).returncode == 0

    def install_system_package(self, package: str) -> bool:
        """Устанавливает системный пакет через apt."""
        if self.check_system_command(package):
            return True

        print(f"🔧 Требуется системный пакет: {package}")

        if self.system == "Windows":
            print("⚠️  Пропущено (Windows)")
            return True

        if not self.is_root:
            print("⚠️  Требуются права root для установки системных пакетов")
            print("   Запустите приложение с: sudo python3 app.py")
            return False

        try:
            print(f"📥 Обновление списков пакетов...")
            subprocess.run(['apt', 'update'], check=True, capture_output=True)

            print(f"📥 Установка {package}...")
            subprocess.run(['apt', 'install', '-y', package], check=True)

            print(f"✅ {package} установлен")
            return True

        except subprocess.CalledProcessError as e:
            print(f"❌ Ошибка установки {package}: {e}")
            return False

    # ─── Настройка прав доступа ────────────────────────────────────────

    def check_uinput_access(self) -> bool:
        """Проверяет доступ к /dev/uinput."""
        if self.system == "Windows":
            return True

        dev_path = '/dev/uinput'
        if not os.path.exists(dev_path):
            print(f"⚠️  Устройство {dev_path} не найдено")
            print("   Попробуйте: sudo modprobe uinput")
            return False

        return os.access(dev_path, os.W_OK)

    def setup_uinput_access(self) -> bool:
        """Настраивает постоянный доступ к /dev/uinput."""
        if self.system == "Windows":
            return True

        dev_path = '/dev/uinput'

        if not os.path.exists(dev_path):
            print("⚠️  /dev/uinput не найден. Загрузите модуль ядра:")
            print("   sudo modprobe uinput")
            return False

        if os.access(dev_path, os.W_OK):
            return True

        print("🔐 Настройка прав доступа к /dev/uinput...")

        if not self.is_root:
            print("⚠️  Требуются права root для настройки прав")
            print("   Запустите: sudo python3 app.py")
            return False

        try:
            # Временные права (до перезагрузки)
            os.chmod(dev_path, 0o666)
            print("✅ Временные права установлены")

            # Постоянные права (udev правило)
            rule_content = 'KERNEL=="uinput", MODE="0666", GROUP="plugdev"\n'
            rule_path = '/etc/udev/rules.d/99-uinput.rules'

            with open(rule_path, 'w') as f:
                f.write(rule_content)
            print(f"✅ udev правило создано: {rule_path}")

            # Применить правила
            subprocess.run(['udevadm', 'control', '--reload-rules'], check=True)
            subprocess.run(['udevadm', 'trigger'], check=True)
            print("✅ udev правила применены")

            return True

        except Exception as e:
            print(f"❌ Ошибка настройки uinput: {e}")
            return False

    # ─── Проверка демона ydotoold ──────────────────────────────────────

    def ensure_ydotoold_running(self) -> bool:
        """Проверяет и запускает демон ydotoold если нужен."""
        if self.system == "Windows":
            return True

        if not self.check_system_command('ydotool'):
            return True  # ydotool не установлен, не проверяем демон

        try:
            # Проверка запущенного процесса
            result = subprocess.run(
                ['pgrep', '-f', 'ydotoold'],
                capture_output=True,
                text=True
            )
            if result.returncode == 0:
                return True

            # Попытка запустить демон
            print("🔄 Запуск демона ydotoold...")
            subprocess.Popen(
                ['sudo', 'ydotoold'],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )
            print("✅ ydotoold запущен")
            return True

        except Exception as e:
            print(f"⚠️  Не удалось запустить ydotoold: {e}")
            return False

    # ─── Основной метод настройки ──────────────────────────────────────

    def run_full_setup(self, auto_install: bool = True) -> bool:
        """
        Запускает полную проверку и установку зависимостей.

        Args:
            auto_install: Если True, автоматически устанавливать пакеты

        Returns:
            True если все критические зависимости доступны
        """
        print("\n" + "=" * 50)
        print("🚀 LiDAR Touch Controller — Проверка системы")
        print("=" * 50)

        # Пропускаем если уже настроено
        if self.setup_marker.exists() and auto_install:
            print("✅ Предыдущая настройка обнаружена (пропускаем)")
            print("   Удалите файл .setup_complete для полной проверки")
            return True

        all_ok = True

        # ─── Python пакеты ─────────────────────────────────────────────
        python_packages = ['numpy', 'matplotlib', 'PIL']
        installed, missing = self.check_python_packages(python_packages)

        if missing:
            print(f"⚠️  Отсутствуют Python пакеты: {', '.join(missing)}")
            if auto_install:
                if not self.install_python_packages(missing):
                    all_ok = False
            else:
                all_ok = False
        else:
            print("✅ Python пакеты: все установлены")

        # ─── Системные утилиты (Linux) ─────────────────────────────────
        if self.system == "Linux":
            system_tools = ['ydotool', 'xrandr']
            for tool in system_tools:
                if not self.check_system_command(tool):
                    print(f"⚠️  Отсутствует утилита: {tool}")
                    if auto_install:
                        if not self.install_system_package(tool):
                            all_ok = False
                    else:
                        all_ok = False
                else:
                    print(f"✅ Утилита {tool}: найдена")

            # ─── Права доступа ─────────────────────────────────────────
            if not self.check_uinput_access():
                print("⚠️  Нет доступа к /dev/uinput")
                if auto_install:
                    if not self.setup_uinput_access():
                        all_ok = False
                else:
                    all_ok = False
            else:
                print("✅ Доступ к /dev/uinput: есть")

            # ─── Демон ydotoold ────────────────────────────────────────
            self.ensure_ydotoold_running()

        # ─── Windows проверки ──────────────────────────────────────────
        elif self.system == "Windows":
            print("✅ Платформа: Windows (специфичные проверки не требуются)")

        # ─── Итог ──────────────────────────────────────────────────────
        print("=" * 50)
        if all_ok:
            print("✅ Все проверки пройдены")
            self.setup_marker.touch()  # Создать маркер
        else:
            print("⚠️  Некоторые проверки не пройдены")
            print("   Приложение может работать с ограничениями")
        print("=" * 50 + "\n")

        return all_ok

    def reset_setup(self):
        """Сбрасывает маркер настройки (для принудительной повторной проверки)."""
        if self.setup_marker.exists():
            self.setup_marker.unlink()
            print("✅ Маркер настройки сброшен")


# ─── Утилита для прямого запуска ──────────────────────────────────────

if __name__ == "__main__":
    setup = SystemSetup()
    setup.run_full_setup(auto_install=True)
    print("\n💡 Теперь можете запустить приложение: python3 app.py")
