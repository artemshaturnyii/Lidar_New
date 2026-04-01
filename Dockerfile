FROM python:3.9

# Установка системных зависимостей
RUN apt-get update && apt-get install -y \
    libgtk-3-0 \
    libdbus-1-3 \
    libxtst6 \
    libxrender1 \
    libxrandr2 \
    libxss1 \
    libglib2.0-0 \
    libxext6 \
    libxcb1 \
    libx11-6 \
    libxfixes3 \
    libxi6 \
    libxinerama1 \
    x11-utils \
    x11-xserver-utils \
    && rm -rf /var/lib/apt/lists/*

# Создание пользователя для запуска GUI приложений
RUN useradd -m -u 1000 user
USER user
WORKDIR /home/user/app

# Копирование requirements и установка Python зависимостей
COPY --chown=user:user requirements.txt .
RUN pip install --no-cache-dir -r requirements.txt

# Копирование исходного кода
COPY --chown=user:user . .

# Команда для запуска приложения
CMD ["python", "app.py"]
