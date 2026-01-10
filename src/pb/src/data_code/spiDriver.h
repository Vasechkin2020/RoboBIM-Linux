#ifndef SPI_DRIVER_H // Защита от повторного включения этого файла
#define SPI_DRIVER_H // Определение макроса защиты

// --- Подключение системных библиотек ---
#include <stdint.h>           // Типы данных фиксированной ширины (uint8_t, uint32_t)
#include <unistd.h>           // Функции POSIX (open, close, read, write, usleep)
#include <stdio.h>            // Стандартный ввод-вывод (printf, perror)
#include <stdlib.h>           // Стандартная библиотека (exit, malloc)
#include <string.h>           // Работа со строками и памятью (memset, strerror)
#include <fcntl.h>            // Флаги управления файлами (O_RDWR)
#include <errno.h>            // Глобальная переменная кодов ошибок
#include <sys/ioctl.h>        // Системный вызов управления устройствами ввода-вывода
#include <linux/types.h>      // Системные типы данных Linux
#include <linux/spi/spidev.h> // Определения для драйвера SPI ядра Linux

// --- Подключение C++ библиотек ---
#include <chrono> // Современная библиотека времени (std::chrono)
#include <map>    // Контейнер "словарь" для хранения GPIO линий

// --- Подключение библиотеки GPIO ---
#include <gpiod.h> // libgpiod v1: стандарт работы с GPIO в современном Linux

// --- Макросы для логирования (совместимость с ROS) ---
#ifdef ROS_BUILD                                                                        // Если компилируем в среде ROS
#include <ros/console.h>                                                                // Подключаем консоль ROS
#define LOG_ERROR(fmt, ...) ROS_ERROR(fmt, ##__VA_ARGS__)                               // Макрос ошибки ROS
#define LOG_INFO(fmt, ...) ROS_INFO(fmt, ##__VA_ARGS__)                                 // Макрос инфо ROS
#else                                                                                   // Если обычная компиляция (g++)
#define LOG_ERROR(fmt, ...) fprintf(stderr, "[SPI_DRV ERROR] " fmt "\n", ##__VA_ARGS__) // Вывод ошибки в stderr
#define LOG_INFO(fmt, ...) printf("[SPI_DRV INFO] " fmt "\n", ##__VA_ARGS__)            // Вывод инфо в stdout
#endif

// Класс-обертка для работы с SPI и GPIO на Raspberry Pi
class SpiDriver
{
private:
    // --- Ресурсы SPI ---
    int fd_spi;        // Файловый дескриптор устройства SPI (например, /dev/spidev0.0)
    uint32_t speed_hz; // Текущая скорость шины в Герцах
    uint8_t mode;      // Режим работы SPI (обычно Mode 0)
    uint8_t bits;      // Количество бит в слове (обычно 8)

    // --- Ресурсы GPIO (libgpiod) ---
    struct gpiod_chip *chip;                  // Указатель на структуру управляющего чипа GPIO
    std::map<int, struct gpiod_line *> lines; // Карта активных пинов (Ключ: номер пина, Значение: указатель на линию)

    // --- Время ---
    std::chrono::steady_clock::time_point start_time_; // Точка времени запуска программы для millis()

    // Вспомогательная функция для закрытия файла SPI
    void closeSpiFd()
    {
        if (fd_spi >= 0)
        {                  // Если файл был открыт (дескриптор валиден)
            close(fd_spi); // Закрываем файловый дескриптор
            fd_spi = -1;   // Помечаем как закрытый
        }
    }

public:
    // Конструктор класса
    SpiDriver() : fd_spi(-1), speed_hz(1000000), mode(0), bits(8), chip(nullptr)
    {                                                   // Инициализация переменных по умолчанию
        start_time_ = std::chrono::steady_clock::now(); // Запоминаем время старта (монотонные часы)
    }

    // Деструктор (RAII - Resource Acquisition Is Initialization)  Гарантирует освобождение ресурсов при удалении объекта
    ~SpiDriver()
    {
        closeSpiFd(); // Закрываем SPI

        // ИСПРАВЛЕНО: Заменили C++17 binding на классический доступ через .second
        // item.first  = номер пина
        // item.second = указатель на линию (struct gpiod_line*)
        for (auto const &item : lines)
        {
            if (item.second)
                gpiod_line_release(item.second); // Возвращаем линию ядру
        }
        lines.clear(); // Очищаем контейнер

        // Закрываем соединение с чипом GPIO
        if (chip != nullptr)
        {
            gpiod_chip_close(chip); // Освобождаем ресурсы самого контроллера
            chip = nullptr;         // Зануляем указатель
        }
    }

    // ЗАПРЕТ КОПИРОВАНИЯ (Rule of Three)
    // Запрещаем конструктор копирования
    SpiDriver(const SpiDriver &) = delete;
    // Запрещаем оператор присваивания
    SpiDriver &operator=(const SpiDriver &) = delete;

    // ======================== SPI ФУНКЦИИ ========================

    // Инициализация SPI устройства
    bool beginSPI(const char *device, uint32_t speed)
    {
        closeSpiFd();     // Закрываем старый файл, если он был открыт
        speed_hz = speed; // Сохраняем желаемую скорость

        fd_spi = open(device, O_RDWR); // Открываем устройство SPI для чтения и записи
        if (fd_spi < 0)
        {                                                                       // Проверка на ошибку открытия
            LOG_ERROR("Can't open SPI device %s: %s", device, strerror(errno)); // Логируем системную ошибку
            return false;                                                       // Возвращаем неудачу
        }

        mode = SPI_MODE_0; // Устанавливаем режим 0 (CPOL=0, CPHA=0)
        // Отправляем команду ядру на установку режима
        if (ioctl(fd_spi, SPI_IOC_WR_MODE, &mode) == -1)
        {
            LOG_ERROR("Can't set SPI mode: %s", strerror(errno)); // Лог ошибки
            closeSpiFd();
            return false; // Закрываем файл и выходим
        }

        bits = 8; // Устанавливаем 8 бит на слово
        // Отправляем команду ядру на установку битности
        if (ioctl(fd_spi, SPI_IOC_WR_BITS_PER_WORD, &bits) == -1)
        {
            LOG_ERROR("Can't set bits: %s", strerror(errno)); // Лог ошибки
            closeSpiFd();
            return false; // Закрываем файл и выходим
        }

        // Отправляем команду ядру на установку скорости
        if (ioctl(fd_spi, SPI_IOC_WR_MAX_SPEED_HZ, &speed_hz) == -1)
        {
            LOG_ERROR("Can't set speed: %s", strerror(errno)); // Лог ошибки
            closeSpiFd();
            return false; // Закрываем файл и выходим
        }

        return true; // Инициализация прошла успешно
    }

    // Передача данных (Full Duplex - одновременный прием и передача)
    bool transferRW(uint8_t *buffer, size_t len)
    {
        if (fd_spi < 0)
        {                                                   // Проверка: инициализирован ли SPI?
            LOG_ERROR("SPI transfer attempt without init"); // Лог ошибки
            return false;                                   // Выход
        }
        if (!buffer || len == 0)
        {                                                      // Проверка аргументов
            LOG_ERROR("SPI transfer with invalid buffer/len"); // Лог ошибки
            return false;                                      // Выход
        }

        struct spi_ioc_transfer tr; // Структура для передачи параметров транзакции ядру
        memset(&tr, 0, sizeof(tr)); // Очищаем структуру нулями (обязательно!)

        tr.tx_buf = (unsigned long)buffer; // Указатель на буфер передачи
        tr.rx_buf = (unsigned long)buffer; // Указатель на буфер приема (перезаписываем тот же буфер)
        tr.len = len;                      // Длина данных
        tr.speed_hz = speed_hz;            // Скорость передачи
        tr.bits_per_word = bits;           // Битность

        // Выполняем транзакцию через системный вызов ioctl
        int ret = ioctl(fd_spi, SPI_IOC_MESSAGE(1), &tr);
        if (ret < 1)
        {                                                          // Если результат меньше 1, значит ошибка
            LOG_ERROR("SPI Transfer failed: %s", strerror(errno)); // Лог системной ошибки
            return false;                                          // Выход
        }
        return true; // Успешная передача
    }

    // ======================== GPIO ФУНКЦИИ ========================

    // Инициализация чипа GPIO
    bool beginGPIO(const char *chip_path = "/dev/gpiochip0")
    { // По умолчанию chip0 (RPi 4)
        if (chip)
            gpiod_chip_close(chip); // Если был открыт другой чип, закрываем его

        chip = gpiod_chip_open(chip_path); // Открываем устройство GPIO
        if (!chip)
        {                                                                         // Проверка успеха
            LOG_ERROR("Can't open GPIO chip %s: %s", chip_path, strerror(errno)); // Лог ошибки
            return false;                                                         // Выход
        }
        return true; // Успех
    }

    // Настройка режима пина (Вход/Выход)
    bool gpioMode(int pin, int direction)
    { // direction: 1=OUT, 0=IN
        if (!chip)
        {                                                                    // Проверяем, открыт ли чип
            LOG_ERROR("GPIO chip not initialized. Call beginGPIO() first."); // Лог ошибки
            return false;                                                    // Выход
        }

        // Если пин уже есть в нашем списке, освобождаем его перед перенастройкой
        if (lines.count(pin))
        {
            gpiod_line_release(lines[pin]); // Возвращаем линию ядру
            lines.erase(pin);               // Удаляем из карты
        }

        // Запрашиваем доступ к линии у ядра по номеру пина
        struct gpiod_line *line = gpiod_chip_get_line(chip, pin);
        if (!line)
        {                                               // Если пин не найден
            LOG_ERROR("Pin %d not found on chip", pin); // Лог ошибки
            return false;                               // Выход
        }

        int ret; // Переменная для кода возврата
        if (direction == 1)
        { // Если нужен режим ВЫХОД
            // Запрашиваем выход, имя потребителя "robot_spi_drv", начальное значение 0
            ret = gpiod_line_request_output(line, "robot_spi_drv", 0);
        }
        else
        { // Если нужен режим ВХОД
            // Запрашиваем вход
            ret = gpiod_line_request_input(line, "robot_spi_drv");
        }

        if (ret < 0)
        {                                                                          // Если запрос не удался (например, пин занят другим процессом)
            LOG_ERROR("Request line failed for pin %d: %s", pin, strerror(errno)); // Лог ошибки
            return false;                                                          // Выход
        }

        lines[pin] = line; // Сохраняем успешную линию в карту
        return true;       // Успех
    }

    // Запись значения в пин (аналог digitalWrite), возвращает успех/неудачу
    bool gpioWrite(int pin, int value)
    {
        auto it = lines.find(pin); // Ищем пин в нашей карте
        if (it == lines.end())
        {                                                                         // Если пин не найден (не был настроен)
            LOG_ERROR("CRITICAL: Attempt to write to uninitialized pin %d", pin); // Лог критической ошибки
            return false;                                                         // Возвращаем неудачу
        }

        // Устанавливаем значение линии
        int ret = gpiod_line_set_value(it->second, value);
        if (ret < 0)
        {                                                                    // Если запись не удалась на уровне ядра
            LOG_ERROR("GPIO write failed pin %d: %s", pin, strerror(errno)); // Лог ошибки
            return false;                                                    // Неудача
        }
        return true; // Успех
    }

    // Чтение значения пина (аналог digitalRead)
    int gpioRead(int pin)
    {
        auto it = lines.find(pin); // Ищем пин
        if (it == lines.end())
        {                                                           // Если не найден
            LOG_ERROR("Attempt to read uninitialized pin %d", pin); // Лог ошибки
            return -1;                                              // Возвращаем -1 как признак ошибки
        }
        return gpiod_line_get_value(it->second); // Возвращаем 0 или 1
    }

    // ======================== УДОБНЫЕ ФУНКЦИИ (CS С ЗАЩИТОЙ) ========================

    // Передача с ручным управлением Chip Select и проверками
    bool transferWithCS(int cs_pin, uint8_t *buffer, size_t len, uint32_t delay_us = 10)
    {
        // 1. Проверяем, инициализирован ли SPI
        if (fd_spi < 0)
        {
            LOG_ERROR("transferWithCS: SPI not initialized"); // Лог
            return false;                                     // Выход
        }

        // 2. Проверяем, настроен ли пин CS
        if (lines.find(cs_pin) == lines.end())
        {
            LOG_ERROR("transferWithCS: CS pin %d not initialized! Call gpioMode first.", cs_pin); // Лог
            return false;                                                                         // Выход
        }

        // 3. Опускаем CS в 0 (активный уровень)
        if (!gpioWrite(cs_pin, 0))
        {
            LOG_ERROR("transferWithCS: Failed to pull CS Low. Aborting transfer."); // Лог
            return false;                                                           // Не начинаем передачу, если CS не сработал
        }

        // Пауза, чтобы Slave успел подготовиться
        if (delay_us > 0)
            delayMicroseconds(delay_us);

        // 4. Передаем данные
        bool transferSuccess = transferRW(buffer, len);
        if (!transferSuccess)
        {                                                      // Если передача не удалась
            LOG_ERROR("transferWithCS: Data transfer failed"); // Лог
            // Мы все равно пойдем дальше, чтобы поднять CS
        }

        // Пауза перед завершением
        if (delay_us > 0)
            delayMicroseconds(delay_us);

        // 5. Поднимаем CS в 1 (неактивный уровень) ВСЕГДА
        if (!gpioWrite(cs_pin, 1))
        {
            LOG_ERROR("transferWithCS: CRITICAL! Failed to pull CS High. Device may be stuck."); // Критический лог
        }

        return transferSuccess; // Возвращаем результат самой передачи данных
    }

    // ======================== ВРЕМЯ (Modern C++) ========================

    // Задержка в микросекундах
    void delayMicroseconds(uint32_t us)
    {
        usleep(us); // Системный вызов паузы
    }

    // Задержка в миллисекундах
    void delay(uint32_t ms)
    {
        usleep(ms * 1000); // usleep работает с мкс, умножаем на 1000
    }

    // Получить время в мс с момента старта программы
    uint32_t millis()
    {
        auto now = std::chrono::steady_clock::now(); // Текущее время
        // Вычисляем разницу со стартом и приводим к миллисекундам
        return (uint32_t)std::chrono::duration_cast<std::chrono::milliseconds>(now - start_time_).count();
    }

    // Получить время в мкс с момента старта программы
    uint64_t micros()
    {
        auto now = std::chrono::steady_clock::now(); // Текущее время
        // Вычисляем разницу со стартом и приводим к микросекундам
        return (uint64_t)std::chrono::duration_cast<std::chrono::microseconds>(now - start_time_).count();
    }
};

#endif // Конец файла SPI_DRIVER_H