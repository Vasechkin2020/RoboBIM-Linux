#ifndef LOGI_H
#define LOGI_H

#pragma once  // Гарантирует, что заголовок будет включён только один раз

#include <cstdio>           // Для функций работы с файлами (fopen, fwrite и т.д.)
#include <cstdarg>          // Для работы с переменным числом аргументов (va_list, va_start и др.)
#include <cstring>          // Для strlen и других строковых функций (используется неявно)
#include <string>           // Для std::string
#include <queue>            // Для очереди сообщений
#include <thread>           // Для создания фонового потока
#include <mutex>            // Для мьютекса, защищающего очередь
#include <condition_variable> // Для уведомления потока о появлении новых сообщений
#include <chrono>           // Для работы с временем и миллисекундами
#include <atomic>           // Для атомарных флагов (stop_flag_, initialized_)
#include <stdexcept>        // Для std::runtime_error при ошибках
#include <sys/stat.h>       // Для проверки существования директории (stat)
#include <unistd.h>         // Для getpid() — получения ID процесса

// ---------- Цвета консоли ----------
#define COL_RESET "\033[0m"     // Сброс цвета текста в консоли
#define COL_RED "\033[31m"      // Красный цвет
#define COL_GREEN "\033[32m"    // Зелёный цвет
#define COL_YELLOW "\033[33m"   // Жёлтый цвет
#define COL_BLUE "\033[34m"     // Синий цвет

// ======================================================
//                 AsyncFileLogger
// ======================================================
class AsyncFileLogger
{
public:
    // Конструктор: принимает путь к папке и префикс имени файла
    AsyncFileLogger(const std::string &path_prefix,
                    const std::string &filename_prefix)
    {
        // Создаём директорию для логов, если она ещё не существует
        createDirectory(path_prefix);

        // Формируем полный путь к файлу: <путь>/<префикс>_ГГГГММДД_ЧЧММСС_PID.log
        filepath_ = path_prefix;
        // Добавляем слэш в конец пути, если его там нет
        if (!filepath_.empty() && filepath_.back() != '/')
            filepath_ += "/";

        // Добавляем префикс, текущую дату-время и PID процесса к имени файла
        filepath_ += filename_prefix + "_" + datetimeNow() + ".log";

        // Открываем файл в режиме добавления (append)
        file_ = fopen(filepath_.c_str(), "a");
        if (!file_)
        {
            // Если не удалось открыть файл — выводим ошибку и бросаем исключение
            perror("AsyncFileLogger fopen");
            throw std::runtime_error("Cannot open log file");
        }

        // Устанавливаем флаг остановки в false (поток должен работать)
        stop_flag_ = false;
        // Запускаем фоновый поток для записи логов в файл
        worker_ = std::thread(&AsyncFileLogger::threadLoop, this);

        // Помечаем логгер как успешно инициализированный
        initialized_ = true;
    }

    // Деструктор: корректно завершает работу логгера
    ~AsyncFileLogger()
    {
        // Завершаем только если логгер был успешно инициализирован
        if (initialized_)
        {
            // Устанавливаем флаг остановки
            stop_flag_ = true;
            // Пробуждаем поток записи, чтобы он проверил флаг остановки
            queue_cv_.notify_one();
            // Ждём завершения фонового потока
            if (worker_.joinable())
                worker_.join();
            // Закрываем файл
            if (file_)
                fclose(file_);
        }
    }

    // ---------------- timestamp ----------------
    // Возвращает строку с текущим временем в формате [ГГГГ-ММ-ДД ЧЧ:ММ:СС.ммм]
    inline std::string timestamp()
    {
        using namespace std::chrono;

        // Получаем текущее время с высокой точностью
        auto now = system_clock::now();
        // Преобразуем в стандартный time_t (секунды)
        auto t = system_clock::to_time_t(now);

        // Разбираем время на компоненты (год, месяц и т.д.)
        std::tm tm{};
        localtime_r(&t, &tm);  // Потокобезопасная версия localtime

        // Извлекаем миллисекунды из временной метки
        auto ms = duration_cast<milliseconds>(now.time_since_epoch()) % 1000;

        // Форматируем строку времени в буфер
        char buf[64];
        snprintf(buf, sizeof(buf),
                 "[%04d-%02d-%02d %02d:%02d:%02d.%03d] ",
                 tm.tm_year + 1900,  // Год (начиная с 1900)
                 tm.tm_mon + 1,      // Месяц (0–11 → 1–12)
                 tm.tm_mday,         // День месяца
                 tm.tm_hour,         // Часы
                 tm.tm_min,          // Минуты
                 tm.tm_sec,          // Секунды
                 (int)ms.count());   // Миллисекунды

        // Возвращаем строку времени
        return std::string(buf);
    }

    // ======================================================
    //               Методы логирования
    // ======================================================

    // --- Только в файл (без вывода в консоль) ---
    void logf(const char *fmt, ...)  // Принимает переменное число аргументов
    {
        // Если логгер не инициализирован — ничего не делаем
        if (!initialized_)
            return;

        // Начинаем обработку переменного числа аргументов
        va_list args;
        va_start(args, fmt);
        // Передаём аргументы во внутренний обработчик без вывода в консоль
        log_v(fmt, args, COL_RESET, false);
        // Завершаем работу с аргументами
        va_end(args);
    }

    // --- В файл и в консоль (обычный белый цвет) ---
    void log(const char *fmt, ...)  // Принимает переменное число аргументов
    {
        if (!initialized_)
            return;

        va_list args;
        va_start(args, fmt);
        log_v(fmt, args, COL_RESET, true);  // true = выводить в консоль
        va_end(args);
    }

    // --- Зелёный лог (успех, нормальная работа) ---
    void log_g(const char *fmt, ...)  // Принимает переменное число аргументов
    {
        va_list args;
        va_start(args, fmt);
        log_v(fmt, args, COL_GREEN, true);
        va_end(args);
    }

    // --- Красный лог (ошибка) ---
    void log_r(const char *fmt, ...)  // Принимает переменное число аргументов
    {
        va_list args;
        va_start(args, fmt);
        log_v(fmt, args, COL_RED, true);
        va_end(args);
    }

    // --- Жёлтый лог (предупреждение) ---
    void log_w(const char *fmt, ...)  // Принимает переменное число аргументов
    {
        va_list args;
        va_start(args, fmt);
        log_v(fmt, args, COL_YELLOW, true);
        va_end(args);
    }

    // --- Синий лог (информационное сообщение) ---
    void log_b(const char *fmt, ...)  // Принимает переменное число аргументов
    {
        va_list args;
        va_start(args, fmt);
        log_v(fmt, args, COL_BLUE, true);
        va_end(args);
    }

private:
    // ======================================================
    //                Внутренние функции
    // ======================================================

    // Общий метод для форматирования и логирования: принимает va_list, цвет и флаг вывода в консоль
    void log_v(const char* fmt, va_list args, const char* color, bool to_console)
    {
        if (!initialized_)
            return;

        // Буфер для форматированного сообщения
        char msg[2048];
        // Форматируем строку с использованием переданного списка аргументов
        vsnprintf(msg, sizeof(msg), fmt, args);

        // Добавляем временную метку к сообщению
        std::string out = timestamp() + msg;

        // Запись в файл (всегда)
        pushToFile(out);
        // Вывод в консоль, если разрешено
        if (to_console)
            printConsole(color, out);
    }

    // Вывод строки в консоль с заданным цветом
    void printConsole(const char *color, const std::string &txt)
    {
        // Устанавливаем цвет
        fputs(color, stdout);
        // Печатаем текст
        fputs(txt.c_str(), stdout);
        // Сбрасываем цвет к стандартному
        fputs(COL_RESET, stdout);
        // Принудительно сбрасываем буфер консоли, чтобы сообщение появилось сразу
        fflush(stdout);
    }

    // Добавление строки в очередь для фоновой записи в файл
    void pushToFile(const std::string &txt)
    {
        // Блокируем очередь на время добавления
        std::lock_guard<std::mutex> lock(queue_mutex_);
        // Помещаем строку в конец очереди
        queue_.push(txt);
        // Уведомляем фоновый поток, что появилось новое сообщение
        queue_cv_.notify_one();
    }

    // Основной цикл фонового потока: читает очередь и пишет в файл
    void threadLoop()
    {
        // Работаем, пока не получим команду на остановку
        while (!stop_flag_)
        {
            // Блокируем мьютекс и ждём, пока очередь не станет непустой или не придёт сигнал остановки
            std::unique_lock<std::mutex> lock(queue_mutex_);
            queue_cv_.wait(lock, [&]
                           { return stop_flag_ || !queue_.empty(); });

            // Пока в очереди есть сообщения — пишем их в файл
            while (!queue_.empty())
            {
                const std::string &s = queue_.front();  // Берём первое сообщение
                fwrite(s.c_str(), 1, s.size(), file_);  // Пишем в файл
                fflush(file_);                          // Сбрасываем буфер файла на диск
                queue_.pop();                           // Удаляем сообщение из очереди
            }
        }
    }

    // Создание директории, если она не существует (работает только для одного уровня вложенности)
    void createDirectory(const std::string &path)
    {
        if (path.empty())
            return;
        struct stat st{};  // Структура для хранения информации о файле/директории
        // Проверяем, существует ли путь
        if (stat(path.c_str(), &st) != 0)
        {
            // Если не существует — создаём директорию с правами 0755 (владелец: rwx, остальные: r-x)
            mkdir(path.c_str(), 0755);
        }
    }

    // Возвращает строку с текущей датой и временем в формате ГГГГММДД_ЧЧММСС (без миллисекунд)
    std::string datetimeNow()
    {
        char buf[32];
        time_t t = time(nullptr);  // Текущее время в секундах
        std::tm tm{};
        localtime_r(&t, &tm);      // Разбор времени на компоненты

        // Форматируем дату-время как одну строку
        snprintf(buf, sizeof(buf),
                 "%04d-%02d-%02d_%02d-%02d-%02d",
                 tm.tm_year + 1900,
                 tm.tm_mon + 1,
                 tm.tm_mday,
                 tm.tm_hour,
                 tm.tm_min,
                 tm.tm_sec);
        return buf;
    }

private:
    // Указатель на открытый файл лога
    FILE *file_ = nullptr;
    // Полный путь к файлу лога
    std::string filepath_;
    // Флаг: успешно ли завершилась инициализация
    std::atomic<bool> initialized_{false};

    // Очередь сообщений для записи в файл
    std::queue<std::string> queue_;
    // Мьютекс для защиты очереди от одновременного доступа
    std::mutex queue_mutex_;
    // Переменная условия для уведомления потока записи
    std::condition_variable queue_cv_;

    // Фоновый поток записи
    std::thread worker_;
    // Атомарный флаг: нужно ли остановить поток
    std::atomic<bool> stop_flag_{false};
};

/*

// ======================================================
//                 Пример использования AsyncFileLogger
// ======================================================
//
// Пример показывает ВСЕ методы логгера:
//  logf()  – только в файл
//  log()   – файл + экран (белый)
//  log_g() – зелёный текст + файл
//  log_r() – красный текст + файл
//  log_w() – жёлтый текст + файл
//  log_b() – синий текст + файл
//
// Чтобы пример заработал, просто добавьте в свой .cpp файл:
//    #include "logi.h"
//    int main() { example_logger_usage(); }
//
// Или временно раскомментируйте main() ниже.
//

static void example_logger_usage()
{
    // Создаём логгер
    AsyncFileLogger logi("/home/pi/RoboBIM-Linux/src/pb/log/", "control_node");

    // --------------------------------------------------
    // 1) Только в файл
    // --------------------------------------------------
    log.logf("Это сообщение попадёт ТОЛЬКО в файл.\n");

    // --------------------------------------------------
    // 2) На экран + в файл (обычный белый)
    // --------------------------------------------------
    log.log("Обычный лог: скорость = %d, ошибка = %.2f\n", 42, 0.123);

    // --------------------------------------------------
    // 3) Цветные логи + запись в файл
    // --------------------------------------------------
    log.log_g("Зелёный лог — всё хорошо!\n");
    log.log_r("Красный лог — ошибка!\n");
    log.log_w("Жёлтый лог — предупреждение!\n");
    log.log_b("Синий лог — информационное сообщение.\n");

    // В конце функции логгер корректно завершится,
    // поток записи закроется в деструкторе.
}

/*
 // Если хочешь можно прямо здесь сделать полноценный main().
 // Он демонстрационный — его можно раскомментировать для теста.

int main() {
    example_logger_usage();
    return 0;
}
*/

#endif