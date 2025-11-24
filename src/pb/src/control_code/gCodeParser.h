
#ifndef GCODEPARSER_H // Проверка, что заголовочный файл не был включен ранее.
#define GCODEPARSER_H // Определение макроса для предотвращения повторного включения.

#include <ros/ros.h> // Подключаем заголовочный файл ROS для работы с параметрами узла.
#include <fstream>   // Для работы с файлами (чтение G-кода).
#include <sstream>   // Для работы со строковыми потоками (парсинг строк).
#include <string>    // Для работы со строками.
#include <vector>    // Для работы с динамическими массивами (хранение команд).
#include <cmath>     // Для математических функций (sin, cos, atan2, sqrt, abs).
#include <exception> // Для обработки исключений (при парсинге float).
#include <iomanip>   // Для управления форматированием вывода чисел.

extern AsyncFileLogger logi; // Объявление внешней переменной для логирования.
// Примечание: предполагается, что SCommand и commandArray объявлены где-то в другом месте.

#ifndef M_PI                        // Проверка, что константа M_PI не определена.
#define M_PI 3.14159265358979323846 // Определение константы числа Пи.
#endif                              // Конец блока определения M_PI.

// Вспомогательная функция для красивого вывода чисел
std::string fmt_float(float val) // Функция для форматирования числа с плавающей точкой.
{
    std::ostringstream oss;                                  // Создаем строковый поток вывода.
    oss << std::fixed << std::setprecision(4) << val;        // Форматируем число: фиксированная точка, 4 знака после запятой.
    std::string s = oss.str();                               // Преобразуем поток в строку.
    s.erase(s.find_last_not_of('0') + 1, std::string::npos); // Удаляем конечные нули.
    if (!s.empty() && s.back() == '.')
        s.pop_back(); // Удаляем конечную точку, если она осталась после удаления нулей.
    return s;         // Возвращаем отформатированную строку.
} // Конец функции fmt_float.

struct GCodeCommand // Структура для хранения одной команды G-кода и ее параметров.
{
    std::string command;                                      // Основная команда (например, "G0", "G1", "M3").
    bool has_l, has_a, has_f, has_p, has_t, has_f_l, has_f_r; // Флаги наличия параметров L, A, F, P, T, F_L, F_R.
    bool has_o, has_x, has_y;                                 // Флаги наличия параметров O, X, Y.
    float l, a, f, p, t, f_l, f_r;                            // Значения параметров L, A, F, P, T (мс), F_L, F_R.
    float o, x, y;                                            // Значения параметров O, X, Y.
    std::string comment;                                      // Комментарий, идущий после команды.
    std::string raw_line;                                     // Исходная (сырая) строка G-кода.
    int line_number;                                          // Номер строки в файле.

    GCodeCommand()                                                // Конструктор по умолчанию.
        : has_l(false), has_a(false), has_f(false), has_p(false), // Инициализация флагов наличия параметров как false.
          has_t(false), has_f_l(false), has_f_r(false),           // Продолжение инициализации флагов.
          has_o(false), has_x(false), has_y(false),               // Продолжение инициализации флагов.
          l(0.0f), a(0.0f), f(0.0f), p(0.0f), t(0.0f),            // Инициализация значений параметров нулем.
          f_l(0.0f), f_r(0.0f),                                   // Инициализация значений параметров нулем.
          o(0.0f), x(0.0f), y(0.0f),                              // Инициализация значений параметров нулем.
          line_number(0)                                          // Инициализация номера строки нулем.
    {
    } // Конец конструктора.
}; // Конец структуры GCodeCommand.

class GCodeParser // Класс, реализующий парсинг G-кода и симуляцию движения робота.
{
private:
    ros::NodeHandle nh_;                      // Объект для работы с параметрами ROS-узла.
    float current_x_, current_y_, current_a_; // Текущая поза робота: X, Y (метры), A (угол в градусах).
    std::string gcode_file_;                  // Путь к файлу G-кода.
    std::vector<GCodeCommand> commands_;      // Вектор для хранения всех разобранных команд.
    float total_time_sec_ = 0.0f;             // Общее симулированное время выполнения (в секундах).

    std::string trim(const std::string &str) // Вспомогательная функция для удаления пробелов/переводов строки с краев строки.
    {
        size_t first = str.find_first_not_of(" \t\n\r"); // Находим индекс первого символа, не являющегося пробелом.
        if (first == std::string::npos)
            return "";                                 // Если строка состоит только из пробелов, возвращаем пустую строку.
        size_t last = str.find_last_not_of(" \t\n\r"); // Находим индекс последнего символа, не являющегося пробелом.
        return str.substr(first, (last - first + 1));  // Возвращаем подстроку от первого до последнего непробельного символа.
    } // Конец функции trim.

    float normalizeAngle(float angle_deg) // Вспомогательная функция для нормализации угла в диапазон (-180, 180] градусов.
    {
        while (angle_deg > 180.0f)
            angle_deg -= 360.0f; // Уменьшаем угол, пока он больше 180.
        while (angle_deg <= -180.0f)
            angle_deg += 360.0f; // Увеличиваем угол, пока он меньше или равен -180.
        return angle_deg;        // Возвращаем нормализованный угол.
    } // Конец функции normalizeAngle.

    bool parseGCodeLine(const std::string &line, GCodeCommand &cmd, int line_number) // Основная функция парсинга одной строки G-кода.
    {
        std::string trimmed_line = trim(line); // Удаляем лишние пробелы с краев.
        if (trimmed_line.empty())
            return false; // Если строка пуста, выходим.

        cmd.raw_line = line;           // Сохраняем исходную строку.
        cmd.line_number = line_number; // Сохраняем номер строки.

        std::stringstream ss(trimmed_line); // Создаем строковый поток для удобного чтения токенов.
        std::string token;                  // Переменная для хранения текущего токена.
        ss >> token;                        // Читаем первый токен (команду G/M).

        if (token.empty() || (token[0] != 'G' && token[0] != 'M')) // Проверяем, что первый токен - это G- или M-команда.
        {
            logi.log_w("Line %d: Invalid command token '%s'\n", line_number, token.c_str()); // Логируем ошибку, если команда недействительна.
            return false;                                                                    // Возвращаем false, если команда некорректна.
        }
        cmd.command = token; // Сохраняем команду.

        while (ss >> token) // Читаем остальные токены (параметры).
        {
            if (token[0] == ';') // Проверяем, не является ли токен началом комментария.
            {
                std::getline(ss, cmd.comment);   // Считываем оставшуюся часть строки как комментарий.
                cmd.comment = trim(cmd.comment); // Удаляем пробелы с краев комментария.
                break;                           // Выходим из цикла парсинга токенов.
            }
            if (token.length() < 2) // Проверяем минимальную длину токена (параметр + значение).
            {
                logi.log_w("Line %d: Skipping short token '%s'\n", line_number, token.c_str()); // Логируем пропуск слишком короткого токена.
                continue;                                                                       // Переходим к следующему токену.
            }

            char param = token[0]; // Первый символ - это имя параметра (L, A, F и т.д.).
            float value;           // Переменная для значения параметра.

            // === Проверка на кириллическую "О" ===
            if (token.length() >= 2 &&                          // Проверяем, что токен достаточно длинный.
                static_cast<unsigned char>(token[0]) == 0xD0 && // Проверяем первый байт (начало кириллической буквы в UTF-8).
                static_cast<unsigned char>(token[1]) == 0x9E)   // Проверяем второй байт ('О' кириллическая).
            {
                logi.log_r("Line %d: Invalid parameter 'О' (Cyrillic O). Use Latin 'O' for relative angle!\n", // Логируем ошибку кириллической буквы.
                           line_number);
                continue; // Переходим к следующему токену.
            }

            try // Начинаем блок для обработки ошибок преобразования строки в число.
            {
                if (param == 'F' && token.length() >= 3 && token[1] == '_') // Специальная обработка для F_L и F_R.
                {
                    std::string suffix = token.substr(2, 1);  // Получаем суффикс (L или R).
                    std::string number_str = token.substr(3); // Получаем числовую часть.
                    if (number_str.empty())                   // Проверяем, что числовая часть не пуста.
                    {
                        logi.log_w("Line %d: Empty number after F_\n", line_number); // Логируем ошибку пустого числа.
                        continue;                                                    // Переходим к следующему токену.
                    }
                    value = std::stof(number_str); // Преобразуем числовую часть в float.
                    if (suffix == "L")             // Если суффикс 'L'.
                    {
                        cmd.f_l = value;    // Сохраняем значение F_L.
                        cmd.has_f_l = true; // Устанавливаем флаг наличия F_L.
                    }
                    else if (suffix == "R") // Если суффикс 'R'.
                    {
                        cmd.f_r = value;    // Сохраняем значение F_R.
                        cmd.has_f_r = true; // Устанавливаем флаг наличия F_R.
                    }
                    else // Неизвестный суффикс F_.
                    {
                        logi.log_w("Line %d: Unknown F suffix '%s'\n", line_number, suffix.c_str()); // Логируем ошибку.
                    }
                }
                else // Стандартная обработка для одиночных параметров.
                {
                    value = std::stof(token.substr(1)); // Преобразуем числовую часть токена в float.
                    if (param == 'F')
                    {
                        cmd.f = value;
                        cmd.has_f = true;
                    } // Параметр F (общая скорость).
                    else if (param == 'L')
                    {
                        cmd.l = value;
                        cmd.has_l = true;
                    } // Параметр L (длина).
                    else if (param == 'A')
                    {
                        cmd.a = value;
                        cmd.has_a = true;
                    } // Параметр A (абсолютный угол).
                    else if (param == 'O')
                    {
                        cmd.o = value;
                        cmd.has_o = true;
                    } // Параметр O (относительный угол).
                    else if (param == 'P')
                    {
                        cmd.p = value;
                        cmd.has_p = true;
                    } // Параметр P (пауза в мс).
                    else if (param == 'T')
                    {
                        cmd.t = value;
                        cmd.has_t = true;
                    } // Параметр T (длительность в мс).
                    else if (param == 'X')
                    {
                        cmd.x = value;
                        cmd.has_x = true;
                    } // Параметр X (координата X).
                    else if (param == 'Y')
                    {
                        cmd.y = value;
                        cmd.has_y = true;
                    } // Параметр Y (координата Y).
                    else // Неизвестный параметр.
                    {
                        logi.log_w("Line %d: Unknown parameter '%c' (code=%d)\n", // Логируем ошибку неизвестного параметра.
                                   line_number, param, static_cast<int>(param));
                        continue; // Переходим к следующему токену.
                    }
                }
            }
            catch (const std::exception &e) // Ловим исключение, если преобразование строки в число не удалось.
            {
                logi.log_w("Line %d: Parse error in '%s': %s\n", line_number, token.c_str(), e.what()); // Логируем ошибку парсинга.
                continue;                                                                               // Переходим к следующему токену.
            }
        } // Конец цикла while (ss >> token).

        // === КОМПАКТНЫЙ ЛОГ ПАРСИНГА В ОДНУ СТРОКУ ===
        std::string parsed_params; // Строка для сбора всех разобранных параметров.
        if (cmd.has_l)
            parsed_params += " L=" + fmt_float(cmd.l); // Добавляем L.
        if (cmd.has_a)
            parsed_params += " A=" + fmt_float(cmd.a); // Добавляем A.
        if (cmd.has_o)
            parsed_params += " O=" + fmt_float(cmd.o); // Добавляем O.
        if (cmd.has_f && !cmd.has_f_l && !cmd.has_f_r)
            parsed_params += " F=" + fmt_float(cmd.f); // Добавляем F (если нет F_L/F_R).
        if (cmd.has_p)
            parsed_params += " P=" + fmt_float(cmd.p); // Добавляем P.
        if (cmd.has_t)
            parsed_params += " T=" + fmt_float(cmd.t); // Добавляем T.
        if (cmd.has_f_l)
            parsed_params += " F_L=" + fmt_float(cmd.f_l); // Добавляем F_L.
        if (cmd.has_f_r)
            parsed_params += " F_R=" + fmt_float(cmd.f_r); // Добавляем F_R.
        if (cmd.has_x)
            parsed_params += " X=" + fmt_float(cmd.x); // Добавляем X.
        if (cmd.has_y)
            parsed_params += " Y=" + fmt_float(cmd.y); // Добавляем Y.
        if (!cmd.comment.empty())
            parsed_params += " ; " + cmd.comment; // Добавляем комментарий.

        logi.log_b("Line %d: Parsed '%s'%s\n", line_number, cmd.command.c_str(), parsed_params.c_str()); // Логируем результат парсинга.
        return true;                                                                                     // Парсинг успешен.
    } // Конец функции parseGCodeLine.

    void executeG0(const GCodeCommand &cmd) // Обработка команды G0 (движение по заданным скоростям колес F_L, F_R в течение времени T).
    {
        logi.log_g(">>> EXECUTING Line %d: %s\n", cmd.line_number, cmd.raw_line.c_str());        // Логируем команду.
        logi.log_b("    BEFORE: X=%.3f, Y=%.3f, A=%.3f°\n", current_x_, current_y_, current_a_); // Выводим текущую позу.

        if (!cmd.has_f_l || !cmd.has_f_r || !cmd.has_t) // Проверяем обязательные параметры.
        {
            logi.log_r("Line %d: G0 requires F_L, F_R, T\n", cmd.line_number); // Логируем ошибку, если параметры отсутствуют.
            return;                                                            // Прерываем выполнение.
        }
        // T здесь, в отличие от M3/M5, считается в секундах, как время движения.
        if (cmd.t <= 0.0f || cmd.f_l < 0.0f || cmd.f_r < 0.0f) // Проверяем корректность значений.
        {
            logi.log_r("Line %d: Invalid G0 params: T=%.3f, F_L=%.3f, F_R=%.3f\n", // Логируем некорректные значения.
                       cmd.line_number, cmd.t, cmd.f_l, cmd.f_r);
            return; // Прерываем выполнение.
        }

        // Сохраняем состояние ДО
        float before_x = current_x_; // Сохраняем X до движения.
        float before_y = current_y_; // Сохраняем Y до движения.
        float before_a = current_a_; // Сохраняем угол A до движения.

        float v = (cmd.f_l + cmd.f_r) / 2.0f;                // Линейная скорость робота (м/с).
        float omega = (cmd.f_r - cmd.f_l) / DISTANCE_WHEELS; // Угловая скорость робота (рад/с).
        float time_sec = cmd.t;                              // Время движения (секунды).

        if (std::abs(omega) < 1e-6f) // Если угловая скорость близка к нулю (движение по прямой).
        {
            float length = v * time_sec; // Вычисляем пройденное расстояние.
            // Движение вперед по текущему углу (A переведен в радианы).
            current_x_ += length * std::cos(before_a * M_PI / 180.0f); // Обновляем X.
            current_y_ += length * std::sin(before_a * M_PI / 180.0f); // Обновляем Y.
            // Угол current_a_ не меняется.
        }
        else // Движение по дуге (Circular Arc Motion).
        {
            float radius = v / omega;                // Вычисляем радиус поворота.
            float theta = omega * time_sec;          // Вычисляем изменение угла (в радианах).
            float sign = (omega > 0) ? 1.0f : -1.0f; // Знак поворота для вычисления центра окружности.

            // Вычисляем координаты центра вращения.
            float center_x = before_x - radius * std::sin(before_a * M_PI / 180.0f);
            float center_y = before_y + radius * std::cos(before_a * M_PI / 180.0f);

            // Вычисляем конечную позу.
            float end_angle_rad = before_a * M_PI / 180.0f + theta;
            current_x_ = center_x + radius * std::sin(end_angle_rad);
            current_y_ = center_y - radius * std::cos(end_angle_rad);
            current_a_ = normalizeAngle(before_a + theta * 180.0f / M_PI); // Обновляем и нормализуем угол.
        }

        total_time_sec_ += time_sec; // Обновляем общее время.

        SCommand command;                   // Создаем команду для массива.
        command.mode = 0;                   // Режим G0.
        command.duration = cmd.t * 1000.0f; // Длительность в мс.
        command.velL = cmd.f_l;             // Скорость левого колеса.
        command.velR = cmd.f_r;             // Скорость правого колеса.
        // Сохраняем траекторию
        command.point_A_x = before_x;    // Начальная координата X.
        command.point_A_y = before_y;    // Начальная координата Y.
        command.point_A_a = before_a;    // Начальный угол A.
        command.point_B_x = current_x_;  // Конечная координата X.
        command.point_B_y = current_y_;  // Конечная координата Y.
        command.point_B_a = current_a_;  // Конечный угол A.
        commandArray.push_back(command); // Добавляем команду в массив.

        logi.log_b("    AFTER:  X=%.3f, Y=%.3f, A=%.3f°\n", current_x_, current_y_, current_a_); // Выводим конечную позу.
        logi.log_b("    DURATION: %.3f sec (TOTAL: %.3f sec)\n", time_sec, total_time_sec_);     // Выводим длительность.
    } // Конец функции executeG0.

    void executeG1(const GCodeCommand &cmd) // Обработка команды G1 (поворот на угол A/O или к точке X/Y).
    {
        logi.log_g(">>> EXECUTING Line %d: %s\n", cmd.line_number, cmd.raw_line.c_str());        // Логируем команду.
        logi.log_b("    BEFORE: X=%.3f, Y=%.3f, A=%.3f°\n", current_x_, current_y_, current_a_); // Выводим текущую позу.

        int mode_count = (cmd.has_a ? 1 : 0) + (cmd.has_o ? 1 : 0) + ((cmd.has_x || cmd.has_y) ? 1 : 0); // Считаем, сколько режимов задано.
        if ((cmd.has_x || cmd.has_y) && !(cmd.has_x && cmd.has_y))                                       // Проверяем, что X и Y заданы либо оба, либо ни один.
        {
            logi.log_r("Line %d: G1 requires both X and Y\n", cmd.line_number); // Логируем ошибку.
            return;                                                             // Прерываем выполнение.
        }
        if (mode_count != 1) // Проверяем, что задан ровно один режим (A, O, или X/Y).
        {
            logi.log_r("Line %d: G1 must have exactly one of A, O, or X/Y\n", cmd.line_number); // Логируем ошибку.
            return;                                                                             // Прерываем выполнение.
        }

        float target_angle = 0.0f; // Целевой угол.
        if (cmd.has_a)
            target_angle = normalizeAngle(cmd.a); // Если задан абсолютный угол A.
        else if (cmd.has_o)
            target_angle = normalizeAngle(current_a_ + cmd.o); // Если задан относительный угол O.
        else if (cmd.has_x && cmd.has_y)                       // Если задана целевая точка X/Y.
        {
            float dx = cmd.x - current_x_;                                                         // Разница по X.
            float dy = cmd.y - current_y_;                                                         // Разница по Y.
            target_angle = (std::abs(dx) < 1e-6f && std::abs(dy) < 1e-6f)                          // Если точка совпадает с текущей.
                               ? current_a_                                                        // Угол не меняем.
                               : normalizeAngle(static_cast<float>(atan2(dy, dx) * 180.0 / M_PI)); // Вычисляем и нормализуем угол к точке.
        }

        // Сохраняем состояние ДО
        float before_x = current_x_; // Сохраняем X до поворота.
        float before_y = current_y_; // Сохраняем Y до поворота.
        float before_a = current_a_; // Сохраняем угол A до поворота.

        float angle_diff = normalizeAngle(target_angle - before_a); // Разница углов, которую нужно преодолеть.
        float wheel_speed = cmd.has_f ? cmd.f : 0.1f;               // Линейная скорость колес при повороте (м/с).
        if (wheel_speed > 0.5f)                                     // Ограничение максимальной скорости.
        {
            logi.log_w("G1: clamping wheel speed F=%.3f -> 0.5 m/s\n", wheel_speed); // Логируем ограничение.
            wheel_speed = 0.5f;                                                      // Устанавливаем максимальное значение.
        }
        if (wheel_speed <= 0.0f) // Проверка на положительную скорость.
        {
            logi.log_r("Line %d: Invalid G1 F=%.3f (must be > 0)\n", cmd.line_number, wheel_speed); // Логируем ошибку.
            return;                                                                                 // Прерываем выполнение.
        }

        // Кинематика поворота на месте (v=0). Скорость поворота колес: V_L = -V_R = wheel_speed.
        float omega_rad_per_sec = (2.0f * wheel_speed) / DISTANCE_WHEELS; // Угловая скорость (рад/с).
        float omega_deg_per_sec = omega_rad_per_sec * (180.0f / M_PI);    // Угловая скорость (град/с).
        float time_sec = std::abs(angle_diff) / omega_deg_per_sec;        // Время, необходимое для поворота.

        // current_x_ и current_y_ не меняются, так как это поворот на месте.
        current_a_ = target_angle;   // Обновляем угол.
        total_time_sec_ += time_sec; // Обновляем общее время.

        SCommand command;               // Создаем команду для массива.
        command.mode = 1;               // Режим G1 (поворот).
        command.angle = target_angle;   // Целевой угол.
        command.velAngle = wheel_speed; // Скорость колес при повороте.
        // Сохраняем траекторию
        command.point_A_x = before_x;    // Начальная координата X.
        command.point_A_y = before_y;    // Начальная координата Y.
        command.point_A_a = before_a;    // Начальный угол A.
        command.point_B_x = current_x_;  // Конечная координата X (та же).
        command.point_B_y = current_y_;  // Конечная координата Y (та же).
        command.point_B_a = current_a_;  // Конечный угол A.
        commandArray.push_back(command); // Добавляем команду в массив.

        logi.log_b("    AFTER:  X=%.3f, Y=%.3f, A=%.3f°\n", current_x_, current_y_, current_a_); // Выводим конечную позу.
        logi.log_b("    DURATION: %.3f sec (TOTAL: %.3f sec)\n", time_sec, total_time_sec_);     // Выводим длительность.
    } // Конец функции executeG1.

    void executeG2(const GCodeCommand &cmd) // Обработка команды G2 (линейное движение на L или к X/Y).
    {
        logi.log_g(">>> EXECUTING Line %d: %s\n", cmd.line_number, cmd.raw_line.c_str());        // Логируем команду.
        logi.log_b("    BEFORE: X=%.3f, Y=%.3f, A=%.3f°\n", current_x_, current_y_, current_a_); // Выводим текущую позу.

        bool has_length = cmd.has_l;             // Флаг наличия параметра L.
        bool has_point = cmd.has_x || cmd.has_y; // Флаг наличия параметров X/Y.

        if (has_point && !(cmd.has_x && cmd.has_y)) // Проверка, что X и Y заданы либо оба, либо ни один.
        {
            logi.log_r("Line %d: G2 requires both X and Y\n", cmd.line_number); // Логируем ошибку.
            return;                                                             // Прерываем выполнение.
        }
        if (has_length && has_point) // G2 не может иметь одновременно L и X/Y.
        {
            logi.log_r("Line %d: G2 cannot have both L and X/Y\n", cmd.line_number); // Логируем ошибку.
            return;                                                                  // Прерываем выполнение.
        }
        if (!has_length && !has_point) // G2 должен иметь либо L, либо X/Y.
        {
            logi.log_r("Line %d: G2 must have L or X/Y\n", cmd.line_number); // Логируем ошибку.
            return;                                                          // Прерываем выполнение.
        }
        // Проверка, что F задана для движения.
        if (!cmd.has_f)
        {
            logi.log_r("Line %d: G2 requires F (feed rate) to determine direction and speed.\n", cmd.line_number); // Логируем ошибку.
            return;                                                                                                // Прерываем выполнение.
        }

        // Сохраняем состояние ДО
        float before_x = current_x_; // Сохраняем X до движения.
        float before_y = current_y_; // Сохраняем Y до движения.
        float before_a = current_a_; // Сохраняем угол A до движения.

        float target_x = before_x; // Целевая X.
        float target_y = before_y; // Целевая Y.
        float length = 0.0f;       // Абсолютная длина пути.

        // F теперь используется для задания скорости И направления.
        float signed_feed_rate = cmd.f;        // Скорость со знаком (F).
        float abs_feed_rate = std::abs(cmd.f); // Абсолютная скорость для расчета времени.

        if (abs_feed_rate > 0.5f) // Ограничение максимальной абсолютной скорости.
        {
            abs_feed_rate = 0.5f;                                                        // Ограничиваем абсолютное значение.
            signed_feed_rate = (cmd.f < 0.0f) ? -0.5f : 0.5f;                            // Восстанавливаем знак.
            logi.log_w("G2: clamping feed rate |F|=%.3f -> 0.5 m/s\n", std::abs(cmd.f)); // Логируем ограничение.
        }

        if (abs_feed_rate < 1e-6f) // Проверка на нулевую скорость.
        {
            logi.log_r("Line %d: Invalid G2 F=%.3f (must be non-zero)\n", cmd.line_number, cmd.f); // Логируем ошибку.
            return;                                                                                // Прерываем выполнение.
        }

        if (has_length) // Движение на заданную длину L.
        {
            if (cmd.l <= 0.0f) // Длина L должна быть строго положительной (абсолютное расстояние!).
            {
                logi.log_r("Line %d: Invalid G2 L=%.3f. L must be strictly positive (absolute distance).\n", cmd.line_number, cmd.l); // Логируем ошибку.
                return;                                                                                                               // Прерываем выполнение.
            }
            length = cmd.l; // Сохраняем длину (положительную).

            // Расчет конечных координат. Используем ЗНАК F через знак signed_feed_rate:
            // 1.0 для вперед (F>=0), -1.0 для назад (F<0).
            float direction = (signed_feed_rate >= 0.0f) ? 1.0f : -1.0f;

            // Смещение равно L, умноженному на направление.
            float signed_length = length * direction;

            // Расчет конечных координат исходя из текущего угла.
            target_x = before_x + signed_length * std::cos(before_a * M_PI / 180.0f);
            target_y = before_y + signed_length * std::sin(before_a * M_PI / 180.0f);
        }
        else if (has_point) // Движение к заданной точке X/Y.
        {
            target_x = cmd.x;                      // Целевая X.
            target_y = cmd.y;                      // Целевая Y.
            float dx = target_x - before_x;        // Смещение по X.
            float dy = target_y - before_y;        // Смещение по Y.
            length = std::sqrt(dx * dx + dy * dy); // Вычисляем длину пути (всегда положительная).

            // ВАЖНО: Мы разрешаем отрицательную скорость F при движении по координатам.
            // Это позволяет ехать к целевой точке (target_x, target_y) ЗАДОМ.
            // Simulatоr (текущий класс) просто перемещает робота в target_x/target_y.
            // Driver (низкий уровень) будет использовать знак signed_feed_rate (-), чтобы крутить колеса назад.
            // Примечание: Пользователь сам отвечает за то, чтобы робот был ориентирован "спиной" к цели,
            // если F отрицательная. Парсер просто исполняет команду перемещения.
        }

        float time_sec = (length > 0.0f) ? (length / abs_feed_rate) : 0.0f; // Время выполнения (длина/абс.скорость).
        current_x_ = target_x;                                              // Обновляем X.
        current_y_ = target_y;                                              // Обновляем Y.
        // current_a_ не меняется.
        total_time_sec_ += time_sec; // Обновляем общее время.

        SCommand command;                  // Создаем команду для массива.
        command.mode = 2;                  // Режим G2 (линейное движение).
        command.len = length;              // Длина пути (абсолютная).
        command.velLen = signed_feed_rate; // Скорость движения (со знаком F для направления!).
        // Сохраняем траекторию
        command.point_A_x = before_x;    // Начальная координата X.
        command.point_A_y = before_y;    // Начальная координата Y.
        command.point_A_a = before_a;    // Начальный угол A.
        command.point_B_x = current_x_;  // Конечная координата X.
        command.point_B_y = current_y_;  // Конечная координата Y.
        command.point_B_a = current_a_;  // Конечный угол A.
        commandArray.push_back(command); // Добавляем команду в массив.

        logi.log_b("    AFTER:  X=%.3f, Y=%.3f, A=%.3f°\n", current_x_, current_y_, current_a_); // Выводим конечную позу.
        logi.log_b("    DURATION: %.3f sec (TOTAL: %.3f sec)\n", time_sec, total_time_sec_);     // Выводим длительность.
    } // Конец функции executeG2.

    void executeG4(const GCodeCommand &cmd) // Обработка команды G4 (пауза P в мс).
    {
        logi.log_g(">>> EXECUTING Line %d: %s\n", cmd.line_number, cmd.raw_line.c_str()); // Логируем команду.

        // Сохраняем состояние ДО (и ПОСЛЕ — оно не меняется)
        float before_x = current_x_; // Сохраняем X.
        float before_y = current_y_; // Сохраняем Y.
        float before_a = current_a_; // Сохраняем угол A.

        float pause_ms = cmd.has_p ? cmd.p : 0.0f; // Получаем длительность паузы в миллисекундах.
        float time_sec = pause_ms / 1000.0f;       // Переводим в секунды.
        total_time_sec_ += time_sec;               // Обновляем общее время.

        SCommand temp;            // Создаем команду для массива.
        temp.mode = 0;            // Режим движения, но без изменения координат/скоростей (0).
        temp.duration = pause_ms; // Длительность паузы в мс.
        // Сохраняем траекторию (без изменений)
        temp.point_A_x = before_x;    // Начальная X.
        temp.point_A_y = before_y;    // Начальная Y.
        temp.point_A_a = before_a;    // Начальный A.
        temp.point_B_x = before_x;    // Конечная X (та же).
        temp.point_B_y = before_y;    // Конечная Y (та же).
        temp.point_B_a = before_a;    // Конечный A (тот же).
        commandArray.push_back(temp); // Добавляем команду в массив.

        logi.log_b("    DURATION: %.3f sec (TOTAL: %.3f sec)\n", time_sec, total_time_sec_); // Выводим длительность.
    } // Конец функции executeG4.

    void executeG9(const GCodeCommand &cmd) // Обработка команды G9 (установка/сброс чего-либо).
    {
        logi.log_g(">>> EXECUTING Line %d: %s\n", cmd.line_number, cmd.raw_line.c_str());       // Логируем команду.
        logi.log_b("    STATE: X=%.3f, Y=%.3f, A=%.3f°\n", current_x_, current_y_, current_a_); // Выводим текущее состояние.

        SCommand temp; // Создаем команду для массива.
        temp.mode = 9; // Режим G9.
        // Сохраняем текущее состояние как траекторию (без изменений)
        temp.point_A_x = current_x_;  // Начальная X.
        temp.point_A_y = current_y_;  // Начальная Y.
        temp.point_A_a = current_a_;  // Начальный A.
        temp.point_B_x = current_x_;  // Конечная X.
        temp.point_B_y = current_y_;  // Конечная Y.
        temp.point_B_a = current_a_;  // Конечный A.
        commandArray.push_back(temp); // Добавляем команду в массив.
    } // Конец функции executeG9.

    void executeG10(const GCodeCommand &cmd) // Обработка команды G10 (Установка начальной позиции).
    {

        logi.log_w(">>> EXECUTING Line %d: %s\n", cmd.line_number, cmd.raw_line.c_str());        // Логируем команду.
        logi.log_b("    BEFORE: X=%.3f, Y=%.3f, A=%.3f°\n", current_x_, current_y_, current_a_); // Выводим позу до изменения.

        ///------------- БЛОК где я должен приехать в точку откуда буду рисовать. Чтобы потом в ней передать управление по gcode
        // ros::Duration(10).sleep(); // Подождем пока
        current_x_ = msg_PoseRotation.x.main;           // Заменяем координаты для правильного расчета
        current_y_ = msg_PoseRotation.y.main;           // Заменяем координаты для правильного расчета
        current_a_ = RAD2DEG(msg_PoseRotation.th.main); // Заменяем координаты для правильного расчета

        logi.log_b("    REAL CURRENT: X=%.3f, Y=%.3f, A=%.3f°\n", current_x_, current_y_, current_a_);  // Считываем реальную позицию
        logi.log_b("    TARGET POINT: X=%.3f, Y=%.3f, A=%.3f°\n", cmd.x, cmd.y, normalizeAngle(cmd.a)); // Считываем реальную позицию

        GCodeCommand cmd_temp;
        cmd_temp.command = "M5"; // Поворот на угол в точку откуда рисовать
        cmd_temp.comment = "Manual command M5 UP morker";
        cmd_temp.raw_line = "M5 Manual command M5 UP morker";
        cmd_temp.t = 2000; // Параметр T (длительность действия).
        cmd_temp.has_t = true;
        executeM5(cmd_temp); // Выполняем M5. Родбем маркера перед любым движением

        cmd_temp = {};           // Сброс значений
        cmd_temp.command = "G1"; // Поворот на угол в точку откуда рисовать
        cmd_temp.comment = "Manual command G1 for rotate to start point";
        cmd_temp.raw_line = "G1 Manual command G1 for rotate to start point";
        cmd_temp.f = 0.05; // Параметр F (общая скорость).
        cmd_temp.has_f = true;
        cmd_temp.x = cmd.x; // Параметр X (координата X).
        cmd_temp.has_x = true;
        cmd_temp.y = cmd.y; // Параметр Y (координата Y).
        cmd_temp.has_y = true;
        executeG1(cmd_temp); // Выполняем G1. Тут должен повернуться в точку откуда надо будет рисовать из своей текущей позиции

        cmd_temp = {};           // Сброс значений
        cmd_temp.command = "G4"; // Пауза
        cmd_temp.comment = "Manual command G4 Pause";
        cmd_temp.raw_line = "G4 Manual command G4 Pause";
        cmd_temp.p = 2000;
        cmd_temp.has_p = true;
        executeG4(cmd_temp); // Выполняем G4.

        cmd_temp = {};           // Сброс значений
        cmd_temp.command = "G2"; // Движение в точку откуда рисовать
        cmd_temp.comment = "Manual command G2 for run to start point";
        cmd_temp.raw_line = "G2 Manual command G2 for run to start point";
        cmd_temp.f = 0.20; // Параметр F (общая скорость).
        cmd_temp.has_f = true;
        cmd_temp.x = cmd.x; // Параметр X (координата X).
        cmd_temp.has_x = true;
        cmd_temp.y = cmd.y; // Параметр Y (координата Y).
        cmd_temp.has_y = true;
        executeG2(cmd_temp); // Выполняем G1. Тут должен поехать в точку откуда надо будет рисовать из своей текущей позиции

        cmd_temp = {};           // Сброс значений
        cmd_temp.command = "G4"; // Пауза
        cmd_temp.comment = "Manual command G4 Pause";
        cmd_temp.raw_line = "G4 Manual command G4 Pause";
        cmd_temp.p = 2000;
        cmd_temp.has_p = true;
        executeG4(cmd_temp); // Выполняем G4.

        cmd_temp = {};           // Сброс значений
        cmd_temp.command = "G1"; // Поворот в угол 0
        cmd_temp.comment = "Manual command G1 for rotate to 0 gradus";
        cmd_temp.raw_line = "G1 Manual command G1 for rotate to 0 gradus";
        cmd_temp.f = 0.05; // Параметр F (общая скорость).
        cmd_temp.has_f = true;
        cmd_temp.a = 0;
        cmd_temp.has_a = true;

        executeG1(cmd_temp); // Выполняем G1. Тут должен повернуться в угол 0

        ///------------------------------------------------------------------------------

        // Получаем новые значения. Если параметр не задан, оставляем текущее значение.
        float new_x = cmd.has_x ? cmd.x : current_x_;                 // Если X задан, берем его.
        float new_y = cmd.has_y ? cmd.y : current_y_;                 // Если Y задан, берем его.
        float new_a = cmd.has_a ? normalizeAngle(cmd.a) : current_a_; // Если A задан, берем его и нормализуем.

        // !!! ОБНОВЛЯЕМ СОСТОЯНИЕ СИМУЛЯТОРА !!!
        current_x_ = new_x; // Устанавливаем новую координату X.
        current_y_ = new_y; // Устанавливаем новую координату Y.
        current_a_ = new_a; // Устанавливаем новый угол A.

        SCommand command; // Создаем команду для массива.
        command.mode = 6; // Режим 6 (Set Pose / G10).

        // Записываем новую позицию в точку А.
        command.point_A_x = current_x_; // Координата X.
        command.point_A_y = current_y_; // Координата Y.
        command.point_A_a = current_a_; // Угол A.

        // Точка B и угол остаются , так как движения нет (точка B - это целевая точка).
        command.point_B_x = current_x_; // Координата X.
        command.point_B_y = current_y_; // Координата Y.
        command.point_B_a = current_a_; // Угол A.

        commandArray.push_back(command); // Добавляем в массив команд.

        logi.log_w("+++ AFTER (SET): X=%.3f, Y=%.3f, A=%.3f°\n", current_x_, current_y_, current_a_); // Логируем новую позу.
    } // Конец функции executeG10.

    void executeM3(const GCodeCommand &cmd) // Обработка команды M3 (включение печати/устройства с длительностью T в мс).
    {
        logi.log_g(">>> EXECUTING Line %d: %s\n", cmd.line_number, cmd.raw_line.c_str());       // Логируем начало выполнения команды.
        logi.log_b("    STATE: X=%.3f, Y=%.3f, A=%.3f°\n", current_x_, current_y_, current_a_); // Выводим текущее состояние.

        if (!cmd.has_t) // Проверяем наличие параметра T (длительность в мс).
        {
            logi.log_w("Line %d: M3 executed without T (duration in ms).\n", cmd.line_number); // Предупреждение, если T не задан.
        }

        float pause_ms = cmd.has_t ? cmd.t : 0.0f; // Получаем длительность в миллисекундах.
        float time_sec = pause_ms / 1000.0f;       // Переводим длительность в секунды для общего времени.
        total_time_sec_ += time_sec;               // Добавляем время выполнения к общему времени.

        SCommand temp;            // Объявляем временную структуру команды.
        temp.mode = 3;            // Устанавливаем режим M3 (3).
        temp.duration = pause_ms; // Присваиваем длительность в миллисекундах.

        // Координаты не меняются.
        temp.point_A_x = current_x_;  // Начальная координата X.
        temp.point_A_y = current_y_;  // Начальная координата Y.
        temp.point_A_a = current_a_;  // Начальный угол A.
        temp.point_B_x = current_x_;  // Конечная координата X.
        temp.point_B_y = current_y_;  // Конечная координата Y.
        temp.point_B_a = current_a_;  // Конечный угол A.
        commandArray.push_back(temp); // Добавляем команду в массив.

        logi.log_b("    M3 DURATION: %.3f sec (TOTAL: %.3f sec)\n", time_sec, total_time_sec_); // Логируем длительность и общее время.
    } // Конец функции executeM3.

    void executeM5(const GCodeCommand &cmd) // Обработка команды M5 (выключение печати/устройства с длительностью T в мс).
    {
        logi.log_g(">>> EXECUTING Line %d: %s\n", cmd.line_number, cmd.raw_line.c_str());       // Логируем начало выполнения команды.
        logi.log_b("    STATE: X=%.3f, Y=%.3f, A=%.3f°\n", current_x_, current_y_, current_a_); // Выводим текущее состояние.

        if (!cmd.has_t) // Проверяем наличие параметра T (длительность в мс).
        {
            logi.log_w("Line %d: M5 executed without T (duration in ms).\n", cmd.line_number); // Предупреждение, если T не задан.
        }

        float pause_ms = cmd.has_t ? cmd.t : 0.0f; // Получаем длительность в миллисекундах.
        float time_sec = pause_ms / 1000.0f;       // Переводим длительность в секунды для общего времени.
        total_time_sec_ += time_sec;               // Добавляем время выполнения к общему времени.

        SCommand temp;            // Объявляем временную структуру команды.
        temp.mode = 5;            // Устанавливаем режим M5 (5).
        temp.duration = pause_ms; // Присваиваем длительность в миллисекундах.

        // Координаты не меняются.
        temp.point_A_x = current_x_;  // Начальная координата X.
        temp.point_A_y = current_y_;  // Начальная координата Y.
        temp.point_A_a = current_a_;  // Начальный угол A.
        temp.point_B_x = current_x_;  // Конечная координата X.
        temp.point_B_y = current_y_;  // Конечная координата Y.
        temp.point_B_a = current_a_;  // Конечный угол A.
        commandArray.push_back(temp); // Добавляем команду в массив.

        logi.log_b("    M5 DURATION: %.3f sec (TOTAL: %.3f sec)\n", time_sec, total_time_sec_); // Логируем длительность и общее время.
    } // Конец функции executeM5.

public:                                                         // Публичный интерфейс класса.
    GCodeParser()                                               // Конструктор класса GCodeParser.
        : nh_("~"),                                             // Инициализация NodeHandle для приватного пространства имен.
          current_x_(0.0f), current_y_(0.0f), current_a_(0.0f), // Инициализация текущей позы нулем.
          total_time_sec_(0.0f),                                // Инициализация общего времени нулем.
          gcode_file_("gcode/my_custom_gcode.gcode")            // Инициализация пути к файлу G-кода значением по умолчанию.
    {
        nh_.param<std::string>("gcode_file", gcode_file_, "$(find pb)/gcode/my_custom_gcode.gcode"); // Получение параметра "gcode_file" из ROS-параметров.
        logi.log_b("GCodeParser init: %s\n", gcode_file_.c_str());                                   // Логируем путь к файлу G-кода.
        if (!nh_.getParam("gcode_file", gcode_file_))                                                // Повторная проверка получения параметра (для логирования).
        {
            logi.log_w("Using default gcode_file: %s\n", gcode_file_.c_str()); // Логируем использование значения по умолчанию.
        }
    } // Конец конструктора.

    // НОВЫЙ МЕТОД: задать начальную позу
    void setInitialPose(float x, float y, float a_deg) // Функция для установки начальной позы робота.
    {
        current_x_ = x;                                                              // Устанавливаем начальную координату X.
        current_y_ = y;                                                              // Устанавливаем начальную координату Y.
        current_a_ = normalizeAngle(a_deg);                                          // Устанавливаем и нормализуем начальный угол.
        total_time_sec_ = 0.0f;                                                      // Сбрасываем общее время.
        commandArray.clear();                                                        // Очищаем массив команд.
        logi.log_b("Initial pose set: X=%.3f, Y=%.3f, A=%.3f°\n", x, y, current_a_); // Логируем установленную позу.
    } // Конец функции setInitialPose.

    void run() // Основная функция для парсинга файла и симуляции выполнения команд.
    {
        // Очищаем предыдущие команды (но сохраняем текущую позу!)
        commands_.clear(); // Очищаем вектор команд.
        // commandArray и total_time_sec_ уже учтены в setInitialPose

        logi.log_g("INITIAL STATE: X=%.3f, Y=%.3f, A=%.3f°\n\n", current_x_, current_y_, current_a_); // Логируем начальное состояние.

        std::ifstream file(gcode_file_.c_str()); // Открываем файл G-кода.
        if (!file.is_open())                     // Проверяем, удалось ли открыть файл.
        {
            logi.log_r("Failed to open G-code file: %s\n", gcode_file_.c_str()); // Логируем ошибку открытия файла.
            return;                                                              // Выходим из функции.
        }

        std::string line;                // Переменная для чтения строки из файла.
        int line_number = 0;             // Счетчик номера строки.
        while (std::getline(file, line)) // Читаем файл построчно.
        {
            ++line_number;     // Увеличиваем номер строки.
            line = trim(line); // Удаляем пробелы.
            // Пропускаем пустые строки, % и ПОЛНОСТЬЮ закомментированные строки
            if (line.empty() || line[0] == '%' || line[0] == ';')
                continue; // Пропускаем пустые/закомментированные строки.

            GCodeCommand cmd;                           // Создаем структуру для хранения команды.
            if (parseGCodeLine(line, cmd, line_number)) // Пытаемся разобрать строку G-кода.
            {
                commands_.push_back(cmd);                                        // Если разбор успешен, добавляем команду в вектор.
                logi.log_b("Line %d: Added: %s\n\n", line_number, line.c_str()); // Логируем добавленную команду.
            }
            else // Если разбор не удался.
            {
                logi.log_w("Line %d: Invalid line\n\n", line_number); // Логируем ошибку недействительной строки.
            }
        }
        file.close();                                                 // Закрываем файл.
        logi.log_b("Total parsed commands: %zu\n", commands_.size()); // Логируем общее количество разобранных команд.

        for (const auto &cmd : commands_) // Цикл по всем разобранным командам для их выполнения.
        {
            if (cmd.command == "G0")
                executeG0(cmd); // Выполняем G0.
            else if (cmd.command == "G1")
                executeG1(cmd); // Выполняем G1.
            else if (cmd.command == "G2")
                executeG2(cmd); // Выполняем G2.
            else if (cmd.command == "G4")
                executeG4(cmd); // Выполняем G4.
            else if (cmd.command == "G9")
                executeG9(cmd);            // Выполняем G9.
            else if (cmd.command == "G10") // НОВОЕ УСЛОВИЕ
                executeG10(cmd);           // Выполняем G10.
            else if (cmd.command == "M3")
                executeM3(cmd); // Выполняем M3 (с измененной логикой).
            else if (cmd.command == "M5")
                executeM5(cmd); // Выполняем M5 (с измененной логикой).
            else
                logi.log_w("Unknown command: %s\n", cmd.command.c_str()); // Логируем неизвестную команду.
        } // Конец цикла выполнения.

        // === ИТОГОВЫЙ ОТЧЁТ ===
        std::string separator(60, '=');          // Создаем строку-разделитель.
        logi.log_g("%s\n", separator.c_str());   // Выводим разделитель.
        logi.log_g("FINAL STATE:\n");            // Заголовок итогового состояния.
        logi.log_g(" X = %.3f m\n", current_x_); // Выводим конечную X.
        logi.log_g(" Y = %.3f m\n", current_y_); // Выводим конечную Y.
        logi.log_g(" A = %.3f°\n", current_a_);  // Выводим конечный угол A.
        if (total_time_sec_ >= 60.0f)            // Если общее время больше минуты.
        {
            logi.log_g("TOTAL CALCULATED TIME: (%.1f minutes)\n", total_time_sec_ / 60.0f); // Выводим время в минутах.
        }
        else // Если общее время меньше минуты.
        {
            logi.log_g("TOTAL CALCULATED TIME: %.3f seconds\n", total_time_sec_); // Выводим время в секундах.
        }
        logi.log_g("%s\n", separator.c_str()); // Выводим разделитель.

        logi.log_w("Execution complete. commandArray size: %zu\n", commandArray.size()); // Логируем завершение и размер массива команд.
        if (!commandArray.empty())                                                       // Если массив команд не пуст.
        {
            for (size_t i = 0; i < commandArray.size(); ++i) // Цикл для вывода информации о каждой команде.
            {
                const SCommand &c = commandArray[i];                                           // Получаем ссылку на команду.
                logi.log("    Cmd[%3zu]: mode=%2d, dur=%8.1f, vL=%+7.3f, vR=%+7.3f, "          // Используем printf (по вашему предпочтению) для вывода информации о команде.
                         "delta_angle=%+8.3f, vel_angle=%+8.3f, len=%+7.3f, vel_len=%+7.3f | " // Продолжение вывода параметров движения.
                         "A(%+8.3f,%+8.3f,%+8.3f) -> B(%+8.3f,%+8.3f,%+8.3f)\n",               // Вывод начальной и конечной позы.
                         i, c.mode, c.duration, c.velL, c.velR,                                // Вывод номера, режима, длительности и скоростей колес.
                         c.angle, c.velAngle, c.len, c.velLen,                                 // Вывод угла, угловой скорости, длины и линейной скорости.
                         c.point_A_x, c.point_A_y, c.point_A_a,                                // Вывод начальной позы.
                         c.point_B_x, c.point_B_y, c.point_B_a);                               // Вывод конечной позы.
            }
        }
    } // Конец функции run.

    const std::vector<SCommand> &getCommandArray() const { return commandArray; } // Геттер для получения массива команд.
    float getCurrentX() const { return current_x_; }                              // Геттер для получения текущей X.
    float getCurrentY() const { return current_y_; }                              // Геттер для получения текущей Y.
    float getCurrentA() const { return current_a_; }                              // Геттер для получения текущего угла A.
    float getTotalTimeSec() const { return total_time_sec_; }                     // Геттер для получения общего времени выполнения.
}; // Конец класса GCodeParser.

/*
===============================================================================
                    G-CODE СПРАВКА (для робота с диф. приводом)
===============================================================================

Общие правила:
- Все координаты (X, Y, L) — в метрах
- Все скорости (F, F_L, F_R) — в метрах/секунду
- Углы (A, O) — в градусах, диапазон [-180, +180)
- Скорость по умолчанию: 0.1 м/с
- Максимальная скорость: 0.5 м/с (автоматически ограничивается)
- Расстояние между колёсами: DISTANCE_WHEELS м
- Используйте ЛАТИНСКИЕ буквы (особенно 'O', а не кириллическую 'О')

Измененные команды:
G0 F_L<скорость> F_R<скорость> T<время_сек>
G1 <режим> [F<скорость>]
G2 <режим> [F<скорость>]
G4 P<мс>
M3 T<мс> (Включение устройства, длительность T в мс)
M5 T<мс> (Выключение устройства, длительность T в мс)

Пример:
    GCodeParser parser;
    parser.setInitialPose(1.0, -0.5, 30.0); // Старт из (1.0, -0.5) с углом 30°
    parser.run();

===============================================================================
*/

#endif // GCODEPARSER_H // Конец заголовочного файла.