
#ifndef GCODEPARSER_H
#define GCODEPARSER_H

#include <ros/ros.h>     // Подключаем ROS для взаимодействия с ROS-системой
#include <ros/package.h> // Для получения пути к пакету
// #include <algorithm>         // Для обработки строк (trim)
#include <std_msgs/String.h> // Подключаем тип сообщения String для публикации в топик
#include <sstream>           // Подключаем stringstream для форматирования строк
#include <fstream>           // Подключаем fstream для чтения G-code файла
#include <string>            // Подключаем string для работы со строками
#include <vector>            // Подключаем vector для хранения команд G-code
#include <cmath>             // Подключаем cmath для математических функций (sin, cos, abs)

struct GCodeCommand // Структура для хранения параметров одной команды G-code
{
    std::string command;                                                    // Код команды (например, G0, G1, M3)
    bool has_l, has_a, has_f, has_p, has_r, has_t, has_f_l, has_f_r, has_d; // Флаги наличия параметров
    float l, a, f, p, r, t, f_l, f_r, d;                                    // Параметры: L (длина), A (угол), F (скорость), P (пауза), R (радиус), T (время), F_L (скорость левого колеса), F_R (скорость правого), D (направление)
    std::string comment;                                                    // Комментарий к команде (после ; )
    std::string raw_line;                                                   // Полная строка G-code
    int line_number;                                                        // Номер строки в файле

    GCodeCommand() : has_l(false), has_a(false), has_f(false), has_p(false),       // Конструктор, инициализирует все флаги false
                     has_r(false), has_t(false), has_f_l(false), has_f_r(false),   // Инициализирует флаги для новых параметров
                     has_d(false), l(0.0), a(0.0), f(0.0), p(0.0), r(0.0), t(0.0), // Инициализирует параметры нулями
                     f_l(0.0), f_r(0.0), d(0.0)
    {
    } // Инициализирует новые параметры F_L, F_R, D
};

// Класс для обработки G-code
class GCodeParser
{
private:
    ros::NodeHandle nh_;                      // NodeHandle для взаимодействия с ROS (приватное пространство имён)
    ros::Publisher pub_;                      // Паблишер для топика robot_position
    bool absolute_mode_;                      // Режим координат: true — абсолютный, false — относительный
    float current_x_, current_y_, current_a_; // Текущая позиция и угол
    std::string gcode_file_;                  // Путь к файлу G-code
    float wheel_base_ = 200.0;                // Ширина базы робота (мм), расстояние между колёсами, нужно уточнить
    std::vector<GCodeCommand> commands_;      // Вектор для хранения разобранных команд G-code

    // Удаление пробелов в начале и конце строки
    std::string trim(const std::string &str)
    {
        size_t first = str.find_first_not_of(" \t"); // Поиск первого непробельного символа
        size_t last = str.find_last_not_of(" \t");   // Поиск последнего непробельного символа
        if (first == std::string::npos)
            return "";                              // Если строка пуста, вернуть пустую
        return str.substr(first, last - first + 1); // Вырезать подстроку
    }

    // Разбор строки G-code
    bool parseGCodeLine(const std::string &line, GCodeCommand &cmd, int line_number) // Парсит строку G-code
    {
        std::string trimmed_line = trim(line); // Удаляем пробелы
        if (trimmed_line.empty())
            return false; // Пропускаем пустые строки

        cmd.raw_line = line;           // Сохраняем исходную строку
        cmd.line_number = line_number; // Сохраняем номер строки

        std::stringstream ss(trimmed_line); // Создаём поток для разбора
        std::string token;                  // Токен для чтения

        ss >> token; // Читаем команду
        if (token.empty() || (token[0] != 'G' && token[0] != 'M'))
        {
            ROS_WARN("Line %d: Invalid command token '%s' in command '%s'", line_number, token.c_str(), line.c_str());
            return false; // Проверяем, что это G или M команда
        }
        cmd.command = token; // Сохраняем команду
        ROS_INFO("Line %d: Parsed command '%s'", line_number, token.c_str());

        while (ss >> token) // Читаем параметры
        {
            if (token[0] == ';') // Если комментарий
            {
                std::getline(ss, cmd.comment);   // Считываем комментарий
                cmd.comment = trim(cmd.comment); // Удаляем пробелы
                ROS_INFO("Line %d: Parsed comment '%s'", line_number, cmd.comment.c_str());
                break; // Прерываем разбор
            }
            if (token.length() < 2) // Пропускаем короткие токены
            {
                ROS_WARN("Line %d: Skipping short token '%s' in command '%s'", line_number, token.c_str(), line.c_str());
                continue;
            }
            char param = token[0]; // Первый символ — параметр
            float value;           // Значение параметра
            try
            {
                // Проверяем, является ли токен F_L или F_R
                if (param == 'F' && token.length() >= 3 && token[1] == '_')
                {
                    std::string suffix = token.substr(2, 1);  // Символ после F_
                    std::string number_str = token.substr(3); // Число после F_X
                    if (number_str.empty())
                    {
                        ROS_WARN("Line %d: Empty number after '%s' in token '%s' in command '%s'",
                                 line_number, token.substr(0, 3).c_str(), token.c_str(), line.c_str());
                        continue;
                    }
                    value = std::stof(number_str); // Преобразуем число
                    if (suffix == "L")
                    {
                        cmd.f_l = value;
                        cmd.has_f_l = true;
                        ROS_INFO("Line %d: Parsed F_L=%.2f from token '%s'", line_number, value, token.c_str());
                    }
                    else if (suffix == "R")
                    {
                        cmd.f_r = value;
                        cmd.has_f_r = true;
                        ROS_INFO("Line %d: Parsed F_R=%.2f from token '%s'", line_number, value, token.c_str());
                    }
                    else
                    {
                        ROS_WARN("Line %d: Invalid F parameter format: token '%s', expected F_L or F_R (suffix=%s) in command '%s'",
                                 line_number, token.c_str(), suffix.c_str(), line.c_str());
                        continue;
                    }
                }
                else
                {
                    value = std::stof(token.substr(1)); // Преобразуем число после первого символа
                    if (param == 'F')                   // Скорость подачи
                    {
                        cmd.f = value;
                        cmd.has_f = true;
                        ROS_INFO("Line %d: Parsed F=%.2f from token '%s'", line_number, value, token.c_str());
                    }
                    else if (param == 'L') // Длина пути
                    {
                        cmd.l = value;
                        cmd.has_l = true;
                        ROS_INFO("Line %d: Parsed L=%.2f from token '%s'", line_number, value, token.c_str());
                    }
                    else if (param == 'A') // Угол
                    {
                        cmd.a = value;
                        cmd.has_a = true;
                        ROS_INFO("Line %d: Parsed A=%.2f from token '%s'", line_number, value, token.c_str());
                    }
                    else if (param == 'P') // Пауза
                    {
                        cmd.p = value;
                        cmd.has_p = true;
                        ROS_INFO("Line %d: Parsed P=%.2f from token '%s'", line_number, value, token.c_str());
                    }
                    else if (param == 'R') // Радиус
                    {
                        cmd.r = value;
                        cmd.has_r = true;
                        ROS_INFO("Line %d: Parsed R=%.2f from token '%s'", line_number, value, token.c_str());
                    }
                    else if (param == 'T') // Время
                    {
                        cmd.t = value;
                        cmd.has_t = true;
                        ROS_INFO("Line %d: Parsed T=%.2f from token '%s'", line_number, value, token.c_str());
                    }
                    else if (param == 'D') // Направление
                    {
                        cmd.d = value;
                        cmd.has_d = true;
                        ROS_INFO("Line %d: Parsed D=%.2f from token '%s'", line_number, value, token.c_str());
                    }
                    else
                    {
                        ROS_WARN("Line %d: Unknown parameter '%c' in token '%s' in command '%s'",
                                 line_number, param, token.c_str(), line.c_str());
                    }
                }
            }
            catch (const std::exception &e)
            {
                ROS_WARN("Line %d: Failed to parse value for token '%s' in command '%s': %s",
                         line_number, token.c_str(), line.c_str(), e.what());
                continue; // Пропускаем при ошибке
            }
        }
        return true; // Успешный разбор
    }

    // Публикация сообщения в топик
    void publishMessage(const std::string &message)
    {
        std_msgs::String msg; // Создание сообщения
        msg.data = message;   // Заполнение сообщения
        pub_.publish(msg);    // Публикация
    }

    // Вычисление дистанции между текущей и новой позицией
    float calculateDistance(float new_x, float new_y)
    {
        float dx = new_x - current_x_;       // Разница по X
        float dy = new_y - current_y_;       // Разница по Y
        return std::sqrt(dx * dx + dy * dy); // Евклидова дистанция в 2D
    }
    // Добавление паузы в массив исполнения
    void executePause(float time_)
    {
        SCommand temp;
        temp.mode = 0;
        temp.duration = time_;
        temp.velL = 0.0;
        temp.velR = 0.0;
        commandArray.push_back(temp);
        ROS_INFO("    executePause %u msec", temp.duration); // Вывод лога
    }

    // // Выполнение G1 (линейное перемещение или поворот)
    // void executeG1(const GCodeCommand &cmd)
    // {
    //     executePause(1000);                          // Добавляем паузу перед любой операцией для стабилизации робота в ключевых точках.
    //     std::stringstream ss;                        // Для формирования лога
    //     float feed_rate = cmd.has_f ? cmd.f : 100.0; // Скорость подачи, по умолчанию 100

    //     // Проверка, является ли команда угловой (только A)
    //     if (cmd.has_a && !cmd.has_x && !cmd.has_y)
    //     {
    //         float new_a = cmd.a; // Новый угол A
    //         if (absolute_mode_)
    //         {
    //             current_a_ = new_a; // Обновление A
    //         }
    //         else
    //         {
    //             current_a_ += new_a; // Относительное смещение A
    //         }
    //         ss << "G1: Rotate to A= " << current_a_ << " F= " << feed_rate << " deg/s";
    //         if (!cmd.comment.empty())
    //             ss << " (" << cmd.comment << ")"; // Добавление комментария
    //         ROS_INFO("%s", ss.str().c_str());     // Вывод лога
    //         publishMessage(ss.str());             // Публикация

    //         SCommand temp;
    //         temp.mode = 2;
    //         temp.angle = cmd.a;
    //         temp.velAngle = cmd.f;
    //         commandArray.push_back(temp);
    //         ROS_INFO("    executeG1 mode= %i angle= %f velAngle=%f ", temp.mode, temp.angle, temp.velAngle); // Вывод лога

    //         return;
    //     }

    //     // Линейное перемещение
    //     float new_x = cmd.has_x ? cmd.x : current_x_; // Новая координата X
    //     float new_y = cmd.has_y ? cmd.y : current_y_; // Новая координата Y
    //     float new_a = cmd.has_a ? cmd.a : current_a_; // Новый угол A
    //     if (absolute_mode_)
    //     {
    //         current_x_ = new_x; // Обновление X
    //         current_y_ = new_y; // Обновление Y
    //         current_a_ = new_a; // Обновление A
    //     }
    //     else
    //     {
    //         current_x_ += cmd.has_x ? cmd.x : 0.0; // Относительное смещение X
    //         current_y_ += cmd.has_y ? cmd.y : 0.0; // Относительное смещение Y
    //         current_a_ += cmd.has_a ? cmd.a : 0.0; // Относительное смещение A
    //     }
    //     float distance = calculateDistance(new_x, new_y); // Вычисление дистанции
    //     ss << "G1: Linear move to X= " << current_x_ << " Y= " << current_y_ << " A= " << current_a_ << " F= " << feed_rate;
    //     ss << ", distance= " << distance << " mm";
    //     if (!cmd.comment.empty())
    //         ss << " (" << cmd.comment << ")"; // Добавление комментария
    //     ROS_INFO("%s", ss.str().c_str());     // Вывод лога
    //     publishMessage(ss.str());             // Публикация
    // }
    //-------------------------------------------------------------------------------------------------------------------------------------

    void executeG0(const GCodeCommand &cmd) // Выполняет G0: движение с F_L, F_R, T
    {
        std::stringstream ss; // Поток для лога

        if (!cmd.has_f_l || !cmd.has_f_r || !cmd.has_t) // Проверяем наличие параметров
        {
            ROS_ERROR("Line %d: Invalid G0 parameters: must specify F_L, F_R, T in command '%s'", cmd.line_number, cmd.raw_line.c_str()); // Ошибка с номером строки и её содержимым
            return;
        }
        float f_l = cmd.f_l; // Скорость левого колеса
        float f_r = cmd.f_r; // Скорость правого колеса
        float time = cmd.t;  // Время движения

        if (time <= 0.0) // Проверяем время
        {
            ROS_ERROR("Line %d: Invalid G0 time: T=%.2f in command '%s'", cmd.line_number, time, cmd.raw_line.c_str());
            return;
        }
        if (f_l < 0.0 || f_r < 0.0) // Проверяем скорости
        {
            ROS_ERROR("Line %d: Invalid G0 wheel speeds: F_L=%.2f, F_R=%.2f in command '%s'", cmd.line_number, f_l, f_r, cmd.raw_line.c_str());
            return;
        }

        // Кинематика
        float v = (f_l + f_r) / 2.0;             // Линейная скорость
        float omega = (f_r - f_l) / wheel_base_; // Угловая скорость
        std::string direction = (omega < 0.0) ? "clockwise" : (omega > 0.0) ? "counterclockwise"
                                                                            : "linear"; // Определяем направление

        if (std::abs(omega) < 0.001) // Прямое движение
        {
            float length = v * time;                                                 // Длина пути
            current_x_ += length * std::cos(current_a_ * M_PI / 180.0);              // Обновляем x
            current_y_ += length * std::sin(current_a_ * M_PI / 180.0);              // Обновляем y
            ss << "G0: Linear move, F_L=" << f_l << " F_R=" << f_r << " T=" << time; // Формируем сообщение
        }
        else // Движение по дуге
        {
            float radius = v / std::abs(omega);                                                                                         // Радиус дуги
            float theta = omega * time;                                                                                                 // Угол дуги (рад)
            float center_x = current_x_ + radius * std::cos((current_a_ + (omega > 0 ? 90.0 : -90.0)) * M_PI / 180.0);                  // x центра дуги
            float center_y = current_y_ + radius * std::sin((current_a_ + (omega > 0 ? 90.0 : -90.0)) * M_PI / 180.0);                  // y центра дуги
            current_x_ = center_x + radius * std::cos((current_a_ + (omega > 0 ? 90.0 : -90.0) + theta * 180.0 / M_PI) * M_PI / 180.0); // Обновляем x
            current_y_ = center_y + radius * std::sin((current_a_ + (omega > 0 ? 90.0 : -90.0) + theta * 180.0 / M_PI) * M_PI / 180.0); // Обновляем y
            current_a_ += theta * 180.0 / M_PI;                                                                                         // Обновляем угол
            ss << "G0: Arc move, F_L=" << f_l << " F_R=" << f_r << " T=" << time << " R=" << radius << " D=" << direction;              // Формируем сообщение
        }

        if (!cmd.comment.empty())
            ss << " (" << cmd.comment << ")";                                                                         // Добавляем комментарий
        ROS_INFO("Line %d: %s", cmd.line_number, ss.str().c_str());                                                   // Логируем с номером строки
        ROS_INFO("Line %d: End position: X=%.2f Y=%.2f A=%.2f", cmd.line_number, current_x_, current_y_, current_a_); // Логируем позицию
        publishMessage(ss.str());                                                                                     // Публикуем

        SCommand command;
        command.mode = 0;         // Задаем режим
        command.duration = cmd.t; // Задаем длительность (время)
        command.velL = cmd.f_l;   // Скорость на левом
        command.velR = cmd.f_r;   // Скорость на правом
        commandArray.push_back(command);
        ROS_INFO("    executeG0 mode= %i duration= %f velL=%f velR=%f ", command.mode, command.duration, command.velL, command.velR); // Вывод лога
    }

    void executeG1(const GCodeCommand &cmd) // Выполняет G1: поворот на месте с A, F
    {
        std::stringstream ss; // Поток для лога

        if (!cmd.has_a) // Проверяем наличие угла
        {
            ROS_ERROR("Line %d: Invalid G1 parameters: must specify A in command '%s'", cmd.line_number, cmd.raw_line.c_str()); // Ошибка
            return;
        }
        float feed_rate = cmd.has_f ? cmd.f : 100.0; // Скорость поворота (градусы/с), по умолчанию 100
        if (feed_rate <= 0.0)                        // Проверяем скорость
        {
            ROS_ERROR("Line %d: Invalid G1 feed rate: F=%.2f in command '%s'", cmd.line_number, feed_rate, cmd.raw_line.c_str());
            return;
        }
        float angle = cmd.a;                      // Угол поворота
        float time = std::abs(angle) / feed_rate; // Время поворота
        if (absolute_mode_)                       // Абсолютный режим
        {
            current_a_ = angle; // Устанавливаем угол
        }
        else // Относительный режим
        {
            current_a_ += angle; // Добавляем угол
        }
        ss << "G1: Rotate to A=" << current_a_ << " F=" << feed_rate << " T=" << time; // Формируем сообщение
        if (!cmd.comment.empty())
            ss << " (" << cmd.comment << ")";                       // Добавляем комментарий
        ROS_INFO("Line %d: %s", cmd.line_number, ss.str().c_str()); // Логируем с номером строки
        publishMessage(ss.str());                                   // Публикуем

        SCommand command;
        command.mode = 1;         // Задаем режим
        command.angle = cmd.a;    // Угол поворота
        command.velAngle = cmd.f; // Скорость поворота
        commandArray.push_back(command);
        ROS_INFO("    executeG1 mode= %i angle= %f velAngle=%f ", command.mode, command.angle, command.velAngle); // Вывод лога
    }

    void executeG2(const GCodeCommand &cmd) // Выполняет G2: линейное движение с L, F
    {
        std::stringstream ss;                        // Поток для лога
        float length = cmd.has_l ? cmd.l : 0.0;      // Длина пути
        float feed_rate = cmd.has_f ? cmd.f : 100.0; // Скорость подачи

        if (!cmd.has_l || !cmd.has_f) // Проверяем параметры
        {
            ROS_ERROR("Line %d: Invalid G2 parameters: must specify L and F in command '%s'", cmd.line_number, cmd.raw_line.c_str()); // Ошибка
            return;
        }
        if (length <= 0.0) // Проверяем длину
        {
            ROS_ERROR("Line %d: Invalid G2 length: L=%.2f in command '%s'", cmd.line_number, length, cmd.raw_line.c_str()); // Ошибка
            return;
        }
        if (feed_rate <= 0.0) // Проверяем скорость
        {
            ROS_ERROR("Line %d: Invalid G2 feed rate: F=%.2f in command '%s'", cmd.line_number, feed_rate, cmd.raw_line.c_str()); // Ошибка
            return;
        }

        float time = length / feed_rate;                            // Вычисляем время
        current_x_ += length * std::cos(current_a_ * M_PI / 180.0); // Обновляем x
        current_y_ += length * std::sin(current_a_ * M_PI / 180.0); // Обновляем y

        ss << "G2: Linear move, L=" << length << " F=" << feed_rate << " T=" << time; // Формируем сообщение
        if (!cmd.comment.empty())
            ss << " (" << cmd.comment << ")";                                                                         // Добавляем комментарий
        ROS_INFO("Line %d: %s", cmd.line_number, ss.str().c_str());                                                   // Логируем с номером строки
        ROS_INFO("Line %d: End position: X=%.2f Y=%.2f A=%.2f", cmd.line_number, current_x_, current_y_, current_a_); // Логируем позицию
        publishMessage(ss.str());

        SCommand command;
        command.mode = 2;       // Задаем режим
        command.len = cmd.l;    // Длинна вектора движения
        command.velLen = cmd.f; // Скорость движения
        commandArray.push_back(command);
        ROS_INFO("    executeG2 mode= %i len= %f velLen=%f ", command.mode, command.len, command.velLen); // Вывод лога                                                                   // Публикуем
    }

    void executeG3(const GCodeCommand &cmd) // Выполняет G3: движение по дуге с R, F, D
    {
        std::stringstream ss;                        // Поток для лога
        float radius = cmd.has_r ? cmd.r : 100.0;    // Радиус дуги
        float feed_rate = cmd.has_f ? cmd.f : 100.0; // Скорость подачи
        float direction = cmd.has_d ? cmd.d : 0.0;   // Направление (0 — против часовой, 1 — по часовой)
        float time = cmd.has_t ? cmd.t : 0.0;        // Время (опционально)

        if (!cmd.has_r || !cmd.has_f || !cmd.has_d) // Проверяем параметры
        {
            ROS_ERROR("Line %d: Invalid G3 parameters: must specify R, F, D in command '%s'", cmd.line_number, cmd.raw_line.c_str()); // Ошибка
            return;
        }
        if (radius <= 0.0) // Проверяем радиус
        {
            ROS_ERROR("Line %d: Invalid G3 radius: R=%.2f in command '%s'", cmd.line_number, radius, cmd.raw_line.c_str()); // Ошибка
            return;
        }
        if (feed_rate <= 0.0) // Проверяем скорость
        {
            ROS_ERROR("Line %d: Invalid G3 feed rate: F=%.2f in command '%s'", cmd.line_number, feed_rate, cmd.raw_line.c_str()); // Ошибка
            return;
        }
        if (direction != 0.0 && direction != 1.0) // Проверяем направление
        {
            ROS_ERROR("Line %d: Invalid G3 direction: D=%.2f, must be 0 or 1 in command '%s'", cmd.line_number, direction, cmd.raw_line.c_str()); // Ошибка
            return;
        }
        if (cmd.has_t && time <= 0.0) // Проверяем время
        {
            ROS_ERROR("Line %d: Invalid G3 time: T=%.2f in command '%s'", cmd.line_number, time, cmd.raw_line.c_str()); // Ошибка
            return;
        }

        float omega = feed_rate / radius; // Угловая скорость
        if (direction == 1.0)
            omega = -omega;                                                                                                         // По часовой — отрицательная ω
        float theta = cmd.has_t ? omega * time : 2.0 * M_PI;                                                                        // Угол дуги (полный круг, если T не указано)
        time = cmd.has_t ? time : std::abs(theta / omega);                                                                          // Вычисляем время, если не указано
        float center_x = current_x_ + radius * std::cos((current_a_ + (omega > 0 ? 90.0 : -90.0)) * M_PI / 180.0);                  // x центра дуги
        float center_y = current_y_ + radius * std::sin((current_a_ + (omega > 0 ? 90.0 : -90.0)) * M_PI / 180.0);                  // y центра дуги
        current_x_ = center_x + radius * std::cos((current_a_ + (omega > 0 ? 90.0 : -90.0) + theta * 180.0 / M_PI) * M_PI / 180.0); // Обновляем x
        current_y_ = center_y + radius * std::sin((current_a_ + (omega > 0 ? 90.0 : -90.0) + theta * 180.0 / M_PI) * M_PI / 180.0); // Обновляем y
        current_a_ += theta * 180.0 / M_PI;                                                                                         // Обновляем угол

        ss << "G3: Arc move, R=" << radius << " F=" << feed_rate << " T=" << time << " D=" << (direction == 1.0 ? "clockwise" : "counterclockwise"); // Формируем сообщение
        if (!cmd.comment.empty())
            ss << " (" << cmd.comment << ")";                                                                         // Добавляем комментарий
        ROS_INFO("Line %d: %s", cmd.line_number, ss.str().c_str());                                                   // Логируем с номером строки
        ROS_INFO("Line %d: End position: X=%.2f Y=%.2f A=%.2f", cmd.line_number, current_x_, current_y_, current_a_); // Логируем позицию
        publishMessage(ss.str());                                                                                     // Публикуем
    }

    // Выполнение G4 (пауза)
    void executeG4(const GCodeCommand &cmd)
    {
        std::stringstream ss;                     // Для формирования лога
        float pause_ms = cmd.has_p ? cmd.p : 0.0; // Длительность паузы в мс
        ss << "G4: Pause for " << pause_ms << " ms";
        if (!cmd.comment.empty())
            ss << " (" << cmd.comment << ")";     // Добавление комментария
        ROS_INFO("%s", ss.str().c_str());         // Вывод лога
        // ros::Duration(pause_ms / 1000.0).sleep(); // Ожидание
        publishMessage(ss.str());                 // Публикация

        executePause(cmd.p); // Добавляем паузу перед любой операцией для стабилизации робота в ключевых точках.
    }

    // // Выполнение G5 (Вращение колесами  определенное время по часовой)
    // void executeG5(const GCodeCommand &cmd)
    // {
    //     std::stringstream ss;                     // Для формирования лога
    //     float pause_ms = cmd.has_p ? cmd.p : 0.0; // Длительность паузы в мс
    //     ss << "G5: Rotate for " << cmd.f << " vel " << pause_ms << " ms";
    //     if (!cmd.comment.empty())
    //         ss << " (" << cmd.comment << ")"; // Добавление комментария
    //     ROS_INFO("%s", ss.str().c_str());     // Вывод лога
    //     // ros::Duration(pause_ms / 1000.0).sleep(); // Ожидание
    //     // publishMessage(ss.str());                 // Публикация

    //     SCommand temp;
    //     temp.mode = 1;
    //     temp.velL = cmd.f;
    //     temp.velR = -cmd.f;
    //     temp.duration = cmd.p;
    //     commandArray.push_back(temp);
    //     ROS_INFO("    executeG5 mode= %i velL= %f velR= %f duration=%f ", temp.mode, temp.velL, temp.velR, temp.duration); // Вывод лога
    // }
    // // Выполнение G6 (Вращение колесами  определенное время против часовой)
    // void executeG6(const GCodeCommand &cmd)
    // {
    //     std::stringstream ss;                     // Для формирования лога
    //     float pause_ms = cmd.has_p ? cmd.p : 0.0; // Длительность паузы в мс
    //     ss << "G6: Rotate for " << cmd.f << " vel " << pause_ms << " ms";
    //     if (!cmd.comment.empty())
    //         ss << " (" << cmd.comment << ")"; // Добавление комментария
    //     ROS_INFO("%s", ss.str().c_str());     // Вывод лога
    //     // ros::Duration(pause_ms / 1000.0).sleep(); // Ожидание
    //     // publishMessage(ss.str());                 // Публикация

    //     SCommand temp;
    //     temp.mode = 1;
    //     temp.velL = -cmd.f;
    //     temp.velR = cmd.f;
    //     temp.duration = cmd.p;
    //     commandArray.push_back(temp);
    //     ROS_INFO("    executeG6 mode= %i velL= %f velR= %f duration=%f ", temp.mode, temp.velL, temp.velR, temp.duration); // Вывод лога
    // }
    // // Выполнение G7 (Движение вперед определенное время)
    // void executeG7(const GCodeCommand &cmd)
    // {
    //     std::stringstream ss;                     // Для формирования лога
    //     float pause_ms = cmd.has_p ? cmd.p : 0.0; // Длительность паузы в мс
    //     ss << "G7: Drive for " << cmd.f << " vel " << pause_ms << " ms";
    //     if (!cmd.comment.empty())
    //         ss << " (" << cmd.comment << ")"; // Добавление комментария
    //     ROS_INFO("%s", ss.str().c_str());     // Вывод лога
    //     // ros::Duration(pause_ms / 1000.0).sleep(); // Ожидание
    //     // publishMessage(ss.str());                 // Публикация

    //     SCommand temp;
    //     temp.mode = 1;
    //     temp.velL = cmd.f;
    //     temp.velR = cmd.f;
    //     temp.duration = cmd.p;
    //     commandArray.push_back(temp);
    //     ROS_INFO("    executeG7 mode= %i velL= %f velR= %f duration=%f ", temp.mode, temp.velL, temp.velR, temp.duration); // Вывод лога
    // }
    // // Выполнение G8 (Движение назад определенное время)
    // void executeG8(const GCodeCommand &cmd)
    // {
    //     std::stringstream ss;                     // Для формирования лога
    //     float pause_ms = cmd.has_p ? cmd.p : 0.0; // Длительность паузы в мс
    //     ss << "G8: Drive for " << cmd.f << " vel " << pause_ms << " ms";
    //     if (!cmd.comment.empty())
    //         ss << " (" << cmd.comment << ")"; // Добавление комментария
    //     ROS_INFO("%s", ss.str().c_str());     // Вывод лога
    //     // ros::Duration(pause_ms / 1000.0).sleep(); // Ожидание
    //     // publishMessage(ss.str());                 // Публикация

    //     SCommand temp;
    //     temp.mode = 1;
    //     temp.velL = -cmd.f;
    //     temp.velR = -cmd.f;
    //     temp.duration = cmd.p;
    //     commandArray.push_back(temp);
    //     ROS_INFO("    executeG8 mode= %i velL= %f velR= %f duration=%f ", temp.mode, temp.velL, temp.velR, temp.duration); // Вывод лога
    // }

    // Выполнение G9 (Переход в начало. Зацикливание программы)
    void executeG9(const GCodeCommand &cmd)
    {
        std::stringstream ss;                     // Для формирования лога
        float pause_ms = cmd.has_p ? cmd.p : 0.0; // Длительность паузы в мс
        ss << "G9: Cicle...";
        if (!cmd.comment.empty())
            ss << " (" << cmd.comment << ")"; // Добавление комментария
        ROS_INFO("%s", ss.str().c_str());     // Вывод лога

        SCommand temp;
        temp.mode = 9;
        commandArray.push_back(temp);
        ROS_INFO("    executeG9 mode= %i ", temp.mode); // Вывод лога
    }

    // Выполнение G28 (возврат домой)
    void executeG28(const GCodeCommand &cmd)
    {
        std::stringstream ss; // Для формирования лога
        current_x_ = 0.0;     // Сброс X
        current_y_ = 0.0;     // Сброс Y
        current_a_ = 0.0;     // Сброс A
        ss << "G28: Return to home position (0, 0)";
        if (!cmd.comment.empty())
            ss << " (" << cmd.comment << ")"; // Добавление комментария
        ROS_INFO("%s", ss.str().c_str());     // Вывод лога
        publishMessage(ss.str());             // Публикация
    }

    // Выполнение G90 (абсолютный режим)
    void executeG90(const GCodeCommand &cmd)
    {
        std::stringstream ss;  // Для формирования лога
        absolute_mode_ = true; // Установка абсолютного режима
        ss << "G90: Absolute coordinate mode";
        if (!cmd.comment.empty())
            ss << " (" << cmd.comment << ")"; // Добавление комментария
        ROS_INFO("%s", ss.str().c_str());     // Вывод лога
        publishMessage(ss.str());             // Публикация
    }

    // Выполнение G91 (относительный режим)
    void executeG91(const GCodeCommand &cmd)
    {
        std::stringstream ss;   // Для формирования лога
        absolute_mode_ = false; // Установка относительного режима
        ss << "G91: Relative coordinate mode";
        if (!cmd.comment.empty())
            ss << " (" << cmd.comment << ")"; // Добавление комментария
        ROS_INFO("%s", ss.str().c_str());     // Вывод лога
        publishMessage(ss.str());             // Публикация
    }

    // Выполнение M3 (включение модуля печати)
    void executeM3(const GCodeCommand &cmd)
    {
        std::stringstream ss; // Для формирования лога
        ss << "M3: Turn on print module";
        if (!cmd.comment.empty())
            ss << " (" << cmd.comment << ")"; // Добавление комментария
        ROS_INFO("%s", ss.str().c_str());     // Вывод лога
        publishMessage(ss.str());             // Публикация
    }

    // Выполнение M5 (выключение модуля печати)
    void executeM5(const GCodeCommand &cmd)
    {
        std::stringstream ss; // Для формирования лога
        ss << "M5: Turn off print module";
        if (!cmd.comment.empty())
            ss << " (" << cmd.comment << ")"; // Добавление комментария
        ROS_INFO("%s", ss.str().c_str());     // Вывод лога
        publishMessage(ss.str());             // Публикация
    }

public:
    // Конструктор
    GCodeParser() : nh_("~"), pub_(nh_.advertise<std_msgs::String>("robot_position", 10)),
                    absolute_mode_(true), current_x_(0.0), current_y_(0.0), current_a_(0.0),
                    gcode_file_("gcode/my_custom_gcode.gcode")
    {
        pub_ = nh_.advertise<std_msgs::String>("robot_position", 10);                                // Инициализируем Publisher для топика /robot_position
        nh_.param<std::string>("gcode_file", gcode_file_, "$(find pb)/gcode/my_custom_gcode.gcode"); // Читаем путь к G-code файлу
        ROS_INFO("GCodeParser initialized with file: %s", gcode_file_.c_str());                      // Логируем инициализацию

        // Чтение параметра gcode_file
        if (nh_.getParam("gcode_file", gcode_file_))
        {
            ROS_INFO("Successfully read parameter 'gcode_file': %s", gcode_file_.c_str()); // Успешное чтение
        }
        else
        {
            ROS_WARN("Failed to read parameter 'gcode_file', using default: %s", gcode_file_.c_str()); // Предупреждение
        }
    }

    // Основной метод для выполнения
    void run() // Основной метод для чтения и выполнения G-code
    {
        std::ifstream file(gcode_file_.c_str()); // Открываем файл G-code
        if (!file.is_open())                     // Проверяем, удалось ли открыть файл
        {
            ROS_ERROR("Failed to open file: %s", gcode_file_.c_str()); // Логируем ошибку
            return;
        }

        std::string line;                // Переменная для хранения строки файла
        int line_number = 0;             // Счётчик строк
        while (std::getline(file, line)) // Читаем построчно
        {
            ++line_number;     // Увеличиваем номер строки
            line = trim(line); // Удаляем пробелы
            if (line.empty() || line[0] == '%')
                continue;                               // Пропускаем пустые строки и %
            GCodeCommand cmd;                           // Создаём структуру команды
            if (parseGCodeLine(line, cmd, line_number)) // Парсим строку с номером
            {
                commands_.push_back(cmd);                   // Добавляем команду
                std::string log = "Added command: " + line; // Формируем лог
                if (!cmd.comment.empty())
                    log += " (" + cmd.comment + ")";               // Добавляем комментарий
                ROS_INFO("Line %d: %s", line_number, log.c_str()); // Логируем с номером строки
            }
            else
            {
                ROS_WARN("Line %d: Invalid line: %s", line_number, line.c_str()); // Логируем ошибку
            }
        }
        file.close(); // Закрываем файл

        ROS_INFO("Parsed G-code commands:");          // Логируем начало вывода команд
        for (size_t i = 0; i < commands_.size(); ++i) // Перебираем все команды
        {
            const GCodeCommand &cmd = commands_[i];       // Текущая команда
            std::stringstream ss;                         // Поток для лога
            ss << "Command " << i << ": " << cmd.command; // Формируем начало сообщения
            if (cmd.has_l)
                ss << " L=" << cmd.l; // Добавляем L длинна пути
            if (cmd.has_a)
                ss << " A=" << cmd.a; // Добавляем A угол поворота
            if (cmd.has_f)
                ss << " F=" << cmd.f; // Добавляем F скорость
            if (cmd.has_p)
                ss << " P=" << cmd.p; // Добавляем P время паузы
            if (cmd.has_r)
                ss << " R=" << cmd.r; // Добавляем R радиус поворота
            if (cmd.has_t)
                ss << " T=" << cmd.t; // Добавляем T длительность
            if (cmd.has_f_l)
                ss << " F_L=" << cmd.f_l; // Добавляем F_L скорость левого
            if (cmd.has_f_r)
                ss << " F_R=" << cmd.f_r; // Добавляем F_R правого скорость
            if (cmd.has_d)
                ss << " D=" << cmd.d; // Добавляем D направление по часовой 1 против 0
            if (!cmd.comment.empty())
                ss << " ; " << cmd.comment;   // Добавляем комментарий
            ROS_INFO("%s", ss.str().c_str()); // Логируем команду
        }
        ROS_INFO("Total commands: %zu", commands_.size()); // Логируем общее число команд

        ros::Rate loop_rate(3);           // Устанавливаем частоту цикла (3 Гц)
        for (const auto &cmd : commands_) // Перебираем команды для выполнения
        {
            if (cmd.command == "G0")
                executeG0(cmd); // Выполняем G0
            else if (cmd.command == "G1")
                executeG1(cmd); // Выполняем G1
            else if (cmd.command == "G2")
                executeG2(cmd); // Выполняем G2
            else if (cmd.command == "G3")
                executeG3(cmd); // Выполняем G3
            else if (cmd.command == "G4")
                executeG4(cmd); // Выполняем G4
            else if (cmd.command == "G9")
                executeG9(cmd); // Выполняем G9
            else if (cmd.command == "G28")
                executeG28(cmd); // Выполняем G28
            else if (cmd.command == "G90")
                executeG90(cmd); // Выполняем G90
            else if (cmd.command == "G91")
                executeG91(cmd); // Выполняем G91
            else if (cmd.command == "M3")
                executeM3(cmd); // Выполняем M3
            else if (cmd.command == "M5")
                executeM5(cmd); // Выполняем M5
            else
                ROS_WARN("Unknown command: %s (%s)", cmd.command.c_str(), cmd.comment.c_str()); // Логируем неизвестную команду
            ros::spinOnce();                                                                    // Обрабатываем ROS-сообщения
            loop_rate.sleep();                                                                  // Задержка для соблюдения частоты
        }

        ROS_INFO("All commands executed"); // Логируем завершение

        // Вывод массива commands_ в консоль через ROS_INFO
        ROS_INFO(" \n commandArray commands:");
        for (size_t i = 0; i < commandArray.size(); ++i)
        {
            const SCommand &cmd = commandArray[i];
            std::stringstream ss;
            ss << "Command " << i << "= ";
            ss << " mode= " << cmd.mode;
            ss << " duration= " << cmd.duration;
            ss << " velL= " << cmd.velL;
            ss << " velR= " << cmd.velR;

            ss << " angle= " << cmd.angle;
            ss << " velAngle= " << cmd.velAngle;

            ss << " len= " << cmd.len;
            ss << " velLen= " << cmd.velLen;
            ROS_INFO("%s", ss.str().c_str()); // Вывод в одну строку
        }
        ROS_INFO("Total commandArray: %zu \n", commandArray.size()); // Общее количество команд

        if (commandArray.empty())
            ROS_WARN(" <commandArray> NULL!");
        else
            ROS_INFO("Count <commandArray>: %zu", commandArray.size());
    }
};

#endif

/*
G0: Задаём скорости левого и правого колёс (F_L, F_R, мм/с) и время движения (T, с). Робот движется (прямо или по дуге) в течение времени T с заданными скоростями.
G1: Задаём угол поворота A (градусы) и скорость поворота F (градусы/с). Робот вращается на месте на угол A с направлением,
    определяемым знаком A (положительный — против часовой стрелки, отрицательный — по часовой).
G2: Задаём длину пути L (мм) и скорость подачи F (мм/с). Робот движется по прямой на расстояние L с заданной скоростью, время вычисляется как T = L/F.
G3: Задаём радиус дуги R (мм), скорость подачи F (мм/с) и направление D (0 — против часовой, 1 — по часовой). Робот движется по дуге радиусом R с линейной скоростью F, время T опционально (по умолчанию — полный круг).

Команда G0:Параметры: F_L, F_R, T.
Кинематика:Линейная скорость: v = (F_L + F_R) / 2.
Угловая скорость: ω = (F_R - F_L) / L, где L — ширина базы (200 мм).
Прямое движение (ω ≈ 0): длина пути L = v * T.
Дуга (ω ≠ 0): радиус R = v / |ω|, угол θ = ω * T.

Команда G1:Параметры: A, F (опционально).
Поворот на месте: ω = F * π / 180, время T = |A| / F, направление по знаку A.

Команда G2:Параметры: L, F.
Время: T = L / F.
Движение: (x = current_x_ + L * cos(current_a_), y = current_y_ + L * sin(current_a_)).

Команда G3:Параметры: R, F, D, T (опционально).
Угловая скорость: ω = F / R (положительная для D=0, отрицательная для D=1).
Угол: θ = ω * T или 2π (полный круг, если T не указан).

*/