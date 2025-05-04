#ifndef GCODEPARSER_H
#define GCODEPARSER_H

#include <ros/ros.h>         // Подключение ROS
#include <std_msgs/String.h> // Для публикации сообщений
#include <fstream>           // Для чтения файла
#include <sstream>           // Для разбора строк
#include <vector>            // Для хранения команд
#include <string>            // Для работы со строками
#include <algorithm>         // Для обработки строк (trim)

// Класс для обработки G-code
class GCodeParser
{
    // Структура для хранения команды G-code
    struct GCodeCommand
    {
        std::string command;                                  // Название команды (G0, G1, M3 и т.д.)
        float x, y, z;                                        // Координаты X, Y, Z
        float i, j;                                           // Центр дуги для G2/G3
        float f;                                              // Скорость подачи
        float p;                                              // Длительность паузы для G4
        bool has_x, has_y, has_z, has_i, has_j, has_f, has_p; // Флаги наличия параметров
        std::string comment;                                  // Комментарий к команде
    };

private:
    ros::NodeHandle nh_;                      // NodeHandle для взаимодействия с ROS
    ros::Publisher pub_;                      // Паблишер для топика robot_position
    std::vector<GCodeCommand> commands_;      // Массив для хранения команд
    bool absolute_mode_;                      // Режим координат: true — абсолютный, false — относительный
    float current_x_, current_y_, current_z_; // Текущая позиция робота
    std::string gcode_file_;                  // Путь к файлу G-code
    const float MAX_X_ = 1000.0;              // Максимальная координата X
    const float MAX_Y_ = 750.0;               // Максимальная координата Y

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
    bool parseGCodeLine(const std::string &line, GCodeCommand &cmd)
    {
        std::string code_part = line;                                                              // Часть строки с кодом
        cmd = GCodeCommand();                                                                      // Инициализация структуры
        cmd.has_x = cmd.has_y = cmd.has_z = cmd.has_i = cmd.has_j = cmd.has_f = cmd.has_p = false; // Сброс флагов

        // Поиск комментария (начинается с ';')
        size_t comment_pos = line.find(';'); // Позиция начала комментария
        if (comment_pos != std::string::npos)
        {
            cmd.comment = trim(line.substr(comment_pos + 1)); // Сохранение комментария
            code_part = trim(line.substr(0, comment_pos));    // Код до комментария
        }
        else
        {
            cmd.comment = "";       // Комментарий отсутствует
            code_part = trim(line); // Код — вся строка
        }

        if (code_part.empty())
            return false; // Пропуск пустых строк

        std::stringstream ss(code_part); // Поток для разбора кода
        std::string token;               // Для хранения токена
        ss >> token;                     // Чтение команды (G0, G1 и т.д.)
        if (token.empty() || (token[0] != 'G' && token[0] != 'M'))
            return false;    // Проверка валидности
        cmd.command = token; // Сохранение команды

        while (ss >> token)
        {
            char param = token[0]; // Тип параметра (X, Y, Z и т.д.)
            try
            {
                float value = std::stof(token.substr(1)); // Преобразование в число
                if (param == 'X')
                {
                    if (value < 0 || value > MAX_X_)
                        throw std::invalid_argument("Координата X вне диапазона"); // Проверка
                    cmd.x = value;
                    cmd.has_x = true; // Сохранение X
                }
                else if (param == 'Y')
                {
                    if (value < 0 || value > MAX_Y_)
                        throw std::invalid_argument("Координата Y вне диапазона"); // Проверка
                    cmd.y = value;
                    cmd.has_y = true; // Сохранение Y
                }
                else if (param == 'Z')
                {
                    if (value < 0)
                        throw std::invalid_argument("Отрицательная координата Z"); // Проверка
                    cmd.z = value;
                    cmd.has_z = true; // Сохранение Z
                }
                else if (param == 'I')
                {
                    cmd.i = value;
                    cmd.has_i = true; // Сохранение I
                }
                else if (param == 'J')
                {
                    cmd.j = value;
                    cmd.has_j = true; // Сохранение J
                }
                else if (param == 'F')
                {
                    if (value <= 0)
                        throw std::invalid_argument("Некорректная скорость F"); // Проверка
                    cmd.f = value;
                    cmd.has_f = true; // Сохранение F
                }
                else if (param == 'P')
                {
                    if (value < 0)
                        throw std::invalid_argument("Отрицательная пауза P"); // Проверка
                    cmd.p = value;
                    cmd.has_p = true; // Сохранение P
                }
            }
            catch (const std::exception &e)
            {
                ROS_WARN("Ошибка разбора параметра в строке: %s (%s)", line.c_str(), e.what()); // Лог ошибки
                return false;                                                                   // Возврат при ошибке
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
    float calculateDistance(float new_x, float new_y, float new_z)
    {
        float dx = new_x - current_x_;                 // Разница по X
        float dy = new_y - current_y_;                 // Разница по Y
        float dz = new_z - current_z_;                 // Разница по Z
        return std::sqrt(dx * dx + dy * dy + dz * dz); // Евклидова дистанция
    }

    // Выполнение G0 (быстрое перемещение)
    void executeG0(const GCodeCommand &cmd)
    {
        std::stringstream ss;                         // Для формирования лога
        float new_x = cmd.has_x ? cmd.x : current_x_; // Новая координата X
        float new_y = cmd.has_y ? cmd.y : current_y_; // Новая координата Y
        float new_z = cmd.has_z ? cmd.z : current_z_; // Новая координата Z
        if (absolute_mode_)
        {
            current_x_ = new_x; // Обновление X
            current_y_ = new_y; // Обновление Y
            current_z_ = new_z; // Обновление Z
        }
        else
        {
            current_x_ += cmd.has_x ? cmd.x : 0.0; // Относительное смещение X
            current_y_ += cmd.has_y ? cmd.y : 0.0; // Относительное смещение Y
            current_z_ += cmd.has_z ? cmd.z : 0.0; // Относительное смещение Z
        }
        ss << "G0: Быстрое перемещение к X=" << current_x_ << " Y=" << current_y_ << " Z=" << current_z_;
        if (!cmd.comment.empty())
            ss << " (" << cmd.comment << ")"; // Добавление комментария
        ROS_INFO("%s", ss.str().c_str());     // Вывод лога
        publishMessage(ss.str());             // Публикация
    }

    // Выполнение G1 (линейное перемещение)
    void executeG1(const GCodeCommand &cmd)
    {
        std::stringstream ss;                         // Для формирования лога
        float feed_rate = cmd.has_f ? cmd.f : 100.0;  // Скорость подачи, по умолчанию 100
        float new_x = cmd.has_x ? cmd.x : current_x_; // Новая координата X
        float new_y = cmd.has_y ? cmd.y : current_y_; // Новая координата Y
        float new_z = cmd.has_z ? cmd.z : current_z_; // Новая координата Z
        if (absolute_mode_)
        {
            current_x_ = new_x; // Обновление X
            current_y_ = new_y; // Обновление Y
            current_z_ = new_z; // Обновление Z
        }
        else
        {
            current_x_ += cmd.has_x ? cmd.x : 0.0; // Относительное смещение X
            current_y_ += cmd.has_y ? cmd.y : 0.0; // Относительное смещение Y
            current_z_ += cmd.has_z ? cmd.z : 0.0; // Относительное смещение Z
        }
        float distance = calculateDistance(new_x, new_y, new_z); // Вычисление дистанции
        ss << "G1: Линейное перемещение к X=" << current_x_ << " Y=" << current_y_ << " Z=" << current_z_ << " F=" << feed_rate;
        ss << ", дистанция=" << distance << " мм";
        if (!cmd.comment.empty())
            ss << " (" << cmd.comment << ")"; // Добавление комментария
        ROS_INFO("%s", ss.str().c_str());     // Вывод лога
        publishMessage(ss.str());             // Публикация
    }

    // Выполнение G2 (дуга по часовой стрелке)
    void executeG2(const GCodeCommand &cmd)
    {
        std::stringstream ss;                        // Для формирования лога
        float center_i = cmd.has_i ? cmd.i : 0.0;    // Смещение центра по X
        float center_j = cmd.has_j ? cmd.j : 0.0;    // Смещение центра по Y
        float feed_rate = cmd.has_f ? cmd.f : 100.0; // Скорость подачи
        if (absolute_mode_)
        {
            current_x_ = cmd.has_x ? cmd.x : current_x_; // Обновление X
            current_y_ = cmd.has_y ? cmd.y : current_y_; // Обновление Y
        }
        else
        {
            current_x_ += cmd.has_x ? cmd.x : 0.0; // Относительное смещение X
            current_y_ += cmd.has_y ? cmd.y : 0.0; // Относительное смещение Y
        }
        ss << "G2: Дуга по часовой к X=" << current_x_ << " Y=" << current_y_ << " I=" << center_i << " J=" << center_j << " F=" << feed_rate;
        if (!cmd.comment.empty())
            ss << " (" << cmd.comment << ")"; // Добавление комментария
        ROS_INFO("%s", ss.str().c_str());     // Вывод лога
        publishMessage(ss.str());             // Публикация
    }

    // Выполнение G3 (дуга против часовой стрелки)
    void executeG3(const GCodeCommand &cmd)
    {
        std::stringstream ss;                        // Для формирования лога
        float center_i = cmd.has_i ? cmd.i : 0.0;    // Смещение центра по X
        float center_j = cmd.has_j ? cmd.j : 0.0;    // Смещение центра по Y
        float feed_rate = cmd.has_f ? cmd.f : 100.0; // Скорость подачи
        if (absolute_mode_)
        {
            current_x_ = cmd.has_x ? cmd.x : current_x_; // Обновление X
            current_y_ = cmd.has_y ? cmd.y : current_y_; // Обновление Y
        }
        else
        {
            current_x_ += cmd.has_x ? cmd.x : 0.0; // Относительное смещение X
            current_y_ += cmd.has_y ? cmd.y : 0.0; // Относительное смещение Y
        }
        ss << "G3: Дуга против часовой к X=" << current_x_ << " Y=" << current_y_ << " I=" << center_i << " J=" << center_j << " F=" << feed_rate;
        if (!cmd.comment.empty())
            ss << " (" << cmd.comment << ")"; // Добавление комментария
        ROS_INFO("%s", ss.str().c_str());     // Вывод лога
        publishMessage(ss.str());             // Публикация
    }

    // Выполнение G4 (пауза)
    void executeG4(const GCodeCommand &cmd)
    {
        std::stringstream ss;                     // Для формирования лога
        float pause_ms = cmd.has_p ? cmd.p : 0.0; // Длительность паузы в мс
        ss << "G4: Пауза на " << pause_ms << " мс";
        if (!cmd.comment.empty())
            ss << " (" << cmd.comment << ")";     // Добавление комментария
        ROS_INFO("%s", ss.str().c_str());         // Вывод лога
        ros::Duration(pause_ms / 1000.0).sleep(); // Ожидание
        publishMessage(ss.str());                 // Публикация
    }

    // Выполнение G28 (возврат домой)
    void executeG28(const GCodeCommand &cmd)
    {
        std::stringstream ss; // Для формирования лога
        current_x_ = 0.0;     // Сброс X
        current_y_ = 0.0;     // Сброс Y
        current_z_ = 0.0;     // Сброс Z
        ss << "G28: Возврат в домашнюю позицию (0, 0, 0)";
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
        ss << "G90: Абсолютный режим координат";
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
        ss << "G91: Относительный режим координат";
        if (!cmd.comment.empty())
            ss << " (" << cmd.comment << ")"; // Добавление комментария
        ROS_INFO("%s", ss.str().c_str());     // Вывод лога
        publishMessage(ss.str());             // Публикация
    }

    // Выполнение M3 (включение шпинделя)
    void executeM3(const GCodeCommand &cmd)
    {
        std::stringstream ss; // Для формирования лога
        ss << "M3: Шпиндель включен";
        if (!cmd.comment.empty())
            ss << " (" << cmd.comment << ")"; // Добавление комментария
        ROS_INFO("%s", ss.str().c_str());     // Вывод лога
        publishMessage(ss.str());             // Публикация
    }

    // Выполнение M5 (выключение шпинделя)
    void executeM5(const GCodeCommand &cmd)
    {
        std::stringstream ss; // Для формирования лога
        ss << "M5: Шпиндель выключен";
        if (!cmd.comment.empty())
            ss << " (" << cmd.comment << ")"; // Добавление комментария
        ROS_INFO("%s", ss.str().c_str());     // Вывод лога
        publishMessage(ss.str());             // Публикация
    }

public:
    // Конструктор
    GCodeParser() : nh_(), pub_(nh_.advertise<std_msgs::String>("robot_position", 10)),
                    absolute_mode_(true), current_x_(0.0), current_y_(0.0), current_z_(0.0),
                    gcode_file_("/control_code/my_custom_gcode.txt")
    {
        nh_.param<std::string>("gcode_file", gcode_file_, gcode_file_); // Чтение параметра gcode_file
        ROS_INFO("Используется файл G-code: %s", gcode_file_.c_str());  // Вывод пути к файлу
    }

    // Основной метод для выполнения
    void run()
    {
        std::ifstream file(gcode_file_.c_str()); // Открытие файла
        if (!file.is_open())
        {
            ROS_ERROR("Не удалось открыть файл: %s", gcode_file_.c_str()); // Вывод ошибки
            return;                                                        // Выход
        }

        std::string line; // Для хранения строки
        while (std::getline(file, line))
        {
            line = trim(line); // Удаление пробелов
            if (line.empty() || line[0] == '%')
                continue;     // Пропуск пустых строк и комментариев
            GCodeCommand cmd; // Создание структуры команды
            if (parseGCodeLine(line, cmd))
            {
                commands_.push_back(cmd); // Добавление команды
                std::string log = "Добавлена команда: " + line;
                if (!cmd.comment.empty())
                    log += " (" + cmd.comment + ")"; // Добавление комментария
                ROS_INFO("%s", log.c_str());         // Вывод лога
            }
            else
            {
                ROS_WARN("Невалидная строка: %s", line.c_str()); // Предупреждение
            }
        }
        file.close(); // Закрытие файла

        ros::Rate loop_rate(1); // Частота 1 Гц
        for (const auto &cmd : commands_)
        {
            if (cmd.command == "G0")
                executeG0(cmd); // Выполнение G0
            else if (cmd.command == "G1")
                executeG1(cmd); // Выполнение G1
            else if (cmd.command == "G2")
                executeG2(cmd); // Выполнение G2
            else if (cmd.command == "G3")
                executeG3(cmd); // Выполнение G3
            else if (cmd.command == "G4")
                executeG4(cmd); // Выполнение G4
            else if (cmd.command == "G28")
                executeG28(cmd); // Выполнение G28
            else if (cmd.command == "G90")
                executeG90(cmd); // Выполнение G90
            else if (cmd.command == "G91")
                executeG91(cmd); // Выполнение G91
            else if (cmd.command == "M3")
                executeM3(cmd); // Выполнение M3
            else if (cmd.command == "M5")
                executeM5(cmd); // Выполнение M5
            else
                ROS_WARN("Неизвестная команда: %s (%s)", cmd.command.c_str(), cmd.comment.c_str()); // Предупреждение
            ros::spinOnce();                                                                        // Обработка сообщений
            loop_rate.sleep();                                                                      // Пауза
        }

        ROS_INFO("Все команды выполнены"); // Сообщение об окончании
    }
};

#endif