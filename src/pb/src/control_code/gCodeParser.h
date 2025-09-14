#ifndef GCODEPARSER_H
#define GCODEPARSER_H

#include <ros/ros.h>         // Подключение ROS
#include <ros/package.h>     // Для получения пути к пакету
#include <std_msgs/String.h> // Для публикации сообщений
#include <fstream>           // Для чтения файла
#include <sstream>           // Для разбора строк
#include <vector>            // Для хранения команд
#include <string>            // Для работы со строками
#include <algorithm>         // Для обработки строк (trim)
#include <cmath>             // Для вычисления дистанции

// Структура для хранения команды G-code
struct GCodeCommand
{
    std::string command;                                  // Название команды (G0, G1, M3 и т.д.)
    float x, y;                                           // Координаты X, Y
    float a;                                              // Угол A
    float i, j;                                           // Центр дуги для G2/G3
    float f;                                              // Скорость подачи (линейная или угловая)
    float p;                                              // Длительность паузы для G4
    bool has_x, has_y, has_a, has_i, has_j, has_f, has_p; // Флаги наличия параметров
    std::string comment;                                  // Комментарий к команде
};

// Класс для обработки G-code
class GCodeParser
{
private:
    ros::NodeHandle nh_;                      // NodeHandle для взаимодействия с ROS (приватное пространство имён)
    ros::Publisher pub_;                      // Паблишер для топика robot_position
    std::vector<GCodeCommand> commands_;      // Массив для хранения команд
    bool absolute_mode_;                      // Режим координат: true — абсолютный, false — относительный
    float current_x_, current_y_, current_a_; // Текущая позиция и угол
    std::string gcode_file_;                  // Путь к файлу G-code
    const float MAX_X_ = 1000.0;              // Максимальная координата X (вверх)
    const float MAX_Y_ = 750.0;               // Максимальная координата Y (влево)
    const float MAX_A_ = 180.0;               // Максимальный угол A (в градусах)

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
        cmd.has_x = cmd.has_y = cmd.has_a = cmd.has_i = cmd.has_j = cmd.has_f = cmd.has_p = false; // Сброс флагов

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
        ss >> token;                     // Чтение команды (G0, G1, M3 и т.д.)
        if (token.empty() || (token[0] != 'G' && token[0] != 'M'))
        {
            ROS_WARN("Invalid command in line: %s", line.c_str()); // Предупреждение
            return false;
        }
        cmd.command = token; // Сохранение команды

        while (ss >> token)
        {
            char param = token[0]; // Тип параметра (X, Y, A и т.д.)
            try
            {
                float value = std::stof(token.substr(1)); // Преобразование в число
                if (param == 'X')
                {
                    if (value < 0 || value > MAX_X_)
                        throw std::invalid_argument("X coordinate out of range");
                    cmd.x = value;
                    cmd.has_x = true; // Сохранение X
                }
                else if (param == 'Y')
                {
                    if (value < 0 || value > MAX_Y_)
                        throw std::invalid_argument("Y coordinate out of range");
                    cmd.y = value;
                    cmd.has_y = true; // Сохранение Y
                }
                else if (param == 'A')
                {
                    if (value < -MAX_A_ || value > MAX_A_)
                        throw std::invalid_argument("A angle out of range");
                    cmd.a = value;
                    cmd.has_a = true; // Сохранение A
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
                        throw std::invalid_argument("Invalid feed rate F");
                    cmd.f = value;
                    cmd.has_f = true; // Сохранение F
                }
                else if (param == 'P')
                {
                    if (value < 0)
                        throw std::invalid_argument("Negative pause duration P");
                    cmd.p = value;
                    cmd.has_p = true; // Сохранение P
                }
            }
            catch (const std::exception &e)
            {
                ROS_WARN("Error parsing parameter in line: %s (%s)", line.c_str(), e.what()); // Лог ошибки
                return false;                                                                 // Возврат при ошибке
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

    // Выполнение G0 (быстрое перемещение)
    void executeG0(const GCodeCommand &cmd)
    {
        std::stringstream ss;                         // Для формирования лога
        float new_x = cmd.has_x ? cmd.x : current_x_; // Новая координата X
        float new_y = cmd.has_y ? cmd.y : current_y_; // Новая координата Y
        float new_a = cmd.has_a ? cmd.a : current_a_; // Новый угол A
        if (absolute_mode_)
        {
            current_x_ = new_x; // Обновление X
            current_y_ = new_y; // Обновление Y
            current_a_ = new_a; // Обновление A
        }
        else
        {
            current_x_ += cmd.has_x ? cmd.x : 0.0; // Относительное смещение X
            current_y_ += cmd.has_y ? cmd.y : 0.0; // Относительное смещение Y
            current_a_ += cmd.has_a ? cmd.a : 0.0; // Относительное смещение A
        }
        ss << "G0: Rapid move to X=" << current_x_ << " Y=" << current_y_ << " A=" << current_a_;
        if (!cmd.comment.empty())
            ss << " (" << cmd.comment << ")"; // Добавление комментария
        ROS_INFO("%s", ss.str().c_str());     // Вывод лога
        publishMessage(ss.str());             // Публикация
    }
    // Добавление паузы в массив исполнения
    void executePause()
    {
        SCommand temp;
        temp.mode = 1;
        temp.duration = 1000;
        temp.velL = 0.0;
        temp.velR = 0.0;
        commandArray.push_back(temp);
        ROS_INFO("    executePause %u msec", temp.duration); // Вывод лога
    }
    // Выполнение G1 (линейное перемещение или поворот)
    void executeG1(const GCodeCommand &cmd)
    {
        executePause();                              // Добавляем паузу перед любой операцией для стабилизации робота в ключевых точках.
        std::stringstream ss;                        // Для формирования лога
        float feed_rate = cmd.has_f ? cmd.f : 100.0; // Скорость подачи, по умолчанию 100

        // Проверка, является ли команда угловой (только A)
        if (cmd.has_a && !cmd.has_x && !cmd.has_y)
        {
            float new_a = cmd.a; // Новый угол A
            if (absolute_mode_)
            {
                current_a_ = new_a; // Обновление A
            }
            else
            {
                current_a_ += new_a; // Относительное смещение A
            }
            ss << "G1: Rotate to A= " << current_a_ << " F= " << feed_rate << " deg/s";
            if (!cmd.comment.empty())
                ss << " (" << cmd.comment << ")"; // Добавление комментария
            ROS_INFO("%s", ss.str().c_str());     // Вывод лога
            publishMessage(ss.str());             // Публикация

            SCommand temp;
            temp.mode = 2;
            temp.angle = cmd.a;
            temp.velAngle = cmd.f;
            commandArray.push_back(temp);
            ROS_INFO("    executeG1 mode= %i angle= %f velAngle=%f ", temp.mode, temp.angle, temp.velAngle); // Вывод лога

            return;
        }

        // Линейное перемещение
        float new_x = cmd.has_x ? cmd.x : current_x_; // Новая координата X
        float new_y = cmd.has_y ? cmd.y : current_y_; // Новая координата Y
        float new_a = cmd.has_a ? cmd.a : current_a_; // Новый угол A
        if (absolute_mode_)
        {
            current_x_ = new_x; // Обновление X
            current_y_ = new_y; // Обновление Y
            current_a_ = new_a; // Обновление A
        }
        else
        {
            current_x_ += cmd.has_x ? cmd.x : 0.0; // Относительное смещение X
            current_y_ += cmd.has_y ? cmd.y : 0.0; // Относительное смещение Y
            current_a_ += cmd.has_a ? cmd.a : 0.0; // Относительное смещение A
        }
        float distance = calculateDistance(new_x, new_y); // Вычисление дистанции
        ss << "G1: Linear move to X= " << current_x_ << " Y= " << current_y_ << " A= " << current_a_ << " F= " << feed_rate;
        ss << ", distance= " << distance << " mm";
        if (!cmd.comment.empty())
            ss << " (" << cmd.comment << ")"; // Добавление комментария
        ROS_INFO("%s", ss.str().c_str());     // Вывод лога
        publishMessage(ss.str());             // Публикация
    }

    // Выполнение G2 (дуга по часовой стрелке)
    void executeG2(const GCodeCommand &cmd)
    {
        executePause();                              // Добавляем паузу перед любой операцией для стабилизации робота в ключевых точках.
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
        ss << "G2: Clockwise arc to X=" << current_x_ << " Y=" << current_y_ << " I=" << center_i << " J=" << center_j << " F=" << feed_rate;
        if (!cmd.comment.empty())
            ss << " (" << cmd.comment << ")"; // Добавление комментария
        ROS_INFO("%s", ss.str().c_str());     // Вывод лога
        publishMessage(ss.str());             // Публикация
    }

    // Выполнение G3 (дуга против часовой стрелки)
    void executeG3(const GCodeCommand &cmd)
    {
        executePause();                              // Добавляем паузу перед любой операцией для стабилизации робота в ключевых точках.
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
        ss << "G3: Counterclockwise arc to X=" << current_x_ << " Y=" << current_y_ << " I=" << center_i << " J=" << center_j << " F=" << feed_rate;
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
        ss << "G4: Pause for " << pause_ms << " ms";
        if (!cmd.comment.empty())
            ss << " (" << cmd.comment << ")";     // Добавление комментария
        ROS_INFO("%s", ss.str().c_str());         // Вывод лога
        ros::Duration(pause_ms / 1000.0).sleep(); // Ожидание
        publishMessage(ss.str());                 // Публикация
    }

    // Выполнение G5 (Вращение колесами  определенное время по часовой)
    void executeG5(const GCodeCommand &cmd)
    {
        std::stringstream ss;                     // Для формирования лога
        float pause_ms = cmd.has_p ? cmd.p : 0.0; // Длительность паузы в мс
        ss << "G5: Rotate for " << cmd.f << " vel " << pause_ms << " ms";
        if (!cmd.comment.empty())
            ss << " (" << cmd.comment << ")";     // Добавление комментария
        ROS_INFO("%s", ss.str().c_str());         // Вывод лога
        // ros::Duration(pause_ms / 1000.0).sleep(); // Ожидание
        // publishMessage(ss.str());                 // Публикация

        SCommand temp;
        temp.mode = 1;
        temp.velL = cmd.f;
        temp.velR = -cmd.f;
        temp.duration = cmd.p;
        commandArray.push_back(temp);
        ROS_INFO("    executeG5 mode= %i velL= %f velR= %f duration=%f ", temp.mode, temp.velL, temp.velR, temp.duration); // Вывод лога
    }
    // Выполнение G6 (Вращение колесами  определенное время против часовой)
    void executeG6(const GCodeCommand &cmd)
    {
        std::stringstream ss;                     // Для формирования лога
        float pause_ms = cmd.has_p ? cmd.p : 0.0; // Длительность паузы в мс
        ss << "G5: Rotate for " << cmd.f << " vel " << pause_ms << " ms";
        if (!cmd.comment.empty())
            ss << " (" << cmd.comment << ")";     // Добавление комментария
        ROS_INFO("%s", ss.str().c_str());         // Вывод лога
        // ros::Duration(pause_ms / 1000.0).sleep(); // Ожидание
        // publishMessage(ss.str());                 // Публикация

        SCommand temp;
        temp.mode = 1;
        temp.velL = -cmd.f;
        temp.velR = cmd.f;
        temp.duration = cmd.p;
        commandArray.push_back(temp);
        ROS_INFO("    executeG6 mode= %i velL= %f velR= %f duration=%f ", temp.mode, temp.velL, temp.velR, temp.duration); // Вывод лога
    }
    // Выполнение G9 (Переход в начало. Зацикливание программы)
    void executeG9(const GCodeCommand &cmd)
    {
        std::stringstream ss;                     // Для формирования лога
        float pause_ms = cmd.has_p ? cmd.p : 0.0; // Длительность паузы в мс
        ss << "G9: Cicle...";
        if (!cmd.comment.empty())
            ss << " (" << cmd.comment << ")";     // Добавление комментария
        ROS_INFO("%s", ss.str().c_str());         // Вывод лога

        SCommand temp;
        temp.mode = 9;
        commandArray.push_back(temp);
        ROS_INFO("    executeG9 mode= %i ", temp.mode); // Вывод лога
    }
    // Выполнение G28 (возврат домой)
    void executeG28(const GCodeCommand &cmd)
    {
        executePause();       // Добавляем паузу перед любой операцией для стабилизации робота в ключевых точках.
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
        executePause();       // Добавляем паузу перед любой операцией для стабилизации робота в ключевых точках.
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
        executePause();       // Добавляем паузу перед любой операцией для стабилизации робота в ключевых точках.
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
    void run()
    {
        std::ifstream file(gcode_file_.c_str()); // Открытие файла
        if (!file.is_open())
        {
            ROS_ERROR("Failed to open file: %s", gcode_file_.c_str()); // Ошибка
            return;                                                    // Выход
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
                std::string log = "Added command: " + line;
                if (!cmd.comment.empty())
                    log += " (" + cmd.comment + ")"; // Добавление комментария
                ROS_INFO("%s", log.c_str());         // Вывод лога
            }
            else
            {
                ROS_WARN("Invalid line: %s", line.c_str()); // Предупреждение
            }
        }
        file.close(); // Закрытие файла

        // Вывод массива commands_ в консоль через ROS_INFO
        ROS_INFO(" \n Parsed G-code commands:");
        for (size_t i = 0; i < commands_.size(); ++i)
        {
            const GCodeCommand &cmd = commands_[i];
            std::stringstream ss;
            ss << "Command " << i << ": " << cmd.command;
            if (cmd.has_x)
                ss << " X= " << cmd.x;
            if (cmd.has_y)
                ss << " Y= " << cmd.y;
            if (cmd.has_a)
                ss << " A= " << cmd.a;
            if (cmd.has_i)
                ss << " I= " << cmd.i;
            if (cmd.has_j)
                ss << " J= " << cmd.j;
            if (cmd.has_f)
                ss << " F= " << cmd.f;
            if (cmd.has_p)
                ss << " P= " << cmd.p;
            if (!cmd.comment.empty())
                ss << " ; " << cmd.comment;
            ROS_INFO("%s", ss.str().c_str()); // Вывод в одну строку
        }
        ROS_INFO("Total commands: %zu \n", commands_.size()); // Общее количество команд

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
            else if (cmd.command == "G5")
                executeG5(cmd); // Выполнение G5
            else if (cmd.command == "G6")
                executeG6(cmd); // Выполнение G6
            else if (cmd.command == "G9")
                executeG9(cmd); // Выполнение G9
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
                ROS_WARN("Unknown command: %s (%s)", cmd.command.c_str(), cmd.comment.c_str()); // Предупреждение
            ros::spinOnce();                                                                    // Обработка сообщений
        }
        ROS_INFO("All commands executed"); // Сообщение об окончании

        // Вывод массива commands_ в консоль через ROS_INFO
        ROS_INFO(" \n commandArray commands:");
        for (size_t i = 0; i < commandArray.size(); ++i)
        {
            const SCommand &cmd = commandArray[i];
            std::stringstream ss;
            ss << "Command " << i << "= ";
            ss << " mode= " << cmd.mode;
            ss << " angle= " << cmd.angle;
            ss << " len= " << cmd.len;
            ss << " velL= " << cmd.velL;
            ss << " velR= " << cmd.velR;
            ss << " velAngle= " << cmd.velAngle;
            ss << " velLen= " << cmd.velLen;
            ss << " duration= " << cmd.duration;
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