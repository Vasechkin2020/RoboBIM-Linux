#ifndef GCODEPARSER_H
#define GCODEPARSER_H

#include <ros/ros.h>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <exception>
#include <iomanip>

extern AsyncFileLogger logi;

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// Вспомогательная функция для красивого вывода чисел
std::string fmt_float(float val) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(4) << val;
    std::string s = oss.str();
    s.erase(s.find_last_not_of('0') + 1, std::string::npos);
    if (!s.empty() && s.back() == '.') s.pop_back();
    return s;
}



struct GCodeCommand
{
    std::string command;
    bool has_l, has_a, has_f, has_p, has_t, has_f_l, has_f_r;
    bool has_o, has_x, has_y;
    float l, a, f, p, t, f_l, f_r;
    float o, x, y;
    std::string comment;
    std::string raw_line;
    int line_number;

    GCodeCommand()
        : has_l(false), has_a(false), has_f(false), has_p(false),
          has_t(false), has_f_l(false), has_f_r(false),
          has_o(false), has_x(false), has_y(false),
          l(0.0f), a(0.0f), f(0.0f), p(0.0f), t(0.0f),
          f_l(0.0f), f_r(0.0f),
          o(0.0f), x(0.0f), y(0.0f),
          line_number(0)
    {}
};

class GCodeParser
{
private:
    ros::NodeHandle nh_;
    float current_x_, current_y_, current_a_; // в метрах и градусах
    std::string gcode_file_;
    const float wheel_base_m_ = 0.38f; // расстояние между колёсами в метрах
    std::vector<GCodeCommand> commands_;
    float total_time_sec_ = 0.0f;

    std::string trim(const std::string &str)
    {
        size_t first = str.find_first_not_of(" \t\n\r");
        if (first == std::string::npos) return "";
        size_t last = str.find_last_not_of(" \t\n\r");
        return str.substr(first, (last - first + 1));
    }

    float normalizeAngle(float angle_deg)
    {
        while (angle_deg > 180.0f) angle_deg -= 360.0f;
        while (angle_deg <= -180.0f) angle_deg += 360.0f;
        return angle_deg;
    }

    bool parseGCodeLine(const std::string &line, GCodeCommand &cmd, int line_number)
    {
        std::string trimmed_line = trim(line);
        if (trimmed_line.empty()) return false;

        cmd.raw_line = line;
        cmd.line_number = line_number;

        std::stringstream ss(trimmed_line);
        std::string token;
        ss >> token;

        if (token.empty() || (token[0] != 'G' && token[0] != 'M'))
        {
            logi.log_w("Line %d: Invalid command token '%s'\n", line_number, token.c_str());
            return false;
        }
        cmd.command = token;

        while (ss >> token)
        {
            if (token[0] == ';')
            {
                std::getline(ss, cmd.comment);
                cmd.comment = trim(cmd.comment);
                break;
            }
            if (token.length() < 2)
            {
                logi.log_w("Line %d: Skipping short token '%s'\n", line_number, token.c_str());
                continue;
            }

            char param = token[0];
            float value;

            // === Проверка на кириллическую "О" ===
            if (token.length() >= 2 &&
                static_cast<unsigned char>(token[0]) == 0xD0 &&
                static_cast<unsigned char>(token[1]) == 0x9E)
            {
                logi.log_r("Line %d: Invalid parameter 'О' (Cyrillic O). Use Latin 'O' for relative angle!\n",
                          line_number);
                continue;
            }

            try
            {
                if (param == 'F' && token.length() >= 3 && token[1] == '_')
                {
                    std::string suffix = token.substr(2, 1);
                    std::string number_str = token.substr(3);
                    if (number_str.empty())
                    {
                        logi.log_w("Line %d: Empty number after F_\n", line_number);
                        continue;
                    }
                    value = std::stof(number_str);
                    if (suffix == "L")
                    {
                        cmd.f_l = value;
                        cmd.has_f_l = true;
                    }
                    else if (suffix == "R")
                    {
                        cmd.f_r = value;
                        cmd.has_f_r = true;
                    }
                    else
                    {
                        logi.log_w("Line %d: Unknown F suffix '%s'\n", line_number, suffix.c_str());
                    }
                }
                else
                {
                    value = std::stof(token.substr(1));
                    if (param == 'F') { cmd.f = value; cmd.has_f = true; }
                    else if (param == 'L') { cmd.l = value; cmd.has_l = true; }
                    else if (param == 'A') { cmd.a = value; cmd.has_a = true; }
                    else if (param == 'O') { cmd.o = value; cmd.has_o = true; }
                    else if (param == 'P') { cmd.p = value; cmd.has_p = true; }
                    else if (param == 'T') { cmd.t = value; cmd.has_t = true; }
                    else if (param == 'X') { cmd.x = value; cmd.has_x = true; }
                    else if (param == 'Y') { cmd.y = value; cmd.has_y = true; }
                    else
                    {
                        logi.log_w("Line %d: Unknown parameter '%c' (code=%d)\n",
                                  line_number, param, static_cast<int>(param));
                        continue;
                    }
                }
            }
            catch (const std::exception &e)
            {
                logi.log_w("Line %d: Parse error in '%s': %s\n", line_number, token.c_str(), e.what());
                continue;
            }
        }

        // === КОМПАКТНЫЙ ЛОГ ПАРСИНГА В ОДНУ СТРОКУ ===
        std::string parsed_params;
        if (cmd.has_l) parsed_params += " L=" + fmt_float(cmd.l);
        if (cmd.has_a) parsed_params += " A=" + fmt_float(cmd.a);
        if (cmd.has_o) parsed_params += " O=" + fmt_float(cmd.o);
        if (cmd.has_f && !cmd.has_f_l && !cmd.has_f_r) parsed_params += " F=" + fmt_float(cmd.f);
        if (cmd.has_p) parsed_params += " P=" + fmt_float(cmd.p);
        if (cmd.has_t) parsed_params += " T=" + fmt_float(cmd.t);
        if (cmd.has_f_l) parsed_params += " F_L=" + fmt_float(cmd.f_l);
        if (cmd.has_f_r) parsed_params += " F_R=" + fmt_float(cmd.f_r);
        if (cmd.has_x) parsed_params += " X=" + fmt_float(cmd.x);
        if (cmd.has_y) parsed_params += " Y=" + fmt_float(cmd.y);
        if (!cmd.comment.empty()) parsed_params += " ; " + cmd.comment;

        logi.log_b("Line %d: Parsed '%s'%s\n", line_number, cmd.command.c_str(), parsed_params.c_str());
        return true;
    }

    void executeG0(const GCodeCommand &cmd)
    {
        logi.log_g(">>> EXECUTING Line %d: %s\n", cmd.line_number, cmd.raw_line.c_str());
        logi.log_b("    BEFORE: X=%.3f, Y=%.3f, A=%.3f°\n", current_x_, current_y_, current_a_);

        if (!cmd.has_f_l || !cmd.has_f_r || !cmd.has_t)
        {
            logi.log_r("Line %d: G0 requires F_L, F_R, T\n", cmd.line_number);
            return;
        }
        if (cmd.t <= 0.0f || cmd.f_l < 0.0f || cmd.f_r < 0.0f)
        {
            logi.log_r("Line %d: Invalid G0 params: T=%.3f, F_L=%.3f, F_R=%.3f\n",
                      cmd.line_number, cmd.t, cmd.f_l, cmd.f_r);
            return;
        }

        // Сохраняем состояние ДО
        float before_x = current_x_;
        float before_y = current_y_;
        float before_a = current_a_;

        float v = (cmd.f_l + cmd.f_r) / 2.0f; // м/с
        float omega = (cmd.f_r - cmd.f_l) / wheel_base_m_; // рад/с
        float time_sec = cmd.t;

        if (std::abs(omega) < 1e-6f)
        {
            float length = v * time_sec;
            current_x_ += length * std::cos(before_a * M_PI / 180.0f);
            current_y_ += length * std::sin(before_a * M_PI / 180.0f);
        }
        else
        {
            float radius = v / std::abs(omega);
            float theta = omega * time_sec;
            float sign = (omega > 0) ? 1.0f : -1.0f;
            float center_x = before_x + radius * std::cos((before_a + sign * 90.0f) * M_PI / 180.0f);
            float center_y = before_y + radius * std::sin((before_a + sign * 90.0f) * M_PI / 180.0f);
            current_x_ = center_x + radius * std::cos((before_a + sign * 90.0f + theta * 180.0f / M_PI) * M_PI / 180.0f);
            current_y_ = center_y + radius * std::sin((before_a + sign * 90.0f + theta * 180.0f / M_PI) * M_PI / 180.0f);
            current_a_ = normalizeAngle(before_a + theta * 180.0f / M_PI);
        }

        total_time_sec_ += time_sec;

        SCommand command;
        command.mode = 0;
        command.duration = cmd.t * 1000.0f;
        command.velL = cmd.f_l;
        command.velR = cmd.f_r;
        // Сохраняем траекторию
        command.point_A_x = before_x;
        command.point_A_y = before_y;
        command.point_A_a = before_a;
        command.point_B_x = current_x_;
        command.point_B_y = current_y_;
        command.point_B_a = current_a_;
        commandArray.push_back(command);

        logi.log_b("    AFTER:  X=%.3f, Y=%.3f, A=%.3f°\n", current_x_, current_y_, current_a_);
        logi.log_b("    DURATION: %.3f sec (TOTAL: %.3f sec)\n", time_sec, total_time_sec_);
    }

    void executeG1(const GCodeCommand &cmd)
    {
        logi.log_g(">>> EXECUTING Line %d: %s\n", cmd.line_number, cmd.raw_line.c_str());
        logi.log_b("    BEFORE: X=%.3f, Y=%.3f, A=%.3f°\n", current_x_, current_y_, current_a_);

        int mode_count = (cmd.has_a ? 1 : 0) + (cmd.has_o ? 1 : 0) + ((cmd.has_x || cmd.has_y) ? 1 : 0);
        if ((cmd.has_x || cmd.has_y) && !(cmd.has_x && cmd.has_y))
        {
            logi.log_r("Line %d: G1 requires both X and Y\n", cmd.line_number);
            return;
        }
        if (mode_count != 1)
        {
            logi.log_r("Line %d: G1 must have exactly one of A, O, or X/Y\n", cmd.line_number);
            return;
        }

        float target_angle = 0.0f;
        if (cmd.has_a) target_angle = normalizeAngle(cmd.a);
        else if (cmd.has_o) target_angle = normalizeAngle(current_a_ + cmd.o);
        else if (cmd.has_x && cmd.has_y)
        {
            float dx = cmd.x - current_x_;
            float dy = cmd.y - current_y_;
            target_angle = (std::abs(dx) < 1e-6f && std::abs(dy) < 1e-6f)
                ? current_a_
                : normalizeAngle(static_cast<float>(atan2(dy, dx) * 180.0 / M_PI));
        }

        // Сохраняем состояние ДО
        float before_x = current_x_;
        float before_y = current_y_;
        float before_a = current_a_;

        float angle_diff = normalizeAngle(target_angle - before_a);
        float wheel_speed = cmd.has_f ? cmd.f : 0.1f; // м/с
        if (wheel_speed > 0.5f)
        {
            logi.log_w("G1: clamping wheel speed F=%.3f → 0.5 m/s\n", wheel_speed);
            wheel_speed = 0.5f;
        }
        if (wheel_speed <= 0.0f)
        {
            logi.log_r("Line %d: Invalid G1 F=%.3f (must be > 0)\n", cmd.line_number, wheel_speed);
            return;
        }

        float omega_rad_per_sec = (2.0f * wheel_speed) / wheel_base_m_;
        float omega_deg_per_sec = omega_rad_per_sec * (180.0f / M_PI);
        float time_sec = std::abs(angle_diff) / omega_deg_per_sec;

        current_a_ = target_angle;
        total_time_sec_ += time_sec;

        SCommand command;
        command.mode = 1;
        command.angle = target_angle;
        command.velAngle = wheel_speed;
        // Сохраняем траекторию
        command.point_A_x = before_x;
        command.point_A_y = before_y;
        command.point_A_a = before_a;
        command.point_B_x = current_x_;
        command.point_B_y = current_y_;
        command.point_B_a = current_a_;
        commandArray.push_back(command);

        logi.log_b("    AFTER:  X=%.3f, Y=%.3f, A=%.3f°\n", current_x_, current_y_, current_a_);
        logi.log_b("    DURATION: %.3f sec (TOTAL: %.3f sec)\n", time_sec, total_time_sec_);
    }

    void executeG2(const GCodeCommand &cmd)
    {
        logi.log_g(">>> EXECUTING Line %d: %s\n", cmd.line_number, cmd.raw_line.c_str());
        logi.log_b("    BEFORE: X=%.3f, Y=%.3f, A=%.3f°\n", current_x_, current_y_, current_a_);

        bool has_length = cmd.has_l;
        bool has_point = cmd.has_x || cmd.has_y;

        if (has_point && !(cmd.has_x && cmd.has_y))
        {
            logi.log_r("Line %d: G2 requires both X and Y\n", cmd.line_number);
            return;
        }
        if (has_length && has_point)
        {
            logi.log_r("Line %d: G2 cannot have both L and X/Y\n", cmd.line_number);
            return;
        }
        if (!has_length && !has_point)
        {
            logi.log_r("Line %d: G2 must have L or X/Y\n", cmd.line_number);
            return;
        }

        // Сохраняем состояние ДО
        float before_x = current_x_;
        float before_y = current_y_;
        float before_a = current_a_;

        float target_x = before_x;
        float target_y = before_y;
        float length = 0.0f;

        if (has_length)
        {
            if (cmd.l <= 0.0f)
            {
                logi.log_r("Line %d: Invalid G2 L=%.3f\n", cmd.line_number, cmd.l);
                return;
            }
            length = cmd.l;
            target_x = before_x + length * std::cos(before_a * M_PI / 180.0f);
            target_y = before_y + length * std::sin(before_a * M_PI / 180.0f);
        }
        else if (has_point)
        {
            target_x = cmd.x;
            target_y = cmd.y;
            float dx = target_x - before_x;
            float dy = target_y - before_y;
            length = std::sqrt(dx*dx + dy*dy);
        }

        float feed_rate = cmd.has_f ? cmd.f : 0.1f; // м/с
        if (feed_rate > 0.5f) feed_rate = 0.5f;
        if (feed_rate <= 0.0f)
        {
            logi.log_r("Line %d: Invalid G2 F=%.3f\n", cmd.line_number, feed_rate);
            return;
        }

        float time_sec = (length > 0.0f) ? (length / feed_rate) : 0.0f;
        current_x_ = target_x;
        current_y_ = target_y;
        total_time_sec_ += time_sec;

        SCommand command;
        command.mode = 2;
        command.len = length;
        command.velLen = feed_rate;
        // Сохраняем траекторию
        command.point_A_x = before_x;
        command.point_A_y = before_y;
        command.point_A_a = before_a;
        command.point_B_x = current_x_;
        command.point_B_y = current_y_;
        command.point_B_a = current_a_;
        commandArray.push_back(command);

        logi.log_b("    AFTER:  X=%.3f, Y=%.3f, A=%.3f°\n", current_x_, current_y_, current_a_);
        logi.log_b("    DURATION: %.3f sec (TOTAL: %.3f sec)\n", time_sec, total_time_sec_);
    }

    void executeG4(const GCodeCommand &cmd)
    {
        logi.log_g(">>> EXECUTING Line %d: %s\n", cmd.line_number, cmd.raw_line.c_str());

        // Сохраняем состояние ДО (и ПОСЛЕ — оно не меняется)
        float before_x = current_x_;
        float before_y = current_y_;
        float before_a = current_a_;

        float pause_ms = cmd.has_p ? cmd.p : 0.0f;
        float time_sec = pause_ms / 1000.0f;
        total_time_sec_ += time_sec;

        SCommand temp;
        temp.mode = 0;
        temp.duration = pause_ms;
        // Сохраняем траекторию (без изменений)
        temp.point_A_x = before_x;
        temp.point_A_y = before_y;
        temp.point_A_a = before_a;
        temp.point_B_x = before_x;
        temp.point_B_y = before_y;
        temp.point_B_a = before_a;
        commandArray.push_back(temp);

        logi.log_b("    DURATION: %.3f sec (TOTAL: %.3f sec)\n", time_sec, total_time_sec_);
    }

    void executeG9(const GCodeCommand &cmd)
    {
        logi.log_g(">>> EXECUTING Line %d: %s\n", cmd.line_number, cmd.raw_line.c_str());
        logi.log_b("    STATE: X=%.3f, Y=%.3f, A=%.3f°\n", current_x_, current_y_, current_a_);

        SCommand temp;
        temp.mode = 9;
        // Сохраняем текущее состояние как траекторию
        temp.point_A_x = current_x_;
        temp.point_A_y = current_y_;
        temp.point_A_a = current_a_;
        temp.point_B_x = current_x_;
        temp.point_B_y = current_y_;
        temp.point_B_a = current_a_;
        commandArray.push_back(temp);
    }

    void executeM3(const GCodeCommand &cmd)
    {
        logi.log_g(">>> EXECUTING Line %d: %s\n", cmd.line_number, cmd.raw_line.c_str());
        logi.log_b("    STATE: X=%.3f, Y=%.3f, A=%.3f°\n", current_x_, current_y_, current_a_);

        SCommand temp;
        temp.mode = 5; // M3
        temp.point_A_x = current_x_;
        temp.point_A_y = current_y_;
        temp.point_A_a = current_a_;
        temp.point_B_x = current_x_;
        temp.point_B_y = current_y_;
        temp.point_B_a = current_a_;
        commandArray.push_back(temp);
    }

    void executeM5(const GCodeCommand &cmd)
    {
        logi.log_g(">>> EXECUTING Line %d: %s\n", cmd.line_number, cmd.raw_line.c_str());
        logi.log_b("    STATE: X=%.3f, Y=%.3f, A=%.3f°\n", current_x_, current_y_, current_a_);

        SCommand temp;
        temp.mode = 6; // M5
        temp.point_A_x = current_x_;
        temp.point_A_y = current_y_;
        temp.point_A_a = current_a_;
        temp.point_B_x = current_x_;
        temp.point_B_y = current_y_;
        temp.point_B_a = current_a_;
        commandArray.push_back(temp);
    }

public:
    GCodeParser()
        : nh_("~"),
          current_x_(0.0f), current_y_(0.0f), current_a_(0.0f),
          total_time_sec_(0.0f),
          gcode_file_("gcode/my_custom_gcode.gcode")
    {
        nh_.param<std::string>("gcode_file", gcode_file_, "$(find pb)/gcode/my_custom_gcode.gcode");
        logi.log_b("GCodeParser init: %s\n", gcode_file_.c_str());
        if (!nh_.getParam("gcode_file", gcode_file_))
        {
            logi.log_w("Using default gcode_file: %s\n", gcode_file_.c_str());
        }
    }

    // НОВЫЙ МЕТОД: задать начальную позу
    void setInitialPose(float x, float y, float a_deg)
    {
        current_x_ = x;
        current_y_ = y;
        current_a_ = normalizeAngle(a_deg);
        total_time_sec_ = 0.0f;
        commandArray.clear();
        logi.log_b("Initial pose set: X=%.3f, Y=%.3f, A=%.3f°\n", x, y, current_a_);
    }

    void run()
    {
        // Очищаем предыдущие команды (но сохраняем текущую позу!)
        commands_.clear();
        // commandArray и total_time_sec_ уже учтены в setInitialPose

        logi.log_g("INITIAL STATE: X=%.3f, Y=%.3f, A=%.3f°\n\n", current_x_, current_y_, current_a_);

        std::ifstream file(gcode_file_.c_str());
        if (!file.is_open())
        {
            logi.log_r("Failed to open G-code file: %s\n", gcode_file_.c_str());
            return;
        }

        std::string line;
        int line_number = 0;
        while (std::getline(file, line))
        {
            ++line_number;
            line = trim(line);
            if (line.empty() || line[0] == '%') continue;

            GCodeCommand cmd;
            if (parseGCodeLine(line, cmd, line_number))
            {
                commands_.push_back(cmd);
                logi.log_b("Line %d: Added: %s\n\n", line_number, line.c_str());
            }
            else
            {
                logi.log_w("Line %d: Invalid line\n\n", line_number);
            }
        }
        file.close();
        logi.log_b("Total parsed commands: %zu\n", commands_.size());

        for (const auto &cmd : commands_)
        {
            if (cmd.command == "G0") executeG0(cmd);
            else if (cmd.command == "G1") executeG1(cmd);
            else if (cmd.command == "G2") executeG2(cmd);
            else if (cmd.command == "G4") executeG4(cmd);
            else if (cmd.command == "G9") executeG9(cmd);
            else if (cmd.command == "M3") executeM3(cmd);
            else if (cmd.command == "M5") executeM5(cmd);
            else
                logi.log_w("Unknown command: %s\n", cmd.command.c_str());
        }

        // === ИТОГОВЫЙ ОТЧЁТ ===
        std::string separator(60, '=');
        logi.log_g("%s\n", separator.c_str());
        logi.log_g("FINAL STATE:\n");
        logi.log_g("  X = %.3f m\n", current_x_);
        logi.log_g("  Y = %.3f m\n", current_y_);
        logi.log_g("  A = %.3f°\n", current_a_);
        if (total_time_sec_ >= 60.0f)
        {
            logi.log_g("TOTAL CALCULATED TIME: (%.1f minutes)\n", total_time_sec_ / 60.0f);
        }
        else
        {
            logi.log_g("TOTAL CALCULATED TIME: %.3f seconds\n", total_time_sec_);
        }
        logi.log_g("%s\n", separator.c_str());

        logi.log_w("Execution complete. commandArray size: %zu\n\n", commandArray.size());
        if (!commandArray.empty())
        {
            for (size_t i = 0; i < commandArray.size(); ++i)
            {
                const SCommand &c = commandArray[i];
                logi.log("Cmd[%zu]: mode=%d, dur=%.1f, vL=%.3f, vR=%.3f, "
                        "Δ=%.3f, vA=%.3f, len=%.3f, vL=%.3f | "
                        "A(%.3f,%.3f,%.3f) → B(%.3f,%.3f,%.3f)\n",
                        i, c.mode, c.duration, c.velL, c.velR,
                        c.angle, c.velAngle, c.len, c.velLen,
                        c.point_A_x, c.point_A_y, c.point_A_a,
                        c.point_B_x, c.point_B_y, c.point_B_a);
            }
        }
    }

    const std::vector<SCommand>& getCommandArray() const { return commandArray; }
    float getCurrentX() const { return current_x_; }
    float getCurrentY() const { return current_y_; }
    float getCurrentA() const { return current_a_; }
    float getTotalTimeSec() const { return total_time_sec_; }
};

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
- Расстояние между колёсами: 0.38 м
- Используйте ЛАТИНСКИЕ буквы (особенно 'O', а не кириллическую 'О')

Новые возможности:
- Задайте начальную позу: parser.setInitialPose(x, y, angle_deg);
- Каждая команда в commandArray содержит:
    point_A_* — состояние ДО выполнения
    point_B_* — состояние ПОСЛЕ выполнения

Команды:
G0 F_L<скорость> F_R<скорость> T<время_сек>
G1 <режим> [F<скорость>]
G2 <режим> [F<скорость>]
G4 P<мс>
G9
M3 / M5

Пример:
    GCodeParser parser;
    parser.setInitialPose(1.0, -0.5, 30.0); // Старт из (1.0, -0.5) с углом 30°
    parser.run();

===============================================================================
*/

#endif // GCODEPARSER_H