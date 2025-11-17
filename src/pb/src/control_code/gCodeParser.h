#ifndef GCODEPARSER_H
#define GCODEPARSER_H

#include <ros/ros.h>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <cmath>
#include <exception>

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


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
    float current_x_, current_y_, current_a_;
    std::string gcode_file_;
    float wheel_base_ = 200.0f;
    std::vector<GCodeCommand> commands_;
    std::vector<SCommand> commandArray;
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
        logi.log_b("Line %d: Parsed command '%s'\n", line_number, token.c_str());

        while (ss >> token)
        {
            if (token[0] == ';')
            {
                std::getline(ss, cmd.comment);
                cmd.comment = trim(cmd.comment);
                logi.log_b("Line %d: Parsed comment '%s'\n", line_number, cmd.comment.c_str());
                break;
            }
            if (token.length() < 2)
            {
                logi.log_w("Line %d: Skipping short token '%s'\n", line_number, token.c_str());
                continue;
            }

            char param = token[0];
            float value;

            // === ПРОВЕРКА НА КИРИЛЛИЧЕСКУЮ "О" ===
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
                        logi.log_b("Line %d: Parsed F_L=%.3f\n", line_number, value);
                    }
                    else if (suffix == "R")
                    {
                        cmd.f_r = value;
                        cmd.has_f_r = true;
                        logi.log_b("Line %d: Parsed F_R=%.3f\n", line_number, value);
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
                    logi.log_b("Line %d: Parsed %c=%.3f\n", line_number, param, value);
                }
            }
            catch (const std::exception &e)
            {
                logi.log_w("Line %d: Parse error in '%s': %s\n", line_number, token.c_str(), e.what());
                continue;
            }
        }
        return true;
    }

    void executeG0(const GCodeCommand &cmd)
    {
        logi.log_b("\n>>> EXECUTING: %s\n", cmd.raw_line.c_str());
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

        float v = (cmd.f_l + cmd.f_r) / 2.0f;
        float omega = (cmd.f_r - cmd.f_l) / wheel_base_;
        float time_sec = cmd.t;

        float start_x = current_x_;
        float start_y = current_y_;
        float start_a = current_a_;

        if (std::abs(omega) < 0.001f)
        {
            float length = v * time_sec;
            current_x_ += length * std::cos(start_a * M_PI / 180.0f);
            current_y_ += length * std::sin(start_a * M_PI / 180.0f);
        }
        else
        {
            float radius = v / std::abs(omega);
            float theta = omega * time_sec;
            float sign = (omega > 0) ? 1.0f : -1.0f;
            float center_x = start_x + radius * std::cos((start_a + sign * 90.0f) * M_PI / 180.0f);
            float center_y = start_y + radius * std::sin((start_a + sign * 90.0f) * M_PI / 180.0f);
            current_x_ = center_x + radius * std::cos((start_a + sign * 90.0f + theta * 180.0f / M_PI) * M_PI / 180.0f);
            current_y_ = center_y + radius * std::sin((start_a + sign * 90.0f + theta * 180.0f / M_PI) * M_PI / 180.0f);
            current_a_ = normalizeAngle(start_a + theta * 180.0f / M_PI);
        }

        total_time_sec_ += time_sec;

        SCommand command;
        command.mode = 0;
        command.duration = cmd.t * 1000.0f;
        command.velL = cmd.f_l;
        command.velR = cmd.f_r;
        commandArray.push_back(command);

        logi.log_b("    AFTER:  X=%.3f, Y=%.3f, A=%.3f°\n", current_x_, current_y_, current_a_);
        logi.log_b("    DURATION: %.3f sec (TOTAL: %.3f sec)\n", time_sec, total_time_sec_);
    }

    void executeG1(const GCodeCommand &cmd)
    {
        logi.log_b("\n>>> EXECUTING: %s\n", cmd.raw_line.c_str());
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
            target_angle = (std::abs(dx) < 1e-3f && std::abs(dy) < 1e-3f)
                ? current_a_
                : normalizeAngle(static_cast<float>(atan2(dy, dx) * 180.0 / M_PI));
        }

        float angle_diff = normalizeAngle(target_angle - current_a_);
        float feed_rate = cmd.has_f ? cmd.f : 0.1f;
        if (feed_rate > 0.5f) feed_rate = 0.5f;
        if (feed_rate <= 0.0f)
        {
            logi.log_r("Line %d: Invalid G1 F=%.3f\n", cmd.line_number, feed_rate);
            return;
        }

        float time_sec = std::abs(angle_diff) / feed_rate;
        current_a_ = target_angle;
        total_time_sec_ += time_sec;

        SCommand command;
        command.mode = 1;
        command.angle = angle_diff;
        command.velAngle = feed_rate;
        commandArray.push_back(command);

        logi.log_b("    AFTER:  X=%.3f, Y=%.3f, A=%.3f°\n", current_x_, current_y_, current_a_);
        logi.log_b("    DURATION: %.3f sec (TOTAL: %.3f sec)\n", time_sec, total_time_sec_);
    }

    void executeG2(const GCodeCommand &cmd)
    {
        logi.log_b("\n>>> EXECUTING: %s\n", cmd.raw_line.c_str());
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

        float target_x = current_x_;
        float target_y = current_y_;
        float length = 0.0f;

        if (has_length)
        {
            if (cmd.l <= 0.0f)
            {
                logi.log_r("Line %d: Invalid G2 L=%.3f\n", cmd.line_number, cmd.l);
                return;
            }
            length = cmd.l;
            target_x = current_x_ + length * std::cos(current_a_ * M_PI / 180.0f);
            target_y = current_y_ + length * std::sin(current_a_ * M_PI / 180.0f);
        }
        else if (has_point)
        {
            target_x = cmd.x;
            target_y = cmd.y;
            float dx = target_x - current_x_;
            float dy = target_y - current_y_;
            length = std::sqrt(dx*dx + dy*dy);
        }

        float feed_rate = cmd.has_f ? cmd.f : 0.1f;
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
        commandArray.push_back(command);

        logi.log_b("    AFTER:  X=%.3f, Y=%.3f, A=%.3f°\n", current_x_, current_y_, current_a_);
        logi.log_b("    DURATION: %.3f sec (TOTAL: %.3f sec)\n", time_sec, total_time_sec_);
    }

    void executeG4(const GCodeCommand &cmd)
    {
        logi.log_b("\n>>> EXECUTING: %s\n", cmd.raw_line.c_str());
        logi.log_b("    BEFORE: X=%.3f, Y=%.3f, A=%.3f°\n", current_x_, current_y_, current_a_);

        float pause_ms = cmd.has_p ? cmd.p : 0.0f;
        float time_sec = pause_ms / 1000.0f;
        total_time_sec_ += time_sec;

        SCommand temp;
        temp.mode = 4;
        temp.duration = pause_ms;
        commandArray.push_back(temp);

        logi.log_b("    AFTER:  X=%.3f, Y=%.3f, A=%.3f°\n", current_x_, current_y_, current_a_);
        logi.log_b("    DURATION: %.3f sec (TOTAL: %.3f sec)\n", time_sec, total_time_sec_);
    }

    void executeG9(const GCodeCommand &cmd)
    {
        logi.log_b("\n>>> EXECUTING: %s\n", cmd.raw_line.c_str());
        logi.log_b("    STATE: X=%.3f, Y=%.3f, A=%.3f°\n", current_x_, current_y_, current_a_);
        SCommand temp;
        temp.mode = 9;
        commandArray.push_back(temp);
    }

    void executeM3(const GCodeCommand &cmd)
    {
        logi.log_b("\n>>> EXECUTING: %s\n", cmd.raw_line.c_str());
        logi.log_b("    STATE: X=%.3f, Y=%.3f, A=%.3f°\n", current_x_, current_y_, current_a_);
    }

    void executeM5(const GCodeCommand &cmd)
    {
        logi.log_b("\n>>> EXECUTING: %s\n", cmd.raw_line.c_str());
        logi.log_b("    STATE: X=%.3f, Y=%.3f, A=%.3f°\n", current_x_, current_y_, current_a_);
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

    void run()
    {
        logi.log_b("INITIAL STATE: X=%.3f, Y=%.3f, A=%.3f°\n", current_x_, current_y_, current_a_);

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
                std::string log = "Added: " + line;
                if (!cmd.comment.empty()) log += " (" + cmd.comment + ")";
                logi.log_b("Line %d: %s\n", line_number, log.c_str());
            }
            else
            {
                logi.log_w("Line %d: Invalid line\n", line_number);
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

        std::string separator(60, '=');
        logi.log_b("\n%s\n", separator.c_str());
        logi.log_b("FINAL STATE:\n");
        logi.log_b("  X = %.3f mm\n", current_x_);
        logi.log_b("  Y = %.3f mm\n", current_y_);
        logi.log_b("  A = %.3f°\n", current_a_);
        logi.log_b("TOTAL CALCULATED TIME: %.3f seconds", total_time_sec_);
        if (total_time_sec_ >= 60.0f)
        {
            logi.log_b(" (%.1f minutes)", total_time_sec_ / 60.0f);
        }
        logi.log_b("\n%s\n", separator.c_str());

        logi.log_b("Execution complete. commandArray size: %zu\n", commandArray.size());
        if (!commandArray.empty())
        {
            for (size_t i = 0; i < commandArray.size(); ++i)
            {
                const SCommand &c = commandArray[i];
                logi.log("Cmd[%zu]: mode=%d, dur=%.1f, vL=%.3f, vR=%.3f, "
                        "Δ=%.3f, vA=%.3f, len=%.3f, vL=%.3f\n",
                        i, c.mode, c.duration, c.velL, c.velR,
                        c.angle, c.velAngle, c.len, c.velLen);
            }
        }
    }

    const std::vector<SCommand>& getCommandArray() const { return commandArray; }
    float getCurrentX() const { return current_x_; }
    float getCurrentY() const { return current_y_; }
    float getCurrentA() const { return current_a_; }
    float getTotalTimeSec() const { return total_time_sec_; }
};

#endif // GCODEPARSER_H

/*
===============================================================================
                    G-CODE СПРАВКА (для робота с диф. приводом)
===============================================================================

Общие правила:
- Все углы — в градусах, в диапазоне [-180, +180)
- Скорости по умолчанию: 0.1 (мм/с для движения, град/с для поворота)
- Максимальная скорость: 0.5 (автоматически ограничивается)
- Параметры чувствительны к регистру: используйте ЛАТИНСКИЕ буквы!
- Комментарии после ';'

-------------------------------------------------------------------------------
G0 F_L<скорость> F_R<скорость> T<время_сек>
    Управление колёсами напрямую.
    Пример: G0 F_L50 F_R50 T2.0  → движение вперёд 2 сек со скоростью 50 мм/с

G1 <режим> [F<скорость>]
    Поворот на месте. Ровно ОДИН из режимов:
    • A<угол>         — абсолютный поворот (например, A90)
    • O<угол>         — относительный поворот (например, O-45)
    • X<коорд> Y<коорд> — поворот лицом к точке (например, X100 Y0)
    Примеры:
        G1 A0 F0.05     → повернуться точно на 0°
        G1 O90          → повернуться на +90° от текущего угла
        G1 X0 Y-100     → повернуться вниз (к точке 0, -100)

G2 <режим> [F<скорость>]
    Линейное движение. Ровно ОДИН из режимов:
    • L<длина>        — движение вперёд на N мм
    • X<коорд> Y<коорд> — движение прямо к точке
    Примеры:
        G2 L100 F0.2    → проехать 100 мм вперёд со скоростью 0.2 мм/с
        G2 X0 Y0        → вернуться в начало координат

G4 P<мс>
    Пауза (в миллисекундах).
    Пример: G4 P2000 → пауза на 2 секунды

G9
    Сигнал зацикливания (для контроллера).

M3 / M5
    Включение / выключение вспомогательного модуля (например, печати).

-------------------------------------------------------------------------------
ВАЖНО:
- Используйте ЛАТИНСКУЮ букву 'O' (не кириллическую 'О') в G1 O...
- Все команды выполняются последовательно.
- Координаты и угол обновляются после каждой команды.
- Общее время выполнения рассчитывается автоматически.

Пример программы:
    G1 A90 F0.1     ; повернуться на 90°
    G2 L200 F0.2    ; проехать 200 мм вперёд
    G1 X0 Y0        ; повернуться домой
    G2 X0 Y0        ; вернуться домой
    G4 P1000        ; подождать 1 сек
    G9              ; зациклить

===============================================================================
*/