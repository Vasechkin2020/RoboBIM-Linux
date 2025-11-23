/*
 * File: gen.cpp
 * Version: 3.0 (2025-11-23)
 * Changes:
 * - FIXED Coordinate Transformation Logic (Global CAD -> Local Robot).
 * - Logic: Subtract Offset first, then Rotate.
 * - Result: If Robot is at (25,10) on drawing, and Line starts at (25,10), Local Point is (0,0).
 * - Input file line 2 defines: Robot_X_on_Map, Robot_Y_on_Map, Rotation_Angle.
 */

#define _USE_MATH_DEFINES
#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <string>
#include <cstdio>
#include <sstream>
#include <algorithm>

// === НАСТРОЙКИ ===
const double MARKER_OFFSET = 0.045;
const double SPEED_DRAW = 0.10;
const double SPEED_MOVE = 0.10;
const double SPEED_TURN = 0.05;
const double SPEED_BACKUP = -0.10;
const char *DEFAULT_OUTPUT_FILENAME = "output_default.gcode";

struct Line
{
    double x1, y1;
    double x2, y2;
};

struct Transform
{
    double origin_x; // Где стоит робот на чертеже (X)
    double origin_y; // Где стоит робот на чертеже (Y)
    double sin_a;
    double cos_a;
};

double to_deg(double rad) { return rad * 180.0 / M_PI; }
double to_rad(double deg) { return deg * M_PI / 180.0; }

std::string trim(const std::string &str)
{
    size_t first = str.find_first_not_of(" \t\r\n");
    if (std::string::npos == first)
        return str;
    size_t last = str.find_last_not_of(" \t\r\n");
    return str.substr(first, (last - first + 1));
}

class GCodeGenerator
{
private:
    FILE *fp;
    Transform trans;

public:
    // Конструктор: принимает координаты РОБОТА НА ЧЕРТЕЖЕ
    GCodeGenerator(const char *filename, double robot_x, double robot_y, double angle_deg)
    {
        // Сохраняем параметры трансформации
        trans.origin_x = robot_x;
        trans.origin_y = robot_y;

        // Угол поворота. Используем отрицательный угол для обратного преобразования (World -> Local),
        // если предполагаем стандартное вращение осей.
        // Для простоты оставим стандартную матрицу, пользователь может инвертировать угол знаком минус.
        double rad = to_rad(angle_deg);
        trans.sin_a = std::sin(rad);
        trans.cos_a = std::cos(rad);

        fp = fopen(filename, "w");
        if (fp)
        {
            fprintf(fp, "; === GENERATED G-CODE (v3.0) ===\n");
            fprintf(fp, "; Output Filename: %s\n", filename);
            fprintf(fp, "; Robot Position on Map: X=%.3f, Y=%.3f, Angle=%.1f\n", robot_x, robot_y, angle_deg);
            fprintf(fp, "; Marker Offset: %.4f\n", MARKER_OFFSET);
            fprintf(fp, "M5 T1000 ; Pen UP\n");
            fprintf(fp, "G1 A0.000 F%.2f\n", SPEED_TURN);
            fprintf(fp, "G2 L%.3f F%.2f ; Initial align\n\n", MARKER_OFFSET, SPEED_BACKUP);
        }
        else
        {
            printf("Critical Error: Could not open output file %s.\n", filename);
        }
    }

    bool is_ready() const { return fp != nullptr; }

    // ГЛАВНОЕ ИСПРАВЛЕНИЕ: CAD (Global) -> Robot (Local)
    void applyTransform(double &x, double &y)
    {
        // 1. Сначала вычитаем координаты робота (Сдвиг начала координат)
        double dx = x - trans.origin_x;
        double dy = y - trans.origin_y;

        // 2. Затем поворачиваем полученный вектор
        // Формула поворота вектора:
        // X' = x*cos - y*sin
        // Y' = x*sin + y*cos
        double x_new = dx * trans.cos_a - dy * trans.sin_a;
        double y_new = dx * trans.sin_a + dy * trans.cos_a;

        x = x_new;
        y = y_new;
    }

    void writeComment(const std::string &comment)
    {
        if (fp)
            fprintf(fp, "; %s\n", comment.c_str());
    }

    void processLine(Line l, const std::string &user_comment = "")
    {
        if (!fp)
            return;

        double raw_x1 = l.x1, raw_y1 = l.y1;
        double raw_x2 = l.x2, raw_y2 = l.y2;

        // Трансформируем обе точки
        applyTransform(l.x1, l.y1);
        applyTransform(l.x2, l.y2);

        // Далее вся логика генерации работает с локальными координатами (l.x, l.y)
        double dx = l.x2 - l.x1;
        double dy = l.y2 - l.y1;
        double angle_rad = std::atan2(dy, dx);
        double angle_deg = to_deg(angle_rad);

        fprintf(fp, "; ---------------------------------------------------\n");
        if (!user_comment.empty())
            fprintf(fp, "; %s\n", user_comment.c_str());

        fprintf(fp, "; CAD:   (%.3f, %.3f) -> (%.3f, %.3f)\n", raw_x1, raw_y1, raw_x2, raw_y2);
        fprintf(fp, "; ROBOT: (%.3f, %.3f) -> (%.3f, %.3f) | Ang: %.2f\n", l.x1, l.y1, l.x2, l.y2, angle_deg);

        // 1. Центр к началу
        fprintf(fp, "M5 T1000\n");
        fprintf(fp, "G2 X%.3f Y%.3f F%.2f ; Center to Start\n", l.x1, l.y1, SPEED_MOVE);

        // 2. Поворот
        fprintf(fp, "G4 P200\n");
        fprintf(fp, "G1 A%.3f F%.2f\n", angle_deg, SPEED_TURN);
        fprintf(fp, "G4 P200\n");

        // 3. Откат
        fprintf(fp, "G2 L%.3f F%.2f ; Back up\n", MARKER_OFFSET, SPEED_BACKUP);

        // 4. Цель
        double off_x = MARKER_OFFSET * std::cos(angle_rad);
        double off_y = MARKER_OFFSET * std::sin(angle_rad);
        double target_x = l.x2 - off_x;
        double target_y = l.y2 - off_y;

        // 5. Рисование
        fprintf(fp, "M3 T1000\n");
        fprintf(fp, "G2 X%.3f Y%.3f F%.2f\n\n", target_x, target_y, SPEED_DRAW);
    }

    void appendInputFile(const char *input_filename)
    {
        if (!fp)
            return;
        std::ifstream infile(input_filename);
        std::string line;
        fprintf(fp, "\n; === ORIGINAL INPUT LINES START ===\n");
        while (std::getline(infile, line))
            fprintf(fp, "; %s\n", line.c_str());
        fprintf(fp, "; === ORIGINAL INPUT LINES END ===\n");
        infile.close();
    }

    // ИЗМЕНЕННАЯ ФУНКЦИЯ FINALIZE
    void finalize()
    {
        if (!fp)
            return;
        fprintf(fp, "\n; === END PROGRAM ===\n");
        fprintf(fp, "M5 T1000\n"); // Поднять инструмент

        // НОВАЯ КОМАНДА: поворот в 0
        fprintf(fp, "G1 A0.000 F%.2f ; Final rotation to 0 degrees\n", SPEED_TURN);

        fclose(fp);
        fp = nullptr;
    }
};

int main()
{
    const char *INPUT_FILENAME = "input_lines.txt";
    std::string output_filename;

    // Переменные для трансформации
    double trans_x = 0.0, trans_y = 0.0, trans_ang = 0.0;

    // 1. Создание примера (если нет файла)
    std::ifstream infile_check(INPUT_FILENAME);
    if (!infile_check.is_open())
    {
        printf("Info: Creating sample %s.\n", INPUT_FILENAME);
        std::ofstream outfile(INPUT_FILENAME);
        if (outfile.is_open())
        {
            outfile << "krvadratR.gcode ; Filename" << std::endl;
            outfile << "25.0 10.0 0.0 ; Робот стоит в X=25, Y=10" << std::endl;
            outfile << "; Линия начинается там же, где стоит робот" << std::endl;
            outfile << "25.0 10.0 26.0 11.0" << std::endl;
            outfile.close();
        }
        infile_check.open(INPUT_FILENAME);
    }
    infile_check.close();

    // 2. Чтение файла
    std::ifstream infile(INPUT_FILENAME);
    if (!infile.is_open())
        return 1;

    std::string line;
    int line_count = 0;

    // Считываем заголовок
    while (std::getline(infile, line))
    {
        line = trim(line);
        if (line.empty())
            continue;

        size_t comment_pos = line.find(';');
        std::string data = (comment_pos != std::string::npos) ? line.substr(0, comment_pos) : line;
        data = trim(data);
        if (data.empty())
            continue;

        if (line_count == 0)
        {
            output_filename = data;
            line_count++;
        }
        else if (line_count == 1)
        {
            if (sscanf(data.c_str(), "%lf %lf %lf", &trans_x, &trans_y, &trans_ang) == 3)
            {
                printf("Robot Position Loaded: X=%.2f, Y=%.2f, Angle=%.2f\n", trans_x, trans_y, trans_ang);
                line_count++;
                break;
            }
            else
            {
                printf("Warning: Invalid transformation line. Using 0 0 0.\n");
                line_count++;
                break;
            }
        }
    }

    if (output_filename.empty())
        output_filename = DEFAULT_OUTPUT_FILENAME;

    // 3. Инициализация
    GCodeGenerator gen(output_filename.c_str(), trans_x, trans_y, trans_ang);
    if (!gen.is_ready())
        return 1;

    printf("Processing coordinates...\n");

    // 4. Чтение координат
    while (std::getline(infile, line))
    {
        std::string original_line = line;
        line = trim(line);
        if (line.empty())
            continue;

        size_t comment_pos = line.find(';');
        std::string data_part = (comment_pos != std::string::npos) ? trim(line.substr(0, comment_pos)) : line;
        std::string comment_part = (comment_pos != std::string::npos) ? trim(line.substr(comment_pos + 1)) : "";

        if (data_part.empty())
        {
            if (!comment_part.empty())
                gen.writeComment(comment_part);
            continue;
        }

        double x1, y1, x2, y2;
        if (sscanf(data_part.c_str(), "%lf %lf %lf %lf", &x1, &y1, &x2, &y2) == 4)
        {
            gen.processLine({x1, y1, x2, y2}, comment_part);
        }
        else
        {
            gen.writeComment("SKIP INVALID: " + original_line);
        }
    }

    infile.close();
    gen.appendInputFile(INPUT_FILENAME);
    gen.finalize();

    printf("Done. Output: %s\n", output_filename.c_str());
    return 0;
}