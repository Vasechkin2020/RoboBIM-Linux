/*
 * File: gen.cpp
 * Version: 3.9.2 (2025-11-23)
 * Changes:
 * - CHANGED: Finalize logic now uses 'L' command for alignment instead of 'X Y'.
 * - Logic:
 *      1. Backup: G2 L(Offset) F(Negative)
 *      2. Align:  G2 L(Offset) F(Positive) <- NEW
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
    std::string comment;
};

struct Transform
{
    double origin_x;
    double origin_y;
    double sin_a;
    double cos_a;
    double angle_deg;
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

// УТИЛИТА: Применяет трансформацию к координатам (глобальная -> локальная)
void applyTransform(double &x, double &y, const Transform &t)
{
    double dx = x - t.origin_x; // Смещение относительно начала робота
    double dy = y - t.origin_y;

    double x_new = dx * t.cos_a - dy * t.sin_a; // Применение поворота
    double y_new = dx * t.sin_a + dy * t.cos_a;

    x = x_new;
    y = y_new;
}

class GCodeGenerator
{
private:
    FILE *fp;
    Transform trans;

    // Храним координаты конца последней нарисованной линии для расчета парковки
    double last_endpoint_x;
    double last_endpoint_y;
    bool has_drawn_anything;

public:
    // Конструктор
    GCodeGenerator(const char *filename, const Transform &t, double first_x_local, double first_y_local)
        : trans(t), last_endpoint_x(0), last_endpoint_y(0), has_drawn_anything(false)
    {
        fp = fopen(filename, "w");
        if (fp)
        {
            fprintf(fp, "; === GENERATED G-CODE (v3.9.3) ===\n");
            fprintf(fp, "; Output Filename: %s\n", filename);
            fprintf(fp, "; Robot System Coordinate Origin on Map: X=%.3f, Y=%.3f, Angle=%.1f\n", t.origin_x, t.origin_y, t.angle_deg);
            fprintf(fp, "; Marker Offset: %.4f\n", MARKER_OFFSET);

            // G10: Просто задаем координаты. Робот считает, что он стоит в точке старта.
            fprintf(fp, "G10 X%.3f Y%.3f A%.3f ; Initial required start point and angle\n",
                    first_x_local, first_y_local, t.angle_deg);

            // СТАРТОВЫЙ БЛОК: Только настройки, никаких движений L
            fprintf(fp, "M5 T2000 ; Pen UP\n");
            // Поворот в 0 не обязателен, так как processLine сам повернет куда надо, но можно оставить для порядка, если хочется начать с красивой позы.
            fprintf(fp, "G1 A0.000 F%.2f ; Set initial angle\n\n", SPEED_TURN);

            // УБРАНО: G2 L... (Initial align).
            // Это лишнее, так как первая же строка processLine сделает "Center to Start" и переедет это движение.
        }
        else
        {
            printf("Critical Error: Could not open output file %s.\n", filename);
        }
    }

    bool is_ready() const { return fp != nullptr; }

    void processLine(Line l, const Transform &t)
    {
        if (!fp)
            return;

        double raw_x1 = l.x1, raw_y1 = l.y1;
        double raw_x2 = l.x2, raw_y2 = l.y2;

        // Трансформируем локально
        applyTransform(l.x1, l.y1, t);
        applyTransform(l.x2, l.y2, t);

        double dx = l.x2 - l.x1;
        double dy = l.y2 - l.y1;
        double angle_rad = std::atan2(dy, dx);
        double angle_deg = to_deg(angle_rad);

        fprintf(fp, "; ---------------------------------------------------\n");
        if (!l.comment.empty())
            fprintf(fp, "; %s\n", l.comment.c_str());

        fprintf(fp, "; CAD:   (%.3f, %.3f) -> (%.3f, %.3f)\n", raw_x1, raw_y1, raw_x2, raw_y2);
        fprintf(fp, "; ROBOT: (%.3f, %.3f) -> (%.3f, %.3f) | Ang: %.2f\n", l.x1, l.y1, l.x2, l.y2, angle_deg);

        // 1. Центр к началу
        fprintf(fp, "M5 T2000\n");
        fprintf(fp, "G2 X%.3f Y%.3f F%.2f ; Center to Start\n", l.x1, l.y1, SPEED_MOVE);

        // 2. Поворот
        fprintf(fp, "G4 P2000\n");
        fprintf(fp, "G1 A%.3f F%.2f\n", angle_deg, SPEED_TURN);
        fprintf(fp, "G4 P2000\n");

        // 3. Откат (Backup)
        fprintf(fp, "G2 L%.3f F%.2f ; Back up\n", MARKER_OFFSET, SPEED_BACKUP);

        // 4. Цель (с компенсацией)
        double off_x = MARKER_OFFSET * std::cos(angle_rad);
        double off_y = MARKER_OFFSET * std::sin(angle_rad);
        double target_x = l.x2 - off_x;
        double target_y = l.y2 - off_y;

        // 5. Рисование
        fprintf(fp, "M3 T2000\n");
        fprintf(fp, "G2 X%.3f Y%.3f F%.2f\n\n", target_x, target_y, SPEED_DRAW);

        // Запоминаем, где фактически остановился ЦЕНТР робота и конец линии
        last_endpoint_x = l.x2;
        last_endpoint_y = l.y2;
        has_drawn_anything = true;
    }

    // ИЗМЕНЕННЫЙ finalize: Теперь использует L для подъезда
    void finalize(bool use_parking, double park_x, double park_y)
    {
        if (!fp)
            return;
        fprintf(fp, "; === FINALIZE SEQUENCE ===\n");
        fprintf(fp, "M5 T2000 ; Pen UP\n");

        // ШАГ 1: "Подъезд". Двигаемся вперед на длину оффсета.
        // Так как мы только что закончили рисовать, мы стоим лицом по направлению линии.
        // Движение вперед на MARKER_OFFSET поставит центр робота ровно в конец линии.
        if (has_drawn_anything)
        {
            fprintf(fp, "G2 L%.3f F%.2f ; Align Center (Forward L to undo offset)\n",
                    MARKER_OFFSET, SPEED_MOVE);
            fprintf(fp, "G4 P1000\n");
        }

        // ШАГ 2: Едем на парковку, если она задана
        if (use_parking && has_drawn_anything)
        {
            // Теперь центр робота находится в (last_endpoint_x, last_endpoint_y)
            double dx = park_x - last_endpoint_x;
            double dy = park_y - last_endpoint_y;
            double angle_to_park = to_deg(std::atan2(dy, dx));

            // Поворот к парковке
            fprintf(fp, "G1 A%.3f F%.2f ; Rotate to Park\n", angle_to_park, SPEED_TURN);
            fprintf(fp, "G4 P1000\n");

            // Едем на парковку
            fprintf(fp, "G2 X%.3f Y%.3f F%.2f ; Move to Park Position\n", park_x, park_y, SPEED_MOVE);
            fprintf(fp, "G4 P1000\n");
        }

        // ШАГ 3: Финальный поворот в ноль
        fprintf(fp, "G1 A0.000 F%.2f ; Final rotation to 0 degrees\n", SPEED_TURN);
        fprintf(fp, "\n; === END PROGRAM ===\n");
    }

    void appendVector(const std::vector<std::string> &lines)
    {
        if (!fp)
            return;
        fprintf(fp, "\n; === ORIGINAL INPUT LINES START (Input backup) ===\n");
        for (const auto &line : lines)
        {
            fprintf(fp, "; %s\n", line.c_str());
        }
        fprintf(fp, "; === ORIGINAL INPUT LINES END ===\n");
    }

    void closeFile()
    {
        if (fp)
        {
            fclose(fp);
            fp = nullptr;
        }
    }
};

int main()
{
    const char *INPUT_FILENAME = "input_lines.txt";
    std::string output_filename;

    Transform trans = {0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<std::string> raw_input_lines;
    std::vector<std::string> body_lines_raw;

    // 1. Чтение файла в память
    std::ifstream infile(INPUT_FILENAME);
    if (!infile.is_open())
    {
        printf("Error: Could not open %s\n", INPUT_FILENAME);
        return 1;
    }

    std::string line;
    int line_count = 0;

    while (std::getline(infile, line))
    {
        raw_input_lines.push_back(line);

        std::string trimmed_line = trim(line);
        if (trimmed_line.empty())
            continue;

        size_t comment_pos = trimmed_line.find(';');
        std::string data = (comment_pos != std::string::npos) ? trimmed_line.substr(0, comment_pos) : trimmed_line;
        data = trim(data);
        if (data.empty())
            continue;

        if (line_count == 0)
        {
            output_filename = data;
        }
        else if (line_count == 1)
        {
            double trans_x, trans_y, trans_ang;
            if (sscanf(data.c_str(), "%lf %lf %lf", &trans_x, &trans_y, &trans_ang) == 3)
            {
                trans.origin_x = trans_x;
                trans.origin_y = trans_y;
                trans.angle_deg = trans_ang;
                double rad = to_rad(trans_ang);
                trans.sin_a = std::sin(rad);
                trans.cos_a = std::cos(rad);
                printf("Robot Position: X=%.2f, Y=%.2f, Angle=%.2f\n", trans.origin_x, trans.origin_y, trans.angle_deg);
            }
        }
        else
        {
            body_lines_raw.push_back(trimmed_line);
        }
        line_count++;
    }
    infile.close();

    if (output_filename.empty())
        output_filename = DEFAULT_OUTPUT_FILENAME;
    if (body_lines_raw.empty())
    {
        printf("Error: No coordinate lines found.\n");
        return 1;
    }

    // 2. Анализ строк
    std::vector<Line> line_list;
    bool has_park = false;
    double park_x = 0.0, park_y = 0.0;

    for (size_t i = 0; i < body_lines_raw.size(); i++)
    {
        std::string current_line = body_lines_raw[i];

        size_t c_pos = current_line.find(';');
        std::string data = (c_pos != std::string::npos) ? current_line.substr(0, c_pos) : current_line;
        std::string comment = (c_pos != std::string::npos) ? trim(current_line.substr(c_pos + 1)) : "";

        double val[4];
        int found = sscanf(data.c_str(), "%lf %lf %lf %lf", &val[0], &val[1], &val[2], &val[3]);

        if (i == body_lines_raw.size() - 1)
        {
            if (found == 2)
            {
                has_park = true;
                park_x = val[0];
                park_y = val[1];
                applyTransform(park_x, park_y, trans);
                printf("Parking Point Detected: CAD(%.2f, %.2f) -> ROBOT(%.3f, %.3f)\n", val[0], val[1], park_x, park_y);
                continue;
            }
        }

        if (found == 4)
        {
            line_list.push_back({val[0], val[1], val[2], val[3], comment});
        }
    }

    if (line_list.empty())
    {
        printf("Error: No valid drawing lines found.\n");
        return 1;
    }

    // 3. Генерация кода
    double first_x_local = line_list[0].x1;
    double first_y_local = line_list[0].y1;
    applyTransform(first_x_local, first_y_local, trans);

    GCodeGenerator gen(output_filename.c_str(), trans, first_x_local, first_y_local);
    if (!gen.is_ready())
        return 1;

    printf("Processing %zu lines...\n", line_list.size());

    for (const auto &l : line_list)
    {
        gen.processLine(l, trans);
    }

    // 4. Завершение
    gen.finalize(has_park, park_x, park_y);

    gen.appendVector(raw_input_lines);
    gen.closeFile();

    printf("Done. Output: %s\n", output_filename.c_str());
    return 0;
}