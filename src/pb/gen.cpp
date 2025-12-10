/*
 * File: gen.cpp
 * Version: 3.8 (2025-11-23)
 * Changes:
 * - FIXED Compilation Error: 'fp' is private. Moved the backup writing logic back into the class method 'appendVector'.
 * - Logic: main() reads the file into a vector, then passes this vector to GCodeGenerator to append at the end of the G-code.
 * - Added: G10 command at the start.
 * - Restored: processLine logic (draw -> finish).
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
const double SPEED_DRAW    = 0.10;
const double SPEED_MOVE    = 0.10;
const double SPEED_TURN    = 0.05;
const double SPEED_BACKUP  = -0.10;
const char* DEFAULT_OUTPUT_FILENAME = "output_default.gcode"; 

struct Line {
    double x1, y1;
    double x2, y2;
    std::string comment; 
};

struct Transform {
    double origin_x;
    double origin_y;
    double sin_a;
    double cos_a;
    double angle_deg;
};

double to_deg(double rad) { return rad * 180.0 / M_PI; }
double to_rad(double deg) { return deg * M_PI / 180.0; }

std::string trim(const std::string& str) {
    size_t first = str.find_first_not_of(" \t\r\n");
    if (std::string::npos == first) return str;
    size_t last = str.find_last_not_of(" \t\r\n");
    return str.substr(first, (last - first + 1));
}

// УТИЛИТА: Применяет трансформацию к координатам (глобальная -> локальная)
void applyTransform(double &x, double &y, const Transform &t) {
    double dx = x - t.origin_x; // Смещение относительно начала робота
    double dy = y - t.origin_y;

    double x_new = dx * t.cos_a - dy * t.sin_a; // Применение поворота
    double y_new = dx * t.sin_a + dy * t.cos_a;

    x = x_new;
    y = y_new;
}

class GCodeGenerator {
private:
    FILE* fp;
    Transform trans;

public:
    // Конструктор
    GCodeGenerator(const char* filename, const Transform &t, double first_x_local, double first_y_local) 
    : trans(t) 
    {
        fp = fopen(filename, "w");
        if (fp) {
            fprintf(fp, "; === GENERATED G-CODE (v3.8) ===\n"); 
            fprintf(fp, "; Output Filename: %s\n", filename);
            fprintf(fp, "; Robot System Coordinate Origin on Map: X=%.3f, Y=%.3f, Angle=%.1f\n", t.origin_x, t.origin_y, t.angle_deg);
            fprintf(fp, "; Marker Offset: %.4f\n", MARKER_OFFSET);

            // G10: Начальная точка ИЗМЕНЕНИЕ: Добавлен A (угол) в команду G10
            fprintf(fp, "G10 X%.3f Y%.3f A%.3f ; Initial required start point and angle in robot's local coordinates\n", 
                    first_x_local, first_y_local, t.angle_deg);
            // СТАРТОВЫЙ БЛОК НАСТРОЙКИ
            fprintf(fp, "M5 T2000 ; Pen UP\n");
            fprintf(fp, "G1 A0.000 F%.2f\n", SPEED_TURN);
            fprintf(fp, "G2 L%.3f F%.2f ; Initial align\n\n", MARKER_OFFSET, SPEED_BACKUP);
        } else {
            printf("Critical Error: Could not open output file %s.\n", filename);
        }
    }

    bool is_ready() const { return fp != nullptr; }

    void writeComment(const std::string& comment) {
        if (fp) fprintf(fp, "; %s\n", comment.c_str());
    }

    void processLine(Line l, const Transform &t) {
        if (!fp) return;

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
        if (!l.comment.empty()) fprintf(fp, "; %s\n", l.comment.c_str());
        
        fprintf(fp, "; CAD:   (%.3f, %.3f) -> (%.3f, %.3f)\n", raw_x1, raw_y1, raw_x2, raw_y2);
        fprintf(fp, "; ROBOT: (%.3f, %.3f) -> (%.3f, %.3f) | Ang: %.2f\n", l.x1, l.y1, l.x2, l.y2, angle_deg);

        // 1. Центр к началу
        fprintf(fp, "M5 T2000\n");
        fprintf(fp, "G2 X%.3f Y%.3f F%.2f ; Center to Start\n", l.x1, l.y1, SPEED_MOVE);

        // 2. Поворот
        fprintf(fp, "G4 P2000\n");
        fprintf(fp, "G1 A%.3f F%.2f\n", angle_deg, SPEED_TURN);
        fprintf(fp, "G4 P2000\n");

        // 3. Откат
        fprintf(fp, "G2 L%.3f F%.2f ; Back up\n", MARKER_OFFSET, SPEED_BACKUP);

        // 4. Цель (с компенсацией)
        double off_x = MARKER_OFFSET * std::cos(angle_rad);
        double off_y = MARKER_OFFSET * std::sin(angle_rad);
        double target_x = l.x2 - off_x; 
        double target_y = l.y2 - off_y;

        // 5. Рисование
        fprintf(fp, "M3 T2000\n");
        fprintf(fp, "G2 X%.3f Y%.3f F%.2f\n\n", target_x, target_y, SPEED_DRAW);
    }
    
    // НОВЫЙ МЕТОД: Принимает готовый вектор строк
    void appendVector(const std::vector<std::string>& lines) {
        if (!fp) return;
        fprintf(fp, "\n; === ORIGINAL INPUT LINES START (Input backup) ===\n");
        for (const auto& line : lines) {
             fprintf(fp, "; %s\n", line.c_str());
        }
        fprintf(fp, "; === ORIGINAL INPUT LINES END ===\n");
    }
    
    void finalize() {
        if (!fp) return;
        fprintf(fp, "\n; === END PROGRAM (Final commands before Input backup) ===\n"); 
        fprintf(fp, "M5 T2000\n"); 
        fprintf(fp, "G1 A0.000 F%.2f ; Final rotation to 0 degrees (Safe finish in place)\n", SPEED_TURN); 
    }

    void closeFile() {
        if (fp) {
            fclose(fp);
            fp = nullptr;
        }
    }
};

int main() {
    const char* INPUT_FILENAME = "input_lines.txt";
    std::string output_filename; 
    
    Transform trans = {0.0, 0.0, 0.0, 0.0, 0.0};
    
    std::vector<Line> line_list;
    std::vector<std::string> raw_input_lines; // Храним исходные строки

    // 1. Создание примера (если нет файла)
    std::ifstream infile_check(INPUT_FILENAME);
    if (!infile_check.is_open()) {
        printf("Info: Creating sample %s.\n", INPUT_FILENAME);
        std::ofstream outfile(INPUT_FILENAME); 
        if (outfile.is_open()) {
            outfile << "krvadratL.gcode ; Filename" << std::endl;
            outfile << "1.0 2.0 0.0 ; Робот стоит в X=1.0, Y=2.0" << std::endl;
            outfile << "; Квадрат 1x1, который начинается там же, где робот" << std::endl;
            outfile << "1.0 2.0 2.0 2.0" << std::endl;
            outfile << "2.0 2.0 2.0 3.0" << std::endl;
            outfile << "2.0 3.0 1.0 3.0" << std::endl;
            outfile << "1.0 3.0 1.0 2.0" << std::endl;
            outfile.close();
        }
    }

    // 2. Чтение файла в память
    std::ifstream infile(INPUT_FILENAME); 
    if (!infile.is_open()) return 1;
    
    std::string line;
    int line_count = 0;
    
    // Чтение всего файла построчно
    while (std::getline(infile, line)) {
        raw_input_lines.push_back(line); // Сохраняем "сырую" строку для бэкапа
        
        std::string trimmed_line = trim(line);
        if (trimmed_line.empty()) continue; 
        
        size_t comment_pos = trimmed_line.find(';');
        std::string data = (comment_pos != std::string::npos) ? trimmed_line.substr(0, comment_pos) : trimmed_line;
        data = trim(data);
        if (data.empty()) continue;

        if (line_count == 0) {
            output_filename = data;
        } else if (line_count == 1) {
            double trans_x, trans_y, trans_ang;
            if (sscanf(data.c_str(), "%lf %lf %lf", &trans_x, &trans_y, &trans_ang) == 3) {
                trans.origin_x = trans_x;
                trans.origin_y = trans_y;
                trans.angle_deg = trans_ang;
                
                double rad = to_rad(trans_ang);
                trans.sin_a = std::sin(rad);
                trans.cos_a = std::cos(rad);
                
                printf("Robot Position Loaded: X=%.2f, Y=%.2f, Angle=%.2f\n", trans.origin_x, trans.origin_y, trans.angle_deg);
            } else {
                printf("Warning: Invalid transformation line. Using 0 0 0.\n");
            }
        } else {
            // Чтение координатных линий
            double x1, y1, x2, y2;
            std::string comment_part = (comment_pos != std::string::npos) ? trim(trimmed_line.substr(comment_pos + 1)) : "";

            if (sscanf(data.c_str(), "%lf %lf %lf %lf", &x1, &y1, &x2, &y2) == 4) {
                line_list.push_back({x1, y1, x2, y2, comment_part});
            } else if (!data.empty()) {
                printf("Warning: Skipped line data: %s\n", data.c_str());
            }
        }
        line_count++;
    }
    infile.close(); 
    
    if (output_filename.empty()) output_filename = DEFAULT_OUTPUT_FILENAME;
    if (line_list.empty()) {
        printf("Error: No coordinate lines found to process.\n");
        return 1;
    }

    // 3. Вычисление локальных координат первой точки
    double first_x_local = line_list[0].x1;
    double first_y_local = line_list[0].y1;
    applyTransform(first_x_local, first_y_local, trans);

    // 4. Инициализация (передаем первую точку для G10)
    GCodeGenerator gen(output_filename.c_str(), trans, first_x_local, first_y_local);
    if (!gen.is_ready()) return 1;
    
    printf("Processing coordinates...\n");

    // 5. Обработка всех линий
    for (const auto& l : line_list) {
        gen.processLine(l, trans);
    }

    // 6. Завершение программы
    gen.finalize(); 
    
    // Передаем вектор строк внутрь класса для записи
    gen.appendVector(raw_input_lines);

    gen.closeFile(); // Закрываем файл

    printf("Done. Output: %s\n", output_filename.c_str());
    return 0;
}