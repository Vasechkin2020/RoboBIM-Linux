/*
 * File: gen.cpp
 * Version: 3.9.7 (2025-11-23)
 * Changes:
 * - FIXED: Mismatched coordinates between G1 (Rotate) and G2 (Draw).
 * - Logic: 
 *      1. Calculate 'target_x/y' (compensated for offset) at the very beginning.
 *      2. Use 'target_x/y' for BOTH the rotation (G1) and the drawing move (G2).
 *      3. Result: G1 and G2 commands now show identical coordinates.
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

// Используется в команде G2 после M3 (когда перо опущено).
// Это F параметр для рисования линии.
const double SPEED_DRAW    = 0.10;

// Используется в командах G2 при перелетах (Jump), выравнивании центра (Align)
// и закрытии офсета (Undo offset), когда перо поднято (M5).
const double SPEED_MOVE    = 0.10;

// Используется в командах G1 A... 
// Определяет скорость вращения колес при развороте робота на месте.
const double SPEED_TURN    = 0.05;

// Используется в командах G2 L... для отката назад.
// Отрицательное значение F говорит парсеру, что ехать нужно назад.
const double SPEED_BACKUP  = -0.10;

const int    DWELL_TIME    = 2000; // Пауза в миллисекундах (для G4 и M3/M5)
const char* DEFAULT_OUTPUT_FILENAME = "output_smart.gcode"; 

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

void applyTransform(double &x, double &y, const Transform &t) {
    double dx = x - t.origin_x;
    double dy = y - t.origin_y;
    double x_new = dx * t.cos_a - dy * t.sin_a;
    double y_new = dx * t.sin_a + dy * t.cos_a;
    x = x_new;
    y = y_new;
}

class GCodeGenerator {
private:
    FILE* fp;
    Transform trans;
    
    double last_endpoint_x;
    double last_endpoint_y;
    bool is_first_line;
    bool has_drawn_anything;

public:
    GCodeGenerator(const char* filename, const Transform &t, double first_x_local, double first_y_local) 
    : trans(t), last_endpoint_x(0), last_endpoint_y(0), is_first_line(true), has_drawn_anything(false)
    {
        fp = fopen(filename, "w");
        if (fp) {
            fprintf(fp, "; === GENERATED G-CODE (v3.9.7 Coords Match) ===\n"); 
            fprintf(fp, "; Output Filename: %s\n", filename);
            fprintf(fp, "; Marker Offset: %.4f\n", MARKER_OFFSET);
            fprintf(fp, "; Dwell Time: %d ms\n", DWELL_TIME);

            fprintf(fp, "G10 X%.3f Y%.3f A%.3f ; Initial pose set\n", 
                    first_x_local, first_y_local, t.angle_deg);
            
            fprintf(fp, "M5 T%d ; Pen UP\n", DWELL_TIME);
        } else {
            printf("Critical Error: Could not open output file %s.\n", filename);
        }
    }

    bool is_ready() const { return fp != nullptr; }

    void processLine(Line l, const Transform &t) {
        if (!fp) return;

        double raw_x1 = l.x1, raw_y1 = l.y1; 
        double raw_x2 = l.x2, raw_y2 = l.y2;
        
        applyTransform(l.x1, l.y1, t);
        applyTransform(l.x2, l.y2, t);

        // 1. Вычисляем геометрию и ЦЕЛЕВУЮ ТОЧКУ (target) заранее
        double dx = l.x2 - l.x1;
        double dy = l.y2 - l.y1;
        double angle_rad = std::atan2(dy, dx);

        // Рассчитываем, где остановится ЦЕНТР робота (Target), чтобы перо попало в l.x2/l.y2
        double off_x = MARKER_OFFSET * std::cos(angle_rad);
        double off_y = MARKER_OFFSET * std::sin(angle_rad);
        double target_x = l.x2 - off_x; 
        double target_y = l.y2 - off_y;
        
        fprintf(fp, "; ---------------------------------------------------\n");
        if (!l.comment.empty()) fprintf(fp, "; %s\n", l.comment.c_str());
        fprintf(fp, "; CAD: (%.3f, %.3f) -> (%.3f, %.3f)\n", raw_x1, raw_y1, raw_x2, raw_y2);

        // ЛОГИКА ПЕРЕХОДОВ
        if (is_first_line) {
            // === СЛУЧАЙ 1: ПЕРВАЯ ЛИНИЯ ===
            fprintf(fp, "; Case: First Line\n");
            // Поворачиваемся сразу на TARGET (туда, куда поедем)
            fprintf(fp, "G1 X%.3f Y%.3f F%.2f ; Rotate to Target\n", target_x, target_y, SPEED_TURN); 
            fprintf(fp, "G4 P%d\n", DWELL_TIME);
            fprintf(fp, "G2 L%.3f F%.2f ; Initial Backup\n", MARKER_OFFSET, SPEED_BACKUP); 

            is_first_line = false;

        } else {
            double dist = std::sqrt(std::pow(l.x1 - last_endpoint_x, 2) + std::pow(l.y1 - last_endpoint_y, 2));
            bool is_connected = (dist < 0.001);

            if (is_connected) {
                // === СЛУЧАЙ 2: СВЯЗНАЯ ЛИНИЯ ===
                fprintf(fp, "; Case: Connected Corner\n");
                fprintf(fp, "M5 T%d\n", DWELL_TIME);
                
                fprintf(fp, "G2 L%.3f F%.2f ; Step into corner\n", MARKER_OFFSET, SPEED_MOVE);
                
                fprintf(fp, "G4 P%d\n", DWELL_TIME);
                // Поворачиваемся на TARGET
                fprintf(fp, "G1 X%.3f Y%.3f F%.2f ; Rotate to Target\n", target_x, target_y, SPEED_TURN);
                fprintf(fp, "G4 P%d\n", DWELL_TIME);

                fprintf(fp, "G2 L%.3f F%.2f ; Backup\n", MARKER_OFFSET, SPEED_BACKUP);

            } else {
                // === СЛУЧАЙ 3: РАЗРЫВ ===
                fprintf(fp, "; Case: Jump to new position\n");
                fprintf(fp, "M5 T%d\n", DWELL_TIME);

                fprintf(fp, "G2 L%.3f F%.2f ; Undo offset\n", MARKER_OFFSET, SPEED_MOVE);
                
                // Здесь поворот на новую СТАРТОВУЮ точку (она без оффсета, т.к. мы туда едем центром)
                fprintf(fp, "G4 P%d\n", DWELL_TIME);
                fprintf(fp, "G1 X%.3f Y%.3f F%.2f ; Rotate to New Start\n", l.x1, l.y1, SPEED_TURN);
                
                fprintf(fp, "G4 P%d\n", DWELL_TIME);
                fprintf(fp, "G2 X%.3f Y%.3f F%.2f ; Fly to New Start\n", l.x1, l.y1, SPEED_MOVE);
                
                // А здесь прицеливаемся уже на TARGET новой линии
                fprintf(fp, "G4 P%d\n", DWELL_TIME);
                fprintf(fp, "G1 X%.3f Y%.3f F%.2f ; Align to Target\n", target_x, target_y, SPEED_TURN);

                fprintf(fp, "G4 P%d\n", DWELL_TIME);
                fprintf(fp, "G2 L%.3f F%.2f ; Backup\n", MARKER_OFFSET, SPEED_BACKUP);
            }
        }

        // РИСОВАНИЕ
        // Теперь координаты в G1 (выше) и G2 (здесь) совпадают
        fprintf(fp, "M3 T%d\n", DWELL_TIME);
        fprintf(fp, "G2 X%.3f Y%.3f F%.2f\n\n", target_x, target_y, SPEED_DRAW);

        last_endpoint_x = l.x2; 
        last_endpoint_y = l.y2;
        has_drawn_anything = true;
    }
    
    void finalize(bool use_parking, double park_x, double park_y) {
        if (!fp) return;
        fprintf(fp, "; === FINALIZE SEQUENCE ===\n"); 
        fprintf(fp, "M5 T%d ; Pen UP\n", DWELL_TIME); 

        // ШАГ 1: Компенсация "недоезда"
        if (has_drawn_anything) {
            fprintf(fp, "G2 L%.3f F%.2f ; Align Center\n", MARKER_OFFSET, SPEED_MOVE);
            fprintf(fp, "G4 P%d\n", DWELL_TIME);
        }

        // ШАГ 2: Парковка
        if (use_parking && has_drawn_anything) {
            // Поворот на ПАРКОВКУ
            fprintf(fp, "G1 X%.3f Y%.3f F%.2f ; Rotate to Park\n", park_x, park_y, SPEED_TURN);
            fprintf(fp, "G4 P%d\n", DWELL_TIME);
            
            // Едем на парковку
            fprintf(fp, "G2 X%.3f Y%.3f F%.2f ; Move to Park\n", park_x, park_y, SPEED_MOVE);
            fprintf(fp, "G4 P%d\n", DWELL_TIME);
        }

        fprintf(fp, "G1 A0.000 F%.2f ; Final rotation\n", SPEED_TURN); 
        fprintf(fp, "\n; === END PROGRAM ===\n"); 
    }

    void appendVector(const std::vector<std::string>& lines) {
        if (!fp) return;
        fprintf(fp, "\n; === ORIGINAL INPUT LINES START ===\n");
        for (const auto& line : lines) fprintf(fp, "; %s\n", line.c_str());
        fprintf(fp, "; === ORIGINAL INPUT LINES END ===\n");
    }

    void closeFile() {
        if (fp) { fclose(fp); fp = nullptr; }
    }
};

int main() {
    const char* INPUT_FILENAME = "input_lines.txt";
    std::string output_filename; 
    
    Transform trans = {0.0, 0.0, 0.0, 0.0, 0.0};
    std::vector<std::string> raw_input_lines;
    std::vector<std::string> body_lines_raw; 

    std::ifstream infile(INPUT_FILENAME); 
    if (!infile.is_open()) {
        printf("Error: Could not open %s\n", INPUT_FILENAME);
        return 1;
    }
    
    std::string line;
    int line_count = 0;
    while (std::getline(infile, line)) {
        raw_input_lines.push_back(line); 
        std::string trimmed_line = trim(line);
        if (trimmed_line.empty()) continue; 
        
        size_t comment_pos = trimmed_line.find(';');
        std::string data = (comment_pos != std::string::npos) ? trimmed_line.substr(0, comment_pos) : trimmed_line;
        data = trim(data);
        if (data.empty()) continue;

        if (line_count == 0) output_filename = data;
        else if (line_count == 1) {
            double tx, ty, ta;
            if (sscanf(data.c_str(), "%lf %lf %lf", &tx, &ty, &ta) == 3) {
                trans.origin_x = tx; trans.origin_y = ty; trans.angle_deg = ta;
                double rad = to_rad(ta);
                trans.sin_a = std::sin(rad); trans.cos_a = std::cos(rad);
            }
        } else {
            body_lines_raw.push_back(trimmed_line);
        }
        line_count++;
    }
    infile.close(); 
    
    if (output_filename.empty()) output_filename = DEFAULT_OUTPUT_FILENAME;
    if (body_lines_raw.empty()) return 1;

    std::vector<Line> line_list;
    bool has_park = false;
    double park_x = 0.0, park_y = 0.0;

    for (size_t i = 0; i < body_lines_raw.size(); i++) {
        std::string current_line = body_lines_raw[i];
        size_t c_pos = current_line.find(';');
        std::string data = (c_pos != std::string::npos) ? current_line.substr(0, c_pos) : current_line;
        std::string comment = (c_pos != std::string::npos) ? trim(current_line.substr(c_pos + 1)) : "";
        
        double val[4];
        int found = sscanf(data.c_str(), "%lf %lf %lf %lf", &val[0], &val[1], &val[2], &val[3]);

        if (i == body_lines_raw.size() - 1 && found == 2) {
            has_park = true;
            park_x = val[0]; park_y = val[1];
            applyTransform(park_x, park_y, trans);
            continue; 
        }
        if (found == 4) line_list.push_back({val[0], val[1], val[2], val[3], comment});
    }

    if (line_list.empty()) return 1;

    double first_x = line_list[0].x1;
    double first_y = line_list[0].y1;
    applyTransform(first_x, first_y, trans);

    GCodeGenerator gen(output_filename.c_str(), trans, first_x, first_y);
    if (!gen.is_ready()) return 1;
    
    printf("Processing %zu lines...\n", line_list.size());
    for (const auto& l : line_list) gen.processLine(l, trans);

    gen.finalize(has_park, park_x, park_y); 
    gen.appendVector(raw_input_lines);
    gen.closeFile();

    printf("Done. Output: %s\n", output_filename.c_str());
    return 0;
}