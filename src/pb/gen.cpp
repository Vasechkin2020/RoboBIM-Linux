#define _USE_MATH_DEFINES // Для M_PI в Windows
#include <iostream>       // Ввод-вывод
#include <fstream>        // Работа с файлами
#include <vector>         // Динамические массивы
#include <cmath>          // Математика (sin, cos, atan2)
#include <string>         // Строки
#include <cstdio>         // printf, fprintf

// === НАСТРОЙКИ РОБОТА ===
const float MARKER_OFFSET = 0.045f; // Смещение маркера вперед от центра (метры)
const float SPEED_DRAW    = 0.10f;  // Скорость рисования (м/с)
const float SPEED_MOVE    = 0.10f;  // Скорость холостого хода (м/с)
const float SPEED_TURN    = 0.05f;  // Скорость поворота (м/с)
const float SPEED_BACKUP  = -0.10f; // Скорость отката (отрицательная!)

// Структура для хранения одной линии
struct Line {
    float x1, y1; // Начало
    float x2, y2; // Конец
};

// Вспомогательная функция для перевода градусов в радианы
float to_rad(float deg) {
    return deg * M_PI / 180.0f;
}

// Вспомогательная функция для перевода радиан в градусы
float to_deg(float rad) {
    return rad * 180.0f / M_PI;
}

// Класс для генерации G-кода
class GCodeGenerator {
private:
    FILE* fp;             // Указатель на файл
    float current_x;      // Текущая X координата Центра Вращения (ЦВ)
    float current_y;      // Текущая Y координата ЦВ
    float current_angle;  // Текущий угол робота (градусы)

public:
    GCodeGenerator(const char* filename) {
        fp = fopen(filename, "w"); // Открываем файл для записи
        // Начальное состояние робота (как в GCodeParser)
        current_x = 0.0f;
        current_y = 0.0f;
        current_angle = 0.0f; 
        
        if (fp) {
            fprintf(fp, "; === GENERATED G-CODE ===\n");
            fprintf(fp, "; Marker Offset: %.3f\n", MARKER_OFFSET);
            fprintf(fp, "M5 T1000 ; Start with pen UP\n");
            // Инициализация: откат, чтобы маркер встал в 0,0 (если робот стартует в 0,0)
            fprintf(fp, "G1 A0.000 F%.2f\n", SPEED_TURN);
            fprintf(fp, "G2 L%.3f F%.2f ; Initial align to 0,0\n\n", MARKER_OFFSET, SPEED_BACKUP);
            // После этого отката ЦВ находится в (-0.045, 0.0)
            current_x = -MARKER_OFFSET;
        }
    }

    ~GCodeGenerator() {
        if (fp) {
            fprintf(fp, "\n; === END OF PROGRAM ===\n");
            // Возврат домой (опционально)
            fprintf(fp, "M5 T1000\n");
            fprintf(fp, "G2 X0.000 Y0.000 F%.2f ; Park Center at 0,0\n", SPEED_MOVE);
            fclose(fp);
        }
    }

    // Основная функция обработки линии
    void processLine(Line l) {
        if (!fp) return;

        fprintf(fp, "; --- Processing Line: (%.3f, %.3f) -> (%.3f, %.3f) ---\n", l.x1, l.y1, l.x2, l.y2);

        // 1. ПРОВЕРКА: Нужно ли ехать к началу линии?
        // Маркер находится там, где закончилась предыдущая линия.
        // Координата маркера сейчас = current_x + offset * cos(angle), но проще проверить разрыв.
        // Для простоты: мы всегда выполняем процедуру поворота в вершине.
        
        // Вычисляем угол линии
        float dx = l.x2 - l.x1;
        float dy = l.y2 - l.y1;
        float line_angle_rad = std::atan2(dy, dx);
        float line_angle_deg = to_deg(line_angle_rad);
        float line_len = std::sqrt(dx*dx + dy*dy);

        // === МАНЕВР В НАЧАЛЕ ЛИНИИ (Поворот и Откат) ===
        
        // Сначала доезжаем ЦВ до точки старта линии (l.x1, l.y1)
        // Если мы рисуем непрерывно, мы уже почти там, но нужно довести ЦВ точно в вершину.
        // Если это разрыв (новая линия далеко), эта команда довезет нас туда (с поднятым маркером).
        
        fprintf(fp, "M5 T1000 ; Pen UP\n");
        fprintf(fp, "G2 X%.3f Y%.3f F%.2f ; Move Center to Start Vertex\n", l.x1, l.y1, SPEED_MOVE);
        
        // Теперь ЦВ стоит в (x1, y1). Поворачиваемся на угол линии.
        fprintf(fp, "G4 P200\n");
        fprintf(fp, "G1 A%.3f F%.2f ; Turn to Line Angle\n", line_angle_deg, SPEED_TURN);
        fprintf(fp, "G4 P200\n");

        // Теперь делаем ОТКАТ НАЗАД, чтобы Маркер встал в (x1, y1).
        // Используем L и отрицательную скорость.
        fprintf(fp, "G2 L%.3f F%.2f ; Back up to place Marker at Start\n", MARKER_OFFSET, SPEED_BACKUP);

        // === РИСОВАНИЕ ЛИНИИ ===
        
        // Рассчитываем, где должен остановиться ЦВ, чтобы Маркер оказался в конце линии (l.x2, l.y2).
        // Target_COR = Target_Marker - Offset_Vector
        float target_cor_x = l.x2 - MARKER_OFFSET * std::cos(line_angle_rad);
        float target_cor_y = l.y2 - MARKER_OFFSET * std::sin(line_angle_rad);

        fprintf(fp, "M3 T1000 ; Pen DOWN\n");
        // Едем по координатам ЦВ. Драйвер сам посчитает расстояния.
        fprintf(fp, "G2 X%.3f Y%.3f F%.2f ; Draw Line\n\n", target_cor_x, target_cor_y, SPEED_DRAW);

        // Обновляем текущее состояние программы (где мы находимся сейчас)
        current_x = target_cor_x;
        current_y = target_cor_y;
        current_angle = line_angle_deg;
    }
};

int main() {
    /*
    // 1. Создаем пример файла с линиями, если его нет
    // Формат: x1 y1 x2 y2 (Конверт 2х1 с клапаном)
    std::ofstream outfile("input_lines.txt");
    if (outfile.is_open()) {
        outfile << "0.0 0.0 2.0 0.0" << std::endl; // Низ
        outfile << "2.0 0.0 2.0 1.0" << std::endl; // Право
        outfile << "2.0 1.0 0.0 0.0" << std::endl; // Диагональ 1
        outfile << "0.0 0.0 0.0 1.0" << std::endl; // Лево
        outfile << "0.0 1.0 2.0 1.0" << std::endl; // Верх
        outfile << "2.0 1.0 1.0 2.0" << std::endl; // Клапан Право
        outfile << "1.0 2.0 0.0 1.0" << std::endl; // Клапан Лево
        outfile << "0.0 1.0 2.0 0.0" << std::endl; // Диагональ 2
        outfile.close();
        printf("Created 'input_lines.txt' with sample envelope data.\n");
    }
    */
    // 2. Открываем файл на чтение
    std::ifstream infile("input_lines.txt");
    if (!infile.is_open()) {
        printf("Error: Could not open input_lines.txt\n");
        return 1;
    }

    // 3. Запускаем генератор
    GCodeGenerator gen("output_drawing.gcode");
    
    float x1, y1, x2, y2;
    int count = 0;

    printf("Reading lines and generating G-code...\n");
    
    // Читаем координаты построчно
    while (infile >> x1 >> y1 >> x2 >> y2) {
        Line l = {x1, y1, x2, y2};
        gen.processLine(l); // Генерируем код для линии
        count++;
    }

    infile.close();
    printf("Done! Processed %d lines. Output saved to 'output_drawing.gcode'.\n", count);
    printf("You can now upload 'output_drawing.gcode' to your robot.\n");

    return 0;
}