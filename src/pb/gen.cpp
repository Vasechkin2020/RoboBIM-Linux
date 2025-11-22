/*
 * File: gen.cpp
 * Version: 2.7 (2025-11-22)
 * Changes:
 * - RESTORED: Input file (input_lines.txt) is **created with sample data IF it does not exist**. If it exists, it is read normally.
 * - Output G-code file (output_filename) **always overwrites** any existing file (fopen mode "w").
 * - Full input_lines.txt content (including filename) is appended to G-code.
 * - Dynamic output filename based on the first line of input_lines.txt.
 * - Added code history section at the top of the file.
 */

#define _USE_MATH_DEFINES
#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <string>
#include <cstdio>
#include <sstream>

// === НАСТРОЙКИ ===
const double MARKER_OFFSET = 0.045; // Смещение маркера (double для точности)
const double SPEED_DRAW    = 0.10; // Скорость рисования (мм/с)
const double SPEED_MOVE    = 0.10; // Скорость холостого хода (мм/с)
const double SPEED_TURN    = 0.05; // Скорость поворота
const double SPEED_BACKUP  = -0.10; // Скорость отката
// Новая константа для имени файла по умолчанию
const char* DEFAULT_OUTPUT_FILENAME = "output_default.gcode"; 

struct Line {
    double x1, y1;
    double x2, y2;
};

// Перевод в градусы
double to_deg(double rad) {
    return rad * 180.0 / M_PI;
}

class GCodeGenerator {
private:
    FILE* fp;

public:
    // Конструктор
    GCodeGenerator(const char* filename) 
    {
        // Всегда открываем файл в режиме "w" (запись), что гарантирует перезапись
        fp = fopen(filename, "w"); 

        if (fp) {
            fprintf(fp, "; === GENERATED G-CODE (Debug Version) ===\n"); // Заголовок
            fprintf(fp, "; Output Filename: %s\n", filename); // Добавили имя файла в заголовок
            fprintf(fp, "; Marker Offset: %.4f\n", MARKER_OFFSET); // Смещение маркера
            fprintf(fp, "M5 T1000 ; Pen UP\n"); // Поднять инструмент
            fprintf(fp, "G1 A0.000 F%.2f\n", SPEED_TURN); // Установить угол 0
            fprintf(fp, "G2 L%.3f F%.2f ; Initial align\n\n", MARKER_OFFSET, SPEED_BACKUP); // Начальный откат
        } else {
             // Сообщаем об ошибке, если не удалось открыть файл для записи
            printf("Critical Error: Could not open output file %s for writing.\n", filename);
        }
    }

    ~GCodeGenerator() {
        if (fp) {
            // Закрытие файла перенесено в finalize()
        }
    }
    
    // Функция для проверки, был ли файл успешно открыт
    bool is_ready() const {
        return fp != nullptr; // Возвращает true, если fp не нулевой указатель
    }

    void processLine(Line l) {
        if (!fp) return;

        double dx = l.x2 - l.x1;
        double dy = l.y2 - l.y1;
        double angle_rad = std::atan2(dy, dx);
        double angle_deg = to_deg(angle_rad);

        fprintf(fp, "; >>> Line: (%.3f, %.3f) -> (%.3f, %.3f) | Angle: %.2f\n", 
                l.x1, l.y1, l.x2, l.y2, angle_deg); // Комментарий о текущей линии

        // 1. Едем центром к вершине НАЧАЛА линии
        fprintf(fp, "M5 T1000 ; Pen UP\n"); // Поднять инструмент
        fprintf(fp, "G2 X%.3f Y%.3f F%.2f ; Center to Start Vertex\n", l.x1, l.y1, SPEED_MOVE); // Переместить центр к начальной точке

        // 2. Поворот
        fprintf(fp, "G4 P200\n"); // Пауза
        fprintf(fp, "G1 A%.3f F%.2f\n", angle_deg, SPEED_TURN); // Поворот
        fprintf(fp, "G4 P200\n"); // Пауза

        // 3. Откат назад (L)
        fprintf(fp, "G2 L%.3f F%.2f ; Back up\n", MARKER_OFFSET, SPEED_BACKUP); // Откат для позиционирования маркера

        // 4. Расчет целевой точки ЦЕНТРА для конца линии
        double off_x = MARKER_OFFSET * std::cos(angle_rad); // Смещение по X
        double off_y = MARKER_OFFSET * std::sin(angle_rad); // Смещение по Y
        
        double target_x = l.x2 - off_x; // Целевая координата X центра
        double target_y = l.y2 - off_y; // Целевая координата Y центра

        // Отладочный комментарий в G-code
        fprintf(fp, "; Math: V(%.3f,%.3f) - Off(%.3f,%.3f) = C(%.3f,%.3f)\n", 
                l.x2, l.y2, off_x, off_y, target_x, target_y); // Подробный расчет в комментарии

        // 5. Рисование
        fprintf(fp, "M3 T1000 ; Pen DOWN\n"); // Опустить инструмент
        fprintf(fp, "G2 X%.3f Y%.3f F%.2f\n\n", target_x, target_y, SPEED_DRAW); // Рисование линии
    }
    
    // Функция для добавления исходного файла
    void appendInputFile(const char* input_filename) {
        if (!fp) return;
        
        std::ifstream infile(input_filename); // Открыть исходный файл
        std::string line; // Строка для чтения
        
        fprintf(fp, "\n; === ORIGINAL INPUT LINES START ===\n"); // Заголовок секции
        
        // Чтение файла построчно, включая первую строку (имя файла)
        while (std::getline(infile, line)) {
            // Запись в G-code с префиксом комментария
            fprintf(fp, "; %s\n", line.c_str()); 
        }
        
        fprintf(fp, "; === ORIGINAL INPUT LINES END ===\n"); // Конец секции
        infile.close();
    }
    
    // Функция для завершения G-code (парковки) и закрытия файла
    void finalize() {
        if (!fp) return;
        fprintf(fp, "\n; === END PROGRAM ===\n"); // Конец программы
        fprintf(fp, "M5 T1000\n"); // Поднять инструмент
        fprintf(fp, "G2 X0.000 Y0.000 F%.2f ; Park\n", SPEED_MOVE); // Парковка в (0,0)
        fclose(fp); // Закрыть файл
        fp = nullptr; // Сбросить указатель
    }
};

int main() {
    const char* INPUT_FILENAME = "input_lines.txt";
    std::string output_filename; 
    
    // 1. ПРОВЕРКА и СОЗДАНИЕ input_lines.txt (если нет)
    std::ifstream infile(INPUT_FILENAME); // Попытка открыть для чтения
    if (!infile.is_open()) {
        // Если файл не существует, создаем его
        printf("Info: Input file %s not found. Creating sample file.\n", INPUT_FILENAME); // Информационное сообщение
        std::ofstream outfile(INPUT_FILENAME); 
        if (outfile.is_open()) {
            outfile << "output_sample.gcode" << std::endl; // <-- Имя выходного файла
            outfile << "0.0 0.0 2.0 0.0" << std::endl;
            outfile << "2.0 0.0 2.0 1.0" << std::endl;
            outfile << "2.0 1.0 0.0 0.0" << std::endl;
            outfile.close();
        }
        // После создания, пытаемся снова открыть его для чтения
        infile.open(INPUT_FILENAME); 
        if (!infile.is_open()) {
            printf("Critical Error: Failed to open or create input file %s.\n", INPUT_FILENAME);
            return 1;
        }
    }
    // Если файл существовал, infile уже открыт.
    
    // 2. Чтение имени выходного файла из input_lines.txt
    std::string first_line;
    if (std::getline(infile, first_line) && !first_line.empty()) {
        output_filename = first_line; 
    } else {
        output_filename = DEFAULT_OUTPUT_FILENAME; 
        printf("Warning: First line of %s is empty. Using default output filename: %s\n", INPUT_FILENAME, DEFAULT_OUTPUT_FILENAME); // Предупреждение
    }

    // 3. Генерация G-code
    GCodeGenerator gen(output_filename.c_str()); 
    
    if (!gen.is_ready()) { 
        infile.close(); // Закрываем входной файл
        return 1; // Завершаем работу, если не удалось открыть файл для записи
    }
    
    // Возвращаем указатель в начало файла, чтобы пропустить имя файла
    infile.seekg(0, std::ios::beg); // Перемещаем указатель в начало файла

    // Снова считываем первую строку, чтобы пропустить ее и начать чтение координат
    std::getline(infile, first_line); 

    double x1, y1, x2, y2;
    // Считывание координат, начиная со второй строки (после имени файла)
    while (infile >> x1 >> y1 >> x2 >> y2) {
        gen.processLine({x1, y1, x2, y2});
    }

    infile.close(); 

    // 4. Добавление исходного кода и завершение
    gen.appendInputFile(INPUT_FILENAME); 
    gen.finalize(); 

    printf("Done. G-code generated (and overwritten) into %s\n", output_filename.c_str());
    return 0;
}