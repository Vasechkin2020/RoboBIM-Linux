#ifndef TRIALATERATIONSOLVER_H
#define TRIALATERATIONSOLVER_H



#include "config.h"
#include "code.h"
#include <stdio.h> // Подключение стандартной библиотеки ввода/вывода
#include <math.h> // Подключение математической библиотеки
#include <vector> // Подключение библиотеки для работы с векторами
#include <cmath> // Подключение современной математической библиотеки
#include <stdexcept> // Подключение для обработки стандартных исключений
#include <algorithm> // Подключение для стандартных алгоритмов

// --- Вспомогательные структуры и функции ---

struct SCircle_W // Структура для хранения окружности (Маяк)
{
    double x; // Координата x центра
    double y; // Координата y центра
    double r; // Радиус
    double weight_factor; // ФАКТОР ВЕСА: 1.0 для дистанции, меньше для угла
}; 

double sqr(double val) 
{
    return val * val; // Функция для вычисления квадрата числа
} 
double ctan(double rad) 
{
    return 1.0 / tan(rad); // Функция для вычисления котангенса
} 

// --- Основной класс решателя (ТОЛЬКО ОБЪЯВЛЕНИЕ) ---

class TrilaterationSolver
{
private:
    SPoint A_prev; // Предыдущая (или начальная) точка для выбора дуги
    std::vector<SCircle_W> all_circles; // Список всех окружностей (из расстояний и углов)
    
    double weight_angle_factor; // КОНСТАНТА НАСТРОЙКИ: Вес для окружностей, полученных из угла

    // Приватные вспомогательные методы
    double get_azimuth_deg(SPoint P_from, SPoint P_to); // Расчет азимута от одной точки к другой (для ориентации)
    double normalize_angle_deg(double angle_deg); // Нормализация угла в диапазон (-180, 180]

public:
    TrilaterationSolver(SPoint prev, double angle_weight = 0.1); // Конструктор, инициализирующий A_prev и вес угла

    // Методы управления состоянием
    void set_A_prev(SPoint new_prev); // Установка нового предыдущего положения (для фильтрации окружностей)
    void clear_circles(); // Очистка массива окружностей перед новым расчетом (для многократного использования)
    void set_angle_weight_factor(double factor); // Установка нового коэффициента веса угла

    // Методы добавления данных
    int get_circle_count() const; // Получить текущее количество окружностей в массиве
    void add_circle_from_distance(SPoint P_beacon, double distance); // Добавление окружности, полученной из измеренной дальности
    void add_filtered_circle_from_angle(SPoint P1, SPoint P2, double angle_deg); // Добавление окружности, полученной из измеренного угла (с фильтрацией по A_prev)

    // Методы решения
    SPoint_Q find_A_by_mnk(); // Решение методом наименьших квадратов (МНК) для определения положения A
    double get_lidar_orientation(const SPoint A_found, const std::vector<SPoint>& beacons, const std::vector<double>& lidar_angles_deg); // Определение ориентации лидара (Psi) через векторное усреднение
    
    // МЕТОДЫ РАСЧЕТА УГЛОВ
    double calculate_angle_BAC(SPoint A, SPoint B, SPoint C); // Расчет горизонтального угла BAC по координатам (возвращает -180, 180])
    double calculate_angle_from_azimuths(double az_AB, double az_AC); // Расчет угла BAC по двум азимутам (возвращает -180, 180])
};

// --- РЕАЛИЗАЦИЯ МЕТОДОВ КЛАССА TrilaterationSolver ---

// Конструктор
TrilaterationSolver::TrilaterationSolver(SPoint prev, double angle_weight) : 
    A_prev(prev), 
    weight_angle_factor(angle_weight) 
{
} // Инициализация A_prev и weight_angle_factor

// Приватные методы
double TrilaterationSolver::get_azimuth_deg(SPoint P_from, SPoint P_to) 
{
    double rad = atan2(P_to.y - P_from.y, P_to.x - P_from.x); // Азимут в радианах
    return RAD2DEG(rad); // Азимут в градусах
} 

double TrilaterationSolver::normalize_angle_deg(double angle_deg) 
{
    while (angle_deg > 180.0) // Пока угол больше 180 градусов
    {
        angle_deg -= 360.0; // Вычитаем 360 градусов
    } 
    while (angle_deg <= -180.0) // Пока угол меньше или равен -180 градусов
    {
        angle_deg += 360.0; // Прибавляем 360 градусов
    } 
    return angle_deg; // Возвращаем нормализованный угол
} 

// Публичные методы управления состоянием
void TrilaterationSolver::set_A_prev(SPoint new_prev) 
{
    A_prev = new_prev; // Устанавливаем новое предыдущее положение
    printf("A_prev updated to: (%.3f, %.3f)\n", A_prev.x, A_prev.y); // Output updated A_prev
} 

void TrilaterationSolver::clear_circles() 
{
    all_circles.clear(); // Очистка вектора all_circles
    printf("\nAll circles cleared for new calculation.\n"); // Output confirmation
}

void TrilaterationSolver::set_angle_weight_factor(double factor) 
{
    weight_angle_factor = factor; // Устанавливаем новый коэффициент
    printf("Angle weight factor updated to: %.4f\n", weight_angle_factor); // Output updated factor
}

// Публичные методы добавления данных
int TrilaterationSolver::get_circle_count() const 
{ 
    return all_circles.size(); // Получить количество окружностей
} 

void TrilaterationSolver::add_circle_from_distance(SPoint P_beacon, double distance) 
{
    // Вес дистанционного измерения всегда 1.0 (Базовый вес)
    all_circles.push_back({P_beacon.x, P_beacon.y, distance, 1.0}); // Добавляем маяк как центр с весом 1.0
} 

// Добавление по углу (С ИСПРАВЛЕННОЙ ЛОГИКОЙ ФИЛЬТРАЦИИ И ОТЛАДОЧНЫМ ВЫВОДОМ)
void TrilaterationSolver::add_filtered_circle_from_angle 
(
    SPoint P1,         // Точка 1 (начало хорды)
    SPoint P2,         // Точка 2 (конец хорды)
    double angle_deg          // Измеренный угол (АБСОЛЮТНОЕ значение)
)
{
    printf("\n--- ANGLE CIRCLE CALCULATION (P1=(%.2f, %.2f), P2=(%.2f, %.2f), Angle=%.2f) ---\n", // Output start
           P1.x, P1.y, P2.x, P2.y, angle_deg); 
    // printf("A_prev: (%.3f, %.3f), Angle Weight: %.4f\n", A_prev.x, A_prev.y, weight_angle_factor); // Output A_prev and weight

    // 1. Геометрические расчеты
    double alpha_rad = DEG2RAD(angle_deg); // Угол в радианах (используем исходный, даже если > 90)
    double sin_alpha = sin(alpha_rad); // Синус угла
    
    if (fabs(sin_alpha) < 1e-7) // Проверка на вырожденный случай
    {
        printf("Error: Angle is too close to 0 or 180 (sin_alpha < 1e-7).\n"); // Output error
        throw std::invalid_argument("Error: Angle is too close to 0 or 180."); 
    } 

    double cot_alpha = ctan(alpha_rad); // Котангенс угла (может быть отрицательным для > 90)
    double dx = P2.x - P1.x; // Разница по X
    double dy = P2.y - P1.y; // Разница по Y
    double d_sq = sqr(dx) + sqr(dy); // Квадрат длины хорды P1-P2
    
    if (d_sq < 1e-12) // Проверка на совпадение точек
    {
        printf("Error: Chord points P1 and P2 coincide.\n"); // Output error
        throw std::invalid_argument("Error: Chord points P1 and P2 coincide."); 
    } 

    double d = sqrt(d_sq); // Длина хорды
    
    double h = (d / 2.0) * cot_alpha; // Смещение от середины (МОЖЕТ БЫТЬ ОТРИЦАТЕЛЬНЫМ!)
    double R = d / (2.0 * fabs(sin_alpha)); // Радиус окружности (всегда положительный)
    
    // printf("Chord Length (d): %.3f, Radius (R): %.3f, Offset (h): %.3f\n", d, R, h); // Output geometry params

    SPoint M = {(P1.x + P2.x) / 2.0, (P1.y + P2.y) / 2.0}; // Середина хорды
    double vx = -dy / d; // Вектор, перпендикулярный хорде (x-компонента) (90 deg CCW)
    double vy = dx / d; // Вектор, перпендикулярный хорде (y-компонента)

    // Два потенциальных центра окружности
    SPoint O1 = {M.x + h * vx, M.y + h * vy}; // Центр 1
    SPoint O2 = {M.x - h * vx, M.y - h * vy}; // Центр 2 (симметричный)
    
    // --- ИСПРАВЛЕННАЯ ЛОГИКА ФИЛЬТРАЦИИ: Минимизировать отклонение от радиуса ---
    
    // Расстояние от O1 до A_prev
    double dist1 = sqrt(sqr(O1.x - A_prev.x) + sqr(O1.y - A_prev.y)); 
    // Расстояние от O2 до A_prev
    double dist2 = sqrt(sqr(O2.x - A_prev.x) + sqr(O2.y - A_prev.y)); 

    // Отклонение расстояния от радиуса R: |D(O_i, A_prev) - R|
    double deviation1 = fabs(dist1 - R); // Насколько Dist1 близко к R
    double deviation2 = fabs(dist2 - R); // Насколько Dist2 близко к R

    // printf("Center O1: (%.3f, %.3f) -> Dist to A_prev: %.3f. Deviation: %.3f\n", O1.x, O1.y, dist1, deviation1); 
    // printf("Center O2: (%.3f, %.3f) -> Dist to A_prev: %.3f. Deviation: %.3f\n", O2.x, O2.y, dist2, deviation2); 
    
    struct SCircle_W chosen_circle; // Выбранная окружность
    
    // Выбираем центр с минимальным отклонением от радиуса R
    if (deviation1 < deviation2) 
    {
        chosen_circle = {O1.x, O1.y, R, weight_angle_factor}; // Выбираем O1
        // printf("Chosen Center: O1 (Min Deviation from R)\n"); // Output chosen center
    } 
    else 
    {
        chosen_circle = {O2.x, O2.y, R, weight_angle_factor}; // Выбираем O2
        // printf("Chosen Center: O2 (Min Deviation from R)\n"); // Output chosen center
    } 
    
    all_circles.push_back(chosen_circle); // Добавляем выбранную окружность
    printf("Added Circle: Center (%.3f, %.3f), R=%.3f, W=%.4f\n", // Output final circle parameters
           chosen_circle.x, chosen_circle.y, chosen_circle.r, chosen_circle.weight_factor);
    // printf("------------------------------------------------------------------------\n"); // Output end
} 

// Публичный метод решения положения (ВЗВЕШЕННЫЙ МНК)
SPoint_Q TrilaterationSolver::find_A_by_mnk()
{
    SPoint A = {0.0, 0.0}; 
    SPoint_Q AQ;
    AQ.A = A;
    AQ.quality = 0;

    int N = all_circles.size(); 
    
    printf("\n--- SOLVER: START (N=%d) Aprev x= %6.3f y= %6.3f ---\n", N, A_prev.x, A_prev.y); 
    
    if (N < 3) 
    {
        printf("Error: At least 3 circles are required.\n"); 
        return AQ; 
    } 

    const struct SCircle_W& base = all_circles[0]; 
    double base_term = sqr(base.x) + sqr(base.y) - sqr(base.r); 
    
    double M_xx = 0.0, M_xy = 0.0, M_yy = 0.0; 
    double V_x = 0.0, V_y = 0.0; 

    // 1. Накопление матриц (Линеаризация и построение нормальных уравнений)
    for (int i = 1; i < N; ++i) 
    {
        const struct SCircle_W& current = all_circles[i]; 
        double Wi = current.weight_factor; // Вес текущей окружности
        
        double Ai = 2.0 * (current.x - base.x); 
        double Bi = 2.0 * (current.y - base.y); 

        double current_term = sqr(current.x) + sqr(current.y) - sqr(current.r); 
        double Ci = current_term - base_term; 
        
        // Взвешенные коэффициенты для WLS
        double WAi = Wi * Ai; 
        double WBi = Wi * Bi; 
        double WCi = Wi * Ci; 

        // Накопление элементов M^T*W*M
        M_xx += Ai * WAi; 
        M_xy += Bi * WAi; 
        M_yy += Bi * WBi; 
        
        // Накопление элементов M^T*W*C
        V_x += Ai * WCi; 
        V_y += Bi * WCi; 
    } 

    // 2. Решение СЛАУ (2x2)
    double det = M_xx * M_yy - M_xy * M_xy; 
    if (fabs(det) < 1e-9) 
    {
        printf("Error: Determinant (%.2e) is near zero. Solution is unstable.\n", det); 
        return AQ; 
    } 

    double InvDet = 1.0 / det; 
    A.x = InvDet * (M_yy * V_x - M_xy * V_y); 
    A.y = InvDet * (M_xx * V_y - M_xy * V_x); 
    
    printf("Found A (WLS): (%.3f, %.3f)\n", A.x, A.y); 

    // 3. Вычисление Геометрического RMS
    double sum_sq_residual_geom = 0.0; 
    for (int i = 0; i < N; ++i) 
    {
        const struct SCircle_W& current = all_circles[i]; 
        double r_calc = sqrt(sqr(A.x - current.x) + sqr(A.y - current.y)); 
        double residual_geom = current.r - r_calc; 
        sum_sq_residual_geom += sqr(residual_geom); 

        printf("Beacon %d (W=%.2f, R=%.3f): Geom. Residual=%.3f m\n", i, current.weight_factor, current.r, residual_geom); 
    }

    int degrees_of_freedom = N - 3; 
    double rmse_geom = 0;
    if (degrees_of_freedom > 0) 
    {
        rmse_geom = sqrt(sum_sq_residual_geom / degrees_of_freedom); 
        
        printf("\n--- Quality Assessment (N=%d, DoF=%d) ", N, degrees_of_freedom); 
        printf(" Geometric RMS (unweighted): **%6.3f m**", rmse_geom); 
    }
    else
    {
        printf("\nSolution is exactly determined (RMSE = N/A).\n"); 
    }

    printf("--- SOLVER: END ---\n"); 
    
    AQ.A = A;
    AQ.quality = rmse_geom; 

    return AQ; 
} 

// Публичный метод определения ориентации (без изменений)
double TrilaterationSolver::get_lidar_orientation(
    const SPoint A_found,                          
    const std::vector<SPoint>& beacons,           
    const std::vector<double>& lidar_angles_deg          
)
{
    if (beacons.size() != lidar_angles_deg.size() || beacons.empty()) 
    {
        printf("\n--- ORIENTATION CALCULATION SKIPPED ---\n"); 
        printf("Error: Missing or mismatched data for orientation calculation.\n"); 
        return 0.0; 
    } 

    printf("\n--- ORIENTATION LSQ CALCULATION ---\n"); 
    
    double sum_sin = 0.0; 
    double sum_cos = 0.0; 
    int N = beacons.size(); 
    
    for (int i = 0; i < N; ++i) 
    {
        double alpha_AM_deg = get_azimuth_deg(A_found, beacons[i]); 
        double theta_lidar_deg = lidar_angles_deg[i]; 

        double psi_raw_deg = alpha_AM_deg - theta_lidar_deg; 
        double psi_norm_deg = normalize_angle_deg(psi_raw_deg); 
        double psi_rad = DEG2RAD(psi_norm_deg); 

        sum_sin += sin(psi_rad); 
        sum_cos += cos(psi_rad); 
        
        printf("Beacon %d (C%.2f): Alpha=%.2f, Theta=%.2f -> Psi_norm=%.2f deg\n", 
               i, beacons[i].x, alpha_AM_deg, theta_lidar_deg, psi_norm_deg); 
    } 

    double orientation_rad = atan2(sum_sin, sum_cos); 
    double orientation_deg = RAD2DEG(orientation_rad); 
    
    printf("Sum Sin: %.4f, Sum Cos: %.4f\n", sum_sin, sum_cos); 
    printf("--- ORIENTATION LSQ CALCULATION: END ---\n"); 

    return orientation_deg; 
} 

// МЕТОДЫ РАСЧЕТА УГЛОВ (без изменений)

double TrilaterationSolver::calculate_angle_BAC(SPoint A, SPoint B, SPoint C) 
{
    double az_AB = get_azimuth_deg(A, B); 
    double az_AC = get_azimuth_deg(A, C); 
    return calculate_angle_from_azimuths(az_AB, az_AC); 
}

double TrilaterationSolver::calculate_angle_from_azimuths(double az_AB, double az_AC)
{
    double angle = az_AC - az_AB; 
    return normalize_angle_deg(angle); 
}

/*
// --- Пример использования (С ИДЕАЛЬНЫМИ ДАННЫМИ И НОВОЙ ФИЛЬТРАЦИЕЙ) ---

int main()
{
    // 1. Исходные данные
    
    SPoint B = {4.0, 0.3}; // Маяк B
    SPoint C = {0.0, 0.5}; // Маяк C
    SPoint D = {0.5, 4.0}; // Маяк D
    SPoint E = {5.0, 4.0}; // Маяк E
    
    std::vector<SPoint> beacons = {B, C, D, E}; // Список маяков
    
    // ИСТИННЫЕ ЗНАЧЕНИЯ 
    double PSI_TRUE = 0.0; // Истинный угол ориентации
    SPoint A_true = {1.0, 2.0}; // Истинное положение A
    
    SPoint A_prev = {1.000, 2.000}; // Предыдущее положение (Для точной отладки задаем A_true)
    
    // --- НАСТРОЙКА ВЕСА УГЛОВ ---
    double angle_weight_factor = 1.0; // КОНСТАНТА НАСТРОЙКИ: 1.0 (равный вес с дистанцией для отладки геометрии)

    // Истинные расстояния (R_true) и заданный шум 
    // Для идеальных углов берем НУЛЕВОЙ шум, чтобы убедиться в чистоте геометрии
    double R_AB_true = 3.448; double noise_R_AB = 0.0; 
    double R_AC_true = 1.802; double noise_R_AC = 0.0; 
    double R_AD_true = 2.061; double noise_R_AD = 0.0; 
    double R_AE_true = 4.472; double noise_R_AE = 0.0; 

    // Углы из трилатерации: ИДЕАЛЬНЫЕ УГЛЫ
    double angle_BAC = 94.15; // Угол <BAC
    double angle_CAD = 132.27; // Угол <CAD
    double angle_DAE = 77.47; // Угол <DAE
    double angle_EAB = 56.10; // Угол <EAB

    // СОГЛАСОВАННЫЕ УГЛЫ ЛИДАРА (Просто пример)
    std::vector<double> lidar_angles_deg = 
    {
        -29.54,    
        -123.69,   
        104.04,    
        26.57      
    };

    // 2. Инициализация решателя
    TrilaterationSolver solver(A_prev, angle_weight_factor); // Передаем настроечный вес 
    
    // 3. Сбор окружностей 
    
    solver.clear_circles(); 
    
    // 4 Дистанционные окружности (Вес 1.0)
    solver.add_circle_from_distance(B, R_AB_true + noise_R_AB); 
    solver.add_circle_from_distance(C, R_AC_true + noise_R_AC); 
    solver.add_circle_from_distance(D, R_AD_true + noise_R_AD); 
    solver.add_circle_from_distance(E, R_AE_true + noise_R_AE); 
    
    // 4 Угловые окружности (Вес 1.0, с отладочным выводом и ИСПРАВЛЕННОЙ фильтрацией)
    try
    {
        solver.add_filtered_circle_from_angle(B, C, angle_BAC); 
        solver.add_filtered_circle_from_angle(C, D, angle_CAD); 
        solver.add_filtered_circle_from_angle(D, E, angle_DAE); 
        solver.add_filtered_circle_from_angle(E, B, angle_EAB); 
    }
    catch(const std::invalid_argument& e)
    {
        fprintf(stderr, "Error collecting angle circles: %s\n", e.what()); // Output error
        return 1; 
    } 

    // 4. Расчет МНК для положения A
    printf("==================================\n"); 
    printf("WLS FOR POSITION A (N=%d, Angle Weight=%.2f)\n", solver.get_circle_count(), angle_weight_factor); 
    printf("==================================\n"); 

    SPoint A_found = solver.find_A_by_mnk(); // Запуск расчета положения

    printf("\n--- POSITION CALCULATION SUMMARY ---\n"); 
    printf("True A: (%.3f, %.3f)\n", A_true.x, A_true.y); 
    printf("Found A (WLS): (%.3f, %.3f)\n", A_found.x, A_found.y); 
    
    // 5. Расчет ориентации лидара
    double orientation_psi = solver.get_lidar_orientation( 
        A_found, 
        beacons, 
        lidar_angles_deg
    ); 

    printf("\n--- ORIENTATION CALCULATION SUMMARY ---\n"); 
    printf("True Psi: %.4f deg\n", PSI_TRUE); 
    printf("Calculated Psi: %.4f deg\n", orientation_psi); 
    
    return 0; 
}
*/
#endif