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

double sqr(double val); // Объявление функции для вычисления квадрата числа
double ctan(double rad); // Объявление функции для вычисления котангенса

// --- Основной класс решателя (ТОЛЬКО ОБЪЯВЛЕНИЕ) ---
class TrilaterationSolver
{
private:
    SPoint A_prev; // Предыдущая (или начальная) точка для выбора дуги
    std::vector<SCircle> all_circles; // Список всех окружностей (из расстояний и углов)

    // Приватные вспомогательные методы
    double get_azimuth_deg(SPoint P_from, SPoint P_to); // Расчет азимута от одной точки к другой (для ориентации)
    double normalize_angle_deg(double angle_deg); // Нормализация угла в диапазон (-180, 180]

public:
    TrilaterationSolver(SPoint prev); // Конструктор, инициализирующий A_prev

    // Методы управления состоянием
    void set_A_prev(SPoint new_prev); // Установка нового предыдущего положения (для фильтрации окружностей)
    void clear_circles(); // Очистка массива окружностей перед новым расчетом (для многократного использования)

    // Методы добавления данных
    int get_circle_count() const; // Получить текущее количество окружностей в массиве
    void add_circle_from_distance(SPoint P_beacon, double distance); // Добавление окружности, полученной из измеренной дальности
    void add_filtered_circle_from_angle(SPoint P1, SPoint P2, double angle_deg); // Добавление окружности, полученной из измеренного угла (с фильтрацией по A_prev)

    // Методы решения
    SPoint find_A_by_mnk(); // Решение методом наименьших квадратов (МНК) для определения положения A
    double get_lidar_orientation(const SPoint A_found, const std::vector<SPoint>& beacons, const std::vector<double>& lidar_angles_deg); // Определение ориентации лидара (Psi) через векторное усреднение
    
    // МЕТОДЫ РАСЧЕТА УГЛОВ
    double calculate_angle_BAC(SPoint A, SPoint B, SPoint C); // Расчет горизонтального угла BAC по координатам
    double calculate_angle_from_azimuths(double az_AB, double az_AC); // Расчет угла BAC по двум азимутам
};

// --- РЕАЛИЗАЦИЯ ВСПОМОГАТЕЛЬНЫХ ФУНКЦИЙ (ВНЕ КЛАССА) ---

double sqr(double val) 
{
    return val * val; // Функция для вычисления квадрата числа
} 
double ctan(double rad) 
{
    return 1.0 / tan(rad); // Функция для вычисления котангенса
} 

// --- РЕАЛИЗАЦИЯ МЕТОДОВ КЛАССА TrilaterationSolver ---

// Конструктор
TrilaterationSolver::TrilaterationSolver(SPoint prev) : A_prev(prev) 
{
} // Инициализация A_prev

// Приватные методы
double TrilaterationSolver::get_azimuth_deg(SPoint P_from, SPoint P_to) // Расчет азимута
{
    // Используется стандартная математическая конвенция atan2
    double rad = atan2(P_to.y - P_from.y, P_to.x - P_from.x); // Азимут в радианах
    return RAD2DEG(rad); // Азимут в градусах
} 

double TrilaterationSolver::normalize_angle_deg(double angle_deg) // Нормализация угла
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
void TrilaterationSolver::set_A_prev(SPoint new_prev) // Установка A_prev
{
    A_prev = new_prev; // Устанавливаем новое предыдущее положение
    printf("A_prev updated to: (%.3f, %.3f)\n", A_prev.x, A_prev.y); // Output updated A_prev
} 

void TrilaterationSolver::clear_circles() // Очистка данных
{
    all_circles.clear(); // Очистка вектора all_circles
    printf("\nAll circles cleared for new calculation.\n"); // Output confirmation
}

// Публичные методы добавления данных
int TrilaterationSolver::get_circle_count() const 
{ 
    return all_circles.size(); // Получить количество окружностей
} 

void TrilaterationSolver::add_circle_from_distance(SPoint P_beacon, double distance) // Добавление по дальности
{
    all_circles.push_back({P_beacon.x, P_beacon.y, distance}); // Добавляем маяк как центр с измеренным радиусом
    // printf("distance = %6.3f \n", distance);
} 

void TrilaterationSolver::add_filtered_circle_from_angle // Добавление по углу (с фильтрацией)
(
    SPoint P1,         // Точка 1 (начало хорды)
    SPoint P2,         // Точка 2 (конец хорды)
    double angle_deg          // Измеренный угол (АБСОЛЮТНОЕ значение)
)
{
    // Угол должен быть положительным модулем для корректного расчета R и h
    double alpha_rad = DEG2RAD(fabs(angle_deg)); // Модуль угла в радианах
    double sin_alpha = sin(alpha_rad); // Синус угла
    
    if (fabs(sin_alpha) < 1e-7) // Проверка на вырожденный случай (угол ~ 0 или ~ 180)
    {
        throw std::invalid_argument("Error: Angle is too close to 0 or 180."); // Error: Angle is too close to 0 or 180
    } 

    double cot_alpha = ctan(alpha_rad); // Котангенс угла
    double dx = P2.x - P1.x; // Разница по X
    double dy = P2.y - P1.y; // Разница по Y
    double d_sq = sqr(dx) + sqr(dy); // Квадрат длины хорды P1-P2
    
    if (d_sq < 1e-12) // Проверка на совпадение точек
    {
        throw std::invalid_argument("Error: Chord points P1 and P2 coincide."); // Error: Chord points P1 and P2 coincide
    } 

    double d = sqrt(d_sq); // Длина хорды
    double AM = d / 2.0; // Половина длины хорды
    
    double h = AM * cot_alpha; // Расстояние от середины хорды до центра окружности
    
    double R = d / (2.0 * fabs(sin_alpha)); // Радиус окружности
    
    SPoint M = {(P1.x + P2.x) / 2.0, (P1.y + P2.y) / 2.0}; // Середина хорды
    double vx = -dy / d; // Вектор, перпендикулярный хорде (x-компонента)
    double vy = dx / d; // Вектор, перпендикулярный хорде (y-компонента)

    // Два потенциальных центра окружности, лежащих по разные стороны от хорды
    SPoint O1 = {M.x + h * vx, M.y + h * vy}; // Первый центр
    SPoint O2 = {M.x - h * vx, M.y - h * vy}; // Второй центр (симметричный)
    
    // --- ЛОГИКА ФИЛЬТРАЦИИ ПО A_prev ---
    double dist1 = sqrt(sqr(O1.x - A_prev.x) + sqr(O1.y - A_prev.y)); // Расстояние до центра O1 от A_prev
    double dist2 = sqrt(sqr(O2.x - A_prev.x) + sqr(O2.y - A_prev.y)); // Расстояние до центра O2 от A_prev

    struct SCircle chosen_circle; // Выбранная окружность
    if (dist1 < dist2) // Сравниваем расстояния
    {
        chosen_circle = {O1.x, O1.y, R}; // Выбираем O1, если он ближе к A_prev
    } 
    else 
    {
        chosen_circle = {O2.x, O2.y, R}; // Выбираем O2, если он ближе
    } 
    
    all_circles.push_back(chosen_circle); // Добавляем выбранную окружность в список
} 

// Публичный метод решения положения
SPoint TrilaterationSolver::find_A_by_mnk()
{
    SPoint A = {0.0, 0.0}; // Искомая точка A, инициализация нулями
    int N = all_circles.size(); // Общее количество окружностей
    
    printf("\n--- POS LSQ SOLVER: START (N=%d) ---\n", N); // Output start of LSQ solver
    
    if (N < 3) // Проверка минимального количества уравнений (для МНК нужно N>=3)
    {
        printf("Error: At least 3 circles are required.\n"); // Error: At least 3 circles are required
        return A; // Возвращаем нулевую точку в случае ошибки
    } 

    const struct SCircle& base = all_circles[0]; // Выбираем первую окружность как базовую (i=0)
    double base_term = sqr(base.x) + sqr(base.y) - sqr(base.r); // Часть C_0 для линеаризации

    double M_xx = 0.0, M_xy = 0.0, M_yy = 0.0; // Элементы симметричной матрицы (M^T*M)
    double V_x = 0.0, V_y = 0.0; // Элементы правой части (M^T*C)

    // 1. Накопление матриц (Линеаризация и построение нормальных уравнений)
    for (int i = 1; i < N; ++i) // Перебираем все окружности, кроме базовой (i=0)
    {
        const struct SCircle& current = all_circles[i]; // Текущая окружность
        
        // Коэффициенты Ai и Bi (2 * (Xi - X0) и 2 * (Yi - Y0))
        double Ai = 2.0 * (current.x - base.x); // Коэффициент при x
        double Bi = 2.0 * (current.y - base.y); // Коэффициент при y

        // Коэффициент Ci (X_i^2 + Y_i^2 - R_i^2) - C_0
        double current_term = sqr(current.x) + sqr(current.y) - sqr(current.r); // Часть C_i
        double Ci = current_term - base_term; // Правая часть уравнения

        // Накопление элементов M^T*M
        M_xx += Ai * Ai; // Накопление элемента M_xx
        M_xy += Ai * Bi; // Накопление элемента M_xy
        M_yy += Bi * Bi; // Накопление элемента M_yy
        // Накопление элементов M^T*C
        V_x += Ai * Ci; // Накопление элемента V_x
        V_y += Bi * Ci; // Накопление элемента V_y
    } 

    // 2. Решение СЛАУ (2x2) по правилу Крамера
    double det = M_xx * M_yy - M_xy * M_xy; // Вычисление определителя
    if (fabs(det) < 1e-9) // Проверка на вырожденность (определитель близок к нулю)
    {
        printf("Error: Determinant (%.2e) is near zero. Solution is unstable.\n", det); // Error: Determinant is near zero
        return A; // Возвращаем нулевую точку
    } 

    double InvDet = 1.0 / det; // Обратный определитель
    A.x = InvDet * (M_yy * V_x - M_xy * V_y); // Решение для x: (M_yy * V_x - M_xy * V_y) / det
    A.y = InvDet * (M_xx * V_y - M_xy * V_x); // Решение для y: (M_xx * V_y - M_xy * V_x) / det
    
    printf("Found A: (%.3f, %.3f)\n", A.x, A.y); // Output found position

    // 3. Вычисление Геометрического RMS (в метрах)
    double sum_sq_residual_geom = 0.0; // Сумма квадратов геометрических невязок
    for (int i = 0; i < N; ++i) // Перебираем все N окружностей
    {
        const struct SCircle& current = all_circles[i]; // Текущая окружность
        
        // Расстояние от найденной точки A до центра окружности (r_calc)
        double r_calc = sqrt(sqr(A.x - current.x) + sqr(A.y - current.y)); // Вычисленное расстояние
        
        // Геометрическая невязка (в метрах): R_измеренное - R_вычисленное
        double residual_geom = current.r - r_calc; // Невязка
        sum_sq_residual_geom += sqr(residual_geom); // Накопление квадрата невязки

        printf("Beacon %d (R=%.3f): Geom. Residual=%.3f m\n", i, current.r, residual_geom); // Output residual for each beacon
    }

    int degrees_of_freedom = N - 3; // Число степеней свободы = N - 3 (x, y, неявный масштаб)

    if (degrees_of_freedom > 0) // Если есть степени свободы
    {
        double rmse_geom = sqrt(sum_sq_residual_geom / degrees_of_freedom); // Геометрическая RMS
        
        printf("\n--- Quality Assessment (N=%d, DoF=%d) ---\n", N, degrees_of_freedom); // Output quality assessment
        printf("**Geometric RMS**: **%.3f m**\n", rmse_geom); // Output geometric RMS
    }
    else
    {
        printf("\nSolution is exactly determined (RMSE = N/A).\n"); // Решение точно определено
    }

    printf("--- POS LSQ SOLVER: END ---\n"); // Output end of LSQ solver

    return A; // Возвращаем найденную точку
} 

// Публичный метод определения ориентации
double TrilaterationSolver::get_lidar_orientation(
    const SPoint A_found,                          // Найденное положение
    const std::vector<SPoint>& beacons,           // Координаты маяков
    const std::vector<double>& lidar_angles_deg          // Углы, измеренные лидаром
)
{
    if (beacons.size() != lidar_angles_deg.size() || beacons.empty()) // Проверка входных данных
    {
        printf("\n--- ORIENTATION CALCULATION SKIPPED ---\n"); // Пропускаем расчет ориентации
        printf("Error: Missing or mismatched data for orientation calculation.\n"); // Error: Missing or mismatched data
        return 0.0; // Возвращаем ноль
    } 

    printf("\n--- ORIENTATION LSQ CALCULATION ---\n"); // Output start of orientation calculation
    
    double sum_sin = 0.0; // Сумма синусов для векторного усреднения
    double sum_cos = 0.0; // Сумма косинусов для векторного усреднения
    int N = beacons.size(); // Количество маяков
    
    for (int i = 0; i < N; ++i) // Перебираем все маяки
    {
        double alpha_AM_deg = get_azimuth_deg(A_found, beacons[i]); // Азимут от A до маяка (Alpha)
        double theta_lidar_deg = lidar_angles_deg[i]; // Угол лидара (Theta)

        double psi_raw_deg = alpha_AM_deg - theta_lidar_deg; // Ненормализованный угол поворота (Psi = Alpha - Theta)
        double psi_norm_deg = normalize_angle_deg(psi_raw_deg); // Нормализованный угол поворота
        double psi_rad = DEG2RAD(psi_norm_deg); // Угол в радианах

        sum_sin += sin(psi_rad); // Накопление синусов
        sum_cos += cos(psi_rad); // Накопление косинусов
        
        printf("Beacon %d (C%.2f): Alpha=%.2f, Theta=%.2f -> Psi_norm=%.2f deg\n", // Output intermediate values
               i, beacons[i].x, alpha_AM_deg, theta_lidar_deg, psi_norm_deg); 
    } 

    double orientation_rad = atan2(sum_sin, sum_cos); // Расчет усредненного угла (векторное усреднение)
    double orientation_deg = RAD2DEG(orientation_rad); // Угол в градусах
    
    printf("Sum Sin: %.4f, Sum Cos: %.4f\n", sum_sin, sum_cos); // Output sums
    printf("--- ORIENTATION LSQ CALCULATION: END ---\n"); // Output end of calculation

    return orientation_deg; // Возвращаем найденную ориентацию
} 

// ФУНКЦИЯ ДЛЯ РАСЧЕТА УГЛА BAC ПО КООРДИНАТАМ
double TrilaterationSolver::calculate_angle_BAC(SPoint A, SPoint B, SPoint C) // A - вершина угла, B, C - боковые точки
{
    double az_AB = get_azimuth_deg(A, B); // Азимут направления AB
    double az_AC = get_azimuth_deg(A, C); // Азимут направления AC
    
    // Угол BAC - это угол между векторами AB и AC, нормализуем его в диапазон (-180, 180]
    return calculate_angle_from_azimuths(az_AB, az_AC); // Вызываем новую функцию
}

// НОВАЯ ФУНКЦИЯ: РАСЧЕТ УГЛА ПО ДВУМ АЗИМУТАМ
double TrilaterationSolver::calculate_angle_from_azimuths(double az_AB, double az_AC)
{
    double angle = az_AC - az_AB; // Разность азимутов
    double angle_norm = normalize_angle_deg(angle); // Нормализация угла в диапазон (-180, 180]

    printf(" %6.2f %6.2f %6.2f | =  %6.2f \n",az_AB,az_AC, angle, angle_norm);

    return angle_norm; // Возвращаем нормализованный угол
}
/*
// --- Пример использования (С согласованными данными: PSI_TRUE = 0.0) ---

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
    
    SPoint A_prev = {1.05, 2.05}; // Предыдущее положение (ЗАДАНО)

    // Истинные расстояния (R_true) и заданный шум 
    double R_AB_true = 3.448; double noise_R_AB = 0.01; // Измерение R_AB с шумом
    double R_AC_true = 1.802; double noise_R_AC = -0.02; // Измерение R_AC с шумом
    double R_AD_true = 2.061; double noise_R_AD = 0.01; // Измерение R_AD с шумом
    double R_AE_true = 4.472; double noise_R_AE = -0.01; // Измерение R_AE с шумом

    // Углы из трилатерации: МОДУЛИ УГЛОВ (без знака, т.к. фильтрация по A_prev восстанавливает знак)
    double angle_BAC = 94.15; // Модуль угла BAC
    double angle_CAD = 132.27; // Модуль угла CAD
    double angle_DAE = 77.47; // Модуль угла DAE
    double angle_EAB = 56.10; // Модуль угла EAB

    // СОГЛАСОВАННЫЕ УГЛЫ ЛИДАРА (Theta_i = Alpha_true - 0.0 + noise)
    std::vector<double> lidar_angles_deg = 
    {
        -29.54 - 0.3,    // Theta_AB = -29.84 (Азимут АВ с шумом)
        -123.69 + 0.2,   // Theta_AC = -123.49 (Азимут АС с шумом)
        104.04 + 0.1,    // Theta_AD = 104.14 (Азимут АD с шумом)
        26.57 - 0.1      // Theta_AE = 26.47 (Азимут АЕ с шумом)
    };

    // 2. Инициализация решателя
    TrilaterationSolver solver(A_prev); // Создание объекта решателя с A_prev
    
    // --- ПРИМЕР ИСПОЛЬЗОВАНИЯ НОВОЙ ФУНКЦИИ ---
    double az_AB_true = solver.get_azimuth_deg(A_true, B); // Азимут A->B (расчет для примера)
    double az_AC_true = solver.get_azimuth_deg(A_true, C); // Азимут A->C (расчет для примера)
    
    printf("\n--- AZIMUTHS AND ANGLE CHECK ---\n"); // Output azimuths and angle check
    printf("True Azimuth AB: %.2f deg\n", az_AB_true); // Output true azimuth AB
    printf("True Azimuth AC: %.2f deg\n", az_AC_true); // Output true azimuth AC
    
    // Расчет угла BAC по азимутам
    double check_angle_az = solver.calculate_angle_from_azimuths(az_AB_true, az_AC_true); // Расчет угла BAC по азимутам
    printf("Angle BAC from Azimuths: %.2f deg (True Module: %.2f deg)\n", check_angle_az, angle_BAC); // Output calculated and true angle
    printf("--- AZIMUTHS AND ANGLE CHECK: END ---\n"); // Output end of azimuths and angle check

    // 3. Сбор окружностей (4 по расстоянию, 4 по углу)
    
    solver.clear_circles(); // Очистка данных перед новым циклом расчета

    solver.add_circle_from_distance(B, R_AB_true + noise_R_AB); // Добавление окружности по дальности AB
    solver.add_circle_from_distance(C, R_AC_true + noise_R_AC); // Добавление окружности по дальности AC
    solver.add_circle_from_distance(D, R_AD_true + noise_R_AD); // Добавление окружности по дальности AD
    solver.add_circle_from_distance(E, R_AE_true + noise_R_AE); // Добавление окружности по дальности AE
    
    try
    {
        // Передаем МОДУЛИ углов. Функция сама выберет правильный центр, ближайший к A_prev.
        solver.add_filtered_circle_from_angle(B, C, angle_BAC); // Добавление окружности по углу BAC
        solver.add_filtered_circle_from_angle(C, D, angle_CAD); // Добавление окружности по углу CAD
        solver.add_filtered_circle_from_angle(D, E, angle_DAE); // Добавление окружности по углу DAE
        solver.add_filtered_circle_from_angle(E, B, angle_EAB); // Добавление окружности по углу EAB
    }
    catch(const std::invalid_argument& e)
    {
        fprintf(stderr, "Error collecting circles: %s\n", e.what()); // Вывод ошибки
        return 1; // Код ошибки
    } 

    // 4. Расчет МНК для положения A
    printf("==================================\n"); // Разделитель
    printf("LSQ FOR POSITION A (N=%d)\n", solver.get_circle_count()); // Заголовок
    printf("==================================\n"); // Разделитель

    SPoint A_found = solver.find_A_by_mnk(); // Запуск расчета положения

    printf("\n--- POSITION CALCULATION SUMMARY ---\n"); // Сводка
    printf("True A: (%.3f, %.3f)\n", A_true.x, A_true.y); // Истинное положение
    printf("Found A: (%.3f, %.3f)\n", A_found.x, A_found.y); // Найденное положение
    
    // 5. Расчет ориентации лидара
    double orientation_psi = solver.get_lidar_orientation( // Запуск расчета ориентации
        A_found, 
        beacons, 
        lidar_angles_deg
    ); 

    printf("\n--- ORIENTATION CALCULATION SUMMARY ---\n"); // Сводка ориентации
    printf("True Psi: %.4f deg\n", PSI_TRUE); // Истинный угол
    printf("Calculated Psi: %.4f deg\n", orientation_psi); // Найденный угол
    
    return 0; // Успешное завершение
}
*/
#endif