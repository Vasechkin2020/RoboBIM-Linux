#ifndef TRIALATERATIONSOLVER_H
#define TRIALATERATIONSOLVER_H

#include "config.h"  // Подключение ваших файлов конфигурации (для RAD2DEG, DEG2RAD)
#include "code.h"    // Подключение ваших вспомогательных файлов (для SPoint и SPoint_Q)
#include <stdio.h>   // Подключение стандартной библиотеки ввода-вывода (для printf)
#include <math.h>    // Подключение математической библиотеки (для M_PI, M_SQRT2)
#include <vector>    // Подключение библиотеки для работы с векторами
#include <cmath>     // Подключение современной математической библиотеки (для fabs, atan2)
#include <stdexcept> // Подключение для обработки стандартных исключений
#include <algorithm> // Подключение для std::max, std::min

// --- ГЛОБАЛЬНЫЕ КОНСТАНТЫ ДЛЯ СТАТИСТИКИ И WLS ---

// 1. Статистические ошибки (из даташита)
const double SIGMA_THETA_DEG = 0.1125; // Инструментальная ошибка одного направления (градусы)
const double SIGMA_D_METERS = 0.030;   // Инструментальная ошибка расстояния (30 мм = 0.030 м)

// 3. Константы для эмпирического расчета углового веса (по требованиям пользователя)
const double D_REF_NORM = 1.0;            // Базовая дальность для W=1.0 (1.0 м)
const double D_MAX_RANGE = 20.0;          // Максимальная дальность для W_min (20.0 м)
const double W_DIST_MIN_AT_DMAX = 0.2;    // Минимальный дистанционный фактор на D_MAX_RANGE (1.0 - 0.8)
const double DELTA_ALPHA_MAX_DEG = 65.0;  // Максимальное отклонение угла (90 - 25 = 65)
const double W_ANGLE_MIN_AT_65DEG = 0.25; // Минимальный угловой фактор (1.0 - 0.75)
const double MIN_WEIGHT_CLAMP = 0.01;     // Минимальный вес для устойчивости WLS

// 4. Нормированные веса (НАШ ПОДХОД)
const double WEIGHT_DISTANCE_NORM = 1.0; // Эталонный вес для расстояния (всегда 1.0)

// 5. Параметры робастности
const double OUTLIER_REJECTION_THRESHOLD_METERS = 0.10; // Порог отбраковки выброса (10 см)
const double BAD_ANGLE_LOW_DEG = 30.0;                  // Нижняя граница "плохого" угла (для веса)
const double BAD_ANGLE_HIGH_DEG = 150.0;                // Верхняя граница "плохого" угла (для веса)

// --- Вспомогательные структуры и функции (внутренние для решателя) ---

struct SCircle_W // Структура для хранения окружности (Маяк/Угол)
{
    double x;             // Координата x центра
    double y;             // Координата y центра
    double r;             // Радиус
    double weight_factor; // ФАКТОР ВЕСА (нормированный, от 0 до 1)
    bool is_active;       // ФЛАГ АКТИВНОСТИ: true, если используется в расчете
};

// SPoint_Q (для code.h)
struct SPoint_Q
{
    SPoint A;               // Найденная точка
    double quality;         // Геометрический RMS (unweighted)
    int used_measurements;  // Сколько измерений использовано в финальном расчете
    int total_measurements; // Всего доступно измерений
    bool has_outliers;      // Были ли выбросы отброшены (только для Robust)
};

double sqr(double val)
{
    return val * val; // Функция для вычисления квадрата числа
}
double ctan(double rad)
{
    return 1.0 / tan(rad); // Функция для вычисления котангенса
}

/**
 * @brief Преобразует угол из любого диапазона (например, 0-360) в диапазон (+-180] и инвертирует знак.
 * * Стандарт: (-180, 180] (Плюс = против часовой стрелки)
 * Требуется: (+180, -180] (Плюс = по часовой стрелке)
 * * @param angle_deg Угол в градусах.
 * @return Нормализованный и инвертированный угол в градусах.
 */
double normalize_and_invert_sign_deg(double angle_deg)
{
    // --- 1. Нормализация в стандартный диапазон (-180, 180] ---

    double a = fmod(angle_deg + 180.0, 360.0); // Сдвигаем диапазон на 180

    if (a < 0.0) // Если результат fmod отрицателен (для некоторых компиляторов)
    {
        a += 360.0; // Приводим к положительному диапазону [0, 360)
    }

    // Получаем стандартный угол: (-180, 180] (Плюс = CCW)
    double standard_normalized_angle = a - 180.0;

    // --- 2. Инверсия знака ---
    // Инвертируем знак, чтобы ПЛЮС соответствовал вращению по часовой стрелке (CW, вправо)
    return -standard_normalized_angle; // Возвращаем инвертированный угол
}

// --- Основной класс решателя ---

class TrilaterationSolver
{
private:
    SPoint A_prev;                      // Предыдущая (или начальная) точка для выбора дуги
    std::vector<SCircle_W> all_circles; // Список всех окружностей (из расстояний и углов)

    // Приватные вспомогательные методы
    double get_azimuth_deg(SPoint P_from, SPoint P_to); // Расчет азимута от одной точки к другой

    // Ядро WLS: выполняет ОДИН проход WLS по АКТИВНЫМ окружностям
    bool perform_wls_pass(SPoint &result_A, double &result_rms, int &used_count, bool print_residuals);

public:
    double normalize_angle_deg_abs(double angle_deg); // Нормализация угла в диапазон (-180, 180] Возвращает всегда плюс по модулю
    double normalize_angle_deg(double angle_deg);     // Нормализация угла в диапазон (-180, 180]

    double angle_diff_deg(double phi, double theta);                               // Угловая разность: phi - theta, нормализованная в [-180, 180)
    double complementary_filter_angle_deg(double theta, double phi, double alpha); // Комплементарный фильтр для углов (в градусах)
    TrilaterationSolver(SPoint prev);                                              // Конструктор, инициализирующий A_prev

    // Методы управления состоянием
    void set_A_prev(SPoint new_prev); // Установка нового предыдущего положения
    void clear_circles();             // Очистка массива окружностей перед новым расчетом

    // Методы добавления данных
    int get_circle_count() const;                                                // Получить текущее количество окружностей
    void add_circle_from_distance(SPoint P_beacon, double distance);             // Добавление окружности из дальности
    void add_filtered_circle_from_angle(SPoint P1, SPoint P2, double angle_deg); // Добавление окружности из угла (с ДИНАМИЧЕСКИМ ВЕСОМ)

    // МЕТОДЫ РЕШЕНИЯ
    SPoint_Q find_A_by_mnk_simple(); // 1. Простой WLS (Один проход, без отбраковки)
    SPoint_Q find_A_by_mnk_robust(); // 2. Робастный WLS (Два прохода, с отбраковкой выбросов)

    // Дополнительные методы
    double get_lidar_orientation(const SPoint A_found, const std::vector<SPoint> &beacons, const std::vector<double> &lidar_angles_deg); // Определение ориентации лидара
    double calculate_angle_BAC(SPoint A, SPoint B, SPoint C);                                                                            // Расчет угла по координатам
    double calculate_angle_from_azimuths(double az_AB, double az_AC);                                                                    // Расчет угла по азимутам
};

// --- РЕАЛИЗАЦИЯ МЕТОДОВ КЛАССА TrilaterationSolver ---

// --- Конструктор и вспомогательные методы ---

TrilaterationSolver::TrilaterationSolver(SPoint prev) : A_prev(prev)
{
} // Инициализация A_prev

double TrilaterationSolver::get_azimuth_deg(SPoint P_from, SPoint P_to)
{
    double rad = atan2(P_to.y - P_from.y, P_to.x - P_from.x); // Азимут в радианах
    return RAD2DEG(rad);                                      // Азимут в градусах (требуется макрос RAD2DEG)
}

double TrilaterationSolver::normalize_angle_deg_abs(double angle_deg)
{
    while (angle_deg > 180.0) // Пока угол больше 180 градусов
    {
        angle_deg -= 360.0; // Вычитаем 360 градусов
    }
    while (angle_deg <= -180.0) // Пока угол меньше или равен -180 градусов
    {
        angle_deg += 360.0; // Прибавляем 360 градусов
    }
    return std::abs(angle_deg); // Возвращаем абсолютное значение
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
    return angle_deg; // Возвращаем
}

void TrilaterationSolver::set_A_prev(SPoint new_prev)
{
    A_prev = new_prev;                                               // Устанавливаем новое предыдущее положение
    logi.log_b("    A_prev updated to: (%+8.3f, %+8.3f)\n", A_prev.x, A_prev.y); // Output updated A_prev
}

void TrilaterationSolver::clear_circles()
{
    all_circles.clear();                                    // Очистка вектора all_circles
    logi.log("    All circles cleared for new calculation.\n"); // Output confirmation
}

int TrilaterationSolver::get_circle_count() const
{
    return all_circles.size(); // Получить количество окружностей
}

void TrilaterationSolver::add_circle_from_distance(SPoint P_beacon, double distance)
{
    // printf("+++ add_circle_from_distance \n");
    // Отладочный вывод: Добавление измерения дальности
    logi.log("    Add Dist: Beacon (%+8.3f, %+8.3f), R=%+8.3f, W=%.4f\n", // Output: Adding distance measurement
           P_beacon.x,                                            // X-coordinate of beacon
           P_beacon.y,                                            // Y-coordinate of beacon
           distance,                                              // Measured distance
           WEIGHT_DISTANCE_NORM                                   // Assigned weight
    );                                                            // Выводим координаты, расстояние и вес

    // Вес дистанционного измерения ВСЕГДА 1.0 (наш эталон)
    all_circles.push_back({P_beacon.x, P_beacon.y, distance, WEIGHT_DISTANCE_NORM, true}); // Добавляем маяк с весом 1.0 и флагом active=true
}

void TrilaterationSolver::add_filtered_circle_from_angle(SPoint P1, SPoint P2, double angle_deg)
{
    logi.log("    ANGLE CIRCLE (P1=(%+7.3f, %+7.3f), P2=(%+7.3f, %+7.3f), Angle= %+8.3f) \n", P1.x, P1.y, P2.x, P2.y, angle_deg);

    double alpha_rad = DEG2RAD(angle_deg); // Угол в радианах
    double sin_alpha = sin(alpha_rad);     // Синус угла

    if (fabs(sin_alpha) < 1e-7) // Проверка на вырожденный случай
    {
        logi.log_r("Error: Angle is too close to 0 or 180 (sin_alpha < 1e-7).\n"); // Output error
        throw std::invalid_argument("Error: Angle is too close to 0 or 180.");
    }

    // --- БЛОК РАСЧЕТА ДИНАМИЧЕСКОГО НОРМИРОВАННОГО ВЕСА (ЭМПИРИЧЕСКАЯ МОДЕЛЬ) ---
    double dynamic_weight_norm = MIN_WEIGHT_CLAMP; // Итоговый нормированный вес (по умолчанию почти 0)
    double W_angle = 0.001;                        // Вес угла
    double W_distance = 0.001;                     // Вес дальности

    if (angle_deg < BAD_ANGLE_LOW_DEG || angle_deg > BAD_ANGLE_HIGH_DEG) // 1. Проверка на "плохой" угол
    {
        // Для нестабильного угла оставляем минимальный вес 1e-9
        logi.log_w("    Warning: Angle (%.2f) is in unstable region. Assigning near-zero weight.\n", angle_deg); // Output warning
    }
    else
    {
        // --- 1. РАСЧЕТ УГЛОВОГО ВЕСА W_angle (Линейное ухудшение от 90 гр. до 0.25)
        double delta_alpha = fabs(angle_deg - 90.0);                             // Отклонение от 90 градусов
        double decay_range_angle = DELTA_ALPHA_MAX_DEG;                          // 65.0 градусов
        double decay_amount_angle = WEIGHT_DISTANCE_NORM - W_ANGLE_MIN_AT_65DEG; // 1.0 - 0.25 = 0.75

        // W_angle: Линейно от 1.0 (при 0) до 0.25 (при 65)
        W_angle = 1.0 - decay_amount_angle * (delta_alpha / decay_range_angle);
        W_angle = std::max(W_angle, W_ANGLE_MIN_AT_65DEG); // Ограничение снизу (0.25)

        // --- 2. РАСЧЕТ ВЕСА ДАЛЬНОСТИ W_distance (Линейное ухудшение от 1 м до 20 м)
        double D_i = sqrt(sqr(P1.x - A_prev.x) + sqr(P1.y - A_prev.y)); // Расстояние до P1
        double D_j = sqrt(sqr(P2.x - A_prev.x) + sqr(P2.y - A_prev.y)); // Расстояние до P2
        double D_max = std::max(D_i, D_j);                              // Максимальное расстояние

        W_distance = 1.0; // По умолчанию 1.0

        if (D_max >= D_REF_NORM) // Ухудшение начинается после 1 метра
        {
            double decay_range_dist = D_MAX_RANGE - D_REF_NORM;                   // 19.0
            double decay_amount_dist = WEIGHT_DISTANCE_NORM - W_DIST_MIN_AT_DMAX; // 1.0 - 0.2 = 0.8

            // W_distance: Линейно от 1.0 (при 1м) до 0.2 (при 20м)
            W_distance = 1.0 - decay_amount_dist * ((D_max - D_REF_NORM) / decay_range_dist);

            // Ограничение: W_distance не может быть меньше минимального (0.2)
            W_distance = std::max(W_distance, W_DIST_MIN_AT_DMAX);
        }

        // --- 3. КОМБИНАЦИЯ ---
        dynamic_weight_norm = W_angle * W_distance;                                // W = W_angle * W_distance
        dynamic_weight_norm = std::min(dynamic_weight_norm, WEIGHT_DISTANCE_NORM); // Ограничение сверху (1.0)
        dynamic_weight_norm = std::max(dynamic_weight_norm, MIN_WEIGHT_CLAMP);     // Ограничение снизу (1e-6)
    }

    logi.log("    dynamic_weight_norm = %.6f (W_angle = %.6f, W_distance = %.6f) \n", dynamic_weight_norm, W_angle, W_distance); // Output calculated weight
    // --- КОНЕЦ БЛОКА ---

    // 2. Расчет геометрии окружности
    double cot_alpha = ctan(alpha_rad); // Котангенс угла
    double dx = P2.x - P1.x;            // Разница по X
    double dy = P2.y - P1.y;            // Разница по Y
    double d_sq = sqr(dx) + sqr(dy);    // Квадрат длины хорды

    if (d_sq < 1e-12)
    {
        throw std::invalid_argument("Error: Chord points P1 and P2 coincide."); // Проверка
    }

    double d = sqrt(d_sq);                  // Длина хорды
    double h = (d / 2.0) * cot_alpha;       // Смещение от середины
    double R = d / (2.0 * fabs(sin_alpha)); // Радиус окружности

    SPoint M = {(P1.x + P2.x) / 2.0, (P1.y + P2.y) / 2.0}; // Середина хорды
    double vx = -dy / d;                                   // Вектор, перпендикулярный хорде (x)
    double vy = dx / d;                                    // Вектор, перпендикулярный хорде (y)

    SPoint O1 = {M.x + h * vx, M.y + h * vy}; // Центр 1
    SPoint O2 = {M.x - h * vx, M.y - h * vy}; // Центр 2

    // 3. Фильтрация дуги (выбор O1 или O2 по близости к A_prev)
    double dist1 = sqrt(sqr(O1.x - A_prev.x) + sqr(O1.y - A_prev.y)); // Расстояние от O1 до A_prev
    double dist2 = sqrt(sqr(O2.x - A_prev.x) + sqr(O2.y - A_prev.y)); // Расстояние от O2 до A_prev
    double deviation1 = fabs(dist1 - R);                              // Отклонение A_prev от R для O1
    double deviation2 = fabs(dist2 - R);                              // Отклонение A_prev от R для O2

    struct SCircle_W chosen_circle; // Выбранная окружность

    if (deviation1 < deviation2)
    {
        chosen_circle = {O1.x, O1.y, R, dynamic_weight_norm, true}; // Выбираем O1
    }
    else
    {
        chosen_circle = {O2.x, O2.y, R, dynamic_weight_norm, true}; // Выбираем O2
    }

    all_circles.push_back(chosen_circle);                                                                                                                  // Добавляем выбранную окружность
    logi.log("    Added Circle: Center (%+7.3f, %+7.3f), R=%+7.3f, W=%+7.4f\n", chosen_circle.x, chosen_circle.y, chosen_circle.r, chosen_circle.weight_factor); // Output final circle parameters
}

// ------------------------------------------------------------------
// --- ПРИВАТНЫЙ МЕТОД: ЯДРО WLS (perform_wls_pass) ---
// ------------------------------------------------------------------

/**
 * @brief Приватный метод, выполняющий ОДИН проход WLS по АКТИВНЫМ окружностям.
 */
bool TrilaterationSolver::perform_wls_pass(SPoint &result_A, double &result_rms, int &used_count, bool print_residuals)
{
    int N = all_circles.size(); // Общее количество окружностей
    used_count = 0;             // Сбрасываем счетчик использованных измерений

    // 1. Выбор базового уравнения (первого активного)
    int base_index = -1; // Индекс базового уравнения
    for (int i = 0; i < N; ++i)
    {
        if (this->all_circles[i].is_active) // Если окружность активна
        {
            base_index = i; // Находим первую активную окружность как базовую
            break;          // Выходим из цикла
        }
    }

    if (base_index == -1) // Ошибка, нет активных уравнений
    {
        if (print_residuals)
            printf("Error: No active base circle found for WLS.\n"); // Output error
        return false;
    }

    const struct SCircle_W &base = this->all_circles[base_index]; // Базовая окружность
    double base_weight = base.weight_factor;                      // Вес W_0 (нормированный)
    // Дисперсия sigma_0^2 = 1 / W_0
    double base_variance = (fabs(base_weight) > 1e-9) ? (1.0 / base_weight) : 1e9;
    double base_term = sqr(base.x) + sqr(base.y) - sqr(base.r); // Константа из уравнения

    double M_xx = 0.0, M_xy = 0.0, M_yy = 0.0; // Матрица M^T*W*M
    double V_x = 0.0, V_y = 0.0;               // Вектор M^T*W*C
    int active_equations = 0;                  // Счетчик активных разностных уравнений

    // 2. Накопление матриц (Линеаризация)
    for (int i = 0; i < N; ++i)
    {
        if (i == base_index)
            continue; // Пропускаем базовое уравнение
        if (!this->all_circles[i].is_active)
            continue; // Пропускаем отброшенные

        active_equations++; // Учитываем используемое уравнение

        const struct SCircle_W &current = this->all_circles[i]; // Текущая окружность

        // --- РАСЧЕТ ВЕСА РАЗНОСТНОГО УРАВНЕНИЯ (W_k) ---
        double current_weight = current.weight_factor;                                          // Вес W_i
        double current_variance = (fabs(current_weight) > 1e-9) ? (1.0 / current_weight) : 1e9; // Дисперсия sigma_i^2
        double combined_variance = current_variance + base_variance;                            // sigma_k^2 = sigma_i^2 + sigma_0^2
        double Wi = 1.0 / combined_variance;                                                    // W_k = 1 / sigma_k^2
        // ---------------------------------------------------
        // printf("  Eq %d vs Base %d : Circle current_weight W= %+8.3f, Eq Wi= %+8.3f \n", i, base_index, current_weight, Wi); // Output WLS weights (добавленный вывод)
        // Коэффициенты линеаризации (Ai*x + Bi*y = Ci)
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

    if (active_equations < 2) // Должно быть хотя бы 2 уравнения для X и Y
    {
        if (print_residuals)
            printf("Error: Not enough active equations (%d) for WLS (min 2 required).\n", active_equations); // Output error
        return false;
    }

    // 3. Решение СЛАУ (2x2)
    double det = M_xx * M_yy - M_xy * M_xy; // Детерминант
    if (fabs(det) < 1e-9)
    {
        if (print_residuals)
            printf("Error: Determinant (%.2e) is near zero. Solution is unstable.\n", det); // Output error
        return false;
    }

    double InvDet = 1.0 / det;                       // Обратный детерминант
    result_A.x = InvDet * (M_yy * V_x - M_xy * V_y); // Решение X
    result_A.y = InvDet * (M_xx * V_y - M_xy * V_x); // Решение Y

    // 4. Вычисление Геометрического RMS
    double sum_sq_residual_geom = 0.0; // Сумма квадратов остатков
    int dof_count = 0;                 // Счетчик активных измерений для DoF

    for (int i = 0; i < N; ++i)
    {
        const struct SCircle_W &current = this->all_circles[i];
        if (!current.is_active)
            continue; // Только активные

        double r_calc = sqrt(sqr(result_A.x - current.x) + sqr(result_A.y - current.y)); // Рассчитанный радиус r_calc
        double residual_geom = current.r - r_calc;                                       // Геометрический остаток
        sum_sq_residual_geom += sqr(residual_geom);                                      // Сумма квадратов
        dof_count++;                                                                     // Увеличиваем счетчик

        if (print_residuals) // Печатаем остатки, если это финальный проход
        {
            // printf("    Beacon %d (W=%.2f, R=%+8.3f): Geom. Residual=%+8.3f m (ACTIVE)\n", i, current.weight_factor, current.r, residual_geom); // Output final residual
        }
    }

    used_count = dof_count; // Записываем количество использованных измерений

    // Степени свободы = (Кол-во активных измерений) - (Кол-во неизвестных, т.е. 2)
    int degrees_of_freedom = dof_count - 2;
    result_rms = (degrees_of_freedom > 0) ? sqrt(sum_sq_residual_geom / degrees_of_freedom) : 0.0; // RMS

    return true; // WLS прошел успешно
}

// ------------------------------------------------------------------
// --- ПУБЛИЧНЫЕ МЕТОДЫ РЕШЕНИЯ ---
// ------------------------------------------------------------------

/**
 * @brief Метод 1: Простой WLS (Один проход, без отбраковки).
 */
SPoint_Q TrilaterationSolver::find_A_by_mnk_simple()
{
    SPoint A = {0.0, 0.0};
    SPoint_Q AQ; // Инициализация структуры качества
    AQ.A = A;
    AQ.quality = 999.0;
    AQ.has_outliers = false; // Нет отбраковки
    AQ.total_measurements = all_circles.size();
    AQ.used_measurements = 0;

    int N = all_circles.size();
    logi.log("    SOLVER: SIMPLE (Pass 1 of 1) (Total N=%d) Aprev x= %6.3f y= %6.3f ---\n", N, A_prev.x, A_prev.y); // Output solver start

    if (N < 3)
    {
        logi.log_r("    --- ERROR: At least 3 circles are required.\n"); // Output error
        return AQ;
    }

    // 1. Активируем ВСЕ окружности
    for (int i = 0; i < N; ++i)
    {
        all_circles[i].is_active = true;
    }

    SPoint A_final = A_prev; // Начинаем с A_prev
    double rms_final = 0.0;
    int used_count_final = 0; // Для записи использованных

    // 2. Выполняем ОДИН проход (print_residuals = true)
    if (perform_wls_pass(A_final, rms_final, used_count_final, true))
    {
        logi.log("Simple WLS Found A: (%+8.3f, %+8.3f)\n", A_final.x, A_final.y); // Output result

        // ЗАПОЛНЕНИЕ ФИНАЛЬНЫХ МЕТРИК
        AQ.A = A_final;
        AQ.quality = rms_final;
        AQ.used_measurements = used_count_final;
    }
    else
    {
        logi.log_r("Error: Simple WLS calculation failed.\n"); // Output error
    }
    logi.log("    Quality Assessment (SIMPLE, N=%d) Geometric RMS: **%6.3f m** (Used: %d, Total: %d, Outliers: No)\n", N, AQ.quality, AQ.used_measurements, AQ.total_measurements); // Output quality assessment

    return AQ;
}

/**
 * @brief Метод 2: Робастный WLS (Два прохода, с отбраковкой выбросов).
 */
SPoint_Q TrilaterationSolver::find_A_by_mnk_robust()
{
    SPoint A = {0.0, 0.0};
    SPoint_Q AQ; // Инициализация структуры качества
    AQ.A = A;
    AQ.quality = 999.0;
    AQ.has_outliers = false;
    AQ.total_measurements = all_circles.size();
    AQ.used_measurements = 0;

    int N = all_circles.size();

    logi.log("    --- SOLVER: ROBUST (Two-Pass) (Total N=%d) IN Aprev x= %6.3f y= %6.3f ---\n", N, A_prev.x, A_prev.y); // Output solver start

    if (N < 3)
    {
        logi.log_r("    --- ERROR: At least 3 circles are required.\n"); // Output error
        return AQ;
    }

    // 1. СБРОС ФЛАГОВ АКТИВНОСТИ
    for (int i = 0; i < N; ++i)
    {
        all_circles[i].is_active = true;
    }

    // --- ПРОХОД 1: Черновой расчет со всеми данными ---
    SPoint A_temp = A_prev;  // Начинаем с A_prev
    double rms_temp = 0.0;   // RMS для первого прохода
    int used_count_temp = 0; // Количество использованных в проходе 1

    // printf("\n--- SOLVER: PASS 1 (Initial WLS) ---\n"); // Output pass 1

    // Выполняем WLS, (print_residuals = false) -> без печати остатков
    if (!perform_wls_pass(A_temp, rms_temp, used_count_temp, false))
    {
        logi.log_r("Error: Pass 1 failed due to insufficient data or instability.\n"); // Output error
        return AQ;                                                                 // Возвращаем с плохим качеством
    }

    logi.log("    Pass 1 Found A: (%+8.3f, %+8.3f), Initial RMS: %+8.3f m (Threshold: %+8.3f m) \n", A_temp.x, A_temp.y, rms_temp, OUTLIER_REJECTION_THRESHOLD_METERS); // Output pass 1 result

    // --- АНАЛИЗ ОСТАТКОВ И ОТБРАКОВКА (на основе A_temp) ---
    int rejected_count = 0; // Счетчик отброшенных

    for (int i = 0; i < N; ++i)
    {
        const struct SCircle_W &current = all_circles[i];                            // Текущая
        double r_calc = sqrt(sqr(A_temp.x - current.x) + sqr(A_temp.y - current.y)); // Рассчитываем радиус до A_temp
        double residual = current.r - r_calc;                                        // Геометрический остаток

        if (fabs(residual) > OUTLIER_REJECTION_THRESHOLD_METERS) // Сравнение с порогом
        {
            all_circles[i].is_active = false; // Отбрасываем
            rejected_count++;
            // printf("   -Beacon %d (R=%+8.3f): Rejected! Residual=%+8.3f m\n", i, current.r, residual); // Output rejection
        }
        else
        {
            // printf("   +Beacon %d (R=%+8.3f): Accepted. Residual=%+8.3f m\n", i, current.r, residual); // Output acceptance
        }
    }

    logi.log_b("    Total rejected measurements: %d \n", rejected_count); // Output rejected count

    // --- ПРОХОД 2: Финальный расчет ---
    SPoint A_final = A_prev;                // Результат для финализации
    double rms_final = rms_temp;            // RMS для финализации
    int used_count_final = used_count_temp; // Изначально берем из первого прохода

    if (rejected_count > 0) // Если были отбракованные
    {
        // printf("\n--- SOLVER: PASS 2 (Final WLS without %d outliers) ---\n", rejected_count); // Output pass 2

        if (!perform_wls_pass(A_final, rms_final, used_count_final, true)) // print_residuals = true
        {
            logi.log_r("    Error: Pass 2 failed. Reverting to Pass 1 result (likely bad).\n"); // Output error
            A_final = A_temp;                                                           // Возвращаем результат Прохода 1
            rms_final = rms_temp;                                                       // Возвращаем RMS Прохода 1
            used_count_final = used_count_temp;
        }
        else
        {
            logi.log("    Pass 2 Found A (Final  WLS without %d outliers): (%+8.3f, %+8.3f)\n", rejected_count, A_final.x, A_final.y); // Output pass 2 result
            AQ.has_outliers = true;                                                                                          // Успешно отбросили выбросы
        }
    }
    else // Если ничего не отбраковано
    {
        // printf("\n--- SOLVER: FINAL RESULT (Pass 1 used as final) ---\n"); // Output final result
        // Повторно вызовем WLS с (print_residuals = true) для печати остатков
        perform_wls_pass(A_final, rms_final, used_count_final, true);
    }

    // 4. Формирование финальных метрик
    AQ.A = A_final;
    AQ.quality = rms_final;
    AQ.used_measurements = used_count_final;

    logi.log("    Quality Assessment (ROBUST, N=%d, Rejected=%d) Geometric RMS: **%6.3f m** (Used: %d, Total: %d, Outliers: %s) --- SOLVER: END --- \n", 
            N, rejected_count, AQ.quality, AQ.used_measurements, AQ.total_measurements, AQ.has_outliers ? "Yes" : "No"); // Output final quality

    return AQ;
}

// --- Дополнительные методы (Ориентация, Расчет углов) ---

double TrilaterationSolver::get_lidar_orientation(
    const SPoint A_found,                       // Найденное положение A
    const std::vector<SPoint> &beacons,         // Координаты маяков
    const std::vector<double> &lidar_angles_deg // Углы на маяки, измеренные лидаром
)
{
    if (beacons.size() != lidar_angles_deg.size() || beacons.empty()) // Проверка на соответствие и наличие данных
    {
        logi.log("    --- ORIENTATION CALCULATION SKIPPED ---\n");                      // Output skip header
        logi.log_r("    Error: Missing or mismatched data for orientation calculation.\n"); // Output error
        return 0.0;                                                                 // Возвращаем 0.0, если данных нет
    }

    logi.log("+++ ORIENTATION LSQ CALCULATION ---\n"); // Output LSQ header

    double sum_sin = 0.0;   // Сумма синусов разностей азимутов
    double sum_cos = 0.0;   // Сумма косинусов разностей азимутов
    int N = beacons.size(); // Количество измерений

    for (int i = 0; i < N; ++i) // Перебираем все пары (маяк, угол лидара)
    {
        double alpha_AM_deg = get_azimuth_deg(A_found, beacons[i]); // Рассчитываем азимут от A до маяка M (в градусах)
        double theta_lidar_deg = lidar_angles_deg[i];               // Угол на маяк, измеренный лидаром (в градусах)

        // Разница между азимутом (истинным) и измеренным углом лидара
        double psi_raw_deg = alpha_AM_deg - theta_lidar_deg;    // Ненормализованная оценка ориентации лидара // 3. ВОТ ЗДЕСЬ ПРОИСХОДИТ ВЫЧИТАНИЕ (Alpha - Theta = Psi)
        double psi_norm_deg = normalize_angle_deg(psi_raw_deg); // Нормализуем разницу в диапазон (-180, 180] (в градусах)
        double psi_rad = DEG2RAD(psi_norm_deg);                 // Переводим нормализованный угол в радианы

        sum_sin += sin(psi_rad); // Накапливаем синус
        sum_cos += cos(psi_rad); // Накапливаем косинус

        logi.log("    Beacon %d: Alpha= %+8.3f, Theta= %+8.3f -> Psi_norm= %+8.3f deg\n", // Output intermediate values
               i, alpha_AM_deg, theta_lidar_deg, psi_norm_deg);                     // Вывод промежуточных результатов
    }

    // Метод усреднения углов с помощью atan2:
    // Ориентация = atan2(Сумма_sin, Сумма_cos)
    double orientation_rad = atan2(sum_sin, sum_cos);  // Результат LSQ в радианах
    double orientation_deg = RAD2DEG(orientation_rad); // Переводим в градусы

    logi.log("    Sum Sin: %.4f, Sum Cos: %.4f | ALCULATION: END\n", sum_sin, sum_cos); // Output sums
    // printf("--- ORIENTATION LSQ CALCULATION: END ---\n");       // Output end LSQ

    return orientation_deg; // Возвращаем финальную ориентацию лидара в градусах
}

double TrilaterationSolver::calculate_angle_BAC(SPoint A, SPoint B, SPoint C)
{
    double az_AB = get_azimuth_deg(A, B);               // Азимут AB
    double az_AC = get_azimuth_deg(A, C);               // Азимут AC
    return calculate_angle_from_azimuths(az_AB, az_AC); // Расчет угла по разности азимутов
}

double TrilaterationSolver::calculate_angle_from_azimuths(double az_AB, double az_AC)
{
    double angle = az_AC - az_AB;          // Разница азимутов
    return normalize_angle_deg_abs(angle); // Нормализация и абсолютное значение
}

// Угловая разность: phi - theta, нормализованная в [-180, 180)
double TrilaterationSolver::angle_diff_deg(double phi, double theta)
{
    double diff = phi - theta;        // Расчет сырой разницы
    return normalize_angle_deg(diff); // Нормализуем разницу для получения кратчайшего пути
}

// --- ФИЛЬТР ---

// Комплементарный фильтр для углов (в градусах)
// alpha: вес нового измерения phi (0 <= alpha <= 1)
// theta: предыдущая оценка
// phi: новое измерение
double TrilaterationSolver::complementary_filter_angle_deg(double theta, double phi, double alpha)
{
    // Ограничение коэффициента
    if (alpha < 0.0)
        alpha = 0.0;
    if (alpha > 1.0)
        alpha = 1.0;

    double diff = angle_diff_deg(phi, theta); // наименьшее угловое смещение (error)
    double updated = theta + alpha * diff;    // движемся на alpha-часть этого смещения
    return normalize_angle_deg(updated);      // гарантируем диапазон [-180, 180)
}

/*
#include <stdio.h> // Для printf
#include <vector> // Для std::vector
#include <cmath> // Для математических функций
// Убедитесь, что ваш TrilaterationSolver.h находится в той же папке или подключен правильно
#include "TrilaterationSolver.h"

// ----------------------------------------------------------------------------------
// --- МИНИМАЛЬНЫЕ ЗАГЛУШКИ ДЛЯ ТЕСТИРОВАНИЯ (Обычно в code.h и config.h) ---
// ----------------------------------------------------------------------------------

// SPoint (для code.h)
struct SPoint
{
    double x;
    double y;
};

// SPoint_Q (для code.h)
struct SPoint_Q
{
    SPoint A;                      // Найденная точка
    double quality;                // Геометрический RMS (unweighted)
    int used_measurements;         // Сколько измерений использовано в финальном расчете
    int total_measurements;        // Всего доступно измерений
    bool has_outliers;             // Были ли выбросы отброшены (только для Robust)
};

// RAD2DEG и DEG2DEG (для config.h)
#define M_PI 3.14159265358979323846
#define RAD2DEG(rad) ((rad) * 180.0 / M_PI) // Макрос для перевода радианов в градусы
#define DEG2RAD(deg) ((deg) * M_PI / 180.0) // Макрос для перевода градусов в радианы

// --- Вспомогательная функция для квадрата (дублируем для main) ---
double sqr(double val)
{
    return val * val;
}
// ----------------------------------------------------------------------------------
// ----------------------------------------------------------------------------------

int main()
{
    // --- 1. ИСХОДНЫЕ ДАННЫЕ И ГЕОМЕТРИЯ ---

    // Истинные значения
    SPoint A_true = {1.0, 2.0}; // Истинное положение A
    SPoint A_prev = {1.0, 2.0}; // Предыдущее положение (Для точной отладки задаем A_true)

    // Координаты маяков
    SPoint B = {4.0, 0.3}; // Маяк B
    SPoint C = {0.0, 0.5}; // Маяк C
    SPoint D = {0.5, 4.0}; // Маяк D
    SPoint E = {5.0, 4.0}; // Маяк E

    // Истинные расстояния (для Теста 1 и 3)
    double R_AB_true = 3.448187; // R_true для B
    double R_AC_true = 1.802776; // R_true для C
    double R_AD_true = 2.061553; // R_true для D
    double R_AE_true = 4.472136; // R_true для E

    // Истинные углы (6 корректных измерений из точки A)
    // 4 последовательных угла
    double angle_BAC = 94.150;
    double angle_CAD = 132.270;
    double angle_DAE = 77.470;
    double angle_EAB = 56.101;
    // 2 непоследовательных угла (ИСПРАВЛЕНО!)
    double angle_BAD_CORRECT = 133.58;  // Угол BAD (B-D)
    double angle_CAE_CORRECT = 150.26;  // Угол CAE (C-E)

    // 2. Создание решателя
    TrilaterationSolver solver(A_prev); // Создаем объект решателя
    printf("--- INITIAL SETUP (THREE TEST SCENARIOS WITH 6 CORRECT ANGLES) ---\n"); // Output initial setup
    printf("True Position: (%+8.3f, %+8.3f)\n", A_true.x, A_true.y); // Output true position
    printf("------------------------------------------------------------------\n"); // Separator

    // ====================================================================
    // --- ТЕСТ 1: ТОЛЬКО ДАЛЬНОСТИ, ПРОСТОЙ WLS (N=4) ---
    // ====================================================================

    printf("\n################################################################################\n"); // Separator
    printf("### TEST 1: DISTANCE ONLY (4 meas.), SIMPLE WLS ###\n"); // Output test 1 start
    printf("################################################################################\n"); // Separator

    // Набор данных: 4 точных измерения дальности
    solver.add_circle_from_distance(B, R_AB_true); // Dist B (W=1.0)
    solver.add_circle_from_distance(C, R_AC_true); // Dist C (W=1.0)
    solver.add_circle_from_distance(D, R_AD_true); // Dist D (W=1.0)
    solver.add_circle_from_distance(E, R_AE_true); // Dist E (W=1.0)

    printf("\n--- RUN SIMPLE WLS ---\n"); // Output run simple WLS
    SPoint_Q result_test1 = solver.find_A_by_mnk_simple();

    printf("\n--- SUMMARY TEST 1 (DISTANCE SIMPLE) ---\n"); // Output summary 1
    printf("Result A: (%.6f, %.6f)\n", result_test1.A.x, result_test1.A.y); // Output result A
    printf("Position Error: %.9f m\n", sqrt(sqr(result_test1.A.x - A_true.x) + sqr(result_test1.A.y - A_true.y))); // Output error
    printf("RMS:      %.6f m\n", result_test1.quality); // Output RMS
    printf("Used/Total: %d/%d\n", result_test1.used_measurements, result_test1.total_measurements); // Output used/total
    printf("----------------------------------------\n"); // Separator

    solver.clear_circles(); // !!! ОБНУЛЯЕМ ТАБЛИЦУ !!!

    // ====================================================================
    // --- ТЕСТ 2: ТОЛЬКО УГЛЫ, ПРОСТОЙ WLS (N=6) ---
    // ====================================================================

    printf("\n################################################################################\n"); // Separator
    printf("### TEST 2: ANGLE ONLY (6 meas.), SIMPLE WLS ###\n"); // Output test 2 start
    printf("################################################################################\n"); // Separator

    // Набор данных: 6 точных измерений углов (все из A)
    solver.add_filtered_circle_from_angle(C, B, angle_BAC); // Угол BAC (C-B)
    solver.add_filtered_circle_from_angle(C, D, angle_CAD); // Угол CAD (C-D)
    solver.add_filtered_circle_from_angle(D, E, angle_DAE); // Угол DAE (D-E)
    solver.add_filtered_circle_from_angle(E, B, angle_EAB); // Угол EAB (E-B)
    solver.add_filtered_circle_from_angle(B, D, angle_BAD_CORRECT); // Угол BAD (B-D)
    solver.add_filtered_circle_from_angle(C, E, angle_CAE_CORRECT); // Угол CAE (C-E)

    printf("\n--- RUN SIMPLE WLS ---\n"); // Output run simple WLS
    SPoint_Q result_test2 = solver.find_A_by_mnk_simple(); // Вызов простого WLS

    printf("\n--- SUMMARY TEST 2 (ANGLE SIMPLE) ---\n"); // Output summary 2
    printf("Result A: (%.6f, %.6f)\n", result_test2.A.x, result_test2.A.y); // Output result A
    printf("Position Error: %.9f m\n", sqrt(sqr(result_test2.A.x - A_true.x) + sqr(result_test2.A.y - A_true.y))); // Output error
    printf("RMS:      %.6f m\n", result_test2.quality); // Output RMS
    printf("Used/Total: %d/%d\n", result_test2.used_measurements, result_test2.total_measurements); // Output used/total
    printf("--------------------------------------\n"); // Separator

    solver.clear_circles(); // !!! ОБНУЛЯЕМ ТАБЛИЦУ !!!

    // ====================================================================
    // --- ТЕСТ 3: СМЕШАННЫЕ ДАННЫЕ (8 meas.) + ВЫБРОС, РОБАСТНЫЙ WLS ---
    // ====================================================================

    printf("\n################################################################################\n"); // Separator
    printf("### TEST 3: MIXED DATA (8 meas.) + OUTLIER, ROBUST WLS ###\n"); // Output test 3 start
    printf("################################################################################\n"); // Separator

    // --- Набор данных: 8 измерений (4 Dist + 4 Angle) с 1 выбросом ---

    // 1. Точная дальность (B)
    solver.add_circle_from_distance(B, R_AB_true);

    // 2. Дальность с ГРУБОЙ ОШИБКОЙ (C) -> Выброс
    double R_AC_BAD = R_AC_true + 0.500; // Ошибка 50 см
    solver.add_circle_from_distance(C, R_AC_BAD);

    // 3. Точная дальность (D)
    solver.add_circle_from_distance(D, R_AD_true);

    // 4. Точная дальность (E)
    solver.add_circle_from_distance(E, R_AE_true);

    // 5-8. 6 угловых измерения
    solver.add_filtered_circle_from_angle(C, B, angle_BAC);
    solver.add_filtered_circle_from_angle(C, D, angle_CAD);
    solver.add_filtered_circle_from_angle(D, E, angle_DAE);
    solver.add_filtered_circle_from_angle(E, B, angle_EAB);
    solver.add_filtered_circle_from_angle(B, D, angle_BAD_CORRECT); // Угол BAD (B-D)
    solver.add_filtered_circle_from_angle(C, E, angle_CAE_CORRECT); // Угол CAE (C-E)

    printf("\n--- RUN ROBUST WLS (EXPECT OUTLIER C) ---\n"); // Output run robust WLS
    SPoint_Q result_test3 = solver.find_A_by_mnk_robust(); // Вызов робастного WLS

    printf("\n--- SUMMARY TEST 3 (MIXED ROBUST) ---\n"); // Output summary 3
    printf("Result A: (%.6f, %.6f)\n", result_test3.A.x, result_test3.A.y); // Output result A
    printf("Position Error: %.9f m\n", sqrt(sqr(result_test3.A.x - A_true.x) + sqr(result_test3.A.y - A_true.y))); // Output error
    printf("RMS:      %.6f m\n", result_test3.quality); // Output RMS
    printf("Used/Total: %d/%d\n", result_test3.used_measurements, result_test3.total_measurements); // Output used/total
    printf("Outliers: %s\n", result_test3.has_outliers ? "Yes" : "No"); // Output outliers
    printf("--------------------------------------\n"); // Separator

    // ====================================================================
    // --- ТЕСТ 4: РАСЧЕТ ОРИЕНТАЦИИ ЛИДАРА (ДОПОЛНЕНИЕ) ---
    // ====================================================================

    printf("\n################################################################################\n"); // Separator
    printf("### TEST 4: LIDAR ORIENTATION CALCULATION ###\n"); // Output test 4 start
    printf("################################################################################\n"); // Separator

    // --- 1. ИСХОДНЫЕ ДАННЫЕ ДЛЯ РАСЧЕТА ОРИЕНТАЦИИ ---

    // Координаты маяков, на которые были прямые измерения (B, C, D, E)
    // Эти переменные уже объявлены ранее в main.
    SPoint beacon_B = B;
    SPoint beacon_C = C;
    SPoint beacon_D = D;
    SPoint beacon_E = E;

    // Измеренные углы лидара (в градусах)
    // Эти углы соответствуют: Lidar_Angle = Azimuth - True_Orientation (20.0 deg)
    double angle_to_B = 310.46;
    double angle_to_C = 216.31;
    double angle_to_D = 84.04;
    double angle_to_E = 6.57;

    // Истинная ориентация лидара для симуляции и сравнения
    double true_orientation_deg = 20.0;

    // --- 2. ЗАПОЛНЕНИЕ ВЕКТОРОВ ИЗ ПЕРЕМЕННЫХ ---

    // Вектор координат маяков
    std::vector<SPoint> orientation_beacons;
    orientation_beacons.push_back(beacon_B);
    orientation_beacons.push_back(beacon_C);
    orientation_beacons.push_back(beacon_D);
    orientation_beacons.push_back(beacon_E);

    // Вектор измеренных углов
    std::vector<double> lidar_angles_deg;
    lidar_angles_deg.push_back(angle_to_B);
    lidar_angles_deg.push_back(angle_to_C);
    lidar_angles_deg.push_back(angle_to_D);
    lidar_angles_deg.push_back(angle_to_E);

    // 1. Вызываем метод расчета ориентации, используя найденную позицию
    double calculated_orientation = solver.get_lidar_orientation(
        result_test3.A,                // Найденное положение (A_final из Pass 2)
        orientation_beacons,           // Координаты маяков (заполнены из переменных)
        lidar_angles_deg               // Измеренные углы лидара (заполнены из переменных)
    );

    // 2. Вывод результата
    printf("\n--- SUMMARY TEST 4 (ORIENTATION) ---\n"); // Output summary 4
    printf("Position A used: (%.6f, %.6f)\n", result_test3.A.x, result_test3.A.y); // Output result A
    printf("Simulated True Orientation: %.2f deg\n", true_orientation_deg); // Output true orientation
    printf("Calculated Orientation Psi: %.4f deg\n", calculated_orientation); // Output calculated orientation
    printf("--------------------------------------\n"); // Separator

    return 0; // Завершение программы
}
*/
#endif // TRIALATERATIONSOLVER_H