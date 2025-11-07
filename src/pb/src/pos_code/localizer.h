
#ifndef LOCALIZER_H
#define LOCALIZER_H

#include <stdio.h>  // Для printf предпочтительней
#include <math.h>   // Для cos, sin, M_PI, hypot, fabs
#include <stdlib.h> // Для rand (имитация шума)

// --- Вспомогательные функции ---
/**
 * @brief Приводит угол к диапазону [-PI, PI]
 * @param angle Ссылка на угол в радианах
 */
void normalizeAngle(double &angle)
{
    while (angle > M_PI)
    {
        angle -= 2.0 * M_PI; // вычитаем 2π, пока угол не станет ≤ π
    }
    while (angle < -M_PI)
    {
        angle += 2.0 * M_PI; // прибавляем 2π, пока угол не станет ≥ -π
    }
}

/**
 * @brief Возвращает кратчайшую разность между двумя углами a и b (a - b)
 * @param a Первый угол
 * @param b Второй угол
 * @return Кратчайшая разность в радианах [-PI, PI]
 */
double angleDiff(double a, double b)
{
    double d = a - b;
    normalizeAngle(d);
    return d;
}

// --- Класс 1: Ваш метод (Rate-Limited Correction) с Прогнозом ---
class RateLimitedLocalizer
{
public:
    double x = 0.0;     // текущая оценка координаты X (метры)
    double y = 0.0;     // текущая оценка координаты Y (метры)
    double theta = 0.0; // текущая оценка угла поворота (радианы)

    // Параметры настройки - "Ограничение шага"
    double max_pos_step = 0.03;   // максимальное расстояние коррекции за 100 мс (метры)
    double max_angle_step = 0.05; // максимальный угол коррекции за 100 мс (радианы)

    // Параметр для компенсации задержки
    double lidar_latency_L = 0.1; // Эмпирическая задержка лидара (100 мс)

    // Скорости, сохраненные на последнем шаге predict, для экстраполяции
    double last_v = 0.0;
    double last_omega = 0.0;

    // Метод прогноза (общий для всех)
    void predict(double v, double omega, double dt)
    {
        last_v = v;         // сохраняем текущие скорости для экстраполяции
        last_omega = omega; // сохраняем текущие угловые скорости

        x += v * cos(theta) * dt; // интегрируем X по модели
        y += v * sin(theta) * dt; // интегрируем Y по модели
        theta += omega * dt;      // интегрируем угол по модели
        normalizeAngle(theta);    // нормализуем угол
    }

    // Метод коррекции с учётом задержки: ограничивает скорость схождения к прогнозируемой точке
    void updatePose_Extrapolated(double x_meas_old, double y_meas_old, double theta_meas_old)
    {
        // --- Шаг 1: Прогноз (Экстраполяция) Измерения ---

        // Прогнозируем, где измерение было бы СЕЙЧАС, если бы двигалось с последней скоростью
        double x_meas_current = x_meas_old + last_v * lidar_latency_L * cos(theta_meas_old);
        double y_meas_current = y_meas_old + last_v * lidar_latency_L * sin(theta_meas_old);
        double theta_meas_current = theta_meas_old + last_omega * lidar_latency_L;
        normalizeAngle(theta_meas_current);

        // --- Шаг 2: Коррекция (Rate-Limited) ---

        // 2.1. Коррекция позиции
        double dx = x_meas_current - x; // разность по X между прогнозом и оценкой
        double dy = y_meas_current - y; // разность по Y
        double dist = hypot(dx, dy);    // расстояние до прогнозируемой точки

        if (dist > 1e-6)
        {
            double ratio = max_pos_step / dist; // доля пути, которую можно пройти (max_step / total_error)
            if (ratio > 1.0)
                ratio = 1.0; // если ошибка мала, идём сразу в неё

            x += dx * ratio; // плавно сдвигаем X
            y += dy * ratio; // плавно сдвигаем Y
        }

        // 2.2. Коррекция угла
        double dtheta = angleDiff(theta_meas_current, theta); // кратчайшая разность углов
        if (fabs(dtheta) > 1e-6)
        {
            double ratio = max_angle_step / fabs(dtheta); // доля угла для поворота (max_step / total_error)
            if (ratio > 1.0)
                ratio = 1.0; // если ошибка мала, поворачиваем сразу

            theta += dtheta * ratio; // плавно поворачиваем
            normalizeAngle(theta);   // нормализуем
        }
    }
};

// --- Класс 2: Комплементарный Фильтр (Blending Correction) с Прогнозом ---
class ComplementaryLocalizer
{
public:
    double x = 0.0;     // текущая оценка координаты X
    double y = 0.0;     // текущая оценка координаты Y
    double theta = 0.0; // текущая оценка угла поворота

    // Параметры настройки - "Коэффициент смешивания (доверия)"
    double alpha_pos = 0.2;   // 20% доверия новому измерению позиции (0.0 - 1.0)
    double alpha_angle = 0.3; // 30% доверия новому измерению угла (0.0 - 1.0)

    // Параметр для компенсации задержки
    double lidar_latency_L = 0.1; // Эмпирическая задержка лидара (100 мс)

    // Скорости, сохраненные на последнем шаге predict, для экстраполяции
    double last_v = 0.0;
    double last_omega = 0.0;

    // Метод прогноза (должен сохранять скорости)
    void predict(double v, double omega, double dt)
    {
        last_v = v;         // сохраняем текущие скорости для экстраполяции
        last_omega = omega; // сохраняем текущие угловые скорости

        x += v * cos(theta) * dt;
        y += v * sin(theta) * dt;
        theta += omega * dt;
        normalizeAngle(theta);
    }

    // Метод коррекции с учётом задержки: смешивает старую оценку с прогнозируемой точкой
    void updatePose_Extrapolated(double x_meas_old, double y_meas_old, double theta_meas_old)
    {
        // --- Шаг 1: Прогноз (Экстраполяция) Измерения ---

        // Прогнозируем, где измерение было бы СЕЙЧАС, если бы двигалось с последней скоростью
        double x_meas_current = x_meas_old + last_v * lidar_latency_L * cos(theta_meas_old);
        double y_meas_current = y_meas_old + last_v * lidar_latency_L * sin(theta_meas_old);
        double theta_meas_current = theta_meas_old + last_omega * lidar_latency_L;
        normalizeAngle(theta_meas_current);

        // --- Шаг 2: Коррекция (Blending) ---

        // 2.1. Коррекция позиции
        double dx = x_meas_current - x; // Ошибка по X между прогнозом и нашей оценкой
        double dy = y_meas_current - y; // Ошибка по Y

        x += alpha_pos * dx; // корректируем X пропорционально ошибке
        y += alpha_pos * dy; // корректируем Y пропорционально ошибке

        // 2.2. Коррекция угла
        double dtheta = angleDiff(theta_meas_current, theta);

        theta += alpha_angle * dtheta; // корректируем угол пропорционально ошибке
        normalizeAngle(theta);
    }
};

/*
// --- Главная функция ---

int main()
{
    // Создаем два экземпляра локализаторов для сравнения
    RateLimitedLocalizer rate_loc;
    ComplementaryLocalizer comp_loc;

    // --- Настройка параметров ---
    // Установим одинаковую задержку
    rate_loc.lidar_latency_L = 0.1;
    comp_loc.lidar_latency_L = 0.1;

    // Параметры скорости коррекции, настроенные для стабильного сравнения
    rate_loc.max_pos_step = 0.04;   // Увеличим max_step, чтобы Rate-Limited лучше справлялся с дрейфом
    comp_loc.alpha_pos = 0.3;       // Увеличим alpha, чтобы комплементарный фильтр быстрее сходился

    const double dt = 0.01;         // Временной шаг: 10 мс (100 Гц)
    const int steps = 2000;         // Общее число шагов: 20 секунд симуляции

    // --- Параметры истинной траектории и дрейфа ---
    const double v_true = 0.8;      // Истинная линейная скорость
    const double omega_true = 0.05; // Истинная угловая скорость

    // Имитация постоянной ошибки одометрии (ДРЕЙФ!)
    const double v_odom_drift = 0.90;   // Одометрия завышает скорость на 10 см/с (сильный дрейф)
    const double omega_odom_drift = 0.0; // Одометрия занижает угловую скорость

    // --- Генератор шума для лидара ---
    srand(1);                       // Фиксированный seed
    double pos_noise_std = 0.1;     // СКО шума лидара 10 см

    printf("Сравнение фильтров с компенсацией задержки (L=%.2fs):\n", rate_loc.lidar_latency_L);
    printf("t (s) | Err Pos (R-Limited) | Err Pos (Complementary)\n");
    printf("-------------------------------------------------------\n");

    // Истинное положение (переменные, которые хранят "правду")
    double true_x = 0.0, true_y = 0.0, true_theta = 0.0;

    // Основной цикл симуляции
    for (int i = 0; i < steps; ++i)
    {
        double t = i * dt;

        // Истинное положение: интегрируем идеальные скорости
        true_x += v_true * cos(true_theta) * dt;
        true_y += v_true * sin(true_theta) * dt;
        true_theta += omega_true * dt;
        normalizeAngle(true_theta);

        // 1. Шаг Прогноза (100 Гц) - Используем ЗАШУМЛЕННУЮ одометрию
        rate_loc.predict(v_odom_drift, omega_odom_drift, dt);
        comp_loc.predict(v_odom_drift, omega_odom_drift, dt);

        // 2. Шаг Коррекции (10 Гц = каждые 10 шагов)
        if (i % 10 == 0)
        {
            // Имитация измерения, сделанного 100 мс назад (L=0.1s)

            // Расчитываем, где ИСТИННОЕ положение было 100 мс назад (для имитации)
            // Истинное t_meas = t - L. В данной симуляции мы не можем "откатиться",
            // поэтому просто берем текущую позицию и добавляем шум. В реальном коде
            // это будет измерение от лидара с меткой времени.

            // Имитация шумного измерения (шум лидара)
            double x_lidar = true_x + ((double)rand() / RAND_MAX - 0.5) * pos_noise_std * 2.0;
            double y_lidar = true_y + ((double)rand() / RAND_MAX - 0.5) * pos_noise_std * 2.0;
            double th_lidar = true_theta + ((double)rand() / RAND_MAX - 0.5) * 0.05 * 2.0; // шум угла

            // Имитируем, что это измерение "застряло" в буфере на 100 мс.
            // Внимание: В реальном коде x_lidar будет соответствовать x(t - L).
            // В нашей упрощенной симуляции мы просто показываем, что мы "корректируем"
            // текущей позицией, которая на самом деле "старая".

            rate_loc.updatePose_Extrapolated(x_lidar, y_lidar, th_lidar);
            comp_loc.updatePose_Extrapolated(x_lidar, y_lidar, th_lidar);
        }

        // Вывод ошибки раз в секунду (каждые 100 шагов)
        if (i % 100 == 0)
        {
            // Ошибка Rate-Limited
            double err_pos_r = hypot(rate_loc.x - true_x, rate_loc.y - true_y);

            // Ошибка Complementary
            double err_pos_c = hypot(comp_loc.x - true_x, comp_loc.y - true_y);

            printf("%.2f  | %.4f                | %.4f\n",
                   t, err_pos_r, err_pos_c);
        }
    }

    return 0;
}
*/


#endif