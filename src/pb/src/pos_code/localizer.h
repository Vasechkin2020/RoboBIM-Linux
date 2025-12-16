#ifndef LOCALIZER_H
#define LOCALIZER_H

#include <stdio.h>
#include <math.h>
#include <stdlib.h>

// --- Вспомогательные функции ---
// void normalizeAngle(double &angle)
// {
//     while (angle > M_PI)
//     {
//         angle -= 2.0 * M_PI; // вычитаем 2π, пока угол не станет ≤ π
//     }
//     while (angle < -M_PI)
//     {
//         angle += 2.0 * M_PI; // прибавляем 2π, пока угол не станет ≥ -π
//     }
// }

// double angleDiff(double a, double b)
// {
//     double d = a - b; // вычисление разности углов
//     normalizeAngle(d);    // нормализация разности
//     return d;             // возврат кратчайшей разности
// }

// --- Класс 1: Rate-Limited Fuser ---
class RateLimitedLocalizer
{
public:
    double max_pos_step = 0.02; // Шаг на который приближаемся к измерению от модели
    double max_angle_step = 0.50; // В градусах

    SPose fuse(const SPose &pose_model_in, // Входное положение Модели (const)
              const SPose &pose_meas_old, // Структура Измерения (старое, const)
              double v_model, double omega_model, // Скорости Модели
              double lidar_latency_L)           // Задержка
    {
        SPose pose_model = pose_model_in; // Копируем входное состояние для локальной модификации

        // --- Шаг 1: Прогноз (Экстраполяция) Измерения ---
        SPose pose_meas_current; 
        pose_meas_current.x = pose_meas_old.x + v_model * lidar_latency_L * cos(pose_meas_old.th); // Прогноз X измерения
        pose_meas_current.y = pose_meas_old.y + v_model * lidar_latency_L * sin(pose_meas_old.th); // Прогноз Y измерения
        pose_meas_current.th = pose_meas_old.th + omega_model * lidar_latency_L;                   // Прогноз Угла измерения
        normalizeAngle180(pose_meas_current.th);                                                       // Нормализация угла

        // --- Шаг 2: Коррекция (Rate-Limited) ---
        double gain = 0.333;  // Общий коэффициент плавности

        // 2.1. Коррекция позиции
        double dx = pose_meas_current.x - pose_model.x; // разность по X между Прогнозным и Моделью
        double dy = pose_meas_current.y - pose_model.y; // разность по Y
        double dist_error = hypot(dx, dy);                    // расстояние до прогнозируемой точки

        if (dist_error > 0.001) // Если есть расхождение (1 мм)
        {

            //ТАК БЫЛО В ПЕРВОМ ВАРИАНТЕ
            // double ratio = max_pos_step / dist_error; // доля пути, которую можно пройти
            // if (ratio > 1.0)                    
            // {
            //     ratio = 1.0; 
            // }
            
            double move_dist = dist_error * gain;// Умная коррекция: хотим исправить 33% ошибки
            if (move_dist > max_pos_step) move_dist = max_pos_step;// Но не больше чем max_pos_step
            double ratio = move_dist / dist_error; // Считаем пропорцию для проекций dx/dy (сохраняем направление вектора ошибки)

            pose_model.x += dx * ratio; // плавно сдвигаем X Модели
            pose_model.y += dy * ratio; // плавно сдвигаем Y Модели
        }

        // 2.2. Коррекция угла
        double dtheta = angle_diff_deg(pose_meas_current.th, pose_model.th); // кратчайшая разность углов

        if (fabs(dtheta) > 0.005) // Если есть угловое расхождение (0.005 градус)
        {
            //ТАК БЫЛО В ПЕРВОМ ВАРИАНТЕ
            // double ratio = max_angle_step / fabs(dtheta); // доля угла для поворота
            // if (ratio > 1.0)                              
            // {
            //     ratio = 1.0; 
            // }
            // pose_model.th += dtheta * ratio; // плавно поворачиваем Угол Модели

            // КОЭФФИЦИЕНТ СГЛАЖИВАНИЯ (Gain)   0.3 означает: исправляем 30% ошибки за один раз. Это уберет дерганье "перекрутил/недокрутил".
            double correction = dtheta * gain;
            // Ограничиваем максимальный шаг (твоя защита)
            if (correction > max_angle_step) correction = max_angle_step;
            if (correction < -max_angle_step) correction = -max_angle_step;

            pose_model.th += correction;


            normalizeAngle180(pose_model.th);   // нормализуем
        }
        
        return pose_model; // Возвращаем новое, слитое положение
    }
};

// --- Класс 2: Complementary Fuser ---
class ComplementaryLocalizer
{
public:
    double alpha_pos = 0.2;
    double alpha_angle = 0.3;

    SPose fuse(const SPose &pose_model_in, // Входное положение Модели (const)
              const SPose &pose_meas_old, // Структура Измерения (старое, const)
              double v_model, double omega_model, // Скорости
              double lidar_latency_L)           // Задержка
    {
        SPose pose_model = pose_model_in; // Копируем входное состояние для локальной модификации
        
        // --- Шаг 1: Прогноз (Экстраполяция) Измерения ---
        SPose pose_meas_current; 
        
        pose_meas_current.x = pose_meas_old.x + v_model * lidar_latency_L * cos(pose_meas_old.th);     // Прогноз X измерения
        pose_meas_current.y = pose_meas_old.y + v_model * lidar_latency_L * sin(pose_meas_old.th);     // Прогноз Y измерения
        pose_meas_current.th = pose_meas_old.th + omega_model * lidar_latency_L;                       // Прогноз Угла измерения
        normalizeAngle180(pose_meas_current.th);                                                           // Нормализация угла

        // --- Шаг 2: Коррекция (Blending) ---

        // 2.1. Коррекция позиции
        double dx = pose_meas_current.x - pose_model.x; // Ошибка по X
        double dy = pose_meas_current.y - pose_model.y; // Ошибка по Y
        double dist = hypot(dx, dy); // Расстояние

        if (dist > 0.001) // Если есть расхождение (1 мм)
        {
            pose_model.x += alpha_pos * dx; // корректируем X Модели пропорционально ошибке
            pose_model.y += alpha_pos * dy; // корректируем Y Модели пропорционально ошибке
        }


        // 2.2. Коррекция угла
        double dtheta = angle_diff_deg(pose_meas_current.th, pose_model.th); // Разность углов

        if (fabs(dtheta) > 0.005) // Если есть угловое расхождение (0.005 градусов)
        {
            pose_model.th += alpha_angle * dtheta; // корректируем Угол Модели пропорционально ошибке
            normalizeAngle180(pose_model.th);         // Нормализация
        }
        
        return pose_model; // Возвращаем новое, слитое положение
    }
};

/*
// --- Главная функция ---

int main()
{
    // Создаем два экземпляра "Фьюзеров" (Fusers) для слияния данных
    RateLimitedLocalizer rate_fuser;     // Экземпляр Rate-Limited
    ComplementaryLocalizer comp_fuser;   // Экземпляр Complementary

    // --- Настройка параметров ---
    const double lidar_latency_L = 0.1; // Задержка лидара/SLAM: 100 мс

    // Настройка параметров слияния
    rate_fuser.max_pos_step = 0.04;     // Максимальный шаг коррекции позиции (0.04 м)
    comp_fuser.alpha_pos = 0.3;         // Коэффициент доверия измерению позиции (30%)

    const double dt = 0.01;             // Временной шаг симуляции: 10 мс (100 Гц)
    const int steps = 2000;             // Общее число шагов: 20 секунд симуляции

    // --- Параметры истинной траектории и дрейфа ---
    const double v_true = 0.8;          // Истинная линейная скорость (0.8 м/с)
    const double omega_true = 0.05;     // Истинная угловая скорость (0.05 рад/с)

    // Имитация постоянной ошибки одометрии (ДРЕЙФ!)
    const double v_odom_drift = 0.90;   // Линейная скорость одометрии (0.9 м/с)
    const double omega_odom_drift = 0.0;// Угловая скорость одометрии (0.0 рад/с)

    // --- Генератор шума для лидара ---
    srand(1);                           // Фиксированный seed для повторяемости шума
    double pos_noise_std = 0.1;         // СКО шума лидара (10 см)
    double angle_noise_std = 0.05;      // СКО шума лидара (0.05 рад)

    printf("Comparison of Fusers with latency compensation (L=%.2fs):\n", lidar_latency_L); // Вывод заголовка
    printf("t (s) | Pos Error (R-Limited) | Pos Error (Complementary)\n");                          // Вывод заголовков столбцов
    printf("-------------------------------------------------------\n");                            // Разделительная линия

    // Истинное положение (для симуляции)
    SPose true_pose; // true_pose.x, .y, .th по умолчанию 0

    // --- СОСТОЯНИЕ МОДЕЛИ (Хранится в main) ---
    SPose model_pose_rate; // Состояние для Rate-Limited
    SPose model_pose_comp; // Состояние для Complementary


    // Основной цикл симуляции
    for (int i = 0; i < steps; ++i)
    {
        double t = i * dt; // Текущее время симуляции

        // Истинное положение: интегрируем идеальные скорости
        true_pose.x += v_true * cos(true_pose.th) * dt;    // Интегрируем истинную позицию X
        true_pose.y += v_true * sin(true_pose.th) * dt;    // Интегрируем истинную позицию Y
        true_pose.th += omega_true * dt;                  // Интегрируем истинный угол
        normalizeAngle(true_pose.th);                     // Нормализуем истинный угол

        // 1. Шаг "Расчет по Модели" (100 Гц) - Интеграция одометрии с ДРЕЙФОМ
        // Выполняется вне классов
        model_pose_rate.x += v_odom_drift * cos(model_pose_rate.th) * dt; // Прогноз X для Rate-Limited
        model_pose_rate.y += v_odom_drift * sin(model_pose_rate.th) * dt; // Прогноз Y для Rate-Limited
        model_pose_rate.th += omega_odom_drift * dt;                 // Прогноз угла для Rate-Limited
        normalizeAngle(model_pose_rate.th);                          // Нормализация

        model_pose_comp.x += v_odom_drift * cos(model_pose_comp.th) * dt; // Прогноз X для Complementary
        model_pose_comp.y += v_odom_drift * sin(model_pose_comp.th) * dt; // Прогноз Y для Complementary
        model_pose_comp.th += omega_odom_drift * dt;                 // Прогноз угла для Complementary
        normalizeAngle(model_pose_comp.th);                          // Нормализация


        // 2. Шаг Слияния (10 Гц = каждые 10 шагов)
        if (i % 10 == 0)
        {
            // Имитация шумного "Измерения" (создаем SPose)
            SPose lidar_pose;
            lidar_pose.x = true_pose.x + ((double)rand() / RAND_MAX - 0.5) * pos_noise_std * 2.0;       // X с шумом
            lidar_pose.y = true_pose.y + ((double)rand() / RAND_MAX - 0.5) * pos_noise_std * 2.0;       // Y с шумом
            lidar_pose.th = true_pose.th + ((double)rand() / RAND_MAX - 0.5) * angle_noise_std * 2.0;    // Угол с шумом
            normalizeAngle(lidar_pose.th);               // Нормализация угла

            // ВЫЗОВ С ИСПОЛЬЗОВАНИЕМ НОВОГО ВОЗВРАЩАЕМОГО ЗНАЧЕНИЯ
            model_pose_rate = rate_fuser.fuse(model_pose_rate, lidar_pose, 
                                             v_odom_drift, omega_odom_drift,           
                                             lidar_latency_L);                           

            model_pose_comp = comp_fuser.fuse(model_pose_comp, lidar_pose, 
                                             v_odom_drift, omega_odom_drift,           
                                             lidar_latency_L);                           
        }

        // Вывод ошибки раз в секунду (каждые 100 шагов)
        if (i % 100 == 0)
        {
            // Ошибка Rate-Limited
            double err_pos_r = hypot(model_pose_rate.x - true_pose.x, model_pose_rate.y - true_pose.y); // Ошибка позиции

            // Ошибка Complementary
            double err_pos_c = hypot(model_pose_comp.x - true_pose.x, model_pose_comp.y - true_pose.y); // Ошибка позиции

            printf("%.2f | %.4f | %.4f\n",
                    t, err_pos_r, err_pos_c); // Вывод времени и ошибок
        }
    }

    return 0; // Завершение программы
}

*/

#endif

