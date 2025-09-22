#ifndef JERK_H
#define JERK_H

#include <stdio.h>  // Подключаем стандартную библиотеку ввода-вывода — для printf (в диагностике)
#include <stdlib.h> // Подключаем стандартную библиотеку — резерв (может пригодиться)
#include <math.h>   // Подключаем математическую библиотеку — для fabs, sqrt
#include <cmath>    // Подключаем математическую библиотеку — для fabs, sqrt
#include <cmath>    // Подключаем математическую библиотеку — для fabs, sqrt
#include "config.h"

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h>

// Состояния профиля
typedef enum
{
    JLP_IDLE = 0,
    JLP_RUNNING = 1,
    JLP_COASTING_J = 2,
    JLP_COASTING_A = 3
} JLP_State;

// Структура состояния профиля
typedef struct
{
    const char *wheel_name; // Имя колеса для логов

    // Текущее состояние движения
    double v_current; // Текущая скорость (м/с)
    double a_current; // Текущее ускорение (м/с²)
    double j_current; // Текущий рывок (м/с³)
    double t_current; // Текущее время профиля (с)
    int phase;        // Фаза: 1=нарастание, 3=сброс (фаза 2 не используется в треугольном)

    // Параметры профиля
    double v_target;      // Целевая скорость (м/с)
    double v_start;       // Начальная скорость профиля (сохраняется при старте)
    double t1_end;        // Время окончания фазы 1
    double t2_end;        // Время окончания фазы 2
    double t_total;       // Общее время профиля
    int has_cruise_phase; // 1 — трапеция, 0 — треугольник
    double a_peak_used;   // Пиковое ускорение
    int direction;        // +1=разгон, -1=торможение

    // Состояние автомата
    JLP_State state;
    int is_finished;

    // Параметры системы
    double j_max; // Макс. рывок (м/с³)
    double a_max; // Макс. ускорение (м/с²)
    double v_max; // Макс. скорость (м/с)

    // Для пересчёта
    int replan_requested;
    double v_target_new;
    double t_coast_j;
    double t_coast_a;
    double t_coast_elapsed;
    double j_coast_sign;
    double a_coast_sign;

    // Диагностика
    int enable_diagnostics;
} JerkLimitedProfile;

// Инициализация профиля
void jlp_init(JerkLimitedProfile *p, const char *name, double v_start, double j_max, double a_max, double v_max)
{
    p->wheel_name = name;
    p->v_current = v_start;
    p->a_current = 0.0;
    p->j_current = 0.0;
    p->t_current = 0.0;
    p->phase = 1;
    p->v_target = v_start;
    p->state = JLP_IDLE;
    p->is_finished = 1;
    p->j_max = j_max;
    p->a_max = a_max;
    p->v_max = v_max;
    p->replan_requested = 0;
    p->t_coast_elapsed = 0.0;
    p->enable_diagnostics = 0;
}

// Запуск профиля
void jlp_start_profile(JerkLimitedProfile *p, double v_target)
{
    // Ограничиваем целевую скорость
    double v_target_limited = v_target;
    if (p->v_max > 0)
    {
        v_target_limited = fmax(-p->v_max, fmin(v_target, p->v_max));
    }

    p->v_target = v_target_limited;
    p->v_start = p->v_current; // ← Сохраняем начальную скорость для аналитического расчёта
    p->t_current = 0.0;
    p->phase = 1;
    p->is_finished = 0;

    double dv = v_target_limited - p->v_current;
    p->direction = (dv >= 0.0) ? 1 : -1;
    double dv_abs = fabs(dv);

    if (dv_abs < 1e-9)
    {
        p->t1_end = p->t2_end = p->t_total = 0.0;
        p->has_cruise_phase = 0;
        p->a_peak_used = 0.0;
        p->state = JLP_IDLE;
        p->is_finished = 1;
        if (p->enable_diagnostics)
            printf("[%s][DIAG] Профиль не требуется — dv=0\n", p->wheel_name);
        return;
    }

    double t_j_full = p->a_max / p->j_max;
    double v_ramp_full = 0.5 * p->a_max * t_j_full;
    double v_ramp_total = 2.0 * v_ramp_full;

    // ИСПРАВЛЕНО: было >=, стало > — при равенстве выбираем треугольник
    if (dv_abs > v_ramp_total)
    {
        // Трапециевидный профиль
        double v_cruise = dv_abs - v_ramp_total;
        double t_cruise = v_cruise / p->a_max;
        p->t1_end = t_j_full;
        p->t2_end = t_j_full + t_cruise;
        p->t_total = t_j_full + t_cruise + t_j_full;
        p->has_cruise_phase = 1;
        p->a_peak_used = p->a_max;
    }
    else
    {
        // Треугольный профиль
        double a_peak = sqrt(dv_abs * p->j_max);
        double t_j = a_peak / p->j_max;
        p->t1_end = t_j;
        p->t2_end = 2.0 * t_j;
        p->t_total = 2.0 * t_j;
        p->has_cruise_phase = 0; // ← Всегда 0 для треугольного
        p->a_peak_used = a_peak;
    }

    p->state = JLP_RUNNING;

    // Отладочный лог — какой профиль выбран
    if (p->enable_diagnostics)
    {
        printf("[%s][DEBUG] dv_abs=%.3f, v_ramp_total=%.3f → выбор: %s\n",
               p->wheel_name, dv_abs, v_ramp_total,
               (dv_abs > v_ramp_total) ? "трапеция" : "треугольник");
        printf("[%s][DIAG] Старт профиля: v_target=%.3f, %s, %s\n",
               p->wheel_name, v_target_limited,
               p->direction > 0 ? "разгон" : "тормож",
               p->has_cruise_phase ? "трапец" : "треуг");
    }
}

// Запрос пересчёта
void jlp_request_replan(JerkLimitedProfile *p, double v_target_new)
{
    p->v_target_new = v_target_new;
    p->replan_requested = 1;
    p->t_coast_elapsed = 0.0;

    if (p->enable_diagnostics)
        printf("[%s][EVENT] Запрошен пересчёт на v=%.3f (v=%.3f, a=%.3f, j=%.3f, state=%d)\n",
               p->wheel_name, v_target_new, p->v_current, p->a_current, p->j_current, p->state);
}

// Один шаг управления
void jlp_step(JerkLimitedProfile *p, double dt)
{
    // Защита от некорректного dt
    if (dt <= 0.0)
    {
        if (p->enable_diagnostics)
        {
            printf("[%s][ERROR] dt=%.6f — недопустимо! Шаг должен быть > 0.\n", p->wheel_name, dt);
        }
        return;
    }

    // Обработка запроса пересчёта — ДО проверки IDLE
    if (p->replan_requested)
    {
        p->t_coast_j = fabs(p->j_current) / p->j_max;
        p->j_coast_sign = (p->j_current >= 0) ? -1.0 : 1.0;

        double a_after_j = p->a_current + p->j_current * p->t_coast_j +
                           0.5 * p->j_coast_sign * p->j_max * p->t_coast_j * p->t_coast_j;

        p->t_coast_a = fabs(a_after_j) / p->j_max;
        p->a_coast_sign = (a_after_j >= 0) ? -1.0 : 1.0;

        p->t_coast_elapsed = 0.0;
        p->state = JLP_COASTING_J;
        p->replan_requested = 0;

        if (p->enable_diagnostics)
            printf("[%s][REPLAN] Переход в COAST_J (t_coast_j=%.3f, t_coast_a=%.3f)\n",
                   p->wheel_name, p->t_coast_j, p->t_coast_a);
    }

    if (p->state == JLP_IDLE)
    {
        if (p->enable_diagnostics)
        {
            printf("[%s] IDLE: v=%.3f, a=%.3f, j=%.3f, t=%.3f, target=%.3f\n",
                   p->wheel_name, p->v_current, p->a_current, p->j_current, p->t_current, p->v_target);
        }
        return;
    }

    // Сброс рывка
    if (p->state == JLP_COASTING_J)
    {
        if (p->t_coast_elapsed >= p->t_coast_j || p->t_coast_j < 1e-6)
        {
            p->state = JLP_COASTING_A;
            p->t_coast_elapsed = 0.0;
            if (p->enable_diagnostics)
                printf("[%s][COAST] Переход в COAST_A\n", p->wheel_name);
        }
        else
        {
            p->j_current = p->j_coast_sign * p->j_max;
            p->a_current += p->j_current * dt;
            p->v_current += p->a_current * dt;
            p->t_coast_elapsed += dt;
        }
        p->t_current += dt;
    }
    // Сброс ускорения
    else if (p->state == JLP_COASTING_A)
    {
        if (p->t_coast_elapsed >= p->t_coast_a || p->t_coast_a < 1e-6)
        {
            p->a_current = 0.0;
            p->j_current = 0.0;
            jlp_start_profile(p, p->v_target_new);
            if (p->enable_diagnostics)
                printf("[%s][REPLAN] Новый профиль стартовал с v=%.3f\n", p->wheel_name, p->v_current);
            p->t_current += dt;
            return;
        }
        else
        {
            p->j_current = p->a_coast_sign * p->j_max;
            p->a_current += p->j_current * dt;
            p->v_current += p->a_current * dt;
            p->t_coast_elapsed += dt;
        }
        p->t_current += dt;
    }
    // Основной профиль
    else if (p->state == JLP_RUNNING)
    {
        // Завершение профиля
        if (p->t_current >= p->t_total || fabs(p->v_current - p->v_target) < 1e-6)
        {
            p->a_current = 0.0;
            p->j_current = 0.0;
            p->is_finished = 1;
            p->state = JLP_IDLE;
            if (p->enable_diagnostics)
                printf("[%s][DIAG] Профиль завершён\n", p->wheel_name);
            if (p->enable_diagnostics)
            {
                printf("[%s] IDLE: v=%.3f, a=%.3f, j=%.3f, t=%.3f, target=%.3f\n",
                       p->wheel_name, p->v_current, p->a_current, p->j_current, p->t_current, p->v_target);
            }
            return;
        }

        double j = 0.0;

        if (p->phase == 1)
        {
            j = p->direction * p->j_max;   // Применяем рывок: +j_max при разгоне, -j_max при торможении
            if (p->t_current >= p->t1_end) // Если достигли конца фазы 1
            {
                if (p->has_cruise_phase) // ← ИСПРАВЛЕНО: если есть фаза 2 (трапеция)
                {
                    p->phase = 2; // Переходим в фазу 2
                    if (p->enable_diagnostics)
                        printf("[%s][DIAG] Переход на фазу 2\n", p->wheel_name);
                }
                else
                {
                    p->phase = 3; // Для треугольника — сразу в фазу 3
                    if (p->enable_diagnostics)
                        printf("[%s][DIAG] Переход на фазу 3\n", p->wheel_name);
                }
            }
        }
        else if (p->phase == 2) // ← ИСПРАВЛЕНО: добавлена фаза 2
        {
            j = 0.0;                       // Рывок = 0 — ускорение постоянно
            if (p->t_current >= p->t2_end) // Если достигли конца фазы 2
            {
                p->phase = 3; // Переходим в фазу 3
                if (p->enable_diagnostics)
                    printf("[%s][DIAG] Переход на фазу 3\n", p->wheel_name);
            }
        }
        else if (p->phase == 3)
        {
            j = -p->direction * p->j_max; // Применяем противоположный рывок для сброса ускорения
        }
        double a_new = p->a_current + j * dt;
        double a_limit = p->direction * p->a_peak_used;
        if (p->direction > 0 && a_new > a_limit)
            a_new = a_limit;
        if (p->direction < 0 && a_new < a_limit)
            a_new = a_limit;

        // АНАЛИТИЧЕСКИЙ расчёт скорости — точный, без накопления ошибки
        double v_new;
        if (p->t_current <= p->t1_end)
        {
            // Фаза 1: разгон ускорения — v(t) = v_start + 0.5 * j * t² * direction
            v_new = p->v_start + p->direction * 0.5 * p->j_max * p->t_current * p->t_current;
        }
        else if (p->has_cruise_phase && p->t_current <= p->t2_end)
        {
            // Фаза 2: постоянное ускорение — ТОЛЬКО для трапециевидного профиля
            double t_cruise = p->t_current - p->t1_end;                                  // Время в фазе 2
            double v_mid = p->v_start + p->direction * 0.5 * p->a_peak_used * p->t1_end; // Скорость в конце фазы 1
            v_new = v_mid + p->direction * p->a_peak_used * t_cruise;                    // Линейный рост скорости
        }
        else
        {
            // Фаза 3: сброс ускорения
            double t_down;
            double v_mid;

            if (p->has_cruise_phase)
            {
                // Для трапеции — начало фазы 3 = конец фазы 2
                t_down = p->t_current - p->t2_end;
                v_mid = p->v_start + p->direction * 0.5 * p->a_peak_used * p->t1_end +
                        p->direction * p->a_peak_used * (p->t2_end - p->t1_end); // Скорость в конце фазы 2
            }
            else
            {
                // Для треугольника — начало фазы 3 = конец фазы 1
                t_down = p->t_current - p->t1_end;
                v_mid = p->v_start + p->direction * 0.5 * p->a_peak_used * p->t1_end; // Скорость в конце фазы 1
            }

            v_new = v_mid + p->direction * (p->a_peak_used * t_down - 0.5 * p->j_max * t_down * t_down);
        }

        // Плавное ограничение — без резкого обнуления ускорения
        if (p->direction > 0 && v_new > p->v_target)
        {
            v_new = p->v_target;
        }
        else if (p->direction < 0 && v_new < p->v_target)
        {
            v_new = p->v_target;
        }

        p->j_current = j;
        p->a_current = a_new;
        p->v_current = v_new;
        p->t_current += dt;
    }

    // Детальный лог на каждом шаге
    if (p->enable_diagnostics)
    {
        const char *phase_str[] = {"-", "Ф1", "-", "Ф3"};
        const char *state_str[] = {"IDLE", "RUN", "COAST_J", "COAST_A"};
        const char *dir_str = (p->direction > 0) ? "разгон" : "тормож";
        const char *profile_str = p->has_cruise_phase ? "трапец" : "треуг";

        printf("[%s] t= %.3f | v= %.3f | a= %.3f | j= %.3f | фаза=%s | сост=%s | цель= %.3f | dir=%s | проф=%s | coast_t= %.3f\n",
               p->wheel_name,
               p->t_current,
               p->v_current,
               p->a_current,
               p->j_current,
               phase_str[p->phase],
               state_str[p->state],
               p->v_target,
               dir_str,
               profile_str,
               p->t_coast_elapsed);
    }
}
/*
// Основная программа
int main()
{
    JerkLimitedProfile left_wheel;
    JerkLimitedProfile right_wheel;

    // Инициализация
    jlp_init(&left_wheel,  "left",  0.0, 2.0, 1.0, 0.5);
    jlp_init(&right_wheel, "right", 0.0, 2.0, 1.0, 0.5);

    left_wheel.enable_diagnostics = 1;
    right_wheel.enable_diagnostics = 1;

    double dt = 0.01;
    int steps = 0;

    double last_desired_speedL = 0.0;
    double last_desired_speedR = 0.0;
    const double EPSILON = 1e-3;

    typedef struct {
        double speedL;
        double speedR;
    } DesiredSpeeds;

    DesiredSpeeds g_desiredSpeed = {0.0, 0.0};

    printf("Лог включён. dt=%.3f с\n", dt);
    printf("================================================================================\n");

    while (steps < 110)  // До 1.1 секунды — чтобы увидеть завершение
    {
        if (steps == 10) {
            g_desiredSpeed.speedL = 0.5;
            g_desiredSpeed.speedR = 0.5;
        }

        if (fabs(g_desiredSpeed.speedL - last_desired_speedL) > EPSILON)
        {
            jlp_request_replan(&left_wheel, g_desiredSpeed.speedL);
            last_desired_speedL = g_desiredSpeed.speedL;
        }

        if (fabs(g_desiredSpeed.speedR - last_desired_speedR) > EPSILON)
        {
            jlp_request_replan(&right_wheel, g_desiredSpeed.speedR);
            last_desired_speedR = g_desiredSpeed.speedR;
        }

        jlp_step(&left_wheel,  dt);
        jlp_step(&right_wheel, dt);

        usleep((unsigned int)(dt * 1000000));
        steps++;
    }

    printf("\n>>> СИМУЛЯЦИЯ ЗАВЕРШЕНА <<<\n");
    return 0;
}
*/
#endif
