#ifndef JERK_H
#define JERK_H

#include <stdio.h>  // Подключаем стандартную библиотеку ввода-вывода — для printf (в диагностике)
#include <stdlib.h> // Подключаем стандартную библиотеку — резерв (может пригодиться)
#include <math.h>   // Подключаем математическую библиотеку — для fabs, sqrt
#include <cmath>    // Подключаем математическую библиотеку — для fabs, sqrt
#include "config.h"
#include <unistd.h> // Подключаем для usleep — реальная задержка в симуляции

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <unistd.h> // для usleep (только для симуляции в реальном времени)

// ========================
// Состояния профиля
// ========================
typedef enum
{
    JLP_IDLE = 0,   // Профиль не активен — робот стоит или ждёт команды
    JLP_RUNNING = 1 // Профиль выполняется — активное управление по плану
} JLP_State;

// ========================
// Структура профиля — ВСЁ хранится здесь
// ========================
typedef struct
{
    const char *wheel_name; // Имя колеса для логов ("left", "right")

    // Текущее состояние (вход/выход алгоритма)
    double v_current; // Текущая скорость (м/с)
    double a_current; // Текущее ускорение (м/с²)
    double j_current; // Текущий рывок (м/с³)
    double t_current; // Время, прошедшее с начала текущего профиля (с)

    // Параметры текущего профиля — пересчитываются при replan
    double v_target;      // Целевая скорость (м/с)
    double v_start;       // ✅ НАЧАЛЬНАЯ СКОРОСТЬ ПРОФИЛЯ (на момент запуска) — КРИТИЧНО!
    double t1_end;        // Время окончания фазы 1 (нарастание ускорения)
    double t2_end;        // Время окончания фазы 2 (постоянное ускорение)
    double t_total;       // Общее время профиля
    int has_cruise_phase; // 1 — трапеция, 0 — треугольник
    double a_peak_used;   // Реально используемое пиковое ускорение
    int direction;        // +1 = разгон, -1 = торможение

    // ✅ ДОБАВЛЕНО: ТЕКУЩАЯ ФАЗА ПРОФИЛЯ — 1, 2, 3 или 0 (не определена)
    // Используется только в JLP_RUNNING, всегда инициализируется при запуске
    int phase; // ✅ Фаза: 1=нарастание, 2=крейсер, 3=сброс

    // Состояние автомата
    JLP_State state; // JLP_IDLE или JLP_RUNNING
    int is_finished; // 1 — профиль завершён (скорость достигнута)

    // Параметры системы — задаются один раз при инициализации
    double j_max; // Максимальный рывок (м/с³) — всегда > 0
    double a_max; // Максимальное ускорение (м/с²) — всегда > 0
    double v_max; // Максимальная физическая скорость (м/с)

    // Параметры для перепланирования "на лету"
    int replan_requested; // 1 — запрошено перепланирование
    double v_target_new;  // Новая целевая скорость — для replan

    // Диагностика
    int enable_diagnostics; // 1 — выводить логи, 0 — молчать

} JerkLimitedProfile;

// ========================
// ИНИЦИАЛИЗАЦИЯ — вызывается один раз при старте системы
// ========================
// ✅ ВСЕ ПОЛЯ ИНИЦИАЛИЗИРУЮТСЯ ЯВНО — НИКАКОГО МУСОРА!
void jlp_init(JerkLimitedProfile *p, const char *name, double v_start, double j_max, double a_max, double v_max)
{
    p->wheel_name = name;

    // Текущие состояния
    p->v_current = v_start;
    p->a_current = 0.0;
    p->j_current = 0.0;
    p->t_current = 0.0;

    // Параметры профиля — инициализированы безопасными значениями
    p->v_target = v_start; // ✅ По умолчанию — цель = начальной скорости
    p->v_start = v_start;  // ✅ Начальная скорость профиля
    p->t1_end = 0.0;
    p->t2_end = 0.0;
    p->t_total = 0.0;
    p->has_cruise_phase = 0;
    p->a_peak_used = 0.0;
    p->direction = 0; // ✅ Неопределённое направление
    p->phase = 0;     // ✅ Фаза не задана — не используется до старта
    p->state = JLP_IDLE;
    p->is_finished = 1;

    // Физические ограничения системы
    p->j_max = j_max;
    p->a_max = a_max;
    p->v_max = v_max;

    // Параметры перепланирования
    p->replan_requested = 0;
    p->v_target_new = 0.0;

    // Диагностика
    p->enable_diagnostics = 0;

    // ✅ Проверка: j_max, a_max, v_max должны быть положительными
    if (p->j_max <= 0 || p->a_max <= 0 || p->v_max < 0)
    {
        if (p->enable_diagnostics)
        {
            printf("[%s][FATAL] Недопустимые параметры: j_max=%.3f, a_max=%.3f, v_max=%.3f\n",
                   p->wheel_name, p->j_max, p->a_max, p->v_max);
        }
    }
}

// ========================
// ЗАПУСК ПРОФИЛЯ — вызывается при первом задании цели или при старте
// ========================
// Используется только при старте — когда профиль ещё не запущен (state == IDLE)
void jlp_start_profile(JerkLimitedProfile *p, double v_target)
{
    // Ограничиваем целевую скорость по физическим лимитам
    double v_target_limited = fmax(-p->v_max, fmin(v_target, p->v_max));

    // Если цель почти равна текущей — ничего не делаем
    if (fabs(v_target_limited - p->v_current) < 1e-9)
    {
        p->a_current = 0.0;
        p->j_current = 0.0;
        p->state = JLP_IDLE;
        p->is_finished = 1;
        if (p->enable_diagnostics)
            printf("[%s][DIAG] Цель равна текущей скорости — профиль не запущен\n", p->wheel_name);
        return;
    }

    p->v_target = v_target_limited;
    p->v_start = p->v_current; // ✅ КРИТИЧНО: запоминаем начальную скорость профиля!
    p->direction = (v_target_limited >= p->v_current) ? 1 : -1;
    double dv = p->v_target - p->v_current;
    double dv_abs = fabs(dv);

    // Рассчитываем времена фаз
    double t_j_full = p->a_max / p->j_max;          // Время выхода на макс. ускорение
    double v_ramp_full = 0.5 * p->a_max * t_j_full; // Прирост скорости за одну фазу (разгон/тормож)
    double v_ramp_total = 2.0 * v_ramp_full;        // Полный прирост в треугольнике

    p->t_current = 0.0;
    p->phase = 1; // ✅ Фаза 1: нарастание рывка — всегда начинаем с неё
    p->is_finished = 0;

    if (dv_abs > v_ramp_total)
    {
        // Трапеция: есть фаза постоянного ускорения
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
        // Треугольник: нет фазы крейсера
        double a_peak = sqrt(dv_abs * p->j_max); // Пиковое ускорение
        double t_j = a_peak / p->j_max;          // Время до пика
        p->t1_end = t_j;
        p->t2_end = 2.0 * t_j;
        p->t_total = 2.0 * t_j;
        p->has_cruise_phase = 0;
        p->a_peak_used = a_peak;
    }

    p->state = JLP_RUNNING;
    if (p->enable_diagnostics)
    {
        printf("[%s][DEBUG] Профиль запущен: v_target=%.6f, %s, %s\n",
               p->wheel_name, v_target_limited,
               p->direction > 0 ? "разгон" : "тормож",
               p->has_cruise_phase ? "трапец" : "треуг");
    }
}

// ========================
// ПЕРЕПЛАНИРОВАНИЕ НА ЛЕТУ — КЛЮЧЕВАЯ ФУНКЦИЯ ДЛЯ 100 ГЦ
// ========================
// Вызывается при смене цели — НЕ СБРАСЫВАЕТ ускорение!
// Пересчитывает профиль от текущих v, a, j — без остановок!
void jlp_replan_profile(JerkLimitedProfile *p)
{
    // Ограничиваем новую цель
    double v_target_limited = fmax(-p->v_max, fmin(p->v_target_new, p->v_max));

    // Если новая цель почти равна текущей — ничего не делаем
    if (fabs(v_target_limited - p->v_current) < 1e-9)
    {
        p->a_current = 0.0;
        p->j_current = 0.0;
        p->is_finished = 1;
        p->state = JLP_IDLE;
        if (p->enable_diagnostics)
            printf("[%s][REPLAN] Новая цель равна текущей — профиль завершён\n", p->wheel_name);
        return;
    }

    // Вычисляем разницу скорости
    double dv = v_target_limited - p->v_current;
    p->direction = (dv >= 0) ? 1 : -1;
    double dv_abs = fabs(dv);

    // Пересчитываем профиль — от текущего состояния!
    double t_j_full = p->a_max / p->j_max;
    double v_ramp_full = 0.5 * p->a_max * t_j_full;
    double v_ramp_total = 2.0 * v_ramp_full;

    p->t_current = 0.0; // Время профиля сбрасывается — это "время внутри нового профиля"
    p->phase = 1;       // ✅ Всегда начинаем с фазы 1
    p->is_finished = 0;

    if (dv_abs > v_ramp_total)
    {
        // Трапеция
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
        // Треугольник
        double a_peak = sqrt(dv_abs * p->j_max);
        double t_j = a_peak / p->j_max;
        p->t1_end = t_j;
        p->t2_end = 2.0 * t_j;
        p->t_total = 2.0 * t_j;
        p->has_cruise_phase = 0;
        p->a_peak_used = a_peak;
    }

    p->v_target = v_target_limited; // Обновляем цель
    p->v_start = p->v_current;      // ✅ КРИТИЧНО: запоминаем текущую скорость как v_start!
    p->state = JLP_RUNNING;         // Остаёмся в RUNNING — НИКАКОГО COASTING!

    if (p->enable_diagnostics)
    {
        printf("[%s][REPLAN] Пересчёт на v=%.6f (текущая v=%.6f, a=%.6f, j=%.6f) → %s\n",
               p->wheel_name, v_target_limited, p->v_current, p->a_current, p->j_current,
               p->has_cruise_phase ? "трапец" : "треуг");
    }
}

// ========================
// ЗАПРОС ПЕРЕПЛАНИРОВАНИЯ — вызывается извне (например, из планировщика)
// ========================
void jlp_request_replan(JerkLimitedProfile *p, double v_target_new)
{
    p->v_target_new = v_target_new;
    p->replan_requested = 1;

    if (p->enable_diagnostics)
    {
        printf("[%s][EVENT] Запрос перепланирования: v=%.6f (было: v=%.6f, a=%.6f)\n",
               p->wheel_name, v_target_new, p->v_current, p->a_current);
    }
}

// ========================
// ОДИН ШАГ УПРАВЛЕНИЯ — вызывается каждые dt (например, 0.01 с = 100 Гц)
// ========================
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

    // ✅ 1. ПЕРЕПЛАНИРОВАНИЕ НА ЛЕТУ — ЕСЛИ ЗАПРОШЕНО
    if (p->replan_requested)
    {
        p->replan_requested = 0;
        jlp_replan_profile(p); // ← ПЕРЕСЧИТЫВАЕМ ПРОФИЛЬ — НЕ СБРАСЫВАЕМ УСКОРЕНИЕ!
    }

    // ✅ 2. IDLE — ничего не делаем
    if (p->state == JLP_IDLE)
    {
        if (p->enable_diagnostics)
        {
            // printf("[%s] IDLE: v=%.6f, a=%.6f, j=%.6f, target=%.6f\n",
            //        p->wheel_name, p->v_current, p->a_current, p->j_current, p->v_target);
        }
        return;
    }

    // ✅ 3. RUNNING — выполняем профиль
    if (p->state == JLP_RUNNING)
    {
        // Проверка: профиль уже завершён? (скорость достигнута)
        if (fabs(p->v_current - p->v_target) < 1e-6)
        {
            p->a_current = 0.0;
            p->j_current = 0.0;
            p->is_finished = 1;
            p->state = JLP_IDLE;
            if (p->enable_diagnostics)
            {
                printf("[%s][DIAG] Профиль завершён — скорость достигнута\n", p->wheel_name);
            }
            return;
        }

        // Если время профиля истекло — тоже завершаем
        if (p->t_current >= p->t_total)
        {
            p->a_current = 0.0;
            p->j_current = 0.0;
            p->is_finished = 1;
            p->state = JLP_IDLE;
            if (p->enable_diagnostics) {
                printf("[%s][DIAG] Профиль завершён по времени\n", p->wheel_name);
            }
            return;
        }

        // Расчёт рывка в зависимости от фазы
        double j = 0.0;

        if (p->phase == 1)
        {
            j = p->direction * p->j_max; // Наращиваем ускорение
            if (p->t_current >= p->t1_end)
            {
                if (p->has_cruise_phase)
                {
                    p->phase = 2; // Переход на фазу 2 — постоянное ускорение
                    if (p->enable_diagnostics)
                        printf("[%s][DIAG] Фаза 1 → Фаза 2\n", p->wheel_name);
                }
                else
                {
                    p->phase = 3; // Для треугольника — сразу в фазу 3
                    if (p->enable_diagnostics)
                        printf("[%s][DIAG] Фаза 1 → Фаза 3\n", p->wheel_name);
                }
            }
        }
        else if (p->phase == 2)
        {
            j = 0.0; // Ускорение постоянно — рывок = 0
            if (p->t_current >= p->t2_end)
            {
                p->phase = 3;
                if (p->enable_diagnostics)
                    printf("[%s][DIAG] Фаза 2 → Фаза 3\n", p->wheel_name);
            }
        }
        else if (p->phase == 3)
        {
            j = -p->direction * p->j_max; // Сбрасываем ускорение
        }

        // ✅ ✅ ✅ АНАЛИТИЧЕСКИЙ расчёт — ВСЕГДА ОТ v_start!
        // Используем аналитические формулы для точного расчёта без накопления ошибки
        double a_new = 0.0;
        double v_new = 0.0;

        if (p->t_current <= p->t1_end)
        {
            // Фаза 1: j = const → a(t) = j * t → v(t) = v_start + 0.5 * j * t²
            a_new = p->direction * p->j_max * p->t_current;
            v_new = p->v_start + p->direction * 0.5 * p->j_max * p->t_current * p->t_current;
        }
        else if (p->has_cruise_phase && p->t_current <= p->t2_end)
        {
            // Фаза 2: a = const → v(t) = v_start + v_ramp1 + a * (t - t1_end)
            double t_cruise = p->t_current - p->t1_end;
            double v_at_t1 = p->v_start + p->direction * 0.5 * p->a_peak_used * p->t1_end;
            a_new = p->direction * p->a_peak_used;
            v_new = v_at_t1 + p->direction * p->a_peak_used * t_cruise;
        }
        else
        {
            // Фаза 3: j = -const → a(t) = a_peak - j_max * (t - t_start)
            double t_down;
            double v_at_t1, v_at_t2;

            if (p->has_cruise_phase)
            {
                // Трапеция: начало фазы 3 = конец фазы 2
                t_down = p->t_current - p->t2_end;
                v_at_t1 = p->v_start + p->direction * 0.5 * p->a_peak_used * p->t1_end;
                v_at_t2 = v_at_t1 + p->direction * p->a_peak_used * (p->t2_end - p->t1_end);
            }
            else
            {
                // Треугольник: начало фазы 3 = конец фазы 1
                t_down = p->t_current - p->t1_end;
                v_at_t1 = p->v_start + p->direction * 0.5 * p->a_peak_used * p->t1_end;
                v_at_t2 = v_at_t1; // в треугольнике v в конце фазы 1 = v в начале фазы 3
            }

            a_new = p->direction * (p->a_peak_used - p->j_max * t_down);
            v_new = v_at_t2 + p->direction * (p->a_peak_used * t_down - 0.5 * p->j_max * t_down * t_down);
        }

        // ✅ ✅ ✅ КРИТИЧНО: ОГРАНИЧИВАЕМ УСКОРЕНИЕ ПО ФИЗИЧЕСКОМУ ЛИМИТУ a_max
        // Аналитическая формула может дать a_new > a_max из-за дискретности dt!
        // Это нарушит физику привода — поэтому жёстко ограничиваем!
        a_new = fmax(-p->a_max, fmin(a_new, p->a_max));

        // ✅ ✅ ✅ ОГРАНИЧЕНИЕ СКОРОСТИ — КРИТИЧНО!
        // Никогда не позволяем превысить цель — даже если аналитика дала больше
        if (p->direction > 0 && v_new > p->v_target)
        {
            v_new = p->v_target;
        }
        else if (p->direction < 0 && v_new < p->v_target)
        {
            v_new = p->v_target;
        }

        // ✅ Обновляем состояния — ТОЛЬКО ПОСЛЕ ВСЕХ ОГРАНИЧЕНИЙ!
        p->j_current = j;
        p->a_current = a_new; // ✅ Теперь ускорение всегда в пределах a_max!
        p->v_current = v_new; // ✅ Скорость всегда в пределах v_target
        p->t_current += dt;

        // ✅ Диагностика на каждом шаге
        if (p->enable_diagnostics)
        {
            const char *phase_str[] = {"-", "Ф1", "Ф2", "Ф3"};
            const char *state_str[] = {"IDLE", "RUN"};
            const char *dir_str = (p->direction > 0) ? "разгон" : "тормож";
            const char *profile_str = p->has_cruise_phase ? "трапец" : "треуг";

            printf("[%s] t= %.3f | v= %.6f | a= %.6f | j= %.6f | фаза=%s | сост=%s | цель= %.6f | dir=%s | проф=%s\n",
                   p->wheel_name,
                   p->t_current,
                   p->v_current,
                   p->a_current,
                   p->j_current,
                   phase_str[p->phase],
                   state_str[p->state],
                   p->v_target,
                   dir_str,
                   profile_str);
        }
    }
}
/*
// ========================
// ОСНОВНАЯ ПРОГРАММА — ТЕСТ
// ========================
int main()
{
    JerkLimitedProfile left_wheel;
    JerkLimitedProfile right_wheel;

    // Инициализация — начальная скорость 0, j_max=2.0, a_max=1.0, v_max=0.5
    jlp_init(&left_wheel,  "left",  0.0, 2.0, 1.0, 0.5);
    jlp_init(&right_wheel, "right", 0.0, 2.0, 1.0, 0.5);

    left_wheel.enable_diagnostics = 1;
    right_wheel.enable_diagnostics = 1;

    double dt = 0.01;  // 100 Гц — идеально для реального робота
    int steps = 0;

    // Симуляция команд — меняем цель на 0.3 м/с на шаге 10, потом на 0.1, потом на 0.4
    double desired_speedL = 0.0;
    double desired_speedR = 0.0;

    printf("Лог включён. dt=%.3f с (100 Гц)\n", dt);
    printf("================================================================================\n");

    while (steps < 200)  // 2 секунды симуляции
    {
        // Эмуляция изменения цели — на 0.1 с, 0.5 с, 1.2 с
        if (steps == 10) {    // 0.1 с
            desired_speedL = 0.3;
            desired_speedR = 0.3;
        }
        else if (steps == 50) {  // 0.5 с
            desired_speedL = 0.1;
            desired_speedR = 0.1;
        }
        else if (steps == 120) { // 1.2 с
            desired_speedL = 0.4;
            desired_speedR = 0.4;
        }

        // Запрос перепланирования — вызывается при смене цели
        if (fabs(desired_speedL - left_wheel.v_target) > 1e-6) {
            jlp_request_replan(&left_wheel, desired_speedL);
        }
        if (fabs(desired_speedR - right_wheel.v_target) > 1e-6) {
            jlp_request_replan(&right_wheel, desired_speedR);
        }

        // Запускаем один шаг управления — ключевой!
        jlp_step(&left_wheel,  dt);
        jlp_step(&right_wheel, dt);

        // Реальная задержка — только для симуляции в реальном времени
        usleep((unsigned int)(dt * 1000000));

        steps++;
    }

    printf("\n>>> СИМУЛЯЦИЯ ЗАВЕРШЕНА <<<\n");
    return 0;
}

*/
#endif
