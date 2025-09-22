#ifndef JERK_H
#define JERK_H

#include <stdio.h>  // Подключаем стандартную библиотеку ввода-вывода — для printf (в диагностике)
#include <stdlib.h> // Подключаем стандартную библиотеку — резерв (может пригодиться)
#include <math.h>   // Подключаем математическую библиотеку — для fabs, sqrt
#include <cmath>    // Подключаем математическую библиотеку — для fabs, sqrt
#include "config.h"
#include <unistd.h> // Подключаем для usleep — реальная задержка в симуляции

// Определяем состояния профиля — УПРОЩЕНАЯ ВЕРСИЯ
typedef enum
{
    JLP_IDLE = 0,    // Профиль не активен — робот стоит
    JLP_RUNNING = 1, // Профиль выполняется — разгон/торможение по плану
    JLP_COASTING = 2 // ЕДИНСТВЕННОЕ состояние сброса — плавно сбрасываем ускорение до 0 (без разделения на J и A)
} JLP_State;

// Структура состояния профиля — ВСЁ хранится здесь
typedef struct
{
    const char *wheel_name; // Указатель на строку с именем колеса — для логов (например, "left", "right")

    // Текущее состояние движения — обновляется на каждом шаге
    double v_current; // Текущая скорость (м/с) — вход и выход алгоритма
    double a_current; // Текущее ускорение (м/с²) — вход и выход алгоритма
    double j_current; // Текущий рывок (м/с³) — вход и выход алгоритма
    double t_current; // Текущее время профиля (с) — увеличивается на dt на каждом шаге
    int phase;        // Текущая фаза профиля: 1=нарастание, 2=крейсер, 3=сброс

    // Параметры текущего активного профиля — рассчитываются при старте или пересчёте
    double v_target;      // Целевая скорость (м/с) — к чему стремимся в текущем профиле
    double v_start;       // Начальная скорость профиля — сохраняется при вызове jlp_start_profile (для аналитического расчёта)
    double t1_end;        // Время окончания фазы 1 — когда заканчивается нарастание ускорения
    double t2_end;        // Время окончания фазы 2 — когда заканчивается постоянное ускорение (если есть)
    double t_total;       // Общее время профиля — сколько длится весь профиль от начала до конца
    int has_cruise_phase; // 1 — есть фаза постоянного ускорения (трапеция), 0 — нет (треугольник)
    double a_peak_used;   // Пиковое ускорение (по модулю) — реальное значение, которое будет использовано
    int direction;        // Направление движения: +1 = разгон, -1 = торможение

    // Состояние конечного автомата — управляет логикой выполнения
    JLP_State state; // Текущее состояние автомата: IDLE, RUNNING, COASTING
    int is_finished; // Флаг завершения: 1 — профиль завершён, 0 — ещё выполняется

    // Параметры системы — задаются один раз при инициализации
    double j_max; // Максимальный рывок (м/с³) — ограничение на скорость изменения ускорения (всегда положительный)
    double a_max; // Максимальное ускорение (м/с²) — ограничение на ускорение/торможение (всегда положительный)
    double v_max; // Максимальная физическая скорость колеса (м/с) — ограничение на целевую скорость

    // Параметры для пересчёта профиля "на лету"
    int replan_requested; // Флаг: 1 — запрошен пересчёт профиля, 0 — нет
    double v_target_new;  // Новая целевая скорость — на которую нужно пересчитать профиль

    // Для состояния COASTING — счётчик времени (опционально, для защиты от зависания)
    double t_coast_elapsed; // Сколько времени уже прошло в состоянии сброса

    // Флаг для включения/выключения диагностики
    int enable_diagnostics; // 1 — выводить диагностические сообщения, 0 — не выводить
} JerkLimitedProfile;

// Инициализация структуры профиля — вызывается один раз при старте системы
void jlp_init(JerkLimitedProfile *p, const char *name, double v_start, double j_max, double a_max, double v_max)
{
    p->wheel_name = name;      // Сохраняем имя колеса для логов
    p->v_current = v_start;    // Устанавливаем начальную скорость из параметра
    p->a_current = 0.0;        // Начальное ускорение — 0
    p->j_current = 0.0;        // Начальный рывок — 0
    p->t_current = 0.0;        // Текущее время профиля — 0
    p->phase = 1;              // Начинаем всегда с фазы 1 — нарастание рывка
    p->v_target = v_start;     // По умолчанию — целевая скорость = начальной
    p->state = JLP_IDLE;       // Начальное состояние — бездействие
    p->is_finished = 1;        // Профиль считается завершённым
    p->j_max = j_max;          // Сохраняем макс. рывок
    p->a_max = a_max;          // Сохраняем макс. ускорение
    p->v_max = v_max;          // Сохраняем макс. скорость колеса
    p->replan_requested = 0;   // Сбрасываем флаг пересчёта
    p->t_coast_elapsed = 0.0;  // Обнуляем счётчик времени сброса
    p->enable_diagnostics = 0; // По умолчанию диагностика отключена
}

// Запуск нового профиля — вызывается при старте или после пересчёта
void jlp_start_profile(JerkLimitedProfile *p, double v_target)
{
    // Ограничиваем целевую скорость физическим лимитом колеса
    double v_target_limited = v_target; // Копируем целевую скорость
    if (p->v_max > 0)
    {                                                                 // Если лимит скорости задан
        v_target_limited = fmax(-p->v_max, fmin(v_target, p->v_max)); // Ограничиваем в диапазоне [-v_max, v_max]
    }

    p->v_target = v_target_limited; // Устанавливаем новую целевую скорость (ограниченную)
    p->v_start = p->v_current;      // Сохраняем начальную скорость для аналитического расчёта
    p->t_current = 0.0;             // Сбрасываем таймер профиля
    p->phase = 1;                   // Всегда начинаем с фазы 1
    p->is_finished = 0;             // Сбрасываем флаг завершения

    double dv = v_target_limited - p->v_current; // Вычисляем разницу между целевой и текущей скоростью
    p->direction = (dv >= 0.0) ? 1 : -1;         // Определяем направление: +1 = разгон, -1 = торможение
    double dv_abs = fabs(dv);                    // Берём модуль — для одинаковых расчётов в обоих направлениях

    if (dv_abs < 1e-9) // Если разница скоростей почти нулевая
    {
        p->t1_end = p->t2_end = p->t_total = 0.0;                              // Обнуляем все времена
        p->has_cruise_phase = 0;                                               // Фаза постоянного ускорения отсутствует
        p->a_peak_used = 0.0;                                                  // Пиковое ускорение — 0
        p->state = JLP_IDLE;                                                   // Переключаемся в состояние бездействия
        p->is_finished = 1;                                                    // Помечаем профиль как завершённый
        if (p->enable_diagnostics)                                             // Если включена диагностика
            printf("[%s][DIAG] Профиль не требуется — dv=0\n", p->wheel_name); // Выводим сообщение
        return;                                                                // Выходим — расчёты не нужны
    }

    double t_j_full = p->a_max / p->j_max;          // Время выхода на макс. ускорение с макс. рывком
    double v_ramp_full = 0.5 * p->a_max * t_j_full; // Прирост скорости за одну фазу нарастания/сброса
    double v_ramp_total = 2.0 * v_ramp_full;        // Суммарный прирост на фазах 1+3

    // ИСПРАВЛЕНО: было >=, теперь > — при равенстве выбираем треугольный профиль
    if (dv_abs > v_ramp_total) // Если нужно БОЛЬШЕ скорости, чем дают фазы 1+3 — нужна фаза 2
    {
        double v_cruise = dv_abs - v_ramp_total;     // Остаток скорости для фазы постоянного ускорения
        double t_cruise = v_cruise / p->a_max;       // Время фазы 2 — сколько держать макс. ускорение
        p->t1_end = t_j_full;                        // Записываем время окончания фазы 1
        p->t2_end = t_j_full + t_cruise;             // Записываем время окончания фазы 2
        p->t_total = t_j_full + t_cruise + t_j_full; // Общее время профиля = фаза1 + фаза2 + фаза3
        p->has_cruise_phase = 1;                     // Указываем, что фаза 2 есть
        p->a_peak_used = p->a_max;                   // Используем макс. ускорение
    }
    else // Если нужно МЕНЬШЕ или РАВНО — треугольный профиль
    {
        double a_peak = sqrt(dv_abs * p->j_max); // Рассчитываем пиковое ускорение для треугольного профиля
        double t_j = a_peak / p->j_max;          // Время выхода на это пиковое ускорение
        p->t1_end = t_j;                         // Время окончания фазы 1
        p->t2_end = 2.0 * t_j;                   // Время окончания фазы 2 = концу профиля
        p->t_total = 2.0 * t_j;                  // Общее время — две симметричные фазы
        p->has_cruise_phase = 0;                 // Фаза 2 отсутствует — треугольный профиль
        p->a_peak_used = a_peak;                 // Записываем реальное пиковое ускорение
    }

    p->state = JLP_RUNNING; // Переключаемся в состояние выполнения профиля

    // Отладочный лог — какой профиль выбран
    if (p->enable_diagnostics) // Если включена диагностика
    {
        printf("[%s][DEBUG] dv_abs=%.3f, v_ramp_total=%.3f → выбор: %s\n", // Выводим параметры выбора
               p->wheel_name, dv_abs, v_ramp_total,
               (dv_abs > v_ramp_total) ? "трапеция" : "треугольник");
        printf("[%s][DIAG] Старт профиля: v_target=%.3f, %s, %s\n", // Выводим информацию о профиле
               p->wheel_name, v_target_limited, p->direction > 0 ? "разгон" : "тормож",
               p->has_cruise_phase ? "трапец" : "треуг");
    }
}

// Запрос пересчёта профиля "на лету" — можно вызывать В ЛЮБОМ СОСТОЯНИИ
void jlp_request_replan(JerkLimitedProfile *p, double v_target_new)
{
    p->v_target_new = v_target_new; // Сохраняем новую целевую скорость
    p->replan_requested = 1;        // Устанавливаем флаг запроса пересчёта
    p->t_coast_elapsed = 0.0;       // Обнуляем счётчик времени сброса

    if (p->enable_diagnostics)                                                                                    // Если включена диагностика
        printf("[%s][EVENT] Запрошен пересчёт на v=%.3f (текущее состояние: v=%.3f, a=%.3f, j=%.3f, state=%d)\n", // Выводим новую цель и текущее состояние
               p->wheel_name, v_target_new, p->v_current, p->a_current, p->j_current, p->state);
}

// Один шаг управления — вызывать строго каждые dt секунд
void jlp_step(JerkLimitedProfile *p, double dt)
{
    // Защита от некорректного dt — если dt <= 0 — ничего не делаем
    if (dt <= 0.0)
    { // Если шаг времени некорректный
        if (p->enable_diagnostics)
        {                                                                                           // Если включена диагностика
            printf("[%s][ERROR] dt=%.6f — недопустимо! Шаг должен быть > 0.\n", p->wheel_name, dt); // Выводим ошибку
        }
        return; // Выходим — не выполняем шаг
    }

    // Обработка запроса на пересчёт — ДО проверки IDLE (чтобы работало даже из IDLE)
    if (p->replan_requested) // Если есть запрос на пересчёт
    {
        p->state = JLP_COASTING;  // ← УПРОЩЕНО: сразу переходим в состояние сброса ускорения (без COASTING_J/COASTING_A)
        p->replan_requested = 0;  // Сбрасываем флаг запроса
        p->t_coast_elapsed = 0.0; // Обнуляем счётчик времени

        if (p->enable_diagnostics)                                                             // Если включена диагностика
            printf("[%s][REPLAN] Переход в COASTING (сброс ускорения до 0)\n", p->wheel_name); // Выводим сообщение
    }

    // Если профиль в состоянии IDLE — ничего не делаем (но логируем, если включено)
    if (p->state == JLP_IDLE)
    { // Если состояние IDLE
        if (p->enable_diagnostics)
        {                                                                      // Если включена диагностика
            printf("[%s] IDLE: v=%.3f, a=%.3f, j=%.3f, t=%.3f, target=%.3f\n", // Выводим состояние
                   p->wheel_name, p->v_current, p->a_current, p->j_current, p->t_current, p->v_target);
        }
        return; // Выходим — расчёты не нужны
    }

    // Обработка состояния: плавный сброс ускорения до 0 (единое состояние COASTING)
    if (p->state == JLP_COASTING) // Если в состоянии сброса ускорения
    {
        double j_coast = 0.0; // Инициализируем рывок для сброса

        // Определяем направление рывка для сброса ускорения до 0
        if (p->a_current > 1e-6)
        {                        // Если ускорение положительное
            j_coast = -p->j_max; // Применяем отрицательный рывок
        }
        else if (p->a_current < -1e-6)
        {                       // Если ускорение отрицательное
            j_coast = p->j_max; // Применяем положительный рывок
        }
        // Если ускорение ~0 — j_coast остаётся 0

        p->j_current = j_coast;            // Устанавливаем рывок
        p->v_current += p->a_current * dt; // Обновляем скорость: v = v_old + a_old * dt (можно заменить на аналитический)
        p->a_current += p->j_current * dt; // Обновляем ускорение: a = a_old + j * dt
        p->t_coast_elapsed += dt;          // Увеличиваем счётчик времени сброса

        // Проверка: если ускорение близко к 0 ИЛИ прошло слишком много времени (защита от зависания)
        if (fabs(p->a_current) < 1e-3 || p->t_coast_elapsed > 10.0)
        {
            p->a_current = 0.0;                                                                         // Явно обнуляем ускорение
            p->j_current = 0.0;                                                                         // Явно обнуляем рывок
            jlp_start_profile(p, p->v_target_new);                                                      // Запускаем новый профиль с новой целевой скоростью
            if (p->enable_diagnostics)                                                                  // Если включена диагностика
                printf("[%s][REPLAN] Новый профиль стартовал с v=%.3f\n", p->wheel_name, p->v_current); // Выводим стартовую скорость
            p->t_current += dt;                                                                         // Увеличиваем время на этом шаге
            return;                                                                                     // Выходим — не выполняем дальше
        }

        p->t_current += dt; // Увеличиваем общее время профиля
    }
    // Обработка состояния: выполнение основного профиля
    else if (p->state == JLP_RUNNING) // Если в состоянии выполнения профиля
    {
        // Проверка: если профиль завершён по времени или достигнута цель
        if (p->t_current >= p->t_total || fabs(p->v_current - p->v_target) < 1e-6) // Если время вышло или скорость достигла цели
        {
            p->a_current = 0.0;  // Обнуляем ускорение
            p->j_current = 0.0;  // Обнуляем рывок
            p->is_finished = 1;  // Помечаем профиль как завершённый
            p->state = JLP_IDLE; // Переключаемся в состояние бездействия
            if (p->enable_diagnostics)
                printf("[%s][DIAG] Профиль завершён\n", p->wheel_name); // Выводим диагностику
            if (p->enable_diagnostics)
            {                                                                      // Если включена диагностика
                printf("[%s] IDLE: v=%.3f, a=%.3f, j=%.3f, t=%.3f, target=%.3f\n", // Выводим финальное состояние
                       p->wheel_name, p->v_current, p->a_current, p->j_current, p->t_current, p->v_target);
            }
            return; // Выходим — на этом шаге больше ничего не делаем
        }

        double j = 0.0; // Инициализируем рывок нулём

        if (p->phase == 1) // Фаза 1: нарастание рывка
        {
            j = p->direction * p->j_max;   // Применяем рывок: +j_max при разгоне, -j_max при торможении
            if (p->t_current >= p->t1_end) // Если достигли конца фазы 1
            {
                if (p->has_cruise_phase) // Если есть фаза 2 (трапеция)
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
        else if (p->phase == 2) // Фаза 2: постоянное ускорение (для трапеции)
        {
            j = 0.0;                       // Рывок = 0 — ускорение постоянно
            if (p->t_current >= p->t2_end) // Если достигли конца фазы 2
            {
                p->phase = 3; // Переходим в фазу 3
                if (p->enable_diagnostics)
                    printf("[%s][DIAG] Переход на фазу 3\n", p->wheel_name);
            }
        }
        else if (p->phase == 3) // Фаза 3: сброс ускорения
        {
            j = -p->direction * p->j_max; // Применяем противоположный рывок для сброса ускорения
        }

        double a_new = p->a_current + j * dt;           // Вычисляем новое ускорение: a = a_old + j * dt
        double a_limit = p->direction * p->a_peak_used; // Рассчитываем лимит ускорения с учётом направления
        if (p->direction > 0 && a_new > a_limit)
            a_new = a_limit; // Ограничиваем сверху при разгоне
        if (p->direction < 0 && a_new < a_limit)
            a_new = a_limit; // Ограничиваем снизу при торможении

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

        // Плавное ограничение скорости — без резкого обнуления ускорения
        if (p->direction > 0 && v_new > p->v_target) // Если разгон и превысили цель
        {
            v_new = p->v_target; // Фиксируем скорость на цели
        }
        else if (p->direction < 0 && v_new < p->v_target) // Если торможение и проскочили цель
        {
            v_new = p->v_target; // Фиксируем скорость на цели
        }

        p->j_current = j;     // Сохраняем текущий рывок
        p->a_current = a_new; // Обновляем текущее ускорение
        p->v_current = v_new; // Обновляем текущую скорость
        p->t_current += dt;   // Увеличиваем текущее время профиля
    }

    // Детальный лог на каждом шаге — если включена диагностика
    if (p->enable_diagnostics) // Если включена диагностика
    {
        const char *phase_str[] = {"-", "Ф1", "Ф2", "Ф3"};                  // Массив строк для фаз — Ф2 добавлена!
        const char *state_str[] = {"IDLE", "RUN", "COAST"};                 // ← УПРОЩЕНО: только 3 состояния
        const char *dir_str = (p->direction > 0) ? "разгон" : "тормож";     // Строка для направления
        const char *profile_str = p->has_cruise_phase ? "трапец" : "треуг"; // Строка для типа профиля

        printf("[%s] t=%.3f | v=%.3f | a=%.3f | j=%.3f | фаза=%s | сост=%s | цель=%.3f | dir=%s | проф=%s | coast_t=%.3f\n", // Форматированный вывод
               p->wheel_name,                                                                                                // Имя колеса
               p->t_current,                                                                                                 // Текущее время
               p->v_current,                                                                                                 // Текущая скорость
               p->a_current,                                                                                                 // Текущее ускорение
               p->j_current,                                                                                                 // Текущий рывок
               phase_str[p->phase],                                                                                          // Текущая фаза
               state_str[p->state],                                                                                          // Текущее состояние
               p->v_target,                                                                                                  // Целевая скорость
               dir_str,                                                                                                      // Направление
               profile_str,                                                                                                  // Тип профиля
               p->t_coast_elapsed);                                                                                          // Время в фазе сброса
    }
}
/*
// Основная программа — для теста
int main()
{
    JerkLimitedProfile left_wheel;                                 // Профиль для левого колеса
    JerkLimitedProfile right_wheel;                                // Профиль для правого колеса

    // Инициализируем профили — БЕЗ начального вызова jlp_start_profile
    jlp_init(&left_wheel,  "left",  0.0, 2.0, 1.0, 0.5);          // Имя, нач. скорость, j_max, a_max, v_max
    jlp_init(&right_wheel, "right", 0.0, 2.0, 1.0, 0.5);          // Аналогично для правого

    left_wheel.enable_diagnostics = 1;                             // Включаем диагностику для левого
    right_wheel.enable_diagnostics = 1;                            // Включаем диагностику для правого

    double dt = 0.01;                                              // Шаг времени — 10 мс
    int steps = 0;                                                 // Счётчик шагов

    double last_desired_speedL = 0.0;                              // Предыдущая цель для левого колеса
    double last_desired_speedR = 0.0;                              // Предыдущая цель для правого колеса
    const double EPSILON = 1e-3;                                   // Порог изменения цели

    // Структура для хранения желаемых скоростей
    typedef struct {
        double speedL;
        double speedR;
    } DesiredSpeeds;

    DesiredSpeeds g_desiredSpeed = {0.0, 0.0};                     // Глобальная структура с желаемыми скоростями

    printf("Лог включён. dt=%.3f с\n", dt);                        // Заголовок лога
    printf("================================================================================\n"); // Разделитель

    // Основной цикл симуляции — 110 шагов (1.1 секунды)
    while (steps < 110)
    {
        // Эмуляция команды — на 10-м шаге (0.1 с) задаём новую цель
        if (steps == 10) {                                         // Если шаг 10
            g_desiredSpeed.speedL = 0.5;                           // Новая цель для левого
            g_desiredSpeed.speedR = 0.5;                           // Новая цель для правого
        }

        // Проверка левого колеса — вызываем replan ТОЛЬКО если цель изменилась значимо
        if (fabs(g_desiredSpeed.speedL - last_desired_speedL) > EPSILON) // Если изменение больше порога
        {
            jlp_request_replan(&left_wheel, g_desiredSpeed.speedL); // Запрашиваем пересчёт
            last_desired_speedL = g_desiredSpeed.speedL;            // Обновляем предыдущее значение
        }

        // Проверка правого колеса — аналогично
        if (fabs(g_desiredSpeed.speedR - last_desired_speedR) > EPSILON) // Если изменение больше порога
        {
            jlp_request_replan(&right_wheel, g_desiredSpeed.speedR); // Запрашиваем пересчёт
            last_desired_speedR = g_desiredSpeed.speedR;             // Обновляем предыдущее значение
        }

        // Выполняем ОДИН ШАГ управления для каждого колеса
        jlp_step(&left_wheel,  dt);                                // Шаг для левого колеса
        jlp_step(&right_wheel, dt);                                // Шаг для правого колеса

        // Реальная задержка — для симуляции в реальном времени (в embedded не нужно)
        usleep((unsigned int)(dt * 1000000));                      // Ждём dt секунд

        steps++;                                                   // Увеличиваем счётчик шагов
    }

    printf("\n>>> СИМУЛЯЦИЯ ЗАВЕРШЕНА <<<\n");                     // Финальное сообщение
    return 0;                                                      // Завершаем программу
}
*/
#endif
