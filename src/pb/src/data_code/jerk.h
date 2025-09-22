#ifndef JERK_H
#define JERK_H

#include <stdio.h>  // Подключаем стандартную библиотеку ввода-вывода — для printf (в диагностике)
#include <stdlib.h> // Подключаем стандартную библиотеку — резерв (может пригодиться)
#include <math.h>   // Подключаем математическую библиотеку — для fabs, sqrt
#include <cmath>    // Подключаем математическую библиотеку — для fabs, sqrt
#include <cmath>    // Подключаем математическую библиотеку — для fabs, sqrt
#include "config.h"

#include <stdio.h>  // Подключаем стандартную библиотеку ввода-вывода — для printf (в диагностике)
#include <stdlib.h> // Подключаем стандартную библиотеку — резерв (может пригодиться)
#include <math.h>   // Подключаем математическую библиотеку — для fabs, sqrt, fmin, fmax
#include <unistd.h> // Подключаем для usleep — реальная задержка в симуляции (в embedded не нужно)

// Определяем состояния профиля — чтобы код был понятнее и читабельнее
typedef enum
{
    JLP_IDLE = 0,       // Профиль не активен — робот стоит
    JLP_RUNNING = 1,    // Профиль выполняется — разгон/торможение по плану
    JLP_COASTING_J = 2, // Режим плавного сброса рывка — перед пересчётом
    JLP_COASTING_A = 3  // Режим плавного сброса ускорения — после сброса рывка
} JLP_State;

// Структура состояния профиля — ВСЁ хранится здесь, чтобы не терять контекст между вызовами
typedef struct
{
    // Для логов — имя профиля (например, "left", "right")
    const char *wheel_name; // Указатель на строку с именем колеса — задаётся при инициализации и не меняется

    // Текущее состояние движения — обновляется на каждом шаге
    double v_current; // Текущая скорость (м/с) — вход и выход алгоритма
    double a_current; // Текущее ускорение (м/с²) — вход и выход алгоритма
    double j_current; // Текущий рывок (м/с³) — вход и выход алгоритма
    double t_current; // Текущее время профиля (с) — увеличивается на dt на каждом шаге
    int phase;        // Текущая фаза профиля: 1=нарастание, 2=крейсер, 3=сброс

    // Параметры текущего активного профиля — рассчитываются при старте или пересчёте
    double v_target;      // Целевая скорость (м/с) — к чему стремимся в текущем профиле
    double t1_end;        // Время окончания фазы 1 — когда заканчивается нарастание ускорения
    double t2_end;        // Время окончания фазы 2 — когда заканчивается постоянное ускорение (если есть)
    double t_total;       // Общее время профиля — сколько длится весь профиль от начала до конца
    int has_cruise_phase; // 1 — есть фаза постоянного ускорения, 0 — нет (треугольный профиль)
    double a_peak_used;   // Пиковое ускорение (по модулю) — реальное значение, которое будет использовано (может быть < a_max)
    int direction;        // Направление движения: +1 = разгон, -1 = торможение

    // Состояние конечного автомата — управляет логикой выполнения
    JLP_State state; // Текущее состояние автомата: IDLE, RUNNING, COASTING_J, COASTING_A
    int is_finished; // Флаг завершения: 1 — профиль завершён, 0 — ещё выполняется

    // Параметры системы — задаются один раз при инициализации
    double j_max; // Максимальный рывок (м/с³) — ограничение на скорость изменения ускорения (всегда положительный)
    double a_max; // Максимальное ускорение (м/с²) — ограничение на ускорение/торможение (всегда положительный)
    double v_max; // Максимальная физическая скорость колеса (м/с) — ограничение на целевую скорость

    // Параметры для пересчёта профиля "на лету"
    int replan_requested;   // Флаг: 1 — запрошен пересчёт профиля, 0 — нет
    double v_target_new;    // Новая целевая скорость — на которую нужно пересчитать профиль
    double t_coast_j;       // Время, необходимое для плавного сброса текущего рывка до 0
    double t_coast_a;       // Время, необходимое для плавного сброса ускорения до 0 (после сброса рывка)
    double t_coast_elapsed; // Сколько времени уже прошло в текущей фазе сброса (J или A)
    double j_coast_sign;    // Знак рывка, который нужно применить для сброса текущего рывка (чтобы плавно сбросить до 0)
    double a_coast_sign;    // Знак рывка, который нужно применить для сброса ускорения (чтобы плавно сбросить до 0)

    // Флаг для включения/выключения диагностики — в продакшене можно отключить
    int enable_diagnostics; // 1 — выводить диагностические сообщения, 0 — не выводить
} JerkLimitedProfile;

// Инициализация структуры профиля — вызывается один раз при старте системы
void jlp_init(JerkLimitedProfile *p, const char *name, double v_start, double j_max, double a_max, double v_max)
{
    p->wheel_name = name;      // Сохраняем имя колеса для логов — например, "left" или "right"
    p->v_current = v_start;    // Устанавливаем начальную скорость из параметра
    p->a_current = 0.0;        // Начальное ускорение — 0 (профиль всегда стартует с нуля)
    p->j_current = 0.0;        // Начальный рывок — 0
    p->t_current = 0.0;        // Текущее время профиля — 0
    p->phase = 1;              // Начинаем всегда с фазы 1 — нарастание рывка
    p->v_target = v_start;     // По умолчанию — целевая скорость = начальной (не двигаемся)
    p->state = JLP_IDLE;       // Начальное состояние — бездействие
    p->is_finished = 1;        // Профиль считается завершённым (пока не запущен)
    p->j_max = j_max;          // Сохраняем макс. рывок — параметр системы
    p->a_max = a_max;          // Сохраняем макс. ускорение — параметр системы
    p->v_max = v_max;          // Сохраняем макс. скорость колеса — физический лимит
    p->replan_requested = 0;   // Сбрасываем флаг пересчёта
    p->t_coast_elapsed = 0.0;  // Обнуляем счётчик времени сброса
    p->enable_diagnostics = 0; // По умолчанию диагностика отключена
}

// Запуск нового профиля — вызывается при старте или после пересчёта
void jlp_start_profile(JerkLimitedProfile *p, double v_target)
{
    // Ограничиваем целевую скорость физическим лимитом колеса — второй уровень защиты
    double v_target_limited = v_target;
    if (p->v_max > 0)
    {
        v_target_limited = fmax(-p->v_max, fmin(v_target, p->v_max)); // Ограничиваем в диапазоне [-v_max, v_max]
    }

    p->v_target = v_target_limited; // Устанавливаем новую целевую скорость (ограниченную)
    p->t_current = 0.0;             // Сбрасываем таймер профиля
    p->phase = 1;                   // Всегда начинаем с фазы 1
    p->is_finished = 0;             // Сбрасываем флаг завершения

    double dv = v_target_limited - p->v_current; // Вычисляем разницу между целевой и текущей скоростью
    p->direction = (dv >= 0.0) ? 1 : -1;         // Определяем направление: +1 = разгон, -1 = торможение
    double dv_abs = fabs(dv);                    // Берём модуль — для одинаковых расчётов в обоих направлениях

    if (dv_abs < 1e-9) // Если разница скоростей почти нулевая — профиль не нужен
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

    if (dv_abs >= v_ramp_total) // Если нужно больше скорости, чем дают фазы 1+3 — нужна фаза 2
    {
        double v_cruise = dv_abs - v_ramp_total;     // Остаток скорости для фазы постоянного ускорения
        double t_cruise = v_cruise / p->a_max;       // Время фазы 2 — сколько держать макс. ускорение
        p->t1_end = t_j_full;                        // Записываем время окончания фазы 1
        p->t2_end = t_j_full + t_cruise;             // Записываем время окончания фазы 2
        p->t_total = t_j_full + t_cruise + t_j_full; // Общее время профиля = фаза1 + фаза2 + фаза3
        p->has_cruise_phase = 1;                     // Указываем, что фаза 2 есть
        p->a_peak_used = p->a_max;                   // Используем макс. ускорение
    }
    else // Если нужно меньше — треугольный профиль
    {
        double a_peak = sqrt(dv_abs * p->j_max); // Рассчитываем пиковое ускорение для треугольного профиля
        double t_j = a_peak / p->j_max;          // Время выхода на это пиковое ускорение
        p->t1_end = t_j;                         // Время окончания фазы 1
        p->t2_end = t_j;                         // В треугольном профиле фазы 2 нет — t2 = t1
        p->t_total = 2.0 * t_j;                  // Общее время — две симметричные фазы
        p->has_cruise_phase = 0;                 // Фаза 2 отсутствует
        p->a_peak_used = a_peak;                 // Записываем реальное пиковое ускорение
    }

    p->state = JLP_RUNNING; // Переключаемся в состояние выполнения профиля

    if (p->enable_diagnostics)                                      // Если включена диагностика
        printf("[%s][DIAG] Старт профиля: v_target=%.1f, %s, %s\n", // Выводим информацию о профиле
               p->wheel_name, v_target_limited, p->direction > 0 ? "разгон" : "торможение",
               p->has_cruise_phase ? "трапеция" : "треугольник");
}

// Запрос пересчёта профиля "на лету" — можно вызывать В ЛЮБОМ СОСТОЯНИИ и В ЛЮБОЙ МОМЕНТ
void jlp_request_replan(JerkLimitedProfile *p, double v_target_new)
{
    // Сохраняем новую целевую скорость — независимо от текущего состояния
    p->v_target_new = v_target_new; // Запоминаем новую цель — она заменяет любую предыдущую

    // Устанавливаем флаг запроса пересчёта — даже если уже был установлен (перезаписываем)
    p->replan_requested = 1; // Флаг говорит: "на следующем шаге начни пересчёт"

    // Сбрасываем накопленное время сброса — чтобы начать с чистого листа (даже если были в середине сброса)
    p->t_coast_elapsed = 0.0; // Обнуляем счётчик времени сброса — начнём заново

    // Если включена диагностика — выводим информацию о запросе (для отладки)
    if (p->enable_diagnostics)                                                                                    // Проверяем, включена ли диагностика
        printf("[%s][EVENT] Запрошен пересчёт на v=%.1f (текущее состояние: v=%.3f, a=%.3f, j=%.3f, state=%d)\n", // Выводим новую цель и текущее состояние
               p->wheel_name, v_target_new, p->v_current, p->a_current, p->j_current, p->state);

    // ВАЖНО: НЕ возвращаемся, даже если state == IDLE — пусть jlp_step сам обработает запрос в своём контексте
    // Это обеспечивает единый путь обработки — независимо от состояния
}

// Один шаг управления — вызывать строго каждые dt секунд в основном цикле или таймере
void jlp_step(JerkLimitedProfile* p, double dt)
{
    // Сначала — обрабатываем запрос на пересчёт, ДАЖЕ ЕСЛИ в IDLE
    if (p->replan_requested)
    {
        p->t_coast_j = fabs(p->j_current) / p->j_max;              // Рассчитываем время сброса рывка до 0
        p->j_coast_sign = (p->j_current >= 0) ? -1.0 : 1.0;        // Определяем знак рывка для плавного сброса

        // Рассчитываем ускорение после сброса рывка: a = a0 + j0*t + 0.5*j_new*t²
        double a_after_j = p->a_current + p->j_current * p->t_coast_j +
                           0.5 * p->j_coast_sign * p->j_max * p->t_coast_j * p->t_coast_j;

        p->t_coast_a = fabs(a_after_j) / p->j_max;                 // Рассчитываем время сброса ускорения до 0
        p->a_coast_sign = (a_after_j >= 0) ? -1.0 : 1.0;           // Определяем знак рывка для сброса ускорения

        p->t_coast_elapsed = 0.0;                                  // Обнуляем счётчик времени сброса
        p->state = JLP_COASTING_J;                                 // Переключаемся в режим сброса рывка — ВЫХОДИМ из IDLE!
        p->replan_requested = 0;                                   // Сбрасываем флаг запроса

        if (p->enable_diagnostics)                                 // Если включена диагностика
            printf("[%s][REPLAN] Переход в COAST_J (t_coast_j=%.3f, t_coast_a=%.3f)\n",
                   p->wheel_name, p->t_coast_j, p->t_coast_a);
    }

    // Теперь — если всё ещё в IDLE — выходим (например, если не было replan_requested)
    if (p->state == JLP_IDLE) {
        // Логируем даже в IDLE — для ясности
        if (p->enable_diagnostics) {
            printf("[%s] IDLE: v=%.3f, a=%.3f, j=%.3f, t=%.3f, target=%.3f\n",
                   p->wheel_name, p->v_current, p->a_current, p->j_current, p->t_current, p->v_target);
        }
        return;                                                    // Если профиль не активен — ничего не делаем
    }

    // Обработка состояния: сброс рывка
    if (p->state == JLP_COASTING_J)
    {
        if (p->t_coast_elapsed >= p->t_coast_j || p->t_coast_j < 1e-6) // Если время сброса рывка истекло или не нужно
        {
            p->state = JLP_COASTING_A;                             // Переходим к сбросу ускорения
            p->t_coast_elapsed = 0.0;                              // Обнуляем счётчик времени
            if (p->enable_diagnostics) printf("[%s][COAST] Переход в COAST_A\n", p->wheel_name); // Выводим диагностику
        }
        else
        {
            p->j_current = p->j_coast_sign * p->j_max;             // Применяем рывок для сброса
            p->a_current += p->j_current * dt;                     // Обновляем ускорение: a = a_old + j * dt
            p->v_current += p->a_current * dt;                     // Обновляем скорость: v = v_old + a_old * dt (метод Эйлера)
            p->t_coast_elapsed += dt;                              // Увеличиваем счётчик времени сброса
        }
        p->t_current += dt;// ВАЖНО: увеличиваем общее время профиля — даже в состоянии сброса!
    }
    // Обработка состояния: сброс ускорения
    else if (p->state == JLP_COASTING_A)
    {
        if (p->t_coast_elapsed >= p->t_coast_a || p->t_coast_a < 1e-6) // Если время сброса ускорения истекло или не нужно
        {
            p->a_current = 0.0;                                    // Явно обнуляем ускорение — для чистоты
            p->j_current = 0.0;                                    // Явно обнуляем рывок — для чистоты

            jlp_start_profile(p, p->v_target_new);                 // Запускаем новый профиль с новой целевой скоростью
            if (p->enable_diagnostics)                             // Если включена диагностика
                printf("[%s][REPLAN] Новый профиль стартовал с v=%.3f\n", p->wheel_name, p->v_current); // Выводим стартовую скорость

            // ВАЖНО: на этом шаге — не делаем ничего больше — выходим
            // Лог уже будет в конце функции
            p->t_current += dt;  // ← Время должно пройти и на этом шаге!
            return;              // ← ВЫХОДИМ — чтобы не выполнять блок RUNNING на этом шаге
        }
        else
        {
            p->j_current = p->a_coast_sign * p->j_max;             // Применяем рывок для сброса ускорения
            p->a_current += p->j_current * dt;                     // Обновляем ускорение
            p->v_current += p->a_current * dt;                     // Обновляем скорость
            p->t_coast_elapsed += dt;                              // Увеличиваем счётчик времени сброса
        }
        p->t_current += dt;// ВАЖНО: увеличиваем общее время профиля — даже в состоянии сброса!
    }
    // Обработка состояния: выполнение основного профиля
    else if (p->state == JLP_RUNNING)
    {
        // Проверка: если профиль завершён по времени или достигнута цель
        if (p->t_current >= p->t_total || fabs(p->v_current - p->v_target) < 1e-6)
        {
            p->a_current = 0.0;                                    // Обнуляем ускорение
            p->j_current = 0.0;                                    // Обнуляем рывок
            p->is_finished = 1;                                    // Помечаем профиль как завершённый
            p->state = JLP_IDLE;                                   // Переключаемся в состояние бездействия
            if (p->enable_diagnostics) printf("[%s][DIAG] Профиль завершён\n", p->wheel_name); // Выводим диагностику
            // Логируем финальное состояние
            if (p->enable_diagnostics) {
                printf("[%s] IDLE: v=%.3f, a=%.3f, j=%.3f, t=%.3f, target=%.3f\n",
                       p->wheel_name, p->v_current, p->a_current, p->j_current, p->t_current, p->v_target);
            }
            return;                                                // Выходим — на этом шаге больше ничего не делаем
        }

        double j = 0.0;                                            // Инициализируем рывок нулём

        if (p->phase == 1)                                         // Фаза 1: нарастание рывка
        {
            j = p->direction * p->j_max;                           // Применяем рывок: +j_max при разгоне, -j_max при торможении
            if (p->t_current >= p->t1_end)                         // Если достигли конца фазы 1
            {
                p->phase = 2;                                      // Переключаемся на фазу 2
                if (p->enable_diagnostics) printf("[%s][DIAG] Переход на фазу 2\n", p->wheel_name); // Выводим диагностику
            }
        }
        else if (p->phase == 2)                                    // Фаза 2: постоянное ускорение
        {
            j = 0.0;                                               // Рывок = 0 — ускорение не меняется
            if (p->t_current >= p->t2_end)                         // Если достигли конца фазы 2
            {
                p->phase = 3;                                      // Переключаемся на фазу 3
                if (p->enable_diagnostics) printf("[%s][DIAG] Переход на фазу 3\n", p->wheel_name); // Выводим диагностику
            }
        }
        else if (p->phase == 3)                                    // Фаза 3: сброс ускорения
        {
            j = -p->direction * p->j_max;                          // Применяем противоположный рывок для сброса ускорения
        }

        double a_new = p->a_current + j * dt;                      // Вычисляем новое ускорение: a = a_old + j * dt
        double a_limit = p->direction * p->a_peak_used;            // Рассчитываем лимит ускорения с учётом направления
        if (p->direction > 0 && a_new > a_limit) a_new = a_limit;  // Ограничиваем сверху при разгоне
        if (p->direction < 0 && a_new < a_limit) a_new = a_limit;  // Ограничиваем снизу при торможении

        double v_new = p->v_current + a_new * dt;  // ← Используем НОВОЕ ускорение, а не старое
        // double v_new = p->v_current + p->a_current * dt;           // Вычисляем новую скорость: v = v_old + a_old * dt (метод Эйлера)

        if (p->direction > 0 && v_new > p->v_target)               // Если разгон и превысили цель
        {
            v_new = p->v_target;                                   // Фиксируем скорость на цели
            a_new = 0.0;                                           // Обнуляем ускорение
            j = 0.0;                                               // Обнуляем рывок
        }
        else if (p->direction < 0 && v_new < p->v_target)          // Если торможение и проскочили цель
        {
            v_new = p->v_target;                                   // Фиксируем скорость на цели
            a_new = 0.0;                                           // Обнуляем ускорение
            j = 0.0;                                               // Обнуляем рывок
        }

        p->j_current = j;                                          // Сохраняем текущий рывок
        p->a_current = a_new;                                      // Обновляем текущее ускорение
        p->v_current = v_new;                                      // Обновляем текущую скорость
        p->t_current += dt;                                        // Увеличиваем текущее время профиля
    }

    // === ДЕТАЛЬНЫЙ ЛОГ НА КАЖДОМ ШАГЕ ===
    if (p->enable_diagnostics)
    {
        const char* phase_str[] = {"-", "Ф1", "Ф2", "Ф3"};
        const char* state_str[] = {"IDLE", "RUN", "COAST_J", "COAST_A"};
        const char* dir_str = (p->direction > 0) ? "разгон" : "тормож";
        const char* profile_str = p->has_cruise_phase ? "трапец" : "треуг";

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
// ============================
// ОСНОВНАЯ ПРОГРАММА (ТЕСТ)
// ============================

int main()
{
    // Создаём два профиля — для левого и правого колеса
    JerkLimitedProfile left_wheel;                                 // Профиль для левого колеса
    JerkLimitedProfile right_wheel;                                // Профиль для правого колеса

    // Инициализируем профили
    jlp_init(&left_wheel,  "left",  0.0, 1.0, 10.0, 2.0);         // Имя, нач. скорость, j_max, a_max, v_max
    jlp_init(&right_wheel, "right", 0.0, 1.0, 10.0, 2.0);         // Аналогично для правого

    // Включаем диагностику для теста
    left_wheel.enable_diagnostics = 1;
    right_wheel.enable_diagnostics = 1;

    // Запускаем начальные профили (можно и не запускать — первый replan всё инициализирует)
    jlp_start_profile(&left_wheel,  0.0);
    jlp_start_profile(&right_wheel, 0.0);

    double dt = 0.01;                                              // Шаг времени — 10 мс (100 Гц)
    int steps = 0;                                                 // Счётчик шагов

    // Предыдущие желаемые скорости — для сравнения
    double last_desired_speedL = 0.0;                              // Предыдущая цель для левого колеса
    double last_desired_speedR = 0.0;                              // Предыдущая цель для правого колеса
    const double EPSILON = 1e-3;                                   // Порог изменения — 0.001 м/с

    // Эмуляция получения команд — в реальной системе будет от джойстика/планировщика
    typedef struct {
        double speedL;
        double speedR;
    } DesiredSpeeds;

    DesiredSpeeds g_desiredSpeed = {0.0, 0.0};                     // Глобальная структура с желаемыми скоростями

    printf("[%s][%s] t\t\tv\t\ta\t\tj\t\tstate\n", "left", "right"); // Заголовок с именами
    printf("---------------------------------------------------------------\n");

    while (steps < 3000)                                           // Максимум 30 сек (3000 шагов по 0.01 с)
    {
        // Эмуляция изменения желаемых скоростей — в реальной системе это будет от внешнего источника
        if (steps == 100) {                                        // На 1-й секунде
            g_desiredSpeed.speedL = 1.5;                           // Новая цель для левого
            g_desiredSpeed.speedR = 1.5;                           // Новая цель для правого
        }
        if (steps == 500) {                                        // На 5-й секунде
            g_desiredSpeed.speedL = 0.5;                           // Замедляемся
            g_desiredSpeed.speedR = 2.0;                           // Поворот (разные скорости)
        }
        if (steps == 1000) {                                       // На 10-й секунде
            g_desiredSpeed.speedL = 0.0;                           // Остановка
            g_desiredSpeed.speedR = 0.0;
        }
        if (steps == 1500) {                                       // На 15-й секунде
            g_desiredSpeed.speedL = -1.0;                          // Задний ход
            g_desiredSpeed.speedR = -1.0;
        }

        // Проверяем левое колесо — вызываем replan ТОЛЬКО если цель изменилась значимо
        if (fabs(g_desiredSpeed.speedL - last_desired_speedL) > EPSILON)
        {
            jlp_request_replan(&left_wheel, g_desiredSpeed.speedL); // Запрашиваем пересчёт
            last_desired_speedL = g_desiredSpeed.speedL;            // Обновляем предыдущее значение
        }

        // Проверяем правое колесо — аналогично
        if (fabs(g_desiredSpeed.speedR - last_desired_speedR) > EPSILON)
        {
            jlp_request_replan(&right_wheel, g_desiredSpeed.speedR); // Запрашиваем пересчёт
            last_desired_speedR = g_desiredSpeed.speedR;             // Обновляем предыдущее значение
        }

        // Выполняем ОДИН ШАГ управления для каждого колеса
        jlp_step(&left_wheel,  dt);
        jlp_step(&right_wheel, dt);

        // Выводим состояние каждые 0.5 сек (50 шагов)
        if (steps % 50 == 0)
        {
            const char* state_str[] = {"IDLE", "RUN", "COAST_J", "COAST_A"}; // Массив строк для состояний
            printf("[%s][%s] %.2f\t%.3f\t%.3f\t%.3f\t%s | %.3f\t%.3f\t%.3f\t%s\n",
                   "left", "right",                                // Имена колонок
                   left_wheel.t_current,                           // Данные левого колеса
                   left_wheel.v_current, left_wheel.a_current, left_wheel.j_current,
                   state_str[left_wheel.state],
                   right_wheel.v_current, right_wheel.a_current, right_wheel.j_current, // Данные правого колеса
                   state_str[right_wheel.state]);
        }

        // Реальная задержка — для симуляции в реальном времени (в embedded не нужно)
        usleep((unsigned int)(dt * 1000000));                      // Ждём dt секунд

        steps++;                                                   // Увеличиваем счётчик шагов
    }

    printf("\n>>> ГОТОВО <<<\n");
    printf("[left]  Финал: v=%.3f, a=%.3f, j=%.3f, state=%d\n",
           left_wheel.v_current, left_wheel.a_current, left_wheel.j_current, left_wheel.state);
    printf("[right] Финал: v=%.3f, a=%.3f, j=%.3f, state=%d\n",
           right_wheel.v_current, right_wheel.a_current, right_wheel.j_current, right_wheel.state);

    return 0;                                                      // Завершаем программу
}

*/
#endif
