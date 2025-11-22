#ifndef L1CONTROLLER_H
#define L1CONTROLLER_H

#pragma once

#include <cmath>     // Для std::sqrt, std::pow, std::sin, std::atan2, std::abs
#include <algorithm> // Для std::max, std::min
#include <cstdio>    // Для использования printf

// Константы
#define M_PI 3.14159265358979323846 // Число Пи

// ====================================================================
// СТРУКТУРЫ
// ====================================================================

/**
 * @brief Структура для представления точки (x, y)
 */
struct Point
{ // Начало структуры Point
    double x; // Координата X
    double y; // Координата Y
}; // Конец структуры Point

// ====================================================================
// КЛАСС L1GUIDANCECONTROLLER
// ====================================================================

/**
 * @brief Класс, реализующий L1-контроллер (Guidance Law)
 * с поддержкой двух методов поиска точки упреждения.
 */
class L1GuidanceController
{ // Начало класса L1GuidanceController
private:
    // Параметры, определяющие физику и управление
    double m_lookahead_L; // Дистанция упреждения L (радиус окружности для Pure Pursuit)
    double m_track_width_W; // Трековая ширина W (база робота для обратной кинематики)
    double m_max_omega; // Максимально допустимая угловая скорость (рад/с)

    // ====================================================================
    // 1. ВСПОМОГАТЕЛЬНЫЕ ГЕОМЕТРИЧЕСКИЕ ФУНКЦИИ
    // ====================================================================

    /**
     * @brief Вспомогательная функция для расчета квадрата расстояния.
     */
    double distanceSq(const Point& p1, const Point& p2) const
    { // Начало функции distanceSq
        return std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2); // (dx^2 + dy^2)
    } // Конец функции distanceSq

    /**
     * @brief Вспомогательная функция: Ограничивает параметр t диапазоном [0, 1].
     */
    double clampT(double t) const
    { // Начало функции clampT
        return std::max(0.0, std::min(t, 1.0)); // Возвращает t, ограниченный отрезком [0, 1]
    } // Конец функции clampT

    /**
     * @brief Функция для нормализации угла в диапазоне [-PI, PI].
     */
    double normalizeAngle(double angle_rad) const
    { // Начало функции normalizeAngle
        angle_rad = std::fmod(angle_rad, 2.0 * M_PI); // Ограничение угла диапазоном (-2*PI, 2*PI)
        if (angle_rad <= -M_PI) // Если угол меньше -PI
            angle_rad += 2.0 * M_PI; // Добавляем 2*PI
        if (angle_rad > M_PI) // Если угол больше PI
            angle_rad -= 2.0 * M_PI; // Вычитаем 2*PI
        return angle_rad; // Возвращаем нормализованный угол
    } // Конец функции normalizeAngle

    /**
     * @brief Функция для вычисления угла (направления) от точки C до точки D.
     */
    double angleBetweenPoints(const Point& C, const Point& D) const
    { // Начало функции angleBetweenPoints
        // Используем atan2(dy, dx) для получения угла
        return std::atan2(D.y - C.y, D.x - C.x); // Угол в радианах (angle(C -> D))
    } // Конец функции angleBetweenPoints


public:
    /**
     * @brief Конструктор класса. Инициализирует параметры L и W.
     * @param L Дистанция упреждения L (look-ahead distance).
     * @param W Трековая ширина W (track width) робота.
     * @param max_omega Максимальная угловая скорость.
     */
    L1GuidanceController(double L, double W, double max_omega = 5.0)
    { // Начало конструктора
        m_lookahead_L = L; // Установка дистанции упреждения
        m_track_width_W = W; // Установка базы робота
        m_max_omega = max_omega; // Установка ограничения угловой скорости
    } // Конец конструктора

    // ====================================================================
    // 2. ВАРИАНТЫ ПОИСКА ТОЧКИ D (Целеуказание)
    // ====================================================================

    /**
     * @brief ВАРИАНТ 1: Геометрически точный поиск D.
     * Находит точное пересечение окружности C(L) с отрезком [A, B] (через квадратное уравнение).
     * Если пересечений нет, возвращает ближайшую точку (проекцию) на отрезке.
     * ГЕОМЕТРИЧЕСКИ БОЛЕЕ КОРРЕКТЕН для L1-контроллера.
     */
    Point findNearestPointD_Exact(const Point &A, const Point &B, const Point &C) const
    { // Начало функции findNearestPointD_Exact
        // 1. ПРОВЕРКА B: Если B внутри или на границе окружности L
        const double LSq = m_lookahead_L * m_lookahead_L; // Квадрат радиуса (L^2)
        if (distanceSq(C, B) <= LSq) // Если d(C, B) <= L
        { // Начало блока if
            return B; // Искомая точка D - это B
        } // Конец блока if

        // 2. ПОИСК ПЕРЕСЕЧЕНИЙ с БЕСКОНЕЧНОЙ прямой AB
        const double dx = B.x - A.x;             // Разность координат X (вектор AB)
        const double dy = B.y - A.y;             // Разность координат Y (вектор AB)
        const double abSq = dx * dx + dy * dy; // |AB|^2

        if (abSq == 0.0) // Защита от A == B
        { // Начало блока if
            return A; // Возвращаем A
        } // Конец блока if

        const double Ax = A.x - C.x; // Компонента (xA - xC)
        const double Ay = A.y - C.y; // Компонента (yA - yC)

        // Коэффициенты для квадратного уравнения a*t^2 + b*t + c = 0
        const double a = abSq;                  // a = |AB|^2
        const double b = 2.0 * (dx * Ax + dy * Ay); // b = 2 * (AB * AC)
        const double c = Ax * Ax + Ay * Ay - LSq;  // c = |AC|^2 - L^2
        const double D = b * b - 4.0 * a * c;   // Дискриминант D

        double bestT = -1.0; // Хранение наилучшего t

        if (D >= 0.0) // Если есть действительные пересечения
        { // Начало блока if
            const double sqrtD = std::sqrt(D);          // Корень из дискриминанта
            const double t1 = (-b + sqrtD) / (2.0 * a); // Параметр t для первого решения
            const double t2 = (-b - sqrtD) / (2.0 * a); // Параметр t для второго решения

            // 3. ФИЛЬТРАЦИЯ: Оставляем t, лежащие на отрезке [0, 1]
            if (t1 >= 0.0 && t1 <= 1.0) 
            {
                bestT = std::max(bestT, t1); // Берем наибольшее t (ближе к B)
            }
            if (t2 >= 0.0 && t2 <= 1.0) 
            {
                bestT = std::max(bestT, t2); // Берем наибольшее t
            }
        } // Конец блока if

        // 4. ВЫБОР РЕШЕНИЯ
        if (bestT >= 0.0) // Если найдено действительное пересечение
        { // Начало блока if
            return { A.x + bestT * dx, A.y + bestT * dy }; // Возвращаем D на отрезке
        } // Конец блока if

        // 5. НЕТ ПЕРЕСЕЧЕНИЙ: Возвращаем ближайшую точку (проекцию)
        const double t_proj = -(dx * Ax + dy * Ay) / abSq; // Параметр t для проекции
        const double t_clamped = clampT(t_proj); // Проекция, ограниченная отрезком [0, 1]

        return // Возвращаем ближайшую точку на отрезке [A, B]
            { 
                A.x + t_clamped * dx, // Px
                A.y + t_clamped * dy  // Py
            }; // Конец возврата
    } // Конец функции findNearestPointD_Exact

    /**
     * @brief ВАРИАНТ 2: Проекционный поиск D.
     * Находит точку D на отрезке, откладывая L от проекции робота на путь.
     * ПРОЩЕ, но ГЕОМЕТРИЧЕСКИ НЕ СООТВЕТСТВУЕТ исходному предположению L1.
     */
    Point findNearestPointD_2var(const Point &A, const Point &B, const Point &C) const
    { // Начало функции findNearestPointD_2var
        // Вектор AB
        const double dx = B.x - A.x; // Вектор пути X
        const double dy = B.y - A.y; // Вектор пути Y

        const double ab_len = std::sqrt(dx*dx + dy*dy); // Длина отрезка |AB|
        if (ab_len < 1e-9) // вырожденный случай
            return A; 

        // Нормированный вектор вдоль пути
        const double ux = dx / ab_len; // ux
        const double uy = dy / ab_len; // uy

        // Вектор AC (от начала отрезка до робота)
        const double ACx = C.x - A.x; // ACx
        const double ACy = C.y - A.y; // ACy

        // Проекция AC на направление пути: расстояние от A до проекции
        double t_proj = ACx * ux + ACy * uy; // Проекция t (скалярное произведение)

        // Ограничиваем t_proj на диапазон [0, длина AB]
        if (t_proj < 0.0) t_proj = 0.0; // Если проекция до A, берем A
        if (t_proj > ab_len) t_proj = ab_len; // Если проекция после B, берем B

        // Какое расстояние нужно отложить вперед от проекции (t_proj + L)
        double target_dist_from_A = t_proj + m_lookahead_L; 

        // Ограничиваем, чтобы D не вышла за конец отрезка B
        if (target_dist_from_A > ab_len) 
            target_dist_from_A = ab_len; // Берем точку B, если L выходит за пределы

        // Искомая точка D = A + u * (target_dist_from_A)
        Point D; // Объект точки D
        D.x = A.x + ux * target_dist_from_A; // Вычисление X
        D.y = A.y + uy * target_dist_from_A; // Вычисление Y

        return D; // Возвращаем D
    } // Конец функции findNearestPointD_2var

    // ====================================================================
    // 3. УПРАВЛЕНИЕ И КИНЕМАТИКА
    // ====================================================================

    /**
     * @brief Рассчитывает требуемую угловую скорость (Омега) с помощью L1-контроллера
     * и конвертирует ее в скорости колес (VL, VR).
     * * @param C Положение робота.
     * @param heading_rad Текущий курс робота.
     * @param D Целевая точка упреждения.
     * @param V Линейная скорость (должна быть > 0).
     * @param omega OUT: Угловая скорость (рад/с).
     * @param VL OUT: Скорость левого колеса (м/с).
     * @param VR OUT: Скорость правого колеса (м/с).
     */
    void calculateControlCommands(
        const Point& C,     
        double heading_rad,
        const Point& D,       
        double V,   
        double& omega,              
        double& VL,                 
        double& VR                  
    ) const
    { // Начало функции calculateControlCommands
        // 1. Находим угол ошибки (Эта / eta)
        double target_angle_rad = angleBetweenPoints(C, D); // Направление к D
        double angle_error_rad = normalizeAngle(target_angle_rad - heading_rad); // Угол упреждения η
        
        // 2. L1-КОНТРОЛЛЕР: ω = (2V / L) * sin(η)
        // Защита от деления на ноль, если L=0 (хотя L должна быть > 0)
        if (m_lookahead_L < 1e-6)
        { // Начало блока if
             omega = 0.0; // Устанавливаем ω=0
        } // Конец блока if
        else 
        { // Начало блока else
            omega = (2.0 * V / m_lookahead_L) * std::sin(angle_error_rad); // Расчет угловой скорости ω
        } // Конец блока else

        // Ограничиваем максимальное значение угловой скорости
        if (std::abs(omega) > m_max_omega) // Если абсолютное значение превышает максимум
        { // Начало блока if
            // Ограничиваем с сохранением знака
            omega = (omega > 0) ? m_max_omega : -m_max_omega; // Clamp to max_omega
        } // Конец блока if

        // 3. ОБРАТНАЯ КИНЕМАТИКА: Конвертация V и ω в скорости колес
        double rotational_component = omega * m_track_width_W / 2.0; // Rotational velocity component (ω * W/2)
        VR = V + rotational_component; // Скорость правого колеса (VR = V + ωW/2)
        VL = V - rotational_component; // Скорость левого колеса (VL = V - ωW/2)
    } // Конец функции calculateControlCommands

    // ====================================================================
    // 4. МЕТОДЫ НАСТРОЙКИ (Сеттеры)
    // ====================================================================
    
    /**
     * @brief Устанавливает новую дистанцию упреждения L.
     */
    void setLookaheadDistance(double L)
    { // Начало функции setLookaheadDistance
        m_lookahead_L = L; // Установка L
    } // Конец функции setLookaheadDistance

    /**
     * @brief Устанавливает новую максимальную угловую скорость.
     */
    void setMaxOmega(double max_omega)
    { // Начало функции setMaxOmega
        m_max_omega = max_omega; // Установка ограничения ω
    } // Конец функции setMaxOmega
}; // Конец класса L1GuidanceController

/*
// ====================================================================
// ПРИМЕР ИСПОЛЬЗОВАНИЯ И HELP
// ====================================================================
// Для тестирования: 
// 1. Сохраните код выше в файл 'L1GuidanceController.h'.
// 2. Создайте новый файл 'main.cpp' и скопируйте следующий код.

int main() 
{
    // --- 1. ПАРАМЕТРЫ СИМУЛЯЦИИ ---
    
    // Исходные данные траектории (один сегмент от A до B)
    Point A = {0.0, 0.0};    // Начальная точка отрезка
    Point B = {100.0, 50.0}; // Конечная точка отрезка
    
    // Параметры робота и симуляции
    Point robot_pos = {2.0, -5.0};  // Текущая позиция робота
    double robot_heading = M_PI / 6.0; // Текущий курс (30 градусов)
    double V_cmd = 0.8;             // Линейная скорость V (м/с) - Команда!
    double L_lookahead = 15.0;      // Дистанция упреждения L
    double dt = 0.1;                // Шаг времени симуляции (секунды)

    // --- 2. ИНИЦИАЛИЗАЦИЯ КОНТРОЛЛЕРА ---
    
    // Создание контроллера: (L, W, MaxOmega)
    L1GuidanceController controller(L_lookahead, DISTANCE_WHEELS);
    
    printf("L1 Guidance Controller Test (V=%.2f, L=%.2f, W=%.2f)\n", V_cmd, L_lookahead, DISTANCE_WHEELS);
    printf("----------------------------------------------------------\n");
    
    // --- 3. ЗАПУСК СИМУЛЯЦИИ И ТЕСТИРОВАНИЕ ОБОИХ ВАРИАНТОВ ---

    // Объявляем переменные для вывода результатов
    double omega, VL, VR;
    Point point_D;
    
    // Тест Варианта 1 (Точная геометрия)
    printf("--- TESTING VARIANT 1: Exact Geometry (findNearestPointD_Exact) ---\n");
    for (int step = 0; step < 25; ++step) 
    {
        // 1. Целеуказание: Находим точку D (Точное пересечение)
        point_D = controller.findNearestPointD_Exact(A, B, robot_pos);
        
        // 2. Управление: Расчет омеги и скоростей колес
        controller.calculateControlCommands(robot_pos, robot_heading, point_D, V_cmd, omega, VL, VR);
        
        // 3. Кинематика: Обновляем состояние (для симуляции)
        robot_heading = controller.normalizeAngle(robot_heading + omega * dt); 
        robot_pos.x += V_cmd * std::cos(robot_heading) * dt;
        robot_pos.y += V_cmd * std::sin(robot_heading) * dt;

        // 4. Вывод результатов
        printf("Step %2d | C(%.1f, %.1f) | D(%.1f, %.1f) | Omega=%.2f -> VR=%.2f | VL=%.2f\n", 
               step, robot_pos.x, robot_pos.y, point_D.x, point_D.y, omega, VR, VL);
    }
    
    // Сброс позиции для второго теста
    robot_pos = {2.0, -5.0}; 
    robot_heading = M_PI / 6.0; 

    // Тест Варианта 2 (Проекция + Упреждение)
    printf("--- TESTING VARIANT 2: Projection + Lookahead (findNearestPointD_2var) ---\n");
    for (int step = 0; step < 25; ++step) 
    {
        // 1. Целеуказание: Находим точку D (Проекционный метод)
        point_D = controller.findNearestPointD_2var(A, B, robot_pos);
        
        // 2. Управление: Расчет омеги и скоростей колес
        controller.calculateControlCommands(robot_pos, robot_heading, point_D, V_cmd, omega, VL, VR);
        
        // 3. Кинематика: Обновляем состояние (для симуляции)
        robot_heading = controller.normalizeAngle(robot_heading + omega * dt); 
        robot_pos.x += V_cmd * std::cos(robot_heading) * dt;
        robot_pos.y += V_cmd * std::sin(robot_heading) * dt;

        // 4. Вывод результатов
        printf("Step %2d | C(%.1f, %.1f) | D(%.1f, %.1f) | Omega=%.2f -> VR=%.2f | VL=%.2f\n", 
               step, robot_pos.x, robot_pos.y, point_D.x, point_D.y, omega, VR, VL);
    }

    return 0;
}

// ====================================================================
// HELP ОПИСАНИЕ КЛАССА L1GuidanceController
// ====================================================================

// НАЗНАЧЕНИЕ:
// Класс 'L1GuidanceController' реализует полный цикл управления движением робота 
// с дифференциальным приводом вдоль заданного отрезка траектории (A к B).

// МЕТОДЫ:
// 1. findNearestPointD_Exact(A, B, C): 
//    - Целеуказание (Guidance). 
//    - Находит точку D, которая является ТОЧНЫМ пересечением окружности радиуса L вокруг робота C с отрезком A->B.
//    - Использует квадратное уравнение. Геометрически корректен для L1.

// 2. findNearestPointD_2var(A, B, C): 
//    - Целеуказание (Guidance).
//    - Находит точку D, откладывая L ВДОЛЬ отрезка A->B от проекции робота C.
//    - Проще в расчетах, но D не обязательно на расстоянии L от C.

// 3. calculateControlCommands(C, heading, D, V, omega, VL, VR):
//    - Управление (Control).
//    - Расчет угла ошибки η = angle(C->D) - heading.
//    - L1-Контроллер: Вычисляет угловую скорость ω = (2V/L) * sin(η).
//    - Обратная Кинематика: Конвертирует V и ω в скорости колес VL и VR.
//      VL = V - (ω * W / 2)
//      VR = V + (ω * W / 2)

// ПАРАМЕТРЫ КЛАССА (настраиваются в конструкторе):
// - L (m_lookahead_L): Дистанция упреждения (Lookahead Distance). Влияет на агрессивность поворота.
// - W (m_track_width_W): База робота (Track Width). Расстояние между колесами.
// - MaxOmega (m_max_omega): Ограничение максимальной угловой скорости (Clamp).
*/

#endif