#pragma once

#include <cmath>     // Для std::sqrt, std::pow, std::sin, std::atan2, std::abs, std::fmod
#include <algorithm> // Для std::max, std::min
#include <cstdio>    // Для использования printf

// Константы
#define M_PI 3.14159265358979323846 // Число Пи

// ====================================================================
// СТРУКТУРЫ
// ====================================================================

// struct SPoint
// {
//     double x;
//     double y;
// };

// ====================================================================
// КЛАСС L1GUIDANCECONTROLLER
// ====================================================================

class L1GuidanceController
{
private:
    // Параметры геометрии
    double m_lookahead_L;        // Дистанция упреждения L
    double m_track_width_W;      // База робота W

    // Параметры ограничений (Абсолютные максимумы)
    double m_max_omega;          // Максимальная угловая скорость
    double m_max_steering_ratio; // Максимальный коэффициент S

    // ====================================================================
    // ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ
    // ====================================================================

    double distanceSq(const SPoint& p1, const SPoint& p2) const
    {
        return std::pow(p1.x - p2.x, 2) + std::pow(p1.y - p2.y, 2);
    }

    double clampT(double t) const
    {
        return std::max(0.0, std::min(t, 1.0));
    }

    double normalizeAngle(double angle_rad) const
    {
        // Нормализация угла в диапазон (-pi, pi]
        angle_rad = std::fmod(angle_rad, 2.0 * M_PI); // Приводим к диапазону (-2pi, 2pi)
        if (angle_rad <= -M_PI) angle_rad += 2.0 * M_PI; // Если меньше -pi, добавляем 2pi
        if (angle_rad > M_PI) angle_rad -= 2.0 * M_PI; // Если больше pi, вычитаем 2pi
        return angle_rad;
    }

    double angleBetweenPoints(const SPoint& C, const SPoint& D) const
    {
        // Вычисление угла вектора CD относительно оси X
        return std::atan2(D.y - C.y, D.x - C.x);
    }

public:
    /**
     * @brief Конструктор.
     * @param L Дистанция упреждения.
     * @param W База робота.
     * @param max_omega Макс. угловая скорость (по умолчанию 5.0).
     * @param max_ratio Макс. коэффициент S (по умолчанию 1.0).
     */
    L1GuidanceController(double L, double W, double max_omega = 5.0, double max_ratio = 1.0)
    {
        m_lookahead_L = L;
        m_track_width_W = W;
        m_max_omega = max_omega;
        m_max_steering_ratio = max_ratio;
    }

    // ====================================================================
    // ПОИСК ТОЧКИ D (Целеуказание)
    // ====================================================================
    
    // ВАРИАНТ 1: Точная геометрия (Рекомендуемый)
    SPoint findNearestPointD_Exact(const SPoint &A, const SPoint &B, const SPoint &C) const
    {
        const double LSq = m_lookahead_L * m_lookahead_L;
        // Если до конца меньше L, возвращаем B (но контроллер все равно сгладит управление)
        if (distanceSq(C, B) <= LSq) { return B; } 

        const double dx = B.x - A.x;
        const double dy = B.y - A.y;
        const double abSq = dx * dx + dy * dy;
        
        if (abSq == 0.0) { return A; }

        const double Ax = A.x - C.x;
        const double Ay = A.y - C.y;
        const double a = abSq;
        const double b = 2.0 * (dx * Ax + dy * Ay);
        const double c = Ax * Ax + Ay * Ay - LSq;
        const double D_disc = b * b - 4.0 * a * c;

        double bestT = -1.0; // Инициализация, выбор MAX t для движения вперед
        if (D_disc >= 0.0) 
        {
            const double sqrtD = std::sqrt(D_disc);
            const double t1 = (-b + sqrtD) / (2.0 * a);
            const double t2 = (-b - sqrtD) / (2.0 * a);
            if (t1 >= 0.0 && t1 <= 1.0) bestT = std::max(bestT, t1); // Берем больший t (дальнюю точку)
            if (t2 >= 0.0 && t2 <= 1.0) bestT = std::max(bestT, t2); // Берем больший t (дальнюю точку)
        }

        if (bestT >= 0.0) 
        {
            return { A.x + bestT * dx, A.y + bestT * dy };
        }

        // Проекция, если нет пересечений
        const double t_proj = -(dx * Ax + dy * Ay) / abSq;
        const double t_clamped = clampT(t_proj);
        return { A.x + t_clamped * dx, A.y + t_clamped * dy };
    }

    // ВАРИАНТ 2: Проекционный
    SPoint findNearestPointD_2var(const SPoint &A, const SPoint &B, const SPoint &C) const
    {
        const double dx = B.x - A.x;
        const double dy = B.y - A.y;
        const double ab_len = std::sqrt(dx*dx + dy*dy);
        if (ab_len < 1e-9) return A;

        const double ux = dx / ab_len;
        const double uy = dy / ab_len;
        const double ACx = C.x - A.x;
        const double ACy = C.y - A.y;

        double t_proj = ACx * ux + ACy * uy;
        if (t_proj < 0.0) t_proj = 0.0;
        if (t_proj > ab_len) t_proj = ab_len;

        double target_dist = t_proj + m_lookahead_L;
        if (target_dist > ab_len) target_dist = ab_len;

        return { A.x + ux * target_dist, A.y + uy * target_dist };
    }

    // ====================================================================
    // УПРАВЛЕНИЕ (ОМЕГА И КОЭФФИЦИЕНТ S) С ЛИНЕЙНЫМ ЗАТУХАНИЕМ
    // ====================================================================

    /**
     * @brief Расчет через УГЛОВУЮ СКОРОСТЬ (Omega).
     * @param B Конечная точка отрезка.
     * @param heading_used_rad Возвращает использованный курс робота (для отладки).
     * @param target_angle_rad Возвращает угол к точке D (для отладки).
     */
    void calculateControlCommands(
        const SPoint& C, double heading_rad, const SPoint& D, const SPoint& B, // Входные данные
        double V,   
        double& omega, double& VL, double& VR, double& angle_error_rad, // Выходные команды (основные)
        double& target_angle_rad, double& heading_used_rad // Выходные данные (для отладки)
    ) const
    {
        // 1. Угол цели и ошибка
        target_angle_rad = angleBetweenPoints(C, D); // Угол от C до D
        heading_used_rad = heading_rad; // Используемый курс (для вывода)
        angle_error_rad = normalizeAngle(target_angle_rad - heading_used_rad); // Нормализованная разница (эталон)
        
        // 2. L1 формула (Сырое значение)
        double omega_raw = 0.0;
        if (m_lookahead_L > 1e-6)
        {
            // V положительный, sin(eta) дает знак.
            omega_raw = (2.0 * V / m_lookahead_L) * std::sin(angle_error_rad);
        }

        // 3. ЛИНЕЙНОЕ ЗАТУХАНИЕ (Fading)
        double dist_to_finish = std::sqrt(distanceSq(C, B)); // Дистанция до финиша (линейная)
        
        // Коэффициент масштаба: 1.0 если далеко, стремится к 0.0 у финиша
        double scale = 1.0;
        if (m_lookahead_L > 1e-6)
        {
            scale = dist_to_finish / m_lookahead_L;
            scale = std::max(0.0, std::min(scale, 1.0)); // Ограничиваем в диапазоне [0.0, 1.0]
        }

        // Динамический лимит
        double current_max_omega = m_max_omega * scale;

        // 4. Ограничение
        omega = omega_raw;
        if (std::abs(omega) > current_max_omega)
        {
            omega = (omega > 0) ? current_max_omega : -current_max_omega;
        }

        // 5. Кинематика
        double rot_comp = omega * m_track_width_W / 2.0;
        VR = V + rot_comp; // Right wheel speed
        VL = V - rot_comp; // Left wheel speed
    }

    /**
     * @brief Расчет через КОЭФФИЦИЕНТ S (Ratio).
     * @param B Конечная точка отрезка.
     * @param heading_used_rad Возвращает использованный курс робота (для отладки).
     * @param target_angle_rad Возвращает угол к точке D (для отладки).
     */
    void calculateControlCommands_Ratio(
        const SPoint& C, double heading_rad, const SPoint& D, const SPoint& B, // Входные данные
        double V,   
        double& S, double& VL, double& VR, double& angle_error_rad, // Выходные команды (основные)
        double& target_angle_rad, double& heading_used_rad // Выходные данные (для отладки)
    ) const
    {
        // 1. Угол цели и ошибка
        target_angle_rad = angleBetweenPoints(C, D); // Угол от C до D
        heading_used_rad = heading_rad; // Используемый курс (для вывода)
        angle_error_rad = normalizeAngle(target_angle_rad - heading_used_rad); // Нормализованная разница (эталон)
        
        // 2. Расчет S (Сырое значение)
        double S_raw = 0.0;
        if (m_lookahead_L > 1e-6)
        {
            S_raw = (m_track_width_W / m_lookahead_L) * std::sin(angle_error_rad);
        }
        
        // 3. ЛИНЕЙНОЕ ЗАТУХАНИЕ (Fading)
        double dist_to_finish = std::sqrt(distanceSq(C, B)); // Дистанция до финиша (линейная)
        
        // Коэффициент масштаба: 1.0 если далеко, стремится к 0.0 у финиша
        double scale = 1.0;
        if (m_lookahead_L > 1e-6)
        {
            scale = dist_to_finish / m_lookahead_L;
            scale = std::max(0.0, std::min(scale, 1.0)); // Ограничиваем в диапазоне [0.0, 1.0]
        }

        // Динамический лимит S
        double current_max_ratio = m_max_steering_ratio * scale;
        
        // 4. Ограничение
        S = S_raw;
        if (std::abs(S) > current_max_ratio)
        {
            S = (S > 0) ? current_max_ratio : -current_max_ratio;
        }

        // 5. Кинематика
        VR = V * (1.0 + S); // Right wheel speed
        VL = V * (1.0 - S); // Left wheel speed
    }

    // ====================================================================
    // СЕТТЕРЫ
    // ====================================================================
    void setLookaheadDistance(double L) { m_lookahead_L = L; }
    void setMaxOmega(double max_omega) { m_max_omega = max_omega; }
    void setMaxSteeringRatio(double max_ratio) { m_max_steering_ratio = max_ratio; }
};