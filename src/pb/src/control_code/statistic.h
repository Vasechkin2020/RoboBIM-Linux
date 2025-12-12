#ifndef SYSTEM_STATISTICS_H
#define SYSTEM_STATISTICS_H

#include <geometry_msgs/Vector3.h> // Для публикации векторов (x,y,z)
#include <vector>                  // Для векторов
#include <cmath>                   // Для математики (hypot, abs)
#include <string>                  // Для строк

class SystemStatistics
{
private:
    // --- Публикаторы ROS ---
    ros::Publisher pub_motion_lin; // Топик для линейного движения (метры)
    ros::Publisher pub_motion_ang; // Топик для углового движения (градусы)
    ros::Publisher pub_drift_lin;  // Топик для линейного дрейфа на стоянке (метры)
    ros::Publisher pub_drift_ang;  // Топик для углового дрейфа на стоянке (градусы)

    bool is_active = false; // Флаг: активна ли статистика (после G10)

    // --- ТЕКУЩЕЕ СОСТОЯНИЕ (Обновляется 1 раз за цикл в main) ---
    SPose curr_odom;  // Текущая одометрия
    SPose curr_model; // Текущая модель
    SPose curr_est;   // Текущая оценка (est)

    // --- ТОЧКИ ДЛЯ РАСЧЕТА ДЕЛЬТ ---
    SPose pose_start_move_odom;  // Позиция Odom в начале движения
    SPose pose_start_move_model; // Позиция Model в начале движения
    SPose pose_start_move_est;   // Позиция Est в начале движения

    SPose pose_finish_prev_odom;  // Позиция Odom в конце ПРЕДЫДУЩЕГО движения (для дрейфа)
    SPose pose_finish_prev_model; // Позиция Model в конце ПРЕДЫДУЩЕГО движения
    SPose pose_finish_prev_est;   // Позиция Est в конце ПРЕДЫДУЩЕГО движения

    // --- НАКОПИТЕЛИ ДЛЯ ОТЧЕТА ---
    double total_theory = 0; // Сумма теоретических длин путей (из G-кода)

    // Суммарное движение (Motion)
    double sum_motion_lin_odom = 0;  // Одометрия
    double sum_motion_lin_model = 0; // Модель
    double sum_motion_lin_est = 0;   // Оценка

    // Суммарный дрейф (Drift - смещение на стоянке)
    double sum_drift_lin_odom = 0;  // Одометрия (должен быть 0)
    double sum_drift_lin_model = 0; // Модель (из-за soft sync)
    double sum_drift_lin_est = 0;   // Оценка (из-за латентности/шума)

    double sum_drift_ang_est = 0; // Накопленный угловой дрейф оценки (для справки)

    // --- ВСПОМОГАТЕЛЬНЫЕ ФУНКЦИИ ---

    // Расчет расстояния между двумя точками (в метрах)
    double get_dist(SPose p1, SPose p2)
    {
        return std::hypot(p1.x - p2.x, p1.y - p2.y); // Корень из суммы квадратов разностей
    }

    // Расчет разницы углов (в градусах по модулю)
    double get_angle_diff(SPose p1, SPose p2)
    {
        double d = p1.th - p2.th; // Разница в радианах
        while (d > M_PI)
            d -= 2 * M_PI; // Нормализация -PI..PI
        while (d <= -M_PI)
            d += 2 * M_PI;                 // Нормализация -PI..PI
        return std::abs(d * 180.0 / M_PI); // Возвращаем модуль в градусах
    }

    // Вывод текущего состояния в лог (для отладки)
    void print_state(const char *label, SPose p)
    {
        logi.log("    %s: x=%+8.3f y=%+8.3f th=%+8.3f\n", label, p.x, p.y, RAD2DEG(p.th)); // Лог позиции
    }

public:
    // Инициализация (вызывать в main после ros::init)
    void init(ros::NodeHandle &nh)
    {
        pub_motion_lin = nh.advertise<geometry_msgs::Vector3>("pb/Control/debug/motion_lin", 1); // Создаем паблишер
        pub_motion_ang = nh.advertise<geometry_msgs::Vector3>("pb/Control/debug/motion_ang", 1); // Создаем паблишер
        pub_drift_lin = nh.advertise<geometry_msgs::Vector3>("pb/Control/debug/drift_lin", 1);   // Создаем паблишер
        pub_drift_ang = nh.advertise<geometry_msgs::Vector3>("pb/Control/debug/drift_ang", 1);   // Создаем паблишер
    }

    // =================================================================================
    // ГЛАВНЫЙ МЕТОД ОБНОВЛЕНИЯ (Вызывать 1 раз в цикле main в начале)
    // =================================================================================
    void update(const pb_msgs::Struct_PoseRotation &msg)
    {
        curr_odom = {msg.x.odom, msg.y.odom, msg.th.odom};     // Сохраняем текущую Odom
        curr_model = {msg.x.model, msg.y.model, msg.th.model}; // Сохраняем текущую Model
        curr_est = {msg.x.est, msg.y.est, msg.th.est};         // Сохраняем текущую Est
    }

    // Сброс и начало сессии (вызывать в case 6 / G10)
    void start_session()
    {
        is_active = true; // Включаем флаг записи

        // Обнуляем все накопители
        total_theory = 0;
        sum_motion_lin_odom = 0;
        sum_motion_lin_model = 0;
        sum_motion_lin_est = 0;
        sum_drift_lin_odom = 0;
        sum_drift_lin_model = 0;
        sum_drift_lin_est = 0;
        sum_drift_ang_est = 0;

        // Запоминаем текущую позицию как базу для первого дрейфа
        pose_finish_prev_odom = curr_odom;
        pose_finish_prev_model = curr_model;
        pose_finish_prev_est = curr_est;
    }

    // НАЧАЛО ДВИЖЕНИЯ (Вызывать в main перед запуском команды движения)
    // Считает, сколько мы сместились, пока стояли (Дрейф)
    void begin_move(int cmd_index)
    {
        if (!is_active)
            return; // Если не активировано G10, выходим

        // 1. Считаем ЛИНЕЙНЫЙ Дрейф (Метры)
        float d_l_o = get_dist(curr_odom, pose_finish_prev_odom);   // Дрейф Odom
        float d_l_m = get_dist(curr_model, pose_finish_prev_model); // Дрейф Model
        float d_l_e = get_dist(curr_est, pose_finish_prev_est);     // Дрейф Est

        // 2. Считаем УГЛОВОЙ Дрейф (Градусы)
        float d_a_o = get_angle_diff(curr_odom, pose_finish_prev_odom);
        float d_a_m = get_angle_diff(curr_model, pose_finish_prev_model);
        float d_a_e = get_angle_diff(curr_est, pose_finish_prev_est);

        // 3. Публикуем в ROS (Метры)
        geometry_msgs::Vector3 msg_l;
        msg_l.x = d_l_o;
        msg_l.y = d_l_m;
        msg_l.z = d_l_e;
        pub_drift_lin.publish(msg_l);

        // 4. Публикуем в ROS (Градусы)
        geometry_msgs::Vector3 msg_a;
        msg_a.x = d_a_o;
        msg_a.y = d_a_m;
        msg_a.z = d_a_e;
        pub_drift_ang.publish(msg_a);

        // 5. Копим сумму
        sum_drift_lin_odom += d_l_o;
        sum_drift_lin_model += d_l_m;
        sum_drift_lin_est += d_l_e;
        sum_drift_ang_est += d_a_e;

        // 6. ЛОГИРУЕМ
        logi.log_b("=== START MOVE [%d] ===\n", cmd_index);
        logi.log(">>> DRIFT (Idle change): Lin Est=%.4fm, Ang Est=%.3f deg\n", d_l_e, d_a_e);
        print_state("START ODOM ", curr_odom);
        print_state("START MODEL", curr_model);
        print_state("START EST  ", curr_est);

        // 7. Запоминаем эту точку как старт активного движения
        pose_start_move_odom = curr_odom;
        pose_start_move_model = curr_model;
        pose_start_move_est = curr_est;
    }

    // КОНЕЦ ДВИЖЕНИЯ (Вызывать в code.h когда speed=0)
    // Считает, сколько мы проехали (Motion)
    // КОНЕЦ ДВИЖЕНИЯ
    void end_move(double theory_len, bool is_rotation) {
        if (!is_active) return;

        if (is_rotation) logi.log_g("=== FINISH ANGLE ===\n");
        else             logi.log_g("=== FINISH VECTOR ===\n");
        
        print_state("END ODOM ", curr_odom);
        print_state("END MODEL", curr_model);
        print_state("END EST  ", curr_est);

        // 1. Считаем ЛИНЕЙНОЕ смещение (для всех режимов!)
        double dist_odom  = get_dist(curr_odom, pose_start_move_odom);
        double dist_model = get_dist(curr_model, pose_start_move_model);
        double dist_est   = get_dist(curr_est, pose_start_move_est);

        // 2. Считаем УГЛОВОЕ смещение
        double ang_odom  = get_angle_diff(curr_odom, pose_start_move_odom);
        double ang_model = get_angle_diff(curr_model, pose_start_move_model);
        double ang_est   = get_angle_diff(curr_est, pose_start_move_est);

        // 3. Обновляем ГЛОБАЛЬНЫЕ НАКОПИТЕЛИ (Суммируем всё!)
        // Даже если мы крутились на месте, линейное смещение - это реальный пробег (ошибка), 
        // его надо добавить в статистику.
        total_theory += theory_len;
        sum_motion_lin_odom  += dist_odom; 
        sum_motion_lin_model += dist_model; 
        sum_motion_lin_est   += dist_est;

        // 4. Логирование и Публикация
        if (is_rotation) {
            // Для поворота главным является угол
            logi.log(">>> DELTA ANGLE: Odom=%.1f, Model=%.1f, Est=%.1f (deg)\n", ang_odom, ang_model, ang_est);
            
            // НО! Мы добавляем вывод линейного смещения (твоя просьба)
            // theory_len для поворота обычно 0.0
            logi.log(">>> LINEAR SHIFT: Theory=%.3f, Odom=%.3f, Model=%.3f, Est=%.3f (m)\n", theory_len, dist_odom, dist_model, dist_est);

            // Публикуем угол в график
            geometry_msgs::Vector3 msg; msg.x = ang_odom; msg.y = ang_model; msg.z = ang_est;
            pub_motion_ang.publish(msg); 
        } 
        else {
            // Для вектора главным является путь
            logi.log(">>> DELTA DIST : Theory=%.3f, Odom=%.3f, Model=%.3f, Est=%.3f (m)\n", theory_len, dist_odom, dist_model, dist_est);
            
            // Публикуем путь в график
            geometry_msgs::Vector3 msg; msg.x = dist_odom; msg.y = dist_model; msg.z = dist_est;
            pub_motion_lin.publish(msg); 
        }

        // Запоминаем финиш
        pose_finish_prev_odom = curr_odom;
        pose_finish_prev_model = curr_model;
        pose_finish_prev_est = curr_est;
    }

    // ПЕЧАТЬ ИТОГОВОГО ОТЧЕТА (В конце работы)
    void print_report()
    {
        if (!is_active)
            return; // Если нет данных, выходим

        // Считаем полные пути (Движение + Дрейф)
        double total_odom = sum_motion_lin_odom + sum_drift_lin_odom;
        double total_model = sum_motion_lin_model + sum_drift_lin_model;
        double total_est = sum_motion_lin_est + sum_drift_lin_est;

        // Лямбда для красивого форматирования с процентами
        auto fmt = [](double val, double total) -> std::string
        {
            double pct = (total > 0.0001) ? (val / total * 100.0) : 0.0;
            char buf[64];
            snprintf(buf, sizeof(buf), "%6.3fm (%5.1f%%)", val, pct);
            return std::string(buf);
        };

        logi.log_g("\n==================================================================================\n");
        logi.log_g("                             FINAL ROUTE STATISTICS                               \n");
        logi.log_g("==================================================================================\n");
        // Шапка таблицы
        logi.log("PARAMETER      | THEORY           | ODOM             | MODEL            | EST              |\n");
        logi.log("---------------+------------------+------------------+------------------+------------------+\n");

        // Строка активного движения
        logi.log("Active Motion  | %s | %s | %s | %s |\n",
                 fmt(total_theory, total_theory).c_str(),
                 fmt(sum_motion_lin_odom, total_odom).c_str(),
                 fmt(sum_motion_lin_model, total_model).c_str(),
                 fmt(sum_motion_lin_est, total_est).c_str());

        // Строка пассивного дрейфа
        logi.log("Passive Drift  | %s | %s | %s | %s |\n",
                 fmt(0.0, total_theory).c_str(),
                 fmt(sum_drift_lin_odom, total_odom).c_str(),
                 fmt(sum_drift_lin_model, total_model).c_str(),
                 fmt(sum_drift_lin_est, total_est).c_str());

        logi.log("---------------+------------------+------------------+------------------+------------------+\n");

        // Строка итогов
        logi.log("TOTAL PATH     | %16.3fm | %16.3fm | %16.3fm | %16.3fm |\n",
                 total_theory, total_odom, total_model, total_est);
        logi.log("----------------------------------------------------------------------------------\n");

        // Расчет финальной ошибки
        double err_est = (total_est - total_theory);
        double err_perc = (total_theory > 0) ? (err_est / total_theory) * 100.0 : 0.0;

        logi.log_w("EST TOTAL LINEAR ERROR: %+.4f m (%+.2f %%)\n", err_est, err_perc);
        logi.log_w("EST TOTAL ANGULAR DRIFT: %.1f deg (Accumulated while standing)\n", sum_drift_ang_est);
        logi.log_g("==================================================================================\n");
    }
};

extern SystemStatistics stats; // Объявляем внешний объект (будет создан в main)

#endif // SYSTEM_STATISTICS_H