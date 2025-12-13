#ifndef SYSTEM_STATISTICS_H
#define SYSTEM_STATISTICS_H

#include <geometry_msgs/Vector3.h>
#include <vector>
#include <cmath>
#include <string>

// Структура для статистики и отладки
class SystemStatistics
{
private:
    // ... (Публикаторы и переменные те же) ...
    ros::Publisher pub_motion_lin;
    ros::Publisher pub_motion_ang;
    ros::Publisher pub_drift_lin;
    ros::Publisher pub_drift_ang;

    bool is_active = false;

    SPose curr_odom, curr_model, curr_est;
    SPose pose_start_move_odom, pose_start_move_model, pose_start_move_est;
    SPose pose_finish_prev_odom, pose_finish_prev_model, pose_finish_prev_est;

    double total_theory = 0;
    double sum_motion_lin_odom = 0, sum_motion_lin_model = 0, sum_motion_lin_est = 0;
    double sum_drift_lin_odom = 0, sum_drift_lin_model = 0, sum_drift_lin_est = 0;
    double sum_drift_ang_est = 0;

    double get_dist(SPose p1, SPose p2) { return std::hypot(p1.x - p2.x, p1.y - p2.y); }

    double get_angle_diff(SPose p1, SPose p2)
    {
        double d = p1.th - p2.th;
        while (d > M_PI)
            d -= 2 * M_PI;
        while (d <= -M_PI)
            d += 2 * M_PI;
        return std::abs(d * 180.0 / M_PI);
    }

    void print_state(const char *label, SPose p)
    {
        logi.log("    %s: x=%+8.3f y=%+8.3f th=%+8.3f\n", label, p.x, p.y, RAD2DEG(p.th));
    }

    // --- ЕДИНЫЙ ФОРМАТ ВЫВОДА ---
    void print_delta_block(const char *header, double theory_val, bool is_rotation,
                           SPose o_curr, SPose m_curr, SPose e_curr,
                           SPose o_ref, SPose m_ref, SPose e_ref)
    {
        logi.log_b("%s\n", header);

        // 1. Координаты
        print_state("POS ODOM ", o_curr);
        print_state("POS MODEL", m_curr);
        print_state("POS EST  ", e_curr);

        // 2. Дельты (Факт)
        double d_ang_o = get_angle_diff(o_curr, o_ref);
        double d_ang_m = get_angle_diff(m_curr, m_ref);
        double d_ang_e = get_angle_diff(e_curr, e_ref);

        double d_lin_o = get_dist(o_curr, o_ref);
        double d_lin_m = get_dist(m_curr, m_ref);
        double d_lin_e = get_dist(e_curr, e_ref);

        // 3. Теория (распределяем значение в нужную строку)
        double th_ang = is_rotation ? theory_val : 0.0;
        double th_dist = is_rotation ? 0.0 : theory_val;

        // 4. Печать
        logi.log(">>> DELTA ANGLE: Theory=%5.3f, Odom=%5.3f, Model=%5.3f, Est=%5.3f (deg)\n",
                 th_ang, d_ang_o, d_ang_m, d_ang_e);
        logi.log(">>> DELTA DIST : Theory=%5.3f, Odom=%5.3f, Model=%5.3f, Est=%5.3f (m)\n\n",
                 th_dist, d_lin_o, d_lin_m, d_lin_e);
    }

public:
    void init(ros::NodeHandle &nh)
    {
        pub_motion_lin = nh.advertise<geometry_msgs::Vector3>("pb/Control/debug/motion_lin", 1);
        pub_motion_ang = nh.advertise<geometry_msgs::Vector3>("pb/Control/debug/motion_ang", 1);
        pub_drift_lin = nh.advertise<geometry_msgs::Vector3>("pb/Control/debug/drift_lin", 1);
        pub_drift_ang = nh.advertise<geometry_msgs::Vector3>("pb/Control/debug/drift_ang", 1);
    }

    void update(const pb_msgs::Struct_PoseRotation &msg)
    {
        curr_odom = {msg.x.odom, msg.y.odom, msg.th.odom};
        curr_model = {msg.x.model, msg.y.model, msg.th.model};
        curr_est = {msg.x.est, msg.y.est, msg.th.est};
    }

    void start_session()
    {
        is_active = true;
        total_theory = 0;
        sum_motion_lin_odom = 0;
        sum_motion_lin_model = 0;
        sum_motion_lin_est = 0;
        sum_drift_lin_odom = 0;
        sum_drift_lin_model = 0;
        sum_drift_lin_est = 0;
        sum_drift_ang_est = 0;
        pose_finish_prev_odom = curr_odom;
        pose_finish_prev_model = curr_model;
        pose_finish_prev_est = curr_est;
    }

    void begin_move(int cmd_index)
    {
        if (!is_active)
            return;

        // Для дрейфа считаем суммы
        float d_l_o = get_dist(curr_odom, pose_finish_prev_odom);
        float d_l_m = get_dist(curr_model, pose_finish_prev_model);
        float d_l_e = get_dist(curr_est, pose_finish_prev_est);
        float d_a_e = get_angle_diff(curr_est, pose_finish_prev_est);

        geometry_msgs::Vector3 msg_l;
        msg_l.x = d_l_o;
        msg_l.y = d_l_m;
        msg_l.z = d_l_e;
        pub_drift_lin.publish(msg_l);
        geometry_msgs::Vector3 msg_a;
        msg_a.x = get_angle_diff(curr_odom, pose_finish_prev_odom);
        msg_a.y = get_angle_diff(curr_model, pose_finish_prev_model);
        msg_a.z = d_a_e;
        pub_drift_ang.publish(msg_a);

        sum_drift_lin_odom += d_l_o;
        sum_drift_lin_model += d_l_m;
        sum_drift_lin_est += d_l_e;
        sum_drift_ang_est += d_a_e;

        char header[64];
        snprintf(header, sizeof(header), "=== START MOVE [%d] (DRIFT CHECK) ===", cmd_index);

        // Theory для дрейфа всегда 0.0 (мы не должны были двигаться)
        print_delta_block(header, 0.0, false,
                          curr_odom, curr_model, curr_est,
                          pose_finish_prev_odom, pose_finish_prev_model, pose_finish_prev_est);

        pose_start_move_odom = curr_odom;
        pose_start_move_model = curr_model;
        pose_start_move_est = curr_est;
    }

    void end_move(double theory_len, bool is_rotation)
    {
        if (!is_active)
            return;

        double d_l_o = get_dist(curr_odom, pose_start_move_odom);
        double d_l_m = get_dist(curr_model, pose_start_move_model);
        double d_l_e = get_dist(curr_est, pose_start_move_est);

        if (is_rotation)
        {
            geometry_msgs::Vector3 msg;
            msg.x = get_angle_diff(curr_odom, pose_start_move_odom);
            msg.y = get_angle_diff(curr_model, pose_start_move_model);
            msg.z = get_angle_diff(curr_est, pose_start_move_est);
            pub_motion_ang.publish(msg);
        }
        else
        {
            geometry_msgs::Vector3 msg;
            msg.x = d_l_o;
            msg.y = d_l_m;
            msg.z = d_l_e;
            pub_motion_lin.publish(msg);

            total_theory += theory_len;
            sum_motion_lin_odom += d_l_o;
            sum_motion_lin_model += d_l_m;
            sum_motion_lin_est += d_l_e;
        }

        const char *h = is_rotation ? "=== FINISH ANGLE ===" : "=== FINISH VECTOR ===";

        // Передаем theory_len и флаг типа движения
        print_delta_block(h, theory_len, is_rotation,
                          curr_odom, curr_model, curr_est,
                          pose_start_move_odom, pose_start_move_model, pose_start_move_est);

        pose_finish_prev_odom = curr_odom;
        pose_finish_prev_model = curr_model;
        pose_finish_prev_est = curr_est;
    }

    // print_report без изменений...
    void print_report()
    {
        if (!is_active)
            return;
        double total_odom = sum_motion_lin_odom + sum_drift_lin_odom;
        double total_model = sum_motion_lin_model + sum_drift_lin_model;
        double total_est = sum_motion_lin_est + sum_drift_lin_est;
        auto fmt = [](double val, double total) -> std::string
        {
            double pct = (total > 0.0001) ? (val / total * 100.0) : 0.0;
            char buf[64];
            snprintf(buf, sizeof(buf), "%6.3fm (%5.1f%%)", val, pct);
            return std::string(buf);
        };
        logi.log_g("============================================================================================\n");
        logi.log_g("                             FINAL ROUTE STATISTICS                               \n");
        logi.log_g("============================================================================================\n");
        logi.log("PARAMETER      | THEORY           | ODOM             | MODEL            | EST              |\n");
        logi.log("---------------+------------------+------------------+------------------+------------------+\n");
        logi.log("Active Motion  | %s | %s | %s | %s |\n", fmt(total_theory, total_theory).c_str(), fmt(sum_motion_lin_odom, total_odom).c_str(), fmt(sum_motion_lin_model, total_model).c_str(), fmt(sum_motion_lin_est, total_est).c_str());
        logi.log("Passive Drift  | %s | %s | %s | %s |\n", fmt(0.0, total_theory).c_str(), fmt(sum_drift_lin_odom, total_odom).c_str(), fmt(sum_drift_lin_model, total_model).c_str(), fmt(sum_drift_lin_est, total_est).c_str());
        logi.log("---------------+------------------+------------------+------------------+------------------+\n");
        logi.log("TOTAL PATH     | %15.3fm | %15.3fm | %15.3fm | %15.3fm |\n", total_theory, total_odom, total_model, total_est);
        logi.log("-------------------------------------------------------------------------------------------+\n");
        double err_est = (total_est - total_theory);
        double err_perc = (total_theory > 0) ? (err_est / total_theory) * 100.0 : 0.0;
        logi.log_w("EST TOTAL LINEAR ERROR: %+.3f m (%+.2f %%)\n", err_est, err_perc);
        logi.log_w("EST TOTAL ANGULAR DRIFT: %.2f deg\n", sum_drift_ang_est);
        logi.log_g("=============================================================================================\n");
    }
};

extern SystemStatistics stats; // Объявляем внешний объект

#endif // SYSTEM_STATISTICS_H