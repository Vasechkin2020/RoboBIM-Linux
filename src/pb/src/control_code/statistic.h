#include <geometry_msgs/Vector3.h>
#include <vector>
#include <cmath>
#include <string>

class SystemStatistics
{
private:
    // 4 Топика для PlotJuggler
    ros::Publisher pub_motion_lin; 
    ros::Publisher pub_motion_ang; 
    ros::Publisher pub_drift_lin;  
    ros::Publisher pub_drift_ang;  
    
    bool is_active = false; 

    // Точки
    SPose pose_start_move_odom, pose_start_move_model, pose_start_move_est;       
    SPose pose_finish_prev_odom, pose_finish_prev_model, pose_finish_prev_est;   

    // Накопители (Метры)
    double total_theory = 0;
    double sum_motion_lin_odom = 0, sum_motion_lin_model = 0, sum_motion_lin_est = 0;
    double sum_drift_lin_odom = 0,  sum_drift_lin_model = 0,  sum_drift_lin_est = 0;

    // Накопители (Градусы) - чисто для справки
    double sum_drift_ang_est = 0; 

    // Вспомогательные функции
    double get_dist(SPose p1, SPose p2) {
        return std::hypot(p1.x - p2.x, p1.y - p2.y);
    }
    
    double get_angle_diff(SPose p1, SPose p2) {
        double d = p1.th - p2.th; 
        while (d > M_PI) d -= 2*M_PI;
        while (d <= -M_PI) d += 2*M_PI;
        return std::abs(d * 180.0 / M_PI); // Градусы по модулю
    }

    // Хелпер для лога
    void print_state(const char* label, SPose p) {
        logi.log("    %s: x=%+8.3f y=%+8.3f th=%+8.3f\n", label, p.x, p.y, RAD2DEG(p.th));
    }

public:
    void init(ros::NodeHandle &nh) {
        pub_motion_lin = nh.advertise<geometry_msgs::Vector3>("pb/Control/debug/motion_lin", 1);
        pub_motion_ang = nh.advertise<geometry_msgs::Vector3>("pb/Control/debug/motion_ang", 1);
        pub_drift_lin  = nh.advertise<geometry_msgs::Vector3>("pb/Control/debug/drift_lin", 1);
        pub_drift_ang  = nh.advertise<geometry_msgs::Vector3>("pb/Control/debug/drift_ang", 1);
    }

    void start_session(SPose o, SPose m, SPose e) {
        is_active = true;
        total_theory = 0;
        sum_motion_lin_odom = 0; sum_motion_lin_model = 0; sum_motion_lin_est = 0;
        sum_drift_lin_odom = 0;  sum_drift_lin_model = 0;  sum_drift_lin_est = 0;
        sum_drift_ang_est = 0;
        
        pose_finish_prev_odom = o;
        pose_finish_prev_model = m;
        pose_finish_prev_est = e;
    }

    // =================================================================================
    // ВЫЗЫВАТЬ В MAIN (перед началом движения)
    // =================================================================================
    void begin_move(int cmd_index, SPose curr_o, SPose curr_m, SPose curr_e) {
        if (!is_active) return;

        // 1. Считаем ЛИНЕЙНЫЙ Дрейф (Метры)
        float d_l_o = get_dist(curr_o, pose_finish_prev_odom);
        float d_l_m = get_dist(curr_m, pose_finish_prev_model);
        float d_l_e = get_dist(curr_e, pose_finish_prev_est);

        // 2. Считаем УГЛОВОЙ Дрейф (Градусы)
        float d_a_o = get_angle_diff(curr_o, pose_finish_prev_odom);
        float d_a_m = get_angle_diff(curr_m, pose_finish_prev_model);
        float d_a_e = get_angle_diff(curr_e, pose_finish_prev_est);

        // 3. Публикуем (Метры)
        geometry_msgs::Vector3 msg_l;
        msg_l.x = d_l_o; msg_l.y = d_l_m; msg_l.z = d_l_e;
        pub_drift_lin.publish(msg_l);

        // 4. Публикуем (Градусы)
        geometry_msgs::Vector3 msg_a;
        msg_a.x = d_a_o; msg_a.y = d_a_m; msg_a.z = d_a_e;
        pub_drift_ang.publish(msg_a);

        // 5. Копим
        sum_drift_lin_odom += d_l_o; sum_drift_lin_model += d_l_m; sum_drift_lin_est += d_l_e;
        sum_drift_ang_est  += d_a_e; // Интересен только дрейф оценки угла

        // 6. ЛОГИРУЕМ СТАРТ И ДРЕЙФ
        logi.log_b("=== START MOVE [%d] ===\n", cmd_index);
        logi.log(">>> DRIFT (Idle change): Lin Est=%.4fm, Ang Est=%.3f deg\n", d_l_e, d_a_e);
        print_state("START ODOM ", curr_o);
        print_state("START MODEL", curr_m);
        print_state("START EST  ", curr_e);

        // Запоминаем старт
        pose_start_move_odom = curr_o;
        pose_start_move_model = curr_m;
        pose_start_move_est = curr_e;
    }

    // =================================================================================
    // ВЫЗЫВАТЬ В CODE.H (когда приехали)
    // =================================================================================
    void end_move(SPose curr_o, SPose curr_m, SPose curr_e, double theory_len, bool is_rotation) {
        if (!is_active) return;

        // ЛОГИРУЕМ ФИНИШ (Текущие координаты)
        if (is_rotation) logi.log_g("=== FINISH ANGLE ===\n");
        else             logi.log_g("=== FINISH VECTOR ===\n");
        
        print_state("END ODOM ", curr_o);
        print_state("END MODEL", curr_m);
        print_state("END EST  ", curr_e);

        geometry_msgs::Vector3 msg;

        // Считаем Движение (Delta)
        if (is_rotation) {
            // --- ПОВОРОТ (Градусы) ---
            msg.x = get_angle_diff(curr_o, pose_start_move_odom);
            msg.y = get_angle_diff(curr_m, pose_start_move_model);
            msg.z = get_angle_diff(curr_e, pose_start_move_est);
            pub_motion_ang.publish(msg); 
            
            logi.log(">>> DELTA ANGLE: Odom=%.1f, Model=%.1f, Est=%.1f (deg)\n", msg.x, msg.y, msg.z);
        } else {
            // --- ЛИНЕЙНОЕ (Метры) ---
            msg.x = get_dist(curr_o, pose_start_move_odom);
            msg.y = get_dist(curr_m, pose_start_move_model);
            msg.z = get_dist(curr_e, pose_start_move_est);
            pub_motion_lin.publish(msg); 

            logi.log(">>> DELTA DIST: Theory=%.3f, Odom=%.3f, Model=%.3f, Est=%.3f (m)\n", 
                     theory_len, msg.x, msg.y, msg.z);

            // В итоговую таблицу метров идут только линейные движения
            total_theory += theory_len;
            sum_motion_lin_odom += msg.x; 
            sum_motion_lin_model += msg.y; 
            sum_motion_lin_est += msg.z;
        }

        // Запоминаем финиш
        pose_finish_prev_odom = curr_o;
        pose_finish_prev_model = curr_m;
        pose_finish_prev_est = curr_e;
    }

    void print_report() {
        if (!is_active) return;
        
        double total_odom = sum_motion_lin_odom + sum_drift_lin_odom;
        double total_model = sum_motion_lin_model + sum_drift_lin_model;
        double total_est = sum_motion_lin_est + sum_drift_lin_est;

        auto fmt = [](double val, double total) -> std::string {
            double pct = (total > 0.0001) ? (val / total * 100.0) : 0.0;
            char buf[64];
            snprintf(buf, sizeof(buf), "%6.3fm (%5.1f%%)", val, pct);
            return std::string(buf);
        };

        logi.log_g("\n==================================================================================\n");
        logi.log_g("                             FINAL ROUTE STATISTICS                               \n");
        logi.log_g("==================================================================================\n");
        logi.log("PARAMETER      | THEORY           | ODOM             | MODEL            | EST              |\n");
        logi.log("---------------+------------------+------------------+------------------+------------------+\n");
        
        logi.log("Active Motion  | %s | %s | %s | %s |\n", 
                 fmt(total_theory, total_theory).c_str(), 
                 fmt(sum_motion_lin_odom, total_odom).c_str(), 
                 fmt(sum_motion_lin_model, total_model).c_str(), 
                 fmt(sum_motion_lin_est, total_est).c_str());

        logi.log("Passive Drift  | %s | %s | %s | %s |\n", 
                 fmt(0.0, total_theory).c_str(), 
                 fmt(sum_drift_lin_odom, total_odom).c_str(), 
                 fmt(sum_drift_lin_model, total_model).c_str(), 
                 fmt(sum_drift_lin_est, total_est).c_str());
        
        logi.log("---------------+------------------+------------------+------------------+------------------+\n");
        logi.log("TOTAL PATH     | %16.3fm | %16.3fm | %16.3fm | %16.3fm |\n", 
                 total_theory, total_odom, total_model, total_est);
        logi.log("----------------------------------------------------------------------------------\n");
        
        double err_est = (total_est - total_theory);
        double err_perc = (total_theory > 0) ? (err_est / total_theory) * 100.0 : 0.0;
        logi.log_w("EST TOTAL LINEAR ERROR: %+.4f m (%+.2f %%)\n", err_est, err_perc);
        logi.log_w("EST TOTAL ANGULAR DRIFT: %.1f deg (Accumulated while standing)\n", sum_drift_ang_est);
        logi.log_g("==================================================================================\n");
    }
};

