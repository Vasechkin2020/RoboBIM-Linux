/*  * Версия: 6.0 * Дата: 2025-11-29 */

#include "pillarScan.h"

class AsyncFileLogger; // ВНЕШНЕЕ ОБЪЯВЛЕНИЕ ЛОГГЕРА
extern AsyncFileLogger logi;

// Конструктор
PillarScanNode::PillarScanNode() : new_scan_available_(false),
                                   initialized_(false),
                                   scans_processed_count_(0),
                                   calibration_done_(false),
                                   total_rays_removed_by_zero_intensity(0),
                                   total_rays_removed_by_low_intensity(0),
                                   total_rays_removed_by_initial_intensity(0)
{
    // 1. Старые паблишеры (RVIZ)
    pub_filtered_scan = nh.advertise<visualization_msgs::Marker>("/rviz/filtered_scan", 1);
    pub_method_1 = nh.advertise<visualization_msgs::Marker>("/rviz/method_1_jump", 1);
    pub_method_2 = nh.advertise<visualization_msgs::Marker>("/rviz/method_2_cluster", 1);
    pub_method_3 = nh.advertise<visualization_msgs::Marker>("/rviz/method_3_minima", 1);
    pub_fused_pillars = nh.advertise<visualization_msgs::Marker>("/rviz/fused_pillars", 1);
    pub_final_markers = nh.advertise<visualization_msgs::MarkerArray>("/rviz/final_pillars", 1);

    pub_custom_struct = nh.advertise<pb_msgs::Struct_PoseScan>("/pb/Scan/PoseLidar", 1);

    // 2. Паблишер результата Umeyama (Калибровка)
    pub_calib_result = nh.advertise<geometry_msgs::PoseStamped>("/pb/scan/umeyama_pose", 1);

    // Паблишер результата MNK (Второе мнение)
    pub_mnk_result = nh.advertise<geometry_msgs::PoseStamped>("/pb/scan/mnk_pose", 1);

    // Паблишер результата Fusion (Слияние)
    pub_fused_result = nh.advertise<geometry_msgs::PoseStamped>("/pb/scan/fused_pose", 1);

    // 4. Инициализация решателя MNK
    SPoint start_p = {0.0, 0.0};
    mnk_solver_ = std::make_unique<TrilaterationSolver>(start_p);

    // Инициализация статистики
    stats_.start_time = ros::Time::now();
    stats_.last_print_time = ros::Time::now();
    // Обнуление map
    stats_.missing_counts["RB"] = 0;
    stats_.missing_counts["RT"] = 0;
    stats_.missing_counts["LT"] = 0;
    stats_.missing_counts["LB"] = 0;
}

/*
 * Расчет позиции вторым методом (MNK / Trilateration) для сравнения
 */
void PillarScanNode::performMnkCalculation(const AlignedPillarVector &pillars)
{
    // Нам нужно минимум 3 столба для трилатерации
    if (pillars.size() < 3)
        return;

    // 1. Очистка и подготовка
    mnk_solver_->clear_circles();

    // Обновляем "предыдущую точку" для решателя результатом Umeyama (или предыдущим MNK), чтобы он правильно выбирал дуги.
    if (calibration_done_)
    {
        SPoint prev;
        prev.x = lidar_calibration_.position.x();
        prev.y = lidar_calibration_.position.y();

        logi.log("MNK Init: Using Umeyama Pose (%+8.3f, %+8.3f) as hint.\n", prev.x, prev.y); // Лог для проверки, что мы взяли не (0,0)

        mnk_solver_->set_A_prev(prev);
    }

    int count_circles = 0;

    // 2. Добавляем ДИСТАНЦИИ (4 измерения)
    for (const auto &p : pillars)
    {
        // MNK требует координат центра маяка
        SPoint beacon;
        beacon.x = p.global.x();
        beacon.y = p.global.y();

        // MNK требует расстояние до ЦЕНТРА.
        // У нас есть phys_dist (до поверхности). Прибавляем радиус.
        // Используем взвешенную дистанцию (final_dist), которую мы посчитали в calculatePillarMetrics
        // Восстанавливаем её: L_center = dist(0,0) до local
        double dist_to_center = p.local.norm();

        mnk_solver_->add_circle_from_distance(beacon, dist_to_center, 1.0); // Используем дефолтный вес 1.0 (или явно пишем 1.0)
        count_circles++;
    }

    // --- Б. ДОБАВЛЯЕМ ДАННЫЕ ЛАЗЕРОВ (Вес 20.0 - высокий приоритет) ---
    // (Этот вектор мы заполнили в processLasers только свежими данными)
    for (const auto &las : lasers_for_fusion_)
    {
        mnk_solver_->add_circle_from_distance(las.beacon, las.dist, las.weight);
        count_circles++;
        logi.log("    + Fusion Laser L%d: Dist=%.4f Weight=%.1f\n", las.id, las.dist, las.weight);
    }

    // 3. Добавляем УГЛЫ (C(4,2) = 6 измерений)
    for (size_t i = 0; i < pillars.size(); ++i)
    {
        for (size_t j = i + 1; j < pillars.size(); ++j)
        {
            SPoint p1 = {pillars[i].global.x(), pillars[i].global.y()};
            SPoint p2 = {pillars[j].global.x(), pillars[j].global.y()};

            // Угол между лучами на столбы в системе лидара
            // Используем углы из local координат (это и есть углы, измеренные лидаром)
            double ang_i = std::atan2(pillars[i].local.y(), pillars[i].local.x()) * 180.0 / M_PI;
            double ang_j = std::atan2(pillars[j].local.y(), pillars[j].local.x()) * 180.0 / M_PI;

            // Разница углов (всегда положительная для решателя)
            double angle_deg = std::abs(ang_i - ang_j);
            if (angle_deg > 180.0)
                angle_deg = 360.0 - angle_deg;

            mnk_solver_->add_filtered_circle_from_angle(p1, p2, angle_deg);
            count_circles++;
        }
    }

    // 4. РАСЧЕТ ПОЗИЦИИ (Robust WLS)
    SPoint_Q result = mnk_solver_->find_A_by_mnk_robust();

    // 5. РАСЧЕТ ОРИЕНТАЦИИ
    std::vector<SPoint> beacons;
    std::vector<double> angles;

    for (const auto &p : pillars)
    {
        beacons.push_back({p.global.x(), p.global.y()});
        double ang = std::atan2(p.local.y(), p.local.x()) * 180.0 / M_PI;
        angles.push_back(ang);
    }

    double orientation = mnk_solver_->get_lidar_orientation(result.A, beacons, angles);

    // 6. ПУБЛИКАЦИЯ
    geometry_msgs::PoseStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "laser"; // Global frame

    msg.pose.position.x = result.A.x;
    msg.pose.position.y = result.A.y;

    msg.pose.position.z = result.quality; // Пишем качество (RMS) в Z для отладки // --- ИЗМЕНЕНИЕ: Z = RMSE (качество в метрах) ---

    // Yaw -> Quaternion
    double yaw_rad = orientation * M_PI / 180.0;
    msg.pose.orientation.z = sin(yaw_rad * 0.5);
    msg.pose.orientation.w = cos(yaw_rad * 0.5);

    pub_mnk_result.publish(msg);

    // СОХРАНЯЕМ ДЛЯ FUSION
    mnk_pose_result_ = msg;
    mnk_rmse_result_ = result.quality;
    mnk_yaw_deg_result_ = orientation; // <--- СОХРАНЯЕМ УГОЛ (в градусах)

    // Лог для сравнения (можно закомментить)
    /*
    logi.log("  [MNK COMPARE] X=%.3f Y=%.3f Ang=%.1f (RMS=%.3f)\n", result.A.x, result.A.y, orientation, result.quality);
    */
}

/*
 * Версия: v9.3 (Fusion with Quality Output)
 */
void PillarScanNode::fuseResults()
{
    double rmse_u = lidar_calibration_.rmse;
    double rmse_m = mnk_rmse_result_;

    // // --- СБОР СТАТИСТИКИ (Новый блок) ---
    //         double diff = std::sqrt(std::pow(lidar_calibration_.position.x() - mnk_pose_result_.pose.position.x, 2) +
    //                                 std::pow(lidar_calibration_.position.y() - mnk_pose_result_.pose.position.y, 2));

    //         stats_.sum_mnk_diff += diff;
    //         stats_.sum_mnk_rmse += rmse_m; // Копим ошибку MNK

    //         if (rmse_m < rmse_u) {
    //             stats_.mnk_better_count++; // MNK победил в этом раунде
    //         }
    //         stats_.mnk_count++;

    if (rmse_u < 1e-4)
        rmse_u = 1e-4;
    if (rmse_m < 1e-4)
        rmse_m = 1e-4;

    // Веса (обратная дисперсия)
    double w_u = 1.0 / (rmse_u * rmse_u);
    double w_m = 1.0 / (rmse_m * rmse_m);
    double total_w = w_u + w_m;

    double nw_u = w_u / total_w;
    double nw_m = w_m / total_w;

    // Координаты
    double x_gold = lidar_calibration_.position.x() * nw_u + mnk_pose_result_.pose.position.x * nw_m;
    double y_gold = lidar_calibration_.position.y() * nw_u + mnk_pose_result_.pose.position.y * nw_m;

    // Углы в градусах для твоего сообщения (SScan th)
    double yaw_u_deg = lidar_calibration_.rotation_deg;
    double yaw_m_deg = mnk_yaw_deg_result_; // Мы его сохранили ранее

    // Векторное слияние угла для Fused
    double ang_u_rad = lidar_calibration_.rotation_deg * M_PI / 180.0;
    double ang_m_rad = mnk_yaw_deg_result_ * M_PI / 180.0;
    double sin_sum = nw_u * std::sin(ang_u_rad) + nw_m * std::sin(ang_m_rad);
    double cos_sum = nw_u * std::cos(ang_u_rad) + nw_m * std::cos(ang_m_rad);
    double yaw_gold_rad = std::atan2(sin_sum, cos_sum);
    double yaw_gold_deg = yaw_gold_rad * 180.0 / M_PI;

    // --- ИЗМЕНЕНИЕ: Расчет итогового качества (Theoretical Fused RMSE) ---
    // RMSE_total = 1 / sqrt(W_total)
    double fused_rmse = std::sqrt(1.0 / total_w);

    // Логирование
    logi.log_b("=== 🏆 GOLDEN RESULT ===\n");
    logi.log("  Weights: Umeyama %5.1f%% (Err=%.1fmm) vs MNK %5.1f%% (Err=%.1fmm)\n",
             nw_u * 100.0, rmse_u * 1000.0,
             nw_m * 100.0, rmse_m * 1000.0);
    logi.log_g("  POS: X= %+8.3f Y= %+8.3f | Ang= %+6.2f deg | Qual= %.1f mm\n",
               x_gold, y_gold, yaw_gold_deg, fused_rmse * 1000.0);

    // --- ЗАПОЛНЕНИЕ ТВОЕГО СООБЩЕНИЯ ---
    pb_msgs::Struct_PoseScan custom_msg;

    // Блок X
    custom_msg.x.umeyama = lidar_calibration_.position.x();
    custom_msg.x.mnk = mnk_pose_result_.pose.position.x;

    if (laser_only_result_.valid) // --> ЗАПОЛНЯЕМ ЛАЗЕРЫ
    {
        custom_msg.x.laser = laser_only_result_.x;
    }
    else
    {
        custom_msg.x.laser = 0.0; // Или NaN, если сообщение поддерживает
    }
    custom_msg.x.fused = x_gold;

    // Блок Y
    custom_msg.y.umeyama = lidar_calibration_.position.y();
    custom_msg.y.mnk = mnk_pose_result_.pose.position.y;

    if (laser_only_result_.valid) // --> ЗАПОЛНЯЕМ ЛАЗЕРЫ
    {
        custom_msg.y.laser = laser_only_result_.y;
    }
    else
    {
        custom_msg.y.laser = 0.0;
    }
    custom_msg.y.fused = y_gold;

    // Блок TH (Угол в градусах)
    custom_msg.th.umeyama = yaw_u_deg;
    custom_msg.th.mnk = yaw_m_deg;
    custom_msg.th.laser = yaw_gold_deg; // Лазеры не дают угол, пишем общий, чтобы график был красивый
    custom_msg.th.fused = yaw_gold_deg;

    // Блок RMSE (Качество)
    custom_msg.rmse.umeyama = rmse_u;
    custom_msg.rmse.mnk = rmse_m;

    if (laser_only_result_.valid) // --> ЗАПОЛНЯЕМ ЛАЗЕРЫ
    {
        custom_msg.rmse.laser = laser_only_result_.rmse;
    }
    else
    {
        custom_msg.rmse.laser = 0.0;
    }
    custom_msg.rmse.fused = fused_rmse;

    // Публикация твоего сообщения
    pub_custom_struct.publish(custom_msg);

    // --- СТАТИСТИКА FUSION (v12) ---
    stats_.sum_fused_rmse += fused_rmse;
    if (fused_rmse > stats_.max_fused_rmse)
        stats_.max_fused_rmse = fused_rmse;

    // Статистика сравнения (была ранее)
    double diff = std::sqrt(std::pow(lidar_calibration_.position.x() - mnk_pose_result_.pose.position.x, 2) +
                            std::pow(lidar_calibration_.position.y() - mnk_pose_result_.pose.position.y, 2));
    stats_.sum_mnk_diff += diff;
    stats_.sum_mnk_rmse += rmse_m;
    if (rmse_m < rmse_u)
        stats_.mnk_better_count++;
    stats_.mnk_count++;
    // --------------------------------

    // Публикация
    geometry_msgs::PoseStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "laser";
    msg.pose.position.x = x_gold;
    msg.pose.position.y = y_gold;

    // Пишем итоговое качество в Z
    msg.pose.position.z = fused_rmse;

    msg.pose.orientation.z = sin(yaw_gold_rad * 0.5);
    msg.pose.orientation.w = cos(yaw_gold_rad * 0.5);

    pub_fused_result.publish(msg);
}

// ЛОГИРОВАНИЕ СЫРОГО СКАНА (Адаптировано под прямой вызов)
void PillarScanNode::logRawScan(const sensor_msgs::LaserScan &scan)
{
    // Убрал проверку scans_collected, пишем всегда (будет много логов, как ты просил)
    logi.logf("\n--- RAW SCAN DUMP (Seq: %d) ---\n", scan.header.seq);
    logi.logf("Index | Angle(deg) | Raw_Range(m) | Raw_Intens\n");
    logi.logf("------------------------------------------\n");

    size_t num_rays = scan.ranges.size();
    bool has_intensities = (scan.intensities.size() == num_rays);

    for (size_t i = 0; i < num_rays; ++i)
    {
        double angle = scan.angle_min + i * scan.angle_increment;
        float r = scan.ranges[i];
        float intensity = has_intensities ? scan.intensities[i] : 0.0f; // Берем напрямую

        logi.logf("%5lu | %10.3f | %12.4f | %10.1f\n",
                  i, angle * 180.0 / M_PI, r, intensity);
    }
    logi.logf("--- END RAW SCAN DUMP ---\n");
}

// ЛОГИРОВАНИЕ ОТФИЛЬТРОВАННОГО (Адаптировано)
void PillarScanNode::logFinalFilteredScan(const AlignedVector2f &points, const std::vector<double> &intensities)
{
    logi.logf("\n--- FINAL FILTERED SCAN DUMP (Valid Points) ---\n");
    logi.logf("Index | Angle(deg) | Filtered_Range(m) | Filtered_Intens\n");
    logi.logf("---------------------------------------------------\n");

    for (size_t i = 0; i < points.size(); ++i)
    {
        // Восстанавливаем угол и дальность из координат для лога (чтобы сохранить твой формат)
        double r = points[i].norm();
        double angle = std::atan2(points[i].y(), points[i].x());
        double intens = (i < intensities.size()) ? intensities[i] : 0.0;

        logi.logf("%5lu | %10.3f | %17.4f | %15.1f\n",
                  i, angle * 180.0 / M_PI, r, intens);
    }
    logi.logf("--- END FINAL FILTERED SCAN DUMP ---\n");
}

// НОВАЯ ФУНКЦИЯ: publishMarkerInArray (Без изменений)
void PillarScanNode::publishMarkerInArray(const visualization_msgs::Marker &marker, ros::Publisher &pub)
{
    visualization_msgs::MarkerArray marker_array;
    marker_array.markers.push_back(marker);
    pub.publish(marker_array);
}

// НОВАЯ ФУНКЦИЯ: createClusterMarkers (Без изменений)
visualization_msgs::MarkerArray PillarScanNode::createClusterMarkers(const std::vector<PillarCandidate> &candidates, const std::string &frame_id)
{
    visualization_msgs::MarkerArray marker_array;
    return marker_array;
}

// ИЗМЕНЕНА: createPointsMarker (Без изменений)
visualization_msgs::Marker PillarScanNode::createPointsMarker(const AlignedVector2f &points, const std::string &frame_id, const std::string &ns, int id, float r, float g, float b, float scale)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = ros::Time::now();
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::Marker::POINTS;
    marker.action = visualization_msgs::Marker::ADD;

    marker.scale.x = scale;
    marker.scale.y = scale;

    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 1.0;

    for (const auto &p : points)
    {
        geometry_msgs::Point pt;
        pt.x = p.x();
        pt.y = p.y();
        pt.z = 0.0;
        marker.points.push_back(pt);
    }

    return marker;
}

// Метод для публикации маркеров финальных столбов (Без изменений)
void PillarScanNode::publishFinalMarkers(const AlignedPillarVector &pillars)
{
    if (pillars.empty())
        return;

    visualization_msgs::MarkerArray marker_array;

    std::string laser_frame_id = meta_scan.header.frame_id;

    for (size_t i = 0; i < pillars.size(); ++i)
    {
        const FinalPillar &p = pillars[i];

        Eigen::Vector2d ref = Eigen::Vector2d(0, 0);
        if (p.name == "RB")
            ref = reference_centers_[0].cast<double>();
        else if (p.name == "RT")
            ref = reference_centers_[1].cast<double>();
        else if (p.name == "LT")
            ref = reference_centers_[2].cast<double>();
        else if (p.name == "LB")
            ref = reference_centers_[3].cast<double>();

        // 1. Маркер-сфера
        visualization_msgs::Marker sphere_marker;
        sphere_marker.header.frame_id = laser_frame_id;
        sphere_marker.header.stamp = ros::Time::now();
        sphere_marker.ns = "final_pillars";
        sphere_marker.id = (int)i * 2;
        sphere_marker.type = visualization_msgs::Marker::SPHERE;
        sphere_marker.action = visualization_msgs::Marker::ADD;
        sphere_marker.pose.position.x = p.global.x();
        sphere_marker.pose.position.y = p.global.y();
        sphere_marker.pose.position.z = 0.0;
        sphere_marker.scale.x = pillar_diam_;
        sphere_marker.scale.y = pillar_diam_;
        sphere_marker.scale.z = 0.1;
        sphere_marker.color.a = 0.5;
        sphere_marker.color.r = 0.0;
        sphere_marker.color.g = 0.0;
        sphere_marker.color.b = 1.0;
        marker_array.markers.push_back(sphere_marker);

        // 2. Текстовый маркер
        visualization_msgs::Marker text_marker;
        text_marker.header.frame_id = laser_frame_id;
        text_marker.header.stamp = ros::Time::now();
        text_marker.ns = "pillar_names";
        text_marker.id = (int)i * 2 + 1;
        text_marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        text_marker.action = visualization_msgs::Marker::ADD;
        text_marker.pose.position.x = p.global.x();
        text_marker.pose.position.y = p.global.y();
        text_marker.pose.position.z = 0.5;
        text_marker.scale.z = 0.3;
        text_marker.color.a = 1.0;
        text_marker.color.r = 1.0;
        text_marker.color.g = 1.0;
        text_marker.color.b = 1.0;
        text_marker.text = p.name + "\nRef:(" + std::to_string((int)(ref.x() * 100)) + "," + std::to_string((int)(ref.y() * 100)) + ")";
        marker_array.markers.push_back(text_marker);
    }

    pub_final_markers.publish(marker_array);
}

void PillarScanNode::publishResultsTimerCallback(const ros::TimerEvent &event)
{
    ROS_INFO("+++ publishResultsTimerCallback");

    // 1. Публикация ЧИСТЫХ ТОЧЕК (clean_points_results_) как Marker::POINTS
    if (clean_points_results_.size() > 0 && meta_scan.header.frame_id != "")
    {
        // Создаем маркер POINTS из вектора 2D-точек clean_points_results_
        // Используем pub_filtered_scan, но публикуем Marker вместо LaserScan
        pub_filtered_scan.publish(createPointsMarker(clean_points_results_,
                                                     meta_scan.header.frame_id,                    // Используем frame_id из meta_scan
                                                     "clean_points", 0, 0.7f, 0.7f, 0.7f, 0.05f)); // Серый цвет, мелкие точки
        ROS_INFO("    0 - clean_points_results_ published as Marker::POINTS");
    }
    else if (filtered_scan_results_.ranges.size() > 0)
    {
        // Исходный LaserScan больше не публикуется
        ROS_INFO("      NOTE: filtered_scan_results_ is no longer published as LaserScan.");
    }

    // 2. Публикация маркеров кластеров (РАЗДЕЛЕНА)
    // Метод 1: Jump (Красный)
    if (marker_m1_results_.points.size() > 0)
    {
        marker_m1_results_.header.stamp = ros::Time::now(); // Обновляем время
        pub_method_1.publish(marker_m1_results_);
        ROS_INFO("    1 - marker_m1_results_");
    }
    // Метод 2: Cluster (Синий)
    if (marker_m2_results_.points.size() > 0)
    {
        marker_m2_results_.header.stamp = ros::Time::now();
        pub_method_2.publish(marker_m2_results_);
        ROS_INFO("    2 - marker_m2_results_");
    }
    // Метод 3: Minima (Желтый)
    if (marker_m3_results_.points.size() > 0)
    {
        marker_m3_results_.header.stamp = ros::Time::now();
        pub_method_3.publish(marker_m3_results_);
        ROS_INFO("    3 - marker_m3_results_");
    }

    // 3. Публикация центров Fusion (fused_centers_results_)
    if (fused_centers_results_.size() > 0 && meta_scan.header.frame_id != "")
    {
        // Нужно пересоздать маркер, используя сохраненные данные и frame_id
        pub_fused_pillars.publish(createPointsMarker(fused_centers_results_,
                                                     meta_scan.header.frame_id, // Используем frame_id из meta_scan
                                                     "fused_centers", 4, 0.0f, 1.0f, 0.0f, 0.15f));
        ROS_INFO("    4 - pub_fused_pillars");
    }

    // 4. Публикация финальных откалиброванных маркеров (final_pillars_results_)
    if (!final_pillars_results_.empty())
    {
        publishFinalMarkers(final_pillars_results_);
        ROS_INFO("    5 - publishFinalMarkers");
    }
}

/*
 * Обработка одного кластера: фильтрация, fitCircle, создание кандидата
 */
bool PillarScanNode::processCluster(const AlignedVector2f &cluster, int method_id,
                                    std::vector<PillarCandidate> &out, AlignedVector2f &out_cluster_points)
{
    size_t c_size = cluster.size();

    // 1. Фильтрация по количеству точек
    if (c_size < min_cluster_points_)
    {
        if (method_id == 1)
            stats_.m1_rejected++;
        else if (method_id == 2)
            stats_.m2_rejected++;
        else if (method_id == 3)
            stats_.m3_rejected++;
        return false;
    }

    // 2. Проверка ширины
    double width = MathUtils::dist2D(cluster.front(), cluster.back());

    // 2. Фильтрация по ширине
    if (width < min_cluster_width_ || width > max_cluster_width_)
    {
        if (method_id == 1)
            stats_.m1_rejected++;
        else if (method_id == 2)
            stats_.m2_rejected++;
        else if (method_id == 3)
            stats_.m3_rejected++;
        return false;
    }

    Eigen::Vector2f center;
    double rmse;

    // 3. Аппроксимация круга (используется как предварительный фильтр)
    if (MathUtils::fitCircle(cluster, pillar_radius_, center, rmse))
    {
        double w_rmse = std::max(0.0, 1.0 - (rmse / rmse_max_tolerance));
        double w_n = std::min(1.0, sqrt((double)c_size) / sqrt((double)n_max_points_norm));
        double w_m = w_method[method_id];

        double w_total = w_rmse * w_n * w_m;

        if (w_total > 0.1)
        {
            PillarCandidate cand;
            cand.center = center;
            cand.rmse = rmse;
            cand.num_points = (int)c_size;
            cand.weight = w_total;
            cand.method_id = method_id;

            // --- НОВОЕ: Копируем точки в кандидата ---
            cand.points = cluster;

            out.push_back(cand);

            // Для отрисовки отладочных маркеров
            for (const auto &p : cluster)
            {
                out_cluster_points.push_back(p);
            }
            // Обновляем статистику успеха
            if (method_id == 1)
                stats_.m1_found++;
            else if (method_id == 2)
                stats_.m2_found++;
            else if (method_id == 3)
                stats_.m3_found++;

            return true;
        }
        else // Отказ по низкому весу
        {
            if (method_id == 1)
                stats_.m1_rejected++;
            else if (method_id == 2)
                stats_.m2_rejected++;
            else if (method_id == 3)
                stats_.m3_rejected++;
        }
    }
    else // Отказ: fitCircle не смог построить круг
    {
        if (method_id == 1)
            stats_.m1_rejected++;
        else if (method_id == 2)
            stats_.m2_rejected++;
        else if (method_id == 3)
            stats_.m3_rejected++;
    }
    return false;
}

// НОВЫЙ МЕТОД: Гибридный расчет метрик (Math + Phys)
void PillarScanNode::calculatePillarMetrics(FinalPillar &pillar)
{
    if (pillar.merged_points.empty())
        return;

    // --- ШАГ 1: Глобальный Fit (Math Expert) ---
    if (MathUtils::fitCircle(pillar.merged_points, pillar_radius_, pillar.fit_center, pillar.fit_rmse))
    {
        double sum_r = 0;
        for (const auto &p : pillar.merged_points)
            sum_r += MathUtils::dist2D(p, pillar.fit_center);
        pillar.fit_radius = sum_r / pillar.merged_points.size();

        pillar.math_dist = pillar.fit_center.norm() - pillar_radius_;
        pillar.math_angle = std::atan2(pillar.fit_center.y(), pillar.fit_center.x());
    }
    else
    {
        pillar.fit_center = Eigen::Vector2f(0, 0);
        for (const auto &p : pillar.merged_points)
            pillar.fit_center += p;
        pillar.fit_center /= (float)pillar.merged_points.size();
        pillar.fit_rmse = 999.9;
        pillar.math_dist = pillar.fit_center.norm();
        pillar.math_angle = std::atan2(pillar.fit_center.y(), pillar.fit_center.x());
    }

    // --- ШАГ 2: Физика (Phys Expert) ---
    AlignedVector2f unique_sorted;
    unique_sorted.reserve(pillar.merged_points.size());

    for (const auto &p : pillar.merged_points)
    {
        double d_to_center = MathUtils::dist2D(p, pillar.fit_center);
        if (std::abs(d_to_center - pillar_radius_) > 0.10)
            continue;
        unique_sorted.push_back(p);
    }

    if (unique_sorted.size() > 1)
    {
        std::sort(unique_sorted.begin(), unique_sorted.end(),
                  [](const Eigen::Vector2f &a, const Eigen::Vector2f &b)
                  { return a.norm() < b.norm(); });
        auto last = std::unique(unique_sorted.begin(), unique_sorted.end(),
                                [](const Eigen::Vector2f &a, const Eigen::Vector2f &b)
                                { return (a - b).norm() < 0.001; });
        unique_sorted.erase(last, unique_sorted.end());
    }

    pillar.phys_points_count = unique_sorted.size();

    // Phys_Dist (Медиана)
    if (pillar.phys_points_count >= 1)
    {
        std::vector<double> dists;
        int start_idx = (pillar.phys_points_count > 3) ? 1 : 0;
        int end_idx = std::min((int)pillar.phys_points_count, start_idx + 5);
        for (int i = start_idx; i < end_idx; ++i)
            dists.push_back(unique_sorted[i].norm());
        pillar.phys_dist = MathUtils::getMedian(dists);
    }
    else
    {
        pillar.phys_dist = pillar.math_dist;
    }

    // Phys_Angle (Среднее 10%)
    if (pillar.phys_points_count >= 1)
    {
        int n_angle = std::max(1, (int)(pillar.phys_points_count * 0.10));
        if (n_angle > 10)
            n_angle = 10;
        double sum_ang = 0;
        for (int i = 0; i < n_angle; ++i)
            sum_ang += std::atan2(unique_sorted[i].y(), unique_sorted[i].x());
        pillar.phys_angle = sum_ang / n_angle;
    }
    else
    {
        pillar.phys_angle = pillar.math_angle;
    }

    // --- ШАГ 3: СЛИЯНИЕ (Веса) ---

    double w_fit = 0.5;
    double rmse_score = std::max(0.0, 1.0 - (pillar.fit_rmse / 0.02));
    double radius_err = std::abs(pillar.fit_radius - pillar_radius_);
    double rad_score = std::max(0.0, 1.0 - (radius_err / 0.02));

    w_fit = (rmse_score * 0.6 + rad_score * 0.4);

    // --- СТАТИСТИКА ГИБРИДА ---
    if (w_fit > 0.5)
    {
        stats_.hybrid_math_dominant++;
    }
    else
    {
        stats_.hybrid_phys_dominant++;
    }

    // 1. Финальный Угол
    double final_angle = w_fit * pillar.math_angle + (1.0 - w_fit) * pillar.phys_angle;

    // 2. Финальная Дистанция
    double final_dist = w_fit * pillar.math_dist + (1.0 - w_fit) * pillar.phys_dist;

    // 3. Реконструкция Центра
    float final_r = final_dist + pillar_radius_;
    pillar.local.x() = final_r * std::cos(final_angle);
    pillar.local.y() = final_r * std::sin(final_angle);

    // --- ЛОГИРОВАНИЕ (ПРЯМОЙ ВЫЗОВ LOGI.LOG) ---
    // Формат с пробелами для Excel сохранен
    logi.log("  [%s] W= %0.2f RMSE= %5.3f dR= %+6.3f | Ang: M= %+7.2f P= %+7.2f -> Final= %+7.2f | Dist: M= %5.3f P= %5.3f -> Final= %5.3f\n",
             pillar.name.c_str(),
             w_fit, pillar.fit_rmse, radius_err,
             pillar.math_angle * 180.0 / M_PI, pillar.phys_angle * 180.0 / M_PI, final_angle * 180.0 / M_PI,
             pillar.math_dist, pillar.phys_dist, final_dist);
}

// // ИЗМЕНЕНО: Добавлен подсчет статистики (Total/Rejected/Accepted)
// std::vector<PillarCandidate> PillarScanNode::detectGenericClustering(const AlignedVector2f &pts, double threshold, int method_id,
//                                                      AlignedVector2f &out_cluster_points)
// {
//     std::vector<PillarCandidate> results;
//     if (pts.size() < min_cluster_points_)
//         return results;

//     // 1. Сбор ВСЕХ кластеров (без фильтрации по размеру пока что)
//     std::vector<AlignedVector2f> clusters;
//     AlignedVector2f current;
//     current.reserve(500);

//     // 1. Линейная кластеризация
//     current.push_back(pts[0]);
//     for (size_t i = 1; i < pts.size(); ++i)
//     {
//         if (MathUtils::dist2D(pts[i], pts[i - 1]) > threshold)
//         {
//             if (current.size() >= min_cluster_points_)
//                 clusters.push_back(std::move(current));
//             current.clear();
//         }
//         current.push_back(pts[i]);
//     }
//     if (current.size() >= min_cluster_points_)
//         clusters.push_back(std::move(current));
//     current.clear();

//     // 2. Циклическое замыкание
//     if (clusters.size() >= 2)
//     {
//         const AlignedVector2f first = clusters.front();
//         const AlignedVector2f last = clusters.back();

//         if (MathUtils::dist2D(last.back(), first.front()) < threshold)
//         {
//             AlignedVector2f merged;
//             merged.reserve(last.size() + first.size());
//             merged.insert(merged.end(), last.begin(), last.end());
//             merged.insert(merged.end(), first.begin(), first.end());

//             clusters.erase(clusters.begin());
//             clusters.pop_back();
//             clusters.push_back(std::move(merged));
//         }
//     }

//     // 3. Обработка и статистика
//     int total_clusters = clusters.size();
//     int accepted_count = 0;

//     for (const auto &cluster : clusters)
//     {
//         // Используем bool результат для подсчета
//         if (processCluster(cluster, method_id, results, out_cluster_points))
//         {
//             accepted_count++;
//         }
//     }

//     // Вывод статистики одной строкой
//     if (total_clusters > 0)
//     {
//         logi.log("Method %d Stats: Total Clusters: %d | Rejected: %d | Accepted: %d\n",
//                  method_id, total_clusters, total_clusters - accepted_count, accepted_count);
//     }

//     return results;
// }

/*
 * Версия: v11.0 (Fix: Cyclic Buffer Merge BEFORE filtering)
 */
std::vector<PillarCandidate> PillarScanNode::detectGenericClustering(const AlignedVector2f &pts, double threshold, int method_id,
                                                                     AlignedVector2f &out_cluster_points)
{
    std::vector<PillarCandidate> results;
    if (pts.size() < min_cluster_points_)
        return results; // Если точек меньше минимума для ОДНОГО кластера, искать нечего

    // 1. Сбор ВСЕХ кластеров (без фильтрации по размеру пока что)
    std::vector<AlignedVector2f> raw_clusters;
    AlignedVector2f current;
    current.reserve(100);

    current.push_back(pts[0]);
    for (size_t i = 1; i < pts.size(); ++i)
    {
        // Разрыв дистанции
        if (MathUtils::dist2D(pts[i], pts[i - 1]) > threshold)
        {
            raw_clusters.push_back(std::move(current)); // Сохраняем любой кластер
            current.clear();
        }
        current.push_back(pts[i]);
    }
    // Сохраняем последний
    raw_clusters.push_back(std::move(current));

    // 2. Циклическое замыкание (Suture)
    // Проверяем, являются ли Последний и Первый кластер частями одного целого
    if (raw_clusters.size() >= 2)
    {
        AlignedVector2f &first = raw_clusters.front();
        AlignedVector2f &last = raw_clusters.back();

        // Проверяем расстояние между последней точкой Конца и первой точкой Начала
        if (MathUtils::dist2D(last.back(), first.front()) < threshold)
        {
            // Слияние: Добавляем точки Первого в конец Последнего
            last.insert(last.end(), first.begin(), first.end());

            // Удаляем первый кластер (он теперь часть последнего)
            raw_clusters.erase(raw_clusters.begin());

            // logi.log("Method %d: Cyclic Merge performed! (End+Start)\n", method_id);
        }
    }

    // 3. Финальная обработка и фильтрация
    int total_clusters = raw_clusters.size();
    int accepted_count = 0;

    for (const auto &cluster : raw_clusters)
    {
        // Вот теперь, когда мы склеили края, проверяем размер и ширину
        // Вот теперь проверяем размер, ширину и форму. Если кластер был разбит, теперь он целый и пройдет проверку
        if (processCluster(cluster, method_id, results, out_cluster_points))
        {
            accepted_count++;
        }
    }

    // Статистика (выводим только если что-то нашли)
    /*
    if (total_clusters > 0 && accepted_count > 0) {
        logi.log("Method %d Stats: Raw Clusters: %d | Rejected: %d | Accepted: %d\n",
                 method_id, total_clusters, total_clusters - accepted_count, accepted_count);
    }
    */

    // Вывод статистики одной строкой
    if (total_clusters > 0)
    {
        logi.log("Method %d Stats: Total Clusters: %d | Rejected: %d | Accepted: %d\n",
                 method_id, total_clusters, total_clusters - accepted_count, accepted_count);
    }

    return results;
}

// ИЗМЕНЕНО: Добавлен подсчет статистики (Total/Rejected/Accepted)
std::vector<PillarCandidate> PillarScanNode::detectLocalMinima(const AlignedVector2f &pts, int method_id,
                                                               AlignedVector2f &out_cluster_points)
{
    std::vector<PillarCandidate> results;
    size_t N = pts.size();
    if (N < min_cluster_points_)
        return results;

    const int MAX_SEG_POINTS = 40;
    const float MAX_RADIAL_DEVIATION = 0.10f;
    const float MAX_NEIGHBOR_DIST = 0.10f;
    const float MAX_NEIGHBOR_DIST2 = MAX_NEIGHBOR_DIST * MAX_NEIGHBOR_DIST;

    std::vector<char> processed_flags(N, 0);
    const int WINDOW_SIZE = 5;

    std::vector<float> r_vals(N);
    for (size_t i = 0; i < N; ++i)
        r_vals[i] = pts[i].norm();

    auto dist2_sq = [&](size_t a, size_t b) -> float
    {
        float dx = pts[a].x() - pts[b].x();
        float dy = pts[a].y() - pts[b].y();
        return dx * dx + dy * dy;
    };

    int found_candidates = 0;
    int accepted_candidates = 0;

    for (size_t i = 0; i < N; ++i)
    {
        if (processed_flags[i])
            continue;

        float r = r_vals[i];
        bool is_min = true;

        for (int k = -WINDOW_SIZE; k <= WINDOW_SIZE; ++k)
        {
            if (k == 0)
                continue;
            size_t j = (i + k + N) % N;
            if (r_vals[j] < r)
            {
                is_min = false;
                break;
            }
        }
        if (!is_min)
            continue;

        // Расширение назад
        std::vector<size_t> back_idx;
        back_idx.reserve(MAX_SEG_POINTS);
        int back_count = 0;
        for (int k = -1; k >= -((int)N); --k)
        {
            size_t cur = (i + k + N) % N;
            size_t next = (i + k + 1 + N) % N;
            if (processed_flags[cur])
                break;
            if (++back_count > MAX_SEG_POINTS)
                break;
            if (std::fabs(r_vals[cur] - r) > MAX_RADIAL_DEVIATION)
                break;
            if (dist2_sq(cur, next) > MAX_NEIGHBOR_DIST2)
                break;
            back_idx.push_back(cur);
        }
        std::reverse(back_idx.begin(), back_idx.end());

        // Расширение вперед
        std::vector<size_t> fwd_idx;
        fwd_idx.reserve(MAX_SEG_POINTS);
        int fwd_count = 0;
        for (size_t k = 1; k < N; ++k)
        {
            size_t cur = (i + k) % N;
            size_t prev = (i + k - 1 + N) % N;
            if (processed_flags[cur])
                break;
            if (++fwd_count > MAX_SEG_POINTS)
                break;
            if (std::fabs(r_vals[cur] - r) > MAX_RADIAL_DEVIATION)
                break;
            if (dist2_sq(cur, prev) > MAX_NEIGHBOR_DIST2)
                break;
            fwd_idx.push_back(cur);
        }

        if (back_idx.size() + 1 + fwd_idx.size() > N)
        {
            size_t allowed = (N > 1) ? (N - 1 - back_idx.size()) : 0;
            if (fwd_idx.size() > allowed)
                fwd_idx.resize(allowed);
        }

        AlignedVector2f cluster;
        cluster.reserve(back_idx.size() + 1 + fwd_idx.size());

        for (size_t idx : back_idx)
            cluster.push_back(pts[idx]);
        cluster.push_back(pts[i]);
        for (size_t idx : fwd_idx)
            cluster.push_back(pts[idx]);

        if (cluster.size() >= min_cluster_points_)
        {
            found_candidates++;

            // Используем bool результат
            if (processCluster(cluster, method_id, results, out_cluster_points))
            {
                accepted_candidates++;
            }

            for (size_t idx : back_idx)
                processed_flags[idx] = 1;
            for (size_t idx : fwd_idx)
                processed_flags[idx] = 1;
            processed_flags[i] = 1;
        }
    }

    if (found_candidates > 0)
    {
        logi.log("Method %d Stats: Found Minima: %d | Rejected: %d | Accepted: %d\n",
                 method_id, found_candidates, found_candidates - accepted_candidates, accepted_candidates);
    }

    return results;
}

/*
 * Версия: v10.1 (Fusion: DBSCAN + Point Accumulation + Hard Filtering + Stats)
 * Дата: 2025-12-02
 */
AlignedPillarVector PillarScanNode::fuseCandidates(const std::vector<PillarCandidate> &candidates)
{
    AlignedPillarVector final_pillars;
    if (candidates.empty())
    {
        // logi.log("DBSCAN Fusion skipped: No candidates.\n");
        return final_pillars;
    }

    // logi.log("\n--- DBSCAN Fusion Start: %lu candidates ---\n", candidates.size());

    // Параметры DBSCAN
    const double eps = fusion_group_radius;
    const double eps2 = eps * eps;
    const int minPts = min_dbscan_points_;

    size_t N = candidates.size();
    std::vector<int> labels(N, -1); // -1 = непосещенная, -2 = шум, >=0 = ID кластера
    int clusterId = 0;

    auto dist2 = [&](size_t a, size_t b) // Лямбда для расстояния
    {
        return (candidates[a].center - candidates[b].center).squaredNorm();
    };

    // ---- Фаза 1: Алгоритм DBSCAN (Без изменений) ----
    for (size_t i = 0; i < N; ++i)
    {
        if (labels[i] != -1)
            continue;
        std::vector<size_t> neighbors;
        neighbors.reserve(16);
        for (size_t j = 0; j < N; ++j)
            if (dist2(i, j) <= eps2)
                neighbors.push_back(j);

        if (neighbors.size() < minPts)
        {
            labels[i] = -2; // Шум
            continue;
        }

        labels[i] = clusterId;
        std::deque<size_t> q(neighbors.begin(), neighbors.end());

        while (!q.empty())
        {
            size_t n = q.front();
            q.pop_front();
            if (labels[n] == -2)
                labels[n] = clusterId; // Был шум -> стал край кластера
            if (labels[n] != -1)
                continue;
            labels[n] = clusterId;
            std::vector<size_t> neigh2;
            for (size_t k = 0; k < N; ++k)
                if (dist2(n, k) <= eps2)
                    neigh2.push_back(k);
            if (neigh2.size() >= minPts)
            {
                for (size_t x : neigh2)
                    if (labels[x] == -1 || labels[x] == -2)
                        q.push_back(x);
            }
        }
        clusterId++;
    }
    // Подсчет статистики шума
    int noise_count = 0;
    for (size_t i = 0; i < N; ++i)
    {
        if (labels[i] == -2)
            noise_count++;
    }
    stats_.dbscan_noise_points += noise_count;

    // ---- Фаза 2 & 3: Сборка и ФИЛЬТРАЦИЯ (Изменено) ----

    // Аккумуляторы
    struct Acc
    {
        std::vector<size_t> idxs;
    };
    std::vector<Acc> acc(clusterId);
    for (size_t i = 0; i < N; ++i)
    {
        if (labels[i] >= 0)
            acc[labels[i]].idxs.push_back(i);
    }

    for (int cid = 0; cid < clusterId; ++cid)
    {
        if (acc[cid].idxs.empty())
            continue;

        FinalPillar fp;
        fp.name = "Pillar_" + std::to_string(cid);
        fp.total_weight = 0.0;

        for (size_t k : acc[cid].idxs) // Сбор данных со всех кандидатов кластера
        {
            fp.total_weight += candidates[k].weight;
            // Копируем точки для расчета метрик // ВАЖНО: Сливаем сырые точки всех кандидатов в один вектор
            fp.merged_points.insert(fp.merged_points.end(),
                                    candidates[k].points.begin(),
                                    candidates[k].points.end());
        }

        if (fp.merged_points.empty())
            continue;

        // Считаем метрики (Центр, Радиус, RMSE)
        calculatePillarMetrics(fp);

        // --- ФЕЙС-КОНТРОЛЬ (ГЛАВНОЕ ИЗМЕНЕНИЕ) ---
        // 1. Проверка радиуса: Реальный = 0.1575м.
        // Разрешаем погрешность +/- 8 см. Если больше - это мусор/человек/стена.
        double radius_diff = std::abs(fp.fit_radius - pillar_radius_);

        // 2. Проверка RMSE круга: Если > 5 см - это не круг.

        if (radius_diff > 0.08)
        {
            logi.log_r("  [Filter] Reject Cluster %d: Bad Radius %.3f m (Diff %.3f > 0.08)\n",
                       cid, fp.fit_radius, radius_diff);
            continue; // ПРОПУСКАЕМ ЭТОТ СТОЛБ
        }

        if (fp.fit_rmse > 0.05)
        {
            logi.log_r("  [Filter] Reject Cluster %d: High RMSE %.3f m\n", cid, fp.fit_rmse);
            continue; // ПРОПУСКАЕМ ЭТОТ СТОЛБ
        }

        final_pillars.push_back(fp);
    }

    // ---- Фаза 4: Выбор Top-4 ----
    if (final_pillars.size() > 4)
    {
        std::sort(final_pillars.begin(), final_pillars.end(),
                  [](const FinalPillar &a, const FinalPillar &b)
                  { return a.total_weight > b.total_weight; });
        final_pillars.resize(4);
    }

    // Обновляем статистику сцены
    if (final_pillars.size() == 4)
        stats_.scans_4_pillars++;
    else if (final_pillars.size() == 3)
        stats_.scans_3_pillars++;
    else
        stats_.scans_bad_count++;

    // Логирование результата
    if (!final_pillars.empty())
    {
        logi.log("DBSCAN Result: %lu valid pillars passed filter.\n", final_pillars.size());
    }

    return final_pillars;
}

/*
 * Ротационно-инвариантная сортировка (поддерживает 3 и 4 столба)
 * Для 3 точек: только проверяет геометрическую согласованность
 * Для 4 точек: полная сортировка и переименование
 */
void PillarScanNode::reorderPillars(AlignedPillarVector &pillars)
{
    logi.log("=== reorderPillars (Flexible) ===\n");

    // Общие структуры данных (используются для всех случаев)
    std::vector<std::string> names = {"RB", "RT", "LT", "LB"};
    std::vector<std::pair<int, int>> ref_pairs = {
        {0, 1}, {0, 2}, {0, 3}, {1, 2}, {1, 3}, {2, 3}};

    if (pillars.size() == 4)
    {
        // --- ОРИГИНАЛЬНАЯ ЛОГИКА ДЛЯ 4 ТОЧЕК ---
        logi.log("Processing 4 pillars (full geometric matching)...\n");

        const double ACCEPTABLE_RMSE = 0.05;

        std::vector<int> current_match = {0, 1, 2, 3};
        std::vector<int> best_match(4);
        double min_cost = std::numeric_limits<double>::max();
        bool perfect_match_found = false;

        do
        {
            double current_cost = 0.0;
            for (int k = 0; k < 6; ++k)
            {
                int i_ref = ref_pairs[k].first;
                int j_ref = ref_pairs[k].second;
                int i_pillar_idx = current_match[i_ref];
                int j_pillar_idx = current_match[j_ref];

                double measured_dist = MathUtils::dist2D(pillars[i_pillar_idx].local,
                                                         pillars[j_pillar_idx].local);
                double expected_dist = d_center[k];
                double error = measured_dist - expected_dist;
                current_cost += error * error;
            }

            if (current_cost < 1e-6 && !perfect_match_found)
            {
                logi.log_g("    Perfect match found! Cost=%.2e\n", current_cost);
                perfect_match_found = true;
                best_match = current_match;
                min_cost = current_cost;
                break;
            }

            if (current_cost < min_cost)
            {
                min_cost = current_cost;
                best_match = current_match;
            }

        } while (std::next_permutation(current_match.begin(), current_match.end()));

        double rmse = std::sqrt(min_cost / 6.0);

        if (rmse > ACCEPTABLE_RMSE)
        {
            logi.log_r("WARNING: High matching error (RMSE=%.3f m). Identification may be unreliable.\n", rmse);
        }
        else
        {
            logi.log_g("Geometric matching successful. RMSE=%.3f m\n", rmse);
        }

        // Детальное логирование
        logi.log("Best match configuration analysis:\n");
        for (int k = 0; k < 6; ++k)
        {
            int i_ref = ref_pairs[k].first;
            int j_ref = ref_pairs[k].second;
            int i_actual = best_match[i_ref];
            int j_actual = best_match[j_ref];

            double measured = MathUtils::dist2D(pillars[i_actual].local,
                                                pillars[j_actual].local);
            double expected = d_center[k];
            double error = std::abs(measured - expected);

            if (error <= 0.001)
                logi.log_g("  %s-%s: %.3f (exp: %.3f) ✓ Δ=%.1f mm\n",
                           names[i_ref].c_str(), names[j_ref].c_str(),
                           measured, expected, error * 1000);
            else if (error <= 0.010)
                logi.log_w("  %s-%s: %.3f (exp: %.3f) ~ Δ=%.1f mm\n",
                           names[i_ref].c_str(), names[j_ref].c_str(),
                           measured, expected, error * 1000);
            else
                logi.log_r("  %s-%s: %.3f (exp: %.3f) ✗ Δ=%.1f mm\n",
                           names[i_ref].c_str(), names[j_ref].c_str(),
                           measured, expected, error * 1000);
        }

        // Пересортировка и переименование
        AlignedPillarVector sorted_pillars(4);
        std::string mapping_log = "Pillar mapping result: ";

        for (int i = 0; i < 4; ++i)
        {
            int original_index = best_match[i];
            sorted_pillars[i] = pillars[original_index];
            sorted_pillars[i].name = names[i];
            sorted_pillars[i].ref_index = i;
            mapping_log += names[i] + "←P" + std::to_string(original_index) + " ";
        }

        pillars = sorted_pillars;
        logi.log("%s\n", mapping_log.c_str());
        logi.log_g("Pillars successfully reordered and renamed.\n");

        // Дополнительный лог
        logi.log("Final pillar coordinates:\n");
        for (const auto &pillar : pillars)
        {
            double angle = atan2(pillar.local.y(), pillar.local.x()) * 180.0 / M_PI;
            logi.log("  %s: [%.3f, %.3f] (angle: %.1f°)\n",
                     pillar.name.c_str(), pillar.local.x(), pillar.local.y(), angle);
        }
    }
    else if (pillars.size() == 3)
    {
        // --- НОВАЯ ЛОГИКА ДЛЯ 3 ТОЧЕК ---
        logi.log("Processing 3 pillars (partial geometric validation)...\n");

        // Для 3 точек мы не можем определить какие именно это столбы,
        // но можем проверить геометрическую согласованность

        // 1. Проверить все возможные комбинации (3 из 4 столбов)
        std::vector<std::vector<int>> possible_combinations = {
            {0, 1, 2}, // RB, RT, LT (нет LB)
            {0, 1, 3}, // RB, RT, LB (нет LT)
            {0, 2, 3}, // RB, LT, LB (нет RT)
            {1, 2, 3}  // RT, LT, LB (нет RB)
        };

        std::vector<std::string> comb_names = {
            "RB-RT-LT (no LB)",
            "RB-RT-LB (no LT)",
            "RB-LT-LB (no RT)",
            "RT-LT-LB (no RB)"};

        std::vector<double> combination_errors(4, 0.0);
        std::vector<std::vector<std::pair<int, int>>> combination_pairs = {
            {{0, 1}, {0, 2}, {1, 2}}, // Пары для RB-RT-LT
            {{0, 1}, {0, 3}, {1, 3}}, // Пары для RB-RT-LB
            {{0, 2}, {0, 3}, {2, 3}}, // Пары для RB-LT-LB
            {{1, 2}, {1, 3}, {2, 3}}  // Пары для RT-LT-LB
        };

        // 2. Для каждой возможной комбинации проверяем геометрию
        logi.log("Testing possible pillar combinations:\n");

        for (int comb_idx = 0; comb_idx < 4; ++comb_idx)
        {
            const auto &comb = possible_combinations[comb_idx];
            const auto &pairs = combination_pairs[comb_idx];

            // Перебираем все 6 перестановок 3 измеренных точек
            std::vector<int> perm = {0, 1, 2};
            double best_comb_error = 1e9;

            do
            {
                double current_error = 0.0;

                // Проверяем 3 расстояния для этой перестановки
                for (int p = 0; p < 3; ++p)
                {
                    int i_ref = pairs[p].first;
                    int j_ref = pairs[p].second;

                    // Находим индексы в reference_centers_
                    int ref_i_idx = comb[i_ref];
                    int ref_j_idx = comb[j_ref];

                    // Соответствующие измеренные точки
                    int meas_i_idx = perm[i_ref];
                    int meas_j_idx = perm[j_ref];

                    double measured_dist = MathUtils::dist2D(pillars[meas_i_idx].local,
                                                             pillars[meas_j_idx].local);
                    double expected_dist = 0.0;

                    // Находим какое это расстояние между эталонными столбами
                    for (int k = 0; k < 6; ++k)
                    {
                        if ((ref_pairs[k].first == ref_i_idx && ref_pairs[k].second == ref_j_idx) ||
                            (ref_pairs[k].first == ref_j_idx && ref_pairs[k].second == ref_i_idx))
                        {
                            expected_dist = d_center[k];
                            break;
                        }
                    }

                    double error = measured_dist - expected_dist;
                    current_error += error * error;
                }

                if (current_error < best_comb_error)
                {
                    best_comb_error = current_error;
                }

            } while (std::next_permutation(perm.begin(), perm.end()));

            combination_errors[comb_idx] = std::sqrt(best_comb_error / 3.0);

            // Логирование
            if (combination_errors[comb_idx] <= 0.05)
            { // 5 см
                logi.log_g("  %s: RMSE = %.1f mm ✓\n",
                           comb_names[comb_idx].c_str(),
                           combination_errors[comb_idx] * 1000);
            }
            else
            {
                logi.log_r("  %s: RMSE = %.1f mm ✗\n",
                           comb_names[comb_idx].c_str(),
                           combination_errors[comb_idx] * 1000);
            }
        }

        // 3. Найти лучшую комбинацию
        int best_comb_idx = std::min_element(combination_errors.begin(),
                                             combination_errors.end()) -
                            combination_errors.begin();

        double best_error = combination_errors[best_comb_idx];

        if (best_error <= 0.05)
        { // 5 см порог
            logi.log_g("\n✅ Best combination: %s (RMSE = %.1f mm)\n",
                       comb_names[best_comb_idx].c_str(), best_error * 1000);

            // Сохраняем информацию о том, какие столбы вероятно видны
            // Эта информация будет использована в performCalibrationThreePillars
            const auto &best_comb = possible_combinations[best_comb_idx];

            logi.log("Probable visible pillars: ");
            for (int idx : best_comb)
            {
                logi.log("%s ", PILLAR_NAMES[idx].c_str());
            }
            logi.log("\n");

            // Временно присваиваем имена (могут быть изменены позже)
            for (size_t i = 0; i < pillars.size(); ++i)
            {
                pillars[i].name = "Pillar_" + std::to_string(i);
                pillars[i].ref_index = -1; // Неизвестно точно
            }

            logi.log_w("⚠️  Note: Pillar identification is tentative for 3 points.\n");
            logi.log_w("    Final identification will be done during calibration.\n");
        }
        else
        {
            logi.log_r("\n❌ No geometrically consistent combination found (min RMSE = %.1f mm)\n",
                       best_error * 1000);
            logi.log_r("    Pillar identification may be unreliable.\n");

            // Все равно присваиваем временные имена
            for (size_t i = 0; i < pillars.size(); ++i)
            {
                pillars[i].name = "Pillar_" + std::to_string(i);
                pillars[i].ref_index = -1;
            }
        }
    }
    else if (pillars.size() == 2)
    {
        // --- МИНИМАЛЬНАЯ ОБРАБОТКА ДЛЯ 2 ТОЧЕК ---
        logi.log_w("Only 2 pillars detected - minimal processing.\n");

        // Просто присваиваем временные имена
        for (size_t i = 0; i < pillars.size(); ++i)
        {
            pillars[i].name = "Pillar_" + std::to_string(i);
            pillars[i].ref_index = -1;
        }

        // Проверяем расстояние между ними
        double dist = MathUtils::dist2D(pillars[0].local, pillars[1].local);
        logi.log("Distance between pillars: %.3f m\n", dist);

        // Сравниваем с возможными эталонными расстояниями
        bool matches_any = false;
        for (int k = 0; k < 6; ++k)
        {
            if (std::abs(dist - d_center[k]) < 0.1)
            { // 10 см порог
                matches_any = true;
                break;
            }
        }

        if (matches_any)
        {
            logi.log_g("  Distance is plausible for pillar pair ✓\n");
        }
        else
        {
            logi.log_r("  Distance does not match any expected pillar pair ✗\n");
        }
    }
    else
    {
        // --- ОБРАБОТКА ДРУГИХ СЛУЧАЕВ ---
        logi.log_r("Unexpected number of pillars: %lu\n", pillars.size());

        for (size_t i = 0; i < pillars.size(); ++i)
        {
            pillars[i].name = "Pillar_" + std::to_string(i);
            pillars[i].ref_index = -1;
        }
    }

    logi.log("=== reorderPillars complete ===\n");
}

/* * Преобразование координат из системы лидара в мировую систему с применением калибровочных параметров */
Eigen::Vector2d PillarScanNode::lidarToWorld(const Eigen::Vector2d &lidar_point)
{
    if (!calibration_done_)
    {
        logi.log_r("Калибровка не выполнена, преобразование невозможно\n");
        return lidar_point;
    }

    return lidar_calibration_.scale_factor *
               lidar_calibration_.rotation_matrix *
               lidar_point +
           lidar_calibration_.position;
}

/* * Преобразование координат из мировой системы в систему лидара * (обратное преобразование) */
Eigen::Vector2d PillarScanNode::worldToLidar(const Eigen::Vector2d &world_point)
{
    if (!calibration_done_)
    {
        logi.log_r("Калибровка не выполнена, преобразование невозможно\n");
        return world_point;
    }

    // Обратное преобразование: P = (1/c) * Rᵀ * (Q - T)
    Eigen::Matrix2d R_inv = lidar_calibration_.rotation_matrix.transpose();
    double c_inv = 1.0 / lidar_calibration_.scale_factor;

    return c_inv * R_inv * (world_point - lidar_calibration_.position);
}

void PillarScanNode::saveResults(const AlignedPillarVector &pillars)
{
    logi.log("--- 💾 СОХРАНЕНИЕ РЕЗУЛЬТАТОВ:\n");

    // 1. Сохраняем идеальные координаты столбов (для обратной совместимости)
    for (const auto &p : pillars)
    {
        if (p.name.find("Pillar_") != std::string::npos)
            continue;

        std::string base = "/pb/scan/result/" + p.name;
        nh.setParam(base + "/x", p.global.x());
        nh.setParam(base + "/y", p.global.y());

        logi.log("  %s: [%.6f, %.6f]\n",
                 p.name.c_str(), p.global.x(), p.global.y());
    }
    logi.log_g("✅ РЕЗУЛЬТАТЫ СОХРАНЕНЫ\n");
}
// Удаление фантомных точек (хвостов) с помощью углового фильтра
AlignedVector2f PillarScanNode::removeEdgeArtifacts(const AlignedVector2f &points, const std::vector<double> &intensities, int &points_removed_by_angle_filter)
{
    logi.log("--- removeEdgeArtifacts ---\n");
    // Очищаем счетчик удаленных точек
    points_removed_by_angle_filter = 0;

    // Получаем общее количество точек до фильтрации
    size_t initial_point_count = points.size();

    // Проверка на минимальный размер скана
    if (initial_point_count < 2)
    {
        return points;
    }

    AlignedVector2f clean_points;                            // Вектор для хранения очищенных точек
    const double ANGLE_THRESHOLD_RAD = edge_angle_threshold; // Порог угла в радианах (берется из params.yaml)

    // ВРЕМЕННЫЙ ЛОГ ЗАГОЛОВКА (v5.5)
    // logi.log("\n--- DETAILED ANGLE FILTER DEBUG LOG (Angle Threshold: %.1f deg) ---\n", edge_angle_threshold * 180.0 / M_PI);
    // logi.log("P_IDX | P_Curr_X | P_Curr_Y | Lidar_Ang | P_Next_X | P_Next_Y | Angle(deg) | ABS_Check_Ang | Decision\n");
    // logi.log("---------------------------------------------------------------------------------------------------\n");

    // Всегда добавляем первую точку (points[0])
    clean_points.push_back(points[0]);

    // Итерируем до предпоследней точки (points[N-2])
    for (size_t i = 0; i < points.size() - 1; ++i)
    {
        const Eigen::Vector2f &P_curr = points[i];     // Текущая точка
        const Eigen::Vector2f &P_next = points[i + 1]; // Следующая точка

        Eigen::Vector2f V_ray = P_curr;               // Вектор луча (от (0,0) до P_curr)
        Eigen::Vector2f V_seg_next = P_next - P_curr; // Вектор сегмента P_curr -> P_next
        double angle_rad = M_PI;                      // Исходный угол [0, 180]
        double angle_check = M_PI / 2.0;              // Угол для проверки параллельности [0, 90]

        // Расчет угла между V_ray и V_seg_next
        if (V_ray.norm() > 0.001 && V_seg_next.norm() > 0.001)
        {
            double dot_prod = V_ray.normalized().dot(V_seg_next.normalized());
            angle_rad = std::acos(std::max(-1.0, std::min(dot_prod, 1.0)));

            // Расчет угла, насколько сегмент параллелен лучу, независимо от направления (v5.3)
            // Это решает проблему симметрии 0 градусов и 180 градусов
            angle_check = std::min(angle_rad, M_PI - angle_rad);
        }

        double angle_deg = angle_rad * 180.0 / M_PI;
        double angle_check_deg = angle_check * 180.0 / M_PI;

        // Расчет абсолютного угла лидара для P_curr (v5.4)
        double lidar_angle_rad = std::atan2(P_curr.y(), P_curr.x());
        double lidar_angle_deg = lidar_angle_rad * 180.0 / M_PI;

        // --- УСЛОВИЕ ФАНТОМА (v5.3) ---
        // Если угол параллельности меньше порога
        if (std::abs(angle_check) < ANGLE_THRESHOLD_RAD)
        {
            points_removed_by_angle_filter++;

            // ВРЕМЕННЫЙ ЛОГ ТЕКУЩЕЙ ИТЕРАЦИИ (v5.5)
            // logi.log("%5lu | %8.3f | %8.3f | %9.3f | %8.3f | %8.3f | %10.3f | %13.3f | %s\n",
            //          i + 1, P_curr.x(), P_curr.y(), lidar_angle_deg, P_next.x(), P_next.y(),
            //          angle_deg, angle_check_deg,
            //          "REMOVED_P_NEXT");

            // Пропускаем P_next (удаляем ее)
            i++;
            continue;
        }

        // Если условие не сработало, P_next — валидная точка, добавляем ее.
        clean_points.push_back(P_next);

        // ВРЕМЕННЫЙ ЛОГ ТЕКУЩЕЙ ИТЕРАЦИИ (v5.5)
        // logi.log("%5lu | %8.3f | %8.3f | %9.3f | %8.3f | %8.3f | %10.3f | %13.3f | %s\n",
        //          i + 1, P_curr.x(), P_curr.y(), lidar_angle_deg, P_next.x(), P_next.y(),
        //          angle_deg, angle_check_deg,
        //          "KEPT_P_NEXT");
    }

    // logi.log("--- END DETAILED ANGLE FILTER DEBUG LOG ---\n");

    // Вывод статистики в конце функции (v5.5)
    size_t final_point_count = clean_points.size();

    logi.log_w("ANGLE FILTER STATS: Initial points: %lu. Removed: %d. Final points: %lu.\n",
               initial_point_count, points_removed_by_angle_filter, final_point_count);

    return clean_points;
}

// Вместо медленного setParam публикуем быстрый топик
void PillarScanNode::saveCalibrationParameters()
{
    if (!calibration_done_)
        return;

    geometry_msgs::PoseStamped msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "laser"; // Или "world", или "odom" - твоя глобальная система

    // 1. Позиция (X, Y)
    msg.pose.position.x = lidar_calibration_.position.x();
    msg.pose.position.y = lidar_calibration_.position.y();

    // --- ИЗМЕНЕНИЕ: Z = RMSE (качество в метрах) ---
    msg.pose.position.z = lidar_calibration_.rmse;

    // 2. Ориентация (Конвертация Yaw -> Quaternion)
    // Угол у нас в радианах: lidar_calibration_.rotation_deg * M_PI / 180.0
    double yaw_rad = lidar_calibration_.rotation_deg * M_PI / 180.0;

    // Формула Эйлера в Кватернион (только вокруг Z)
    msg.pose.orientation.x = 0.0;
    msg.pose.orientation.y = 0.0;
    msg.pose.orientation.z = sin(yaw_rad * 0.5);
    msg.pose.orientation.w = cos(yaw_rad * 0.5);

    // 3. Публикация (Мгновенно)
    pub_calib_result.publish(msg);

    // Старые setParam удалены для скорости.
    // Если очень нужно сохранять в параметры (редко), можно добавить счетчик
    // и делать это раз в 100 циклов. Но для 10Гц лучше топик.

    logi.log_g("    ✅ Calibration parameters saved to ROS Parameter Server\n");
}

/*
 * Универсальная калибровка (Маршрутизатор) - ТЕПЕРЬ ВОЗВРАЩАЕТ BOOL
 */
// bool PillarScanNode::performCalibration(AlignedPillarVector &pillars)
// {
//     if (pillars.size() == 4)
//     {
//         // Стандартный случай
//         return performCalibrationFourPillars(pillars);
//     }
//     else if (pillars.size() == 3)
//     {
//         // Сложный случай (Треугольник)
//         logi.log_w("⚠️ 3 pillars detected -> Calling 3-PT Calibration\n");
//         return performCalibrationThreePillars(pillars);
//     }
//     else if (pillars.size() > 4)
//     {
//         // Избыточность
//         logi.log_w("⚠️ %lu pillars detected -> Selecting Best 4\n", pillars.size());
//         selectBestFourPillars(pillars);

//         if (pillars.size() == 4)
//         {
//             return performCalibrationFourPillars(pillars);
//         }
//         else
//         {
//             logi.log_r("❌ Failed to select pillars\n");
//             lidar_calibration_.clear();
//             return false;
//         }
//     }
//     else
//     {
//         // Мало данных
//         logi.log_r("❌ Insufficient pillars: %lu (need 3 or 4)\n", pillars.size());
//         lidar_calibration_.clear();
//         return false;
//     }
// }

/*
 * Универсальная калибровка с попыткой восстановления при сбоях
 * Версия: v10.2 (Auto-Recovery / RANSAC with Stats)
 */
bool PillarScanNode::performCalibration(AlignedPillarVector &pillars)
{
    // 1. Если видим 4 столба - пробуем идеальный вариант
    if (pillars.size() == 4)
    {
        // Создаем копию, чтобы 4-точечный метод не испортил исходные данные при неудаче
        // (хотя он меняет только локальные имена, но лучше перестраховаться)
        AlignedPillarVector pillars_copy = pillars;

        if (performCalibrationFourPillars(pillars_copy))
        {
            pillars = pillars_copy; // Применяем успех
            return true;
        }

        logi.log_w("⚠️ 4-Point Algo Failed. Trying subsets (3-Point RANSAC)...\n");

        // --- СПАСАТЕЛЬНЫЙ КРУГ: Перебор подмножеств из 3-х точек ---
        // У нас есть [0, 1, 2, 3]. Пробуем выкинуть каждый по очереди.

        for (int i = 0; i < 4; ++i)
        {
            // Создаем набор из 3-х столбов, исключая i-й
            AlignedPillarVector subset;
            for (int j = 0; j < 4; ++j)
            {
                if (i == j)
                    continue;
                subset.push_back(pillars[j]);
            }

            // Пробуем откалиброваться по этому треугольнику
            // (performCalibrationThreePillars сама внутри делает перебор гипотез)
            if (performCalibrationThreePillars(subset))
            {
                logi.log_g("✅ RANSAC Recovered! Used subset excluding index %d\n", i);

                // --- СТАТИСТИКА: УРА, СПАСЛИСЬ! ---
                stats_.mode_4pt_ransac++;
                // Заметь: внутри performCalibrationThreePillars уже увеличился
                // счетчик mode_3pt и calib_success. Это нормально.
                // mode_4pt_ransac покажет именно факт "переключения" логики.
                // ----------------------------------

                // Важно: subset теперь содержит 4 столба (3 измеренных + 1 восстановленный)
                // Обновляем основной вектор pillars
                pillars = subset;
                return true;
            }
        }

        logi.log_r("❌ RANSAC Failed. Data is too noisy.\n");
        lidar_calibration_.clear();
        return false;
    }

    // 2. Если видим 3 столба
    else if (pillars.size() == 3)
    {
        return performCalibrationThreePillars(pillars);
    }

    // 3. Если видим > 4 (избыточность)
    else if (pillars.size() > 4)
    {
        logi.log_w("⚠️ %lu pillars detected -> Selecting Best 4\n", pillars.size());
        selectBestFourPillars(pillars);

        // Рекурсивный вызов себя же, чтобы пройти логику пункта 1 (4 точки + RANSAC)
        return performCalibration(pillars);
    }

    else
    {
        logi.log_r("❌ Insufficient pillars: %lu (need 3 or 4)\n", pillars.size());
        lidar_calibration_.clear();
        stats_.calib_fail++; // Тоже считаем как провал
        return false;
    }
}

/*
 * Версия: v10.2 (Calibration 4-PT with Statistics)
 * Дата: 2025-12-02
 */
bool PillarScanNode::performCalibrationFourPillars(AlignedPillarVector &pillars)
{
    // logi.log("=== 4-PILLAR CALIBRATION ===\n");

    if (pillars.size() != 4)
    {
        logi.log_r("Error: Expected 4 pillars, got %lu\n", pillars.size());
        return false;
    }

    // 1. ПОДГОТОВКА МАТРИЦ
    Eigen::Matrix2Xd P(2, 4), Q(2, 4);
    for (int i = 0; i < 4; ++i)
    {
        P.col(i) = pillars[i].local.cast<double>();
        Q.col(i) = reference_centers_[i].cast<double>();
    }

    // 2. ЦЕНТРОИДЫ И ЦЕНТРИРОВАНИЕ
    Eigen::Vector2d mu_P = P.rowwise().mean();
    Eigen::Vector2d mu_Q = Q.rowwise().mean();
    Eigen::Matrix2Xd P_centered = P.colwise() - mu_P;
    Eigen::Matrix2Xd Q_centered = Q.colwise() - mu_Q;

    // 3. SVD И РАСЧЕТ ПАРАМЕТРОВ (UMEYAMA)
    Eigen::Matrix2d H = P_centered * Q_centered.transpose();
    Eigen::JacobiSVD<Eigen::Matrix2d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
    Eigen::Matrix2d U = svd.matrixU();
    Eigen::Matrix2d V = svd.matrixV();

    Eigen::Matrix2d R = V * U.transpose();
    if (R.determinant() < 0)
    {
        Eigen::Matrix2d D = Eigen::Matrix2d::Identity();
        D(1, 1) = -1.0;
        R = V * D * U.transpose();
    }

    double denominator = P_centered.squaredNorm();
    if (denominator < 1e-12)
        return false;
    double c = svd.singularValues().sum() / denominator;
    Eigen::Vector2d T = mu_Q - c * R * mu_P;

    // 4. СОХРАНЕНИЕ В СТРУКТУРУ
    lidar_calibration_.scale_factor = c;
    double rotation_rad = atan2(R(1, 0), R(0, 0));
    lidar_calibration_.rotation_deg = rotation_rad * 180.0 / M_PI;
    lidar_calibration_.rotation_matrix = R;
    lidar_calibration_.position = T;

    // 5. ЛОГИРОВАНИЕ ПАРАМЕТРОВ (Компактно)
    logi.log("--- CALIBRATION RESULTS ---\n");
    logi.log("  Scale: %.6f (Err: %+.2f%%) | Angle: %.2f°\n", c, (c - 1.0) * 100.0, lidar_calibration_.rotation_deg);
    logi.log("  Pos:   X=%.4f m, Y=%.4f m (Dist from origin: %.3f m)\n", T.x(), T.y(), T.norm());

    // 6. ВАЛИДАЦИЯ И ОШИБКИ
    logi.log("--- VALIDATION ---\n");
    double max_error = 0.0;
    double total_error_sq = 0.0;

    for (int i = 0; i < 4; ++i)
    {
        Eigen::Vector2d p = P.col(i);
        Eigen::Vector2d q_exp = Q.col(i);
        Eigen::Vector2d q_calc = c * R * p + T;
        double err = (q_calc - q_exp).norm();

        total_error_sq += err * err;
        if (err > max_error)
            max_error = err;

        // Вывод одной строкой: Имя | Измерено -> Трансформировано -> Эталон | Ошибка
        char status = (err <= 0.001) ? '+' : (err <= 0.010 ? '~' : '!');
        logi.log("  %s: Meas[%+8.3f,%+8.3f] -> Trans[%+8.3f,%+8.3f] (Ref[%+8.3f,%+8.3f]) | Err: %+7.2f mm %c\n",
                 pillars[i].name.c_str(), p.x(), p.y(), q_calc.x(), q_calc.y(), q_exp.x(), q_exp.y(), err * 1000, status);
    }

    double rmse = std::sqrt(total_error_sq / 4.0);
    const double RMSE_THRESHOLD = 0.05;

    if (rmse <= RMSE_THRESHOLD)
    {
        calibration_done_ = true;
        lidar_calibration_.rmse = rmse; // <--- ДОБАВИТЬ ЭТУ СТРОКУ
        for (int i = 0; i < 4; ++i)
            pillars[i].global = reference_centers_[i].cast<double>();
        final_pillars_results_ = pillars;

        // --- СТАТИСТИКА УСПЕХА ---
        stats_.calib_success++;
        stats_.mode_4pt_perfect++; // Идеальный режим
        stats_.sum_rmse += rmse;
        if (rmse > stats_.max_rmse)
            stats_.max_rmse = rmse;
        // -------------------------

        logi.log_g("✅ SUCCESS! RMSE=%+6.1f mm (Max=%+6.1f mm)\n", rmse * 1000, max_error * 1000);
        saveCalibrationParameters();
        publishFinalMarkers(final_pillars_results_);
        return true;
    }
    else
    {
        // --- СТАТИСТИКА ПРОВАЛА ---
        stats_.calib_fail++;
        // --------------------------
        logi.log_r("❌ FAILED: RMSE=%.1f mm > %.1f mm\n", rmse * 1000, RMSE_THRESHOLD * 1000);
        lidar_calibration_.clear();
        return false;
    }
}
/*
 * Версия: v10.2 (Calibration 3-PT with Statistics)
 * Дата: 2025-12-02
 */
bool PillarScanNode::performCalibrationThreePillars(AlignedPillarVector &pillars)
{
    logi.log("\n=== 3-PILLAR CALIBRATION (Triangle Matching) ===\n");

    if (pillars.size() != 3)
    {
        logi.log_r("Error: Expected 3 pillars, got %lu\n", pillars.size());
        return false;
    }

    const std::vector<std::string> ref_names = {"RB", "RT", "LT", "LB"};

    double best_rmse = 999.9;
    std::vector<int> best_mapping(3);
    Eigen::Matrix2d best_R;
    Eigen::Vector2d best_T;
    double best_scale = 1.0;
    int best_missing_idx = -1;

    // 1. ПЕРЕБОР ГИПОТЕЗ (Какой столб отсутствует?)
    for (int missing_idx = 0; missing_idx < 4; ++missing_idx)
    {
        std::vector<int> ref_indices;
        for (int i = 0; i < 4; ++i)
            if (i != missing_idx)
                ref_indices.push_back(i);

        std::vector<int> p_indices = {0, 1, 2};

        do
        {
            // А. ГЕОМЕТРИЧЕСКАЯ ПРОВЕРКА
            double geom_error_sq = 0.0;
            bool geom_fail = false;

            for (int k = 0; k < 3; ++k)
            {
                int idx_A = k;
                int idx_B = (k + 1) % 3;
                double d_meas = MathUtils::dist2D(pillars[p_indices[idx_A]].local, pillars[p_indices[idx_B]].local);
                double d_ref = MathUtils::dist2D(reference_centers_[ref_indices[idx_A]], reference_centers_[ref_indices[idx_B]]);
                double diff = std::abs(d_meas - d_ref);

                if (diff > 0.15)
                {
                    geom_fail = true;
                    break;
                }
                geom_error_sq += diff * diff;
            }

            if (geom_fail)
                continue;

            // Б. РЕШЕНИЕ SVD
            Eigen::Matrix2Xd P_mat(2, 3), Q_mat(2, 3);
            for (int i = 0; i < 3; ++i)
            {
                P_mat.col(i) = pillars[p_indices[i]].local.cast<double>();
                Q_mat.col(i) = reference_centers_[ref_indices[i]].cast<double>();
            }

            Eigen::Vector2d mu_P = P_mat.rowwise().mean();
            Eigen::Vector2d mu_Q = Q_mat.rowwise().mean();
            Eigen::Matrix2Xd P_centered = P_mat.colwise() - mu_P;
            Eigen::Matrix2Xd Q_centered = Q_mat.colwise() - mu_Q;

            Eigen::Matrix2d H = P_centered * Q_centered.transpose();
            Eigen::JacobiSVD<Eigen::Matrix2d> svd(H, Eigen::ComputeFullU | Eigen::ComputeFullV);
            Eigen::Matrix2d R_cand = svd.matrixV() * svd.matrixU().transpose();

            if (R_cand.determinant() < 0)
            {
                Eigen::Matrix2d D = Eigen::Matrix2d::Identity();
                D(1, 1) = -1.0;
                R_cand = svd.matrixV() * D * svd.matrixU().transpose();
            }

            double c_cand = svd.singularValues().sum() / P_centered.squaredNorm();
            Eigen::Vector2d T_cand = mu_Q - c_cand * R_cand * mu_P;

            // В. ПРОВЕРКА КАЧЕСТВА (RMSE)
            double current_rmse_sq = 0.0;
            for (int i = 0; i < 3; ++i)
            {
                Eigen::Vector2d p_val = P_mat.col(i);
                Eigen::Vector2d q_val = Q_mat.col(i);
                Eigen::Vector2d p_trans = c_cand * R_cand * p_val + T_cand;
                current_rmse_sq += (p_trans - q_val).squaredNorm();
            }
            double current_rmse = std::sqrt(current_rmse_sq / 3.0);

            if (current_rmse < best_rmse && std::abs(c_cand - 1.0) < 0.1)
            {
                best_rmse = current_rmse;
                best_R = R_cand;
                best_T = T_cand;
                best_scale = c_cand;
                best_missing_idx = missing_idx;
                // Запоминаем, какой столб (0,1,2) какому эталону соответствует
                for (int i = 0; i < 3; ++i)
                    best_mapping[p_indices[i]] = ref_indices[i];
            }

        } while (std::next_permutation(p_indices.begin(), p_indices.end()));
    }

    // 3. АНАЛИЗ РЕЗУЛЬТАТА
    const double RMSE_THRESHOLD_3PT = 0.03;

    if (best_missing_idx != -1)
    {
        // --- ДЕТАЛЬНЫЙ ЛОГ ---
        logi.log("--- CALIBRATION RESULTS ---\n");
        logi.log("  Hypothesis: Missing Pillar = %s\n", ref_names[best_missing_idx].c_str());

        double rot_deg = std::atan2(best_R(1, 0), best_R(0, 0)) * 180.0 / M_PI;
        logi.log("  Scale: %.6f (Err: %+.2f%%) | Angle: %.2f°\n", best_scale, (best_scale - 1.0) * 100.0, rot_deg);
        logi.log("  Pos:   X=%.4f m, Y=%.4f m (Dist from origin: %.3f m)\n", best_T.x(), best_T.y(), best_T.norm());

        if (best_rmse > RMSE_THRESHOLD_3PT)
        {
            logi.log_r("❌ FAILED: Best RMSE %.1f mm > Threshold %.1f mm\n", best_rmse * 1000, RMSE_THRESHOLD_3PT * 1000);
            return false;
        }

        // --- ТАБЛИЦА ВАЛИДАЦИИ ---
        logi.log("--- VALIDATION (3 Visible) ---\n");

        for (int i = 0; i < 3; ++i)
        {
            int ref_idx = best_mapping[i];
            Eigen::Vector2d p = pillars[i].local.cast<double>();
            Eigen::Vector2d q_exp = reference_centers_[ref_idx].cast<double>();
            Eigen::Vector2d q_calc = best_scale * best_R * p + best_T;
            double err = (q_calc - q_exp).norm();

            char status = (err <= 0.001) ? '+' : (err <= 0.010 ? '~' : '!');
            logi.log("  %s: Meas[%+6.3f,%+6.3f] -> Trans[%+6.3f,%+6.3f] (Ref[%+6.3f,%+6.3f]) | Err: %5.1f mm %c\n",
                     ref_names[ref_idx].c_str(),
                     p.x(), p.y(), q_calc.x(), q_calc.y(), q_exp.x(), q_exp.y(),
                     err * 1000, status);
        }

        // Применяем результат
        calibration_done_ = true;
        lidar_calibration_.rmse = best_rmse; // <--- ДОБАВЬ ЭТУ СТРОКУ СЮДА
        lidar_calibration_.scale_factor = best_scale;
        lidar_calibration_.rotation_matrix = best_R;
        lidar_calibration_.position = best_T;
        lidar_calibration_.rotation_deg = rot_deg;

        // --- СТАТИСТИКА УСПЕХА ---
        stats_.calib_success++;
        stats_.mode_3pt++; // Режим 3 точек
        stats_.sum_rmse += best_rmse;
        if (best_rmse > stats_.max_rmse)
            stats_.max_rmse = best_rmse;
        // Записываем, какого столба не было
        stats_.missing_counts[ref_names[best_missing_idx]]++;
        // -------------------------

        // Обновляем имена видимых столбов
        for (int i = 0; i < 3; ++i)
        {
            int ref_idx = best_mapping[i];
            pillars[i].name = ref_names[ref_idx];
            pillars[i].global = reference_centers_[ref_idx].cast<double>();
            pillars[i].is_estimated = false;
        }

        // --- ВОССТАНОВЛЕНИЕ НЕВИДИМОГО ---
        FinalPillar missing_pillar;
        missing_pillar.name = ref_names[best_missing_idx];
        missing_pillar.global = reference_centers_[best_missing_idx].cast<double>();

        // Обратная проекция (с исправлением типов Eigen)
        missing_pillar.local = ((1.0 / best_scale) * best_R.transpose() * (missing_pillar.global - best_T)).cast<float>();
        missing_pillar.is_estimated = true;

        logi.log("--- RECONSTRUCTION ---\n");
        logi.log("  %s (Missing): Reconstructed at Local[%+6.3f, %+6.3f]\n",
                 missing_pillar.name.c_str(), missing_pillar.local.x(), missing_pillar.local.y());

        pillars.push_back(missing_pillar);

        logi.log_g("✅ 3-PT SUCCESS! RMSE=%.1f mm\n", best_rmse * 1000);

        saveCalibrationParameters();
        final_pillars_results_ = pillars;
        publishFinalMarkers(final_pillars_results_);

        return true;
    }
    else
    {
        logi.log_r("❌ FAILED: No geometric match found among hypotheses.\n");
        stats_.calib_fail++; // Статистика провала
        return false;
    }
}

/*
 * Выбор лучших 4 столбов из большего количества
 */
void PillarScanNode::selectBestFourPillars(AlignedPillarVector &pillars)
{
    if (pillars.size() <= 4)
        return;

    logi.log("Selecting best 4 pillars from %lu candidates...\n", pillars.size());

    // Сортируем по убыванию total_weight
    std::sort(pillars.begin(), pillars.end(),
              [](const FinalPillar &a, const FinalPillar &b)
              {
                  return a.total_weight > b.total_weight;
              });

    // Оставляем только 4 лучших
    pillars.resize(4);

    // Переименовываем
    for (size_t i = 0; i < pillars.size(); ++i)
    {
        pillars[i].name = "Pillar_" + std::to_string(i);
    }

    logi.log_g("Selected 4 pillars with highest weights\n");
}

// Инициализация (перенесли сюда тяжелую логику)
void PillarScanNode::init()
{
    logi.log("\n=== PillarScanNode v7.0 Init (Loop-based) ===\n");
    loadParameters();      // Твой метод
    initReferenceSystem(); // Твой метод (LM)

    logi.log("Subscribing to /scan...\n");
    scan_sub = nh.subscribe("/scan", 10, &PillarScanNode::scanCallback, this);

    logi.log("Subscribing to pb/Data/Modul...\n");
    sub_modul = nh.subscribe("pb/Data/Modul", 3, &PillarScanNode::modulCallback, this);

    initialized_ = true;
    logi.log_g("Initialization complete. Waiting for data...\n");
}

// Коллбек: только копирует данные
void PillarScanNode::scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    std::lock_guard<std::mutex> lock(scan_mutex_); // <--- Защита
    current_scan_ = *scan;
    new_scan_available_ = true;
    if (meta_scan.header.frame_id.empty())
        meta_scan = *scan;
}

// Метод обработки (вызываем в цикле)
void PillarScanNode::process()
{
    sensor_msgs::LaserScan scan_copy;
    {
        std::lock_guard<std::mutex> lock(scan_mutex_); // <--- Защита
        if (!initialized_ || !new_scan_available_)
            return;
        scan_copy = current_scan_; // Копируем данные, чтобы быстро освободить мьютекс
        new_scan_available_ = false;
    }
    processPipeline(scan_copy); // Запускаем пайплайн для одного скана Дальше работаем с scan_copy вместо current_scan_
    publishVisuals();               // Публикуем результат
}

void PillarScanNode::modulCallback(const pb_msgs::Struct_Modul2Data::ConstPtr &msg)
{
    std::lock_guard<std::mutex> lock(modul_mutex_);
    last_modul_msg_ = *msg;
    last_modul_rcv_time_ = ros::Time::now(); // <--- Запоминаем время прихода
    has_modul_data_ = true;
    // logi.log("Laser Data Received: ID=%d\n", msg->id); // Можно раскомментить для отладки
}

// Метод публикации (код взят из твоего старого таймера)
void PillarScanNode::publishVisuals()
{
    if (clean_points_results_.size() > 0 && meta_scan.header.frame_id != "")
        pub_filtered_scan.publish(createPointsMarker(clean_points_results_, meta_scan.header.frame_id, "clean_points", 0, 0.7f, 0.7f, 0.7f, 0.05f));

    if (marker_m1_results_.points.size() > 0)
    {
        marker_m1_results_.header.stamp = ros::Time::now();
        pub_method_1.publish(marker_m1_results_);
    }
    if (marker_m2_results_.points.size() > 0)
    {
        marker_m2_results_.header.stamp = ros::Time::now();
        pub_method_2.publish(marker_m2_results_);
    }
    if (marker_m3_results_.points.size() > 0)
    {
        marker_m3_results_.header.stamp = ros::Time::now();
        pub_method_3.publish(marker_m3_results_);
    }
    if (fused_centers_results_.size() > 0 && meta_scan.header.frame_id != "")
    {
        pub_fused_pillars.publish(createPointsMarker(fused_centers_results_, meta_scan.header.frame_id, "fused_centers", 4, 0.0f, 1.0f, 0.0f, 0.15f));
    }
    if (!final_pillars_results_.empty())
    {
        publishFinalMarkers(final_pillars_results_);
    }
}

void PillarScanNode::loadParameters()
{
    logi.log_r("\n--- Loading YAML Parameters ---\n");

    // Вспомогательная лямбда для безопасного чтения double
    auto loadParam = [this](const std::string &key, double &var, double default_val, const char *name)
    {
        if (!nh.getParam(key, var))
        {
            logi.log_r("WARNING: Param '%s' not found, using default = %.4f\n", name, default_val);
            var = default_val;
            return false;
        }
        else
        {
            logi.log("Loaded '%s' = %.4f\n", name, var);
            return true;
        }
    };

    // Вспомогательная лямбда для int
    auto loadParamInt = [this](const std::string &key, int &var, int default_val, const char *name)
    {
        if (!nh.getParam(key, var))
        {
            logi.log_r("WARNING: Param '%s' not found, using default = %d\n", name, default_val);
            var = default_val;
            return false;
        }
        else
        {
            logi.log("Loaded '%s' = %d\n", name, var);
            return true;
        }
    };

    // 1. Диаметр столба и радиус
    loadParam("/pb_config/scan_node/pillar_diametr", pillar_diam_, 0.315, "pillar_diametr");
    pillar_radius_ = pillar_diam_ / 2.0;

    // 2. Эталонные расстояния между поверхностями (6 штук)
    loadParam("/pb_config/scan_node/pillar_0_1", d_surf[0], 10.5, "pillar_0_1");
    loadParam("/pb_config/scan_node/pillar_0_2", d_surf[1], 12.0, "pillar_0_2");
    loadParam("/pb_config/scan_node/pillar_0_3", d_surf[2], 4.8, "pillar_0_3");
    loadParam("/pb_config/scan_node/pillar_1_2", d_surf[3], 5.5, "pillar_1_2");
    loadParam("/pb_config/scan_node/pillar_1_3", d_surf[4], 11.5, "pillar_1_3");
    loadParam("/pb_config/scan_node/pillar_2_3", d_surf[5], 4.8, "pillar_2_3");

    // Вычисление расстояний между центрами
    for (int i = 0; i < 6; ++i)
    {
        d_center[i] = d_surf[i] + pillar_diam_;
    }

    // Логирование расстояний (исправлены индексы!)
    logi.log_b("    pillar_diametr: %.4f\n", pillar_diam_);
    logi.log("    pillar_0_1 RB-RT (surf): %.4f -> (center): %.4f\n", d_surf[0], d_center[0]);
    logi.log("    pillar_0_2 RB-LT (surf): %.4f -> (center): %.4f\n", d_surf[1], d_center[1]);
    logi.log("    pillar_0_3 RB-LB (surf): %.4f -> (center): %.4f\n", d_surf[2], d_center[2]);
    logi.log("    pillar_1_2 RT-LT (surf): %.4f -> (center): %.4f\n", d_surf[3], d_center[3]);
    logi.log("    pillar_1_3 RT-LB (surf): %.4f -> (center): %.4f\n", d_surf[4], d_center[4]);
    logi.log("    pillar_2_3 LT-LB (surf): %.4f -> (center): %.4f\n", d_surf[5], d_center[5]);

    // --- Параметры фильтрации кластеров ---
    loadParam("/pb_config/scan_node/min_cluster_width", min_cluster_width_, 0.20, "min_cluster_width");
    loadParam("/pb_config/scan_node/max_cluster_width", max_cluster_width_, 0.40, "max_cluster_width");
    loadParamInt("/pb_config/scan_node/min_cluster_points", min_cluster_points_, 11, "min_cluster_points");

    logi.log_b("    Cluster Filter Config:\n");
    logi.log("    Min Cluster Width: %.3f m\n", min_cluster_width_);
    logi.log("    Max Cluster Width: %.3f m\n", max_cluster_width_);
    logi.log("    Min Cluster Points: %d\n", min_cluster_points_);

    // Параметры детекции
    loadParam("/pb_config/scan_node/jump_dist_threshold", jump_dist_threshold, 0.33, "jump_dist_threshold");
    loadParam("/pb_config/scan_node/cluster_dist_threshold", cluster_dist_threshold, 0.05, "cluster_dist_threshold");
    logi.log("    Detection: jump_dist_threshold=%.2f, cluster_dist_threshold=%.2f\n", jump_dist_threshold, cluster_dist_threshold);

    // Веса методов
    loadParam("/pb_config/scan_node/w_method_1_jump", w_method[1], 1.0, "w_method_1_jump");
    loadParam("/pb_config/scan_node/w_method_2_cluster", w_method[2], 0.9, "w_method_2_cluster");
    loadParam("/pb_config/scan_node/w_method_3_minima", w_method[3], 0.8, "w_method_3_minima");
    logi.log_b("    Weights: method_1=%.2f, method_2=%.2f, method_3=%.2f\n", w_method[1], w_method[2], w_method[3]);

    // Параметры фильтрации сканов
    loadParam("/pb_config/scan_node/min_range_filter", min_range_filter, 0.2, "min_range");
    loadParam("/pb_config/scan_node/max_range_filter", max_range_filter, 15.0, "max_range");
    loadParam("/pb_config/scan_node/neighbor_radius_filter", neighbor_radius_filter, 0.3, "neighbor_radius");
    loadParamInt("/pb_config/scan_node/min_neighbors_filter", min_neighbors_filter, 3, "min_neighbors");
    loadParam("/pb_config/scan_node/intensity_min_threshold", intensity_min_threshold, 10.0, "intensity_min_threshold");
    loadParam("/pb_config/scan_node/edge_angle_threshold_deg", edge_angle_threshold, 15.0, "edge_angle_threshold_deg");
    edge_angle_threshold *= M_PI / 180.0; // в радианы

    logi.log("    Filter Range: [%.2f, %.2f], KNN: R=%.2f, N=%d\n", min_range_filter, max_range_filter, neighbor_radius_filter, min_neighbors_filter);
    logi.log("    Artifact Filter: I_min=%.2f, Angle_rad=%.4f (%.1f deg)\n", intensity_min_threshold, edge_angle_threshold, edge_angle_threshold * 180.0 / M_PI);

    // Параметры Fusion
    loadParam("/pb_config/scan_node/rmse_max_tolerance", rmse_max_tolerance, 0.01, "rmse_max_tolerance");
    loadParamInt("/pb_config/scan_node/n_max_points_norm", n_max_points_norm, 100, "n_max_points_norm");
    loadParam("/pb_config/scan_node/fusion_group_radius", fusion_group_radius, 0.2, "fusion_group_radius");
    loadParamInt("/pb_config/scan_node/min_dbscan_points", min_dbscan_points_, 2, "min_dbscan_points");

    logi.log_b("    DBSCAN Fusion Config:\n");
    logi.log("    Fusion: RMSE_max=%.4f, N_max=%d \n", rmse_max_tolerance, n_max_points_norm);
    logi.log("    Min DBSCAN Points (minPts): %d\n", min_dbscan_points_);
    logi.log("    Fusion Group Radius: %.3f m\n", fusion_group_radius);

    // --- НОВОЕ: Параметры лазеров ---
    loadParam("/pb_config/lasers/dist_offset_box", lasers_dist_offset_box_, 0.042, "lasers_dist_offset_box");
    // Если параметр не задан в yaml, используем твое значение 0.0636396
    loadParam("/pb_config/lasers/motor_bias", lasers_motor_bias_, 0.0636396, "lasers_motor_bias");

    // Инициализация геометрии моторов (как ты описал)
    // Motor 0 (IV четверть: +bias, -bias, +45 deg)
    laser_configs_[0] = {lasers_motor_bias_, -lasers_motor_bias_, 45.0};
    // Motor 1 (III четверть: -bias, -bias, -45 deg)
    laser_configs_[1] = {-lasers_motor_bias_, -lasers_motor_bias_, -45.0};
    // Motor 2 (II четверть: -bias, +bias, -135 deg)
    laser_configs_[2] = {-lasers_motor_bias_, lasers_motor_bias_, -135.0};
    // Motor 3 (I четверть: +bias, +bias, +135 deg)
    laser_configs_[3] = {lasers_motor_bias_, lasers_motor_bias_, 135.0};

    logi.log_b("    Laser Config: OffsetBox=%.4f, Bias=%.4f\n", lasers_dist_offset_box_, lasers_motor_bias_);

    logi.log_r("--- Parameters Loaded ---\n");
}

// ИЗМЕНЕНА: initReferenceSystem (v6.1 - LM Optimization)
// Находит координаты эталонной системы с помощью нелинейной оптимизации
// Levenberg-Marquardt, минимизируя невязки по всем 6 расстояниям.
void PillarScanNode::initReferenceSystem()
{
    reference_centers_.resize(4);

    logi.log("\n=== Reference System Initialization (Levenberg-Marquardt) ===\n");

    // Начальное приближение — трилатерация
    double L = d_center[0];
    reference_centers_[0] = Eigen::Vector2f(0, 0);        // RB (0)
    reference_centers_[1] = Eigen::Vector2f((float)L, 0); // RT (1)

    // LT (2) - начальное приближение
    // x2 = (d_0_1^2 + d_0_2^2 - d_1_2^2) / (2*d_0_1)
    double x2 = (L * L + d_center[1] * d_center[1] - d_center[3] * d_center[3]) / (2 * L);
    double y2 = std::sqrt(std::max(0.0, d_center[1] * d_center[1] - x2 * x2));
    reference_centers_[2] = Eigen::Vector2f((float)x2, (float)y2);

    // LB (3) - начальное приближение
    // x3 = (d_0_1^2 + d_0_3^2 - d_1_3^2) / (2*d_0_1)
    double x3 = (L * L + d_center[2] * d_center[2] - d_center[4] * d_center[4]) / (2 * L);
    double y3 = std::sqrt(std::max(0.0, d_center[2] * d_center[2] - x3 * x3));
    reference_centers_[3] = Eigen::Vector2f((float)x3, (float)y3);

    logi.log_b("Initial approximation:\n");
    for (int i = 0; i < 4; ++i)
        logi.log(" P%d: [%.4f, %.4f]\n", i, reference_centers_[i].x(), reference_centers_[i].y());
    logi.log("End approximation:\n");

    // --- LM ОПТИМИЗАЦИЯ ---
    // Параметры: x2, y2, x3, y3
    Eigen::Vector4d params;
    params << reference_centers_[2].x(), reference_centers_[2].y(),
        reference_centers_[3].x(), reference_centers_[3].y();

    const int max_iter = 50;
    const double tol = 1e-12; // Более строгий критерий сходимости
    double lambda = 0.001;
    double best_cost = std::numeric_limits<double>::max();
    Eigen::Vector4d best_params = params;

    for (int iter = 0; iter < max_iter; ++iter)
    {
        Eigen::Vector2d p2(params[0], params[1]); // LT (2)
        Eigen::Vector2d p3(params[2], params[3]); // LB (3)
        Eigen::Vector2d rb(0, 0);                 // RB (0)
        Eigen::Vector2d rt(L, 0);                 // RT (1)

        // Остатки (6 расстояний) - ИСПРАВЛЕННЫЕ ИНДЕКСЫ
        Eigen::VectorXd residuals(6);
        residuals[0] = (rt - rb).norm() - d_center[0]; // RB-RT = 4.0150
        residuals[1] = (p2 - rb).norm() - d_center[1]; // RB-LT = 6.0930
        residuals[2] = (p3 - rb).norm() - d_center[2]; // RB-LB = 3.5350
        residuals[3] = (p2 - rt).norm() - d_center[3]; // RT-LT = 3.8250
        residuals[4] = (p3 - rt).norm() - d_center[4]; // RT-LB = 5.0990
        residuals[5] = (p2 - p3).norm() - d_center[5]; // LT-LB = 4.4940

        double cost = residuals.squaredNorm();

        if (cost < best_cost)
        {
            best_cost = cost;
            best_params = params;
        }

        if (cost < tol)
        {
            logi.log(" Converged after %d iterations (cost=%.2e)\n", iter, cost);
            break;
        }

        if (iter % 10 == 0)
        {
            logi.log(" Iter %d: cost=%.2e, lambda=%.2e\n", iter, cost, lambda);
        }

        // Якобиан 6x4 (J) - ИСПРАВЛЕННЫЕ ИНДЕКСЫ
        Eigen::Matrix<double, 6, 4> J = Eigen::Matrix<double, 6, 4>::Zero();

        auto jacobian_row = [&](int row, const Eigen::Vector2d &p, const Eigen::Vector2d &fixed, int px, int py)
        {
            double d = (p - fixed).norm();
            if (d > 1e-12)
            {
                J(row, px) = (p.x() - fixed.x()) / d;
                J(row, py) = (p.y() - fixed.y()) / d;
            }
        };

        jacobian_row(1, p2, rb, 0, 1); // residuals[1] = RB-LT
        jacobian_row(2, p3, rb, 2, 3); // residuals[2] = RB-LB
        jacobian_row(3, p2, rt, 0, 1); // residuals[3] = RT-LT
        jacobian_row(4, p3, rt, 2, 3); // residuals[4] = RT-LB

        // residuals[5] = LT-LB
        double d_lt_lb = (p2 - p3).norm();
        if (d_lt_lb > 1e-12)
        {
            J(5, 0) = (p2.x() - p3.x()) / d_lt_lb;
            J(5, 1) = (p2.y() - p3.y()) / d_lt_lb;
            J(5, 2) = (p3.x() - p2.x()) / d_lt_lb;
            J(5, 3) = (p3.y() - p2.y()) / d_lt_lb;
        }

        // RB-RT (r5) - строка остается нулевой, т.к. P0 и P1 фиксированы.

        Eigen::Matrix4d H = J.transpose() * J;
        Eigen::Vector4d g = J.transpose() * residuals;

        // LM итерация
        bool step_accepted = false;
        for (int lm_iter = 0; lm_iter < 10; ++lm_iter)
        {
            Eigen::Matrix4d H_lm = H + lambda * Eigen::Matrix4d::Identity();
            Eigen::Vector4d delta = H_lm.ldlt().solve(-g);

            if (!delta.allFinite())
            {
                lambda *= 10;
                continue;
            }

            Eigen::Vector4d params_new = params + delta;
            Eigen::Vector2d p2_new(params_new[0], params_new[1]);
            Eigen::Vector2d p3_new(params_new[2], params_new[3]);

            // Новые остатки (после исправления)
            Eigen::VectorXd res_new(6);
            res_new[0] = (rt - rb).norm() - d_center[0];         // RB-RT
            res_new[1] = (p2_new - rb).norm() - d_center[1];     // RB-LT
            res_new[2] = (p3_new - rb).norm() - d_center[2];     // RB-LB
            res_new[3] = (p2_new - rt).norm() - d_center[3];     // RT-LT
            res_new[4] = (p3_new - rt).norm() - d_center[4];     // RT-LB
            res_new[5] = (p2_new - p3_new).norm() - d_center[5]; // LT-LB

            double new_cost = res_new.squaredNorm();

            if (new_cost < cost)
            {
                params = params_new;
                lambda = std::max(lambda / 10.0, 1e-12);
                step_accepted = true;
                break;
            }
            else
            {
                lambda *= 10;
            }
        }

        if (!step_accepted)
        {
            logi.log(" LM: No best improvement, stopping at iter %d\n", iter);
            break;
        }
    }

    // Используем лучшие параметры
    params = best_params;
    reference_centers_[2] = Eigen::Vector2f((float)params[0], (float)params[1]);
    reference_centers_[3] = Eigen::Vector2f((float)params[2], (float)params[3]);

    // Детальная проверка ошибок
    logi.log("=== Distance Validation ===\n");
    double max_error = 0.0;

    std::vector<std::string> dist_names = {"RB-RT", "RB-LT", "RB-LB", "RT-LT", "RT-LB", "LT-LB"};
    std::vector<std::pair<int, int>> indices = {{0, 1}, {0, 2}, {0, 3}, {1, 2}, {1, 3}, {2, 3}};

    for (int i = 0; i < 6; ++i)
    {
        double measured = MathUtils::dist2D(reference_centers_[indices[i].first],
                                            reference_centers_[indices[i].second]);
        double expected = d_center[i];
        double error = std::abs(measured - expected);

        if (error > max_error)
            max_error = error;

        if (error <= 0.001) // 1 мм
            logi.log_g(" %s: %.4f (expected: %.4f) ✓ Δ=%.1f mm\n",
                       dist_names[i].c_str(), measured, expected, error * 1000);
        else if (error <= 0.010) // 10 мм
            logi.log_w(" %s: %.4f (expected: %.4f) ~ Δ=%.1f mm\n",
                       dist_names[i].c_str(), measured, expected, error * 1000);
        else
            logi.log_r(" %s: %.4f (expected: %.4f) ✗ Δ=%.1f mm\n",
                       dist_names[i].c_str(), measured, expected, error * 1000);
    }

    // Финальное решение
    if (max_error > 0.010) // 10 мм
    {
        logi.log_r("\nFATAL: Pillar geometry inconsistent! Max error = %.1f mm\n", max_error * 1000);
        reference_centers_.clear();
    }
    else
    {
        logi.log_g("\nGeometry validated. Max residual = %.1f mm ✓\n", max_error * 1000);
        logi.log_g("Final coordinates:\n");
        std::vector<std::string> names = {"RB", "RT", "LT", "LB"};
        for (int i = 0; i < 4; ++i)
            logi.log_g(" %s: [%.4f, %.4f]\n", names[i].c_str(),
                       reference_centers_[i].x(), reference_centers_[i].y());
    }

    // =================================================================================
    // [НОВОЕ] ЗАПИСЬ ОПТИМИЗИРОВАННЫХ КООРДИНАТ В ROSPARAM
    // =================================================================================
    logi.log("--- 💾 UPDATING ROS PARAMS (Global Reference) ---\n");

    // Пишем в те же пути, что указаны в твоем .yaml (/pb_config/pillars/...)
    // Это позволит другим нодам читать уточненные данные, как будто они были в конфиге.
    for (int i = 0; i < 4; ++i)
    {
        std::string param_base = "/pb_config/pillars/pillar_" + std::to_string(i);

        // Преобразуем float (Eigen) в double (ROS param)
        double x_val = (double)reference_centers_[i].x();
        double y_val = (double)reference_centers_[i].y();

        nh.setParam(param_base + "_x", x_val);
        nh.setParam(param_base + "_y", y_val);

        // logi.log("  Set %s_x/y: [%.4f, %.4f]\n", param_base.c_str(), x_val, y_val);
    }
    logi.log_g("✅ Optimized pillar coordinates saved to Parameter Server.\n");
    // =================================================================================

    logi.log("==============================================\n");
}

// /*
//  * НОВЫЙ МЕТОД: Расчет позиции чисто по Лазерам
//  * Вызывается из processPipeline ПЕРЕД fuseResults
//  */
// void PillarScanNode::processLasers()
// {
//     // Очищаем буфер слияния перед новым циклом
//     lasers_for_fusion_.clear();

//     // Сброс результата в начале "Laser Only"
//     laser_only_result_.valid = false;
//     laser_only_result_.x = 0;
//     laser_only_result_.y = 0;
//     laser_only_result_.rmse = 0;
//     laser_only_result_.used_count = 0;

//     // --- ЭТАП 1: Проверка наличия данных и связи ---
//     pb_msgs::Struct_Modul2Data data;
//     std::string fail_reason = "";

//     { // Фигурные скобки { ... } здесь создают область видимости (scope). Это критически важно для работы с многопоточностью и мьютексами (std::lock_guard).
//         std::lock_guard<std::mutex> lock(modul_mutex_);

//         if (!has_modul_data_) // Данные еще ни разу не приходили
//         {
//             logi.log("LASER SKIP: Waiting for first packet...\n");
//             return; // Тут можно молча выйти, чтобы не спамить при старте, или раскомментить лог
//         }

//         double time_diff = (ros::Time::now() - last_modul_rcv_time_).toSec();
//         if (time_diff > 0.2)
//         { // Таймаут 200 мс
//             logi.log_r("LASER SKIP: Connection Timeout (Last seen %.3fs ago)\n", time_diff);
//             return;
//         }

//         data = last_modul_msg_; // Копируем данные
//     } // Мьютекс будет разблокирован автоматически, когда переменная lock уничтожится. Переменная lock уничтожается, когда выполнение кода доходит до закрывающей скобки }.

//     // --- ЭТАП 2: Проверка готовности системы ---
//     if (!calibration_done_)
//     {
//         logi.log_w("LASER SKIP: Lidar Calibration not done yet.\n");
//         return;
//     }

//     // --- ЭТАП 3: Обработка измерений ---

//     // Берем позицию робота от Umeyama
//     double robot_x = lidar_calibration_.position.x();
//     double robot_y = lidar_calibration_.position.y();
//     double robot_th = lidar_calibration_.rotation_deg;

//     TrilaterationSolver local_solver({robot_x, robot_y});
//     int valid_local_cnt = 0;
//     std::string debug_status = ""; // Строка для накопления статусов по каждому лазеру (L0:OK L1:Fail...)

//     // int valid_lasers = 0;
//     // for (int i = 0; i < 4; ++i)
//     // {
//     //     double corrected_dist = 0;
//     //     Eigen::Vector2d pillar_global;

//     //     // Передаем debug_status по ссылке, чтобы validate функция дописала туда причину
//     //     if (validateAndPrepareLaser(i, data, robot_x, robot_y, robot_th, corrected_dist, pillar_global, debug_status))
//     //     {
//     //         SPoint beacon = {pillar_global.x(), pillar_global.y()};
//     //         local_solver.add_circle_from_distance(beacon, corrected_dist);
//     //         valid_lasers++;
//     //     }
//     // }



//     for (int i = 0; i < 4; ++i)
//     {
//         double corrected_dist = 0;
//         Eigen::Vector2d pillar_global;
//         std::string log_part = "";

//         // 1. Базовая валидация (Геометрия + Статус + Грубый порог времени 333мс)
//         // ВАЖНО: В validateAndPrepareLaser убедись, что стоит проверка if (meas.time > 333)
//         if (validateAndPrepareLaser(i, data, robot_x, robot_y, robot_th, corrected_dist, pillar_global, log_part))
//         {
//             // Данные геометрически верны и моложе 333 мс

//             // А. Добавляем в ЛОКАЛЬНЫЙ решатель (для статистики)
//             SPoint beacon = {pillar_global.x(), pillar_global.y()};
//             local_solver.add_circle_from_distance(beacon, corrected_dist); // Вес 1.0 тут не важен, т.к. одни лазеры
//             valid_local_cnt++;

//             // Б. Проверяем свежесть для FUSION (строгий отбор)
//             int age = data.laser[i].time;
//             if (age <= TIME_LIMIT_FUSION)
//             {
//                 // Данные свежие! Добавляем в буфер для общего MNK
//                 FusionLaserMeas fusion_item;
//                 fusion_item.beacon = beacon;
//                 fusion_item.dist = corrected_dist;
//                 fusion_item.weight = LASER_WEIGHT;
//                 fusion_item.id = i;
//                 lasers_for_fusion_.push_back(fusion_item);

//                 // Помечаем в логе, что ушло в Fusion
//                 debug_status += "L" + std::to_string(i) + ":FUSION ";
//             }
//             else
//             {
//                 // Валидно, но старовато для управления
//                 debug_status += "L" + std::to_string(i) + ":LocalOnly(" + std::to_string(age) + "ms) ";
//             }
//         }
//         else
//         {
//             // Не прошло валидацию
//             debug_status += log_part;
//         }
//     }

//     // --- ЭТАП 4: Решение и Финальный лог ---

//     if (valid_local_cnt >= 3)
//     {
//         SPoint_Q result = local_solver.find_A_by_mnk_simple();

//         laser_only_result_.x = result.A.x;
//         laser_only_result_.y = result.A.y;
//         laser_only_result_.rmse = result.quality;
//         laser_only_result_.used_count = valid_local_cnt;
//         laser_only_result_.valid = true;

//         // // Лог с акцентом на то, сколько ушло в Fusion
//         // logi.log_g("LASER STATS: Local=%d/4 | ToFusion=%lu | Pos: %.3f, %.3f | Details: %s\n",
//         //            valid_local_cnt, lasers_for_fusion_.size(), result.A.x, result.A.y, debug_status.c_str());

//         // --- НОВАЯ СТРОКА ЛОГА ---
//         // X, Y - координаты
//         // RMSE - точность в миллиметрах
//         // Used - сколько лазеров участвовало в геометрии
//         // Fusion - сколько из них пошло в общий котел (свежие)
//         // Sts - детальный статус
//         logi.log_g("LASER_CALC: Pos=[%.4f, %.4f] | RMSE=%.1f mm | Used=%d/4 (Fusion=%lu) | Sts: %s\n", 
//                    result.A.x, 
//                    result.A.y, 
//                    result.quality * 1000.0, // Переводим метры в мм для удобства
//                    valid_local_cnt, 
//                    lasers_for_fusion_.size(), 
//                    debug_status.c_str());
//     }
//     else
//     {
//         // // Вот тут мы пишем, почему не посчитали, даже если связь есть
//         // logi.log_w("LASER FAIL: Not enough valid (%d/4). Threshold >= 3. Reasons: %s\n",
//         //            valid_lasers, debug_status.c_str());
//         // Если лазеров мало для локального расчета, но для Fusion они есть - это нормально
//         if (!lasers_for_fusion_.empty())
//         {
//             logi.log("LASER PARTIAL: Local calc skipped (%d < 3), but %lu sent to Fusion. Details: %s\n",
//                      valid_local_cnt, lasers_for_fusion_.size(), debug_status.c_str());
//         }
//         else
//         {
//             logi.log_w("LASER FAIL: No valid data. %s\n", debug_status.c_str());
//         }
//     }
// }


/*
 * НОВЫЙ МЕТОД: Расчет позиции чисто по Лазерам
 * Вызывается из processPipeline ПЕРЕД fuseResults
 */
void PillarScanNode::processLasers()
{
    // Очищаем буфер слияния перед новым циклом
    lasers_for_fusion_.clear();

    // Сброс результата в начале "Laser Only"
    laser_only_result_.valid = false;
    laser_only_result_.x = 0;
    laser_only_result_.y = 0;
    laser_only_result_.rmse = 0;
    laser_only_result_.used_count = 0;

    // --- ЭТАП 1: Проверка наличия данных и связи ---
    pb_msgs::Struct_Modul2Data data;
    
    // Переменная fail_reason нам больше не нужна, так как мы пишем лог сразу при отказе
    // std::string fail_reason = ""; 

    { // Фигурные скобки { ... } здесь создают область видимости (scope). Это критически важно для работы с многопоточностью и мьютексами (std::lock_guard).
        std::lock_guard<std::mutex> lock(modul_mutex_);

        if (!has_modul_data_) // Данные еще ни разу не приходили
        {
            // Ограничиваем лог, чтобы не спамить
            static int wait_cnt = 0;
            if (wait_cnt++ % 20 == 0) logi.log("LASER SKIP: Waiting for first packet...\n");
            return; // Тут можно молча выйти, чтобы не спамить при старте, или раскомментить лог
        }

        double time_diff = (ros::Time::now() - last_modul_rcv_time_).toSec();
        if (time_diff > 0.2)
        { // Таймаут 200 мс
            // Ограничиваем лог
            static int to_cnt = 0;
            if (to_cnt++ % 20 == 0) logi.log_r("LASER SKIP: Connection Timeout (Last seen %.3fs ago)\n", time_diff);
            return;
        }

        data = last_modul_msg_; // Копируем данные
    } // Мьютекс будет разблокирован автоматически, когда переменная lock уничтожится. Переменная lock уничтожается, когда выполнение кода доходит до закрывающей скобки }.

    // --- ЭТАП 2: Проверка готовности системы ---
    if (!calibration_done_)
    {
        // Можно ограничить частоту лога и тут
        static int calib_cnt = 0;
        if (calib_cnt++ % 20 == 0) logi.log_w("LASER SKIP: Lidar Calibration not done yet.\n");
        return;
    }

    // --- ЭТАП 3: Обработка измерений ---

    // Берем позицию робота от Umeyama
    double robot_x = lidar_calibration_.position.x();
    double robot_y = lidar_calibration_.position.y();
    double robot_th = lidar_calibration_.rotation_deg;

    TrilaterationSolver local_solver({robot_x, robot_y});
    int valid_local_cnt = 0;
    std::string debug_status = ""; // Строка для накопления статусов по каждому лазеру (L0:OK L1:Fail...)
    std::string debug_details_str = ""; // Строка для детальных статусов (T.., Err..)

    for (int i = 0; i < 4; ++i)
    {
        double corrected_dist = 0;
        Eigen::Vector2d pillar_global;
        std::string log_part = ""; // Часть лога для текущего лазера

        // 1. Базовая валидация (Геометрия + Статус + Грубый порог времени TIME_LIMIT_LOCAL)
        // ВАЖНО: В validateAndPrepareLaser убедись, что стоит проверка if (meas.time > TIME_LIMIT_LOCAL)
        // Метод validateAndPrepareLaser теперь также возвращает log_part с деталями ошибки/времени
        bool passed_validation = validateAndPrepareLaser(i, data, robot_x, robot_y, robot_th, 
                                                         corrected_dist, pillar_global, log_part);
        
        debug_details_str += log_part; // Накапливаем детали для отдельной строки лога

        if (passed_validation)
        {
            // Данные геометрически верны и моложе TIME_LIMIT_LOCAL (333 мс)

            // А. Добавляем в ЛОКАЛЬНЫЙ решатель (для статистики и проверки геометрии)
            SPoint beacon = {pillar_global.x(), pillar_global.y()};
            local_solver.add_circle_from_distance(beacon, corrected_dist); // Вес 1.0 тут не важен, т.к. одни лазеры
            valid_local_cnt++;

            // Б. Проверяем свежесть для FUSION (строгий отбор - 130 мс)
            int age = data.laser[i].time;
            if (age <= TIME_LIMIT_FUSION)
            {
                // Данные свежие! Добавляем в буфер для общего MNK
                FusionLaserMeas fusion_item;
                fusion_item.beacon = beacon;
                fusion_item.dist = corrected_dist;
                fusion_item.weight = LASER_WEIGHT;
                fusion_item.id = i;
                lasers_for_fusion_.push_back(fusion_item);

                // Помечаем в логе, что ушло в Fusion
                debug_status += "L" + std::to_string(i) + ":FUSION ";
            }
            else
            {
                // Валидно, но старовато для управления (между 130 и 333 мс)
                debug_status += "L" + std::to_string(i) + ":LocOnly ";
            }
        }
        else // Если НЕ прошло валидацию
        {
            // Не прошло валидацию. Разбираем причину по сырым данным, 
            // так как в log_part уже записан человекочитаемый текст
            const auto& l_raw = data.laser[i];
             
            if (l_raw.status != 0 || l_raw.numPillar < 0) {
                 debug_status += "L" + std::to_string(i) + ":Fail ";
            }
            else if (l_raw.time > TIME_LIMIT_LOCAL) {
                 debug_status += "L" + std::to_string(i) + ":Old ";
            }
            else {
                 // Если статус ОК, время ОК, но валидация не прошла -> значит это Гейтинг
                 debug_status += "L" + std::to_string(i) + ":Gate ";
            }
        }
    }

    // --- ЭТАП 4: Решение и Финальный лог ---

    if (valid_local_cnt >= 3)
    {
        SPoint_Q result = local_solver.find_A_by_mnk_simple();

        laser_only_result_.x = result.A.x;
        laser_only_result_.y = result.A.y;
        laser_only_result_.rmse = result.quality;
        laser_only_result_.used_count = valid_local_cnt;
        laser_only_result_.valid = true;

        // // Лог с акцентом на то, сколько ушло в Fusion
        // logi.log_g("LASER STATS: Local=%d/4 | ToFusion=%lu | Pos: %.3f, %.3f | Details: %s\n",
        //            valid_local_cnt, lasers_for_fusion_.size(), result.A.x, result.A.y, debug_status.c_str());

        // --- НОВАЯ СТРОКА ЛОГА (ДЕТАЛИ) ---
        logi.log_b("LASER_DETAILS: %s\n", debug_details_str.c_str());

        // --- НОВАЯ СТРОКА ЛОГА (ИТОГ) ---
        // X, Y - координаты
        // RMSE - точность в миллиметрах
        // Used - сколько лазеров участвовало в геометрии
        // Fusion - сколько из них пошло в общий котел (свежие)
        // Sts - детальный статус
        logi.log_g("LASER_CALC: Pos=[%.4f, %.4f] | RMSE=%.1f mm | Used=%d/4 (Fusion=%lu) | Sts: %s\n", 
                   result.A.x, 
                   result.A.y, 
                   result.quality * 1000.0, // Переводим метры в мм для удобства
                   valid_local_cnt, 
                   lasers_for_fusion_.size(), 
                   debug_status.c_str());
    }
    else
    {
        // // Вот тут мы пишем, почему не посчитали, даже если связь есть
        // logi.log_w("LASER FAIL: Not enough valid (%d/4). Threshold >= 3. Reasons: %s\n",
        //            valid_lasers, debug_status.c_str());
        
        // Выводим детали даже при неудаче, чтобы видеть причины
        if (!debug_details_str.empty()) {
             logi.log_b("LASER_DETAILS: %s\n", debug_details_str.c_str());
        }

        // Если лазеров мало для локального расчета, но для Fusion они есть - это нормально
        if (!lasers_for_fusion_.empty())
        {
            logi.log("LASER PARTIAL: Local calc skipped (%d < 3), but %lu sent to Fusion. Details: %s\n",
                     valid_local_cnt, lasers_for_fusion_.size(), debug_status.c_str());
        }
        else
        {
            logi.log_w("LASER FAIL: No valid data. %s\n", debug_status.c_str());
        }
    }
}

// bool PillarScanNode::validateAndPrepareLaser(int laser_idx,
//                                              const pb_msgs::Struct_Modul2Data &data,
//                                              double robot_x, double robot_y, double robot_th_deg,
//                                              double &out_corrected_dist,
//                                              Eigen::Vector2d &out_pillar_global,
//                                              std::string &log_str)
// {
//     const auto &meas = data.laser[laser_idx]; // Ссылка на данные
//     char buf[128];

//     // --- ЛОГ СЫРЫХ ДАННЫХ (DEBUG) ---
//     // Выводим всё, что пришло, чтобы исключить глюки парсинга
//     // logi.log("  [DEBUG L%d] RAW: Dist=%.4f, Ang=%.2f, Time=%d, Stat=%d, TgtPillar=%d\n",
//     //          laser_idx, meas.distance, meas.angle, meas.time, meas.status, meas.numPillar);

//     // 1. Статус
//     if (meas.status != 0)
//     {
//         snprintf(buf, sizeof(buf), "L%d:ErrStat(%d) ", laser_idx, meas.status);
//         log_str += buf;
//         return false;
//     }

//     // 2. Номер столба
//     if (meas.numPillar < 0 || meas.numPillar > 3)
//     {
//         snprintf(buf, sizeof(buf), "L%d:NoPillar ", laser_idx);
//         log_str += buf;
//         return false;
//     }

//     // 3. Время (Обновили порог до 333 мс)
//     if (meas.time > TIME_LIMIT_LOCAL)
//     {
//         snprintf(buf, sizeof(buf), "L%d:Old(%dms) ", laser_idx, meas.time);
//         log_str += buf;
//         return false;
//     }

//     // --- РАСЧЕТ ГЕОМЕТРИИ С ЛОГАМИ ---

//     // А. Фактическое расстояние (Вал -> Столб)
//     double r_fact = meas.distance + lasers_dist_offset_box_ + pillar_radius_;

//     // Б. Конфигурация мотора
//     double mount_th_deg = laser_configs_[laser_idx].mount_th_deg;
//     double lx = laser_configs_[laser_idx].x_loc;
//     double ly = laser_configs_[laser_idx].y_loc;

//     // В. Глобальная позиция мотора
//     double robot_th_rad = robot_th_deg * M_PI / 180.0;

//     // Поворот вектора смещения (lx, ly) на угол робота
//     double dx_rot = lx * cos(robot_th_rad) - ly * sin(robot_th_rad);
//     double dy_rot = lx * sin(robot_th_rad) + ly * cos(robot_th_rad);

//     double mx_global = robot_x + dx_rot;
//     double my_global = robot_y + dy_rot;

//     // logi.log("  [DEBUG L%d] GEOM: Robot(%.3f, %.3f, %.1f°) -> MountLoc(%.3f, %.3f) -> MotorGlobal(%.3f, %.3f)\n",
//     //          laser_idx, robot_x, robot_y, robot_th_deg, lx, ly, mx_global, my_global);

//     // Г. Угол луча
//     // angle - угол башни. mount_th - установочный. robot_th - робота.
//     // Складываем всё в градусах для наглядности
//     double beam_global_deg = robot_th_deg + mount_th_deg + (-meas.angle); // Знак минус делаем так как у меня моторы плюс по часовой
//     double beam_global_rad = beam_global_deg * M_PI / 180.0;

//     // logi.log("  [DEBUG L%d] ANGLE: Rob(%.1f) + Mnt(%.1f) + Turret(%.1f) = BeamGlobal(%.1f°)\n",
//     //          laser_idx, robot_th_deg, mount_th_deg, meas.angle, beam_global_deg);

//     // Д. Точка удара (Виртуальный столб)
//     double hit_x = mx_global + r_fact * cos(beam_global_rad);
//     double hit_y = my_global + r_fact * sin(beam_global_rad);

//     // Е. Идеальный столб (Цель)
//     int pid = meas.numPillar;
//     // Защита от выхода за границы массива (на всякий случай)
//     if (pid >= reference_centers_.size())
//         return false;
//     Eigen::Vector2d ideal_pillar = reference_centers_[pid].cast<double>();

//     // Ж. Гейтинг
//     double err_dist = sqrt(pow(hit_x - ideal_pillar.x(), 2) + pow(hit_y - ideal_pillar.y(), 2));

//     // logi.log("  [DEBUG L%d] CHECK: R_fact=%.3f -> Hit(%.3f, %.3f) vs Ideal_P%d(%.3f, %.3f) | Err=%.3f m\n",
//     //          laser_idx, r_fact, hit_x, hit_y, pid, ideal_pillar.x(), ideal_pillar.y(), err_dist);

//     if (err_dist > 0.1) // Насколько центры столбов могут не совпадать и все равно пройдет столб проверку
//     {
//         snprintf(buf, sizeof(buf), "L%d:Gate(%.2fm) ", laser_idx, err_dist);
//         log_str += buf;
//         return false;
//     }

//     // --- РАСЧЕТ ПОПРАВКИ (ПРОЕКЦИЯ) ---
//     double v_cm_x = mx_global - robot_x;
//     double v_cm_y = my_global - robot_y;

//     double v_cp_x = ideal_pillar.x() - robot_x;
//     double v_cp_y = ideal_pillar.y() - robot_y;
//     double len_cp = sqrt(v_cp_x * v_cp_x + v_cp_y * v_cp_y);
//     if (len_cp < 0.001)
//         len_cp = 0.001;
//     v_cp_x /= len_cp;
//     v_cp_y /= len_cp;

//     double projection = v_cm_x * v_cp_x + v_cm_y * v_cp_y;
//     out_corrected_dist = r_fact + projection;
//     out_pillar_global = ideal_pillar;

//     // logi.log("  [DEBUG L%d] PROJ: Center->Motor (%.3f, %.3f) proj on Center->Pillar = %.4f. Final Dist=%.4f\n",
//     //          laser_idx, v_cm_x, v_cm_y, projection, out_corrected_dist);

//     snprintf(buf, sizeof(buf), "L%d:OK ", laser_idx);
//     log_str += buf;

//     return true;
// }


bool PillarScanNode::validateAndPrepareLaser(int laser_idx,
                                             const pb_msgs::Struct_Modul2Data &data,
                                             double robot_x, double robot_y, double robot_th_deg,
                                             double &out_corrected_dist,
                                             Eigen::Vector2d &out_pillar_global,
                                             std::string &log_str)
{
    const auto &meas = data.laser[laser_idx]; // Ссылка на данные
    char buf[128];

    // --- ЛОГ СЫРЫХ ДАННЫХ (DEBUG) ---
    // Выводим всё, что пришло, чтобы исключить глюки парсинга
    // logi.log("  [DEBUG L%d] RAW: Dist=%.4f, Ang=%.2f, Time=%d, Stat=%d, TgtPillar=%d\n",
    //          laser_idx, meas.distance, meas.angle, meas.time, meas.status, meas.numPillar);

    // 1. Статус
    if (meas.status != 0)
    {
        // Изменил формат для единообразия в логе: [Stat: X]
        snprintf(buf, sizeof(buf), "L%d:[Stat:%d] ", laser_idx, meas.status);
        log_str += buf;
        return false;
    }

    // 2. Номер столба
    if (meas.numPillar < 0 || meas.numPillar > 3)
    {
        // Изменил формат для единообразия: [NoPillar]
        snprintf(buf, sizeof(buf), "L%d:[NoPillar] ", laser_idx);
        log_str += buf;
        return false;
    }

    // --- РАСЧЕТ ГЕОМЕТРИИ С ЛОГАМИ ---
    // (Перенесли расчет выше, чтобы знать ошибку Err даже для старых данных)

    // А. Фактическое расстояние (Вал -> Столб)
    double r_fact = meas.distance + lasers_dist_offset_box_ + pillar_radius_;

    // Б. Конфигурация мотора
    double mount_th_deg = laser_configs_[laser_idx].mount_th_deg;
    double lx = laser_configs_[laser_idx].x_loc;
    double ly = laser_configs_[laser_idx].y_loc;

    // В. Глобальная позиция мотора
    double robot_th_rad = robot_th_deg * M_PI / 180.0;

    // Поворот вектора смещения (lx, ly) на угол робота
    double dx_rot = lx * cos(robot_th_rad) - ly * sin(robot_th_rad);
    double dy_rot = lx * sin(robot_th_rad) + ly * cos(robot_th_rad);

    double mx_global = robot_x + dx_rot;
    double my_global = robot_y + dy_rot;

    // logi.log("  [DEBUG L%d] GEOM: Robot(%.3f, %.3f, %.1f°) -> MountLoc(%.3f, %.3f) -> MotorGlobal(%.3f, %.3f)\n",
    //          laser_idx, robot_x, robot_y, robot_th_deg, lx, ly, mx_global, my_global);

    // Г. Угол луча
    // angle - угол башни. mount_th - установочный. robot_th - робота.
    // Складываем всё в градусах для наглядности
    double beam_global_deg = robot_th_deg + mount_th_deg + (-meas.angle); // Знак минус делаем так как у меня моторы плюс по часовой
    double beam_global_rad = beam_global_deg * M_PI / 180.0;

    // logi.log("  [DEBUG L%d] ANGLE: Rob(%.1f) + Mnt(%.1f) + Turret(%.1f) = BeamGlobal(%.1f°)\n",
    //          laser_idx, robot_th_deg, mount_th_deg, meas.angle, beam_global_deg);

    // Д. Точка удара (Виртуальный столб)
    double hit_x = mx_global + r_fact * cos(beam_global_rad);
    double hit_y = my_global + r_fact * sin(beam_global_rad);

    // Е. Идеальный столб (Цель)
    int pid = meas.numPillar;
    // Защита от выхода за границы массива (на всякий случай)
    if (pid >= reference_centers_.size())
        return false;
    Eigen::Vector2d ideal_pillar = reference_centers_[pid].cast<double>();

    // Ж. Гейтинг (Расчет ошибки)
    double err_dist = sqrt(pow(hit_x - ideal_pillar.x(), 2) + pow(hit_y - ideal_pillar.y(), 2));

    // --- ЗАПИСЬ В ЛОГ (ДЕТАЛИ) ---
    // Выводим: Время (ms) и Ошибку (mm) в понятном формате с выравниванием
    snprintf(buf, sizeof(buf), "L%d:[%3dms, Err:%5.1fmm] ", laser_idx, meas.time, err_dist * 1000.0);
    log_str += buf;

    // --- ПРОВЕРКИ (ФИЛЬТРЫ) ---

    // 1. Проверка Гейтинга
    if (err_dist > 0.1) // Насколько центры столбов могут не совпадать и все равно пройдет столб проверку
    {
        // Мы уже записали ошибку в log_str выше, просто выходим
        return false;
    }

    // 2. Проверка Времени (Обновили порог до 333 мс)
    // Проверяем ПОСЛЕ расчета геометрии, чтобы видеть ошибку Err даже у старых данных
    if (meas.time > TIME_LIMIT_LOCAL)
    {
        // Мы уже записали время в log_str выше, просто выходим
        return false;
    }

    // --- РАСЧЕТ ПОПРАВКИ (ПРОЕКЦИЯ) ---
    double v_cm_x = mx_global - robot_x;
    double v_cm_y = my_global - robot_y;

    double v_cp_x = ideal_pillar.x() - robot_x;
    double v_cp_y = ideal_pillar.y() - robot_y;
    double len_cp = sqrt(v_cp_x * v_cp_x + v_cp_y * v_cp_y);
    if (len_cp < 0.001)
        len_cp = 0.001;
    v_cp_x /= len_cp;
    v_cp_y /= len_cp;

    double projection = v_cm_x * v_cp_x + v_cm_y * v_cp_y;
    out_corrected_dist = r_fact + projection;
    out_pillar_global = ideal_pillar;

    // logi.log("  [DEBUG L%d] PROJ: Center->Motor (%.3f, %.3f) proj on Center->Pillar = %.4f. Final Dist=%.4f\n",
    //          laser_idx, v_cm_x, v_cm_y, projection, out_corrected_dist);

    // Успех (статус OK добавлять не обязательно, так как отсутствие Fail/Old/Gate уже означает OK,
    // но можно оставить для отладки)
    // snprintf(buf, sizeof(buf), "L%d:OK ", laser_idx); 
    // log_str += buf; 

    return true;
}

/*
 * Версия: v10.0 (Full Pipeline: Multithread + Stats + Dual Solver + Fusion)
 * Дата: 2025-12-02
 */
void PillarScanNode::processPipeline(const sensor_msgs::LaserScan &scan)
{
    // 1. ЗАСЕКАЕМ ВРЕМЯ
    auto start_time = std::chrono::high_resolution_clock::now();

    logi.log_r("\n\n\n --- processPipeline ---\n");

    // 1. Статистика входа
    stats_.total_scans++;
    stats_.total_points_raw += scan.ranges.size();

    // 1. ЛОГ СЫРОГО СКАНА
    // logRawScan(scan); // 2. Логирование (внутри метода стоит защита: пишем только 1-й скан или редко)

    int points_removed_by_angle_filter = 0;
    int removed_by_zero = 0;
    int removed_by_low = 0;
    int removed_by_range = 0;   // <--- НОВОЕ: Счетчик удаления по дальности
    int removed_by_invalid = 0; // <--- НОВОЕ: Счетчик NaN/Inf

    AlignedVector2f initial_points;
    std::vector<double> point_intensities;

    bool has_intensities = (scan.intensities.size() == scan.ranges.size());
    initial_points.reserve(scan.ranges.size());

    for (size_t i = 0; i < scan.ranges.size(); ++i)
    {
        float r = scan.ranges[i];
        float intensity = has_intensities ? scan.intensities[i] : 0.0f;

        // 1. Проверка на NaN/Inf
        if (std::isnan(r) || std::isinf(r))
        {
            removed_by_invalid++; // <--- Считаем
            continue;
        }

        // 2. Проверка дальности
        if (r < min_range_filter || r > max_range_filter)
        {
            removed_by_range++; // <--- Считаем
            continue;
        }

        // 3. Проверка интенсивности
        if (has_intensities)
        {
            if (intensity == 0.0f)
            {
                removed_by_zero++;
                continue;
            }
            if (intensity < intensity_min_threshold)
            {
                removed_by_low++;
                continue;
            }
        }

        double angle = scan.angle_min + i * scan.angle_increment;
        initial_points.emplace_back((float)(r * cos(angle)), (float)(r * sin(angle)));
        point_intensities.push_back(intensity);
    }

    // Обновляем глобальную статистику
    total_rays_removed_by_zero_intensity += removed_by_zero;
    total_rays_removed_by_low_intensity += removed_by_low;
    total_rays_removed_by_initial_intensity = total_rays_removed_by_zero_intensity + total_rays_removed_by_low_intensity;

    // ЛОГ СТАТИСТИКИ (ТЕПЕРЬ ПОЛНЫЙ И КОРРЕКТНЫЙ)
    // Выводим данные за текущий скан, чтобы проверить математику
    logi.log("\n--- FILTERING STATISTICS (Current Scan) ---\n");
    logi.log("1. Total rays: %lu\n", scan.ranges.size());
    logi.log("2. Invalid (NaN/Inf): %d\n", removed_by_invalid);
    logi.log("3. Range Filter (<%.2f or >%.2f): %d\n", min_range_filter, max_range_filter, removed_by_range);
    logi.log_r("4. Zero Intensity: %d\n", removed_by_zero);
    logi.log_b("5. Low Intensity (<%.1f): %d\n", intensity_min_threshold, removed_by_low);
    logi.log("6. Points remaining: %lu\n", initial_points.size());

    // Статистика после первичной фильтрации
    stats_.total_points_filtered += initial_points.size();

    // Математика теперь: Total - Invalid - Range - Zero - Low == Remaining
    if (initial_points.empty())
    {
        scans_processed_count_++;
        return;
    }

    // 2. Угловой фильтр
    AlignedVector2f clean_points = removeEdgeArtifacts(initial_points, point_intensities, points_removed_by_angle_filter);

    // Обновляем статистику фильтра
    stats_.angle_filter_removed += points_removed_by_angle_filter;

    // logFinalFilteredScan(clean_points, point_intensities); // Логирование отфильтрованных (тоже защищено внутри метода)

    // Выводим сколько удалил угловой фильтр
    logi.log_b("7. Removed by Angle Filter: %d\n", points_removed_by_angle_filter);
    logi.log_b("8. Final clean points: %lu\n", clean_points.size());

    clean_points_results_ = clean_points;

    if (clean_points.empty())
    {
        scans_processed_count_++;
        return;
    }

    // 3. ДЕТЕКЦИЯ (ПАРАЛЛЕЛЬНО)
    std::vector<PillarCandidate> all_candidates;
    AlignedVector2f clusters_m1, clusters_m2, clusters_m3;

    // Запускаем 3 метода одновременно в разных потоках
    // std::ref гарантирует, что мы передаем ссылки на вектора clusters_mX, чтобы они заполнились

    auto future_m1 = std::async(std::launch::async, [&]()
                                { return detectGenericClustering(clean_points, jump_dist_threshold, 1, clusters_m1); });

    auto future_m2 = std::async(std::launch::async, [&]()
                                { return detectGenericClustering(clean_points, cluster_dist_threshold, 2, clusters_m2); });

    auto future_m3 = std::async(std::launch::async, [&]()
                                { return detectLocalMinima(clean_points, 3, clusters_m3); });

    // Ждем завершения и забираем результаты (.get() блокирует поток до готовности)
    auto c1 = future_m1.get();
    auto c2 = future_m2.get();
    auto c3 = future_m3.get();

    // Объединяем результаты в общий список (это делается быстро и последовательно)
    all_candidates.insert(all_candidates.end(), c1.begin(), c1.end());
    all_candidates.insert(all_candidates.end(), c2.begin(), c2.end());
    all_candidates.insert(all_candidates.end(), c3.begin(), c3.end());

    // Создаем маркеры для отладки (в основном потоке, так как ROS publish не всегда потокобезопасен)
    marker_m1_results_ = createPointsMarker(clusters_m1, scan.header.frame_id, "method_1_jump", 1, 1.0f, 0.0f, 0.0f, 0.08f);
    marker_m2_results_ = createPointsMarker(clusters_m2, scan.header.frame_id, "method_2_cluster", 2, 0.0f, 0.0f, 1.0f, 0.08f);
    marker_m3_results_ = createPointsMarker(clusters_m3, scan.header.frame_id, "method_3_minima", 3, 1.0f, 1.0f, 0.0f, 0.08f);

    // Статистика теперь выводится тут, так как методы внутри потоков могут перемешать логи
    logi.log("Total candidates found: %lu (M1=%lu, M2=%lu, M3=%lu)\n",
             all_candidates.size(), c1.size(), c2.size(), c3.size());

    // 4. FUSION (БЕЗ ИЗМЕНЕНИЙ)
    AlignedPillarVector final_pillars = fuseCandidates(all_candidates);

    AlignedVector2f current_fused_centers;
    for (const auto &fp : final_pillars)
        current_fused_centers.push_back(fp.local);
    fused_centers_results_ = current_fused_centers;

    // 5. КАЛИБРОВКА И СЛИЯНИЕ
    if (final_pillars.size() >= 3)
    {
        // А. Сортировка
        if (final_pillars.size() >= 4)
            reorderPillars(final_pillars);

        // Б. Метод 1 (Umeyama) - Основной
        bool umeyama_success = performCalibration(final_pillars);

        // В. Метод 2 (MNK) и Слияние - только если есть калибровка
        // ЗАПУСКАЕМ MNK ТОЛЬКО ЕСЛИ ТЕКУЩАЯ КАЛИБРОВКА УСПЕШНА
        if (umeyama_success)
        {
            try
            {
                performMnkCalculation(final_pillars); //
                processLasers();                      // Считаем независимо, просто чтобы получить цифры
                fuseResults();                        // Запускаем слияние (старое + лазеры добавим в сообщение внутри)
            }
            catch (const std::exception &e)
            {
                logi.log_r("MNK/Fusion Error: %s\n", e.what());
            }
        }
    }

    // 2. СЧИТАЕМ ВРЕМЯ ВЫПОЛНЕНИЯ
    auto end_time = std::chrono::high_resolution_clock::now();
    double latency = std::chrono::duration<double, std::milli>(end_time - start_time).count();

    stats_.sum_latency_ms += latency;
    if (latency > stats_.max_latency_ms)
        stats_.max_latency_ms = latency;

    // --- 7. ПЕЧАТЬ СТАТИСТИКИ (Раз в 15 секунд) ---
    if ((ros::Time::now() - stats_.last_print_time).toSec() > 15.0)
    {
        printSessionStatistics();
        stats_.last_print_time = ros::Time::now();
    }

    scans_processed_count_++;
}

/*
 * Версия: v12.0 (MAXIMUM INFORMATION REPORT)
 */
void PillarScanNode::printSessionStatistics()
{
    double run_time = (ros::Time::now() - stats_.start_time).toSec();
    long long N = stats_.total_scans;

    if (N == 0)
        return;

    logi.log_b("\n==================== [SESSION REPORT v12] ====================\n");
    logi.log("  Time: %.1f s | Scans: %lld | Freq: %.1f Hz\n",
             run_time, N, (double)N / run_time);

    // --- 1. ПРОИЗВОДИТЕЛЬНОСТЬ (НОВОЕ) ---
    logi.log("\n  [1. PERFORMANCE LATENCY]\n");
    logi.log("    Avg Latency:       %.1f ms (Calculation time per frame)\n", stats_.sum_latency_ms / N);
    logi.log("    Max Latency:       %.1f ms (Worst case lag)\n", stats_.max_latency_ms);
    // Бюджет 100мс для 10Гц
    double load_pct = (stats_.sum_latency_ms / N) / 100.0 * 100.0;
    logi.log("    CPU Load (10Hz):   %.1f%% of time budget used\n", load_pct);

    // --- 2. ВХОДНЫЕ ДАННЫЕ ---
    logi.log("\n  [2. RAW DATA QUALITY]\n");
    logi.log("    Raw Points:        %lld (avg)\n", stats_.total_points_raw / N);
    logi.log("    Filter Cut:        %lld (avg) removed by Range/Intensity\n",
             (stats_.total_points_raw - stats_.total_points_filtered) / N);
    logi.log("    Angle Artif. Cut:  %lld (avg) removed by Edge Filter\n",
             stats_.angle_filter_removed / N);
    logi.log("    Final Input:       %lld (avg) points used for detection\n",
             (stats_.total_points_filtered - stats_.angle_filter_removed) / N);

    // --- 3. ДЕТЕКЦИЯ ---
    logi.log("\n  [3. DETECTION PIPELINE]\n");
    auto print_met = [&](const char *name, long long found, long long rejected)
    {
        long long total = found + rejected;
        double rej_rate = total > 0 ? (double)rejected / total * 100.0 : 0.0;
        logi.log("    %-10s | Total: %-5lld -> Filtered: %-5lld (%4.1f%%) -> ACCEPTED: %lld\n",
                 name, total, rejected, rej_rate, found);
    };
    print_met("M1(Jump)", stats_.m1_found, stats_.m1_rejected);
    print_met("M2(Clust)", stats_.m2_found, stats_.m2_rejected);
    print_met("M3(Min)", stats_.m3_found, stats_.m3_rejected);

    // --- 4. СЦЕНА ---
    logi.log("\n  [4. SCENE & FUSION]\n");
    logi.log("    DBSCAN Noise:      %lld pts/scan (Unclustered)\n", stats_.dbscan_noise_points / N);
    logi.log("    Garbage Clusters:  %lld (Total rejected by Radius > 8cm)\n",
             stats_.clusters_rejected_radius + stats_.clusters_rejected_rmse);

    // Гибрид
    long long total_hybrid = stats_.hybrid_math_dominant + stats_.hybrid_phys_dominant;
    if (total_hybrid > 0)
    {
        double math_pct = (double)stats_.hybrid_math_dominant / total_hybrid * 100.0;
        logi.log("    Hybrid Preference: Math(Circle Fit) %.1f%% vs Phys(Median) %.1f%%\n",
                 math_pct, 100.0 - math_pct);
    }

    // --- 5. КАЛИБРОВКА (РЕЖИМЫ) ---
    logi.log("\n  [5. CALIBRATION MODES]\n");
    long long total_calib = stats_.calib_success + stats_.calib_fail;
    double success_rate = total_calib > 0 ? (double)stats_.calib_success / total_calib * 100.0 : 0.0;

    logi.log("    Success Rate:      %.1f%%\n", success_rate);
    logi.log("    Mode 4-PT (Ideal): %lld (%.1f%%)\n",
             stats_.mode_4pt_perfect,
             total_calib > 0 ? (double)stats_.mode_4pt_perfect / total_calib * 100.0 : 0);
    logi.log("    Mode RANSAC (SOS): %lld (Recovered from garbage)\n", stats_.mode_4pt_ransac);
    logi.log("    Mode 3-PT (Tri):   %lld (One pillar missing)\n", stats_.mode_3pt);

    logi.log("    Blind Spots (Lost):\n");
    logi.log("      RB: %-4lld RT: %-4lld LT: %-4lld LB: %-4lld\n",
             stats_.missing_counts["RB"], stats_.missing_counts["RT"],
             stats_.missing_counts["LT"], stats_.missing_counts["LB"]);

    // --- 6. ТОЧНОСТЬ И ФИНАЛ (ГЛАВНОЕ) ---
    logi.log("\n  [6. FINAL ACCURACY]\n");

    long long valid_samples = stats_.calib_success > 0 ? stats_.calib_success : 1;
    long long mnk_samples = stats_.mnk_count > 0 ? stats_.mnk_count : 1;

    double avg_u = (stats_.sum_rmse / valid_samples) * 1000.0;
    double avg_m = (stats_.sum_mnk_rmse / mnk_samples) * 1000.0;
    double avg_fused = (stats_.sum_fused_rmse / mnk_samples) * 1000.0; // Fused считается только когда есть оба

    logi.log("    Umeyama RMSE:      %.2f mm\n", avg_u);
    logi.log("    MNK (Tri) RMSE:    %.2f mm\n", avg_m);

    // Сравнение
    if (stats_.mnk_count > 0)
    {
        double improvement = avg_u - avg_fused;
        logi.log("    ---------------------------\n");
        logi.log("    FINAL FUSED RMSE:  %.2f mm (Theoretical Precision)\n", avg_fused);
        logi.log("    Max Fused Error:   %.2f mm (Worst case)\n", stats_.max_fused_rmse * 1000.0);
        logi.log("    ---------------------------\n");

        logi.log("    Solver Diff:       %.2f mm (Agreements)\n", (stats_.sum_mnk_diff / mnk_samples) * 1000.0);
        double win_rate = (double)stats_.mnk_better_count / mnk_samples * 100.0;
        logi.log("    MNK Win Rate:      %.1f%% (Was cleaner than Umeyama)\n", win_rate);
    }

    logi.log_b("==============================================================\n");
}
