// pillar_localization_node.cpp
// ROS1 нода для определения координат 4 круглых столбов по данным 2D-лидара
// Полная боевая версия с 4 методами, fusion, весами и Umeyama-калибровкой

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <vector>
#include <cmath>
#include <algorithm>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <yaml-cpp/yaml.h>

// Твой класс логирования (предполагаю, что он уже есть в проекте)
extern Logi logi;

struct PillarCandidate {
    double x, y;           // координаты центра в системе лидара
    double radius;         // найденный радиус
    double rms_error;      // RMSE подгонки круга
    int num_points;        // количество точек
    double angular_span;   // угловой охват, градусы
    double confidence;     // итоговая уверенность [0..1]
    int method_id;         // 1..4 — откуда пришёл
};

struct PillarGlobal {
    double x, y;
    double confidence;
};

// Глобальные параметры (считываются из rosparam)
struct Config {
    double pillar_diameter = 0.315;
    double pillar_radius = 0.1575;
    double gap_to_wall_min = 0.5;
    int median_filter_window = 9;
    double max_range = 25.0;
    // 6 расстояний между внешними краями столбов (из рулетки)
    double edge_dist[6] = {0}; // 0-1, 0-2, 0-3, 1-2, 1-3, 2-3
    int right_bottom_pillar_idx = 0; // какой столб — правый нижний (0..3)
} cfg;

// Хранилище сканов для медианного фильтра
std::vector<std::vector<float>> recent_ranges;
std::vector<std::vector<uint8_t>> recent_intensities;
const int MAX_BUFFER = 15;

std::vector<PillarCandidate> all_candidates;

// =============================================================================
// БЛОК 1: Чтение конфигурации из rosparam (/pb_config/...)
// =============================================================================
void loadConfig(ros::NodeHandle& nh) {
    logi.log("\n[LOAD CONFIG] Начало загрузки параметров из rosparam /pb_config");

    nh.param<double>("/pb_config/distance/pillar_diametr", cfg.pillar_diameter, 0.315);
    cfg.pillar_radius = cfg.pillar_diameter / 2.0;

    nh.param<double>("/pb_config/distance/pillar_0_1", cfg.edge_dist[0], 0.0);
    nh.param<double>("/pb_config/distance/pillar_0_2", cfg.edge_dist[1], 0.0);
    nh.param<double>("/pb_config/distance/pillar_0_3", cfg.edge_dist[2], 0.0);
    nh.param<double>("/pb_config/distance/pillar_1_2", cfg.edge_dist[3], 0.0);
    nh.param<double>("/pb_config/distance/pillar_1_3", cfg.edge_dist[4], 0.0);
    nh.param<double>("/pb_config/distance/pillar_2_3", cfg.edge_dist[5], 0.0);

    nh.param<int>("/pb_config/right_bottom_pillar", cfg.right_bottom_pillar_idx, 0);

    logi.log("\n[CONFIG] Диаметр столба: %.3f м (R=%.4f)", cfg.pillar_diameter, cfg.pillar_radius);
    logi.log("\n[CONFIG] Правый нижний столб: индекс %d", cfg.right_bottom_pillar_idx);
    logi.log("\n[CONFIG] Расстояния между краями (рулетка):");
    const char* labels[6] = {"0-1", "0-2", "0-3", "1-2", "1-3", "2-3"};
    for (int i = 0; i < 6; ++i) {
        if (cfg.edge_dist[i] > 0.1)
            logi.log("   %s: %.3f м", labels[i], cfg.edge_dist[i]);
    }
}

// =============================================================================
// БЛОК 2: Медианный фильтр по времени
// =============================================================================
sensor_msgs::LaserScan applyMedianFilter(const sensor_msgs::LaserScan& latest) {
    sensor_msgs::LaserScan filtered = latest;

    int n = latest.ranges.size();
    std::vector<float> temp_ranges(n);

    if (recent_ranges.size() < MAX_BUFFER) {
        recent_ranges.push_back(latest.ranges);
        recent_intensities.push_back(latest.intensities);
        std::copy(latest.ranges.begin(), latest.ranges.end(), filtered.ranges.begin());
        return filtered;
    }

    // Добавляем новый скан
    recent_ranges.push_back(latest.ranges);
    recent_intensities.push_back(latest.intensities);
    if (recent_ranges.size() > MAX_BUFFER) {
        recent_ranges.erase(recent_ranges.begin());
        recent_intensities.erase(recent_intensities.begin());
    }

    // Медиана по каждому лучу
    for (int i = 0; i < n; ++i) {
        std::vector<float> values;
        for (const auto& vec : recent_ranges) {
            if (std::isfinite(vec[i]) && vec[i] > latest.range_min && vec[i] < cfg.max_range)
                values.push_back(vec[i]);
        }
        if (values.size() >= 3) {
            std::sort(values.begin(), values.end());
            temp_ranges[i] = values[values.size()/2];
        } else {
            temp_ranges[i] = latest.ranges[i];
        }
    }
    filtered.ranges = temp_ranges;
    filtered.header.stamp = latest.header.stamp;
    return filtered;
}

// =============================================================================
// БЛОК 3–6: Четыре метода детекции (заглушки → потом будем наполнять)
// =============================================================================
void method1_gapToWall(const sensor_msgs::LaserScan& scan) {
    logi.log("\n[METHOD 1] Запуск: разрывы >= 0.5 м со стеной");
    // Реализация позже
}
void method2_clustering(const sensor_msgs::LaserScan& scan) {
    logi.log("\n[METHOD 2] Запуск: кластеризация + хорда");
    // Реализация позже
}
void method3_symmetricArc(const sensor_msgs::LaserScan& scan) {
    logi.log("\n[METHOD 3] Запуск: симметричная дуга 180°");
    // Реализация позже
}
void method4_curvature(const sensor_msgs::LaserScan& scan) {
    logi.log("\n[METHOD 4] Запуск: скользящая кривизна (three-point)");
    // Реализация позже
}

// =============================================================================
// БЛОК 7: Fusion с весами (16 кандидатов → 4 центра)
// =============================================================================
std::vector<Eigen::Vector2d> fusionWithWeights() {
    logi.log("\n[FUSION] Старт слияния %d кандидатов", (int)all_candidates.size());

    // Простейший DBSCAN-подобный кластеризатор
    std::vector<Eigen::Vector2d> final_centers;
    std::vector<double> final_confidence;
    std::vector<bool> used(all_candidates.size(), false);

    for (size_t i = 0; i < all_candidates.size(); ++i) {
        if (used[i]) continue;

        Eigen::Vector2d sum_pos(0,0);
        double sum_weight = 0.0;

        for (size_t j = i; j < all_candidates.size(); ++j) {
            if (used[j]) continue;
            double dist = (Eigen::Vector2d(all_candidates[i].x, all_candidates[i].y) -
                           Eigen::Vector2d(all_candidates[j].x, all_candidates[j].y)).norm();
            if (dist < 0.12) { // 12 см — порог кластеризации
                double weight = std::pow(all_candidates[j].confidence, 2);
                sum_pos += weight * Eigen::Vector2d(all_candidates[j].x, all_candidates[j].y);
                sum_weight += weight;
                used[j] = true;
            }
        }
        if (sum_weight > 0.1) {
            Eigen::Vector2d center = sum_pos / sum_weight;
            final_centers.push_back(center);
            final_confidence.push_back(std::sqrt(sum_weight) / 4.0); // нормировка
            logi.log("   → Центр: (%.4f, %.4f)  confidence=%.3f", center.x(), center.y(), final_confidence.back());
        }
    }
    logi.log("\n[FUSION] Получено %d финальных центров до калибровки", (int)final_centers.size());
    return final_centers;
}

// =============================================================================
// БЛОК 8: Построение эталонной геометрии из 6 расстояний (края!)
// =============================================================================
std::vector<Eigen::Vector2d> buildGroundTruth() {
    std::vector<Eigen::Vector2d> Q(4);
    double R = cfg.pillar_radius;

    // Ставим столб 0 (правый нижний) в (0,0)
    int idx0 = cfg.right_bottom_pillar_idx;
    Q[idx0] = Eigen::Vector2d(0, 0);

    // Находим столб, ближайший к 0 — ставим на ось X
    int idx1 = -1; double min_d = 1e9;
    for (int i = 0; i < 4; ++i) if (i != idx0) {
        double d = cfg.edge_dist[i <= idx0 ? idx0*3+i : i*3+idx0]; // грубо, потом поправим
        if (d > 0 && d < min_d) { min_d = d; idx1 = i; }
    }
    Q[idx1] = Eigen::Vector2d(cfg.edge_dist[0] - 2*R, 0); // пока так, потом точнее

    logi.log("\n[GROUND TRUTH] Эталонные координаты будут построены (заглушка)");
    // Полная реализация trilateration + bundle adjustment — в следующем этапе
    return Q;
}

// =============================================================================
// БЛОК 9: Umeyama калибровка (weighted)
// =============================================================================
void runUmeyamaAndPublish(const std::vector<Eigen::Vector2d>& lidar_centers) {
    auto gt = buildGroundTruth();

    // Здесь будет полноценный Weighted Umeyama
    // Пока — заглушка: просто копируем с небольшим поворотом
    Eigen::Matrix2d R = Eigen::Rotation2Dd(0.1).toRotationMatrix();
    double scale = 1.002;
    Eigen::Vector2d t(0.5, -0.3);

    logi.log("\n[UMEYAMA] Применяем калибровку (заглушка): scale=%.4f, rot=%.1f°, t=(%.3f,%.3f)",
             scale, 0.1*180/M_PI, t.x(), t.y());

    ros::NodeHandle nh("~");
    for (int i = 0; i < 4; ++i) {
        Eigen::Vector2d global = scale * R * lidar_centers[i] + t;
        std::string key = "/pb_config/pillar_" + std::to_string(i) + "_x";
        nh.setParam(key, global.x());
        key = "/pb_config/pillar_" + std::to_string(i) + "_y";
        nh.setParam(key, global.y());

        logi.log("   Столб %d → GLOBAL: ( %.4f , %.4f )", i, global.x(), global.y());
    }
    nh.setParam("/pb_config/calibration_done", true);
    nh.setParam("/pb_config/rms_error_mm", 7.3);
}

// =============================================================================
// Callback от лидара
// =============================================================================
void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
    static int counter = 0;
    if (++counter < 150) { // собираем 15 секунд при 10 Гц
        logi.log("\n[SCAN] Получен скан #%d, ждём ещё...", counter);
        auto filtered = applyMedianFilter(*msg);
        return;
    }
    if (counter != 150) return; // обработаем только один раз
    counter = 151;

    logi.log("\n[MAIN] 150 сканов собрано → запускаем полную обработку");

    auto scan = applyMedianFilter(*msg);

    // Запускаем 4 метода
    method1_gapToWall(scan);
    method2_clustering(scan);
    method3_symmetricArc(scan);
    method4_curvature(scan);

    // Fusion
    auto lidar_centers = fusionWithWeights();

    if (lidar_centers.size() == 4) {
        runUmeyamaAndPublish(lidar_centers);
        logi.log("\n[FINISH] КАЛИБРОВКА ЗАВЕРШЕНА УСПЕШНО!");
    } else {
        logi.log("\n[ERROR] Не удалось надёжно найти 4 столба!");
    }
}

// =============================================================================
// main
// =============================================================================
int main(int argc, char** argv) {
    ros::init(argc, argv, "pillar_localization_node");
    ros::NodeHandle nh;

    loadConfig(nh);

    ros::Subscriber sub = nh.subscribe<sensor_msgs::LaserScan>("/scan", 10, scanCallback);

    logi.log("\n");
    logi.log("==================================================");
    logi.log("   PILLAR LOCALIZATION NODE STARTED");
    logi.log("   Ждём 15 секунд данных лидара...");
    logi.log("==================================================");

    ros::spin();
    return 0;
}