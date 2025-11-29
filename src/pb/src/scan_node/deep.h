#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <vector>
#include <Eigen/Dense>
#include <Eigen/Geometry>
#include <cmath>
#include <algorithm>
#include <fstream>

// Структуры данных
struct Point2D {
    double x, y;
    Point2D(double x = 0, double y = 0) : x(x), y(y) {}
};

struct Circle {
    Point2D center;
    double radius;
    double rmse;
    std::vector<Point2D> points;
    int method_id;
    double weight;
    
    Circle() : radius(0), rmse(0), method_id(0), weight(0) {}
};

class PillarDetector {
private:
    ros::NodeHandle nh_;
    ros::Subscriber laser_sub_;
    std::vector<sensor_msgs::LaserScan> accumulated_scans_;
    int max_scans_ = 100;
    bool processing_ = false;
    
    // Параметры из YAML
    double pillar_diameter_;
    double pillar_radius_;
    std::vector<double> inter_pillar_distances_; // 6 расстояний между краями
    
    // Эталонные данные
    std::vector<Point2D> reference_centers_;
    
public:
    PillarDetector() : pillar_diameter_(0.315), pillar_radius_(0.1575) {
        // Инициализация
        loadParameters();
        initializeReferenceSystem();
        
        // Подписка на лидар
        laser_sub_ = nh_.subscribe("/scan", 100, &PillarDetector::laserScanCallback, this);
        
        logi("PillarDetector инициализирован. Ожидание %d сканов...", max_scans_);
    }
    
    void loadParameters() {
        // Загрузка параметров из ROS parameter server
        nh_.param<double>("/pb_config/distance/pillar_diameter", pillar_diameter_, 0.315);
        pillar_radius_ = pillar_diameter_ / 2.0;
        
        double dist;
        inter_pillar_distances_.resize(6);
        nh_.param<double>("/pb_config/distance/pillar_0_1", inter_pillar_distances_[0], 0.0);
        nh_.param<double>("/pb_config/distance/pillar_0_2", inter_pillar_distances_[1], 0.0);
        nh_.param<double>("/pb_config/distance/pillar_0_3", inter_pillar_distances_[2], 0.0);
        nh_.param<double>("/pb_config/distance/pillar_1_2", inter_pillar_distances_[3], 0.0);
        nh_.param<double>("/pb_config/distance/pillar_1_3", inter_pillar_distances_[4], 0.0);
        nh_.param<double>("/pb_config/distance/pillar_2_3", inter_pillar_distances_[5], 0.0);
        
        logi("Загружены параметры: диаметр=%.3fм, радиус=%.3fм", pillar_diameter_, pillar_radius_);
        logi("Расстояния между столбами: %.3f, %.3f, %.3f, %.3f, %.3f, %.3f", 
             inter_pillar_distances_[0], inter_pillar_distances_[1], inter_pillar_distances_[2],
             inter_pillar_distances_[3], inter_pillar_distances_[4], inter_pillar_distances_[5]);
    }
    
    void initializeReferenceSystem() {
        // Построение эталонной системы координат из измерений рулетки
        logi("Инициализация эталонной системы координат...");
        
        // Преобразование расстояний между краями в расстояния между центрами
        std::vector<double> center_distances(6);
        for (int i = 0; i < 6; i++) {
            center_distances[i] = inter_pillar_distances_[i] + pillar_diameter_;
        }
        
        // Построение координат столбов в глобальной системе
        // Столб 0 (правый нижний) в (0,0)
        reference_centers_.resize(4);
        reference_centers_[0] = Point2D(0, 0);
        
        // Столб 1 (правый верхний) на оси X
        reference_centers_[1] = Point2D(center_distances[0], 0);
        
        // Столб 2 (левый нижний) на оси Y
        reference_centers_[2] = Point2D(0, center_distances[1]);
        
        // Столб 3 (левый верхний) через пересечение окружностей
        double d01 = center_distances[0];
        double d02 = center_distances[1];
        double d12 = center_distances[3];
        
        // Используем формулу пересечения окружностей
        double x3 = (d01*d01 + d02*d02 - d12*d12) / (2*d01);
        double y3 = std::sqrt(d02*d02 - x3*x3);
        reference_centers_[3] = Point2D(x3, y3);
        
        logi("Эталонные координаты построены:");
        for (int i = 0; i < 4; i++) {
            logi("Столб %d: (%.3f, %.3f)", i, reference_centers_[i].x, reference_centers_[i].y);
        }
    }
    
    void laserScanCallback(const sensor_msgs::LaserScan::ConstPtr& scan) {
        if (processing_) {
            return; // Пропускаем сканы во время обработки
        }
        
        accumulated_scans_.push_back(*scan);
        logi("Получен скан #%d. Всего: %d/%d", 
             accumulated_scans_.size(), accumulated_scans_.size(), max_scans_);
        
        if (accumulated_scans_.size() >= max_scans_) {
            processing_ = true;
            logi("=== НАЧАЛО ОБРАБОТКИ %d СКАНОВ ===", accumulated_scans_.size());
            processAccumulatedScans();
            accumulated_scans_.clear();
            processing_ = false;
        }
    }
    
    void processAccumulatedScans() {
        // Этап 1: Преобразование и фильтрация данных
        std::vector<Point2D> combined_points = preprocessScans();
        logi("Этап 1 завершен. Объединенное облако точек: %d точек", combined_points.size());
        
        // Этап 2: Детектирование кандидатов тремя методами
        std::vector<Circle> all_candidates;
        
        std::vector<Circle> candidates_method1 = detectByGap(combined_points);
        logi("Метод 1 (Разрывы): найдено %d кандидатов", candidates_method1.size());
        
        std::vector<Circle> candidates_method2 = detectByClustering(combined_points);
        logi("Метод 2 (Кластеризация): найдено %d кандидатов", candidates_method2.size());
        
        std::vector<Circle> candidates_method3 = detectByLocalMinima(combined_points);
        logi("Метод 3 (Лок. минимумы): найдено %d кандидатов", candidates_method3.size());
        
        // Объединение кандидатов
        all_candidates.insert(all_candidates.end(), candidates_method1.begin(), candidates_method1.end());
        all_candidates.insert(all_candidates.end(), candidates_method2.begin(), candidates_method2.end());
        all_candidates.insert(all_candidates.end(), candidates_method3.begin(), candidates_method3.end());
        
        logi("Всего кандидатов от всех методов: %d", all_candidates.size());
        
        // Этап 3: Взвешенное усреднение и финальный отбор
        std::vector<Point2D> final_centers = weightedFusion(all_candidates);
        logi("Этап 3 завершен. Найдено %d финальных центров", final_centers.size());
        
        // Этап 4: Калибровка и преобразование в глобальную систему
        if (final_centers.size() == 4) {
            std::vector<Point2D> global_coords = calibrateAndTransform(final_centers);
            saveResults(global_coords);
        } else {
            logi("ОШИБКА: Найдено %d центров вместо 4. Требуется ручная проверка.", final_centers.size());
        }
        
        logi("=== ОБРАБОТКА ЗАВЕРШЕНА ===");
    }
    
    std::vector<Point2D> preprocessScans() {
        logi("Начало предобработки %d сканов...", accumulated_scans_.size());
        
        std::vector<Point2D> all_points;
        int total_points = 0;
        int filtered_points = 0;
        
        for (const auto& scan : accumulated_scans_) {
            std::vector<Point2D> scan_points = convertScanToPoints(scan);
            std::vector<Point2D> filtered_scan_points = temporalFilter(scan_points);
            
            all_points.insert(all_points.end(), filtered_scan_points.begin(), filtered_scan_points.end());
            
            total_points += scan_points.size();
            filtered_points += filtered_scan_points.size();
        }
        
        logi("Фильтрация: %d -> %d точек (%.1f%%)", 
             total_points, filtered_points, (filtered_points * 100.0) / total_points);
        
        return all_points;
    }
    
    std::vector<Point2D> convertScanToPoints(const sensor_msgs::LaserScan& scan) {
        std::vector<Point2D> points;
        
        for (size_t i = 0; i < scan.ranges.size(); i++) {
            float range = scan.ranges[i];
            
            // Фильтрация некорректных значений
            if (std::isinf(range) || std::isnan(range) || range > scan.range_max || range < scan.range_min) {
                continue;
            }
            
            // Преобразование в декартовы координаты
            float angle = scan.angle_min + i * scan.angle_increment;
            double x = range * std::cos(angle);
            double y = range * std::sin(angle);
            
            points.push_back(Point2D(x, y));
        }
        
        return points;
    }
    
    std::vector<Point2D> temporalFilter(const std::vector<Point2D>& points) {
        // Упрощенная временная фильтрация - в реальности нужна более сложная логика
        std::vector<Point2D> filtered;
        
        for (const auto& point : points) {
            // Простейший фильтр - убираем точки слишком близко и слишком далеко
            double dist = std::sqrt(point.x * point.x + point.y * point.y);
            if (dist > 0.5 && dist < 20.0) { // Настройте под вашу задачу
                filtered.push_back(point);
            }
        }
        
        return filtered;
    }
    
    // МЕТОД 1: Детектирование по разрывам
    std::vector<Circle> detectByGap(const std::vector<Point2D>& points) {
        logi("Запуск Метода 1 (Разрывы)...");
        std::vector<Circle> candidates;
        
        // Сортируем точки по углу для анализа последовательности
        std::vector<Point2D> sorted_points = points;
        std::sort(sorted_points.begin(), sorted_points.end(), 
                 [](const Point2D& a, const Point2D& b) {
                     return std::atan2(a.y, a.x) < std::atan2(b.y, b.x);
                 });
        
        // Поиск разрывов в последовательности точек
        const double GAP_THRESHOLD = 0.6; // Минимальный разрыв 0.6м
        
        for (size_t i = 1; i < sorted_points.size(); i++) {
            double dist = distance(sorted_points[i-1], sorted_points[i]);
            
            if (dist > GAP_THRESHOLD) {
                // Найден разрыв - анализируем область перед разрывом
                Point2D gap_point = sorted_points[i-1];
                
                // Собираем точки вокруг разрыва
                std::vector<Point2D> cluster_points = extractPointsAround(sorted_points, gap_point, 0.3);
                
                if (cluster_points.size() > 5) { // Минимум 5 точек для аппроксимации
                    Circle circle = fitCircle(cluster_points);
                    circle.method_id = 1;
                    circle.weight = calculateCandidateWeight(circle, cluster_points.size());
                    
                    if (isValidCircle(circle)) {
                        candidates.push_back(circle);
                    }
                }
            }
        }
        
        logi("Метод 1: проанализировано %d разрывов", candidates.size());
        return candidates;
    }
    
    // МЕТОД 2: Евклидова кластеризация
    std::vector<Circle> detectByClustering(const std::vector<Point2D>& points) {
        logi("Запуск Метода 2 (Кластеризация)...");
        std::vector<Circle> candidates;
        
        // Простая реализация кластеризации по расстоянию
        std::vector<std::vector<Point2D>> clusters = euclideanCluster(points, 0.2, 5);
        logi("Найдено кластеров: %d", clusters.size());
        
        for (const auto& cluster : clusters) {
            if (cluster.size() >= 10) { // Минимум 10 точек в кластере
                Circle circle = fitCircle(cluster);
                circle.method_id = 2;
                circle.weight = calculateCandidateWeight(circle, cluster.size());
                
                // Проверяем, что кластер похож на столб по размеру
                if (isValidCircle(circle)) {
                    candidates.push_back(circle);
                    logi("Кластер: %d точек, центр (%.3f, %.3f), радиус %.3f", 
                         cluster.size(), circle.center.x, circle.center.y, circle.radius);
                }
            }
        }
        
        return candidates;
    }
    
    // МЕТОД 3: Локальные минимумы
    std::vector<Circle> detectByLocalMinima(const std::vector<Point2D>& points) {
        logi("Запуск Метода 3 (Локальные минимумы)...");
        std::vector<Circle> candidates;
        
        // Сортируем точки по расстоянию до начала координат
        std::vector<Point2D> sorted_points = points;
        std::sort(sorted_points.begin(), sorted_points.end(),
                 [](const Point2D& a, const Point2D& b) {
                     return (a.x*a.x + a.y*a.y) < (b.x*b.x + b.y*b.y);
                 });
        
        // Поиск локальных минимумов (ближайших точек)
        const size_t WINDOW_SIZE = 10;
        
        for (size_t i = WINDOW_SIZE; i < sorted_points.size() - WINDOW_SIZE; i++) {
            bool is_minima = true;
            double current_dist = distanceToOrigin(sorted_points[i]);
            
            // Проверяем окрестность
            for (size_t j = i - WINDOW_SIZE; j <= i + WINDOW_SIZE; j++) {
                if (j != i && distanceToOrigin(sorted_points[j]) < current_dist) {
                    is_minima = false;
                    break;
                }
            }
            
            if (is_minima) {
                // Найден локальный минимум - собираем точки вокруг
                std::vector<Point2D> cluster_points = extractPointsAround(sorted_points, sorted_points[i], 0.4);
                
                if (cluster_points.size() > 5) {
                    Circle circle = fitCircle(cluster_points);
                    circle.method_id = 3;
                    circle.weight = calculateCandidateWeight(circle, cluster_points.size());
                    
                    if (isValidCircle(circle)) {
                        candidates.push_back(circle);
                    }
                }
            }
        }
        
        logi("Метод 3: найдено %d локальных минимумов", candidates.size());
        return candidates;
    }
    
    std::vector<Circle> weightedFusion(const std::vector<Circle>& candidates) {
        logi("Начало взвешенного объединения %d кандидатов...", candidates.size());
        
        if (candidates.empty()) {
            return std::vector<Circle>();
        }
        
        // Группируем кандидатов по близости (простая кластеризация)
        std::vector<std::vector<Circle>> clusters;
        std::vector<bool> processed(candidates.size(), false);
        
        const double CLUSTER_EPS = 0.5; // 50см для объединения кандидатов
        
        for (size_t i = 0; i < candidates.size(); i++) {
            if (!processed[i]) {
                std::vector<Circle> cluster;
                cluster.push_back(candidates[i]);
                processed[i] = true;
                
                // Поиск всех близких кандидатов
                for (size_t j = i + 1; j < candidates.size(); j++) {
                    if (!processed[j]) {
                        double dist = distance(candidates[i].center, candidates[j].center);
                        if (dist < CLUSTER_EPS) {
                            cluster.push_back(candidates[j]);
                            processed[j] = true;
                        }
                    }
                }
                
                clusters.push_back(cluster);
            }
        }
        
        logi("Образовано %d кластеров кандидатов", clusters.size());
        
        // Взвешенное усреднение для каждого кластера
        std::vector<Circle> fused_candidates;
        
        for (const auto& cluster : clusters) {
            if (cluster.size() >= 2) { // Объединяем только если есть несколько кандидатов
                Circle fused;
                double total_weight = 0.0;
                
                for (const auto& candidate : cluster) {
                    fused.center.x += candidate.center.x * candidate.weight;
                    fused.center.y += candidate.center.y * candidate.weight;
                    fused.radius += candidate.radius * candidate.weight;
                    total_weight += candidate.weight;
                }
                
                if (total_weight > 0) {
                    fused.center.x /= total_weight;
                    fused.center.y /= total_weight;
                    fused.radius /= total_weight;
                    fused.weight = total_weight / cluster.size(); // Средний вес
                    
                    fused_candidates.push_back(fused);
                    
                    logi("Объединенный кластер: %d кандидатов, вес=%.3f, центр=(%.3f,%.3f)", 
                         cluster.size(), fused.weight, fused.center.x, fused.center.y);
                }
            } else {
                // Одиночные кандидаты проходят как есть
                fused_candidates.push_back(cluster[0]);
            }
        }
        
        // Отбираем 4 лучших кандидата по весу
        std::sort(fused_candidates.begin(), fused_candidates.end(),
                 [](const Circle& a, const Circle& b) { return a.weight > b.weight; });
        
        if (fused_candidates.size() > 4) {
            fused_candidates.resize(4);
        }
        
        logi("После объединения: %d кандидатов", fused_candidates.size());
        return fused_candidates;
    }
    
    std::vector<Point2D> calibrateAndTransform(const std::vector<Point2D>& local_centers) {
        logi("Начало калибровки и преобразования координат...");
        
        if (local_centers.size() != 4) {
            logi("ОШИБКА: Для калибровки требуется 4 центра, получено %d", local_centers.size());
            return std::vector<Point2D>();
        }
        
        // Преобразование в матрицы Eigen для вычисления преобразования
        Eigen::MatrixXd source(4, 2);
        Eigen::MatrixXd target(4, 2);
        
        for (int i = 0; i < 4; i++) {
            source(i, 0) = local_centers[i].x;
            source(i, 1) = local_centers[i].y;
            
            target(i, 0) = reference_centers_[i].x;
            target(i, 1) = reference_centers_[i].y;
        }
        
        // Вычисление similarity-преобразования (масштаб, поворот, сдвиг)
        Eigen::Matrix2d rotation;
        double scale;
        Eigen::Vector2d translation;
        
        computeSimilarityTransform(source, target, rotation, scale, translation);
        
        logi("Преобразование: масштаб=%.6f, поворот=[%.3f,%.3f;%.3f,%.3f], сдвиг=(%.3f,%.3f)",
             scale, rotation(0,0), rotation(0,1), rotation(1,0), rotation(1,1),
             translation(0), translation(1));
        
        // Применение преобразования ко всем точкам
        std::vector<Point2D> global_coords;
        for (const auto& local_point : local_centers) {
            Eigen::Vector2d local_vec(local_point.x, local_point.y);
            Eigen::Vector2d global_vec = scale * rotation * local_vec + translation;
            global_coords.push_back(Point2D(global_vec(0), global_vec(1)));
        }
        
        // Валидация результатов
        double max_error = 0.0;
        for (int i = 0; i < 4; i++) {
            double error = distance(global_coords[i], reference_centers_[i]);
            max_error = std::max(max_error, error);
            logi("Столб %d: локальный (%.3f,%.3f) -> глобальный (%.3f,%.3f), ошибка=%.3fм",
                 i, local_centers[i].x, local_centers[i].y,
                 global_coords[i].x, global_coords[i].y, error);
        }
        
        logi("Максимальная ошибка преобразования: %.3fм", max_error);
        
        return global_coords;
    }
    
    void saveResults(const std::vector<Point2D>& global_coords) {
        logi("Сохранение результатов в ROS параметры...");
        
        for (int i = 0; i < global_coords.size(); i++) {
            std::string param_x = "/pb_config/result/pillar_" + std::to_string(i) + "_x";
            std::string param_y = "/pb_config/result/pillar_" + std::to_string(i) + "_y";
            
            nh_.setParam(param_x, global_coords[i].x);
            nh_.setParam(param_y, global_coords[i].y);
            
            logi("Столб %d: (%.6f, %.6f) -> %s, %s", 
                 i, global_coords[i].x, global_coords[i].y, param_x.c_str(), param_y.c_str());
        }
        
        logi("Результаты успешно сохранены в ROS parameter server");
    }
    
    // Вспомогательные математические функции
    double distance(const Point2D& a, const Point2D& b) {
        return std::sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
    }
    
    double distanceToOrigin(const Point2D& p) {
        return std::sqrt(p.x * p.x + p.y * p.y);
    }
    
    std::vector<Point2D> extractPointsAround(const std::vector<Point2D>& points, 
                                           const Point2D& center, double radius) {
        std::vector<Point2D> result;
        for (const auto& p : points) {
            if (distance(p, center) <= radius) {
                result.push_back(p);
            }
        }
        return result;
    }
    
    Circle fitCircle(const std::vector<Point2D>& points) {
        // Упрощенная аппроксимация окружности - в реальности используйте RANSAC
        Circle circle;
        
        // Вычисление центра как среднего арифметического
        double sum_x = 0, sum_y = 0;
        for (const auto& p : points) {
            sum_x += p.x;
            sum_y += p.y;
        }
        
        circle.center.x = sum_x / points.size();
        circle.center.y = sum_y / points.size();
        
        // Вычисление среднего радиуса
        double sum_r = 0;
        for (const auto& p : points) {
            sum_r += distance(p, circle.center);
        }
        circle.radius = sum_r / points.size();
        
        // Вычисление RMSE
        double sum_sq_error = 0;
        for (const auto& p : points) {
            double error = std::abs(distance(p, circle.center) - circle.radius);
            sum_sq_error += error * error;
        }
        circle.rmse = std::sqrt(sum_sq_error / points.size());
        
        return circle;
    }
    
    double calculateCandidateWeight(const Circle& circle, int points_count) {
        // Вычисление веса кандидата на основе нескольких факторов
        
        double weight_method = 1.0; // Базовый вес метода
        switch (circle.method_id) {
            case 1: weight_method = 0.9; break; // Метод разрывов
            case 2: weight_method = 1.0; break; // Кластеризация
            case 3: weight_method = 0.8; break; // Локальные минимумы
        }
        
        // Вес по количеству точек
        double weight_points = std::min(1.0, points_count / 20.0);
        
        // Вес по соответствию радиусу
        double radius_error = std::abs(circle.radius - pillar_radius_);
        double weight_radius = std::exp(-std::pow(radius_error / (0.5 * pillar_radius_), 2));
        
        // Вес по качеству аппроксимации
        double weight_rmse = std::exp(-circle.rmse / 0.05);
        
        return weight_method * weight_points * weight_radius * weight_rmse;
    }
    
    bool isValidCircle(const Circle& circle) {
        // Проверка валидности найденной окружности
        return circle.radius > 0.1 && circle.radius < 0.3 && // Радиус в разумных пределах
               circle.rmse < 0.1;                            // Приемлемая ошибка аппроксимации
    }
    
    std::vector<std::vector<Point2D>> euclideanCluster(const std::vector<Point2D>& points, 
                                                     double cluster_tolerance, int min_cluster_size) {
        // Упрощенная реализация евклидовой кластеризации
        std::vector<std::vector<Point2D>> clusters;
        std::vector<bool> processed(points.size(), false);
        
        for (size_t i = 0; i < points.size(); i++) {
            if (!processed[i]) {
                std::vector<Point2D> cluster;
                std::vector<size_t> queue;
                
                queue.push_back(i);
                processed[i] = true;
                
                while (!queue.empty()) {
                    size_t current_idx = queue.back();
                    queue.pop_back();
                    cluster.push_back(points[current_idx]);
                    
                    // Поиск соседей
                    for (size_t j = 0; j < points.size(); j++) {
                        if (!processed[j] && distance(points[current_idx], points[j]) < cluster_tolerance) {
                            queue.push_back(j);
                            processed[j] = true;
                        }
                    }
                }
                
                if (cluster.size() >= min_cluster_size) {
                    clusters.push_back(cluster);
                }
            }
        }
        
        return clusters;
    }
    
    void computeSimilarityTransform(const Eigen::MatrixXd& source, const Eigen::MatrixXd& target,
                                  Eigen::Matrix2d& rotation, double& scale, Eigen::Vector2d& translation) {
        // Реализация алгоритма Umeyama для similarity-преобразования
        Eigen::Vector2d source_mean = source.colwise().mean();
        Eigen::Vector2d target_mean = target.colwise().mean();
        
        Eigen::MatrixXd source_centered = source.rowwise() - source_mean.transpose();
        Eigen::MatrixXd target_centered = target.rowwise() - target_mean.transpose();
        
        double source_scale = std::sqrt((source_centered.array().square().sum()) / source.rows());
        double target_scale = std::sqrt((target_centered.array().square().sum()) / target.rows());
        
        scale = target_scale / source_scale;
        
        Eigen::Matrix2d covariance = target_centered.transpose() * source_centered / source.rows();
        
        Eigen::JacobiSVD<Eigen::Matrix2d> svd(covariance, Eigen::ComputeFullU | Eigen::ComputeFullV);
        
        rotation = svd.matrixU() * svd.matrixV().transpose();
        
        // Ensure proper rotation (determinant = 1)
        if (rotation.determinant() < 0) {
            Eigen::Matrix2d V = svd.matrixV();
            V.col(1) *= -1;
            rotation = svd.matrixU() * V.transpose();
        }
        
        translation = target_mean - scale * rotation * source_mean;
    }
    
    void logi(const char* format, ...) {
        // Заглушка для вашего логгера - замените на вызов вашего класса
        char buffer[256];
        va_list args;
        va_start(args, format);
        vsnprintf(buffer, sizeof(buffer), format, args);
        va_end(args);
        
        ROS_INFO("%s", buffer);
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "pillar_detector_node");
    ros::NodeHandle nh;
    
    PillarDetector detector;
    
    ROS_INFO("Pillar Detector node запущен");
    ros::spin();
    
    return 0;
}