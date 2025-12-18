#ifndef PILLAR_NODE_H
#define PILLAR_NODE_H
/*
 * Версия: 6.0
 * Дата: 2025-11-29
 */

#include <ros/ros.h>               // Подключение основной библиотеки ROS
#include <sensor_msgs/LaserScan.h> // Подключение типа сообщения для данных лидара
#include <ros/topic.h>             // Необходим для ros::topic::waitForMessage
#include <vector>                  // Работа с динамическими массивами
#include <cmath>                   // Математические функции
#include <string>                  // Работа со строками
#include <algorithm>               // Различные алгоритмы (включая min/max)
#include <stdarg.h>                // Для реализации printf-подобного логгера
#include <Eigen/Dense>             // Обязательно: библиотека линейной алгебры
#include <limits>                  // Для std::numeric_limits
#include <numeric>                 // Для std::accumulate
#include <deque>                   // Требуется для std::deque
#include <future>                  // <-- НУЖНО ДЛЯ ПАРАЛЛЕЛЬНОСТИ

// --- Заголовки для RViz (Только стандартные ROS) ---
#include <visualization_msgs/MarkerArray.h> // Для публикации MarkerArray
#include <visualization_msgs/Marker.h>      // Для публикации одиночного Marker
#include <geometry_msgs/Point.h>            // Для точек внутри маркеров
#include <geometry_msgs/PoseStamped.h>      // <-- ДОБАВИТЬ ЭТУ СТРОКУ
#include <pb_msgs/Struct_PoseScan.h>
#include <pb_msgs/Struct_Modul2Data.h> // Подключаем сообщение от Modul

#include "../genStruct.h"        // Тут все общие структуры. Истользуются и Data и Main и Head
#include "trilaterationSolver.h" // Файл для функций для формирования топиков в нужном виде и формате

// Константа для математических вычислений (pi)
#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif

// --------------------------------------------------------------------------------------
// 1. Вспомогательные классы и структуры (Без изменений)
// --------------------------------------------------------------------------------------

// ВНЕШНЕЕ ОБЪЯВЛЕНИЕ ЛОГГЕРА
class AsyncFileLogger;
extern AsyncFileLogger logi;

// Тип для выровненного вектора Eigen::Vector2f
using AlignedVector2f = std::vector<Eigen::Vector2f, Eigen::aligned_allocator<Eigen::Vector2f>>;

// Структура кандидата
struct PillarCandidate
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    Eigen::Vector2f center;
    double rmse;
    int num_points;
    double weight;
    int method_id;

    // НОВОЕ: Сырые точки этого кандидата (чтобы не потерять их при слиянии)
    AlignedVector2f points;
};

// Структура финального столба (Расширенная для Гибридного метода)
struct FinalPillar
{
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    std::string name;
    Eigen::Vector2f local; // Итоговый центр (выбранный алгоритмом)
    Eigen::Vector2d global;
    double total_weight = 0.0;
    int ref_index = -1;

    bool is_estimated = false;
    double estimation_confidence = 1.0;

    // --- НОВЫЕ ПОЛЯ ДЛЯ ГИБРИДНОГО МЕТОДА ---

    // 1. Сырые данные
    AlignedVector2f merged_points; // Общее облако точек от всех методов

    // 2. Метод "Math" (Global FitCircle)
    Eigen::Vector2f fit_center; // Центр по общему фиту
    double fit_radius = 0.0;    // Радиус по общему фиту
    double fit_rmse = 0.0;      // Ошибка фита

    // !!! ВОТ ЭТИХ ПОЛЕЙ НЕ ХВАТАЛО !!!
    double math_dist = 0.0;  // Дистанция по формуле круга
    double math_angle = 0.0; // Угол на центр круга

    // 3. Метод "Phys" (Статистика + Реконструкция)
    double phys_dist = 0.0;      // Медианное расстояние до поверхности
    double phys_angle = 0.0;     // Средний угол на центр
    Eigen::Vector2f phys_center; // Центр, реконструированный через (Dist + Config_R)
    int phys_points_count = 0;   // Количество точек, прошедших фильтр

    // 4. Решение
    std::string chosen_method;   // "FIT" или "PHYS" (для лога)
    std::string decision_reason; // Объяснение (для лога)
};

using AlignedPillarVector = std::vector<FinalPillar, Eigen::aligned_allocator<FinalPillar>>; // Тип для выровненного вектора FinalPillar Мы указываем контейнеру использовать специальный аллокатор Eigen

// Класс математических утилит (Без изменений)
class MathUtils
{
public:
    static double dist2D(const Eigen::Vector2f &p1, const Eigen::Vector2f &p2)
    {
        return (p1 - p2).norm();
    }

    static bool fitCircle(const AlignedVector2f &points, double expected_radius,
                          Eigen::Vector2f &out_center, double &out_rmse)
    {
        size_t n = points.size();
        if (n < 3)
            return false;

        Eigen::MatrixXd A(n, 3);
        Eigen::VectorXd B(n);

        for (size_t i = 0; i < n; ++i)
        {
            double x = (double)points[i].x();
            double y = (double)points[i].y();

            A(i, 0) = 2.0 * x;
            A(i, 1) = 2.0 * y;
            A(i, 2) = 1.0;
            B(i) = x * x + y * y;
        }

        Eigen::Vector3d sol = A.bdcSvd(Eigen::ComputeThinU | Eigen::ComputeFullV).solve(B);

        out_center.x() = (float)sol(0);
        out_center.y() = (float)sol(1);

        double r_sq = sol(2) + sol(0) * sol(0) + sol(1) * sol(1);
        double r_fit = (r_sq > 0) ? sqrt(r_sq) : expected_radius;

        double sum_sq = 0;
        for (const auto &p : points)
        {
            double d = dist2D(p, out_center);
            sum_sq += pow(d - r_fit, 2);
        }
        out_rmse = sqrt(sum_sq / n);
        return true;
    }

    static double getMedian(std::vector<double> &values)
    {
        if (values.empty())
            return 0.0;
        size_t n = values.size();
        size_t median_idx = n / 2;

        std::nth_element(values.begin(), values.begin() + median_idx, values.end());

        if (n % 2 != 0)
        {
            return values[median_idx];
        }
        else
        {
            double v2 = values[median_idx];
            double v1 = *std::max_element(values.begin(), values.begin() + median_idx);

            return (v1 + v2) / 2.0;
        }
    }
};

// --------------------------------------------------------------------------------------
// 2. Основной Класс Ноды
// --------------------------------------------------------------------------------------

class PillarScanNode
{
public:
    PillarScanNode(); // Конструктор
    void init();      // Инициализация (перенесли сюда тяжелую логику)
    void process();   // Метод обработки (вызываем в цикле)

private:
    ros::NodeHandle nh;
    ros::Subscriber scan_sub;
    ros::Timer publish_timer_;

    // --- Паблишеры ---
    ros::Publisher pub_filtered_scan;

    ros::Publisher pub_method_1; // Jump (Красный)
    ros::Publisher pub_method_2; // Cluster (Синий)
    ros::Publisher pub_method_3; // Minima (Желтый)

    ros::Publisher pub_fused_pillars;
    ros::Publisher pub_final_markers;
    ros::Publisher pub_calib_result;  // <-- НОВЫЙ ПАБЛИШЕР
    ros::Publisher pub_mnk_result;    // <--- НОВЫЙ: MNK (Trilateration)
    ros::Publisher pub_fused_result;  // Не забудь добавить в конструктор: nh.advertise...("/pb/scan/fused_pose", 1);
    ros::Publisher pub_custom_struct; // Наш новый супер-топик

    // --- Параметры ---
    double pillar_diam_;
    double pillar_radius_;
    double d_surf[6];
    double d_center[6];
    double min_range_filter;
    double max_range_filter;
    double neighbor_radius_filter;
    int min_neighbors_filter;
    double jump_dist_threshold;
    double cluster_dist_threshold;
    double rmse_max_tolerance;
    int n_max_points_norm;
    double w_method[4];

    // --- Параметры кластеризации (НОВЫЕ, v5.8) ---
    double min_cluster_width_; // Минимальная физическая ширина кластера [м]
    double max_cluster_width_; // Максимальная физическая ширина кластера [м]
    int min_cluster_points_;   // Минимальное количество точек в кластере (НОВЫЙ, v5.9)

    // --- Параметры DBSCAN Fusion (НОВЫЙ, v6.1) ---
    double fusion_group_radius;
    int min_dbscan_points_; // minPts для DBSCAN-слияния

    // ПАРАМЕТРЫ ДЛЯ ФИЛЬТРАЦИИ ХВОСТОВ (v4.4)
    double intensity_min_threshold;
    double edge_angle_threshold;

    AlignedVector2f reference_centers_;

    // --- ПЕРЕМЕННЫЕ ДЛЯ ЛОГИКИ (ИЗМЕНЕНО) ---
    sensor_msgs::LaserScan current_scan_; // Храним последний скан
    bool new_scan_available_;             // Флаг: пришли новые данные
    bool initialized_;                    // Флаг: init() выполнен
    sensor_msgs::LaserScan meta_scan;     // Для заголовков (frame_id)
    long long scans_processed_count_;     // <--- ДОБАВИТЬ ЭТУ СТРОКУ (Счетчик сканов)

    // Глобальный счетчик для лучей, отброшенных по интенсивности за все 100 сканов (v4.7)
    long long total_rays_removed_by_zero_intensity;
    long long total_rays_removed_by_low_intensity;
    long long total_rays_removed_by_initial_intensity;

    bool calibration_done_;
    AlignedPillarVector final_pillars_results_;

    // Хранение промежуточных результатов для постоянной публикации
    sensor_msgs::LaserScan filtered_scan_results_;

    // ИЗМЕНЕНО: Вместо массива маркеров храним каждый маркер отдельно
    visualization_msgs::Marker marker_m1_results_;
    visualization_msgs::Marker marker_m2_results_;
    visualization_msgs::Marker marker_m3_results_;

    AlignedVector2f fused_centers_results_;
    AlignedVector2f clean_points_results_; // <-- НОВОЕ: Для сохранения точек после углового фильтра

    // Объект твоего решателя
    std::unique_ptr<TrilaterationSolver> mnk_solver_; // Инициализируем его в конструкторе, передавая начальную точку (0,0)
    geometry_msgs::PoseStamped mnk_pose_result_;      // Результаты MNK для слияния
    double mnk_rmse_result_ = 0.0;
    double mnk_yaw_deg_result_ = 0.0; // <--- НОВОЕ ПОЛЕ

    // --- НОВОЕ: Лазерные дальномеры ---
    ros::Subscriber sub_modul;                  // Подписчик на данные модуля
    pb_msgs::Struct_Modul2Data last_modul_msg_; // Хранение последнего сообщения
    std::mutex modul_mutex_;                    // Мьютекс для защиты данных
    bool has_modul_data_ = false;               // Флаг, что данные вообще приходили

    // Параметры конфигурации лазеров
    double lasers_dist_offset_box_; // 0.042
    double lasers_motor_bias_;      // 0.0636... (половина стороны)

    ros::Time last_modul_rcv_time_; // Время получения последнего пакета ROS

    // Структура для хранения геометрии мотора
    struct LaserMountConfig
    {
        double x_loc;        // Локальный X относительно центра
        double y_loc;        // Локальный Y относительно центра
        double mount_th_deg; // Угол установки корпуса мотора
    };
    LaserMountConfig laser_configs_[4]; // Конфигурация 4-х моторов

    // Результаты расчета "Только Лазеры" (для вывода в топик и лог)
    struct LaserResult
    {
        double x = 0.0;
        double y = 0.0;
        double rmse = 0.0;
        int used_count = 0;
        bool valid = false;
    } laser_only_result_;

    // --- НОВЫЕ МЕТОДЫ ---
    void modulCallback(const pb_msgs::Struct_Modul2Data::ConstPtr &msg);
    void processLasers(); // Основной метод обработки лазеров

    // Вспомогательный метод валидации одного измерения
    bool validateAndPrepareLaser(int laser_idx,
                                 const pb_msgs::Struct_Modul2Data &data,
                                 double robot_x, double robot_y, double robot_th_deg,
                                 double &out_corrected_dist,
                                 Eigen::Vector2d &out_pillar_global,
                                 std::string &log_str); // <--- НОВЫЙ АРГУМЕНТ

    const std::vector<std::string> PILLAR_NAMES = {"RB", "RT", "LT", "LB"}; // Имена столбов для удобства

    // Структура для хранения калибровки лидара
    struct LidarCalibration
    {
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW // <--- ОБЯЗАТЕЛЬНО ДОБАВИТЬ ЭТУ СТРОКУ
            double scale_factor = 1.0;
        double rotation_deg = 0.0;
        Eigen::Vector2d position = Eigen::Vector2d(0, 0);
        Eigen::Matrix2d rotation_matrix = Eigen::Matrix2d::Identity();
        double rmse = 0.0; // <--- НОВОЕ ПОЛЕ

        void clear()
        {
            scale_factor = 1.0;
            rotation_deg = 0.0;
            position = Eigen::Vector2d(0, 0);
            rotation_matrix = Eigen::Matrix2d::Identity();
        }
    };

    LidarCalibration lidar_calibration_; //  для хранения калибровки

    // --- СТАТИСТИКА (Бортовой самописец) ---
    struct SessionStats
    {
        ros::Time start_time;
        ros::Time last_print_time;

        // 1. Входные данные
        long long total_scans = 0;
        long long total_points_raw = 0;
        long long total_points_filtered = 0; // После дальности/интенсивности
        long long angle_filter_removed = 0;  // Удалено угловым фильтром

        // 2. Детекция (Atomic для многопоточности)
        std::atomic<long long> m1_found{0};
        std::atomic<long long> m1_rejected{0};
        std::atomic<long long> m2_found{0};
        std::atomic<long long> m2_rejected{0};
        std::atomic<long long> m3_found{0};
        std::atomic<long long> m3_rejected{0};

        // 3. Fusion & DBSCAN
        long long dbscan_noise_points = 0;
        long long clusters_rejected_radius = 0; // Самый важный показатель мусора
        long long clusters_rejected_rmse = 0;
        long long scans_4_pillars = 0;
        long long scans_3_pillars = 0;
        long long scans_bad_count = 0; // <3 или >4

        // 4. Калибровка (Режимы)
        long long calib_success = 0;
        long long calib_fail = 0;
        long long mode_4pt_perfect = 0;
        long long mode_4pt_ransac = 0; // Спасения
        long long mode_3pt = 0;

        // 5. Точность
        double sum_rmse = 0.0;
        double max_rmse = 0.0;
        double sum_mnk_diff = 0.0; // Накопленная разница между методами
        long long mnk_count = 0;

        // 6. Пропуски столбов (Blind Spots)
        std::map<std::string, long long> missing_counts;

        long long hybrid_math_dominant = 0; // Сколько раз победил fitCircle
        long long hybrid_phys_dominant = 0; // Сколько раз победила Медиана

        double sum_mnk_rmse = 0.0;      // Накопленная ошибка MNK
        long long mnk_better_count = 0; // Сколько раз MNK был точнее Umeyama

        // Итоговое качество
        double sum_fused_rmse = 0.0; // Накопленная ошибка после слияния
        double max_fused_rmse = 0.0; // Худшая ошибка слияния

        // Производительность
        double sum_latency_ms = 0.0; // Накопленное время обработки (мс)
        double max_latency_ms = 0.0; // Максимальное время обработки (мс)
    };

    SessionStats stats_; // Экземпляр статистики

    //*********************

private:
    // --- Методы (Только объявления) ---
    void loadParameters();
    void initReferenceSystem();
    void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan);
    void publishVisuals();
    void processPipeline(const sensor_msgs::LaserScan &scan);

    // Утилиты
    void logRawScan(const sensor_msgs::LaserScan &scan);
    void logFinalFilteredScan(const AlignedVector2f &points, const std::vector<double> &intensities);
    void publishFinalMarkers(const AlignedPillarVector &pillars);
    visualization_msgs::Marker createPointsMarker(const AlignedVector2f &points, const std::string &frame_id, const std::string &ns, int id, float r, float g, float b, float scale);

    // Алгоритмы
    AlignedVector2f removeEdgeArtifacts(const AlignedVector2f &points, const std::vector<double> &intensities, int &points_removed_by_angle_filter); // Удаление фантомных точек (хвостов) с помощью углового фильтра
    std::vector<PillarCandidate> detectGenericClustering(const AlignedVector2f &pts, double threshold, int method_id, AlignedVector2f &out_cluster_points);
    std::vector<PillarCandidate> detectLocalMinima(const AlignedVector2f &pts, int method_id, AlignedVector2f &out_cluster_points);
    bool processCluster(const AlignedVector2f &cluster, int method_id, std::vector<PillarCandidate> &out, AlignedVector2f &out_cluster_points);
    AlignedPillarVector fuseCandidates(const std::vector<PillarCandidate> &candidates);
    void calculatePillarMetrics(FinalPillar &pillar);
    void reorderPillars(AlignedPillarVector &pillars);

    // Калибровка
    bool performCalibration(AlignedPillarVector &pillars);
    bool performCalibrationFourPillars(AlignedPillarVector &pillars);
    bool performCalibrationThreePillars(AlignedPillarVector &pillars);
    void selectBestFourPillars(AlignedPillarVector &pillars);

    void performMnkCalculation(const AlignedPillarVector &pillars);
    void fuseResults();
    void saveCalibrationParameters();

    void publishMarkerInArray(const visualization_msgs::Marker &marker, ros::Publisher &pub);                                          // НОВАЯ ФУНКЦИЯ: publishMarkerInArray (Без изменений)
    visualization_msgs::MarkerArray createClusterMarkers(const std::vector<PillarCandidate> &candidates, const std::string &frame_id); // НОВАЯ ФУНКЦИЯ: createClusterMarkers (Без изменений)
    void publishResultsTimerCallback(const ros::TimerEvent &event);                                                                    // ИЗМЕНЕНА: publishResultsTimerCallback (v5.6)
    Eigen::Vector2d lidarToWorld(const Eigen::Vector2d &lidar_point);                                                                  /* * Преобразование координат из системы лидара в мировую систему с применением калибровочных параметров */
    Eigen::Vector2d worldToLidar(const Eigen::Vector2d &world_point);                                                                  /* * Преобразование координат из мировой системы в систему лидара * (обратное преобразование) */
    void saveResults(const AlignedPillarVector &pillars);
    // Метод для печати (объявление)
    void printSessionStatistics();
};

#endif