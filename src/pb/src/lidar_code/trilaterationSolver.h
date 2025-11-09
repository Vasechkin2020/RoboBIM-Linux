#include <stdio.h> 
#include <math.h> 
#include <vector> 
#include <cmath> 
#include <stdexcept> 
#include <algorithm> 

// Макрос для константы Пи
#ifndef M_PI
#define M_PI 3.14159265358979323846 // Определение числа Пи
#endif

// --- Вспомогательные структуры и функции ---
struct SPoint // Структура для хранения точки
{
    double x; // Координата x
    double y; // Координата y
}; 

struct SCircle // Структура для хранения окружности (Маяк)
{
    double x; // Координата x центра
    double y; // Координата y центра
    double r; // Радиус
}; 

double sqr(double val) 
{
    return val * val; // Квадрат числа
} 

double DEG2RAD(double deg) 
{
    return deg * M_PI / 180.0; // Градусы в радианы
} 

double RAD2DEG(double rad) 
{
    return rad * 180.0 / M_PI; // Радианы в градусы
} 

double ctan(double rad) 
{
    return 1.0 / tan(rad); // Котангенс
} 

// --- Основной класс решателя ---

class TrilaterationSolver
{
private:
    struct SPoint A_prev; // Предыдущая (или начальная) точка для выбора дуги
    std::vector<SCircle> all_circles; // Список всех окружностей (из расстояний и углов)

    double get_azimuth_deg(struct SPoint P_from, struct SPoint P_to) // Расчет азимута от P_from до P_to
    {
        // Используется стандартная математическая конвенция atan2
        double rad = atan2(P_to.y - P_from.y, P_to.x - P_from.x); // Азимут в радианах
        return RAD2DEG(rad); // Азимут в градусах
    } 

    double normalize_angle_deg(double angle_deg) // Нормализация угла в диапазон (-180, 180]
    {
        while (angle_deg > 180.0) 
        {
            angle_deg -= 360.0; // Если угол > 180, вычитаем 360
        } 
        while (angle_deg <= -180.0) 
        {
            angle_deg += 360.0; // Если угол <= -180, прибавляем 360
        } 
        return angle_deg; // Возвращаем нормализованный угол
    } 

public:
    TrilaterationSolver(struct SPoint prev) : A_prev(prev) 
    {
    } // Конструктор с начальной точкой

    int get_circle_count() const 
    { 
        return all_circles.size(); // Получить количество окружностей
    } 

    void add_circle_from_distance(struct SPoint P_beacon, double distance) // Добавление окружности по расстоянию
    {
        all_circles.push_back({P_beacon.x, P_beacon.y, distance}); // Добавляем маяк как центр с измеренным радиусом
    } 

    void add_filtered_circle_from_angle // Добавление окружности, полученной из измеренного угла
    (
        struct SPoint P1,         // Точка 1 (начало хорды)
        struct SPoint P2,         // Точка 2 (конец хорды)
        double angle_deg          // Измеренный угол (АБСОЛЮТНОЕ значение)
    )
    {
        // Угол должен быть положительным модулем для корректного расчета R и h
        double alpha_rad = DEG2RAD(fabs(angle_deg)); // Модуль угла в радианах
        double sin_alpha = sin(alpha_rad); // Синус угла
        
        if (fabs(sin_alpha) < 1e-7) // Проверка на вырожденный случай (угол ~ 0 или ~ 180)
        {
            throw std::invalid_argument("Error: Angle is too close to 0 or 180."); // Error: Angle is too close to 0 or 180
        } 

        double cot_alpha = ctan(alpha_rad); // Котангенс угла
        double dx = P2.x - P1.x; // Разница по X
        double dy = P2.y - P1.y; // Разница по Y
        double d_sq = sqr(dx) + sqr(dy); // Квадрат длины хорды P1-P2
        
        if (d_sq < 1e-12) // Проверка на совпадение точек
        {
            throw std::invalid_argument("Error: Chord points P1 and P2 coincide."); // Error: Chord points P1 and P2 coincide
        } 

        double d = sqrt(d_sq); // Длина хорды
        double AM = d / 2.0; // Половина длины хорды
        
        double h = AM * cot_alpha; // Расстояние от середины хорды до центра окружности
        
        double R = d / (2.0 * fabs(sin_alpha)); // Радиус окружности

        struct SPoint M = {(P1.x + P2.x) / 2.0, (P1.y + P2.y) / 2.0}; // Середина хорды
        double vx = -dy / d; // Вектор, перпендикулярный хорде (x-компонента)
        double vy = dx / d; // Вектор, перпендикулярный хорде (y-компонента)

        // Два потенциальных центра окружности
        struct SPoint O1 = {M.x + h * vx, M.y + h * vy}; 
        struct SPoint O2 = {M.x - h * vx, M.y - h * vy}; 
        
        // --- ВОССТАНОВЛЕННАЯ ЛОГИКА ФИЛЬТРАЦИИ ПО A_prev ---
        double dist1 = sqrt(sqr(O1.x - A_prev.x) + sqr(O1.y - A_prev.y)); // Расстояние до центра O1
        double dist2 = sqrt(sqr(O2.x - A_prev.x) + sqr(O2.y - A_prev.y)); // Расстояние до центра O2

        struct SCircle chosen_circle; // Выбранная окружность
        if (dist1 < dist2) 
        {
            chosen_circle = {O1.x, O1.y, R}; // Выбираем O1, если он ближе к A_prev
        } 
        else 
        {
            chosen_circle = {O2.x, O2.y, R}; // Выбираем O2, если он ближе
        } 
        // --- КОНЕЦ ВОССТАНОВЛЕННОЙ ЛОГИКИ ---

        all_circles.push_back(chosen_circle); // Добавляем выбранную окружность
    } 

    // --- МЕТОД 3: МНК-РЕШАТЕЛЬ ПОЛОЖЕНИЯ С RMSE ---
    struct SPoint find_A_by_mnk()
    {
        struct SPoint A = {0.0, 0.0}; // Искомая точка A
        int N = all_circles.size(); // Общее количество окружностей
        
        printf("\n--- POS LSQ SOLVER: START (N=%d) ---\n", N); // Output start of LSQ solver
        
        if (N < 3) // Проверка минимального количества уравнений
        {
            printf("Error: At least 3 circles are required.\n"); // Error: At least 3 circles are required
            return A;
        } 

        const struct SCircle& base = all_circles[0]; // Выбираем первую окружность как базовую
        double base_term = sqr(base.x) + sqr(base.y) - sqr(base.r); // Part C_0

        double M_xx = 0.0, M_xy = 0.0, M_yy = 0.0; // Elements of the normal equation matrix M^T*M
        double V_x = 0.0, V_y = 0.0; // Elements of the right-hand side V = M^T*C

        // 1. Накопление матриц (Линеаризация и построение нормальных уравнений)
        for (int i = 1; i < N; ++i) // Iterate over all circles except the base one
        {
            const struct SCircle& current = all_circles[i]; 
            
            // Coefficients Ai and Bi
            double Ai = 2.0 * (current.x - base.x); 
            double Bi = 2.0 * (current.y - base.y); 

            // Coefficient Ci
            double current_term = sqr(current.x) + sqr(current.y) - sqr(current.r); 
            double Ci = current_term - base_term; 

            // Accumulate M^T*M and M^T*C
            M_xx += Ai * Ai; 
            M_xy += Ai * Bi;
            M_yy += Bi * Bi;
            V_x += Ai * Ci;
            V_y += Bi * Ci;
        } 

        // 2. Решение СЛАУ (2x2)
        double det = M_xx * M_yy - M_xy * M_xy; // Determinant
        if (fabs(det) < 1e-9) // Check for singularity
        {
            printf("Error: Determinant (%.2e) is near zero. Solution is unstable.\n", det); // Error: Determinant is near zero
            return A;
        } 

        double InvDet = 1.0 / det; // Inverse determinant
        A.x = InvDet * (M_yy * V_x - M_xy * V_y); // Solve for x
        A.y = InvDet * (M_xx * V_y - M_xy * V_x); // Solve for y
        
        printf("Found A: (%.4f, %.4f)\n", A.x, A.y); // Output found position

        // 3. Вычисление Геометрического RMS (в метрах)
        double sum_sq_residual_geom = 0.0; // Sum of squared geometric residuals
        for (int i = 0; i < N; ++i) // Iterate over all N circles
        {
            const struct SCircle& current = all_circles[i]; 
            
            // Distance from found point A to the circle center (r_calc)
            double r_calc = sqrt(sqr(A.x - current.x) + sqr(A.y - current.y)); 
            
            // Geometric residual (in meters): R_measured - R_calculated
            double residual_geom = current.r - r_calc; 
            sum_sq_residual_geom += sqr(residual_geom); 

            printf("Beacon %d (R=%.4f): Geom. Residual=%.4f m\n", i, current.r, residual_geom); // Output residual for each beacon
        }

        int degrees_of_freedom = N - 3; // Degrees of freedom = N - 3 (x, y, implicit scale factor)

        if (degrees_of_freedom > 0)
        {
            double rmse_geom = sqrt(sum_sq_residual_geom / degrees_of_freedom); // Geometric RMS
            
            printf("\n--- Quality Assessment (N=%d, DoF=%d) ---\n", N, degrees_of_freedom); // Output quality assessment
            printf("**Geometric RMS**: **%.4f m**\n", rmse_geom); // Output geometric RMS
        }
        else
        {
            printf("\nSolution is exactly determined (RMSE = N/A).\n"); // Solution is exactly determined
        }

        printf("--- POS LSQ SOLVER: END ---\n"); // Output end of LSQ solver

        return A; // Return found point
    } 

    // --- МЕТОД 4: ОПРЕДЕЛЕНИЕ ОРИЕНТАЦИИ ЛИДАРА (ВЕКТОРНЫЙ МНК) ---
    double get_lidar_orientation(
        const struct SPoint A_found,                          // Найденное положение
        const std::vector<struct SPoint>& beacons,           // Координаты маяков
        const std::vector<double>& lidar_angles_deg          // Углы, измеренные лидаром
    )
    {
        if (beacons.size() != lidar_angles_deg.size() || beacons.empty()) // Check data
        {
            printf("\n--- ORIENTATION CALCULATION SKIPPED ---\n"); // Skip orientation calculation
            printf("Error: Missing or mismatched data for orientation calculation.\n"); // Error: Missing or mismatched data
            return 0.0; 
        } 

        printf("\n--- ORIENTATION LSQ CALCULATION ---\n"); // Output start of orientation calculation
        
        double sum_sin = 0.0; // Sum of sines
        double sum_cos = 0.0; // Sum of cosines
        int N = beacons.size(); // Number of beacons
        
        for (int i = 0; i < N; ++i) // Iterate over all beacons
        {
            double alpha_AM_deg = get_azimuth_deg(A_found, beacons[i]); // Azimuth from A to beacon (Alpha)
            double theta_lidar_deg = lidar_angles_deg[i]; // Lidar angle (Theta)

            double psi_raw_deg = alpha_AM_deg - theta_lidar_deg; // Unnormalized rotation angle (Psi = Alpha - Theta)
            double psi_norm_deg = normalize_angle_deg(psi_raw_deg); // Normalized angle
            double psi_rad = DEG2RAD(psi_norm_deg); // Angle in radians

            sum_sin += sin(psi_rad); // Accumulate sines
            sum_cos += cos(psi_rad); // Accumulate cosines
            
            printf("Beacon %d (C%.2f): Alpha=%.2f, Theta=%.2f -> Psi_norm=%.2f deg\n", // Output intermediate values
                   i, beacons[i].x, alpha_AM_deg, theta_lidar_deg, psi_norm_deg); 
        } 

        double orientation_rad = atan2(sum_sin, sum_cos); // Calculate averaged angle (vector averaging)
        double orientation_deg = RAD2DEG(orientation_rad); // Angle in degrees
        
        printf("Sum Sin: %.4f, Sum Cos: %.4f\n", sum_sin, sum_cos); // Output sums
        printf("--- ORIENTATION LSQ CALCULATION: END ---\n"); // Output end of calculation

        return orientation_deg; // Return found orientation
    } 
};

// --- Пример использования (С согласованными данными: PSI_TRUE = 0.0) ---

int main()
{
    // 1. Исходные данные
    
    struct SPoint B = {4.0, 0.3}; // Маяк B
    struct SPoint C = {0.0, 0.5}; // Маяк C
    struct SPoint D = {0.5, 4.0}; // Маяк D
    struct SPoint E = {5.0, 4.0}; // Маяк E
    
    std::vector<struct SPoint> beacons = {B, C, D, E}; // Список маяков
    
    // ИСТИННЫЕ ЗНАЧЕНИЯ 
    double PSI_TRUE = 0.0; // Истинный угол ориентации
    struct SPoint A_true = {1.0, 2.0}; // Истинное положение A
    
    struct SPoint A_prev = {1.05, 2.05}; // Предыдущее положение (ЗАДАНО)

    // Истинные расстояния (R_true) и заданный шум 
    double R_AB_true = 3.448; double noise_R_AB = 0.01; 
    double R_AC_true = 1.802; double noise_R_AC = -0.02; 
    double R_AD_true = 2.061; double noise_R_AD = 0.01; 
    double R_AE_true = 4.472; double noise_R_AE = -0.01; 

    // Углы из трилатерации: МОДУЛИ УГЛОВ (без знака, т.к. фильтрация по A_prev восстанавливает знак)
    double angle_BAC = 94.15; // Модуль: 94.15
    double angle_CAD = 132.27; // Модуль: 132.27
    double angle_DAE = 77.47; // Модуль: 77.47
    double angle_EAB = 56.10; // Модуль: 56.10

    // СОГЛАСОВАННЫЕ УГЛЫ ЛИДАРА (Theta_i = Alpha_true - 0.0 + noise)
    std::vector<double> lidar_angles_deg = 
    {
        -29.54 - 0.3,    // Theta_AB = -29.84
        -123.69 + 0.2,   // Theta_AC = -123.49 (Используется Ваш корректный Az_C)
        104.04 + 0.1,    // Theta_AD = 104.14
        26.57 - 0.1      // Theta_AE = 26.47
    };

    // 2. Инициализация решателя
    TrilaterationSolver solver(A_prev); 

    // 3. Сбор окружностей (4 по расстоянию, 4 по углу)
    solver.add_circle_from_distance(B, R_AB_true + noise_R_AB); 
    solver.add_circle_from_distance(C, R_AC_true + noise_R_AC); 
    solver.add_circle_from_distance(D, R_AD_true + noise_R_AD); 
    solver.add_circle_from_distance(E, R_AE_true + noise_R_AE); 
    
    try
    {
        // Передаем МОДУЛИ углов. Функция сама выберет правильный центр, ближайший к A_prev.
        solver.add_filtered_circle_from_angle(B, C, angle_BAC); 
        solver.add_filtered_circle_from_angle(C, D, angle_CAD); 
        solver.add_filtered_circle_from_angle(D, E, angle_DAE); 
        solver.add_filtered_circle_from_angle(E, B, angle_EAB); 
    }
    catch(const std::invalid_argument& e)
    {
        fprintf(stderr, "Error collecting circles: %s\n", e.what()); 
        return 1;
    } 

    // 4. Расчет МНК для положения A
    printf("==================================\n"); 
    printf("LSQ FOR POSITION A (N=%d)\n", solver.get_circle_count()); 
    printf("==================================\n"); 

    struct SPoint A_found = solver.find_A_by_mnk(); 

    printf("\n--- POSITION CALCULATION SUMMARY ---\n"); 
    printf("True A: (%.4f, %.4f)\n", A_true.x, A_true.y); 
    printf("Found A: (%.4f, %.4f)\n", A_found.x, A_found.y); 
    
    // 5. Расчет ориентации лидара
    double orientation_psi = solver.get_lidar_orientation( 
        A_found, 
        beacons, 
        lidar_angles_deg
    ); 

    printf("\n--- ORIENTATION CALCULATION SUMMARY ---\n"); 
    printf("True Psi: %.4f deg\n", PSI_TRUE); 
    printf("Calculated Psi: %.4f deg\n", orientation_psi); 
    
    return 0; 
}