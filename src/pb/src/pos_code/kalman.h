#ifndef KALMAN_H
#define KALMAN_H



#include <stdio.h>   // Для printf
#include <cmath>     // Для sin, cos, atan2, sqrt

// --- 1. Вспомогательный класс: KalmanAngle ---
/*
 * KalmanAngle.h - Простой 1D фильтр Калмана
 *
 * Этот класс реализует 1D фильтр Калмана, который используется  * для слияния данных гироскопа и акселерометра для получения * одного угла (например, крена или тангажа).
 *
 * Он оценивает состояние из двух переменных:
 * 1. Угол (angle)
 * 2. Смещение гироскопа (bias)
 */
class KalmanAngle
{
public:
    KalmanAngle()
    {
        // Настроечные параметры фильтра (ковариации шумов)
        Q_angle = 0.1;   // Шум процесса (для угла) Q_angle: Начните с 0.001, увеличивайте если фильтр слишком "доверяет" гироскопу
        Q_bias = 0.1;    // Шум процесса (для смещения гироскопа) Q_bias: Начните с 0.003, увеличивайте если смещение быстро меняется
        R_measure = 0.001;  // Шум измерения (для акселерометра) _measure: Начните с 0.03, увеличивайте если данные с акселерометра зашумлены

        angle = 0.0;       // Начальный угол (в радианах)
        bias = 0.0;        // Начальное смещение гироскопа

        // Инициализация ненулевой неопределенности для ускорения сходимости
        P[0][0] = 1.0;     
        P[0][1] = 0.0;
        P[1][0] = 0.0;
        P[1][1] = 1.0;     
    }

    // Метод обновления фильтра
    // newRate - новое значение с гироскопа (град/с или рад/с)
    // newAngle - новое значение с акселерометра (град или рад)
    // dt - время, прошедшее с последнего обновления (в секундах)
    double update(double newRate, double newAngle, double dt)
    {
        if (dt < 0.0001)   // Проверка dt на ноль
        {
            return angle;
        }
        // --- ЭТАП ПРЕДСКАЗАНИЯ (PREDICT) ---
        // Основан на данных гироскопа (модель процесса)

        // 1. Предсказываем новый угол:
        // Угол = старый угол + (скорость - смещение) * dt
        angle += (newRate - bias) * dt; // Рассчитываем новый угол
        
        // 2. Предсказываем новую ковариационную матрицу P
        // P_k = F * P_{k-1} * F^T + Q
        // (Здесь реализована упрощенная, но математически корректная форма)
        P[0][0] += dt * (dt * P[1][1] - P[0][1] - P[1][0]) + Q_angle; // Обновление P[0][0]
        
        P[0][1] -= dt * P[1][1];        
        P[1][0] = P[0][1];              
        
        P[1][1] += Q_bias * dt; // Обновляем P[1][1], учитывая дрейф смещения

        // --- ЭТАП КОРРЕКЦИИ (UPDATE) ---
        // Основан на данных акселерометра (измерение)

        // 1. Инновация (ошибка)
        // y = z - H * x (наше измерение - предсказанное значение)
        double y = newAngle - angle;    // Разница между измеренным (acc) и предсказанным (gyro) углом
        double S = P[0][0] + R_measure; // Ошибка измерения (ковариация инновации) // S = H * P * H^T + R

        // 3. Коэффициент Калмана (Kalman Gain)        // K = P * H^T * S^{-1}
        double K[2];                    // Вектор коэффициента Калмана (K[0] для угла, K[1] для смещения)
        K[0] = P[0][0] / S;             // Расчет K[0]
        K[1] = P[1][0] / S;             // Расчет K[1]

        // 4. Коррекция состояния (наш новый, уточненный угол)// x = x + K * y
        angle += K[0] * y;              // Корректируем угол
        bias += K[1] * y;               // Корректируем смещение гироскопа

        // 5. Коррекция ковариационной матрицы P // P = (I - K * H) * P
        double P00_temp = P[0][0];      
        double P01_temp = P[0][1];

        P[0][0] -= K[0] * P00_temp;     // Обновление матрицы ковариации
        P[0][1] -= K[0] * P01_temp;
        
        P[1][0] = P[0][1];              
        
        P[1][1] -= K[1] * P01_temp;

        return angle;                   // Возвращаем отфильтрованный угол
    }

    void setAngle(double newAngle) { angle = newAngle; } // Установить угол

private:
    double Q_angle, Q_bias, R_measure;
    double angle, bias;
    double P[2][2];
};

// --- 2. Основной класс: IMUManager ---
/*  * IMUManager.h - УПРОЩЕННЫЙ класс для обработки IMU  */
class IMUManager
{
public:
    IMUManager() // Конструктор
    {
        roll = 0.0;         
        pitch = 0.0;        
    }

        // Главный метод обновления
    void update(double gx, double gy, double ax, double ay, double az, double dt)
    {
        // --- 1. Нормализация акселерометра ---
        double magnitude = sqrt(ax * ax + ay * ay + az * az);
        if (magnitude < 0.0001) // Защита только от деления на ноль
            return;

        double n_ax = ax / magnitude;
        double n_ay = ay / magnitude;
        double n_az = az / magnitude;

        // --- 2. Расчет углов по акселерометру ---
        double roll_acc = atan2(n_ax, n_az);
        double pitch_acc = atan2(-n_ay, sqrt(n_ax * n_ax + n_az * n_az)); // Тут меняем ось если датчик стоит по другому

        // --- 3. Обновление фильтров Калмана ---
        pitch = kalmanPitch.update(gx, pitch_acc, dt);
        roll = kalmanRoll.update(gy, roll_acc, dt);   
    }

    // --- Методы доступа (Getters) ---
    double getRoll() { return roll; } // Крен относительно нуля
    double getPitch() { return pitch; } // Тангаж относительно нуля

private:
    KalmanAngle kalmanRoll;
    KalmanAngle kalmanPitch;
    double roll, pitch;
};

/*
// --- 3. Пример использования (main.cpp) ---
int main()
{
    IMUManager imu;

    // --- 1. СИМУЛЯЦИЯ КАЛИБРОВКИ ---
    double initial_pitch_rad = 1.2 * (3.1415926535 / 180.0);
    
    double ax_calib = -sin(initial_pitch_rad);
    double ay_calib = 0.005;
    double az_calib = cos(initial_pitch_rad);
    
    imu.calibrateOffsets(ax_calib, ay_calib, az_calib); 

    // --- 2. СИМУЛЯЦИЯ ДВИЖЕНИЯ ---
    double dt = 0.01;
    double rad_to_deg = 180.0 / 3.1415926535;

    for (int i = 0; i < 200; i++)
    {
        double pitch_target_rad = 6.2 * (3.1415926535 / 180.0); 

        double ax_raw = -sin(pitch_target_rad); 
        double ay_raw = ay_calib;
        double az_raw = cos(pitch_target_rad);

        double gx_raw = 0.0; 
        double gy_raw = 0.0;

        imu.update(gx_raw, gy_raw, ax_raw, ay_raw, az_raw, dt); 

        // --- 4. Вывод результатов ---
        if (i % 20 == 0) 
        {
            double rel_pitch = imu.getRelativePitch() * rad_to_deg;
            double acc_x = imu.getLinAccX();

            printf("Т:%.2fс | Rel Pitch=%.2f deg | Lin Ax=%.4f G\n", 
                   (double)i * dt, rel_pitch, acc_x);
        }
    }

    return 0;
}

*/

class CKalman
{
// private:
//     float currentestimate = 0.0; // Начальные значение
//     float lastestimate = 0.0;
//     float kalmangain = 0.0;

//     // Эти переменные установить через функцию перед использванием. Определают как фильтрует фильтр
//     float _err_measure = 0; // разброс измерения (ожидаемый шум измерения)
//     float _q = 0;           // скорость изменения значений (0.001-1, варьировать самому)

// public:
//     //*****************************************************************************
//     // float filterVar1(float value);                   // Вариант реализации первый взято от https://alexgyver.ru/lessons/filters/ или https://alexgyver.ru/gyverfilters/
//     void setParametr1(double errmeasure_, double q_) // Эти переменные установить через функцию перед использванием. Определяют как фильтрует фильтр //Кажется что перемеенные перепутаны местами, но так работает ???!!!
//     {
//         _err_measure = errmeasure_;
//         _q = q_;
//     }
//     float filterVar1(float newVal)
//     // float simpleKalman(float newVal)
//     {
//         float _kalman_gain, _current_estimate;
//         static float _err_estimate = _err_measure;
//         static float _last_estimate;
//         _kalman_gain = (float)_err_estimate / (_err_estimate + _err_measure);
//         _current_estimate = _last_estimate + (float)_kalman_gain * (newVal - _last_estimate);
//         _err_estimate = (1.0 - _kalman_gain) * _err_estimate + abs(_last_estimate - _current_estimate) * _q;
//         _last_estimate = _current_estimate;
//         return _current_estimate;
//     }

    //*************************************************************************************************************************************************************
private:
    // double X0; // predicted state
    // double P0; // predicted covariance

    // double F; // factor of real value to previous real value
    // double Q; // measurement noise
    // double H; // factor of measured value to real value
    // double R; // environment noise

    // double State = 0; // Начальные значение
    // double K = 0;
    //*******************
    double X0x; // predicted state
    double P0x; // predicted covariance

    double Fx; // factor of real value to previous real value
    double Qx; // measurement noise
    double Hx; // factor of measured value to real value
    double Rx; // environment noise

    double Statex = 0; // Начальные значение
    double Kx = 0;
    //*******************
    double X0y; // predicted state
    double P0y; // predicted covariance

    double Fy; // factor of real value to previous real value
    double Qy; // measurement noise
    double Hy; // factor of measured value to real value
    double Ry; // environment noise

    double Statey = 0; // Начальные значение
    double Ky = 0;

public:
    // double filterVar2(double data);       // Вараинт реализации 2 взято из https://habr.com/ru/post/140274/
    // void setParametr(double r, double q) // Эти переменные установить через функцию перед использванием. Определяют как фильтрует фильтр
    // {
    //     Q = q; // Q — определение шума процесса является более сложной задачей. В любом случае, можно подобрать этот параметр для обеспечения требуемого уровня фильтрации.
    //     R = r; // R — ошибка измерения может быть определена испытанием измерительных приборов и определением погрешности их измерения.
    //     F = 1;
    //     H = 1;
    // }
    // double filterVar2(double data)
    // {
    //     static double Covariance = R;
    //     // time update - prediction
    //     X0 = F * State;
    //     P0 = F * Covariance * F + Q;

    //     // measurement update - correction
    //     K = H * P0 / (H * P0 * H + R);
    //     State = X0 + K * (data - H * X0);
    //     Covariance = (1 - K * H) * P0;
    //     return State;
    // }

    // Эти переменные установить через функцию перед использванием. Определяют как фильтрует фильтр
    void setParametrX(double r, double q)
    {
        Qx = q; // Q — определение шума процесса является более сложной задачей. В любом случае, можно подобрать этот параметр для обеспечения требуемого уровня фильтрации.
        Rx = r; // R — ошибка измерения может быть определена испытанием измерительных приборов и определением погрешности их измерения.
        Fx = 1;
        Hx = 1;
    }
    // Эти переменные установить через функцию перед использванием. Определяют как фильтрует фильтр
    void setParametrY(double r, double q)
    {
        Qy = q; // Q — определение шума процесса является более сложной задачей. В любом случае, можно подобрать этот параметр для обеспечения требуемого уровня фильтрации.
        Ry = r; // R — ошибка измерения может быть определена испытанием измерительных приборов и определением погрешности их измерения.
        Fy = 1;
        Hy = 1;
    }
    // Расчет по координате Х, на вход подаем измерение и значение по Х посчитанное по модели движения. Это прошлая позиция плюсс укорение на время
    double calcX(double izmerenie_, double model_)
    {
        static double Covariance = Rx;
        // time update - prediction
        Fx = model_ / Statex; // Делим значение которое получили  ранее по модели движения на предыдущее значение и получаем матрицу трансформации(перехода состояния) (динамическая модель системы)
        X0x = Fx * Statex;
        P0x = Fx * Covariance * Fx + Qx;

        // measurement update - correction
        Kx = Hx * P0x / (Hx * P0x * Hx + Rx);
        Statex = X0x + Kx * (izmerenie_ - Hx * X0x);
        Covariance = (1 - Kx * Hx) * P0x;
        return Statex;
    }
    // Расчет по координате Y, на вход подаем измерение и значение по Y посчитанное по модели движения. Это прошлая позиция плюсс укорение на время
    double calcY(double izmerenie_, double model_)
    {
        static double Covariance = Ry;
        // time update - prediction
        Fy = model_ / Statey; // Делим значение которое получили  ранее по модели движения на предыдущее значение и получаем матрицу трансформации(перехода состояния) (динамическая модель системы)
        X0y = Fy * Statey;
        P0y = Fy * Covariance * Fy + Qy;

        // measurement update - correction
        Ky = Hy * P0y / (Hy * P0y * Hy + Ry);
        Statey = X0y + Ky * (izmerenie_ - Hy * X0y);
        Covariance = (1 - Ky * Hy) * P0y;
        return Statey;
    }
};



































#endif