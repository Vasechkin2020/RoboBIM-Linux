#ifndef KALMAN_H
#define KALMAN_H

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