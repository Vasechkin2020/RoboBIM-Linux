#ifndef MYKALMAN_H
#define MYKALMAN_H

class MyKalman
{
private:
    //*****************************************************************************
    float currentestimate = 0.0; // Начальные значение
    float lastestimate = 0.0;
    float kalmangain = 0.0;

    // Эти переменные установить через функцию перед использванием. Определают как фильтрует фильтр
    float _err_measure = 0; // разброс измерения (ожидаемый шум измерения)
    float _q = 0;        // скорость изменения значений (0.001-1, варьировать самому)
    //*****************************************************************************
    //-----------------------------------------------------------------------------

    double X0; // predicted state
    double P0; // predicted covariance

    double F; // factor of real value to previous real value
    double Q; // measurement noise
    double H; // factor of measured value to real value
    double R; // environment noise

    double State = 0; // Начальные значение
    double K = 0;

    //-----------------------------------------------------------------------------

public:
    MyKalman(/* args */);
    ~MyKalman();
    //*****************************************************************************
    float filterVar1(float value);  // Вариант реализации первый взято от https://alexgyver.ru/lessons/filters/ или https://alexgyver.ru/gyverfilters/
    void setParametr1( double errmeasure_, double q_) // Эти переменные установить через функцию перед использванием. Определяют как фильтрует фильтр //Кажется что перемеенные перепутаны местами, но так работает ???!!!
    {
        _err_measure = errmeasure_;
        _q = q_;
    }
    //*****************************************************************************
    double filterVar2(double data); // Вараинт реализации 2 взято из https://habr.com/ru/post/140274/
    void setParametr2(double r, double q) // Эти переменные установить через функцию перед использванием. Определяют как фильтрует фильтр
    {
        Q = q; //Q — определение шума процесса является более сложной задачей. В любом случае, можно подобрать этот параметр для обеспечения требуемого уровня фильтрации.
        R = r; //R — ошибка измерения может быть определена испытанием измерительных приборов и определением погрешности их измерения.
        F = 1;
        H = 1;
    }
};

//*****************************************************************************
// функция фильтрации
float MyKalman::filterVar1(float newVal)
//float simpleKalman(float newVal) 
{
  float _kalman_gain, _current_estimate;
  static float _err_estimate = _err_measure;
  static float _last_estimate;
  _kalman_gain = (float)_err_estimate / (_err_estimate + _err_measure);
  _current_estimate = _last_estimate + (float)_kalman_gain * (newVal - _last_estimate);
  _err_estimate =  (1.0 - _kalman_gain) * _err_estimate + fabs(_last_estimate - _current_estimate) * _q;
  _last_estimate = _current_estimate;
  return _current_estimate;
}
//*****************************************************************************
double MyKalman::filterVar2(double data)
{
    static double Covariance = R;
    //time update - prediction
    X0 = F * State;
    P0 = F * Covariance * F + Q;

    //measurement update - correction
    K = H * P0 / (H * P0 * H + R);
    State = X0 + K * (data - H * X0);
    Covariance = (1 - K * H) * P0;
    return State;
}

MyKalman::MyKalman(/* args */) // Конструктор класса
{
}

MyKalman::~MyKalman() // Деструктор класса
{
}
#endif