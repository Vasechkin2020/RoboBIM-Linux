#ifndef PILLAR_H
#define PILLAR_H

#define RAD2DEG(x) ((x) * 180. / M_PI)
#define DEG2RAD(x) ((x) * M_PI / 180.)

#define PEREHOD 0.2 // Разрыв между столбом и обьектом за ним

// Класс для столба по которому определяем свою позицию
class Pillar
{
private:
    /* data */
    struct lidar_stru
    {
        float angle = 0;  // Напрвление с лидара
        float ranges = 0; // дистанция с лидара
    };

    // Структура куда записываем все столбы которые мы нашли на основе сканировавания по лидару
    struct pillar_stru
    {
        lidar_stru data[1024];    // данные с лидара по этому столбу
        float angle_left = 0;     // крайний левый угол
        float angle_right = 0;    // крайний правый угол
        float angle_dist_min = 0; // угол с лидара с минимальным растоянием
        float azimuth = 0;        // Итоговый азимут на столб
        float dist_min = 0;
        float dist_max = 0;
        float x = 0; // Расчетные координаты столба исходя из предыдущего положения платформы
        float y = 0;
    };

    // Структура входящих данных где указаны точные координаты установленных столбов
    struct pillarOut_stru
    {
        float x = 0;         // Координата по Х
        float y = 0;         // Координата по У
        float direction = 0; // Направление на столб по данным с лидара после сопоставления столбов
    };

public:

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan);
    void poiskPillar(int a_, int b_, const sensor_msgs::LaserScan::ConstPtr &scan);
    
    int c = 0;                 // Номер группы точек для массива

    pillar_stru pillar[16];      // Массив котором храним данные по столбам
    pillarOut_stru pillarOut[4]; // Массив котором храним данные по столбам

    Pillar(/* args */);
    ~Pillar();

    // lidar_stru data[1024]; // данные с лидара по этому столбу
    //  float angle_left = 0; // крайний левый угол
    //  float angle_right = 0; // крайний правый угол
    //  float angle_dist_min = 0; // угол с лидара с минимальным растоянием
    //  float azimuth = 0;  // Итоговый азимут на столб
    //  float dist_min = 0;
    //  float dist_max = 0;
    //  int diametr = 0;      // диаметр столба
    //  int x = 0;            // координата по х центра столба
    //  int y = 0;            // координата по у центра столба
    //  bool status = false;  // статус активная ли в данный момент
    //  int num = 0;          // номер расположения от 1 до 3, отсчет против часовой, как лидар вращается
};

Pillar::Pillar(/* args */)
{
}

Pillar::~Pillar()
{
}

void Pillar::poiskPillar(int a_, int b_, const sensor_msgs::LaserScan::ConstPtr &scan)
{
    static float width = 0;    // Ширина обьекта
    float min_dist = 1000;
    float min_angle = 0;
    float max_dist = 0;
    for (int i = a_; i < b_; i++) // Перебираем точки без последней, так как она уже за столбом по дальности
    {
        if (isfinite(scan->ranges[i])) // Проверка на нормальность значений а то попадаются бесконечность inf
        {
            if (scan->ranges[i] < min_dist) // Ищем минимальную дистанцию
            {
                min_dist = scan->ranges[i];
                min_angle = RAD2DEG(scan->angle_min + scan->angle_increment * i); // запоминаем дистанцию и угол при ней
            }
            if (scan->ranges[i] > max_dist)
                max_dist = scan->ranges[i]; // Ищем максимальную дистанцию
        }
    }
    width = -RAD2DEG(scan->angle_min + scan->angle_increment * a_) + RAD2DEG(scan->angle_min + scan->angle_increment * b_); // Находим разницу между градусами с учтом последней точки для точного измерения
    width = width / 2;                                                                                                    // Находим половину угла между крайними точками
    width = sin(DEG2RAD(width)) * 2 * max_dist;                                                                           // Вычисляем ширину столба
    // ROS_INFO("Point b= %i degree= %f ranges= %f previous= %f width = %f", b, degree, ranges, previous, width);         // Запоминаем вторую точку  и начинаем анализировать эту группу точек
    ROS_INFO("width = %f", width);         // Ширина столба
    if (width > 0.2 && width < 0.4) // Если ширина похожа на наш столб
    {
        ROS_INFO("Pillar width= %.2f min_dist= %.2f max_dist= %.2f Angle = %.3f angle = %.3f", width, min_dist, max_dist, RAD2DEG(scan->angle_min + scan->angle_increment * a_), RAD2DEG(scan->angle_min + scan->angle_increment * (b_ - 1)));
        ROS_INFO("Angle = %.3f ", (RAD2DEG(scan->angle_min + scan->angle_increment * a_) + RAD2DEG(scan->angle_min + scan->angle_increment * (b_ - 1))) / 2);

        int x = 0;
        for (int i = a_ - 1; i <= b_; i++) // Перебираем точки с захватом крайних для анализа если нужно будет
        {
            pillar[c].data[x].angle = (RAD2DEG(scan->angle_min + scan->angle_increment * i));
            pillar[c].data[x].ranges = scan->ranges[i];
            x++;
        }
        pillar[c].angle_left = RAD2DEG(scan->angle_min + scan->angle_increment * (b_ - 1));
        pillar[c].angle_right = RAD2DEG(scan->angle_min + scan->angle_increment * a_);
        pillar[c].angle_dist_min = min_angle;
        pillar[c].dist_min = min_dist;
        pillar[c].dist_max = max_dist;

        pillar[c].azimuth = (pillar[c].angle_left + pillar[c].angle_right) / 2; // Берем середину столба, хотя из-за вращения происходит смещения и лучше бы еще как-то уточнять

        c++;
    }
    else
    {
        ROS_INFO("BED !!! Pillar width= %.2f min_dist= %.2f max_dist= %.2f Angle = %.3f angle = %.3f", width, min_dist, max_dist, RAD2DEG(scan->angle_min + scan->angle_increment * a_), RAD2DEG(scan->angle_min + scan->angle_increment * (b_ - 1)));
        ROS_INFO("Angle = %.3f ", (RAD2DEG(scan->angle_min + scan->angle_increment * a_) + RAD2DEG(scan->angle_min + scan->angle_increment * (b_ - 1))) / 2);
    }
    c = 0; // Обнуляем для следующего сканирования
}

//Функция которую вызываем из колбека по расчету места столбов
void Pillar::scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    float previous = 0; // Предыдущее значение дистанции
    int a = 0;          // Начало группы точек
    int b = 0;          // Конец группы точек
    bool flag = false;         // Флаг поиска точки
    int count = scan->scan_time / scan->time_increment;

    ROS_INFO("I heard a laser scan %s[%d]:", scan->header.frame_id.c_str(), count);
    // ROS_INFO("angle_range, %f, %f", RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max));

    for (int i = 0; i < count; i++) // Пробегаем по всему массиву и ищем группы точек
    {
        float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i); // Азиммут в градусах для данной точки
        float ranges = scan->ranges[i];                                      // Дистанция для данной точки
        if (isfinite(ranges)) // Проверка что там нормальное число а не бесконечность
        {
            if (i != 0) // Для первого элемента просто его запоминаем в
            {
                if (ranges - previous > PEREHOD && flag) // Если дальность текущей точки больше предыдущей больше чем на 0,3 метра и ранее выла найдена точка a
                {
                    b = i;
                    ROS_INFO("Point b= %i degree= %f ranges= %f previous= %f", a, degree, ranges, previous);
                    poiskPillar(a, b, scan);
                    flag = false; // Сбрасываем флаг для следующего расчета
                }
                if (previous - ranges > PEREHOD) // Если дальность текущей точки меньше предыдущей больше чем на 0,3 метра то
                {
                    a = i; // Запоминаем первую точку
                    flag = true;
                    ROS_INFO("Point a= %i degree= %f ranges= %f previous= %f", a, degree, ranges, previous);
                }
            }

            previous = ranges; // запоминаем для следующего сравнения
        }
        // if (degree > -120 && degree < -50) //Печать для отладки диапазон
        // {
        //     ROS_INFO(": [%f, %f]", degree, scan->ranges[i]);
        // }
    }
    ROS_INFO("End");
}

#endif
