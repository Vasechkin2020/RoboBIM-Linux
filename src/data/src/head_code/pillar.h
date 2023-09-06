#ifndef PILLAR_H
#define PILLAR_H

#define RAD2DEG(x) ((x) * 180. / M_PI)
#define DEG2RAD(x) ((x) * M_PI / 180.)

#define PEREHOD 0.15 // Разрыв между столбом и обьектом за ним

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
        float width = 0; // Ширина столба
        float x = 0;     // Расчетные координаты столба исходя из предыдущего положения платформы
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

    int c = 0;          // Номер группы точек для массива
    int max_pillar = 0; // Колличество столбов отобраееых по алгоритму в массив

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
    static float width = 0; // Ширина обьекта
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

    float angle_a = RAD2DEG(scan->angle_min + scan->angle_increment * a_);
    float angle_b = RAD2DEG(scan->angle_min + scan->angle_increment * b_);
    float angle_middle = (angle_a + angle_b) / 2;

    float angle_ab = angle_b - angle_a; // Находим разницу между градусами
    // width = -RAD2DEG(scan->angle_min + scan->angle_increment * a_) + RAD2DEG(scan->angle_min + scan->angle_increment * b_); // Находим разницу между градусами с учтом последней точки для точного измерения
    float angle_sin = angle_ab / 2;                   // Находим половину угла между крайними точками
    width = (sin(DEG2RAD(angle_sin)) * max_dist) * 2; // Вычисляем ширину столба

    // ROS_INFO("Point b= %i width = %f", b, width);         // Запоминаем вторую точку  и начинаем анализировать эту группу точек
    // ROS_INFO("angle_ab= %f max_dist= %f width = %f", angle_ab, max_dist, width); // Ширина столба

    if (width > 0.2 && width < 0.4) // Если ширина похожа на наш столб
    {

        // ROS_INFO("=== Pillar width= %.2f min_dist= %.2f max_dist= %.2f Angle a = %.3f Angle b= %.3f Angle_middle= %.3f", width, min_dist, max_dist, RAD2DEG(scan->angle_min + scan->angle_increment * a_), RAD2DEG(scan->angle_min + scan->angle_increment * (b_ - 1)));
        // ROS_INFO("=== Pillar width= %.2f min_dist= %.2f max_dist= %.2f Angle a = %.3f Angle b= %.3f Angle_middle= %.3f", width, min_dist, max_dist, angle_a, angle_b, angle_middle);
        // ROS_INFO("Angle middle= %.3f ", (RAD2DEG(scan->angle_min + scan->angle_increment * a_) + RAD2DEG(scan->angle_min + scan->angle_increment * (b_ - 1))) / 2);
        // ROS_INFO("=====");

        int x = 0;
        for (int i = a_ - 1; i <= b_; i++) // Перебираем точки с захватом крайних для анализа если нужно будет
        {
            pillar[c].data[x].angle = (RAD2DEG(scan->angle_min + scan->angle_increment * i));
            pillar[c].data[x].ranges = scan->ranges[i];
            x++;
        }
        pillar[c].angle_left = angle_b;
        pillar[c].angle_right = angle_a;
        pillar[c].angle_dist_min = min_angle;
        pillar[c].dist_min = min_dist;
        pillar[c].dist_max = max_dist;
        pillar[c].width = width;
        pillar[c].azimuth = angle_middle; // Берем середину столба, хотя из-за вращения происходит смещения и лучше бы еще как-то уточнять
        c++;
    }
    else
    {
        // ROS_INFO("BED width --->>> Pillar width= %.2f min_dist= %.2f max_dist= %.2f Angle = %.3f angle = %.3f", width, min_dist, max_dist, RAD2DEG(scan->angle_min + scan->angle_increment * a_), RAD2DEG(scan->angle_min + scan->angle_increment * (b_ - 1)));
        // ROS_INFO("Angle = %.3f ", (RAD2DEG(scan->angle_min + scan->angle_increment * a_) + RAD2DEG(scan->angle_min + scan->angle_increment * (b_ - 1))) / 2);
    }
}

// Функция которую вызываем из колбека по расчету места столбов
void Pillar::scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    float previous = 0; // Предыдущее значение дистанции
    int a = 0;          // Начало группы точек
    int b = 0;          // Конец группы точек
    bool flag = false;  // Флаг поиска точки
    int count = scan->scan_time / scan->time_increment;

    ROS_INFO("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! ");
    ROS_INFO("I heard a laser scan %s[%d]:", scan->header.frame_id.c_str(), count);
    // ROS_INFO("angle_range, %f, %f", RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max));

    for (int i = 0; i < count; i++) // Пробегаем по всему массиву и ищем группы точек
    {
        float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i); // Азиммут в градусах для данной точки
        float ranges = scan->ranges[i];                                      // Дистанция для данной точки
        if (isfinite(ranges))                                                // Проверка что там нормальное число а не бесконечность
        {
            if (i != 0) // Для первого элемента просто его запоминаем в
            {
                if (ranges - previous > PEREHOD && flag) // Если дальность текущей точки больше предыдущей больше чем на 0,3 метра и ранее выла найдена точка a
                {
                    b = i;
                    ROS_INFO("Point b= %i degree= %f ranges= %f previous ranges= %f , Perepad= %f", i, degree, ranges, previous, ranges - previous);
                    ROS_INFO("=====");
                    poiskPillar(a, b, scan);
                    flag = false; // Сбрасываем флаг для следующего расчета
                }
                if (previous - ranges > PEREHOD) // Если дальность текущей точки меньше предыдущей больше чем на 0,3 метра то
                {
                    a = i; // Запоминаем первую точку
                    flag = true;
                    ROS_INFO("Point a= %i degree= %f ranges= %f previous ranges= %f , Perepad= %f", i, degree, ranges, previous, ranges - previous);
                }
                // if (true)
                // {
                //     if (abs(ranges - previous) > 0.1)
                //     {
                //         ROS_INFO("Point search = %i degree= %f ranges= %f previous ranges= %f , Perepad= %f", i, degree, ranges, previous, ranges - previous);
                //     }

                // }
            }

            previous = ranges; // запоминаем для следующего сравнения
        }
        // if (degree > -120 && degree < -50) //Печать для отладки диапазон
        // {
        //     ROS_INFO(": [%f, %f]", degree, scan->ranges[i]);
        // }
    }

    ROS_INFO("All pillar= %i", c);
    max_pillar = c; // Сохраняем число найденных столбов
    c = 0;          // Обнуляем для следующего сканирования

    ROS_INFO("End");

    for (int i = 0; i < max_pillar; i++)
    {
        ROS_INFO("i= %i width= %f angle_left= %f angle_right= %f azimuth= %f angle_dist_min= %f dist_min- %f dist_max= %f", i, pillar[i].width, pillar[i].angle_left, pillar[i].angle_right, pillar[i].azimuth, pillar[i].angle_dist_min, pillar[i].dist_min, pillar[i].dist_max);
    }
}

#endif
