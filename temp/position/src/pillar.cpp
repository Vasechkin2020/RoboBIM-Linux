/*
RoboBIM 28/02/2023 MVV
 */

#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"

#define RAD2DEG(x) ((x)*180. / M_PI)
#define DEG2RAD(x) ((x)*M_PI / 180.)

#define PEREHOD 0.2  // Разрыв между столбом и обьектом за ним

struct lidar_stru
{
    float angle = 0;  // Напрвление с лидара
    float ranges = 0; // дистанция с лидара
};

struct pillar_stru
{
    lidar_stru data[1024];    // данные с лидара по этому столбу
    float angle_left = 0;     // крайний левый угол
    float angle_right = 0;    // крайний правый угол
    float angle_dist_min = 0; // угол с лидара с минимальным растоянием
    float azimuth = 0;        // Итоговый азимут на столб
    float dist_min = 0;
    float dist_max = 0;
};

pillar_stru pillar[16]; // Массив котором храним данные по столбам

void scanCallback(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    static float previous = 0; // Предыдущее значение дистанции
    static int a = 0;          // Начало группы точек
    static int b = 0;          // Конец группы точек
    int c = 0;                 // Номер группы точек для массива
    static float width = 0;    // Ширина обьекта
    static float angle = 0;    // Угол на обьект
    static float range = 0;    // Жальность до обьекта
    bool flag = false;         // Флаг поиска точки
    int count = scan->scan_time / scan->time_increment;
    ROS_INFO("I heard a laser scan %s[%d]:", scan->header.frame_id.c_str(), count);
    // ROS_INFO("angle_range, %f, %f", RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max));

    for (int i = 0; i < count; i++) // Пробегаем по всему массиву и ищем группы точек
    {
        float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i); // Азиммут в градусах для данной точки
        float ranges = scan->ranges[i];                                      // Дистанция для данной точки
        if (degree > -120 && degree < -50)
        {
            ROS_INFO(": [%f, %f]", degree, scan->ranges[i]);
        }
        if (isfinite(ranges)) // Проверка что там нормальное число а не бесконечность
        {
            if (i != 0) // Для первого элемента просто его запоминаем в
            {
                if (previous - ranges > PEREHOD) // Если дальность текущей точки меньше предыдущей больше чем на 0,3 метра то
                {
                    a = i; // Запоминаем первую точку
                    flag = true;
                    ROS_INFO("Point a= %i degree= %f ranges= %f previous= %f", a, degree, ranges, previous);
                }
                if (ranges - previous > PEREHOD && flag) // Если дальность текущей точки больше предыдущей больше чем на 0,3 метра и ранее выла найдена точка a
                {
                    b = i;
                    ROS_INFO("Point b= %i degree= %f ranges= %f previous= %f", a, degree, ranges, previous);

                    float min_dist = 1000;
                    float min_angle = 0;
                    float max_dist = 0;
                    for (int i = a; i < b; i++) // Перебираем точки без последней, так как она уже за столбом по дальности
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
                    width = -RAD2DEG(scan->angle_min + scan->angle_increment * a) + RAD2DEG(scan->angle_min + scan->angle_increment * b); // Находим разницу между градусами с учтом последней точки для точного измерения
                    width = width / 2;                                                                                                    // Находим половину угла между крайними точками
                    width = sin(DEG2RAD(width)) * 2 * max_dist;                                                                           // Вычисляем ширину столба
                    // ROS_INFO("Point b= %i degree= %f ranges= %f previous= %f width = %f", b, degree, ranges, previous, width);            // Запоминаем вторую точку  и начинаем анализировать эту группу точек
                    if (width > 0.2 && width < 0.4) // Если ширина похожа на наш столб
                    {
                        ROS_INFO("Pillar width= %.2f min_dist= %.2f max_dist= %.2f Angle = %+8.3f angle = %+8.3f", width, min_dist, max_dist, RAD2DEG(scan->angle_min + scan->angle_increment * a), RAD2DEG(scan->angle_min + scan->angle_increment * (b - 1)));
                        ROS_INFO("Angle = %+8.3f ", (RAD2DEG(scan->angle_min + scan->angle_increment * a) + RAD2DEG(scan->angle_min + scan->angle_increment * (b - 1))) / 2);

                        int x = 0;
                        for (int i = a - 1; i <= b; i++) // Перебираем точки с захватом крайних для анализа если нужно будет
                        {
                            pillar[c].data[x].angle = (RAD2DEG(scan->angle_min + scan->angle_increment * i));
                            pillar[c].data[x].ranges = scan->ranges[i];
                            x++;
                        }
                        pillar[c].angle_left = RAD2DEG(scan->angle_min + scan->angle_increment * (b - 1));
                        pillar[c].angle_right = RAD2DEG(scan->angle_min + scan->angle_increment * a);
                        pillar[c].angle_dist_min = min_angle;
                        pillar[c].dist_min = min_dist;
                        pillar[c].dist_max = max_dist;

                        pillar[c].azimuth = (pillar[c].angle_left + pillar[c].angle_right) / 2; // Берем середину столба, хотя из-за вращения происходит смещения и лучше бы еще как-то уточнять

                        c++;
                    }
                    else
                    {
                        ROS_INFO("BED !!! Pillar width= %.2f min_dist= %.2f max_dist= %.2f Angle = %+8.3f angle = %+8.3f", width, min_dist, max_dist, RAD2DEG(scan->angle_min + scan->angle_increment * a), RAD2DEG(scan->angle_min + scan->angle_increment * (b - 1)));
                        ROS_INFO("Angle = %+8.3f ", (RAD2DEG(scan->angle_min + scan->angle_increment * a) + RAD2DEG(scan->angle_min + scan->angle_increment * (b - 1))) / 2);
                    }
                    // Сбрасываем переменные для следующего расчета
                    flag = false;
                }
            }

            previous = ranges; // запоминаем для следующего сравнения
        }
    }
    ROS_INFO("End");
}
int main(int argc, char **argv)
{
    ros::init(argc, argv, "position");
    ros::NodeHandle n;

    ros::Subscriber sub = n.subscribe<sensor_msgs::LaserScan>("/scan", 1000, scanCallback);

    ros::spin(); // блокируют выход основного потока до тех пор, пока ROS не вызовет завершение работы - Ctrl + C

    return 0;
}
