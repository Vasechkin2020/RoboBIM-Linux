#ifndef PILLAR_H
#define PILLAR_H

// Класс для столба по которому определяем свою позицию
class Pillar
{
private:
    /* data */
    struct lidar_stru
    {
        float angle = 0;    // Напрвление с лидара
        float ranges = 0; // дистанция с лидара
    };

public:
    Pillar(/* args */);
    ~Pillar();
    lidar_stru data[1024]; // данные с лидара по этому столбу
    float angle_left = 0; // крайний левый угол
    float angle_right = 0; // крайний правый угол
    float angle_dist_min = 0; // угол с лидара с минимальным растоянием 
    float azimuth = 0;  // Итоговый азимут на столб
    float dist_min = 0;
    float dist_max = 0;
    int diametr = 0;      // диаметр столба
    int x = 0;            // координата по х центра столба
    int y = 0;            // координата по у центра столба
    bool status = false;  // статус активная ли в данный момент
    int num = 0;          // номер расположения от 1 до 3, отсчет против часовой, как лидар вращается
};

Pillar::Pillar(/* args */)
{
}

Pillar::~Pillar()
{
}

#endif

