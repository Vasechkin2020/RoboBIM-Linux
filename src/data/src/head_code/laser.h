#ifndef LASER_H
#define LASER_H

#include "config.h"
#include "pillar.h"

/* Класс в котором считаем и храним все что связанос управление лазерами на верхнем уровне
    У нас приняты 3 системы координат:
    - Глобальная
    - Центральная (это центр основного блока в SolidWorks и относительно его считаем все расположение на роботе(машинке)). Совпадает с центром Лидара, моно еще называть Лидарной.
    - Локальные у отдельных устройств. Лазеров, печатающей головки и прочего...
 */
class CLaser
{

public:
    void calcAnglePillarForLaser(CPillar::SPillar *pillar_, SPoseLidar &poseLidar); // Расчет углов в локальной системе лазеров на столбы для передачи на нижний уровень для исполнения
    float anglePillarInLidar[4];                                               // угол на этот столб из точки где находится лидар в Лидарной системе координат
    float anglePillarInLaser[4];                                               // Углы в локальных системах Лазеров, которые передаем на нижний уровень к исполнению
    CLaser(/* args */);
    ~CLaser();

private:
    const float bias = 0.0636396; // Смещение от нулевой точки в солидворкс Установлена на одинаковом удалении от осей Х и У. Симметрично.
    SPose poseLaser[4];           // Позиции систем координат лазеров в Центральной системе координат
    float offset = 0;             // Смещение от точки откуда производим измерения лазером до оси вращения мотора (центра системы координат лазера)
};

CLaser::CLaser(/* args */)
{
    poseLaser[0].x = bias;    // Смещение от нулевой точки в солидворкс
    poseLaser[0].y = bias;    // Смещение от нулевой точки в солидворкс
    poseLaser[0].theta = -45; // Направление оси Y относительно оси Y Центральнйо системы координат
    //---
    poseLaser[1].x = bias;   // Смещение от нулевой точки в солидворкс
    poseLaser[1].y = -bias;  // Смещение от нулевой точки в солидворкс
    poseLaser[1].theta = 45; // Направление оси Y относительно оси Y Центральнйо системы координат
    //---
    poseLaser[2].x = -bias;   // Смещение от нулевой точки в солидворкс
    poseLaser[2].y = -bias;   // Смещение от нулевой точки в солидворкс
    poseLaser[2].theta = 135; // Направление оси Y относительно оси Y Центральнйо системы координат
    //---
    poseLaser[3].x = -bias;    // Смещение от нулевой точки в солидворкс
    poseLaser[3].y = bias;     // Смещение от нулевой точки в солидворкс
    poseLaser[3].theta = -135; // Направление оси Y относительно оси Y Центральнйо системы координат
    //---
    offset = 30; // Смещение от точки откуда производим измерения лазером до оси вращения мотора (центра системы координат лазера)
}

CLaser::~CLaser()
{
}
// Расчет углов в локальной системе лазеров на столбы для передачи на нижний уровень для исполнения
void CLaser::calcAnglePillarForLaser(CPillar::SPillar *pillar_, SPoseLidar &poseLidar_) // Берем координаты(положение) столбов и позицию лидара в глобальной системе
{
    SPoint pointPillarInLidar[4];
    SPoint pointGlobal;
    SPose poseLidar = poseLidar_.mode2; // Выбираем результаты какого обсчета будем использовать
    for (int i = 0; i < 4; i++)         // Перебираем столбы и для каждого находим его координаты(положение) в Центральной(Лидарной) системе координат
    {
        pointGlobal.x = pillar_[i].x_true; // Берем глобальные истинные координаты столба
        pointGlobal.y = pillar_[i].y_true;
        pointPillarInLidar[i] = pointGlobal2Local(pointGlobal, poseLidar); // получим координаты столба в Лидарной системе
    }
    // Обнуляем углы для моторов лазера
    for (int i = 0; i < 4; i++)
    {
        anglePillarInLaser[i] = 0; // Обнуляем угол что-бы сразу увидеть что лазер не используется. Он встанет в нулевое(нерабочее) положение если мы не присвоим правильный угол
    }

    // Распределяем столбы по лазерам. Каждый обслуживает свой сектор
    for (int i = 0; i < 4; i++)
    {
        float alfa = angleThetaFromPoint(pointPillarInLidar[i]); // Получаем угол в Лидарной системе
        (alfa < 0) ? alfa += 360 : alfa = alfa;
        anglePillarInLidar[i]=alfa;

        SPoint point;
        // Распределяем какой лазер будет светить на этот столб исходя из своего сектора обслуживания
        if (alfa > 0 && alfa <= 90) // Лазер 0
        {
            point = pointGlobal2Local(pointPillarInLidar[i], poseLaser[0]); // Пересчитываем координаты из Лидарной системы в локальную конкретного лазера
            anglePillarInLaser[0] = angleThetaFromPoint(point);
        }
        if (alfa > 90 && alfa <= 180) // Лазер 1
        {
            point = pointGlobal2Local(pointPillarInLidar[i], poseLaser[1]); // Пересчитываем координаты из Лидарной системы в локальную конкретного лазера
            anglePillarInLaser[1] = angleThetaFromPoint(point);
        }
        if (alfa > 180 && alfa <= 270) // Лазер 2
        {
            point = pointGlobal2Local(pointPillarInLidar[i], poseLaser[2]); // Пересчитываем координаты из Лидарной системы в локальную конкретного лазера
            anglePillarInLaser[2] = angleThetaFromPoint(point);
        }
        if (alfa > 270 && alfa <= 360) // Лазер 3
        {
            point = pointGlobal2Local(pointPillarInLidar[i], poseLaser[3]); // Пересчитываем координаты из Лидарной системы в локальную конкретного лазера
            anglePillarInLaser[3] = angleThetaFromPoint(point);
        }
        ROS_INFO("anglePillarInLidar= %.3f ", alfa);
    }
    for (int i = 0; i < 4; i++)
    {
        ROS_INFO("anglePillarInLaser= %.3f ", anglePillarInLaser[i]);
    }
}

#endif