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
    void calcAnglePillarForLaser(CPillar::SPillar *pillar_, SPoseLidar &poseLidar_); // Расчет углов в локальной системе лазеров на столбы для передачи на нижний уровень для исполнения
    void calcPointPillarFromLaser(CPillar::SPillar *pillar_);                        // Расчет положения столбов в лидарной системе на основании данных с датчиков
    float anglePillarInLidar[4];                                                     // угол на этот столб из точки где находится лидар в Лидарной системе координат
    float anglePillarInLaser[4];                                                     // Углы в локальных системах Лазеров, которые передаем на нижний уровень к исполнению
    CLaser(/* args */);
    ~CLaser();
    SPose _poseLaser[4]; // Позиции систем координат лазеров в Центральной системе координат "base"

private:
    const float bias = 0.0636396; // Смещение от нулевой точки в солидворкс Установлена на одинаковом удалении от осей Х и У. Симметрично.
    float offset = 0;             // Смещение от точки откуда производим измерения лазером до оси вращения мотора (центра системы координат лазера)
    int matrixLaserPillar[4][5];  // Матрица Лазеров-Столбов. В ней строки - номера лазеров, столюцы - номера столбов которые лазер может обслуживать Столбец 5 для записи количества столбов
};

CLaser::CLaser(/* args */)
{
    _poseLaser[0].x = bias; // Смещение от нулевой точки в солидворкс
    _poseLaser[0].y = bias; // Смещение от нулевой точки в солидворкс
    _poseLaser[0].th = -45; // Направление оси Y относительно оси Y Центральнйо системы координат
    //---
    _poseLaser[1].x = bias;  // Смещение от нулевой точки в солидворкс
    _poseLaser[1].y = -bias; // Смещение от нулевой точки в солидворкс
    _poseLaser[1].th = 45;   // Направление оси Y относительно оси Y Центральнйо системы координат
    //---
    _poseLaser[2].x = -bias; // Смещение от нулевой точки в солидворкс
    _poseLaser[2].y = -bias; // Смещение от нулевой точки в солидворкс
    _poseLaser[2].th = 135;  // Направление оси Y относительно оси Y Центральнйо системы координат
    //---
    _poseLaser[3].x = -bias; // Смещение от нулевой точки в солидворкс
    _poseLaser[3].y = bias;  // Смещение от нулевой точки в солидворкс
    _poseLaser[3].th = -135; // Направление оси Y относительно оси Y Центральнйо системы координат
    //---
    offset = 30; // Смещение от точки откуда производим измерения лазером до оси вращения мотора (центра системы координат лазера)
}

CLaser::~CLaser()
{
}
// Расчет углов в локальной системе лазеров на столбы для передачи на нижний уровень для исполнения
void CLaser::calcAnglePillarForLaser(CPillar::SPillar *pillar_, SPoseLidar &poseLidar_) // Берем координаты(положение) столбов и позицию лидара в глобальной системе
{
    SPoint pointPillarInLidar[4];       // Координаты столбов в центральной системе координат "base"
    SPoint pointPillarInLaser[4];       // Координаты столбов в индивидуально лазерно системе координат "laser 0-3"
    SPose poseLidar = poseLidar_.mode1; // Выбираем результаты какого обсчета будем использовать
    // Обнуляем углы для моторов лазера
    for (int i = 0; i < 4; i++)
    {
        anglePillarInLaser[i] = 0; // Обнуляем угол что-бы сразу увидеть что лазер не используется. Он встанет в нулевое(нерабочее) положение если мы не присвоим правильный угол
    }
    // Перебираем столбы и для каждого находим его координаты(положение) в Центральной(Лидарной) системе координат "base"
    for (int i = 0; i < 4; i++)
    {
        SPoint pointGlobal;
        pointGlobal.x = pillar_[i].x_true; // Берем глобальные истинные координаты столба
        pointGlobal.y = pillar_[i].y_true;
        pointPillarInLidar[i] = pointGlobal2Local(pointGlobal, poseLidar); // получим координаты столба в Лидарной системе "base"
    }
    // Очищаем матрицу от прошлых значений записываем везде -1
    for (int i = 0; i < 4; i++) // Перебираем лазеры
    {
        for (int j = 0; j < 4; j++) // Перебираем столбы
        {
            matrixLaserPillar[i][j] = -1; // Очищаем матрицу от прошлых значений записываем везде -1
        }
        matrixLaserPillar[i][4] = 0; // Обнуляем счетчик столбов
    }
    //-----------------------------------------------------------------------
    // Для каждого лазера находим какие столбы он видит и может обслуживать. Для этого каждый столб перводим в локальную систему координат лазера и считаем угол на какой надо повернуться чтобы в него попасть
    for (int i = 0; i < 4; i++) // Перебираем лазеры
    {
        for (int j = 0; j < 4; j++) // Перебираем столбы
        {
            SPoint pointTemp = pointGlobal2Local(pointPillarInLidar[j], _poseLaser[i]); // получим координаты столба в лазерной системе "laser"
            float alfa = angleThetaFromPoint(pointTemp);                                // Получаем угол в лазерной системе
            if (alfa > 15 && alfa < 165)                                                // Сектор в котором мы обслуживаем лазеры
            {
                matrixLaserPillar[i][matrixLaserPillar[i][4]] = j; // Записываем номер столба который можем обслужить этим лазером
                matrixLaserPillar[i][4]++;                         // Считаем годные столбы
            }
        }
    }
    int tableLaser[4]{-1, -1, -1, -1}; // Таблица в которую собираем итоговые сопоставления лазеров и столбов
    //-----------------------------------------------------------------------
    int count = 0;              // Количество распределённых столбов
    for (int k = 0; k < 4; k++) // Делаем 4 поиска лазеров с 1 столбом. Это максимум возможных вариантов
    {
        for (int i = 0; i < 4; i++) // Перебираем лазеры
        {
            if (matrixLaserPillar[i][4] == 1) // Если есть только 1 столб, то его и назначаем
            {
                int numPillar = -1;
                for (int m = 0; m < 4; m++)
                {
                    numPillar = matrixLaserPillar[i][m]; // Запоминаем номер столба.
                    if (numPillar >= 0)                  // Ищем номер этого столба, в какой он ячейке? он будет не равен -1
                    {
                        break; // Нашли что надо и перестаем перебирать
                    }
                }

                tableLaser[i] = numPillar; // Номер в маассиве это номер лазера, а значение номер столба который лазеру назначен
                count++;                   // Есть распредленный столб
                // Нужно назначить и убрать из матрицы у других лазеров этот столб
                for (int n = 0; n < 4; n++) // Перебираем лазеры
                {
                    for (int j = 0; j < 4; j++) // Перебираем столбы
                    {
                        if (matrixLaserPillar[n][j] == numPillar) // Если там есть такой столб, то его убираем и уменьшаем счетчик
                        {
                            matrixLaserPillar[n][j] = -1;
                            matrixLaserPillar[n][4]--;
                        }
                    }
                }
            }
        }
    }
    //-----------------------------------------------------------------------
    ROS_INFO("!!!! Pillar warn... % i", count);
    // float zetta[4][2];
    // int countZetta = 0;
    // while (count < 4) // ЕСЛИ ВДРУГ ТАКАЯ СИТУАЦИЯ ЧТО ПО ПРЕДЫДУЩЕМУ АЛГОРИТМУ НЕ ВСЕ СТОЛБЫ РАСПРЕДЕЛИЛИ ТО ПРОБУЕМ ИНАЧЕ Пока все 4 столба не распределим.  // Делаем пока не распределим все лазеры по столбам
    // {
    //     ROS_WARN("!!!! Pillar warn... % i",count);
    //     for (int i = 0; i < 4; i++) // Перебираем лазеры, это строки матрицы
    //     {
    //         if (matrixLaserPillar[i][4] == 2) // Если остались лазеры у которых еще 2 столба в возможном обслуживании, то выбор делаем по минимальному углу от осевого
    //         {
    //             for (int j = 0; j < 4; j++)
    //             {
    //                 if (matrixLaserPillar[i][j] >= 0) // Находим номера столбов
    //                 {
    //                     zetta[countZetta][0] = abs(angleThetaFromPoint(pointPillarInLidar[j]) - 90); // Получаем угол в Лидарной системе минус 90 как от осевого и по модулю для 1 столба
    //                     zetta[countZetta][1] = matrixLaserPillar[i][j];                              // Запоминаем номер столба, с этим углом
    //                     countZetta++;
    //                 }
    //             }
    //             if (zetta[0][0] < zetta[1][0]) // Находим столб с наименьшим углом
    //             {
    //                 tableLaser[i] = zetta[0][1]; // Номер в маассиве это номер лазера, а значение номер столба который лазеру назначен
    //             }
    //             else
    //             {
    //                 tableLaser[i] = zetta[1][1]; // Номер в маассиве это номер лазера, а значение номер столба который лазеру назначен
    //             }
    //             count++;                     // Есть распредленный столб
    //         }
    //     }
    // }
    //-----------------------------------------------------------------------

    // Распределяем столбы по лазерам. Каждый обслуживает свой сектор
    for (int i = 0; i < 4; i++)
    {
        float alfa = angleThetaFromPoint(pointPillarInLidar[i]); // Получаем угол в Лидарной системе
        (alfa < 0) ? alfa += 360 : alfa = alfa;
        anglePillarInLidar[i] = alfa;
        ROS_INFO("anglePillarInLidar= %.3f ", alfa);
        ROS_INFO("---");

        SPoint point;
        // Распределяем какой лазер будет светить на этот столб исходя из ранее посчитанной таблицы сопоставлений столюов и лазеров
        if (tableLaser[i] >= 0)
        {
            point = pointGlobal2Local(pointPillarInLidar[i], _poseLaser[tableLaser[i]]); // Пересчитываем координаты из Лидарной системы в локальную конкретного лазера, ранее сопоставленного в таблицу tableLaser[i]
            anglePillarInLaser[i] = angleThetaFromPoint(point);
        }
        else
        {
            anglePillarInLaser[i] = 0;
        }
    }
    for (int i = 0; i < 4; i++)
    {
        g_angleLaser[i] = anglePillarInLaser[i];
        g_numPillar[i] = tableLaser[i];
        ROS_INFO("anglePillarInLaser= %.3f numPillarForLaser = %i ", anglePillarInLaser[i], tableLaser[i]);
    }
}

void CLaser::calcPointPillarFromLaser(CPillar::SPillar *pillar_) // Расчет положения столбов в лидарной системе на основании данных с датчиков
{
    // Сначала на идеальных данных сделать Меняем на фиксы идеальные
    msg_Modul2Data.laser[0].distance = 2.805 - OFFSET_LAZER;
    // msg_Modul2Data.laser[0].angle = 67.43;
    // msg_Modul2Data.laser[0].numPillar = 0;
    msg_Modul2Data.laser[1].distance = 1.279 - OFFSET_LAZER;
    // msg_Modul2Data.laser[1].angle = 42.55;
    // msg_Modul2Data.laser[1].numPillar = 1;
    msg_Modul2Data.laser[2].distance = 1.599 - OFFSET_LAZER;
    // msg_Modul2Data.laser[2].angle = 143.6;
    
    // msg_Modul2Data.laser[2].numPillar = 2;
    msg_Modul2Data.laser[3].distance = 2.980 - OFFSET_LAZER;
    // msg_Modul2Data.laser[3].angle = 105.43;
    // msg_Modul2Data.laser[3].numPillar = 3;

    SPoint p_laser;
    SPoint p_lidar;
    SPoint P00;
    float len;
    for (int i = 0; i < 4; i++)
    {
        msg_Modul2Data.laser[i].distance += OFFSET_LAZER;                                          // Добавляем смещение по креплению. Зависит от напечатанного крепления
        p_laser = pointFromTetha(msg_Modul2Data.laser[i].angle, msg_Modul2Data.laser[i].distance); // По расстоянию и углу куда был напрвлен лазер посчитать координаты в лазерной систме координат
        p_lidar = pointLocal2Global(p_laser, _poseLaser[msg_Modul2Data.laser[i].numPillar]);       // Превести координаты в лидарную систему учитывая каким лазером светили
        len = vectorLen(p_lidar, P00);                                                             // Надоди растояние до столба влидврной систме
        pillar_[msg_Modul2Data.laser[i].numPillar].distance_laser = len;                           // Записываем для дальнейшей обработки
    }

    // SPoint p1_laser = pointFromTetha(msg_Modul2Data.laser[1].angle, msg_Modul2Data.laser[1].distance);
    // SPoint p2_laser = pointFromTetha(msg_Modul2Data.laser[2].angle, msg_Modul2Data.laser[2].distance);
    // SPoint p3_laser = pointFromTetha(msg_Modul2Data.laser[3].angle, msg_Modul2Data.laser[3].distance);

    // ROS_INFO("P0 x_Lidar= %f y_Lidar = %f ", p0_lidar.x, p0_lidar.y);
    // SPoint p1_lidar = pointLocal2Global(p1_laser, _poseLaser[msg_Modul2Data.laser[1].numPillar]);
    // ROS_INFO("P1 x_Lidar= %f y_Lidar = %f ", p1_lidar.x, p1_lidar.y);
    // SPoint p2_lidar = pointLocal2Global(p2_laser, _poseLaser[msg_Modul2Data.laser[2].numPillar]);
    // ROS_INFO("P2 x_Lidar= %f y_Lidar = %f ", p2_lidar.x, p2_lidar.y);
    // SPoint p3_lidar = pointLocal2Global(p3_laser, _poseLaser[msg_Modul2Data.laser[3].numPillar]);
    // ROS_INFO("P3 x_Lidar= %f y_Lidar = %f ", p3_lidar.x, p3_lidar.y);

    // ROS_INFO("vectorLen0= %f ", len0);
    // float len1 = vectorLen(p1_lidar, P00);
    // ROS_INFO("vectorLen1= %f ", len1);
    // float len2 = vectorLen(p2_lidar, P00);
    // ROS_INFO("vectorLen2= %f ", len2);
    // float len3 = vectorLen(p3_lidar, P00);
    // ROS_INFO("vectorLen3= %f ", len3);

    // pillar_[msg_Modul2Data.laser[1].numPillar].distance_laser = len1;
    // pillar_[msg_Modul2Data.laser[2].numPillar].distance_laser = len2;
    // pillar_[msg_Modul2Data.laser[3].numPillar].distance_laser = len3;
    // pillar_[0].distance_laser = 2.805;
    // pillar_[1].distance_laser = 1.279;
    // pillar_[2].distance_laser = 1.599;
    // pillar_[3].distance_laser = 2.980;
}
#endif