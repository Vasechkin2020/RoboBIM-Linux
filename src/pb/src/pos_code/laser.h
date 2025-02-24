#ifndef LASER_H
#define LASER_H

#include "config.h"
#include "pillar.h"
#include <cmath> // Библиотека для математических функций

/* Класс в котором считаем и храним все что связанос управление лазерами на верхнем уровне
    У нас приняты 3 системы координат:
    - Глобальная
    - Центральная (это центр основного блока в SolidWorks и относительно его считаем все расположение на роботе(машинке)). Совпадает с центром Лидара, можно еще называть Лидарной.
    - Локальные у отдельных устройств. Лазеров, печатающей головки и прочего...
 */
class CLaser
{

public:
    struct SMatrixPillar
    {
        int n = -1;      // Номер столюа на который может светить датчик
        float angle = 0; // Угол при котром он туда светит
    };
    void calcAnglePillarForLaser(CPillar::SPillar *pillar_, SPose &poseLidar_); // Расчет углов в локальной системе лазеров на столбы для передачи на нижний уровень для исполнения
    void calcPointPillarFromLaser(CPillar::SPillar *pillar_);                   // Расчет положения столбов в лидарной системе на основании данных с датчиков
    void deleteNum(SMatrixPillar (&matrixLaserPillar_)[4][5], int num_);
    void pillar_1(SMatrixPillar (&matrixLaserPillar_)[4][5], SMatrixPillar (&tableLaser_)[4], int &count_); // Сопоставление столба если может обслужить только 1 лазер
    float anglePillarInLidar[4];                                                                            // угол на этот столб из точки где находится лидар в Лидарной системе координат
    float anglePillarInLaser[4];                                                                            // Углы в локальных системах Лазеров, которые передаем на нижний уровень к исполнению
    CLaser(/* args */);
    ~CLaser();
    SPose _poseLaser[4]; // Позиции систем координат лазеров в Центральной системе координат "lidar"

private:
    const float bias = 0.0636396; // Смещение от нулевой точки в солидворкс Установлена на одинаковом удалении от осей Х и У. Симметрично.
    float offset = 0;             // Смещение от точки откуда производим измерения лазером до оси вращения мотора (центра системы координат лазера)

    SMatrixPillar matrixLaserPillar[4][5]; // Матрица Лазеров-Столбов. В ней строки - номера лазеров, столюцы - номера столбов которые лазер может обслуживать Столбец 5 для записи количества столбов
};

CLaser::CLaser(/* args */)
{
    _poseLaser[0].x = bias;  // Смещение от нулевой точки в солидворкс
    _poseLaser[0].y = -bias; // Смещение от нулевой точки в солидворкс
    _poseLaser[0].th = 45;   // Направление оси X относительно оси X Центральнйо системы координат Вращение против часовй, по стандарту РОС
    //---
    _poseLaser[1].x = -bias; // Смещение от нулевой точки в солидворкс
    _poseLaser[1].y = -bias; // Смещение от нулевой точки в солидворкс
    _poseLaser[1].th = -45;  // Направление оси X относительно оси X Центральнйо системы координат Вращение против часовй, по стандарту РОС
    //---
    _poseLaser[2].x = -bias; // Смещение от нулевой точки в солидворкс
    _poseLaser[2].y = bias;  // Смещение от нулевой точки в солидворкс
    _poseLaser[2].th = -135; // Направление оси X относительно оси X Центральнйо системы координат Вращение против часовй, по стандарту РОС
    //---
    _poseLaser[3].x = bias; // Смещение от нулевой точки в солидворкс
    _poseLaser[3].y = bias; // Смещение от нулевой точки в солидворкс
    _poseLaser[3].th = 135; // Направление оси X относительно оси X Центральнйо системы координат Вращение против часовй, по стандарту РОС
    //---
    offset = 30; // Смещение от точки откуда производим измерения лазером до оси вращения мотора (центра системы координат лазера)
}

CLaser::~CLaser()
{
}
// void funArray(int (&arr)[4][5]) Образец как надо обьявлять
// // Метод убирает из матрицы у других лазеров этот столб
void CLaser::deleteNum(SMatrixPillar (&matrixLaserPillar_)[4][5], int num_)
{

    for (int n = 0; n < 4; n++) // Перебираем лазеры
    {
        for (int j = 0; j < 4; j++) // Перебираем столбы
        {
            if (matrixLaserPillar_[n][j].n == num_) // Если там есть такой столб, то его убираем и уменьшаем счетчик
            {
                matrixLaserPillar_[n][j].n = -1;
                matrixLaserPillar_[n][4].n--;
            }
        }
    }
}

void CLaser::pillar_1(SMatrixPillar (&matrixLaserPillar_)[4][5], SMatrixPillar (&tableLaser_)[4], int &count_) // Сопоставление столба если может обслужить только 1 лазер
{
    // printf("Start pillar_1... \n");
    for (int k = 0; k < 4; k++) // Делаем 4 поиска лазеров с 1 столбом. Это максимум возможных вариантов
    {
        for (int i = 0; i < 4; i++) // Перебираем лазеры
        {
            // printf("Laser i= %i ", i);
            // printf("matrixLaserPillar[i][4]= %i \n", matrixLaserPillar_[i][4].n); // Матрица Лазеров-Столбов. В ней строки - номера лазеров, столюцы - номера столбов которые лазер может обслуживать Столбец 5 для записи количества столбов
            if (matrixLaserPillar_[i][4].n == 1) // Если есть только 1 столб, то его и назначаем
            {
                int numPillar = -1;
                float angle_pillar = 0;
                for (int m = 0; m < 4; m++)
                {
                    numPillar = matrixLaserPillar_[i][m].n;        // Запоминаем номер столба.
                    angle_pillar = matrixLaserPillar_[i][m].angle; // Запоминаем угол на столб.
                    if (numPillar >= 0)                            // Ищем номер этого столба, в какой он ячейке? он будет не равен -1
                    {
                        break; // Нашли что надо и перестаем перебирать
                    }
                }

                tableLaser_[i].n = numPillar;        // Номер в маассиве это номер лазера, а значение номер столба который лазеру назначен
                tableLaser_[i].angle = angle_pillar; // Номер в маассиве это номер лазера, а значение номер столба который лазеру назначен
                count_++;                            // Есть распредленный столб
                matrixLaserPillar_[i][4].n = 0;      // Обнуляем счетчик столбов
                // printf("numPillar00= %i angle_pillar % .3f \n", tableLaser_[i].n, tableLaser_[i].angle);
                //  Нужно назначить и убрать из матрицы у других лазеров этот столб
                //  for (int n = 0; n < 4; n++) // Перебираем лазеры
                //  {
                //      for (int j = 0; j < 4; j++) // Перебираем столбы
                //      {
                //          if (matrixLaserPillar[n][j].n == tableLaser[i].n) // Если там есть такой столб, то его убираем и уменьшаем счетчик
                //          {
                //              matrixLaserPillar[n][j].n = -1;
                //              matrixLaserPillar[n][4].n--;
                //          }
                //      }
                //  }
                deleteNum(matrixLaserPillar_, tableLaser_[i].n); // Убираем из двухмерного массива уже сопоставленный столб из других вариантов
            }
        }
    }
    // printf("End pillar_1... \n");
}

// Расчет углов в локальной системе лазеров на столбы для передачи на нижний уровень для исполнения
void CLaser::calcAnglePillarForLaser(CPillar::SPillar *pillar_, SPose &poseLidar_) // Берем координаты(положение) столбов и позицию лидара в глобальной системе
{
    ROS_INFO("+++ calcAnglePillarForLaser");
    SPoint pointPillarInLidar[4]; // Координаты столбов в центральной системе координат "lidar"
    SPoint pointPillarInLaser[4]; // Координаты столбов в индивидуально лазерно системе координат "laser 0-3"
    SPose poseLidar = poseLidar_; // Выбираем результаты какого обсчета будем использовать
    // Обнуляем углы для моторов лазера
    for (int i = 0; i < 4; i++)
    {
        anglePillarInLaser[i] = 0; // Обнуляем угол что-бы сразу увидеть что лазер не используется. Он встанет в нулевое(нерабочее) положение если мы не присвоим правильный угол
    }
    // Перебираем столбы и для каждого находим его координаты(положение) в системе координат "Base"
    for (int i = 0; i < 4; i++)
    {
        SPoint pointGlobal;
        pointGlobal.x = pillar_[i].x_true; // Берем глобальные истинные координаты столба
        pointGlobal.y = pillar_[i].y_true;
        pointPillarInLidar[i] = pointGlobal2LocalRos(pointGlobal, poseLidar); // получим координаты столба в системе "Base"
        // ROS_INFO("Koordinat %i in /Base/ x= % .3f y= % .3f \n", i, pointPillarInLidar[i].x, pointPillarInLidar[i].y);
    }
    // printf("----\n");

    float lenPillar[4] = {0}; // Длинна вектора от центра координат до столба
    float lenMotor[4] = {0};  // Длинна вектора от центра координат до моторов
    float angPillar[4] = {0}; // Угол от оси Х которая прямо на столб. Вправо по часовой минус, Влево против частовой плюс
    float angMotor[4] = {0};  // Угол от оси Х которая прямо на столб. Вправо по часовой минус, Влево против частовой плюс

    ROS_INFO("  ");
    for (int i = 0; i < 4; i++)
    {
        lenPillar[i] = sqrt(pow(pointPillarInLidar[i].x, 2) + pow(pointPillarInLidar[i].y, 2)); // Теорема Пифагора // Находим длинну до столба от центра системы координат
        angPillar[i] = RAD2DEG(atan2(pointPillarInLidar[i].y, pointPillarInLidar[i].x)); // Находим его и правим его смотря в какой чатверти круга он находится
        ROS_INFO("    angPillar[%i] = %.3f x = %.3f y = %.3f len = %.3f", i, angPillar[i], pointPillarInLidar[i].x, pointPillarInLidar[i].y, lenPillar[i]);

        lenMotor[i] = sqrt(pow(_poseLaser[i].x, 2) + pow(_poseLaser[i].y, 2)); // Теорема Пифагора // Находим длинну до мотора от центра системы координат
        angMotor[i] = RAD2DEG(atan2(_poseLaser[i].y, _poseLaser[i].x)); // Находим его и правим его смотря в какой чатверти круга он находится
        ROS_INFO("    angMotor[%i]  = %.3f x = %.3f y = %.3f len = %.3f \n", i, angMotor[i], _poseLaser[i].x, _poseLaser[i].y, lenMotor[i]);
    }
    //************** Функция расчета углов на моторы. Используется как проверка основного расчета. По формуле Максима Вадима.
    for (int i = 0; i < 4; i++) // Перебираем моторы
    {
        for (int j = 0; j < 4; j++) // Перебираем столбы
        {
            float alfa = angPillar[j] - angMotor[i];
            // ROS_INFO("    alfa = %.3f", alfa);
            float ang = DEG2RAD(alfa);

            float ll = lenPillar[j];
            float ee1 = ll - 0.09 * cos(ang);
            float ee2 = 0.09 * sin(ang);

            // ROS_INFO("    ll = %.3f ee1 = %.3f ee2 = %.3f ang = %.3f", ll, ee1, ee2, ang);
            float cc = atan2(ee2, ee1);
            float mm = RAD2DEG(cc);
            // float cc = RAD2DEG(atan((lenPillar[i] - 0.09 * cos(ang)) / (0.09 * sin(ang))));
            // ROS_INFO("    [%i] ang = %.3f cc = %.3f rad   %.3f gradus", i, ang, cc, mm);
            // ROS_INFO("    [%i] alfa = %.3f    cc = %.3f rad       %.3f gradus", i, alfa, cc, mm);
            float itog;
            float itog2;
            itog = alfa + mm;
            if (itog < 0) // Если там насчитали по часовой
                itog2 = abs(itog) + 90;
            else if (itog > 90)
                itog2 = (360 - itog) + 90;
            else
                itog2 = 90 - itog;

            ROS_INFO("    ITOG[%i] = %8.3f                         ITOG2 = %8.3f ", i, itog, itog2);
            // ROS_INFO("    Var1= %.3f Var2= %.3f",90-itog, 90+ itog);
        }
        ROS_INFO(" ");
    }
    // Очищаем матрицу от прошлых значений записываем везде -1
    for (int i = 0; i < 4; i++) // Перебираем лазеры
    {
        for (int j = 0; j < 4; j++) // Перебираем столбы
        {
            matrixLaserPillar[i][j].n = -1; // Очищаем матрицу от прошлых значений записываем везде -1
        }
        matrixLaserPillar[i][4].n = 0; // Обнуляем счетчик столбов
    }
    //-----------------------------------------------------------------------
    // Для каждого лазера находим какие столбы он видит и может обслуживать. Для этого каждый столб перводим в локальную систему координат лазера и считаем угол на какой надо повернуться чтобы в него попасть
    for (int i = 0; i < 4; i++) // Перебираем лазеры
    {
        // printf("Laser %i \n", i);
        for (int j = 0; j < 4; j++) // Перебираем столбы
        {
            SPoint pointTemp = pointGlobal2LocalRos(pointPillarInLidar[j], _poseLaser[i]); // получим координаты столба в лазерной системе /laser/
            // printf("x= % .4f y= %.4f ", pointTemp.x, pointTemp.y);
            float alfa = -angleThetaFromPointRos(pointTemp); // Получаем угол в лазерной системе ставим МИНУС так как у меня положительные углы по часовой а в РОС против часовой и ответ приходит как в РОС
            if (alfa > 10 && alfa < 170)                     // Сектор в котором мы обслуживаем лазеры
            {
                matrixLaserPillar[i][matrixLaserPillar[i][4].n].n = j;        // Записываем номер столба который можем обслужить этим лазером
                matrixLaserPillar[i][matrixLaserPillar[i][4].n].angle = alfa; // Записываем угол в /Laser/ системе при котором попадаем на столб
                matrixLaserPillar[i][4].n++;                                  // Считаем годные столбы
                // printf("angle Alfa in /Laser/ = % .3f ", alfa);
                // printf("mojet obsujit %i \n", j);
            }
        }
        // printf("=\n");
    }
    // printf("---\n");
    SMatrixPillar tableLaser[4]; // {-1, -1, -1, -1}; // Таблица в которую собираем итоговые сопоставления лазеров и столбов
    for (int i = 0; i < 4; i++)
    {
        tableLaser[i].n = -1;
        tableLaser[i].angle = 0;
    }

    //-----------------------------------------------------------------------
    int count = 0; // Количество распределённых столбов

    pillar_1(matrixLaserPillar, tableLaser, count); // Сопоставление столбов если их может однозначно обслуживать только один лазер
    //-----------------------------------------------------------------------
    // ROS_INFO("!!!! Pillar warn... % i", count);

    for (int i = 0; i < 4; i++)
    {
        // ROS_INFO("numPillar11= %i angle_pillar % .3f \n", tableLaser[i].n, tableLaser[i].angle);
    }

    for (int k = 0; k < 4; k++) // Делаем 4 поиска лазеров с 2 столбом. Это максимум возможных вариантов
    {
        // ROS_WARN("!!!! Pillar warn... % i", count);
        for (int i = 0; i < 4; i++) // Перебираем лазеры, это строки матрицы
        {
            // ROS_INFO(" %i matrixLaserPillar2[i][4]= %i \n", i, matrixLaserPillar[i][4].n); // Матрица Лазеров-Столбов. В ней строки - номера лазеров, столюцы - номера столбов которые лазер может обслуживать Столбец 5 для записи количества столбов
            if (matrixLaserPillar[i][4].n == 2) // Если остались лазеры у которых еще 2 столба в возможном обслуживании, то выбор делаем по минимальному углу от осевого
            {
                float minAngle = 180; // Начальный самый большой
                float angleNum = 0;
                int minNum = -1;
                for (int j = 0; j < 4; j++)
                {
                    if (matrixLaserPillar[i][j].n >= 0) // Находим номера столбов
                    {
                        float a = abs(matrixLaserPillar[i][j].angle - 90); // Считаем угол от осевого, а он 90 градусов
                        // ROS_INFO("a= % .3f minAngle = % .3f === ", a, minAngle);
                        if (a < minAngle)
                        {
                            // printf("i= %i j= %i a= % .3f ", i, j, a);
                            minAngle = a;
                            angleNum = matrixLaserPillar[i][j].angle;
                            minNum = matrixLaserPillar[i][j].n; // Запоминаем номер столба
                            // ROS_INFO(" minNum %i minAngle % .3f \n", minNum, minAngle);
                        }
                    }
                }
                // ROS_INFO(" ITOG minNum %i minAngle % .3f \n", minNum, minAngle);
                tableLaser[i].n = minNum;
                tableLaser[i].angle = angleNum;
                count++;                       // Есть распредленный столб
                matrixLaserPillar[i][4].n = 0; // Обнуляем счетчик столбов которые он может обслужить у данного лазера, так как он распределен
                // ROS_INFO("numPillar33= %i angle_pillar % .3f \n", tableLaser[i].n, tableLaser[i].angle);

                // Нужно назначить и убрать из матрицы у других лазеров этот столб
                deleteNum(matrixLaserPillar, tableLaser[i].n); // Убираем из двухмерного массива уже сопоставленный столб из других вариантов
                break;                                         // Прерываем перебор, так как двойки могли стать в 1 и нужен перебор по единице
            }
        }
        pillar_1(matrixLaserPillar, tableLaser, count); // Сопоставление столбов если их может однозначно обслуживать только один лазер
    }
    ROS_INFO("    Raspredelenie Pillar - %i", count);

    // printf("!!!! Pillar warn REZULTAT... % i", count);
    // printf(" === \n");
    // for (int i = 0; i < 4; i++)
    // {
    //     printf("REZULTAT numPillar77= %i angle_pillar % .3f \n", tableLaser[i].n, tableLaser[i].angle);
    // }
    // printf(" === \n");

    // Распределяем столбы по лазерам. Каждый обслуживает свой сектор
    // ROS_INFO("---");
    // for (int i = 0; i < 4; i++)
    // {
    //     float alfa = angleThetaFromPoint(pointPillarInLidar[i]); // Получаем угол в Лидарной системе
    //     (alfa < 0) ? alfa += 360 : alfa = alfa;
    //     anglePillarInLidar[i] = alfa;
    //     ROS_INFO("anglePillarInLidar= %.3f ", alfa);

    //     SPoint point;
    //     // Распределяем какой лазер будет светить на этот столб исходя из ранее посчитанной таблицы сопоставлений столбов и лазеров
    //     if (tableLaser[i].n >= 0)
    //     {
    //         point = pointGlobal2Local(pointPillarInLidar[i], _poseLaser[tableLaser[i].n]); // Пересчитываем координаты из Лидарной системы в локальную конкретного лазера, ранее сопоставленного в таблицу tableLaser[i]
    //         anglePillarInLaser[i] = angleThetaFromPoint(point);
    //     }
    //     else
    //     {
    //         anglePillarInLaser[i] = 0;
    //     }
    // }
    for (int i = 0; i < 4; i++)
    {
        anglePillarInLaser[i] = tableLaser[i].angle; // Для визуализации в RVIZ
        g_angleLaser[i] = tableLaser[i].angle;       // Для топика ?
        g_numPillar[i] = tableLaser[i].n;
        ROS_INFO("    numPillar = %i -> angle= %8.3f", g_numPillar[i], g_angleLaser[i]);
    }
    // printf("\n");
    // ROS_INFO("--- calcAnglePillarForLaser"); 
}

void CLaser::calcPointPillarFromLaser(CPillar::SPillar *pillar_) // Расчет положения столбов в лидарной системе на основании данных с датчиков
{
    // Сначала на идеальных данных сделать Меняем на фиксы идеальные
    //  msg_Modul2Data.laser[0].distance = 2.76343;
    //  msg_Modul2Data.laser[0].angle = 67.5522;
    //  msg_Modul2Data.laser[0].numPillar = 0;

    // msg_Modul2Data.laser[1].distance = 1.23827;
    // msg_Modul2Data.laser[1].angle = 42.463;
    // msg_Modul2Data.laser[1].numPillar = 1;

    // msg_Modul2Data.laser[2].distance = 1.35932;
    // msg_Modul2Data.laser[2].angle = 144.737;
    // msg_Modul2Data.laser[2].numPillar = 2;

    // msg_Modul2Data.laser[3].distance = 2.93866;
    // msg_Modul2Data.laser[3].angle = 105.687;
    // msg_Modul2Data.laser[3].numPillar = 3;

    SPoint p_laser;
    SPoint p_lidar;
    SPoint P00;
    float len;
    ROS_INFO("+++ calcPointPillarFromLaser");
    for (int i = 0; i < 4; i++)
    {
        // ROS_INFO("distance SRC = %f", msg_Modul2Data.laser[i].distance);
        // ROS_INFO("numPillar = %i", msg_Modul2Data.laser[i].numPillar);
        msg_Modul2Data.laser[i].distance += (OFFSET_LAZER); // Добавляем смещение по креплению. Зависит от напечатанного крепления
        // printf("distance + OFFSET = %f \n", msg_Modul2Data.laser[i].distance);
        msg_Modul2Data.laser[i].distance += (PILLAR_RADIUS); // Добавляем смещение по креплению. Зависит от напечатанного крепления
        // printf("distance + RADIUS = %f \n", msg_Modul2Data.laser[i].distance);
        // printf(" numPillar = %i ", msg_Modul2Data.laser[i].numPillar);
        p_laser = pointFromTetha(msg_Modul2Data.laser[i].angle, msg_Modul2Data.laser[i].distance); // По расстоянию и углу куда был напрвлен лазер посчитать координаты в лазерной систме координат
        // printf("Local x= %f y= %f ", p_laser.x, p_laser.y);
        // printf("Local Angle Theta = %f \n", _poseLaser[msg_Modul2Data.laser[i].numPillar].th);
        p_lidar = pointLocal2GlobalRos(p_laser, _poseLaser[msg_Modul2Data.laser[i].numPillar]); // Превести координаты в /Base/ систему учитывая каким лазером светили
        // ROS_INFO("Global x= %f y= %f ",p_lidar.x,p_lidar.y);
        len = vectorLen(p_lidar, P00); // Находим растояние до столба в лидарной систме
        // ROS_INFO(" Global len = %f", len);
        pillar_[msg_Modul2Data.laser[i].numPillar].distance_laser = len; // Записываем для дальнейшей обработки
    }
    // ROS_INFO("/ distance_laser |");
    for (int i = 0; i < 4; i++)
    {
        ROS_INFO("    numPillar = %i -> distance_laser = %f", msg_Modul2Data.laser[i].numPillar, pillar_[i].distance_laser);
    }
    // printf("\n");

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
    ROS_INFO("--- calcPointPillarFromLaser");
}
#endif