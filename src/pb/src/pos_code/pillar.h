#ifndef PILLAR_H
#define PILLAR_H

#include "config.h"

#define PILLAR_RADIUS 0.1575    // Радиус столба 315 мм
#define PEREHOD 0.15            // Разрыв между столбом и обьектом за ним
#define OTKLONENIE_PILLAR 0.315 // Отклонение между истинным и лидарным столбом при котором они сопоставляются
#define DELTA_MINIMUMA 0.03     // Величина отклонения на которую увеличиваем минимальную дистация до столба при отборе угла на середину, что-бы не первую попавшийся минимум брать +-3 см как в даташите пока взял для тестов

/* Класс для всего что связано с внешними установленными столбами, по которым определяем свою позицию

*/
class CPillar
{

public:
    struct SPillar // Итоговая структура по столбам. Ее потом опубликуем
    {
        bool status = false; // Статус сопоставлен столб или нет
        float azimuth = 0;   // Лидарные значения появляются при сопоставлении
        float hypotenuse = 0;
        float distance_lidar = 0; // Расстояние до столба измеренное лидаром
        float distance_laser = 0; // Расстояние до столба измеренное лазером
        float x_true = 0;         // Истинные значения
        float y_true = 0;
        float theta_true1 = 0; // Угол какой должен быть из текущих координат от У в глобальной системе. Куда смотрит нос лидара
        float theta_true2 = 0; // Угол какой должен быть из текущих координат
        float theta_true3 = 0; // Угол какой должен быть из текущих координат
        float x_lidar = 0;
        float y_lidar = 0;
    };
    SPillar pillar[4];   // Массив столбов с итоговым результатом где все столбы сопоставленны
    int countPillar = 0; // Колличество столбов с истинными координатами не знаем точно сколько загрузим

    // ************************** ПУБЛИЧНЫЕ ФУНКЦИИ КЛАССА *********************************************
    CPillar(/* args */);
    ~CPillar();
    void parsingLidar(const sensor_msgs::LaserScan::ConstPtr &scan, SPose &poseLidarMode_); // Метод которую вызываем из колбека по расчету места столбов
    void parsingPillar(pb_msgs::topicPillar &pillar_);                                      // Метод которую вызываем из колбека по расчету места столбов
    void comparisonPillar();                                                                // Метод где сопоставляю столбы
    void getLocationMode1(SPose &poseReturn_, SPose pose_);                                 // Метод возврщает положение центра лидара усредненного по всем столбам и расчетам по меоду расчета по "растояние до столбов по лидару"
    void getLocationMode2(SPose &poseReturn_, SPose pose_);                                 // Метод возврщает положение центра лидара усредненного по всем столбам и расчетам по методу расчета по "углам межлу столбами по лидару"
    void getLocationMode3(SPose &poseReturn_, SPose pose_);                                 // Метод возврщает положение центра лидара усредненного по всем столбам и расчетам по методу расчета по "данные по расстоянию по лазерам"
private:
    struct SLidar
    {
        float degree = 0;   // Направление с лидара Градус
        float distance = 0; // Дистанция с лидара Диcтанция
    };

    // Структура куда записываем все столбы которые мы нашли на основе сканировавания по лидару
    struct SPillarLidar
    {
        SLidar data[256];           // данные с лидара по этому столбу, всего 256 точек максимум
        float angle_left = 0;       // крайний левый угол
        float angle_right = 0;      // крайний правый угол
        float angle_dist_min = 0;   // угол с лидара с минимальным растоянием. Его считаем итоговым напрвлением на столб
        float angle_middle_min = 0; // Расчетный средний угол по точкам с дистанцией около минимальной на дельту отличается
        float angle_middle = 0;     // Угол на примерную середину столба. Считается как середина точек участвующих в формировании стобла
        float azimuth = 0;          // Итоговый угол на столб. Пока считаем как угол на минимум. Или можно усреднять например со средним углом. Нало посмотреть что будет точнее
        float dist_min = 0;
        float width = 0;      // Ширина столба
        float x_lidarXY = 0;  // Расчетные координаты столба в системе координат лидара
        float y_lidarXY = 0;  // Расчетные координаты столба в системе координат лидара
        float x_globalXY = 0; // Расчетные координаты столба в глобальной системе координат
        float y_globalXY = 0; // Расчетные координаты столба в глобальной системе координат
    };

    // ************************** ПРИВАТНЫЕ ПЕРЕМЕННЫЕ КЛАССА *********************************************
    int countPillarLidar = 0; // Колличество столбов отбираемых по алгоритму в массив с Лидара Не знаем точно сколько обнаружим
    SPoint poseLidarMode1[8]; // Позиции лидара по расчетам окружностей вокруг столбов по дпннвм лидара
    int count_poseLidarMode1 = 0;
    SPoint poseLidarMode2[32]; // Позиции лидара по расчетам огружностей по 2 столбам и углу между ними более 30 и менее 120
    int count_poseLidarMode2 = 0;
    SPoint poseLidarMode3[8]; // Позиции лидара по расчетам окружностей вокруг столбов по дпнным лазеров
    int count_poseLidarMode3 = 0;
    SCircle circleAnglePillar[32];   // Окружности которые получаем на основании углов между столбами для расчета по 2 методу
    int count_circleAnglePillar = 0; // Подсчет числа окружностей что получили

    SPillarLidar pillarLidar[16]; // Массив котором храним данные по столбам

    // ************************** ПРИВАТНЫЕ ФУНКЦИИ КЛАССА *********************************************
    float getTheta(SPose poseLidar_, int mode_); // Метод возвращает угол между ось. Y вверх и напрвлением лидара вперед
    void poiskPillar(int a_, int b_, SLidar *lidarData, SPose &poseLidarMode_);
    SCircle2 getCircle(SPoint p1_, SPoint p2_, float angle);                             // Метод возвращает окружность(координаты и радиус) которая ближе всего к точке старого измерения
    int getCrossing(SCircle c1_, SCircle c2_, SPose pose_, SPoint &pointX_, float len_); // Метод возвращает одну из двух точек пеерсечения двух окружностей ближайшую к заданной
    // SPose calculationPose(SPoint point_, SPillar pillar_);                               // Метод считает угол на который повернут ноль лидара в глобальной системе координат
};

CPillar::CPillar(/* args */)
{
}

CPillar::~CPillar()
{
}
// Метод возврщает положение центра лидара усредненного по всем столбам по методу "растояние до столбов по лидару"
void CPillar::getLocationMode3(SPose &poseReturn_, SPose pose_) // На вход подается последняя полученная/посчитанная позиция лидара
{
    ROS_INFO("+++ getLocationMode3 ");
    ROS_INFO("pose IN x= %.3f y= %.3f th= %.3f", pose_.x, pose_.y, pose_.th);
    ROS_INFO("distance_laser %.3f %.3f %.3f %.3f", pillar[0].distance_laser, pillar[1].distance_laser, pillar[2].distance_laser, pillar[3].distance_laser);

    float a1, a2;
    SCircle c1, c2;
    SPoint point;
    SPose poseLidar;          // Позиции лидара по расчетам
    count_poseLidarMode3 = 0; // Обнуляем счетчик

    for (int i = 0; i < countPillar - 1; i++) //        Перебираем столбы, и для каждого пересечение окружностей со следующим
    {
        for (int j = i + 1; j < countPillar; j++)
        {
            // if ((pillar[i].distance_laser != 0) || (pillar[j].distance_laser != 0)) // Если не нули и есть какие-то показания
            // {
            c1.x = pillar[i].x_true; // Формируем окружности
            c1.y = pillar[i].y_true;
            c1.r = pillar[i].distance_laser;
            //printf("DIAMETR distance_laser= %f i= %i ", pillar[i].distance_laser * 2, i);
            c2.x = pillar[j].x_true;
            c2.y = pillar[j].y_true;
            c2.r = pillar[j].distance_laser;
            int rez = getCrossing(c1, c2, pose_, point, 0.1); // Считаем пересечение окружнойстей и в итоге получаем текущее положение по 2 окружностям отбираем те точки котрые не дальше 0,1 метра
            if (rez == 1)                                     // Если результат True значит нашли пересечения иначе пропускаем
            {
                poseLidarMode3[count_poseLidarMode3].x = point.x;
                poseLidarMode3[count_poseLidarMode3].y = point.y;
                count_poseLidarMode3++;
                // printf(" | newCrossing ");
                // printf("c1.x=%.3f y=%.3f r=%.3f ", c1.x, c1.y, c1.r);
                // printf(" | c2.x=%.3f y=%.3f r=%.3f ", c2.x, c2.y, c2.r);
                // printf(" | point x=%.3f y=%.3f \n", point.x, point.y);
            }
            //}
        }
    }
    ROS_INFO("!!! count_poseLidarMode3 CROSSING = %i \n", count_poseLidarMode3);
    if (count_poseLidarMode3 != 0)
    {
        // Усредняем все найденные координаты
        for (int i = 0; i < count_poseLidarMode3; i++)
        {
            poseLidar.x += poseLidarMode3[i].x;
            poseLidar.y += poseLidarMode3[i].y;
        }
        poseLidar.x = poseLidar.x / count_poseLidarMode3;
        poseLidar.y = poseLidar.y / count_poseLidarMode3;

        poseLidar.th = 90 - getTheta(poseLidar, 3); // Получаем угол куда смотрит нос лидара в системе "odom"
        // ROS_INFO("---");                          //
        ROS_WARN_THROTTLE(THROTTLE_PERIOD_3,"MODE3 pose.x= %.3f y= %.3f theta= %.3f ", poseLidar.x, poseLidar.y, poseLidar.th);
        // ROS_INFO("==="); //
        // return poseLidar;
        poseReturn_ = poseLidar; // Если насчитали новую позицию то возвращаем новую, иначе не меняем
    }
    ROS_INFO("--- getLocationMode3 ");
}
// Метод возврщает положение центра лидара усредненного по всем столбам по методу "растояние до столбов по лидару"
void CPillar::getLocationMode1(SPose &poseReturn_, SPose pose_) // На вход подается последняя полученная/посчитанная позиция лидара
{
    ROS_INFO("+++ getLocationMode1");
    // ROS_INFO("pose IN x= %.3f y= %.3f theta= %.3f ", pose_.x, pose_.y, pose_.theta);
    // ROS_INFO("---"); //
    float a1, a2;
    SCircle c1, c2;
    SPoint point;
    SPose poseLidar; // Позиции лидара по расчетам

    count_poseLidarMode1 = 0; // Обнуляем счетчик

    for (int i = 0; i < countPillar - 1; i++) //        Перебираем столбы, и для каждого пересечение окружностей со следующим
    {
        for (int j = i + 1; j < countPillar; j++)
        {
            c1.x = pillar[i].x_true; // Формируем окружности
            c1.y = pillar[i].y_true;
            c1.r = pillar[i].distance_lidar + PILLAR_RADIUS;
            c2.x = pillar[j].x_true;
            c2.y = pillar[j].y_true;
            c2.r = pillar[j].distance_lidar + PILLAR_RADIUS;
            int rez = getCrossing(c1, c2, pose_, point, 0.1); // Считаем пересечение окружнойстей и в итоге получаем текущее положение по 2 окружностям отбираем те точки котрые не дальше 0,1 метра
            if (rez == 1)                                     // Если результат True значит нашли пересечения иначе пропускаем
            {
                poseLidarMode1[count_poseLidarMode1].x = point.x;
                poseLidarMode1[count_poseLidarMode1].y = point.y;
                count_poseLidarMode1++;
            }
        }
    }
    if (count_poseLidarMode1 != 0)
    {
        // Усредняем все найденные координаты
        for (int i = 0; i < count_poseLidarMode1; i++)
        {
            poseLidar.x += poseLidarMode1[i].x;
            poseLidar.y += poseLidarMode1[i].y;
        }
        poseLidar.x = poseLidar.x / count_poseLidarMode1;
        poseLidar.y = poseLidar.y / count_poseLidarMode1;

        poseLidar.th = 90 - getTheta(poseLidar, 1); // Получаем угол куда смотрит нос лидара в системе "odom" + 90 так как сситема координат повернута относительно /odom/
        ROS_WARN_THROTTLE(THROTTLE_PERIOD_3,"MODE1 pose.x= %.3f y= %.3f theta= %.3f ", poseLidar.x, poseLidar.y, poseLidar.th);
        // return poseLidar;
        poseReturn_ = poseLidar;
    }
    else
    {
        ROS_ERROR("MODE1 count_poseLidarMode1= %i \n", count_poseLidarMode1);
    }
    ROS_INFO("--- getLocationMode1");
}

// Метод возвращает угол между ось. Y вверх и напрвлением лидара вперед Сравнивается с Azimit который посчитали ранее
float CPillar::getTheta(SPose poseLidar_, int mode_)
{
    // Находим куда смотрит лидар на основе полученного положения и результатов с лидара фактических
    float gamma = 0;
    int count = 0; // Для усреднения считаем сколько столбов было
    for (int i = 0; i < countPillar; i++)
    {
        // printf("pillar[i].status = %i \n",pillar[i].status);
        if (pillar[i].status) // Берем только сопоставленные столбы
        {
            float a = (pillar[i].x_true - poseLidar_.x);
            float b = (pillar[i].y_true - poseLidar_.y);
            float alfa = RAD2DEG(atan2(a, b));
            // ROS_INFO("a= %.3f b= %.3f  alfa= %.3f ", a, b, alfa); // Это вектор из нашего текущего положения на столб и угол на него от оси Y
            (alfa < 0) ? alfa += 360 : alfa = alfa;
            if (mode_ == 1)
            {
                pillar[i].theta_true1 = alfa;
            }
            if (mode_ == 2)
            {
                pillar[i].theta_true2 = alfa;
            }
            if (mode_ == 3)
            {
                pillar[i].theta_true3 = alfa;
            }
            float delta = (alfa - pillar[i].azimuth);
            float delta2 = delta;
            if (delta < -180) // Если больше значит угол не с той стороны измерили
            {
                delta2 = delta + 360;
            }
            if (delta > 180) // Если больше значит угол не с той стороны измерили
            {
                delta2 = delta - 360;
            }
            gamma = gamma + delta2;
            count++; // Считаем на сколько отличаются углы, это значит на столько повернут нос лидара
            // ROS_INFO("alfa360= %.3f azimuth= %.3f delta= %.3f delta2= %.3f ", pillar[i].azimuth, alfa, delta, delta2); //
        }
        else
        {
            ROS_WARN(" !!! getTheta Pillar %i status FALSE mode %i", i, mode_); //
        }
        // ROS_INFO("---"); //
    }
    // ROS_INFO("gamma= %.3f count= %i ", gamma, count);
    float rez = 0;
    if (count != 0)
    {
        rez = (gamma / count); // Усредняем угол
    }
    return rez;
}

// Метод возврщает положение центра лидара усредненного по всем столбам по методу "углы между столбами по лидару"
void CPillar::getLocationMode2(SPose &poseReturn_, SPose pose_) // На вход подается последняя полученная/посчитанная позиция лидара
{
    ROS_INFO("getLocationMode2");
    // ROS_INFO("pose IN x= %.3f y= %.3f theta= %.3f ", pose_.x, pose_.y, pose_.theta);
    SPoint p1, p2, p3;
    float a1, a2;
    SCircle2 c;
    SCircle c1, c2;
    SPoint point;
    SPose poseLidar; // Позиции лидара по расчетам

    count_circleAnglePillar = 0;
    count_poseLidarMode2 = 0; // Обнуляем счетчик

    for (int i = 0; i < countPillar - 1; i++) //        Перебираем столбы, и для каждой пары формируем окружность
    {
        for (int j = i + 1; j < countPillar; j++)
        {
            p1.x = pillar[i].x_true; // Формируем пары столбов, получаем угол между ними и если он в пределах от 30 до 150 градусов то считаем окружность которую потом переберем
            p1.y = pillar[i].y_true;
            p2.x = pillar[j].x_true;
            p2.y = pillar[j].y_true;
            a1 = pillar[j].azimuth - pillar[i].azimuth;
            (a1 < 0) ? (a1 = a1 + 360) : a1 = a1;   // Проверка и приведение если через ноль столбы
            (a1 > 180) ? (a1 = 360 - a1) : a1 = a1; // Проверка и приведение если через ноль столбы
            if (a1 > 30 && a1 < 150)                // Проверка угла. Если вне диапазона то результаты не точные
            {
                c = getCircle(p1, p2, a1); // Ищем 2 окружности по 2 точкам и углу между ними  https://lektsia.com/1x1ff.html https://studfile.net/preview/1853275/page:2/ https://vk.com/@theoryofsailing-navigaciya-glava-5-33-opredelenie-mesta-po-dvum-gorizontalny
                c.c1.r = c.c1.r + 0.01;    // Гипотеза что если увеличить радиус чуть-чуть по будет больше пересечений и лучше результат )
                c.c2.r = c.c2.r + 0.01;
                circleAnglePillar[count_circleAnglePillar] = c.c1; // Записываем 2 окружности
                count_circleAnglePillar++;
                circleAnglePillar[count_circleAnglePillar] = c.c2;
                count_circleAnglePillar++;
            }
        }
    }
    ROS_INFO("count_circleAnglePillar = %i", count_circleAnglePillar);
    for (int i = 0; i < count_circleAnglePillar - 1; i++) //        Перебираем окружности, и для каждого пересечение окружностей со следующим
    {
        for (int j = i + 1; j < count_circleAnglePillar; j++)
        {
            c1 = circleAnglePillar[i];
            c2 = circleAnglePillar[j];
            int rez = getCrossing(c1, c2, pose_, point, 0.1); // Считаем пересечение окружнойстей и в итоге получаем текущее положение по 2 окружностям
            if (rez == 1)                                     // Если результат 1 значит нашли пересечения иначе пропускаем
            {
                poseLidarMode2[count_poseLidarMode2].x = point.x;
                poseLidarMode2[count_poseLidarMode2].y = point.y;
                count_poseLidarMode2++;
            }
        }
    }
    ROS_INFO("count_poseLidarMode2 = %i", count_poseLidarMode2);
    if (count_poseLidarMode2 > 100)
    {
        ROS_ERROR("count_poseLidarMode2 ERROR !!!!");
        // return poseLidar;
    }
    if (count_poseLidarMode2 > 2) // Если есть хотя бы 2 значения то считаем и усредняем.Иначе возвращаем что и было.
    {
        // Усредняем все найденные координаты
        for (int i = 0; i < count_poseLidarMode2; i++)
        {
            // printf("% .3f % .3f | ", poseLidarMode2[i].x, poseLidarMode2[i].y);
            poseLidar.x += poseLidarMode2[i].x;
            poseLidar.y += poseLidarMode2[i].y;
        }
        // printf("\n");
        poseLidar.x = poseLidar.x / count_poseLidarMode2;
        poseLidar.y = poseLidar.y / count_poseLidarMode2;
        poseLidar.th = 90 - getTheta(poseLidar, 2); // Получаем угол куда смотрит нос лидара в системе "odom"
        ROS_WARN_THROTTLE(THROTTLE_PERIOD_3,"MODE2 pose.x= %.3f y= %.3f theta= %.3f ", poseLidar.x, poseLidar.y, poseLidar.th);
        poseReturn_ = poseLidar;

        // // Находим куда смотрит лидар на основе полученного положения и результтов с лидара фактических
        // float gamma = 0;
        // for (int i = 0; i < countPillar; i++)
        // {
        //     float a = (pillar[i].x_true - poseLidar.x);
        //     float b = (pillar[i].y_true - poseLidar.y);
        //     float alfa = RAD2DEG(atan2(a, b));
        //     (alfa < 0) ? alfa += 360 : alfa = alfa;
        //     pillar[i].theta_true2 = alfa;
        //     gamma = gamma + (alfa - pillar[i].azimuth); // Считаем на сколько отличаются углы
        // }
        // poseLidar.theta = gamma / countPillar; // Усредняем угол погрешности
    }

    else
        ROS_ERROR("MODE2 ERROR count_poseLidarMode2 < 2");

    // ROS_INFO("===");
    // return poseLidar;

    // for (int i = 0; i < countPillar; i++) //        Перебираем столбы, потом ищем 3 штуки начиная с текущего
    // {
    //     p1.x = pillar[i].x_true;
    //     p1.y = pillar[i].y_true;
    //     int j = i + 1;
    //     (j >= countPillar) ? (j = j - countPillar) : j = j;
    //     p2.x = pillar[j].x_true;
    //     p2.y = pillar[j].y_true;
    //     a1 = pillar[j].azimuth - pillar[i].azimuth;
    //     (a1 < 0) ? (a1 = a1 + 360) : a1 = a1; // Проверка и приведение если через ноль столбы
    //     int k = i + 2;
    //     (k >= countPillar) ? (k = k - countPillar) : k = k;
    //     p3.x = pillar[k].x_true;
    //     p3.y = pillar[k].y_true;
    //     a2 = pillar[k].azimuth - pillar[j].azimuth;
    //     (a2 < 0) ? (a2 = a2 + 360) : a2 = a2; // Проверка и приведение если через ноль столбы
    //     c1 = getCircle(p1, p2, a1, p0);       // Ищем окружность по 2 точкам и углу между ними, для выбора окружности из 2 используем расстояние до предыдущей позиции https://lektsia.com/1x1ff.html https://studfile.net/preview/1853275/page:2/ https://vk.com/@theoryofsailing-navigaciya-glava-5-33-opredelenie-mesta-po-dvum-gorizontalny
    //     c2 = getCircle(p2, p3, a2, p0);       // Ищем окружность по 2 точкам и углу между ними, для выбора окружности из 2 используем расстояние до предыдущей позиции https://trans-service.org/ru.php?section=info&page=navi&subpage=opr_mest_observ_04
    //     ROS_INFO("c1_x= %.3f  c1_y= %.3f  / c1_r= %.3f ", c1.x, c1.y, c1.r);
    //     ROS_INFO("c2_x= %.3f  c2_y= %.3f  / c2_r= %.3f ", c2.x, c2.y, c2.r);
    //     SPoint point = getCrossing(c1, c2, p0);           // Считаем пересечение окружнойстей и в итоге получаем текущее положение по 2 окружностям
    //     poseLidar[i] = calculationPose(point, pillar[i]); // Заполняем позицию лидара посчитанную по 3 столбам начиная с заданного, расчитываем теоретический азимут исходя из позиции и первого столба в расчете
    // }

    // a1 = 52.9435;

    // ROS_INFO("p1x= %.3f p1y= %.3f  / p2x= %.3f p2y= %.3f  / angle = %.3f / p0x= %.3f p0y= %.3f ", p1.x, p1.y, p2.x, p2.y, a1, p0.x, p0.y);

    // a1 = pillar[1].azimuth - pillar[0].azimuth;
    // a2 = 66.8014;

    // SCircle c2 = getCircle(p2, p3, a2, pose_);
    // ROS_INFO("c2x= %.3f  c2y= %.3f  / c2r= %.3f ", c2.x, c2.y, c2.r);

    // SPose ret;
    // ret.x = poseLidar.x;
    // ret.y = poseLidar.y;
    // ret.theta = poseLidar.theta;
    ROS_INFO("end getLocationMode2");
}
// Функция возвращает 2 окружности(координаты и радиус)
// Основанно на идее что вписанный угол в окружность равен половине угла из центра на эту же хорду. Далее по формулам отсюда  Нахождение центра окружности по 2м точкам и углу https://dxdy.ru/topic4265.html
SCircle2 CPillar::getCircle(SPoint p1_, SPoint p2_, float angle_) // На вход две точки, угол между ними и координаты старого местонахождения
{
    SCircle2 circle;
    SCircle c1;
    SCircle c2;
    angle_ = DEG2RAD(angle_); // Переводим в радианы
    // Находим радиус
    float temp1 = (sqr(p1_.x - p2_.x) + sqr(p1_.y - p2_.y));
    float temp2 = 4 * sqr(sin(angle_));
    c1.r = c2.r = sqrt(temp1 / temp2); // Находим радиус окружности
    // Таких окружностей может быть две. Находим координнаты центра для обоих
    temp1 = (p1_.x + p2_.x) / 2.0;
    temp2 = (p1_.y - p2_.y) / 2.0;
    c1.x = temp1 + (temp2 * ctan(angle_));
    c2.x = temp1 - (temp2 * ctan(angle_));
    temp1 = (p1_.x - p2_.x) / 2.0;
    temp2 = (p1_.y + p2_.y) / 2.0;
    c1.y = temp2 - (temp1 * ctan(angle_));
    c2.y = temp2 + (temp1 * ctan(angle_));

    circle.c1 = c1; // Возвращаем 2 окружности
    circle.c2 = c2;

    return circle;
}
// Функция возвращает одну из двух точек пеерсечения двух окружностей ближайшую к заданной
int CPillar::getCrossing(SCircle c1_, SCircle c2_, SPose pose_, SPoint &pointX_, float len_) // https://algolist.manual.ru/maths/geom/intersect/circlecircle2d.php https://planetcalc.ru/8098/ https://paulbourke.net/geometry/circlesphere/
{
    int ret = 0;
    // Находим раастояние между центрами окружностей
    SPoint p0;
    p0.x = pose_.x;
    p0.y = pose_.y;
    SPoint p1;
    p1.x = c1_.x;
    p1.y = c1_.y;
    SPoint p2;
    p2.x = c2_.x;
    p2.y = c2_.y;
    float d = vectorLen(p1, p2); // Растояние между центрами окружностей
    // Делаем проверку на расстояние между точками
    if ((d >= c1_.r + c2_.r) || (d < abs(c1_.r - c2_.r))) // Если расстояние меньше суммы радиусов то окружности не пересекаются Если равно то одна точка, нам не подходит/ Если меньше разницы то окружность внутри другой тоже не наш случай
    {
        // ROS_WARN("Circles do not fit the conditions !!! d= %.3f  r1= %.3f r2= %.3f", d, c1_.r, c2_.r);
        ret = -1; // Ошибка по окружностям
        return ret;
    }

    float a = (sqr(c1_.r) - sqr(c2_.r) + sqr(d)) / (2.0 * d);
    float h = sqrt(sqr(c1_.r) - sqr(a));

    SPoint p3; // Промежуточная точка для расчетов
    p3.x = c1_.x + ((a / d) * (c2_.x - c1_.x));
    p3.y = c1_.y + ((a / d) * (c2_.y - c1_.y));

    SPoint p4, p5; // Итоговые точки пересечений

    p4.x = p3.x - ((h / d) * (c2_.y - c1_.y));
    p4.y = p3.y + ((h / d) * (c2_.x - c1_.x));

    p5.x = p3.x + ((h / d) * (c2_.y - c1_.y));
    p5.y = p3.y - ((h / d) * (c2_.x - c1_.x));

    float vl1 = vectorLen(p4, p0);
    float vl2 = vectorLen(p5, p0);
    (vl1 < vl2) ? pointX_ = p4 : pointX_ = p5; // Находим центр какой окружности ближе к последним координатам
    float vl3 = vectorLen(pointX_, p0);
    if (isnan(vl3))
        ROS_ERROR("NAN vl3");
    if (vl3 > len_ || isnan(vl3)) // Если меньший вектор больше минимального заданного то пересечения не подходят или
    {
        ret = -2;      // Нет подходящих пересечений
        pointX_.x = 0; // Возвращаем нули
        pointX_.y = 0;
        // ROS_WARN("Not getCrossing. vectorLen > 0.1 m");
        return ret;
    }
    ret = 1; // Все хорошо
    return ret;
}
// Считаем отклонение азимута полученного с лидара от направления которое должно быть от итогового местоположения
// SPose CPillar::calculationPose(SPoint point_, SPillar pillar_)
// {
//     SPose ret;
//     ret.x = point_.x;
//     ret.y = point_.y;

//     return ret;
// }

// Разбор столбов из топика в массив
void CPillar::parsingPillar(pb_msgs::topicPillar &pillar_)
{
    countPillar = 0; // Обнуляем счетчик столбов
    ROS_INFO("+++ parsingPillar");
    for (int i = 0; i < pillar_.pillar.static_size; i++)
    {
        pillar[i].x_true = pillar_.pillar[i].x;
        pillar[i].y_true = pillar_.pillar[i].y;
        countPillar++;
        ROS_INFO("pillarTrue[ %i ].x= %.3f pillarTrue[ %i ].y= %.3f ", i, pillar[i].x_true, i, pillar[i].y_true);
    }
    ROS_INFO("--- parsingPillar");
}

// Функция сопоставления истинных столбов с лидарными
void CPillar::comparisonPillar()
{
    ROS_INFO("+++ comparisonPillar");
    float delta_x = 0;
    float delta_y = 0;
    float hypotenuse = 0;
    int countComparisonPillar = 0;

    for (int i = 0; i < countPillar; i++) // Перебираем все истинные столбы и ищем соответствие из лидарных столбов]
    {
        pillar[i].status = false; // Сбрасываем старое сопоставление, остаются только истинные значения столбов. Сопоставляться должно каждый раз!!! или столб не участвует в расчетах дальше.
        pillar[i].azimuth = 0;
        pillar[i].x_lidar = 0;
        pillar[i].y_lidar = 0;
        // printf("i= %i \n",i);

        for (int j = 0; j < countPillarLidar; j++) // Перебираем лидарные столбы
        {
            delta_x = abs(pillar[i].x_true - pillarLidar[j].x_globalXY); // Находим разницу в координатах
            delta_y = abs(pillar[i].y_true - pillarLidar[j].y_globalXY); //
            hypotenuse = sqrt(sqr(delta_x) + sqr(delta_y));              // Теорема пифагора
            if (hypotenuse < OTKLONENIE_PILLAR)                          // Если отклонение меньше предела то столб сопоставили и прерываем перебор
            {
                pillar[i].status = true; // Статус что есть сопоставление
                pillar[i].azimuth = pillarLidar[j].azimuth;
                pillar[i].hypotenuse = hypotenuse;
                pillar[i].distance_lidar = pillarLidar[j].dist_min;
                pillar[i].x_lidar = pillarLidar[j].x_globalXY;
                pillar[i].y_lidar = pillarLidar[j].y_globalXY;
                //printf("comparisonPillar %i azimuth = %.3f distance_lidar= %.3f hypotenuse= %.3f", i, pillar[i].azimuth, pillar[i].distance_lidar, hypotenuse);
                countComparisonPillar++;
                break;
            }
            else
            {
                // ROS_WARN("!!! NO comparisonPillar %i hypotenuse= %.3f", i, hypotenuse);
            }
        }
    }
    ROS_INFO("countComparisonPillar %i", countComparisonPillar);
    ROS_INFO("end comparisonPillar.");
}

// Функция которую вызываем из колбека по расчету места столбов. Поиск лидарных столбов.
// Что-бы данные приходили по часовой стрелке в настройках установить   <param name="inverted"            type="bool"   value="true"/>
// Ноль градусов у нее там где хвостик и поэтому +180 чтобы получить 360 оборот правильный вправо
void CPillar::parsingLidar(const sensor_msgs::LaserScan::ConstPtr &scan, SPose &poseLidarMode_)
{
    ROS_INFO("parsingLidar");
    float distance_pred = 0;    // Предыдущее значение дистанции
    int a = 0;                  // Начало группы точек
    int b = 0;                  // Конец группы точек
    bool flag = false;          // Флаг поиска точки
    SLidar lidarData[4096];     // Выделяем массив из расчета что всего 32000 за секунду и вращаемся 10 Герц с запасом
    SLidar lidarDataLast[4096]; // Выделяем массив из расчета что всего 32000 за секунду и вращаемся 10 Герц с запасом
    SLidar lidarDataSrc[4096];  // Преобразованный к моему виду массив
    countPillarLidar = 0;       // Обнуляем для нового сканирования, каждый раз можем находить разное число столбов
    int count = scan->scan_time / scan->time_increment;

    // ROS_INFO("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! ");
    ROS_INFO("=== Lidar data received %s [%d]:", scan->header.frame_id.c_str(), count);
    // ROS_INFO("angle_range, %f, %f", RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max));

    int j = 0;
    for (int i = 0; i < count; i++) // Пробегаем по всему массиву, копируем в свой и считаем сколько хороших данных пришло и для дальнейшего преобразования
    {
        float distance = scan->ranges[i];                                    // Дистанция для данной точки
        float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i); // Градус для данной точки
        if (isfinite(distance))                                              // Проверка что там нормальное число а не бесконечность и не ноль
        {
            lidarDataSrc[j].degree = degree;     // Градус в градусах для данной точки
            lidarDataSrc[j].distance = distance; // Дистанция для данной точки
            j++;
            // if (i < 5)
            // {
            //     ROS_INFO("i = %i degree= %.3f distance= %.3f", i, degree, distance);
            // }
            // if (i > 3575)
            // {
            //     ROS_INFO("i = %i degree= %.3f distance= %.3f", i, degree, distance);
            // }
        }
    }
    // printf("Lidar good data count = %i ", j);
    int k = j;
    for (int i = 0; i < j; i++) // Пробегаем по всему своему массиву c исходными данными с конца и преобразуем (инвертируем) его в нормальный понятный для меня вид. Вращение вправо и 360 градусов
    {
        k--;
        lidarData[i].degree = 180 - lidarDataSrc[k].degree; // Конец копируем в начало и превращаем в 360 градусов
        lidarData[i].distance = lidarDataSrc[k].distance;
        // if (i < 5)
        // {
        //     ROS_INFO("i = %i degree= %.4f distance= %.4f", i, lidarData[i].degree, lidarData[i].distance);
        // }
        // if (i > 3450)
        // {
        //     ROS_INFO("i = %i degree= %.4f distance= %.4f", i, lidarData[i].degree, lidarData[i].distance);
        // }
    }
    ROS_INFO("transformed data = %i", k);
    for (int i = 0; i < j; i++) // Пробегаем по всему новому массиву и ищем группы точек
    {
        if (i != 0) // Для первого элемента просто его запоминаем в
        {
            if (lidarData[i].distance - distance_pred > PEREHOD && flag) // Если дальность текущей точки больше предыдущей больше чем на 0,3 метра и ранее была найдена точка a
            {
                b = i;
                // ROS_INFO("Point a= %i degree= %.3f distance= %.3f ", a, lidarData[a].degree, lidarData[a].distance);
                // ROS_INFO("Point b= %i degree= %.3f distance= %.3f \n", b, lidarData[b].degree, lidarData[b].distance);
                //  ROS_INFO("=====");
                poiskPillar(a, b, lidarData, poseLidarMode_);
                flag = false; // Сбрасываем флаг для следующего расчета
            }
            if (distance_pred - lidarData[i].distance > PEREHOD) // Если дальность текущей точки меньше предыдущей больше чем на 0,3 метра то
            {
                a = i; // Запоминаем первую точку
                // ROS_INFO("Point a= %i degree= %f distance= %f distance_pred = %f , Perepad= %f", a, lidarData[a].degree, lidarData[a].distance, distance_pred, lidarData[a].distance - distance_pred);
                flag = true;
            }
        }
        distance_pred = lidarData[i].distance; // запоминаем для следующего сравнения
        // if (lidarData[i].distance > -50 && lidarData[i].distance < -20) // Печать для отладки диапазон
        // {
        //     ROS_INFO(": [%f, %f]", degree, scan->ranges[i]);
        // }
    }

    if (flag) // Если ранее нашли точку а, начало столба и не нашли конец, то надо снова искать сначала массива
    {
        int l = 0;                  // Счетчик в новом массиве
        for (int i = 0; i < j; i++) // Пробегаем по всему новому массиву и ищем группы точек
        {
            if (lidarData[i].distance - distance_pred > PEREHOD && flag) // Если дальность текущей точки больше предыдущей больше чем на 0,3 метра и ранее была найдена точка a
            {
                b = i;
                // ROS_WARN("=== LAST PILLAR ==");
                // ROS_WARN("=====");
                // ROS_WARN("Point a= %i degree= %.3f distance= %.3f ", a, lidarData[a].degree, lidarData[a].distance);
                // ROS_WARN("Point b= %i degree= %.3f distance= %.3f \n", b, lidarData[b].degree, lidarData[b].distance);
                // ROS_WARN("=====");

                for (int i = a; i < j; i++) // Пишем в новый массив ту часть которая в конце старого массива.
                {
                    lidarDataLast[l] = lidarData[i];
                    l++;
                }
                for (int i = 0; i <= b; i++) // Пишем в новый массив ту часть которая в начале старого массива.
                {
                    lidarDataLast[l].distance = lidarData[i].distance;
                    lidarDataLast[l].degree = lidarData[i].degree + 360;
                    l++;
                }
                lidarDataLast[l].distance = lidarData[b + 1].distance; // Пишем еще один элемент чтобы угол правильно посчитать
                lidarDataLast[l].degree = lidarData[b + 1].degree + 360;

                poiskPillar(0, l, lidarDataLast, poseLidarMode_); // Передаем новый массив, его смотрис от 0 позиции до конца
                // flag = false; // Сбрасываем флаг для следующего расчета
                break;
            }
            distance_pred = lidarData[i].distance; // запоминаем для следующего сравнения
        }
    }
    ROS_INFO("Found countPillarLidar= %i ", countPillarLidar);
    ROS_INFO("end parsingLidar.");
}

void CPillar::poiskPillar(int a_, int b_, SLidar *lidarData, SPose &poseLidarMode_)
{
    static float width = 0; // Ширина обьекта
    float dist_min = 1000;
    float angle_min = 0;
    // Надодим минимальное растояние до столба, но угол на эту точку будет первый попавшийся при перебеоре, а нам нужно точнее найти центр. Так как много точек может быть с таким же растонием или очень близким к нему
    for (int i = a_; i < b_; i++) // Перебираем точки без последней, так как она уже за столбом по дальности
    {
        if (lidarData[i].distance < dist_min) // Ищем минимальную дистанцию. Может бытьнесколько значений с одинаковым растояние и надо бы среднюю найти
        {
            dist_min = lidarData[i].distance;
            angle_min = lidarData[i].degree; // запоминаем дистанцию и угол при ней
            (angle_min > 360) ? (angle_min -= 360) : (angle_min = angle_min);
        }
    }
    float angle_middle_min = 0;
    int k = 0;
    for (int i = a_; i < b_; i++) // Пробегаем еще раз
    {
        if (lidarData[i].distance < dist_min + DELTA_MINIMUMA) // Ищем не просто минимальную дистанцию, а все точки которые отличаются от минисмальной на заданную величину
        {
            angle_middle_min += lidarData[i].degree; // И складываем их углы
            k++;
        }
    }
    angle_middle_min = angle_middle_min / k; // Находим средний угол.
    (angle_middle_min > 360) ? (angle_middle_min -= 360) : (angle_middle_min = angle_middle_min);

    float angle_a = lidarData[a_].degree;
    float angle_b = lidarData[b_].degree;
    float angle_middle = (angle_a + angle_b) / 2.0; // Находим угол на середину
    (angle_middle > 360) ? (angle_middle -= 360) : (angle_middle = angle_middle);

    float angle_sin = (angle_b - angle_a) / 2.0; // Находим половину угла между крайними точками

    width = (tan(DEG2RAD(angle_sin)) * (dist_min + PILLAR_RADIUS)) * 2; // Вычисляем ширину столба По минимальному расстоянию

    // ROS_INFO("Point b= %i width = %f", b, width);         // Запоминаем вторую точку  и начинаем анализировать эту группу точек
    // ROS_INFO("angle_ab= %f max_dist= %f width = %f", angle_ab, max_dist, width); // Ширина столба

    if (width > 0.2 && width < 0.4) // Если ширина похожа на наш столб
    {
        int x = 0;
        for (int i = a_; i <= b_; i++) // Перебираем точки с захватом крайних для анализа если нужно будет
        {
            pillarLidar[countPillarLidar].data[x].degree = lidarData[i].degree;
            pillarLidar[countPillarLidar].data[x].distance = lidarData[i].distance;
            x++;
        }
        pillarLidar[countPillarLidar].angle_left = angle_a;
        pillarLidar[countPillarLidar].angle_right = angle_b;
        pillarLidar[countPillarLidar].angle_dist_min = angle_min;          // Угол при на точку с минимальной дистанцией которую нашли первой
        pillarLidar[countPillarLidar].angle_middle_min = angle_middle_min; // Угол полученный расчетам как средний угол точек у которых дистанция до них была мешьше минимальной плюс дельта
        pillarLidar[countPillarLidar].angle_middle = angle_middle;         // Берем середину столба, хотя из-за вращения происходит смещения и лучше бы еще как-то уточнять
        // pillarLidar[countPillarLidar].azimuth = (angle_min + angle_middle) / 2.0; // Итоговый угол на столб. Пока считаем как угол на минимум. Или можно усреднять например со средним углом. Нало посмотреть что будет точнее
        // pillarLidar[countPillarLidar].azimuth = angle_middle_min; // Итоговый угол на столб. Берется как средний по точкам расчетный
        pillarLidar[countPillarLidar].azimuth = angle_middle + offsetAngle; // Итоговый угол на столб. Берется как средний по точкам расчетный
        pillarLidar[countPillarLidar].dist_min = dist_min;
        pillarLidar[countPillarLidar].width = width;
        pillarLidar[countPillarLidar].x_lidarXY = cos(DEG2RAD(-pillarLidar[countPillarLidar].azimuth)) * (pillarLidar[countPillarLidar].dist_min + PILLAR_RADIUS); // Находим координаты по формулам. К минимальному растоянию прибавляем радиус столба
        pillarLidar[countPillarLidar].y_lidarXY = sin(DEG2RAD(-pillarLidar[countPillarLidar].azimuth)) * (pillarLidar[countPillarLidar].dist_min + PILLAR_RADIUS); // Находим координаты по формулам. К минимальному растоянию прибавляем радиус столба
        //printf("x_lidarXY = % .3f y_lidarXY = % .3f poseLidarMode_.th = % .3f | ", pillarLidar[countPillarLidar].x_lidarXY, pillarLidar[countPillarLidar].y_lidarXY, poseLidarMode_.th);

        // SPoint car_XY = povorotSystemCoordinate(pillarLidar[countPillarLidar].x_lidarXY, pillarLidar[countPillarLidar].y_lidarXY, -poseLidarMode_.th - 90); // Поворачиваем систему координат/ Угол с минусом так как вращаем против часовой
        // pillarLidar[countPillarLidar].x_globalXY = car_XY.x + poseLidarMode_.x;                                                                       // Прибавляем смещение. Это раастояние где находится машина относительно глобальной системы координат нуля. И получаем координаты в глобальной системе координат
        // pillarLidar[countPillarLidar].y_globalXY = car_XY.y + poseLidarMode_.y;
        // printf("x_globalXY = % .3f y_globalXY = % .3f \n", pillarLidar[countPillarLidar].x_globalXY, pillarLidar[countPillarLidar].y_globalXY);

        SPoint in;
        in.x = pillarLidar[countPillarLidar].x_lidarXY;
        in.y = pillarLidar[countPillarLidar].y_lidarXY;
        SPoint TTT = pointLocal2GlobalRos(in, poseLidarMode_);
        pillarLidar[countPillarLidar].x_globalXY = TTT.x; // Прибавляем смещение. Это раастояние где находится машина относительно глобальной системы координат нуля. И получаем координаты в глобальной системе координат
        pillarLidar[countPillarLidar].y_globalXY = TTT.y;
        //printf("x_global = % .3f y_global = % .3f | ", TTT.x, TTT.y);

        // Старый вариант, без передачи перемнной через метод, а прямо использовани глобальной перемнной
        // SPoint car_XY = povorotSystemCoordinate(pillarLidar[countPillarLidar].x_lidarXY, pillarLidar[countPillarLidar].y_lidarXY, -g_poseLidar.mode1.th); // Поворачиваем систему координат/ Угол с минусом так как вращаем против часовой
        // pillarLidar[countPillarLidar].x_globalXY = car_XY.x + g_poseLidar.mode1.x;                                                                           // Прибавляем смещение. Это раастояние где находится машина относительно глобальной системы координат нуля. И получаем координаты в глобальной системе координат
        // pillarLidar[countPillarLidar].y_globalXY = car_XY.y + g_poseLidar.mode1.y;

        // ROS_INFO(" Pillar Angle a = %.3f Angle b= %.3f ", pillarLidar[countPillarLidar].angle_left, pillarLidar[countPillarLidar].angle_right);
        // ROS_INFO(" Angle_middle= %.3f Angle_dist_min= %.3f angle_middle_min= %.3f Angle_azimuth= %.3f ", pillarLidar[countPillarLidar].angle_middle, pillarLidar[countPillarLidar].angle_dist_min, pillarLidar[countPillarLidar].angle_middle_min, pillarLidar[countPillarLidar].azimuth);
        //printf(" Angle_middle= %.3f dist_min= %.3f middle_min= %.3f \n", pillarLidar[countPillarLidar].angle_middle, pillarLidar[countPillarLidar].angle_dist_min, pillarLidar[countPillarLidar].angle_middle_min);
        // ROS_INFO(" dist_min= %.3f width= %.3f ", pillarLidar[countPillarLidar].dist_min, pillarLidar[countPillarLidar].width);
        // ROS_INFO(" x_lidarXY= %.3f y_lidarXY= %.3f ", pillarLidar[countPillarLidar].x_lidarXY, pillarLidar[countPillarLidar].y_lidarXY);
        // ROS_INFO(" x_globalXY= %.3f y_globalXY= %.3f ", pillarLidar[countPillarLidar].x_globalXY, pillarLidar[countPillarLidar].y_globalXY);

        // ROS_INFO("=====");
        countPillarLidar++;
    }
    else
    {
        // ROS_INFO("BED width --->>> Pillar width= %.2f min_dist= %.2f max_dist= %.2f Angle = %.3f angle = %.3f", width, min_dist, max_dist, RAD2DEG(scan->angle_min + scan->angle_increment * a_), RAD2DEG(scan->angle_min + scan->angle_increment * (b_ - 1)));
        // ROS_INFO("Angle = %.3f ", (RAD2DEG(scan->angle_min + scan->angle_increment * a_) + RAD2DEG(scan->angle_min + scan->angle_increment * (b_ - 1))) / 2);
    }
}

#endif
