#ifndef PILLAR_H
#define PILLAR_H

#include "config.h"

#define RAD2DEG(x) ((x) * 180. / M_PI)
#define DEG2RAD(x) ((x) * M_PI / 180.)
#define PILLAR_RADIUS 0.1575  // Радиус столба 315 мм
#define PEREHOD 0.15          // Разрыв между столбом и обьектом за ним
#define OTKLONENIE_PILLAR 0.3 // Отклонение межлу истинным и лидарным столююов при котором они сопостовляются

// Класс для столба по которому определяем свою позицию
class CPillar
{
private:
    struct SLidar
    {
        float degree = 0;   // Направление с лидара Градус
        float distance = 0; // Дистанция с лидара Диcтанция
    };


    struct SPillar // Итоговая структура по столбам. Ее потом опубликуем
    {
        bool status = false; // Статус сопоставлен столб или нет
        float azimuth = 0; // Дидарные значения появляются при сопоставлении
        float hypotenuse = 0;
        float distance = 0;
        float x_true = 0; // Истинные значения задаются из топика
        float y_true = 0;
        float x_lidar = 0;
        float y_lidar = 0;
    };

    // Структура куда записываем все столбы которые мы нашли на основе сканировавания по лидару
    struct SPillarLidar
    {
        SLidar data[256];         // данные с лидара по этому столбу, всего 256 точек максимум
        float angle_left = 0;     // крайний левый угол
        float angle_right = 0;    // крайний правый угол
        float angle_dist_min = 0; // угол с лидара с минимальным растоянием. Его считаем итоговым напрвлением на столб
        float angle_middle = 0;   // Угол на примерную середину столба. Считается как середина точек участвующих в формировании стобла
        float azimuth = 0;        // Итоговый угол на столб. Пока считаем как угол на минимум. Или можно усреднять например со средним углом. Нало посмотреть что будет точнее
        float dist_min = 0;
        float width = 0;      // Ширина столба
        float x_lidarXY = 0;  // Расчетные координаты столба в системе координат лидара
        float y_lidarXY = 0;  // Расчетные координаты столба в системе координат лидара
        float x_globalXY = 0; // Расчетные координаты столба в глобальной системе координат
        float y_globalXY = 0; // Расчетные координаты столба в глобальной системе координат
    };

    int countPillarLidar = 0; // Колличество столбов отбираемых по алгоритму в массив с Лидара Не знаем точно сколько обнаружим

    // ************************** ПРИВАТНЫЕ ФУНКЦИИ КЛАССА *********************************************
    SPoint povorotSystemCoordinate(float xloc_, float yloc_, float theta_);
    void poiskPillar(int a_, int b_, SLidar *lidarData);
    SPillarLidar pillarLidar[32]; // Массив котором храним данные по столбам

public:
    SPillar pillar[16];           // Массив столбов с итоговым результатом где все столбы сопоставленны
    int countPillar = 0;      //  // Колличество столбов с истинными координатами не знаем точно сколько загрузим

    // ************************** ПУБЛИЧНЫЕ ФУНКЦИИ КЛАССА *********************************************
    CPillar(/* args */);
    ~CPillar();
    void parsingLidar(const sensor_msgs::LaserScan::ConstPtr &scan);     // Функция которую вызываем из колбека по расчету места столбов
    void parsingPillar(data::topicPillar &pillar_);                      // Функция которую вызываем из колбека по расчету места столбов
    void comparisonPillar(); // Функция где сопоставляю столбы

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

CPillar::CPillar(/* args */)
{
}

CPillar::~CPillar()
{
}

// Формулы поворота системы координат. Угол задавать отрицательный если поворачиваем против часовой к нулю который вверх
SPoint CPillar::povorotSystemCoordinate(float xloc_, float yloc_, float theta_)
{
    SPoint ret;
    theta_ = DEG2RAD(theta_); // Превращаем в радианы из градусов
    ret.x = xloc_ * cos(theta_) - yloc_ * sin(theta_);
    ret.y = xloc_ * sin(theta_) + yloc_ * cos(theta_);
    return ret;
}
// Разбор столбов из топика в массив
void CPillar::parsingPillar(data::topicPillar &pillar_)
{
    countPillar = 0; // Обнуляем счетчик столбов
    ROS_INFO("------------------------- parsingPillar -------------------------------------");
    for (int i = 0; i < pillar_.pillar.static_size; i++)
    {
        pillar[i].x_true = pillar_.pillar[i].x;
        pillar[i].y_true = pillar_.pillar[i].y;
        countPillar++;
        ROS_INFO("pillarTrue[ %i ].x= %.3f pillarTrue[ %i ].y= %.3f ", i, pillar[i].x_true, i, pillar[i].y_true);
    }
    ROS_INFO("-------------------------               -------------------------------------");
}

// Функция сопоставления столбов
void CPillar::comparisonPillar()
{
    float delta_x = 0;
    float delta_y = 0;
    float hypotenuse = 0;

    for (int i = 0; i < countPillar; i++) // Перебираем все истинные столбы и ищем соответствие из лидарных столбов]
    {
        pillar[i].status= false; // Сбрасываем старое сопоставление, остаются только истинные значения столбов. Сопоставляться должно каждый раз!!! или столб не удаствует в расчетах дальше.
        pillar[i].azimuth= 0;
        pillar[i].x_lidar= 0;
        pillar[i].y_lidar= 0;
        //printf("i= %i \n",i);

        for (int j = 0; j < countPillarLidar; j++) // Перебираем лидарные столбы
        {
            delta_x = abs(pillar[i].x_true - pillarLidar[j].x_globalXY); // Находим разницу в координатах
            delta_y = abs(pillar[i].y_true - pillarLidar[j].y_globalXY); //
            hypotenuse = sqrt(sqr(delta_x) + sqr(delta_y));                // Теорема пифагора
            if (hypotenuse < OTKLONENIE_PILLAR) //Если отклонение меньше предела то столб сопоставили и прерываем перебор
            {
                pillar[i].status = true; // Статус что есть сопоставление
                pillar[i].azimuth = pillarLidar[j].azimuth;
                pillar[i].hypotenuse = hypotenuse;
                pillar[i].distance = pillarLidar[j].dist_min;
                pillar[i].x_lidar = pillarLidar[j].x_globalXY;
                pillar[i].y_lidar = pillarLidar[j].y_globalXY;
                // printf("          hypotenuse= %.3f \n",pillar[i].hypotenuse);
                // printf("          azimuth= %.3f \n",pillar[i].azimuth);
                // printf("          x_lidar= %.3f \n",pillar[i].x_true);
                // printf("          x_lidar= %.3f \n",pillar[i].x_lidar);
                // printf("          x_lidar= %.3f \n",pillar[i].y_true);
                // printf("          dist_min= %.3f \n",pillarLidar[j].dist_min);
                break;
            }
        }
    }
}

// Функция которую вызываем из колбека по расчету места столбов
// Что-бы данные приходили по часовой стрелке в настройках установить   <param name="inverted"            type="bool"   value="true"/>
// Ноль градусов у нее там где хвостик и поэтому +180 чтобы получить 360 оборот правильный вправо
void CPillar::parsingLidar(const sensor_msgs::LaserScan::ConstPtr &scan)
{
    float distance_pred = 0; // Предыдущее значение дистанции
    int a = 0;               // Начало группы точек
    int b = 0;               // Конец группы точек
    bool flag = false;       // Флаг поиска точки
    SLidar lidarData[4096];  // Выделяем массив из расчета что всего 32000 за секунду и вращаемся 10 Герц с запасом
    int count = scan->scan_time / scan->time_increment;
    countPillarLidar = 0; // Обнуляем для нового сканирования, каждый раз можем находить разное число столбов

    // ROS_INFO("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!! ");
    ROS_INFO("Lidar data received %s [%d]:", scan->header.frame_id.c_str(), count);
    // ROS_INFO("angle_range, %f, %f", RAD2DEG(scan->angle_min), RAD2DEG(scan->angle_max));

    int j = 0;
    for (int i = 0; i < count; i++) // Пробегаем по всему массиву и преобразуем его в нормальный понятный для меня вид. Вращение вправо и 360 градусов
    {
        float distance = scan->ranges[i];                                    // Дистанция для данной точки
        float degree = RAD2DEG(scan->angle_min + scan->angle_increment * i); // Градус для данной точки
        if (isfinite(distance))                                              // Проверка что там нормальное число а не бесконечность и не ноль
        {
            lidarData[j].degree = degree + 180; // Градус в градусах для данной точки
            lidarData[j].distance = distance;   // Дистанция для данной точки
            // ROS_INFO("Point j= %i degree= %f distance= %f ", j, lidarData[j].degree, lidarData[j].distance);
            j++;
        }
    }

    for (int i = 0; i < j; i++) // Пробегаем по всему новому массиву и ищем группы точек
    {
        if (i != 0) // Для первого элемента просто его запоминаем в
        {
            if (lidarData[i].distance - distance_pred > PEREHOD && flag) // Если дальность текущей точки больше предыдущей больше чем на 0,3 метра и ранее была найдена точка a
            {
                b = i;
                //ROS_INFO("Point a= %i degree= %.3f distance= %.3f ", a, lidarData[a].degree, lidarData[a].distance);
                //ROS_INFO("Point b= %i degree= %.3f distance= %.3f \n", b, lidarData[b].degree, lidarData[b].distance);
                // ROS_INFO("=====");
                poiskPillar(a, b, lidarData);
                flag = false; // Сбрасываем флаг для следующего расчета
            }
            if (distance_pred - lidarData[i].distance > PEREHOD) // Если дальность текущей точки меньше предыдущей больше чем на 0,3 метра то
            {
                a = i; // Запоминаем первую точку
                // ROS_INFO("Point a= %i degree= %f distance= %f distance_pred = %f , Perepad= %f", a, lidarData[a].degree, lidarData[a].distance, distance_pred, lidarData[a].distance - distance_pred);
                flag = true;
            }
            // if (true)
            // {
            //     if (abs(ranges - previous) > 0.1)
            //     {
            //         ROS_INFO("Point search = %i degree= %f ranges= %f previous ranges= %f , Perepad= %f", i, degree, ranges, previous, ranges - previous);
            //     }

            // }
        }

        distance_pred = lidarData[i].distance;                          // запоминаем для следующего сравнения
        if (lidarData[i].distance > -50 && lidarData[i].distance < -20) // Печать для отладки диапазон
        {
            // ROS_INFO(": [%f, %f]", degree, scan->ranges[i]);
        }
    }

    ROS_INFO("Found countPillarLidar= %i", countPillarLidar);

    //ROS_INFO("End");
}

void CPillar::poiskPillar(int a_, int b_, SLidar *lidarData)
{
    static float width = 0; // Ширина обьекта
    float dist_min = 1000;
    float angle_min = 0;

    for (int i = a_; i < b_; i++) // Перебираем точки без последней, так как она уже за столбом по дальности
    {
        if (lidarData[i].distance < dist_min) // Ищем минимальную дистанцию. Может бытьнесколько значений с одинаковым растояние и надо бы среднюю найти
        {
            dist_min = lidarData[i].distance;
            angle_min = lidarData[i].degree; // запоминаем дистанцию и угол при ней
        }
    }

    float angle_a = lidarData[a_].degree;
    float angle_b = lidarData[b_].degree;
    float angle_middle = (angle_a + angle_b) / 2.0; // Находим угол на середину
    float angle_sin = (angle_b - angle_a) / 2.0;    // Находим половину угла между крайними точками

    width = (tan(DEG2RAD(angle_sin)) * (dist_min + PILLAR_RADIUS)) * 2; // Вычисляем ширину столба По минимальному расстоянию

    // ROS_INFO("Point b= %i width = %f", b, width);         // Запоминаем вторую точку  и начинаем анализировать эту группу точек
    // ROS_INFO("angle_ab= %f max_dist= %f width = %f", angle_ab, max_dist, width); // Ширина столба

    if (width > 0.2 && width < 0.4) // Если ширина похожа на наш столб
    {
        int x = 0;
        for (int i = a_ - 1; i <= b_; i++) // Перебираем точки с захватом крайних для анализа если нужно будет
        {
            pillarLidar[countPillarLidar].data[x].degree = lidarData[i].degree;
            pillarLidar[countPillarLidar].data[x].distance = lidarData[i].distance;
            x++;
        }
        pillarLidar[countPillarLidar].angle_left = angle_b;
        pillarLidar[countPillarLidar].angle_right = angle_a;
        pillarLidar[countPillarLidar].angle_dist_min = angle_min;
        pillarLidar[countPillarLidar].angle_middle = angle_middle;                // Берем середину столба, хотя из-за вращения происходит смещения и лучше бы еще как-то уточнять
        pillarLidar[countPillarLidar].azimuth = (angle_min + angle_middle) / 2.0; // Итоговый угол на столб. Пока считаем как угол на минимум. Или можно усреднять например со средним углом. Нало посмотреть что будет точнее
        pillarLidar[countPillarLidar].dist_min = dist_min;
        pillarLidar[countPillarLidar].width = width;
        pillarLidar[countPillarLidar].x_lidarXY = sin(DEG2RAD(pillarLidar[countPillarLidar].azimuth)) * (pillarLidar[countPillarLidar].dist_min + PILLAR_RADIUS); // Находим координаты по формулам. К минимальному растоянию прибавляем радиус столба
        pillarLidar[countPillarLidar].y_lidarXY = cos(DEG2RAD(pillarLidar[countPillarLidar].azimuth)) * (pillarLidar[countPillarLidar].dist_min + PILLAR_RADIUS); // Находим координаты по формулам. К минимальному растоянию прибавляем радиус столба

        SPoint car_XY = povorotSystemCoordinate(pillarLidar[countPillarLidar].x_lidarXY, pillarLidar[countPillarLidar].y_lidarXY, -car.position.theta); // Поворачиваем систему координат/ Угол с минусом так как вращаем против часовой
        pillarLidar[countPillarLidar].x_globalXY = car_XY.x + car.position.x;                                                                       // Прибавляем смещение. Это раастояние где находится машина относительно глобальной системы координат нуля. И получаем координаты в глобальной системе координат
        pillarLidar[countPillarLidar].y_globalXY = car_XY.y + car.position.y;

        // ROS_INFO(" Pillar Angle a = %.3f Angle b= %.3f ", pillarLidar[countPillarLidar].angle_left, pillarLidar[countPillarLidar].angle_right);
        // ROS_INFO(" Angle_middle= %.3f Angle_dist_min= %.3f Angle_azimuth= %.3f ", pillarLidar[countPillarLidar].angle_middle, pillarLidar[countPillarLidar].angle_dist_min, pillarLidar[countPillarLidar].azimuth);
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
