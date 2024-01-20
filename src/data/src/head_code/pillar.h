#ifndef PILLAR_H
#define PILLAR_H

#include "config.h"

#define PILLAR_RADIUS 0.1575  // Радиус столба 315 мм
#define PEREHOD 0.15          // Разрыв между столбом и обьектом за ним
#define OTKLONENIE_PILLAR 0.3 // Отклонение межлу истинным и лидарным столююов при котором они сопостовляются
#define DELTA_MINIMUMA 0.03   // Величина отклонения на которую увеличиваем минимальную дистация до столба при отборе угла на середину, что-бы не первую попавшийся минимум брать +-3 см как в даташите пока взял для тестов

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
        float azimuth = 0;   // Дидарные значения появляются при сопоставлении
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

    int countPillarLidar = 0; // Колличество столбов отбираемых по алгоритму в массив с Лидара Не знаем точно сколько обнаружим

    // ************************** ПРИВАТНЫЕ ФУНКЦИИ КЛАССА *********************************************
    SPoint povorotSystemCoordinate(float xloc_, float yloc_, float theta_);
    void poiskPillar(int a_, int b_, SLidar *lidarData);
    SPillarLidar pillarLidar[32]; // Массив котором храним данные по столбам

public:
    SPillar pillar[16];  // Массив столбов с итоговым результатом где все столбы сопоставленны
    int countPillar = 0; //  // Колличество столбов с истинными координатами не знаем точно сколько загрузим

    // ************************** ПУБЛИЧНЫЕ ФУНКЦИИ КЛАССА *********************************************
    CPillar(/* args */);
    ~CPillar();
    void parsingLidar(const sensor_msgs::LaserScan::ConstPtr &scan);    // Функция которую вызываем из колбека по расчету места столбов
    void parsingPillar(data::topicPillar &pillar_);                     // Функция которую вызываем из колбека по расчету места столбов
    void comparisonPillar();                                            // Функция где сопоставляю столбы
    SCircle getCircle(SPoint p1_, SPoint p2_, float angle, SPoint p0_); // Функция возвращает окружность(координаты и радиус) которая ближе всего к точке старого измерения
    SPoint getCrossing(SCircle c1_, SCircle c2_, SPoint p0_);           // Функция возвращает одну из двух точек пеерсечения двух окружностей ближайшую к заданной
    SPoint getLocation();                                               // Метод возврщает положение центра лидара усредненного по всем столбам и расчетам

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
// Метод возврщает положение центра лидара усредненного по всем столбам и расчетам
SPoint CPillar::getLocation()
{
    SPoint ret;
    SPoint p0;
    p0.x = 2.0;
    p0.y = 0.0;
    SPoint p1;
    SPoint p2;
    SPoint p3;
    p1.x = pillar[3].x_true;
    p1.y = pillar[3].y_true;

    p2.x = pillar[0].x_true;
    p2.y = pillar[0].y_true;

    float a1;
    // a1 = pillar[0].azimuth - pillar[3].azimuth;
    a1 = 52.9435;

    // ROS_INFO("p1x= %.3f p1y= %.3f  / p2x= %.3f p2y= %.3f  / angle = %.3f / p0x= %.3f p0y= %.3f ", p1.x, p1.y, p2.x, p2.y, a1, p0.x, p0.y);
    SCircle c1 = getCircle(p1, p2, a1, p0);
    ROS_INFO("c1x= %.3f  c1y= %.3f  / c1r= %.3f ", c1.x, c1.y, c1.r);

    p1.x = pillar[0].x_true;
    p1.y = pillar[0].y_true;

    p2.x = pillar[1].x_true;
    p2.y = pillar[1].y_true;

    // a1 = pillar[1].azimuth - pillar[0].azimuth;
    a1 = 66.8014;

    SCircle c2 = getCircle(p1, p2, a1, p0);
    ROS_INFO("c2x= %.3f  c2y= %.3f  / c2r= %.3f ", c2.x, c2.y, c2.r);

    ret = getCrossing(c1, c2, p0);
    return ret;
}
// Функция возвращает окружность(координаты и радиус) которая ближе всего к точке старого измерения
// Основанно на идее что вписанный угол в окружность равен половине угла из центра на эту же хорду. Далее по формулам отсюда  Нахождение центра окружности по 2м точкам и углу https://dxdy.ru/topic4265.html
SCircle CPillar::getCircle(SPoint p1_, SPoint p2_, float angle_, SPoint p0_) // На вход две точки, угол между ними и координаты старого местонахождения
{
    SCircle circle;
    SPoint c1;
    SPoint c2;
    angle_ = DEG2RAD(angle_); // Переводим в радианы
    // Находим радиус
    float temp1 = (sqr(p1_.x - p2_.x) + sqr(p1_.y - p2_.y));
    float temp2 = 4 * sqr(sin(angle_));
    circle.r = sqrt(temp1 / temp2); // Находим радиус окружности
    // Таких оргужностей может быть две. Находим координнаты центра для обоих
    temp1 = (p1_.x + p2_.x) / 2.0;
    temp2 = (p1_.y - p2_.y) / 2.0;
    c1.x = temp1 + (temp2 * ctan(angle_));
    c2.x = temp1 - (temp2 * ctan(angle_));
    temp1 = (p1_.x - p2_.x) / 2.0;
    temp2 = (p1_.y + p2_.y) / 2.0;
    c1.y = temp2 - (temp1 * ctan(angle_));
    c2.y = temp2 + (temp1 * ctan(angle_));

    if (vectorLen(c1, p0_) < vectorLen(c2, p0_)) // Находим центр какой окружности ближе к последним координатам
    {
        circle.x = c1.x;
        circle.y = c1.y;
    }
    else
    {
        circle.x = c2.x;
        circle.y = c2.y;
    }

    return circle;
}
// Функция возвращает одну из двух точек пеерсечения двух окружностей ближайшую к заданной
SPoint CPillar::getCrossing(SCircle c1_, SCircle c2_, SPoint p0_) // https://algolist.manual.ru/maths/geom/intersect/circlecircle2d.php https://planetcalc.ru/8098/ https://paulbourke.net/geometry/circlesphere/
{
    SPoint ret;
    // Находим раастояние между центрами окружностей
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
        ROS_WARN("Circles do not fit the conditions !!! d= %.3f  r1= %.3f r2= %.3f", d, c1_.r, c2_.r);
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

    if (vectorLen(p4, p0_) < vectorLen(p5, p0_)) // Находим центр какой окружности ближе к последним координатам
    {
        ret = p4;
    }
    else
    {
        ret = p5;
    }

    return ret;
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
        pillar[i].status = false; // Сбрасываем старое сопоставление, остаются только истинные значения столбов. Сопоставляться должно каждый раз!!! или столб не удаствует в расчетах дальше.
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
                // ROS_INFO("Point a= %i degree= %.3f distance= %.3f ", a, lidarData[a].degree, lidarData[a].distance);
                // ROS_INFO("Point b= %i degree= %.3f distance= %.3f \n", b, lidarData[b].degree, lidarData[b].distance);
                //  ROS_INFO("=====");
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

    // ROS_INFO("End");
}

void CPillar::poiskPillar(int a_, int b_, SLidar *lidarData)
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
        pillarLidar[countPillarLidar].angle_dist_min = angle_min;          // Угол при на точку с минимальной дистанцией которую нашли первой
        pillarLidar[countPillarLidar].angle_middle_min = angle_middle_min; // Угол полученный расчетам как средний угол точек у которых дистанция до них была мешьше минимальной плюс дельта
        pillarLidar[countPillarLidar].angle_middle = angle_middle;         // Берем середину столба, хотя из-за вращения происходит смещения и лучше бы еще как-то уточнять
        // pillarLidar[countPillarLidar].azimuth = (angle_min + angle_middle) / 2.0; // Итоговый угол на столб. Пока считаем как угол на минимум. Или можно усреднять например со средним углом. Нало посмотреть что будет точнее
        pillarLidar[countPillarLidar].azimuth = angle_middle_min; // Итоговый угол на столб. Берется как средний по точкам расчетный
        pillarLidar[countPillarLidar].dist_min = dist_min;
        pillarLidar[countPillarLidar].width = width;
        pillarLidar[countPillarLidar].x_lidarXY = sin(DEG2RAD(pillarLidar[countPillarLidar].azimuth)) * (pillarLidar[countPillarLidar].dist_min + PILLAR_RADIUS); // Находим координаты по формулам. К минимальному растоянию прибавляем радиус столба
        pillarLidar[countPillarLidar].y_lidarXY = cos(DEG2RAD(pillarLidar[countPillarLidar].azimuth)) * (pillarLidar[countPillarLidar].dist_min + PILLAR_RADIUS); // Находим координаты по формулам. К минимальному растоянию прибавляем радиус столба

        SPoint car_XY = povorotSystemCoordinate(pillarLidar[countPillarLidar].x_lidarXY, pillarLidar[countPillarLidar].y_lidarXY, -car.position.theta); // Поворачиваем систему координат/ Угол с минусом так как вращаем против часовой
        pillarLidar[countPillarLidar].x_globalXY = car_XY.x + car.position.x;                                                                           // Прибавляем смещение. Это раастояние где находится машина относительно глобальной системы координат нуля. И получаем координаты в глобальной системе координат
        pillarLidar[countPillarLidar].y_globalXY = car_XY.y + car.position.y;

        // ROS_INFO(" Pillar Angle a = %.3f Angle b= %.3f ", pillarLidar[countPillarLidar].angle_left, pillarLidar[countPillarLidar].angle_right);
        // ROS_INFO(" Angle_middle= %.3f Angle_dist_min= %.3f angle_middle_min= %.3f Angle_azimuth= %.3f ", pillarLidar[countPillarLidar].angle_middle, pillarLidar[countPillarLidar].angle_dist_min, pillarLidar[countPillarLidar].angle_middle_min, pillarLidar[countPillarLidar].azimuth);
        ROS_INFO(" Angle_middle= %.3f Angle_dist_min= %.3f angle_middle_min= %.3f ", pillarLidar[countPillarLidar].angle_middle, pillarLidar[countPillarLidar].angle_dist_min, pillarLidar[countPillarLidar].angle_middle_min);
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
