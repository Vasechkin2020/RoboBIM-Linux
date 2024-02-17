#ifndef LIB_H
#define LIB_H
#include </opt/ros/melodic/include/ros/ros.h>
#include <ros/ros.h>

struct SPoint // Точка
{
    double x = 0; //
    double y = 0; //
};
struct SPose
{
  double x = 0;  // Координата по Х
  double y = 0;  // Координата по Y
  double th = 0; // Направление носа
};

// Преобразование координат точки из Локальной системы координат в Глобальную
// Задаем координаты точки в Локальной системе и задаем позицию самой Локальной системы в координатах Глобальной системы
// Получаем координаты точки в Глобальной системе
SPoint pointLocal2Global(SPoint pointLocal_, SPose poseLocal_)
{
    SPoint ret;
    double x = pointLocal_.x;
    double y = pointLocal_.y;
    double xnew = x * cos(poseLocal_.th) + y * sin(poseLocal_.th); // Поворачиваем по формулам поворота системы координат
    double ynew = -x * sin(poseLocal_.th) + y * cos(poseLocal_.th);
    ret.x = xnew + poseLocal_.x; // Добавляем смещение
    ret.y = ynew + poseLocal_.y;
    return ret;
}



#endif