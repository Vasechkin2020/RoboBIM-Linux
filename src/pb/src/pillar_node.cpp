#include <ros/ros.h>                   // Библиотека ROS для работы с узлами
#include <sensor_msgs/LaserScan.h>     // Сообщения для данных лидара
#include <visualization_msgs/Marker.h> // Сообщения для визуализации в RViz
#include <vector>                      // Стандартный вектор C++
#include <cmath>                       // Математические функции (sin, cos, sqrt)
#include <signal.h>                    // Для обработки Ctrl+C

#include "genStruct.h" // Тут все общие структуры. Истользуются и Data и Main и Head
#include "pillar_code/pillarDetector.h"

// Переменная для остановки программы по Ctrl+C
int keep_running = 1;

// Функция, которая срабатывает при нажатии Ctrl+C
static void stopProgram(int signal)
{
    keep_running = 0; // Устанавливаем флаг, чтобы остановить цикл
    ros::shutdown();  // Завершаем работу ROS
}

// Главная функция программы
int main(int argc, char **argv)
{
    // Настраиваем обработку Ctrl+C
    signal(SIGINT, stopProgram);

    // Инициализируем ROS с именем узла "pillar_node"
    ros::init(argc, argv, "pillar_node");
    // Создаём объект детектора столбов
    PillarDetector detector;

    // Создаём цикл с частотой 10 Гц
    ros::Rate loop_rate(2);
    // Пока ROS работает и не нажат Ctrl+C
    while (ros::ok() && keep_running)
    {
        ros::spinOnce();   // Обрабатываем входящие сообщения
        loop_rate.sleep(); // Ждём, чтобы поддерживать частоту 10 Гц
    }

    ROS_INFO("Program stopped");
    return 0;
}