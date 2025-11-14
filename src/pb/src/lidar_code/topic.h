#ifndef TOPIC_H
#define TOPIC_H

/*
 Класс для функций для формирования топиков в нужном виде и формате и всех публикаций
*/
#include "config.h"
#include "pillarDetector.h"
#include "pillar.h"

extern SPoseLidar g_poseLidar; // Позиции лидара по расчетам Центральная система координат

class CTopic
{
public:
    CTopic(/* args */) // Реализация конструктора
    {
        color1.r = 1.0;
        color1.g = 0.0;
        color1.b = 0.0;
        color1.a = 1.0; // Красный
        color2.r = 0.0;
        color2.g = 1.0;
        color2.b = 0.0;
        color2.a = 1.0; // Зеленый
        color3.r = 0.0;
        color3.g = 0.0;
        color3.b = 1.0;
        color3.a = 1.0; // Синий
    }
    ~CTopic() // Реализация деструктора
    {
    }

    void visualStartPose(SPose startPose2d)
    {
        geometry_msgs::PoseStamped startPose_msg;      // Начальная позиция отображаем в RVIZ
        startPose_msg.header.stamp = ros::Time::now(); // Время ROS
        startPose_msg.header.frame_id = "laser";
        startPose_msg.pose.position.x = startPose2d.x;
        startPose_msg.pose.position.y = startPose2d.y;
        float theta = DEG2RAD(startPose2d.th); //
        geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(theta);
        startPose_msg.pose.orientation = quat;
        // ROS_INFO("startPose_msg Quaternion x =%.3f y =%.3f z =%.3f w =%.3f theta = %.3f", quat.x, quat.y, quat.z, quat.w, theta);

        pub_StartPose.publish(startPose_msg); // Публикация полученных данных

        visualization_msgs::Marker marker;
        marker.header.frame_id = "laser";
        marker.header.stamp = ros::Time::now(); // Время ROS;
        marker.ns = "position";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = startPose2d.x;
        marker.pose.position.y = startPose2d.y;
        marker.pose.position.z = 0.1;
        marker.pose.orientation = tf::createQuaternionMsgFromYaw(0);
        marker.scale.x = 0.05;
        marker.scale.y = 0.05;
        marker.scale.z = 0.2;
        marker.color.a = 0.5; // Don't forget to set the alpha!
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 0.0;

        pub_markerPosition.publish(marker);

        marker.id = 1;
        marker.pose.position.x = 3.0;
        marker.pose.position.y = 3.0;
        pub_markerPosition.publish(marker);
    }
    void visualPillarPoint(CPillar pillar_) // Готовим одиночные точки столбов для RVIZ  в системе координат "odom"
    {
        visualization_msgs::Marker marker;
        marker.header.frame_id = "base";
        marker.header.stamp = ros::Time::now(); // Время ROS;
        marker.ns = "pillars";
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation = tf::createQuaternionMsgFromYaw(0);
        marker.scale.x = 0.315;
        marker.scale.y = 0.315;
        marker.scale.z = 0.5;

        marker.color.a = 0.7; // Don't forget to set the alpha!
        marker.color.r = 0.8;
        marker.color.g = 0.2;
        marker.color.b = 0.8;

        SPoint point_local, point_global;

        for (size_t i = 0; i < 4; i++)
        {
            point_global.x = pillar_.pillar[i].x_true;
            point_global.y = pillar_.pillar[i].y_true;
            point_local = pointGlobal2Local(point_global, g_poseLidar.modeFused); // Вариант поворота по стандарту РОС что вращение против часовой
            // point_local = pointGlobal2Local(point_global, g_transformGlobal2Local); // Вариант поворота по стандарту РОС что вращение против часовой
            // ROS_INFO("Global x = %.2f y = %.2f | Local x = %.2f y = %.2f | Pose x = %.2f y = %.2f th grad = %.2f th rad = %.2f",
            //     point_global.x,point_global.y,point_local.x,point_local.y,g_transformGlobal2Local.x, g_transformGlobal2Local.y, g_transformGlobal2Local.th, DEG2RAD(g_transformGlobal2Local.th));
            marker.pose.position.x = point_local.x;
            marker.pose.position.y = point_local.y;
            marker.pose.position.z = 0.0;
            marker.id = i;
            pub_markerPillar.publish(marker);
        }
    }
    /*
        void publicationPillarAll(CPillar pillar_) // Формируем перемнную с собщением для публикации
        {
            // ROS_INFO("+++ %i",pillar.countPillar);
            pb_msgs::PillarOut pillarAll_msg; // Перемеенная в которую пишем данные для опубликования в топик
            pb_msgs::pillar data;
            pillarAll_msg.data.clear(); // Очищаем старые точки из массива
            for (int i = 0; i < pillar_.countPillar; i++)
            {
                data.status = pillar_.pillar[i].status;
                data.azimuth = pillar_.pillar[i].azimuth;
                data.hypotenuse = pillar_.pillar[i].hypotenuse;
                data.distance_lidar = pillar_.pillar[i].distance_lidar;
                data.x_true = pillar_.pillar[i].x_true;
                data.y_true = pillar_.pillar[i].y_true;
                data.theta_true1 = pillar_.pillar[i].theta_true1;
                data.theta_true2 = pillar_.pillar[i].theta_true2;
                data.x_lidar = pillar_.pillar[i].x_lidar;
                data.y_lidar = pillar_.pillar[i].y_lidar;
                // ROS_INFO("Status= %i azimuth= %.3f",pillar_out_msg.data[i].status,pillar_out_msg.data[i].azimuth);
                pillarAll_msg.data.push_back(data);
            }
            pub_PillarAll.publish(pillarAll_msg); // Публикуем информацию по столбам

            // pointPillar0_msg.header.stamp = ros::Time::now();
            // pointPillar0_msg.header.frame_id = "odom";
            // pointPillar0_msg.point.x = pillar_.pillar[0].y_lidar;
            // pointPillar0_msg.point.y = pillar_.pillar[0].x_lidar;

            // pointPillarA_msg.header.stamp = ros::Time::now();
            // pointPillarA_msg.header.frame_id = "odom";
            // pointPillarA_msg.points.clear(); // Очищаем старые точки из массива
            // geometry_msgs::Point32 p[4];
            // for (int i = 0; i < pillar_.countPillar; i++)
            // {
            //     p[i].x = pillar_.pillar[0].x_lidar + 1;
            //     p[i].y = pillar_.pillar[0].y_lidar + 1;
            //     pointPillarA_msg.points.push_back(p[i]);
            // }
        }
    */
    void publicationPoseLidar() // Формируем перемнную с собщением для публикации
    {
        pb_msgs::Struct_PoseLidar msg; // Обобщенные данные в моем формате о всех вариантах расчета позиции

        msg.modeDist.x = g_poseLidar.modeDist.x;
        msg.modeDist.y = g_poseLidar.modeDist.y;
        msg.modeDist.th = g_poseLidar.modeDist.th;

        msg.modeAngle.x = g_poseLidar.modeAngle.x;
        msg.modeAngle.y = g_poseLidar.modeAngle.y;
        msg.modeAngle.th = g_poseLidar.modeAngle.th;

        msg.modeFused.x = g_poseLidar.modeFused.x;
        msg.modeFused.y = g_poseLidar.modeFused.y;
        msg.modeFused.th = g_poseLidar.modeFused.th;

        msg.modeClaster.x = g_poseLidar.modeClaster.x;
        msg.modeClaster.y = g_poseLidar.modeClaster.y;
        msg.modeClaster.th = g_poseLidar.modeClaster.th;

        msg.mnkDist.x = g_poseLidar.mnkDist.x;
        msg.mnkDist.y = g_poseLidar.mnkDist.y;
        msg.mnkDist.th = g_poseLidar.mnkDist.th;
        msg.quality_mknDist = g_poseLidar.quality_mknDist;

        msg.mnkAngle.x = g_poseLidar.mnkAngle.x;
        msg.mnkAngle.y = g_poseLidar.mnkAngle.y;
        msg.mnkAngle.th = g_poseLidar.mnkAngle.th;
        msg.quality_mknAngle = g_poseLidar.quality_mknAngle;

        msg.mnkFused.x = g_poseLidar.mnkFused.x;
        msg.mnkFused.y = g_poseLidar.mnkFused.y;
        msg.mnkFused.th = g_poseLidar.mnkFused.th;
        msg.quality_mknFused = g_poseLidar.quality_mknFused;

        msg.countMatchPillar = g_poseLidar.countMatchPillar;
        msg.countCrossCircle = g_poseLidar.countCrossCircle;

        for (size_t i = 0; i < 4; i++)
        {
            float temp = -g_poseLidar.azimut[i]; // Переделываем под стандарт ROS.
            if (temp < -180)
                temp = 360 + temp;
            msg.azimut[i] = temp;
        }
        
        // msg.azimut[1] = g_poseLidar.azimut[1];
        // msg.azimut[2] = g_poseLidar.azimut[2];
        // msg.azimut[3] = g_poseLidar.azimut[3];

        pub_poseLidar.publish(msg); // Публикуем информацию по позиции лидара
    }
    // Отобращение стрелкой где начало и куда смотрит в Mode0 1 2
    void visualPublishOdomMode_1()
    {
        // Публикация Одометрии
        nav_msgs::Odometry modeDist_msg;
        modeDist_msg.header.stamp = ros::Time::now(); // Время ROS
        modeDist_msg.header.frame_id = "laser";       // Поза в этом сообщении должна быть указана в системе координат, заданной header.frame_id.
        // set the position
        modeDist_msg.pose.pose.position.x = g_poseLidar.modeDist.x;
        modeDist_msg.pose.pose.position.y = g_poseLidar.modeDist.y;
        // geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(DEG2RAD(-g_poseLidar.modeDist.th + 90)); // Минус так как вращение у меня по часовой а не по "буравчику" и + 90 так как считал я что 0 градусов это по оси Y Глядел на стену надо переписывать
        geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(DEG2RAD(g_poseLidar.modeDist.th)); // Минус так как вращение у меня по часовой а не по "буравчику" и + 90 так как считал я что 0 градусов это по оси Y Глядел на стену надо переписывать
        modeDist_msg.pose.pose.orientation = quat;
        // set the velocity
        modeDist_msg.child_frame_id = "laser"; // Поворот в этом сообщении должен быть указан в системе координат, заданной child_frame_id
        modeDist_msg.twist.twist.linear.x = 0;
        modeDist_msg.twist.twist.linear.y = 0;
        modeDist_msg.twist.twist.angular.z = 0;
        publish_modeDist.publish(modeDist_msg); // Публикация полученных данных

        // geometry_msgs::PoseStamped poseLidarmodeDist_msg; // Перемеенная в которую сохраняем данные лидара из сообщения
        // ros_time = ros::Time::now(); // Время ROS
        // poseLidarmodeDist_msg.header.stamp = ros_time;
        // poseLidarmodeDist_msg.header.frame_id = "odom";
        // poseLidarmodeDist_msg.pose.position.x = g_poseLidar.modeDist.x;
        // poseLidarmodeDist_msg.pose.position.y = g_poseLidar.modeDist.y;
        // poseLidarmodeDist_msg.pose.orientation = tf::createQuaternionMsgFromYaw(DEG2RAD(-g_poseLidar.modeDist.th + 90)); // + 90 Так как у них оси расположены не так как я меня. У меня ноль вверх а у них вправо и вращение у них против часовой
        // pub_PoseLidarmodeDist.publish(poseLidarmodeDist_msg);                                                            // Публикуем информацию по позиции лидара modeDist
    }
    // Отобращение стрелкой где начало и куда смотрит в Mode0 1 2
    void visualPublishOdomMode_2()
    {
        // Публикация Одометрии
        nav_msgs::Odometry modeAngle_msg;
        modeAngle_msg.header.stamp = ros::Time::now(); // Время ROS
        modeAngle_msg.header.frame_id = "laser";       // Поза в этом сообщении должна быть указана в системе координат, заданной header.frame_id.
        // set the position
        modeAngle_msg.pose.pose.position.x = g_poseLidar.modeAngle.x;
        modeAngle_msg.pose.pose.position.y = g_poseLidar.modeAngle.y;
        // geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(DEG2RAD(-g_poseLidar.modeAngle.th + 90)); // Минус так как вращение у меня по часовой а не по "буравчику" и + 90 так как считал я что 0 градусов это по оси Y Глядел на стену надо переписывать
        geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(DEG2RAD(g_poseLidar.modeAngle.th)); // Минус так как вращение у меня по часовой а не по "буравчику" и + 90 так как считал я что 0 градусов это по оси Y Глядел на стену надо переписывать
        modeAngle_msg.pose.pose.orientation = quat;
        // set the velocity
        modeAngle_msg.child_frame_id = "laser"; // Поворот в этом сообщении должен быть указан в системе координат, заданной child_frame_id
        modeAngle_msg.twist.twist.linear.x = 0;
        modeAngle_msg.twist.twist.linear.y = 0;
        modeAngle_msg.twist.twist.angular.z = 0;
        publish_modeAngle.publish(modeAngle_msg); // Публикация полученных данных

        // geometry_msgs::PoseStamped poseLidarmodeAngle_msg; // Перемеенная в которую сохраняем данные лидара из сообщения
        // ros_time = ros::Time::now(); // Время ROS
        // poseLidarmodeAngle_msg.header.stamp = ros_time;
        // poseLidarmodeAngle_msg.header.frame_id = "odom";
        // poseLidarmodeAngle_msg.pose.position.x = g_poseLidar.modeAngle.x;
        // poseLidarmodeAngle_msg.pose.position.y = g_poseLidar.modeAngle.y;
        // poseLidarmodeAngle_msg.pose.orientation = tf::createQuaternionMsgFromYaw(DEG2RAD(-g_poseLidar.modeAngle.th + 90)); // + 90 Так как у них оси расположены не так как я меня. У меня ноль вверх а у них вправо и вращение у них против часовой
        // pub_PoseLidarmodeAngle.publish(poseLidarmodeAngle_msg);                                                            // Публикуем информацию по позиции лидара modeAngle
    }

    // Функция для визуализации кластеров, столбов и лидара в RViz (вызывается раз в секунду)
    void visualizeClasters(std::vector<PillarDetector::ClusterInfo> cluster_info_list) // Список информации о кластерах)
    {
        // Создаём маркер для кластеров
        visualization_msgs::Marker cluster_marker;
        cluster_marker.header.frame_id = "laser";                 // Система координат лидара
        cluster_marker.header.stamp = ros::Time::now();           // Текущая метка времени
        cluster_marker.ns = "clusters";                           // Пространство имён для кластеров
        cluster_marker.type = visualization_msgs::Marker::POINTS; // Тип маркера - точки
        cluster_marker.action = visualization_msgs::Marker::ADD;  // Действие - добавить
        cluster_marker.pose.orientation.w = 1.0;                  // Ориентация (без вращения)
        cluster_marker.scale.x = 0.03;                            // Размер точки по x (5 см)
        cluster_marker.scale.y = 0.03;                            // Размер точки по y
        cluster_marker.id = 0;                                    // Идентификатор маркера
        int aa = 1;

        // Заполняем маркер кластерами (локальные координаты для RViz)
        for (int i = 0; i < cluster_info_list.size(); i++)
        {
            if (aa == 1)
            {
                color = color2;
                aa = 0;
            }
            else
            {
                color = color3;
                aa = 1;
            }
            for (int j = 0; j < cluster_info_list[i].points.size(); j++)
            {
                geometry_msgs::Point point;
                point.x = cluster_info_list[i].points[j].x; // Локальная координата x
                point.y = cluster_info_list[i].points[j].y; // Локальная координата y
                point.z = 0.0;                              // Высота (z=0, так как 2D)
                cluster_marker.points.push_back(point);     // Добавляем точку в список
                cluster_marker.colors.push_back(color);     // Добавляем цвет в список
            }
        }
        cluster_publisher.publish(cluster_marker);
        // Выводим информацию о публикации
        ROS_INFO("    RVIZ pub %d clusters", (int)cluster_info_list.size());
    }
    // Список найденных столбов
    void visualizePillars(std::vector<PillarDetector::Pillar> pillars)
    {
        
        visualization_msgs::MarkerArray marker_array;// Создаем массив маркеров для цилиндров


        // Заполняем маркер столбами (локальные координаты для RViz)
        for (int i = 0; i < pillars.size(); i++)
        {
            visualization_msgs::Marker pillar_marker;                  // Создаём маркер для столбов
            pillar_marker.header.frame_id = "laser";                   // Система координат лидара
            pillar_marker.header.stamp = ros::Time::now();             // Текущая метка времени
            pillar_marker.ns = "pillars";                              // Пространство имён
            pillar_marker.type = visualization_msgs::Marker::CYLINDER; // Тип маркера - список сфер
            pillar_marker.action = visualization_msgs::Marker::ADD;    // Действие - добавить
            pillar_marker.scale.x = 0.315;                             // Размер сферы по x (диаметр столба)
            pillar_marker.scale.y = 0.315;                             // Размер сферы по y
            pillar_marker.scale.z = 0.5;                               // Размер сферы по z
            pillar_marker.id = i;                                      // Идентификатор маркера
            pillar_marker.color = color2;
            pillar_marker.lifetime = ros::Duration(0.5); // Маркер исчезнет через 0,5 секунд

            pillar_marker.pose.position.x = pillars[i].x_center; // Локальная координата x центра столба
            pillar_marker.pose.position.y = pillars[i].y_center; // Локальная координата y центра столба
            pillar_marker.pose.position.z = 0.0;                 // Высота (z=0, так как 2D)
            pillar_marker.pose.orientation.w = 1.0;              // Ориентация (без вращения)

            if (pillars[i].match == true)
            {  
                //marker_publisher.publish(pillar_marker); // Отправляем маркеры в RViz
                marker_array.markers.push_back(pillar_marker);// Добавляем маркер в массив
            }
        }
        
        marker_publisher.publish(marker_array); // Публикуем массив маркеров

        // ROS_INFO("    RVIZ pub with %d points %d pillars,", (int)pillar_marker.points.size(), (int)pillars.size());// Выводим информацию о публикации
        ROS_INFO("    RVIZ pub %d pillars,", (int)pillars.size()); // Выводим информацию о публикации
    }

    void visualizeLidar()
    {
        // Создаём маркер для лидара с направлением в глобальной системе координат
        visualization_msgs::Marker lidar_marker;
        lidar_marker.header.frame_id = "laser";                 // Используем систему laser для простоты
        lidar_marker.header.stamp = ros::Time::now();           // Текущая метка времени
        lidar_marker.ns = "lidar_sphere";                       // Пространство имён
        lidar_marker.type = visualization_msgs::Marker::SPHERE; // Тип маркера - сфера
        lidar_marker.action = visualization_msgs::Marker::ADD;  // Добавление маркера

        // Параметры масштабирования (размер сферы)
        lidar_marker.scale.x = 0.2; // Диаметр сферы по оси X
        lidar_marker.scale.y = 0.2; // Диаметр сферы по оси Y
        lidar_marker.scale.z = 0.1; // Диаметр сферы по оси Z

        // Цвет сферы (RGBA, значения от 0 до 1)
        lidar_marker.color.r = 0.5; // Красный
        lidar_marker.color.g = 0.6; // Зеленый
        lidar_marker.color.b = 0.7; // Синий
        lidar_marker.color.a = 0.5; // Прозрачность (1.0 = полностью непрозрачный)

        // Позиция сферы
        lidar_marker.pose.position.x = 0.0;
        lidar_marker.pose.position.y = 0.0;
        lidar_marker.pose.position.z = 0.0;

        // Ориентация сферы (обычно не требуется для сферы)
        lidar_marker.pose.orientation.x = 0.0;
        lidar_marker.pose.orientation.y = 0.0;
        lidar_marker.pose.orientation.z = 0.0;
        lidar_marker.pose.orientation.w = 1.0;

        // Отправляем маркеры в RViz
        lidar_publisher.publish(lidar_marker);

        // Выводим информацию о публикации
        ROS_INFO("    RVIZ pub lidar position with orientation");
    }

    void transformLidar() // Публикуем системы трансормаций из одних систем координат в другие
    {
        // --------------------------------- base laser Для ЛИДАРА---------------------------------------
        geometry_msgs::TransformStamped tfOdomLaser;
        tfOdomLaser.header.stamp = ros::Time::now();
        tfOdomLaser.header.frame_id = "base";
        tfOdomLaser.child_frame_id = "laser";
        tfOdomLaser.transform.translation.x = 0;
        tfOdomLaser.transform.translation.y = 0;
        tfOdomLaser.transform.translation.z = 0.0;
        tfOdomLaser.transform.rotation = tf::createQuaternionMsgFromYaw(DEG2RAD(-180)); // Из градусов в радианы добавляем минус, так как в РОС вращение плюс против часовой, а у меня по часовой
        tfBroadcaster.sendTransform(tfOdomLaser);                                       // Публикация системы преобразования из odom в map Тут динамически, а статически выглядит так   <node pkg="tf" type="static_transform_publisher" name="static_map_odom_tf" args="0 0 0 0 0 0 map odom 100" /> <!--http://wiki.ros.org/tf-->
    }

    // Отобращение стрелкой напрвлениена сопоставленные столбы из центра где лидар расположен
    void visualPoseAngleLaser()
    {
        visualization_msgs::MarkerArray marker_array; // Создаем MarkerArray
        for (int i = 0; i < 4; ++i)
        {
            visualization_msgs::Marker arrow_marker; // Создаем маркер типа ARROW
            arrow_marker.header.frame_id = "base";   // Система координат
            arrow_marker.header.stamp = ros::Time::now();
            arrow_marker.ns = "arrows";
            arrow_marker.id = i;                                   // Уникальный ID для каждой стрелки
            arrow_marker.type = visualization_msgs::Marker::ARROW; // Тип маркера - стрелка
            arrow_marker.action = visualization_msgs::Marker::ADD;

            // Настройка масштаба (длина и толщина стрелки)
            arrow_marker.scale.x = 5.0; // Длина стрелки
            arrow_marker.scale.y = 0.02;
            arrow_marker.scale.z = 0.02; // Толщина стрелки

            arrow_marker.color = color3; // Настройка цвета

            // Устанавливаем позицию и ориентацию стрелки
            arrow_marker.pose.position.x = 0.0;                                                              // Расположение по X
            arrow_marker.pose.position.y = 0.0;                                                              // Расположение по Y
            arrow_marker.pose.position.z = 0.0;                                                              // Расположение по Z
            arrow_marker.pose.orientation = tf::createQuaternionMsgFromYaw(-DEG2RAD(g_poseLidar.azimut[i])); // Минус так как у меня вращение по часовой

            marker_array.markers.push_back(arrow_marker); // Добавляем маркер в массив
        }
        pub_pointers.publish(marker_array);
    }

private:
    ros::NodeHandle _nh;
    tf::TransformBroadcaster tfBroadcaster;            // Вещание данных преобразования систем координат
    ros::Time ros_time;                                // Время ROS
    std_msgs::ColorRGBA color, color1, color2, color3; // Добавляем цвета для каждой точки

    //--------------------------------- ПУБЛИКАЦИЯ В ТОПИКИ -------------------------------------------------

    ros::Publisher pub_poseLidar = _nh.advertise<pb_msgs::Struct_PoseLidar>("pb/Lidar/Pose", 8); // Это мы публикуем итоговую информацию по позици лидара обобщенную
    // ros::Publisher pub_PillarAll = _nh.advertise<pb_msgs::PillarOut>("pbPos/PillarAll", 16); // Это мы публикуем итоговую обобщенную информацию по столбам где все данные указаны НАФИГА?

    ros::Publisher pub_markerPillar = _nh.advertise<visualization_msgs::Marker>("pb/rviz/Lidar/pillarStatic", 0);     // Публикуем столбы как маркер тип цилиндр
    ros::Publisher pub_markerPosition = _nh.advertise<visualization_msgs::Marker>("pb/rviz/Lidar/markerPosition", 0); // Публикуем столики как точки позиций
    ros::Publisher pub_StartPose = _nh.advertise<geometry_msgs::PoseStamped>("pb/rviz/Lidar/StartPose", 16);          // Для публикации стартовой позиции

    ros::Publisher publish_modeDist = _nh.advertise<nav_msgs::Odometry>("pb/rviz//Lidar//modeDist", 8); // Это мы создаем публикатор и определяем название топика в рос
    ros::Publisher publish_modeAngle = _nh.advertise<nav_msgs::Odometry>("pb/rviz/Lidar/modeAngle", 8); // Это мы создаем публикатор и определяем название топика в рос

    // ros::Publisher marker_publisher = _nh.advertise<visualization_msgs::Marker>("pb/rviz/Lidar/pillar_markers", 1);   // Создаём publisher для отправки маркеров столбов в RViz Это одиночные а не в масиве
    ros::Publisher marker_publisher = _nh.advertise<visualization_msgs::MarkerArray>("pb/rviz/Lidar/pillarDinamic", 3);// Публикуем массив маркеров
    ros::Publisher cluster_publisher = _nh.advertise<visualization_msgs::Marker>("pb/rviz/Lidar/cluster_markers", 1); // Создаём publisher для отправки маркеров кластеров в RViz
    ros::Publisher lidar_publisher = _nh.advertise<visualization_msgs::Marker>("pb/rviz/Lidar/lidar_marker", 1);      // Создаём publisher для отправки маркера лидара в RViz

    // Создаем публикатор для MarkerArray// СТРЕЛКИ на столбы
    ros::Publisher pub_pointers = _nh.advertise<visualization_msgs::MarkerArray>("pb/rviz/Lidar/pointers", 10);

    // ros::Publisher pub_PoseLidarmodeDist = _nh.advertise<geometry_msgs::PoseStamped>("pbRviz/PoseLidarmodeDist", 16); // Это мы публикуем итоговую информацию по позици лидара расчет по modeDist
    // ros::Publisher pub_PoseLidarmodeAngle = _nh.advertise<geometry_msgs::PoseStamped>("pbRviz/PoseLidarmodeAngle", 16); // Это мы публикуем итоговую информацию по позици лидара расчет по modeAngle
};

#endif