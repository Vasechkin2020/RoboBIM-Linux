#ifndef TOPIC_H
#define TOPIC_H

/*
 Класс для функций для формирования топиков в нужном виде и формате и всех публикаций
*/
#include "config.h"
#include "pillar.h"

extern SPoseLidar g_poseLidar; // Позиции лидара по расчетам Центральная система координат

class CTopic
{
public:
    CTopic(/* args */)// Реализация конструктора
    {}
    ~CTopic() // Реализация деструктора
    {}

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
        marker.header.frame_id = "laser";
        marker.header.stamp = ros::Time::now(); // Время ROS;
        marker.ns = "pillars";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::CYLINDER;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.position.x = pillar_.pillar[0].x_true;
        marker.pose.position.y = pillar_.pillar[0].y_true;
        marker.pose.position.z = 0.25;
        marker.pose.orientation = tf::createQuaternionMsgFromYaw(0);
        marker.scale.x = 0.31;
        marker.scale.y = 0.31;
        marker.scale.z = 0.5;
        marker.color.a = 0.7; // Don't forget to set the alpha!
        marker.color.r = 0.8;
        marker.color.g = 0.2;
        marker.color.b = 0.8;

        pub_markerPillar.publish(marker);

        marker.id = 1;
        marker.pose.position.x = pillar_.pillar[1].x_true;
        marker.pose.position.y = pillar_.pillar[1].y_true;
        pub_markerPillar.publish(marker);
        marker.id = 2;
        marker.pose.position.x = pillar_.pillar[2].x_true;
        marker.pose.position.y = pillar_.pillar[2].y_true;
        pub_markerPillar.publish(marker);
        marker.id = 3;
        marker.pose.position.x = pillar_.pillar[3].x_true;
        marker.pose.position.y = pillar_.pillar[3].y_true;
        pub_markerPillar.publish(marker);
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
        pb_msgs::Struct_PoseLidar poseLidarAll_msg; // Обобщенные данные в моем формате о всех вариантах расчета позиции

        poseLidarAll_msg.mode.x = g_poseLidar.mode.x;
        poseLidarAll_msg.mode.y = g_poseLidar.mode.y;
        poseLidarAll_msg.mode.th = g_poseLidar.mode.th;

        poseLidarAll_msg.mode1.x = g_poseLidar.mode1.x;
        poseLidarAll_msg.mode1.y = g_poseLidar.mode1.y;
        poseLidarAll_msg.mode1.th = g_poseLidar.mode1.th;

        poseLidarAll_msg.mode2.x = g_poseLidar.mode2.x;
        poseLidarAll_msg.mode2.y = g_poseLidar.mode2.y;
        poseLidarAll_msg.mode2.th = g_poseLidar.mode2.th;

        poseLidarAll_msg.mode3.x = g_poseLidar.mode3.x;
        poseLidarAll_msg.mode3.y = g_poseLidar.mode3.y;
        poseLidarAll_msg.mode3.th = g_poseLidar.mode3.th;

        pub_poseLidar.publish(poseLidarAll_msg); // Публикуем информацию по позиции лидара
    }
    // Отобращение стрелкой где начало и куда смотрит в Mode0 1 2
    void visualPublishOdomMode_1()
    {
        // Публикация Одометрии
        nav_msgs::Odometry mode1_msg;
        mode1_msg.header.stamp = ros::Time::now(); // Время ROS
        mode1_msg.header.frame_id = "laser";        // Поза в этом сообщении должна быть указана в системе координат, заданной header.frame_id.
        // set the position
        mode1_msg.pose.pose.position.x = g_poseLidar.mode1.x;
        mode1_msg.pose.pose.position.y = g_poseLidar.mode1.y;
        // geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(DEG2RAD(-g_poseLidar.mode1.th + 90)); // Минус так как вращение у меня по часовой а не по "буравчику" и + 90 так как считал я что 0 градусов это по оси Y Глядел на стену надо переписывать
        geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(DEG2RAD(g_poseLidar.mode1.th)); // Минус так как вращение у меня по часовой а не по "буравчику" и + 90 так как считал я что 0 градусов это по оси Y Глядел на стену надо переписывать
        mode1_msg.pose.pose.orientation = quat;
        // set the velocity
        mode1_msg.child_frame_id = "laser"; // Поворот в этом сообщении должен быть указан в системе координат, заданной child_frame_id
        mode1_msg.twist.twist.linear.x = 0;
        mode1_msg.twist.twist.linear.y = 0;
        mode1_msg.twist.twist.angular.z = 0;
        publish_Mode1.publish(mode1_msg); // Публикация полученных данных

        // geometry_msgs::PoseStamped poseLidarMode1_msg; // Перемеенная в которую сохраняем данные лидара из сообщения
        // ros_time = ros::Time::now(); // Время ROS
        // poseLidarMode1_msg.header.stamp = ros_time;
        // poseLidarMode1_msg.header.frame_id = "odom";
        // poseLidarMode1_msg.pose.position.x = g_poseLidar.mode1.x;
        // poseLidarMode1_msg.pose.position.y = g_poseLidar.mode1.y;
        // poseLidarMode1_msg.pose.orientation = tf::createQuaternionMsgFromYaw(DEG2RAD(-g_poseLidar.mode1.th + 90)); // + 90 Так как у них оси расположены не так как я меня. У меня ноль вверх а у них вправо и вращение у них против часовой
        // pub_PoseLidarMode1.publish(poseLidarMode1_msg);                                                            // Публикуем информацию по позиции лидара mode1
    }
    // Отобращение стрелкой где начало и куда смотрит в Mode0 1 2
    void visualPublishOdomMode_2()
    {
        // Публикация Одометрии
        nav_msgs::Odometry mode2_msg;
        mode2_msg.header.stamp = ros::Time::now(); // Время ROS
        mode2_msg.header.frame_id = "laser";        // Поза в этом сообщении должна быть указана в системе координат, заданной header.frame_id.
        // set the position
        mode2_msg.pose.pose.position.x = g_poseLidar.mode2.x;
        mode2_msg.pose.pose.position.y = g_poseLidar.mode2.y;
        // geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(DEG2RAD(-g_poseLidar.mode2.th + 90)); // Минус так как вращение у меня по часовой а не по "буравчику" и + 90 так как считал я что 0 градусов это по оси Y Глядел на стену надо переписывать
        geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(DEG2RAD(g_poseLidar.mode2.th)); // Минус так как вращение у меня по часовой а не по "буравчику" и + 90 так как считал я что 0 градусов это по оси Y Глядел на стену надо переписывать
        mode2_msg.pose.pose.orientation = quat;
        // set the velocity
        mode2_msg.child_frame_id = "laser"; // Поворот в этом сообщении должен быть указан в системе координат, заданной child_frame_id
        mode2_msg.twist.twist.linear.x = 0;
        mode2_msg.twist.twist.linear.y = 0;
        mode2_msg.twist.twist.angular.z = 0;
        publish_Mode2.publish(mode2_msg); // Публикация полученных данных

        // geometry_msgs::PoseStamped poseLidarMode2_msg; // Перемеенная в которую сохраняем данные лидара из сообщения
        // ros_time = ros::Time::now(); // Время ROS
        // poseLidarMode2_msg.header.stamp = ros_time;
        // poseLidarMode2_msg.header.frame_id = "odom";
        // poseLidarMode2_msg.pose.position.x = g_poseLidar.mode2.x;
        // poseLidarMode2_msg.pose.position.y = g_poseLidar.mode2.y;
        // poseLidarMode2_msg.pose.orientation = tf::createQuaternionMsgFromYaw(DEG2RAD(-g_poseLidar.mode2.th + 90)); // + 90 Так как у них оси расположены не так как я меня. У меня ноль вверх а у них вправо и вращение у них против часовой
        // pub_PoseLidarMode2.publish(poseLidarMode2_msg);                                                            // Публикуем информацию по позиции лидара mode2
    }
    // Отобращение стрелкой где начало и куда смотрит в Mode3
    void visualPublishOdomMode_3()
    {
        // Публикация Одометрии
        nav_msgs::Odometry mode_msg;
        mode_msg.header.stamp = ros::Time::now(); // Время ROS
        mode_msg.header.frame_id = "laser";        // Поза в этом сообщении должна быть указана в системе координат, заданной header.frame_id.
        // set the position
        mode_msg.pose.pose.position.x = g_poseLidar.mode3.x;
        mode_msg.pose.pose.position.y = g_poseLidar.mode3.y;
        geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(DEG2RAD(g_poseLidar.mode3.th)); // Минус так как вращение у меня по часовой а не по "буравчику" и + 90 так как считал я что 0 градусов это по оси Y Глядел на стену надо переписывать
        mode_msg.pose.pose.orientation = quat;
        // set the velocity
        mode_msg.child_frame_id = "laser"; // Поворот в этом сообщении должен быть указан в системе координат, заданной child_frame_id
        mode_msg.twist.twist.linear.x = 0;
        mode_msg.twist.twist.linear.y = 0;
        mode_msg.twist.twist.angular.z = 0;
        publish_Mode3.publish(mode_msg); // Публикация полученных данных
    }

private:
    ros::NodeHandle _nh;
    ros::Time ros_time;                     // Время ROS

    //--------------------------------- ПУБЛИКАЦИЯ В ТОПИКИ -------------------------------------------------

    ros::Publisher pub_poseLidar = _nh.advertise<pb_msgs::Struct_PoseLidar>("pbLidar/PoseLidar", 8);          // Это мы публикуем итоговую информацию по позици лидара обобщенную
    // ros::Publisher pub_PillarAll = _nh.advertise<pb_msgs::PillarOut>("pbPos/PillarAll", 16); // Это мы публикуем итоговую обобщенную информацию по столбам где все данные указаны НАФИГА?
    ros::Publisher pub_markerPillar = _nh.advertise<visualization_msgs::Marker>("pbRviz/markerPillar", 0);    // Публикуем столбы как маркер тип цилиндр
    ros::Publisher pub_markerPosition = _nh.advertise<visualization_msgs::Marker>("pbRviz/markerPosition", 0); // Публикуем столики как точки позиций
    ros::Publisher pub_StartPose = _nh.advertise<geometry_msgs::PoseStamped>("pbRviz/StartPose", 16); // Для публикации стартовой позиции

    ros::Publisher publish_Mode0 = _nh.advertise<nav_msgs::Odometry>("pbRviz/mode0", 8);     // Это мы создаем публикатор и определяем название топика в рос
    ros::Publisher publish_Mode1 = _nh.advertise<nav_msgs::Odometry>("pbRviz/mode1", 8);     // Это мы создаем публикатор и определяем название топика в рос
    ros::Publisher publish_Mode2 = _nh.advertise<nav_msgs::Odometry>("pbRviz/mode2", 8);     // Это мы создаем публикатор и определяем название топика в рос
    ros::Publisher publish_Mode3 = _nh.advertise<nav_msgs::Odometry>("pbRviz/mode3", 8);     // Это мы создаем публикатор и определяем название топика в рос

    // ros::Publisher pub_PoseLidarMode1 = _nh.advertise<geometry_msgs::PoseStamped>("pbRviz/PoseLidarMode1", 16); // Это мы публикуем итоговую информацию по позици лидара расчет по mode1
    // ros::Publisher pub_PoseLidarMode2 = _nh.advertise<geometry_msgs::PoseStamped>("pbRviz/PoseLidarMode2", 16); // Это мы публикуем итоговую информацию по позици лидара расчет по mode2
};

#endif