#ifndef TOPIC_H
#define TOPIC_H

/*
 Класс для функций для формирования топиков в нужном виде и формате и всех публикаций
*/
#include "config.h"
#include "pillar.h"

class CTopic
{
public:
    CTopic(/* args */);
    ~CTopic();
    //**************************** ОБЬЯВЛЕНИЕ ПРОЦЕДУР **********************************
    void transformBase(SPose poseBase_);     // Публикуем трансформации для системы координат
    void transformLidar(SPose poseLidar_);   // Публикуем трансформации для системы координат
    void transformLaser(CLaser &laser_);     // Публикуем трансформации для системы координат
    void visualPillarPoint(CPillar pillar_); // Формируем перемнную с собщением для публикации
    void visualStartPose();
    void visualPoseLidarMode_1_2();            // Формируем перемнную с собщением для публикации
    void visualPoseAngleLaser(CLaser &laser_); // Формируем перемнную с собщением для публикации по углам лазера

    void publicationPoseLidarAll();                                 // Формируем перемнную с собщением для публикации по позиции лидара
    void publicationControlModul(pb_msgs::Struct_Data2Modul data_); // Публикация данных для управления Modul
    void publicationAngleLaser(CLaser &laser_);                     // Формируем перемнную с собщением для публикации по углам лазера
    void publicationPillarAll(CPillar pillar_);                     // Формируем перемнную с собщением для публикации

    // Перенес из data_node **************
    // void transform(); // Публикуем трансформации для системы координат
    // void transformWheel(SPose pose_);
    // void transformUnited(SPose pose_);
    // void transformMpu(SPose pose_);

    void visualPublishOdomMode_0();
    void visualPublishOdomMode_1();
    void visualPublishOdomMode_2();
    void visualPublishOdomMode_3();
    void visualPublishOdomMode_123();

    void visualPublishOdomMode_11();
    void visualPublishOdomMode_12();
    void visualPublishOdomMode_13();

    // void publishOdomMpu();
    // void publishOdomUnited();
    //***********************

private:
    ros::NodeHandle _nh;
    tf::TransformBroadcaster tfBroadcaster; // Вещание данных преобразования систем координат
    ros::Time ros_time;                     // Время ROS

    //--------------------------------- ПУБЛИКАЦИЯ В ТОПИКИ -------------------------------------------------

    ros::Publisher pub_ControlModul = _nh.advertise<pb_msgs::Struct_Data2Modul>("pbPos/ControlModul", 16); // Это мы публикуем структуру которую отправляем к исполнению на драйвер

    ros::Publisher pub_poseLidar = _nh.advertise<pb_msgs::Struct_PoseLidar>("pbPos/PoseLidar", 8);          // Это мы публикуем итоговую информацию по позици лидара обобщенную
    ros::Publisher pub_poseRotation = _nh.advertise<pb_msgs::Struct_PoseRotation>("pbPos/PoseRotation", 8); // Это мы публикуем итоговую информацию по позици лидара обобщенную

    ros::Publisher pub_AngleLLAll = _nh.advertise<pb_msgs::SAngleLaserLidar>("pbPos/AngleLLAll", 16); // Это мы публикуем итоговую информацию по углам лазера для нижнего уровня

    ros::Publisher pub_PillarAll = _nh.advertise<pb_msgs::PillarOut>("pbPos/PillarAll", 16); // Это мы публикуем итоговую обобщенную информацию по столбам где все данные указаны НАФИГА?

    ros::Publisher pub_markerPillar = _nh.advertise<visualization_msgs::Marker>("markerPillar0", 0); // Публикуем столбы как маркер тип цилиндр

    ros::Publisher pub_StartPose = _nh.advertise<geometry_msgs::PoseStamped>("pbRviz/StartPose", 16); // Для публикации стартовой позиции

    // СТРЕЛКИ на столбы
    ros::Publisher pub_poseLaser0 = _nh.advertise<geometry_msgs::PoseStamped>("pbRviz/PoseLaser0", 16); // Публикатор для позиции лазера на моторе 0
    ros::Publisher pub_poseLaser1 = _nh.advertise<geometry_msgs::PoseStamped>("pbRviz/PoseLaser1", 16); // Публикатор для позиции лазера на моторе 1
    ros::Publisher pub_poseLaser2 = _nh.advertise<geometry_msgs::PoseStamped>("pbRviz/PoseLaser2", 16); // Публикатор для позиции лазера на моторе 2
    ros::Publisher pub_poseLaser3 = _nh.advertise<geometry_msgs::PoseStamped>("pbRviz/PoseLaser3", 16); // Публикатор для позиции лазера на моторе 3

    ros::Publisher publish_Mode0 = _nh.advertise<nav_msgs::Odometry>("pbRviz/mode0", 8);     // Это мы создаем публикатор и определяем название топика в рос
    ros::Publisher publish_Mode1 = _nh.advertise<nav_msgs::Odometry>("pbRviz/mode1", 8);     // Это мы создаем публикатор и определяем название топика в рос
    ros::Publisher publish_Mode2 = _nh.advertise<nav_msgs::Odometry>("pbRviz/mode2", 8);     // Это мы создаем публикатор и определяем название топика в рос
    ros::Publisher publish_Mode3 = _nh.advertise<nav_msgs::Odometry>("pbRviz/mode3", 8);     // Это мы создаем публикатор и определяем название топика в рос
    ros::Publisher publish_Mode11 = _nh.advertise<nav_msgs::Odometry>("pbRviz/odom11", 8);   // Это мы создаем публикатор и определяем название топика в рос
    ros::Publisher publish_Mode12 = _nh.advertise<nav_msgs::Odometry>("pbRviz/odom12", 8);   // Это мы создаем публикатор и определяем название топика в рос
    ros::Publisher publish_Mode13 = _nh.advertise<nav_msgs::Odometry>("pbRviz/odom13", 8);   // Это мы создаем публикатор и определяем название топика в рос
    ros::Publisher publish_Mode123 = _nh.advertise<nav_msgs::Odometry>("pbRviz/mode123", 8); // Это мы создаем публикатор и определяем название топика в рос

    // ros::Publisher pub_PoseLidarMode1 = _nh.advertise<geometry_msgs::PoseStamped>("pbRviz/PoseLidarMode1", 16); // Это мы публикуем итоговую информацию по позици лидара расчет по mode1
    // ros::Publisher pub_PoseLidarMode2 = _nh.advertise<geometry_msgs::PoseStamped>("pbRviz/PoseLidarMode2", 16); // Это мы публикуем итоговую информацию по позици лидара расчет по mode2

    // ros::Publisher publish_OdomWheel = _nh.advertise<nav_msgs::Odometry>("pbMain/odom/Wheel", 3); // Это мы создаем публикатор и определяем название топика в рос
    //  nav_msgs::Odometry odomUnited_msg;
    //  ros::Publisher publish_OdomUnited = _nh.advertise<nav_msgs::Odometry>("pbData/odom/United", 3); // Это мы создаем публикатор и определяем название топика в рос
    //  nav_msgs::Odometry odomMpu_msg;
    //  ros::Publisher publish_OdomMpu = _nh.advertise<nav_msgs::Odometry>("pbData/odom/Mpu", 3); // Это мы создаем публикатор и определяем название топика в рос

    // nav_msgs::Odometry encoderOdom_msg;
    // ros::Publisher pub_encoderOdom = _nh.advertise<nav_msgs::Odometry>("pbinfo/encoderOdom", 16);

    // nav_msgs::Odometry mpuOdom_msg;
    // ros::Publisher pub_mpuOdom = _nh.advertise<nav_msgs::Odometry>("pbinfo/mpuOdom", 16);
    //*******************
};

CTopic::CTopic(/* args */)
{
}

CTopic::~CTopic()
{
}

// Публикация данных для управления Modul
void CTopic::publicationControlModul(pb_msgs::Struct_Data2Modul data_)
{
    pub_ControlModul.publish(data_);
    // printf("++++++++++++++++++publicationControlModul ++++++++++++++++++ \n");
    // for (int i = 0; i < 4; i++)
    // {
    //     printf("i = %i ", i);
    //     printf(" g_angleLaser = %f ", g_angleLaser[i]);
    //     printf(" g_numPillar = %i \n", g_numPillar[i]);
    // }
    // printf("++++++++++++++++++++++++++++++++++++ \n");
}
void CTopic::visualStartPose()
{
    geometry_msgs::PoseStamped startPose_msg;      // Начальная позиция отображаем в RVIZ
    startPose_msg.header.stamp = ros::Time::now(); // Время ROS
    startPose_msg.header.frame_id = "odom";
    startPose_msg.pose.position.x = msg_startPose2d.x;
    startPose_msg.pose.position.y = msg_startPose2d.y;
    float theta = DEG2RAD(msg_startPose2d.theta); //
    geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(theta);
    startPose_msg.pose.orientation = quat;
    // ROS_INFO("startPose_msg Quaternion x =%.3f y =%.3f z =%.3f w =%.3f theta = %.3f", quat.x, quat.y, quat.z, quat.w, theta);

    pub_StartPose.publish(startPose_msg); // Публикация полученных данных
}
void CTopic::visualPillarPoint(CPillar pillar_) // Готовим одиночные точки столбов для RVIZ  в системе координат "odom"
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = "odom";
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

void CTopic::publicationPillarAll(CPillar pillar_) // Формируем перемнную с собщением для публикации
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
// Публикация углов на столбы по данным лидара и лазера?
void CTopic::publicationAngleLaser(CLaser &laser_)
{
    pb_msgs::SAngleLaserLidar angleLLAll_msg; // Перемеенная в которую сохраняем данные лидара из сообщения
    pb_msgs::SAngleLL data;
    angleLLAll_msg.data.clear(); // Очищаем старые точки из массива
    for (int i = 0; i < 4; i++)
    {
        data.angleFromLaser = laser_.anglePillarInLaser[i];
        data.angleFromLidar = laser_.anglePillarInLidar[i];
        angleLLAll_msg.data.push_back(data);
    }
    pub_AngleLLAll.publish(angleLLAll_msg); // Публикуем информацию по углам лазера
}

void CTopic::publicationPoseLidarAll() // Формируем перемнную с собщением для публикации
{
    pb_msgs::Struct_PoseLidar poseLidarAll_msg; // Обобщенные данные в моем формате о всех вариантах расчета позиции

    poseLidarAll_msg.mode10.x = g_poseLidar.mode10.x;
    poseLidarAll_msg.mode10.y = g_poseLidar.mode10.y;
    poseLidarAll_msg.mode10.th = g_poseLidar.mode10.th;

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
// Отобращение стрелкой где начало и куда смотрят лазеры
void CTopic::visualPoseAngleLaser(CLaser &laser_)
{
    geometry_msgs::PoseStamped poseLaser_msg; // Позиция лазера установленного на моторе 0

    ros_time = ros::Time::now(); // Время ROS
    poseLaser_msg.header.stamp = ros_time;
    poseLaser_msg.header.frame_id = "laser0";
    poseLaser_msg.pose.orientation = tf::createQuaternionMsgFromYaw(DEG2RAD(-laser_.anglePillarInLaser[0])); // Минус так как у меня вращение по часовой
    pub_poseLaser0.publish(poseLaser_msg);                                                                   // Публикуем информацию по позиции луча лазераустановленного на моторе 0 в своей локальной ситеме координат laser0

    poseLaser_msg.header.frame_id = "laser1";
    poseLaser_msg.pose.orientation = tf::createQuaternionMsgFromYaw(DEG2RAD(-laser_.anglePillarInLaser[1])); // Минус так как у меня вращение по часовой
    pub_poseLaser1.publish(poseLaser_msg);                                                                   // Публикуем информацию по позиции луча лазераустановленного на моторе 0 в своей локальной ситеме координат laser0

    poseLaser_msg.header.frame_id = "laser2";
    poseLaser_msg.pose.orientation = tf::createQuaternionMsgFromYaw(DEG2RAD(-laser_.anglePillarInLaser[2])); // Минус так как у меня вращение по часовой
    pub_poseLaser2.publish(poseLaser_msg);                                                                   // Публикуем информацию по позиции луча лазераустановленного на моторе 0 в своей локальной ситеме координат laser0

    poseLaser_msg.header.frame_id = "laser3";
    poseLaser_msg.pose.orientation = tf::createQuaternionMsgFromYaw(DEG2RAD(-laser_.anglePillarInLaser[3])); // Минус так как у меня вращение по часовой
    pub_poseLaser3.publish(poseLaser_msg);                                                                   // Публикуем информацию по позиции луча лазераустановленного на моторе 0 в своей локальной ситеме координат laser0
}

// Отобращение стрелкой где начало и куда смотрит в Mode0 1 2
void CTopic::visualPublishOdomMode_1()
{
    // Публикация Одометрии
    nav_msgs::Odometry mode1_msg;
    mode1_msg.header.stamp = ros::Time::now(); // Время ROS
    mode1_msg.header.frame_id = "odom";        // Поза в этом сообщении должна быть указана в системе координат, заданной header.frame_id.
    // set the position
    mode1_msg.pose.pose.position.x = g_poseLidar.mode1.x;
    mode1_msg.pose.pose.position.y = g_poseLidar.mode1.y;
    // geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(DEG2RAD(-g_poseLidar.mode1.th + 90)); // Минус так как вращение у меня по часовой а не по "буравчику" и + 90 так как считал я что 0 градусов это по оси Y Глядел на стену надо переписывать
    geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(DEG2RAD(g_poseLidar.mode1.th)); // Минус так как вращение у меня по часовой а не по "буравчику" и + 90 так как считал я что 0 градусов это по оси Y Глядел на стену надо переписывать
    mode1_msg.pose.pose.orientation = quat;
    // set the velocity
    mode1_msg.child_frame_id = "odom"; // Поворот в этом сообщении должен быть указан в системе координат, заданной child_frame_id
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
void CTopic::visualPublishOdomMode_2()
{
    // Публикация Одометрии
    nav_msgs::Odometry mode2_msg;
    mode2_msg.header.stamp = ros::Time::now(); // Время ROS
    mode2_msg.header.frame_id = "odom";        // Поза в этом сообщении должна быть указана в системе координат, заданной header.frame_id.
    // set the position
    mode2_msg.pose.pose.position.x = g_poseLidar.mode2.x;
    mode2_msg.pose.pose.position.y = g_poseLidar.mode2.y;
    // geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(DEG2RAD(-g_poseLidar.mode2.th + 90)); // Минус так как вращение у меня по часовой а не по "буравчику" и + 90 так как считал я что 0 градусов это по оси Y Глядел на стену надо переписывать
    geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(DEG2RAD(g_poseLidar.mode2.th)); // Минус так как вращение у меня по часовой а не по "буравчику" и + 90 так как считал я что 0 градусов это по оси Y Глядел на стену надо переписывать
    mode2_msg.pose.pose.orientation = quat;
    // set the velocity
    mode2_msg.child_frame_id = "odom"; // Поворот в этом сообщении должен быть указан в системе координат, заданной child_frame_id
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
void CTopic::visualPublishOdomMode_3()
{
    // Публикация Одометрии
    nav_msgs::Odometry mode_msg;
    mode_msg.header.stamp = ros::Time::now(); // Время ROS
    mode_msg.header.frame_id = "odom";        // Поза в этом сообщении должна быть указана в системе координат, заданной header.frame_id.
    // set the position
    mode_msg.pose.pose.position.x = g_poseLidar.mode3.x;
    mode_msg.pose.pose.position.y = g_poseLidar.mode3.y;
    geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(DEG2RAD(g_poseLidar.mode3.th)); // Минус так как вращение у меня по часовой а не по "буравчику" и + 90 так как считал я что 0 градусов это по оси Y Глядел на стену надо переписывать
    mode_msg.pose.pose.orientation = quat;
    // set the velocity
    mode_msg.child_frame_id = "odom"; // Поворот в этом сообщении должен быть указан в системе координат, заданной child_frame_id
    mode_msg.twist.twist.linear.x = 0;
    mode_msg.twist.twist.linear.y = 0;
    mode_msg.twist.twist.angular.z = 0;
    publish_Mode3.publish(mode_msg); // Публикация полученных данных
}
// Отобращение стрелкой где начало и куда смотрит в Mode123
void CTopic::visualPublishOdomMode_123()
{
    // Публикация Одометрии
    // nav_msgs::Odometry mode_msg;
    // mode_msg.header.stamp = ros::Time::now(); // Время ROS
    // mode_msg.header.frame_id = "odom";        // Поза в этом сообщении должна быть указана в системе координат, заданной header.frame_id.
    // // set the position
    // mode_msg.pose.pose.position.x = g_poseLidar.mode123.x;
    // mode_msg.pose.pose.position.y = g_poseLidar.mode123.y;
    // geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(DEG2RAD(g_poseLidar.mode123.th)); // Минус так как вращение у меня по часовой а не по "буравчику" и + 90 так как считал я что 0 градусов это по оси Y Глядел на стену надо переписывать
    // mode_msg.pose.pose.orientation = quat;
    // // set the velocity
    // mode_msg.child_frame_id = "odom"; // Поворот в этом сообщении должен быть указан в системе координат, заданной child_frame_id
    // mode_msg.twist.twist.linear.x = 0;
    // mode_msg.twist.twist.linear.y = 0;
    // mode_msg.twist.twist.angular.z = 0;
    // publish_Mode123.publish(mode_msg); // Публикация полученных данных
}

// Отобращение стрелкой где начало и куда смотрит в Mode11
void CTopic::visualPublishOdomMode_11()
{
    // Публикация Одометрии
    // nav_msgs::Odometry mode_msg;
    // mode_msg.header.stamp = ros::Time::now(); // Время ROS
    // mode_msg.header.frame_id = "odom";        // Поза в этом сообщении должна быть указана в системе координат, заданной header.frame_id.
    // // set the position
    // mode_msg.pose.pose.position.x = odomMode11.pose.x;
    // mode_msg.pose.pose.position.y = odomMode11.pose.y;
    // geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(odomMode11.pose.th); // Минус так как вращение у меня по часовой а не по "буравчику" и + 90 так как считал я что 0 градусов это по оси Y Глядел на стену надо переписывать
    // mode_msg.pose.pose.orientation = quat;
    // // set the velocity
    // mode_msg.child_frame_id = "odom"; // Поворот в этом сообщении должен быть указан в системе координат, заданной child_frame_id
    // mode_msg.twist.twist.linear.x = odomMode11.twist.vx;
    // mode_msg.twist.twist.linear.y = odomMode11.twist.vy;
    // mode_msg.twist.twist.angular.z = odomMode11.twist.vth;
    // publish_Mode11.publish(mode_msg); // Публикация полученных данных
}
// Отобращение стрелкой где начало и куда смотрит в Mode12
void CTopic::visualPublishOdomMode_12()
{
    // Публикация Одометрии
    // nav_msgs::Odometry mode_msg;
    // mode_msg.header.stamp = ros::Time::now(); // Время ROS
    // mode_msg.header.frame_id = "odom";        // Поза в этом сообщении должна быть указана в системе координат, заданной header.frame_id.
    // // set the position
    // mode_msg.pose.pose.position.x = odomMode12.pose.x;
    // mode_msg.pose.pose.position.y = odomMode12.pose.y;
    // geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(odomMode12.pose.th); // Минус так как вращение у меня по часовой а не по "буравчику" и + 90 так как считал я что 0 градусов это по оси Y Глядел на стену надо переписывать
    // mode_msg.pose.pose.orientation = quat;
    // // set the velocity
    // mode_msg.child_frame_id = "odom"; // Поворот в этом сообщении должен быть указан в системе координат, заданной child_frame_id
    // mode_msg.twist.twist.linear.x = odomMode12.twist.vx;
    // mode_msg.twist.twist.linear.y = odomMode12.twist.vy;
    // mode_msg.twist.twist.angular.z = odomMode12.twist.vth;
    // publish_Mode12.publish(mode_msg); // Публикация полученных данных
}
// Отобращение стрелкой где начало и куда смотрит в Mode13
void CTopic::visualPublishOdomMode_13()
{
    // Публикация Одометрии
    // nav_msgs::Odometry mode_msg;
    // mode_msg.header.stamp = ros::Time::now(); // Время ROS
    // mode_msg.header.frame_id = "odom";        // Поза в этом сообщении должна быть указана в системе координат, заданной header.frame_id.
    // // set the position
    // mode_msg.pose.pose.position.x = odomMode13.pose.x;
    // mode_msg.pose.pose.position.y = odomMode13.pose.y;
    // geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(odomMode13.pose.th); // Минус так как вращение у меня по часовой а не по "буравчику" и + 90 так как считал я что 0 градусов это по оси Y Глядел на стену надо переписывать
    // mode_msg.pose.pose.orientation = quat;
    // // set the velocity
    // mode_msg.child_frame_id = "odom"; // Поворот в этом сообщении должен быть указан в системе координат, заданной child_frame_id
    // mode_msg.twist.twist.linear.x = odomMode13.twist.vx;
    // mode_msg.twist.twist.linear.y = odomMode13.twist.vy;
    // mode_msg.twist.twist.angular.z = odomMode13.twist.vth;
    // publish_Mode13.publish(mode_msg); // Публикация полученных данных
}

void CTopic::visualPublishOdomMode_0()
{
    // nav_msgs::Odometry mode0_msg;              // Публикация Одометрии
    // mode0_msg.header.stamp = ros::Time::now(); // Время ROS
    // mode0_msg.header.frame_id = "odom";        // Поза в этом сообщении должна быть указана в системе координат, заданной header.frame_id.
    // // set the position
    // mode0_msg.pose.pose.position.x = odomMode0.pose.x;
    // mode0_msg.pose.pose.position.y = odomMode0.pose.y;
    // // geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(-odomMode0.pose.th + DEG2RAD(90)); // Минус так как вращение у меня по часовой а не по "буравчику" и + 90 так как считал я что 0 градусов это по оси Y Глядел на стену надо переписывать
    // geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(odomMode0.pose.th); // Минус так как вращение у меня по часовой а не по "буравчику" и + 90 так как считал я что 0 градусов это по оси Y Глядел на стену надо переписывать
    // mode0_msg.pose.pose.orientation = quat;
    // // set the velocity
    // mode0_msg.child_frame_id = "odom"; // Поворот в этом сообщении должен быть указан в системе координат, заданной child_frame_id
    // mode0_msg.twist.twist.linear.x = odomMode0.twist.vx;
    // mode0_msg.twist.twist.linear.y = odomMode0.twist.vy;
    // mode0_msg.twist.twist.angular.z = odomMode0.twist.vth;
    // publish_Mode0.publish(mode0_msg); // Публикация полученных данных

    // Публиация системы трансформации для TF Решил что для одометрии не нужно еще и системы координат публиковать TF
    // geometry_msgs::TransformStamped tfOdomWheel;
    // tfOdomWheel.header.stamp = ros_time;
    // tfOdomWheel.header.frame_id = "odom";
    // tfOdomWheel.child_frame_id = "wheel";
    // tfOdomWheel.transform.translation.x = odomMode0.pose.x;
    // tfOdomWheel.transform.translation.y = odomMode0.pose.y;
    // tfOdomWheel.transform.translation.z = 0.1;
    // tfOdomWheel.transform.rotation = tf::createQuaternionMsgFromYaw(-odomMode0.pose.th); // Из градусов в радианы далее подладить под своё представление
    // tfBroadcaster.sendTransform(tfOdomWheel);
    //-------------------------------------------
}

// void CTopic::publishOdomUnited()
// {
// 	transformUnited(odomUnited.pose);				// Публиация системы трансформации
// 	odomUnited_msg.header.stamp = ros::Time::now(); // Время ROS
// 	odomUnited_msg.header.frame_id = "odom";		// Поза в этом сообщении должна быть указана в системе координат, заданной header.frame_id.
// 	// set the position
// 	odomUnited_msg.pose.pose.position.x = odomUnited.pose.x;
// 	odomUnited_msg.pose.pose.position.y = odomUnited.pose.y;
// 	geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(-odomUnited.pose.th);
// 	odomUnited_msg.pose.pose.orientation = quat;
// 	// set the velocity
// 	odomUnited_msg.child_frame_id = "odom"; // Поворот в этом сообщении должен быть указан в системе координат, заданной child_frame_id
// 	odomUnited_msg.twist.twist.linear.x = odomUnited.twist.vx;
// 	odomUnited_msg.twist.twist.linear.y = odomUnited.twist.vy;
// 	odomUnited_msg.twist.twist.angular.z = odomUnited.twist.vth;
// 	publish_OdomUnited.publish(odomUnited_msg); // Публикация полученных данных
// }
// void CTopic::publishOdomMpu()
// {
// 	transformMpu(odomMpu.pose);
// 	odomMpu_msg.header.stamp = ros::Time::now(); // Время ROS
// 	odomMpu_msg.header.frame_id = "odom";		 // Поза в этом сообщении должна быть указана в системе координат, заданной header.frame_id.
// 	// set the position
// 	odomMpu_msg.pose.pose.position.x = odomMpu.pose.x;
// 	odomMpu_msg.pose.pose.position.y = odomMpu.pose.y;
// 	geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(odomMpu.pose.th);
// 	odomMpu_msg.pose.pose.orientation = quat;
// 	// set the velocity
// 	odomMpu_msg.child_frame_id = "odom"; // Поворот в этом сообщении должен быть указан в системе координат, заданной child_frame_id
// 	odomMpu_msg.twist.twist.linear.x = odomMpu.twist.vx;
// 	odomMpu_msg.twist.twist.linear.y = odomMpu.twist.vy;
// 	odomMpu_msg.twist.twist.angular.z = odomMpu.twist.vth;
// 	publish_OdomMpu.publish(odomMpu_msg); // Публикация полученных данных
// }

//*****************************************************************************************************************************************************************

void CTopic::transformBase(SPose poseBase_) // Публикуем системы трансормаций из одних систем координат в другие
{
    ros_time = ros::Time::now(); // Время ROS
    // --------------------------------- map odom  ---------------------------------------
    geometry_msgs::TransformStamped tfMapOdom;
    tfMapOdom.header.stamp = ros_time;
    tfMapOdom.header.frame_id = "map";
    tfMapOdom.child_frame_id = "odom";
    tfMapOdom.transform.translation.x = 0.0;
    tfMapOdom.transform.translation.y = 0.0;
    tfMapOdom.transform.translation.z = 0.0;
    tfMapOdom.transform.rotation = tf::createQuaternionMsgFromYaw(DEG2RAD(0)); // Из градусов в радианы далле подладить под своё представление
    tfBroadcaster.sendTransform(tfMapOdom);                                    // Публикация системы преобразования из odom в map Тут динамически, а статически выглядит так   <node pkg="tf" type="static_transform_publisher" name="static_map_odom_tf" args="0 0 0 0 0 0 map odom 100" /> <!--http://wiki.ros.org/tf-->

    // --------------------------------- odom base ---------------------------------------
    geometry_msgs::TransformStamped tfOdomBase;
    tfOdomBase.header.stamp = ros_time;
    tfOdomBase.header.frame_id = "odom";
    tfOdomBase.child_frame_id = "base";
    // printf("AAAAAAAAAAA % .3f %.3f", poseBase_.x,poseBase_.y);
    tfOdomBase.transform.translation.x = poseBase_.x;
    tfOdomBase.transform.translation.y = poseBase_.y;
    tfOdomBase.transform.translation.z = 0.1;
    // tfOdomBase.transform.rotation = tf::createQuaternionMsgFromYaw(DEG2RAD(-poseBase_.th + 90)); // Из градусов в радианы далле подладить под своё представление +90 так как считал что ноль это ось У а не Х при отладке. надо переделывать
    tfOdomBase.transform.rotation = tf::createQuaternionMsgFromYaw(DEG2RAD(poseBase_.th)); // Из градусов в радианы далле подладить под своё представление +90 так как считал что ноль это ось У а не Х при отладке. надо переделывать
    tfBroadcaster.sendTransform(tfOdomBase);                                               // Публикация системы преобразования из odom в map Тут динамически, а статически выглядит так   <node pkg="tf" type="static_transform_publisher" name="static_map_odom_tf" args="0 0 0 0 0 0 map odom 100" /> <!--http://wiki.ros.org/tf-->
}
void CTopic::transformLidar(SPose poseLidar_) // Публикуем системы трансормаций из одних систем координат в другие
{
    // --------------------------------- base laser Для ЛИДАРА---------------------------------------
    geometry_msgs::TransformStamped tfOdomLaser;
    tfOdomLaser.header.stamp = ros_time;
    tfOdomLaser.header.frame_id = "base";
    tfOdomLaser.child_frame_id = "laser";
    tfOdomLaser.transform.translation.x = 0;
    tfOdomLaser.transform.translation.y = 0;
    // tfOdomLaser.transform.translation.x = poseLidar_.x;
    // tfOdomLaser.transform.translation.y = poseLidar_.y;
    tfOdomLaser.transform.translation.z = 0.3;
    // tfOdomLaser.transform.rotation = tf::createQuaternionMsgFromYaw(DEG2RAD(poseLidar_.th - 180)); // Из градусов в радианы добавляем минус, так как в РОС вращение плюс против часовой, а у меня по часовой
    tfOdomLaser.transform.rotation = tf::createQuaternionMsgFromYaw(DEG2RAD(-180)); // Из градусов в радианы добавляем минус, так как в РОС вращение плюс против часовой, а у меня по часовой
    tfBroadcaster.sendTransform(tfOdomLaser);                                       // Публикация системы преобразования из odom в map Тут динамически, а статически выглядит так   <node pkg="tf" type="static_transform_publisher" name="static_map_odom_tf" args="0 0 0 0 0 0 map odom 100" /> <!--http://wiki.ros.org/tf-->
}
void CTopic::transformLaser(CLaser &laser_) // Публикуем системы трансормаций из одних систем координат в другие
{
    // --------------------------------- base laser0---------------------------------------
    geometry_msgs::TransformStamped tfOdomLaser0;
    tfOdomLaser0.header.stamp = ros_time;
    tfOdomLaser0.header.frame_id = "base";
    tfOdomLaser0.child_frame_id = "laser0";
    tfOdomLaser0.transform.translation.x = laser_._poseLaser[0].x;
    tfOdomLaser0.transform.translation.y = laser_._poseLaser[0].y;
    tfOdomLaser0.transform.translation.z = 0.05;
    tfOdomLaser0.transform.rotation = tf::createQuaternionMsgFromYaw(DEG2RAD(laser_._poseLaser[0].th)); // по стандарту РОС
    tfBroadcaster.sendTransform(tfOdomLaser0);                                                          // Публикация системы преобразования из odom в map Тут динамически, а статически выглядит так   <node pkg="tf" type="static_transform_publisher" name="static_map_odom_tf" args="0 0 0 0 0 0 map odom 100" /> <!--http://wiki.ros.org/tf-->
    // --------------------------------- base laser1---------------------------------------
    geometry_msgs::TransformStamped tfOdomLaser1;
    tfOdomLaser1.header.stamp = ros_time;
    tfOdomLaser1.header.frame_id = "base";
    tfOdomLaser1.child_frame_id = "laser1";
    tfOdomLaser1.transform.translation.x = laser_._poseLaser[1].x;
    tfOdomLaser1.transform.translation.y = laser_._poseLaser[1].y;
    tfOdomLaser1.transform.translation.z = 0.05;
    tfOdomLaser1.transform.rotation = tf::createQuaternionMsgFromYaw(DEG2RAD(laser_._poseLaser[1].th)); // по стандарту РОС
    tfBroadcaster.sendTransform(tfOdomLaser1);                                                          // Публикация системы преобразования из odom в map Тут динамически, а статически выглядит так   <node pkg="tf" type="static_transform_publisher" name="static_map_odom_tf" args="0 0 0 0 0 0 map odom 100" /> <!--http://wiki.ros.org/tf-->

    // --------------------------------- base laser2---------------------------------------
    geometry_msgs::TransformStamped tfOdomLaser2;
    tfOdomLaser2.header.stamp = ros_time;
    tfOdomLaser2.header.frame_id = "base";
    tfOdomLaser2.child_frame_id = "laser2";
    tfOdomLaser2.transform.translation.x = laser_._poseLaser[2].x;
    tfOdomLaser2.transform.translation.y = laser_._poseLaser[2].y;
    tfOdomLaser2.transform.translation.z = 0.05;
    tfOdomLaser2.transform.rotation = tf::createQuaternionMsgFromYaw(DEG2RAD(laser_._poseLaser[2].th)); // по стандарту РОС
    tfBroadcaster.sendTransform(tfOdomLaser2);                                                          // Публикация системы преобразования из odom в map Тут динамически, а статически выглядит так   <node pkg="tf" type="static_transform_publisher" name="static_map_odom_tf" args="0 0 0 0 0 0 map odom 100" /> <!--http://wiki.ros.org/tf-->
    // --------------------------------- base laser3---------------------------------------
    geometry_msgs::TransformStamped tfOdomLaser3;
    tfOdomLaser3.header.stamp = ros_time;
    tfOdomLaser3.header.frame_id = "base";
    tfOdomLaser3.child_frame_id = "laser3";
    tfOdomLaser3.transform.translation.x = laser_._poseLaser[3].x;
    tfOdomLaser3.transform.translation.y = laser_._poseLaser[3].y;
    tfOdomLaser3.transform.translation.z = 0.05;
    tfOdomLaser3.transform.rotation = tf::createQuaternionMsgFromYaw(DEG2RAD(laser_._poseLaser[3].th)); // по стандарту РОС
    tfBroadcaster.sendTransform(tfOdomLaser3);                                                          // Публикация системы преобразования из odom в map Тут динамически, а статически выглядит так   <node pkg="tf" type="static_transform_publisher" name="static_map_odom_tf" args="0 0 0 0 0 0 map odom 100" /> <!--http://wiki.ros.org/tf-->
}

// void CTopic::transformWheel(SPose pose_) // Публикуем системы трансформаций из одних систем координат в другие
// {
// 	geometry_msgs::TransformStamped tfOdomWheel;
// 	tfOdomWheel.header.stamp = ros_time;
// 	tfOdomWheel.header.frame_id = "odom";
// 	tfOdomWheel.child_frame_id = "wheel";
// 	tfOdomWheel.transform.translation.x = pose_.x;
// 	tfOdomWheel.transform.translation.y = pose_.y;
// 	tfOdomWheel.transform.translation.z = 0.1;
// 	tfOdomWheel.transform.rotation = tf::createQuaternionMsgFromYaw(-pose_.th); // Из градусов в радианы далее подладить под своё представление
// 	tfBroadcaster.sendTransform(tfOdomWheel);									// Публикация системы преобразования из odom в map Тут динамически, а статически выглядит так   <node pkg="tf" type="static_transform_publisher" name="static_map_odom_tf" args="0 0 0 0 0 0 map odom 100" /> <!--http://wiki.ros.org/tf-->
// }
// void CTopic::transformUnited(SPose pose_) // Публикуем системы трансормаций из одних систем координат в другие
// {
// 	geometry_msgs::TransformStamped tfOdomUnited;
// 	tfOdomUnited.header.stamp = ros_time;
// 	tfOdomUnited.header.frame_id = "odom";
// 	tfOdomUnited.child_frame_id = "united";
// 	tfOdomUnited.transform.translation.x = pose_.x;
// 	tfOdomUnited.transform.translation.y = pose_.y;
// 	tfOdomUnited.transform.translation.z = 0.1;
// 	tfOdomUnited.transform.rotation = tf::createQuaternionMsgFromYaw(-pose_.th); // Из градусов в радианы далее подладить под своё представление
// 	tfBroadcaster.sendTransform(tfOdomUnited);									 // Публикация системы преобразования из odom в map Тут динамически, а статически выглядит так   <node pkg="tf" type="static_transform_publisher" name="static_map_odom_tf" args="0 0 0 0 0 0 map odom 100" /> <!--http://wiki.ros.org/tf-->
// }
// void CTopic::transformMpu(SPose pose_) // Публикуем системы трансормаций из одних систем координат в другие
// {
// 	geometry_msgs::TransformStamped tfOdomMpu;
// 	tfOdomMpu.header.stamp = ros_time;
// 	tfOdomMpu.header.frame_id = "odom";
// 	tfOdomMpu.child_frame_id = "mpu";
// 	tfOdomMpu.transform.translation.x = pose_.x;
// 	tfOdomMpu.transform.translation.y = pose_.y;
// 	tfOdomMpu.transform.translation.z = 0.2;
// 	tfOdomMpu.transform.rotation = tf::createQuaternionMsgFromYaw(-pose_.th); // Из градусов в радианы далее подладить под своё представление
// 	tfBroadcaster.sendTransform(tfOdomMpu);									  // Публикация системы преобразования из odom в map Тут динамически, а статически выглядит так   <node pkg="tf" type="static_transform_publisher" name="static_map_odom_tf" args="0 0 0 0 0 0 map odom 100" /> <!--http://wiki.ros.org/tf-->
// }

#endif