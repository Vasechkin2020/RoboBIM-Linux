#ifndef TOPIC_H
#define TOPIC_H

/*
 Класс для функций для формирования топиков в нужном виде и формате и всех публикаций
*/
#include "config.h"
#include "pillar.h"
#include "code.h"

class CTopic
{
public:
    CTopic(/* args */);
    ~CTopic();
    //**************************** ОБЬЯВЛЕНИЕ ПРОЦЕДУР **********************************
    void transformBase(SPose poseBase_); // Публикуем трансформации для системы координат
    void transformLaser(CLaser &laser_); // Публикуем трансформации для системы координат
    void transformRotation();            // Публикуем трансформации систем координат

    void visualPillarPoint(CPillar pillar_); // Формируем перемнную с собщением для публикации
    void visualStartPose();
    void visualPoseLidarMode_1_2();            // Формируем перемнную с собщением для публикации
    void visualPoseAngleLaser(CLaser &laser_); // Формируем перемнную с собщением для публикации по углам лазера

    void publicationPoseBase();     // Формируем перемнную с собщением для публикации по позиции лидара
    void publicationPoseRotattion(); // Вывод в топик данных с координатами и углом точки вращения Rotation
    void publicationLinAngVel();     // Вывод в топик данных с данными угловой и линейной скоростью

    void publicationControlModul();             // Публикация данных для управления Modul
    void publicationAngleLaser(CLaser &laser_); // Формируем перемнную с собщением для публикации по углам лазера
    void publicationPillarAll(CPillar pillar_); // Формируем перемнную с собщением для публикации

    void visualPublishOdomMode_0();
    void visualPublishOdomMode_1();
    void visualPublishOdomMode_2();
    void visualPublishOdomMode_123();

    // void publishOdomUnited();
    //***********************

private:
    ros::NodeHandle _nh;
    tf::TransformBroadcaster tfBroadcaster; // Вещание данных преобразования систем координат
    ros::Time ros_time;                     // Время ROS

    //--------------------------------- ПУБЛИКАЦИЯ В ТОПИКИ -------------------------------------------------

    ros::Publisher pub_ControlModul = _nh.advertise<pb_msgs::Struct_Data2Modul>("pb/Pos/ControlModul", 1); // Это мы публикуем структуру которую отправляем к исполнению на драйвер
 
    ros::Publisher pub_poseBase = _nh.advertise<pb_msgs::Struct_PoseBase>("pb/Pos/PoseBase", 1);          // Это мы публикуем итоговую информацию по позици лидара обобщенную
    ros::Publisher pub_poseRotation = _nh.advertise<pb_msgs::Struct_PoseRotation>("pb/Pos/PoseRotation", 1); // Это мы публикуем итоговую информацию по позици лидара обобщенную

    ros::Publisher pub_linAngVel = _nh.advertise<pb_msgs::SLinAngVel>("pb/Pos/LinAngVel", 1);                // Это мы публикуем итоговую информацию линейной скорости угловой

    ros::Publisher pub_AngleLLAll = _nh.advertise<pb_msgs::SAngleLaserLidar>("pb/Pos/AngleLLAll", 1); // Это мы публикуем итоговую информацию по углам лазера для нижнего уровня

    ros::Publisher pub_PillarAll = _nh.advertise<pb_msgs::PillarOut>("pb/Pos/PillarAll", 1); // Это мы публикуем итоговую обобщенную информацию по столбам где все данные указаны НАФИГА?

    ros::Publisher pub_markerPillar = _nh.advertise<visualization_msgs::Marker>("pb/rviz/Pose/markerPillar0", 0);    // Публикуем столбы как маркер тип цилиндр
    ros::Publisher pub_markerPosition = _nh.advertise<visualization_msgs::Marker>("pb/rviz/Pose/markerPosition", 0); // Публикуем столики как точки позиций

    ros::Publisher pub_StartPose = _nh.advertise<geometry_msgs::PoseStamped>("pb/rviz/Pose/StartPose", 1); // Для публикации стартовой позиции

    // СТРЕЛКИ на столбы
    ros::Publisher pub_poseLaser0 = _nh.advertise<geometry_msgs::PoseStamped>("pb/rviz/Pose/Laser0", 16); // Публикатор для позиции лазера на моторе 0
    ros::Publisher pub_poseLaser1 = _nh.advertise<geometry_msgs::PoseStamped>("pb/rviz/Pose/Laser1", 16); // Публикатор для позиции лазера на моторе 1
    ros::Publisher pub_poseLaser2 = _nh.advertise<geometry_msgs::PoseStamped>("pb/rviz/Pose/Laser2", 16); // Публикатор для позиции лазера на моторе 2
    ros::Publisher pub_poseLaser3 = _nh.advertise<geometry_msgs::PoseStamped>("pb/rviz/Pose/Laser3", 16); // Публикатор для позиции лазера на моторе 3

    ros::Publisher publish_Mode0 = _nh.advertise<nav_msgs::Odometry>("pb/rviz/Pose/mode0", 8);     // Это мы создаем публикатор и определяем название топика в рос
    ros::Publisher publish_Mode1 = _nh.advertise<nav_msgs::Odometry>("pb/rviz/Pose/mode1", 8);     // Это мы создаем публикатор и определяем название топика в рос
    ros::Publisher publish_Mode2 = _nh.advertise<nav_msgs::Odometry>("pb/rviz/Pose/mode2", 8);     // Это мы создаем публикатор и определяем название топика в рос
    ros::Publisher publish_Mode3 = _nh.advertise<nav_msgs::Odometry>("pb/rviz/Pose/mode3", 8);     // Это мы создаем публикатор и определяем название топика в рос
    ros::Publisher publish_Mode11 = _nh.advertise<nav_msgs::Odometry>("pb/rviz/Pose/odom11", 8);   // Это мы создаем публикатор и определяем название топика в рос

    // ros::Publisher pub_PoseLidarMode1 = _nh.advertise<geometry_msgs::PoseStamped>("pb/rviz/PoseLidarMode1", 16); // Это мы публикуем итоговую информацию по позици лидара расчет по mode1
    // ros::Publisher pub_PoseLidarMode2 = _nh.advertise<geometry_msgs::PoseStamped>("pb/vriz/PoseLidarMode2", 16); // Это мы публикуем итоговую информацию по позици лидара расчет по mode2

    // ros::Publisher publish_OdomWheel = _nh.advertise<nav_msgs::Odometry>("pbMain/odom/Wheel", 3); // Это мы создаем публикатор и определяем название топика в рос
    //  nav_msgs::Odometry odomUnited_msg;
    //  ros::Publisher publish_OdomUnited = _nh.advertise<nav_msgs::Odometry>("pbData/odom/fused", 3); // Это мы создаем публикатор и определяем название топика в рос
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
void CTopic::publicationControlModul()
{
    static pb_msgs::Struct_Data2Modul dataControlModul2;
    dataControlModul2.controlMotor.mode = 1;
    dataControlModul2.controlLaser.mode = 1; // Тут указываем режи. С какой частотой будут работать датчик. Если 1 то с маленькой, если 2 то быстрее
    // Поворачиваем на этот угол
    dataControlModul2.controlMotor.angle[0] = g_angleLaser[0];
    dataControlModul2.controlMotor.numPillar[0] = g_numPillar[0];
    dataControlModul2.controlMotor.angle[1] = g_angleLaser[1];
    dataControlModul2.controlMotor.numPillar[1] = g_numPillar[1];
    dataControlModul2.controlMotor.angle[2] = g_angleLaser[2];
    dataControlModul2.controlMotor.numPillar[2] = g_numPillar[2];
    dataControlModul2.controlMotor.angle[3] = g_angleLaser[3];
    dataControlModul2.controlMotor.numPillar[3] = g_numPillar[3];
    pub_ControlModul.publish(dataControlModul2);
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
    startPose_msg.pose.position.x = startPose.x;
    startPose_msg.pose.position.y = startPose.y;
    float theta = DEG2RAD(startPose.th); //
    geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(theta);
    startPose_msg.pose.orientation = quat;
    // ROS_INFO("startPose_msg Quaternion x =%+8.3f y =%+8.3f z =%+8.3f w =%+8.3f theta = %+8.3f", quat.x, quat.y, quat.z, quat.w, theta);

    pub_StartPose.publish(startPose_msg); // Публикация полученных данных

    visualization_msgs::Marker marker;
    marker.header.frame_id = "odom";
    marker.header.stamp = ros::Time::now(); // Время ROS;
    marker.ns = "position";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CYLINDER;
    marker.action = visualization_msgs::Marker::ADD;
    marker.pose.position.x = startPose.x;
    marker.pose.position.y = startPose.y;
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
        // ROS_INFO("Status= %i azimuth= %+8.3f",pillar_out_msg.data[i].status,pillar_out_msg.data[i].azimuth);
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

void CTopic::publicationPoseBase() // Формируем перемнную с собщением для публикации
{
    pb_msgs::Struct_PoseBase poseBase_msg; // Обобщенные данные в моем формате о всех вариантах расчета позиции

    // poseBase_msg.x.odom = g_poseLidar.odom.x;
    // poseBase_msg.y.odom = g_poseLidar.odom.y;
    // poseBase_msg.th.odom = g_poseLidar.odom.th;

    // poseBase_msg.x.fused = g_poseLidar.fused.x;
    // poseBase_msg.y.fused = g_poseLidar.fused.y;
    // poseBase_msg.th.fused = g_poseLidar.fused.th;

    poseBase_msg.x.meas = g_poseLidar.meas.x;
    poseBase_msg.y.meas = g_poseLidar.meas.y;
    poseBase_msg.th.meas = g_poseLidar.meas.th;

    poseBase_msg.x.main = g_poseLidar.main.x;
    poseBase_msg.y.main = g_poseLidar.main.y;
    poseBase_msg.th.main = g_poseLidar.main.th;

    poseBase_msg.azimut[0] =  g_poseLidar.azimut[0];
    poseBase_msg.azimut[1] =  g_poseLidar.azimut[1];
    poseBase_msg.azimut[2] =  g_poseLidar.azimut[2];
    poseBase_msg.azimut[3] =  g_poseLidar.azimut[3];

    pub_poseBase.publish(poseBase_msg); // Публикуем информацию по позиции лидара
}
void CTopic::publicationPoseRotattion() // Вывод в топик данных с координатами и углом точки вращения Rotation
{
    pb_msgs::Struct_PoseRotation msg; // Обобщенные данные в моем формате о всех вариантах расчета позиции

    // msg.theta = g_poseRotation.theta;

    msg.x.odom = g_poseRotation.odom.x;
    msg.x.imu = g_poseRotation.imu.x;
    msg.x.fused = g_poseRotation.fused.x;
    msg.x.meas = g_poseRotation.meas.x;
    msg.x.main = g_poseRotation.main.x;

    msg.y.odom = g_poseRotation.odom.y;
    msg.y.imu = g_poseRotation.imu.y;
    msg.y.fused = g_poseRotation.fused.y;
    msg.y.meas = g_poseRotation.meas.y;
    msg.y.main = g_poseRotation.main.y;
    
    msg.th.odom = g_poseRotation.odom.th;
    msg.th.imu = g_poseRotation.imu.th;
    msg.th.fused = g_poseRotation.fused.th;
    msg.th.meas = g_poseRotation.meas.th;
    msg.th.main = g_poseRotation.main.th;
   
    msg.theta = DEG2RAD(g_angleEuler.yaw);

    pub_poseRotation.publish(msg); // Публикуем информацию по позиции лидара
}
void CTopic::publicationLinAngVel() // Вывод в топик данных с данными угловой и линейной скоростью
{
    pub_linAngVel.publish(msg_LinAngVel); // Публикуем информацию по позиции лидара
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
// void CTopic::visualPublishOdomMode_1()
// {
//     // Публикация Одометрии
//     nav_msgs::Odometry mode1_msg;
//     mode1_msg.header.stamp = ros::Time::now(); // Время ROS
//     mode1_msg.header.frame_id = "odom";        // Поза в этом сообщении должна быть указана в системе координат, заданной header.frame_id.
//     // set the position
//     mode1_msg.pose.pose.position.x = g_poseLidar.mode1.x;
//     mode1_msg.pose.pose.position.y = g_poseLidar.mode1.y;
//     // geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(DEG2RAD(-g_poseLidar.mode1.th + 90)); // Минус так как вращение у меня по часовой а не по "буравчику" и + 90 так как считал я что 0 градусов это по оси Y Глядел на стену надо переписывать
//     geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(DEG2RAD(g_poseLidar.mode1.th)); // Минус так как вращение у меня по часовой а не по "буравчику" и + 90 так как считал я что 0 градусов это по оси Y Глядел на стену надо переписывать
//     mode1_msg.pose.pose.orientation = quat;
//     // set the velocity
//     mode1_msg.child_frame_id = "odom"; // Поворот в этом сообщении должен быть указан в системе координат, заданной child_frame_id
//     mode1_msg.twist.twist.linear.x = 0;
//     mode1_msg.twist.twist.linear.y = 0;
//     mode1_msg.twist.twist.angular.z = 0;
//     publish_Mode1.publish(mode1_msg); // Публикация полученных данных

//     // geometry_msgs::PoseStamped poseLidarMode1_msg; // Перемеенная в которую сохраняем данные лидара из сообщения
//     // ros_time = ros::Time::now(); // Время ROS
//     // poseLidarMode1_msg.header.stamp = ros_time;
//     // poseLidarMode1_msg.header.frame_id = "odom";
//     // poseLidarMode1_msg.pose.position.x = g_poseLidar.mode1.x;
//     // poseLidarMode1_msg.pose.position.y = g_poseLidar.mode1.y;
//     // poseLidarMode1_msg.pose.orientation = tf::createQuaternionMsgFromYaw(DEG2RAD(-g_poseLidar.mode1.th + 90)); // + 90 Так как у них оси расположены не так как я меня. У меня ноль вверх а у них вправо и вращение у них против часовой
//     // pub_PoseLidarMode1.publish(poseLidarMode1_msg);                                                            // Публикуем информацию по позиции лидара mode1
// }
// Отобращение стрелкой где начало и куда смотрит в Mode0 1 2
// void CTopic::visualPublishOdomMode_2()
// {
//     // Публикация Одометрии
//     nav_msgs::Odometry mode2_msg;
//     mode2_msg.header.stamp = ros::Time::now(); // Время ROS
//     mode2_msg.header.frame_id = "odom";        // Поза в этом сообщении должна быть указана в системе координат, заданной header.frame_id.
//     // set the position
//     mode2_msg.pose.pose.position.x = g_poseLidar.mode2.x;
//     mode2_msg.pose.pose.position.y = g_poseLidar.mode2.y;
//     // geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(DEG2RAD(-g_poseLidar.mode2.th + 90)); // Минус так как вращение у меня по часовой а не по "буравчику" и + 90 так как считал я что 0 градусов это по оси Y Глядел на стену надо переписывать
//     geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(DEG2RAD(g_poseLidar.mode2.th)); // Минус так как вращение у меня по часовой а не по "буравчику" и + 90 так как считал я что 0 градусов это по оси Y Глядел на стену надо переписывать
//     mode2_msg.pose.pose.orientation = quat;
//     // set the velocity
//     mode2_msg.child_frame_id = "odom"; // Поворот в этом сообщении должен быть указан в системе координат, заданной child_frame_id
//     mode2_msg.twist.twist.linear.x = 0;
//     mode2_msg.twist.twist.linear.y = 0;
//     mode2_msg.twist.twist.angular.z = 0;
//     publish_Mode2.publish(mode2_msg); // Публикация полученных данных

//     // geometry_msgs::PoseStamped poseLidarMode2_msg; // Перемеенная в которую сохраняем данные лидара из сообщения
//     // ros_time = ros::Time::now(); // Время ROS
//     // poseLidarMode2_msg.header.stamp = ros_time;
//     // poseLidarMode2_msg.header.frame_id = "odom";
//     // poseLidarMode2_msg.pose.position.x = g_poseLidar.mode2.x;
//     // poseLidarMode2_msg.pose.position.y = g_poseLidar.mode2.y;
//     // poseLidarMode2_msg.pose.orientation = tf::createQuaternionMsgFromYaw(DEG2RAD(-g_poseLidar.mode2.th + 90)); // + 90 Так как у них оси расположены не так как я меня. У меня ноль вверх а у них вправо и вращение у них против часовой
//     // pub_PoseLidarMode2.publish(poseLidarMode2_msg);                                                            // Публикуем информацию по позиции лидара mode2
// }
// Отобращение стрелкой где начало и куда смотрит в Mode3
// void CTopic::visualPublishOdomMode_3()
// {
//     // Публикация Одометрии
//     nav_msgs::Odometry mode_msg;
//     mode_msg.header.stamp = ros::Time::now(); // Время ROS
//     mode_msg.header.frame_id = "odom";        // Поза в этом сообщении должна быть указана в системе координат, заданной header.frame_id.
//     // set the position
//     mode_msg.pose.pose.position.x = g_poseLidar.mode3.x;
//     mode_msg.pose.pose.position.y = g_poseLidar.mode3.y;
//     geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(DEG2RAD(g_poseLidar.mode3.th)); // Минус так как вращение у меня по часовой а не по "буравчику" и + 90 так как считал я что 0 градусов это по оси Y Глядел на стену надо переписывать
//     mode_msg.pose.pose.orientation = quat;
//     // set the velocity
//     mode_msg.child_frame_id = "odom"; // Поворот в этом сообщении должен быть указан в системе координат, заданной child_frame_id
//     mode_msg.twist.twist.linear.x = 0;
//     mode_msg.twist.twist.linear.y = 0;
//     mode_msg.twist.twist.angular.z = 0;
//     publish_Mode3.publish(mode_msg); // Публикация полученных данных
// }
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
// Публикуем системы трансормаций из одних систем координат в другие
void CTopic::transformBase(SPose poseBase_) 
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
    // printf("AAAAAAAAAAA % .3f %+8.3f", poseBase_.x,poseBase_.y);
    tfOdomBase.transform.translation.x = poseBase_.x;
    tfOdomBase.transform.translation.y = poseBase_.y;
    tfOdomBase.transform.translation.z = 0.1;
    // tfOdomBase.transform.rotation = tf::createQuaternionMsgFromYaw(DEG2RAD(-poseBase_.th + 90)); // Из градусов в радианы далле подладить под своё представление +90 так как считал что ноль это ось У а не Х при отладке. надо переделывать
    tfOdomBase.transform.rotation = tf::createQuaternionMsgFromYaw(DEG2RAD(poseBase_.th)); // Из градусов в радианы далле подладить под своё представление +90 так как считал что ноль это ось У а не Х при отладке. надо переделывать
    tfBroadcaster.sendTransform(tfOdomBase);                                               // Публикация системы преобразования из odom в map Тут динамически, а статически выглядит так   <node pkg="tf" type="static_transform_publisher" name="static_map_odom_tf" args="0 0 0 0 0 0 map odom 100" /> <!--http://wiki.ros.org/tf-->
}

void CTopic::transformRotation() // Публикуем системы трансормаций из одних систем координат в другие
{
    // --------------------------------- base laser Для ЛИДАРА---------------------------------------
    geometry_msgs::TransformStamped tf;
    tf.header.stamp = ros_time;
    tf.header.frame_id = "base";
    tf.child_frame_id = "rotation";
    tf.transform.translation.x = transformLidar2Rotation.x; // минус 0.95 Задаем в начале в переменной// Данные для трасформации из Lidar в Rotation
    tf.transform.translation.y = transformLidar2Rotation.y;
    tf.transform.translation.z = -0.1;                                  // показываем по уровню пола
    tf.transform.rotation = tf::createQuaternionMsgFromYaw(DEG2RAD(0)); // Из градусов в радианы добавляем минус, так как в РОС вращение плюс против часовой, а у меня по часовой
    tfBroadcaster.sendTransform(tf);                                    // Публикация системы преобразования из odom в map Тут динамически, а статически выглядит так   <node pkg="tf" type="static_transform_publisher" name="static_map_odom_tf" args="0 0 0 0 0 0 map odom 100" /> <!--http://wiki.ros.org/tf-->
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


#endif