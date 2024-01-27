#ifndef TOPIC_H
#define TOPIC_H

/*
 Класс для функций для формирования топиков в нужном виде и формате и всех публикаций
*/
#include <ros/ros.h>

class CTopic
{
private:
    //--------------------------------- ПУБЛИКАЦИЯ В ТОПИКИ -------------------------------------------------
    data::Struct_AngleLaser angleLaser_msg; // Перемеенная в которую сохраняем данные лидара из сообщения

    geometry_msgs::PoseStamped startPose_msg; // Начальная позиция отображаем в RVIZ
    data::Struct_PoseLidar poseLidarAll_msg;          // Обобщенные данные в моем формате о всех вариантах расчета позиции
    geometry_msgs::PoseStamped poseLidarMode1_msg; // Перемеенная в которую сохраняем данные лидара из сообщения
    geometry_msgs::PoseStamped poseLidarMode2_msg; // Перемеенная в которую сохраняем данные лидара из сообщения

    data::Struct_Joy joy2Head_msg;  // Структура в которую пишем обработанные данные от джойстика и потом публикуем в топик
    data::Struct_Joy joy2Head_prev; // Структура в которую пишем обработанные данные от джойстика предыдущее состоние

    data::PillarOut pillarAll_msg; // Перемеенная в которую пишем данные для опубликования в топик
    geometry_msgs::PointStamped pillar0_msg;
    geometry_msgs::PointStamped pillar1_msg;
    geometry_msgs::PointStamped pillar2_msg;
    geometry_msgs::PointStamped pillar3_msg;

    ros::NodeHandle _nh;
    ros::Publisher pub_PillarAll = _nh.advertise<data::PillarOut>("pbPillarAll", 16);        // Это мы публикуем итоговую информацию по столбам
    ros::Publisher pub_topicPillar0 = _nh.advertise<geometry_msgs::PointStamped>("pbPillar0", 16); // Для публикации конкретного столба
    ros::Publisher pub_topicPillar1 = _nh.advertise<geometry_msgs::PointStamped>("pbPillar1", 16); // Для публикации конкретного столба
    ros::Publisher pub_topicPillar2 = _nh.advertise<geometry_msgs::PointStamped>("pbPillar2", 16); // Для публикации конкретного столба
    ros::Publisher pub_topicPillar3 = _nh.advertise<geometry_msgs::PointStamped>("pbPillar3", 16); // Для публикации конкретного столба

    ros::Publisher pub_StartPose = _nh.advertise<geometry_msgs::PoseStamped>("pbStartPose", 16); // Это мы обьявляем структуру для публикации которую сформировали по данным с джойстика

    ros::Publisher pub_poseLidarAll = _nh.advertise<data::Struct_PoseLidar>("pbPoseLidarAll", 16);    // Это мы публикуем итоговую информацию по позици лидара обобщенную
    ros::Publisher pub_PoseLidarMode1 = _nh.advertise<geometry_msgs::PoseStamped>("pbPoseLidarMode1", 16); // Это мы публикуем итоговую информацию по позици лидара расчет по mode1
    ros::Publisher pub_PoseLidarMode2 = _nh.advertise<geometry_msgs::PoseStamped>("pbPoseLidarMode2", 16); // Это мы публикуем итоговую информацию по позици лидара расчет по mode1

    ros::Publisher pub_AngleLaser = _nh.advertise<data::Struct_AngleLaser>("pbAngleLaser", 16); // Это мы публикуем итоговую информацию по углам лазера для нижнего уровня

    ros::Publisher pub_Joy2Head = _nh.advertise<data::Struct_Joy>("Joy2Head", 16);             // Это мы публикуем структуру которую сформировали по данным с джойстика
    ros::Publisher pub_Head2Driver = _nh.advertise<data::Struct_Data2Driver>("Head2Data", 16); // Это мы публикуем структуру которую отправляем к исполнению на драйвер

public:

    CTopic(/* args */);
    ~CTopic();
    //**************************** ОБЬЯВЛЕНИЕ ПРОЦЕДУР **********************************
    void visualPillarAll();      // Формируем перемнную с собщением для публикации
    void visualPillarPoint(); // Формируем перемнную с собщением для публикации
    void visulStartPose();
    void visualPoseLidarAll();   // Формируем перемнную с собщением для публикации по позиции лидара
    void visualPoseLidarMode(); // Формируем перемнную с собщением для публикации
    void visualAngleLaser();  // Формируем перемнную с собщением для публикации по углам лазера
};

CTopic::CTopic(/* args */)
{
}

CTopic::~CTopic()
{
}

void CTopic::visulStartPose()
{
    startPose_msg.header.stamp = ros::Time::now(); // Время ROS
    startPose_msg.header.frame_id = "odom";
    startPose_msg.pose.position.x = msg_startPose2d.x;
    startPose_msg.pose.position.y = msg_startPose2d.y;
    float theta = DEG2RAD(-msg_startPose2d.theta + 90); // + 90 Так как у них оси расположены не так как я меня. У меня ноль вверх а у них вправо и вращение у них против часовой
    geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(theta);
    startPose_msg.pose.orientation = quat;
    ROS_INFO("Quaternion x =%.3f y =%.3f z =%.3f w =%.3f theta = %.3f", quat.x, quat.y, quat.z, quat.w, theta);

    pub_StartPose.publish(startPose_msg); // Публикация полученных данных
}
void CTopic::visualPillarPoint() // Готовим одиночные точки столбов для RVIZ  в системе координат "odom"
{
    ros::Time ros_time = ros::Time::now(); // Время ROS

    pillar0_msg.header.stamp = ros_time;
    pillar0_msg.header.frame_id = "odom";
    pillar0_msg.point.x = g_pillar.pillar[0].x_true;
    pillar0_msg.point.y = g_pillar.pillar[0].y_true;

    pillar1_msg.header.stamp = ros_time;
    pillar1_msg.header.frame_id = "odom";
    pillar1_msg.point.x = g_pillar.pillar[1].x_true;
    pillar1_msg.point.y = g_pillar.pillar[1].y_true;

    pillar2_msg.header.stamp = ros_time;
    pillar2_msg.header.frame_id = "odom";
    pillar2_msg.point.x = g_pillar.pillar[2].x_true;
    pillar2_msg.point.y = g_pillar.pillar[2].y_true;
    ;

    pillar3_msg.header.stamp = ros_time;
    pillar3_msg.header.frame_id = "odom";
    pillar3_msg.point.x = g_pillar.pillar[3].x_true;
    pillar3_msg.point.y = g_pillar.pillar[3].y_true;

    pub_topicPillar0.publish(pillar0_msg); // Публикация полученных данных
    pub_topicPillar1.publish(pillar1_msg); // Публикация полученных данных
    pub_topicPillar2.publish(pillar2_msg); // Публикация полученных данных
    pub_topicPillar3.publish(pillar3_msg); // Публикация полученных данных
}

void CTopic::visualPillarAll() // Формируем перемнную с собщением для публикации
{
    // ROS_INFO("!!! %i",pillar.countPillar);
    // pillar_out_msg.data[0].azimuth = 0;
    for (int i = 0; i < g_pillar.countPillar; i++)
    {
        pillarAll_msg.data[i].status = g_pillar.pillar[i].status;
        pillarAll_msg.data[i].azimuth = g_pillar.pillar[i].azimuth;
        pillarAll_msg.data[i].hypotenuse = g_pillar.pillar[i].hypotenuse;
        pillarAll_msg.data[i].distance = g_pillar.pillar[i].distance;
        pillarAll_msg.data[i].x_true = g_pillar.pillar[i].x_true;
        pillarAll_msg.data[i].y_true = g_pillar.pillar[i].y_true;
        pillarAll_msg.data[i].theta_true1 = g_pillar.pillar[i].theta_true1;
        pillarAll_msg.data[i].theta_true2 = g_pillar.pillar[i].theta_true2;
        pillarAll_msg.data[i].y_lidar = g_pillar.pillar[i].y_lidar;
        pillarAll_msg.data[i].x_lidar = g_pillar.pillar[i].x_lidar;
        // ROS_INFO("Status= %i azimuth= %.3f",pillar_out_msg.data[i].status,pillar_out_msg.data[i].azimuth);
    }
    pub_PillarAll.publish(pillarAll_msg); // Публикуем информацию по столбам

    // pointPillar0_msg.header.stamp = ros::Time::now();
    // pointPillar0_msg.header.frame_id = "odom";
    // pointPillar0_msg.point.x = g_pillar.pillar[0].y_lidar;
    // pointPillar0_msg.point.y = g_pillar.pillar[0].x_lidar;

    // pointPillarA_msg.header.stamp = ros::Time::now();
    // pointPillarA_msg.header.frame_id = "odom";
    // pointPillarA_msg.points.clear(); // Очищаем старые точки из массива
    // geometry_msgs::Point32 p[4];
    // for (int i = 0; i < g_pillar.countPillar; i++)
    // {
    //     p[i].x = g_pillar.pillar[0].x_lidar + 1;
    //     p[i].y = g_pillar.pillar[0].y_lidar + 1;
    //     pointPillarA_msg.points.push_back(p[i]);
    // }
}

void CTopic::visualAngleLaser() // Формируем перемнную с собщением для публикации
{
    for (int i = 0; i < 4; i++)
    {
        angleLaser_msg.laser[i] = laser.anglePillarInLaser[i];
        angleLaser_msg.lidar[i] = laser.anglePillarInLidar[i];
    }
    pub_AngleLaser.publish(angleLaser_msg); // Публикуем информацию по углам лазера
}

void CTopic::visualPoseLidarAll() // Формируем перемнную с собщением для публикации
{
    poseLidarAll_msg.mmode1.x = g_poseLidar.mode1.x;
    poseLidarAll_msg.mmode1.y = g_poseLidar.mode1.y;
    poseLidarAll_msg.mmode1.theta = g_poseLidar.mode1.theta;

    poseLidarAll_msg.mmode2.x = g_poseLidar.mode2.x;
    poseLidarAll_msg.mmode2.y = g_poseLidar.mode2.y;
    poseLidarAll_msg.mmode2.theta = g_poseLidar.mode2.theta;
    
    pub_poseLidarAll.publish(poseLidarAll_msg);           // Публикуем информацию по позиции лидара
}
void CTopic::visualPoseLidarMode() // Формируем перемнную с собщением для публикации
{
    //---------------------------------------------------------------------------------
    ros::Time ros_time = ros::Time::now(); // Время ROS
    
    poseLidarMode1_msg.header.stamp = ros_time;
    poseLidarMode1_msg.header.frame_id = "odom";
    poseLidarMode1_msg.pose.position.x = g_poseLidar.mode1.x;
    poseLidarMode1_msg.pose.position.y = g_poseLidar.mode1.y;
    poseLidarMode1_msg.pose.position.z = g_poseLidar.mode1.theta;
    poseLidarMode1_msg.pose.orientation = tf::createQuaternionMsgFromYaw(DEG2RAD(-g_poseLidar.mode1.theta + 90));// + 90 Так как у них оси расположены не так как я меня. У меня ноль вверх а у них вправо и вращение у них против часовой
  
    poseLidarMode2_msg.header.stamp = ros_time;
    poseLidarMode2_msg.header.frame_id = "odom";
    poseLidarMode2_msg.pose.position.x = g_poseLidar.mode2.x;
    poseLidarMode2_msg.pose.position.y = g_poseLidar.mode2.y;
    poseLidarMode2_msg.pose.position.z = g_poseLidar.mode2.theta;
    poseLidarMode2_msg.pose.orientation = tf::createQuaternionMsgFromYaw(DEG2RAD(-g_poseLidar.mode2.theta + 90));// + 90 Так как у них оси расположены не так как я меня. У меня ноль вверх а у них вправо и вращение у них против часовой

    //publisher_poseLidarMode1.publish(poseLidarMode1_msg); // Публикуем информацию по позиции лидара mode1
    //publisher_poseLidarMode2.publish(poseLidarMode2_msg); // Публикуем информацию по позиции лидара mode1
}

#endif