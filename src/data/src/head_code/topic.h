#ifndef TOPIC_H
#define TOPIC_H

/*
 Класс для функций для формирования топиков в нужном виде и формате и всех публикаций
*/
#include <ros/ros.h>

class CTopic
{
private:
    ros::NodeHandle _nh;
    tf::TransformBroadcaster tfBroadcaster; // Вещание данных преобразования систем координат
    //--------------------------------- ПУБЛИКАЦИЯ В ТОПИКИ -------------------------------------------------
    data::SAngleLaserLidar angleLLAll_msg;     // Перемеенная в которую сохраняем данные лидара из сообщения
    geometry_msgs::PoseStamped poseLaser0_msg; // Позиция лазера установленного на моторе 0
    geometry_msgs::PoseStamped poseLaser1_msg; // Позиция лазера установленного на моторе 1
    geometry_msgs::PoseStamped poseLaser2_msg; // Позиция лазера установленного на моторе 2
    geometry_msgs::PoseStamped poseLaser3_msg; // Позиция лазера установленного на моторе 3

    geometry_msgs::PoseStamped startPose_msg;      // Начальная позиция отображаем в RVIZ
    data::Struct_PoseLidar poseLidarAll_msg;       // Обобщенные данные в моем формате о всех вариантах расчета позиции
    geometry_msgs::PoseStamped poseLidarMode1_msg; // Перемеенная в которую сохраняем данные лидара из сообщения
    geometry_msgs::PoseStamped poseLidarMode2_msg; // Перемеенная в которую сохраняем данные лидара из сообщения

    data::Struct_Joy joy2Head_msg;  // Структура в которую пишем обработанные данные от джойстика и потом публикуем в топик
    data::Struct_Joy joy2Head_prev; // Структура в которую пишем обработанные данные от джойстика предыдущее состоние

    data::PillarOut pillarAll_msg; // Перемеенная в которую пишем данные для опубликования в топик
    geometry_msgs::PointStamped pillar0_msg;
    geometry_msgs::PointStamped pillar1_msg;
    geometry_msgs::PointStamped pillar2_msg;
    geometry_msgs::PointStamped pillar3_msg;

    ros::Publisher pub_PillarAll = _nh.advertise<data::PillarOut>("pbPillarAll", 16);              // Это мы публикуем итоговую информацию по столбам
    ros::Publisher pub_topicPillar0 = _nh.advertise<geometry_msgs::PointStamped>("pbPillar0", 16); // Для публикации конкретного столба
    ros::Publisher pub_topicPillar1 = _nh.advertise<geometry_msgs::PointStamped>("pbPillar1", 16); // Для публикации конкретного столба
    ros::Publisher pub_topicPillar2 = _nh.advertise<geometry_msgs::PointStamped>("pbPillar2", 16); // Для публикации конкретного столба
    ros::Publisher pub_topicPillar3 = _nh.advertise<geometry_msgs::PointStamped>("pbPillar3", 16); // Для публикации конкретного столба

    ros::Publisher pub_AngleLLAll = _nh.advertise<data::SAngleLaserLidar>("pbAngleLLAll", 16);     // Это мы публикуем итоговую информацию по углам лазера для нижнего уровня
    ros::Publisher pub_poseLaser0 = _nh.advertise<geometry_msgs::PoseStamped>("pbPoseLaser0", 16); // Публикатор для позиции лазера на моторе 0
    ros::Publisher pub_poseLaser1 = _nh.advertise<geometry_msgs::PoseStamped>("pbPoseLaser1", 16); // Публикатор для позиции лазера на моторе 1
    ros::Publisher pub_poseLaser2 = _nh.advertise<geometry_msgs::PoseStamped>("pbPoseLaser2", 16); // Публикатор для позиции лазера на моторе 2
    ros::Publisher pub_poseLaser3 = _nh.advertise<geometry_msgs::PoseStamped>("pbPoseLaser3", 16); // Публикатор для позиции лазера на моторе 3

    ros::Publisher pub_StartPose = _nh.advertise<geometry_msgs::PoseStamped>("pbStartPose", 16); // Это мы обьявляем структуру для публикации которую сформировали по данным с джойстика

    ros::Publisher pub_poseLidarAll = _nh.advertise<data::Struct_PoseLidar>("pbPoseLidarAll", 16);         // Это мы публикуем итоговую информацию по позици лидара обобщенную
    ros::Publisher pub_PoseLidarMode1 = _nh.advertise<geometry_msgs::PoseStamped>("pbPoseLidarMode1", 16); // Это мы публикуем итоговую информацию по позици лидара расчет по mode1
    ros::Publisher pub_PoseLidarMode2 = _nh.advertise<geometry_msgs::PoseStamped>("pbPoseLidarMode2", 16); // Это мы публикуем итоговую информацию по позици лидара расчет по mode1

    ros::Publisher pub_Joy2Head = _nh.advertise<data::Struct_Joy>("Joy2Head", 16);             // Это мы публикуем структуру которую сформировали по данным с джойстика
    ros::Publisher pub_Head2Driver = _nh.advertise<data::Struct_Data2Driver>("Head2Data", 16); // Это мы публикуем структуру которую отправляем к исполнению на драйвер
    ros::Time ros_time; // Время ROS
public:
    CTopic(/* args */);
    ~CTopic();
    //**************************** ОБЬЯВЛЕНИЕ ПРОЦЕДУР **********************************
    void visualPillarAll();   // Формируем перемнную с собщением для публикации
    void visualPillarPoint(); // Формируем перемнную с собщением для публикации
    void visulStartPose();
    void visualPoseLidarAll();   // Формируем перемнную с собщением для публикации по позиции лидара
    void visualPoseLidarMode();  // Формируем перемнную с собщением для публикации
    void visualAngleLaser();     // Формируем перемнную с собщением для публикации по углам лазера
    void visualPoseAngleLaser(); // Формируем перемнную с собщением для публикации по углам лазера
    void transform(); // Публикуем трансформации для системы координат
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
    ros_time = ros::Time::now(); // Время ROS

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
    data::pillar data;
    pillarAll_msg.data.clear(); // Очищаем старые точки из массива
    for (int i = 0; i < g_pillar.countPillar; i++)
    {
        data.status = g_pillar.pillar[i].status;
        data.azimuth = g_pillar.pillar[i].azimuth;
        data.hypotenuse = g_pillar.pillar[i].hypotenuse;
        data.distance = g_pillar.pillar[i].distance;
        data.x_true = g_pillar.pillar[i].x_true;
        data.y_true = g_pillar.pillar[i].y_true;
        data.theta_true1 = g_pillar.pillar[i].theta_true1;
        data.theta_true2 = g_pillar.pillar[i].theta_true2;
        data.y_lidar = g_pillar.pillar[i].y_lidar;
        data.x_lidar = g_pillar.pillar[i].x_lidar;
        // ROS_INFO("Status= %i azimuth= %.3f",pillar_out_msg.data[i].status,pillar_out_msg.data[i].azimuth);
        pillarAll_msg.data.push_back(data);
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
    data::SAngleLL data;
    angleLLAll_msg.data.clear(); // Очищаем старые точки из массива
    for (int i = 0; i < 4; i++)
    {
        data.angleFromLaser = laser.anglePillarInLaser[i];
        data.angleFromLidar = laser.anglePillarInLidar[i];
        angleLLAll_msg.data.push_back(data);
    }
    pub_AngleLLAll.publish(angleLLAll_msg); // Публикуем информацию по углам лазера
}

void CTopic::transform() // Публикуем системы трансормаций из одних систем координат в другие
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
    tfMapOdom.transform.rotation = tf::createQuaternionMsgFromYaw(DEG2RAD(0));; // Из градусов в радианы далле подладить под своё представление

    // --------------------------------- odom base ---------------------------------------
    tfBroadcaster.sendTransform(tfMapOdom); // Публикация системы преобразования из odom в map Тут динамически, а статически выглядит так   <node pkg="tf" type="static_transform_publisher" name="static_map_odom_tf" args="0 0 0 0 0 0 map odom 100" /> <!--http://wiki.ros.org/tf-->

    geometry_msgs::TransformStamped tfOdomBase;
    tfOdomBase.header.stamp = ros_time;
    tfOdomBase.header.frame_id = "odom";
    tfOdomBase.child_frame_id = "base";
    tfOdomBase.transform.translation.x = g_poseLidar.mode2.x;
    tfOdomBase.transform.translation.y = g_poseLidar.mode2.y;
    tfOdomBase.transform.translation.z = 0.0;
    tfOdomBase.transform.rotation = tf::createQuaternionMsgFromYaw(DEG2RAD(-g_poseLidar.mode2.theta));; // Из градусов в радианы далле подладить под своё представление
    tfBroadcaster.sendTransform(tfOdomBase); // Публикация системы преобразования из odom в map Тут динамически, а статически выглядит так   <node pkg="tf" type="static_transform_publisher" name="static_map_odom_tf" args="0 0 0 0 0 0 map odom 100" /> <!--http://wiki.ros.org/tf-->


    // --------------------------------- base laser Для ЛИДАРА---------------------------------------
    geometry_msgs::TransformStamped tfOdomLaser;
    tfOdomLaser.header.stamp = ros_time;
    tfOdomLaser.header.frame_id = "base";
    tfOdomLaser.child_frame_id = "laser";
    tfOdomLaser.transform.translation.x = 0;
    tfOdomLaser.transform.translation.y = 0;
    tfOdomLaser.transform.translation.z = 0.4;
    tfOdomLaser.transform.rotation = tf::createQuaternionMsgFromYaw(DEG2RAD(-0));; // Из градусов в радианы добавляем минус, так как в РОС вращение плюс против часовой, а у меня по часовой
    tfBroadcaster.sendTransform(tfOdomLaser); // Публикация системы преобразования из odom в map Тут динамически, а статически выглядит так   <node pkg="tf" type="static_transform_publisher" name="static_map_odom_tf" args="0 0 0 0 0 0 map odom 100" /> <!--http://wiki.ros.org/tf-->


    // --------------------------------- base laser0---------------------------------------
    geometry_msgs::TransformStamped tfOdomLaser0;
    tfOdomLaser0.header.stamp = ros_time;
    tfOdomLaser0.header.frame_id = "base";
    tfOdomLaser0.child_frame_id = "laser0";
    tfOdomLaser0.transform.translation.x = laser.poseLaser[0].x;
    tfOdomLaser0.transform.translation.y = laser.poseLaser[0].y;
    tfOdomLaser0.transform.translation.z = 0.3;
    tfOdomLaser0.transform.rotation = tf::createQuaternionMsgFromYaw(DEG2RAD(-laser.poseLaser[0].theta));; // Из градусов в радианы добавляем минус, так как в РОС вращение плюс против часовой, а у меня по часовой
    tfBroadcaster.sendTransform(tfOdomLaser0); // Публикация системы преобразования из odom в map Тут динамически, а статически выглядит так   <node pkg="tf" type="static_transform_publisher" name="static_map_odom_tf" args="0 0 0 0 0 0 map odom 100" /> <!--http://wiki.ros.org/tf-->
    // --------------------------------- base laser1---------------------------------------
    geometry_msgs::TransformStamped tfOdomLaser1;
    tfOdomLaser1.header.stamp = ros_time;
    tfOdomLaser1.header.frame_id = "base";
    tfOdomLaser1.child_frame_id = "laser1";
    tfOdomLaser1.transform.translation.x = laser.poseLaser[1].x;
    tfOdomLaser1.transform.translation.y = laser.poseLaser[1].y;
    tfOdomLaser1.transform.translation.z = 0.3;
    tfOdomLaser1.transform.rotation = tf::createQuaternionMsgFromYaw(DEG2RAD(-laser.poseLaser[1].theta));; // Из градусов в радианы добавляем минус, так как в РОС вращение плюс против часовой, а у меня по часовой
    tfBroadcaster.sendTransform(tfOdomLaser1); // Публикация системы преобразования из odom в map Тут динамически, а статически выглядит так   <node pkg="tf" type="static_transform_publisher" name="static_map_odom_tf" args="0 0 0 0 0 0 map odom 100" /> <!--http://wiki.ros.org/tf-->

    // --------------------------------- base laser2---------------------------------------
    geometry_msgs::TransformStamped tfOdomLaser2;
    tfOdomLaser2.header.stamp = ros_time;
    tfOdomLaser2.header.frame_id = "base";
    tfOdomLaser2.child_frame_id = "laser2";
    tfOdomLaser2.transform.translation.x = laser.poseLaser[2].x;
    tfOdomLaser2.transform.translation.y = laser.poseLaser[2].y;
    tfOdomLaser2.transform.translation.z = 0.3;
    tfOdomLaser2.transform.rotation = tf::createQuaternionMsgFromYaw(DEG2RAD(-laser.poseLaser[2].theta));; // Из градусов в радианы добавляем минус, так как в РОС вращение плюс против часовой, а у меня по часовой
    tfBroadcaster.sendTransform(tfOdomLaser2); // Публикация системы преобразования из odom в map Тут динамически, а статически выглядит так   <node pkg="tf" type="static_transform_publisher" name="static_map_odom_tf" args="0 0 0 0 0 0 map odom 100" /> <!--http://wiki.ros.org/tf-->
    // --------------------------------- base laser3---------------------------------------
    geometry_msgs::TransformStamped tfOdomLaser3;
    tfOdomLaser3.header.stamp = ros_time;
    tfOdomLaser3.header.frame_id = "base";
    tfOdomLaser3.child_frame_id = "laser3";
    tfOdomLaser3.transform.translation.x = laser.poseLaser[3].x;
    tfOdomLaser3.transform.translation.y = laser.poseLaser[3].y;
    tfOdomLaser3.transform.translation.z = 0.3;
    tfOdomLaser3.transform.rotation = tf::createQuaternionMsgFromYaw(DEG2RAD(-laser.poseLaser[3].theta));; // Из градусов в радианы добавляем минус, так как в РОС вращение плюс против часовой, а у меня по часовой
    tfBroadcaster.sendTransform(tfOdomLaser3); // Публикация системы преобразования из odom в map Тут динамически, а статически выглядит так   <node pkg="tf" type="static_transform_publisher" name="static_map_odom_tf" args="0 0 0 0 0 0 map odom 100" /> <!--http://wiki.ros.org/tf-->


}

void CTopic::visualPoseAngleLaser() // Формируем перемнную с собщением для публикации по углам лазера
{
    ros_time = ros::Time::now(); // Время ROS
    poseLaser0_msg.header.stamp = ros_time;
    poseLaser0_msg.header.frame_id = "laser0";
    poseLaser0_msg.pose.orientation = tf::createQuaternionMsgFromYaw(DEG2RAD(-laser.anglePillarInLaser[0] + 90)); // + 90 Так как у них оси расположены не так как я меня. У меня ноль вверх а у них вправо и вращение у них против часовой
    pub_poseLaser0.publish(poseLaser0_msg);                                                                       // Публикуем информацию по позиции луча лазераустановленного на моторе 0 в своей локальной ситеме координат laser0

    poseLaser1_msg.header.stamp = ros_time;
    poseLaser1_msg.header.frame_id = "laser1";
    poseLaser1_msg.pose.orientation = tf::createQuaternionMsgFromYaw(DEG2RAD(-laser.anglePillarInLaser[1] + 90)); // + 90 Так как у них оси расположены не так как я меня. У меня ноль вверх а у них вправо и вращение у них против часовой
    pub_poseLaser1.publish(poseLaser1_msg);                                                                       // Публикуем информацию по позиции луча лазераустановленного на моторе 0 в своей локальной ситеме координат laser0

    poseLaser2_msg.header.stamp = ros_time;
    poseLaser2_msg.header.frame_id = "laser2";
    poseLaser2_msg.pose.orientation = tf::createQuaternionMsgFromYaw(DEG2RAD(-laser.anglePillarInLaser[2] + 90)); // + 90 Так как у них оси расположены не так как я меня. У меня ноль вверх а у них вправо и вращение у них против часовой
    pub_poseLaser2.publish(poseLaser2_msg);                                                                       // Публикуем информацию по позиции луча лазераустановленного на моторе 0 в своей локальной ситеме координат laser0

    poseLaser3_msg.header.stamp = ros_time;
    poseLaser3_msg.header.frame_id = "laser3";
    poseLaser3_msg.pose.orientation = tf::createQuaternionMsgFromYaw(DEG2RAD(-laser.anglePillarInLaser[3] + 90)); // + 90 Так как у них оси расположены не так как я меня. У меня ноль вверх а у них вправо и вращение у них против часовой
    pub_poseLaser3.publish(poseLaser3_msg);                                                                       // Публикуем информацию по позиции луча лазераустановленного на моторе 0 в своей локальной ситеме координат laser0
}

void CTopic::visualPoseLidarAll() // Формируем перемнную с собщением для публикации
{
    poseLidarAll_msg.mmode1.x = g_poseLidar.mode1.x;
    poseLidarAll_msg.mmode1.y = g_poseLidar.mode1.y;
    poseLidarAll_msg.mmode1.theta = g_poseLidar.mode1.theta;

    poseLidarAll_msg.mmode2.x = g_poseLidar.mode2.x;
    poseLidarAll_msg.mmode2.y = g_poseLidar.mode2.y;
    poseLidarAll_msg.mmode2.theta = g_poseLidar.mode2.theta;

    pub_poseLidarAll.publish(poseLidarAll_msg); // Публикуем информацию по позиции лидара
}
void CTopic::visualPoseLidarMode() // Формируем перемнную с собщением для публикации
{
    //---------------------------------------------------------------------------------
    ros_time = ros::Time::now(); // Время ROS

    poseLidarMode1_msg.header.stamp = ros_time;
    poseLidarMode1_msg.header.frame_id = "odom";
    poseLidarMode1_msg.pose.position.x = g_poseLidar.mode1.x;
    poseLidarMode1_msg.pose.position.y = g_poseLidar.mode1.y;
    poseLidarMode1_msg.pose.orientation = tf::createQuaternionMsgFromYaw(DEG2RAD(-g_poseLidar.mode1.theta + 90)); // + 90 Так как у них оси расположены не так как я меня. У меня ноль вверх а у них вправо и вращение у них против часовой
    pub_PoseLidarMode1.publish(poseLidarMode1_msg);                                                               // Публикуем информацию по позиции лидара mode1

    poseLidarMode2_msg.header.stamp = ros_time;
    poseLidarMode2_msg.header.frame_id = "odom";
    poseLidarMode2_msg.pose.position.x = g_poseLidar.mode2.x;
    poseLidarMode2_msg.pose.position.y = g_poseLidar.mode2.y;
    poseLidarMode2_msg.pose.orientation = tf::createQuaternionMsgFromYaw(DEG2RAD(-g_poseLidar.mode2.theta + 90)); // + 90 Так как у них оси расположены не так как я меня. У меня ноль вверх а у них вправо и вращение у них против часовой
    pub_PoseLidarMode2.publish(poseLidarMode2_msg);                                                               // Публикуем информацию по позиции лидара mode1
}

#endif