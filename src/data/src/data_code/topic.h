#ifndef TOPIC_H
#define TOPIC_H

/*
 Класс для функций для формирования топиков в нужном виде и формате и всех публикаций
*/
#include <ros/ros.h>

class CTopic
{
public:
    CTopic(/* args */);
    ~CTopic();
    //**************************** ОБЬЯВЛЕНИЕ ПРОЦЕДУР **********************************
    void transform(SPose pose_);          // Публикуем трансформации для системы координат
    void publicationControlDriver(data::SControlDriver data_); // Публикация данных разобранных из джойстика
    void processingSPI();										   // Сбор данных по результатам обмена по шине SPI по обоим контроллерам
    void visualEncoderMpu();
    void visualEncoderOdom();
    void processing_Driver2Data();
    void dataProcessing_Modul(); // Обработка полученных данных и копирование их для публикации в топике

private:
    ros::NodeHandle _nh;
    tf::TransformBroadcaster tfBroadcaster; // Вещание данных преобразования систем координат

    nav_msgs::Odometry odomEncoder_msg;
    nav_msgs::Odometry odomMpu_msg;

    data::SDriver2Data Driver2Data_msg; // Это структуры которые мы заполняем и потом публикуем
    data::Struct_Info_SPI spi_msg;      // Это структуры которые мы заполняем и потом публикуем

    data::Struct_ModulMotor modul_motor_msg;   // Это структуры сообщений которые мы заполняем и потом публикуем
    data::Struct_ModulLidar modul_lidar_msg;   // Это структуры которые мы заполняем и потом публикуем
    data::Struct_ModulMicric modul_micric_msg; // Это структуры которые мы заполняем и потом публикуем
    //--------------------------------- ПУБЛИКАЦИЯ В ТОПИКИ -------------------------------------------------

    ros::Publisher publish_Driver2Data = _nh.advertise<data::SDriver2Data>("pbData/Driver", 16);  // Это мы публикуем структуру которую получили с драйвера
    ros::Publisher publish_Spi = _nh.advertise<data::Struct_Info_SPI>("pbInfo/Spi", 16);          // Это мы создаем публикатор и определяем название топика в рос
    ros::Publisher publish_OdomEncoder = _nh.advertise<nav_msgs::Odometry>("pbOdom/Encoder", 16); // Это мы создаем публикатор и определяем название топика в рос
    ros::Publisher publish_OdomMpu = _nh.advertise<nav_msgs::Odometry>("pbOdom/Mpu", 16);         // Это мы создаем публикатор и определяем название топика в рос

    ros::Publisher publish_ModulMotor = _nh.advertise<data::Struct_ModulMotor>("modulMotor", 16);    // Это мы создаем публикатор и определяем название топика в рос
    ros::Publisher publish_ModulLidar = _nh.advertise<data::Struct_ModulLidar>("modulLidar", 16);    // Это мы создаем публикатор и определяем название топика в рос
    ros::Publisher publish_ModulMicric = _nh.advertise<data::Struct_ModulMicric>("modulMicric", 16); // Это мы создаем публикатор и определяем название топика в рос

    // ros::Publisher pub_JoyData = _nh.advertise<data::SJoy>("pbInfo/JoyData", 16);                       // Это мы публикуем структуру которую сформировали по данным с джойстика

    // ros::Publisher pub_encoderOdom = _nh.advertise<nav_msgs::Odometry>("pbinfo/encoderOdom", 16);
    // ros::Publisher pub_mpuOdom = _nh.advertise<nav_msgs::Odometry>("pbinfo/mpuOdom", 16);

    ros::Time ros_time; // Время ROS
};

CTopic::CTopic(/* args */)
{
}

CTopic::~CTopic()
{
}

void CTopic::transform(SPose pose_) // Публикуем системы трансормаций из одних систем координат в другие
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
    tfOdomBase.transform.translation.x = pose_.x;
    tfOdomBase.transform.translation.y = pose_.y;
    tfOdomBase.transform.translation.z = 0.1;
    tfOdomBase.transform.rotation = tf::createQuaternionMsgFromYaw(-pose_.th); // Из градусов в радианы далее подладить под своё представление
    tfBroadcaster.sendTransform(tfOdomBase);                                                 // Публикация системы преобразования из odom в map Тут динамически, а статически выглядит так   <node pkg="tf" type="static_transform_publisher" name="static_map_odom_tf" args="0 0 0 0 0 0 map odom 100" /> <!--http://wiki.ros.org/tf-->
}
// Публикация данных разобранных из джойстика
// void CTopic::publicationControlDriver(data::SControlDriver data_)
// {
//     pub_ControlDriver.publish(data_);
// }

void CTopic::visualEncoderOdom()
{
	odomEncoder_msg.header.stamp = ros::Time::now(); // Время ROS
	odomEncoder_msg.header.frame_id = "odom";		 // Поза в этом сообщении должна быть указана в системе координат, заданной header.frame_id.
	// set the position
	odomEncoder_msg.pose.pose.position.x = encoder.pose.x;
	odomEncoder_msg.pose.pose.position.y = encoder.pose.y;
	geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(-encoder.pose.th);
	odomEncoder_msg.pose.pose.orientation = quat;
	// set the velocity
	odomEncoder_msg.child_frame_id = "odom"; // Поворот в этом сообщении должен быть указан в системе координат, заданной child_frame_id
	odomEncoder_msg.twist.twist.linear.x = encoder.twist.vx;
	odomEncoder_msg.twist.twist.linear.y = encoder.twist.vy;
	odomEncoder_msg.twist.twist.angular.z = encoder.twist.vth;
    publish_OdomEncoder.publish(odomEncoder_msg); // Публикация полученных данных
}
void CTopic::visualEncoderMpu()
{
	odomMpu_msg.header.stamp = ros::Time::now(); // Время ROS
	odomMpu_msg.header.frame_id = "odom";		 // Поза в этом сообщении должна быть указана в системе координат, заданной header.frame_id.
	// set the position
	odomMpu_msg.pose.pose.position.x = mpu.pose.x;
	odomMpu_msg.pose.pose.position.y = mpu.pose.y;
	float theta = DEG2RAD(mpu.pose.th); //
	geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(theta);
	odomMpu_msg.pose.pose.orientation = quat;
	// set the velocity
	odomMpu_msg.child_frame_id = "odom"; // Поворот в этом сообщении должен быть указан в системе координат, заданной child_frame_id
	odomMpu_msg.twist.twist.linear.x = mpu.twist.vx;
	odomMpu_msg.twist.twist.linear.y = mpu.twist.vy;
	odomMpu_msg.twist.twist.angular.z = mpu.twist.vth;
}

// Сбор данных по результатам обмена по шине SPI по обоим контроллерам
void CTopic::processingSPI()
{
	spi_msg.ModulData.all = Modul2Data.spi.all; // Собираем для публикации данные о результатах обмена полученных из Modul о том как он принял по SPI данные отправленные Data
	spi_msg.ModulData.bed = Modul2Data.spi.bed;

	spi_msg.DataModul.all = data_modul_all; // Собираем для публикации данные о результатах обмена из Data о том как он принял по SPI данные отправленные Modul
	spi_msg.DataModul.bed = data_modul_bed;
	//-------------------------------------------------------------------------------------
	spi_msg.DriverData.all = Driver2Data.spi.all; // Собираем для публикации данные о результатах обмена полученных из Modul о том как он принял по SPI данные отправленные Data
	spi_msg.DriverData.bed = Driver2Data.spi.bed;

	spi_msg.DataDriver.all = data_driver_all; // Собираем для публикации данные о результатах обмена из Data о том как он принял по SPI данные отправленные Modul
	spi_msg.DataDriver.bed = data_driver_bed;
    publish_Spi.publish(spi_msg); // Публикация собранных данных по обмену по шине SPI
}

// Копирование полученных данных в структуру для публикации в топике
void CTopic::processing_Driver2Data()
{
	// Копируем полученные по SPI данные в сообщение которое потом опубликуем
	Driver2Data_msg.id = Driver2Data.id;

	Driver2Data_msg.mpu.status = Driver2Data.bno055.status;
	Driver2Data_msg.mpu.angleEuler.roll = Driver2Data.bno055.angleEuler.x;
	Driver2Data_msg.mpu.angleEuler.pitch = Driver2Data.bno055.angleEuler.y;
	Driver2Data_msg.mpu.angleEuler.yaw = Driver2Data.bno055.angleEuler.z;

	Driver2Data_msg.laserL.distance = Driver2Data.laserL.distance;
	Driver2Data_msg.laserL.status = Driver2Data.laserL.status;

	Driver2Data_msg.laserR.distance = Driver2Data.laserR.distance;
	Driver2Data_msg.laserR.status = Driver2Data.laserR.status;
	
	Driver2Data_msg.uzi.distance = Driver2Data.uzi.distance;
	Driver2Data_msg.uzi.status = Driver2Data.uzi.status;
}

// Обработка полученных данных и копирование их для публикации в топике
void CTopic::dataProcessing_Modul()
{
	//----------------------  msg_Modul_info_send ----------------------
	modul_motor_msg.id = Modul2Data.id;
	modul_lidar_msg.id = Modul2Data.id;
	modul_micric_msg.id = Modul2Data.id;

	modul_motor_msg.id = Modul2Data.pinMotorEn; // Стутус пина управления драйвером моторов, включен драйвер или нет
	for (int i = 0; i < 4; i++)
	{
		modul_motor_msg.motor[i].status = Modul2Data.motor[i].status;			//
		modul_motor_msg.motor[i].position = Modul2Data.motor[i].position;		//
		modul_motor_msg.motor[i].destination = Modul2Data.motor[i].destination; //

		modul_lidar_msg.lidar[i].status = Modul2Data.lidar[i].status;	  //
		modul_lidar_msg.lidar[i].distance = Modul2Data.lidar[i].distance; //
		modul_lidar_msg.lidar[i].angle = Modul2Data.lidar[i].angle;		  //

		modul_micric_msg.micric[i] = Modul2Data.micric[i]; // Состояние концевиков
	}
}


#endif