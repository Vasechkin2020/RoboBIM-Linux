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
	void transform(); // Публикуем трансформации для системы координат
	void transformWheel(SPose pose_);
	void transformUnited(SPose pose_);
	void transformMpu(SPose pose_);

	void publishOdomMpu();
	void publishOdomWheel();
	void publishOdomUnited();

	void processingSPI();		   // Сбор данных по результатам обмена по шине SPI по обоим контроллерам
	void processing_Modul2Data();  // Обработка полученных данных и копирование их для публикации в топике
	void processing_Print2Data();  // Обработка полученных данных и копирование их для публикации в топике
	void processing_Driver2Data(); // Обработка полученных данных и копирование их для публикации в топике

private:
	ros::NodeHandle _nh;
	tf::TransformBroadcaster tfBroadcaster; // Вещание данных преобразования систем координат

	//--------------------------------- ПУБЛИКАЦИЯ В ТОПИКИ -------------------------------------------------
	pb_msgs::Struct_Driver2Data Driver2Data_msg;														  // Это структуры которые мы заполняем и потом публикуем
	ros::Publisher publish_Driver2Data = _nh.advertise<pb_msgs::Struct_Driver2Data>("pbData/Driver", 3); // Это мы публикуем структуру которую получили с драйвера

	pb_msgs::Struct_Modul2Data Modul2Data_msg;														   // Это структуры которые мы заполняем и потом публикуем
	ros::Publisher publish_Modul2Data = _nh.advertise<pb_msgs::Struct_Modul2Data>("pbData/Modul", 3); // Это мы создаем публикатор и определяем название топика в рос

	pb_msgs::Struct_Info_SPI spi_msg;														// Это структуры которые мы заполняем и потом публикуем
	ros::Publisher publish_Spi = _nh.advertise<pb_msgs::Struct_Info_SPI>("pbData/Spi", 3); // Это мы создаем публикатор и определяем название топика в рос

	nav_msgs::Odometry odomWheel_msg;
	ros::Publisher publish_OdomWheel = _nh.advertise<nav_msgs::Odometry>("pbData/odom/Wheel", 3); // Это мы создаем публикатор и определяем название топика в рос
	nav_msgs::Odometry odomUnited_msg;
	ros::Publisher publish_OdomUnited = _nh.advertise<nav_msgs::Odometry>("pbData/odom/United", 3); // Это мы создаем публикатор и определяем название топика в рос
	nav_msgs::Odometry odomMpu_msg;
	ros::Publisher publish_OdomMpu = _nh.advertise<nav_msgs::Odometry>("pbData/odom/Mpu", 3); // Это мы создаем публикатор и определяем название топика в рос

	// ros::Publisher pub_JoyData = _nh.advertise<pb_msgs::SJoy>("pbInfo/JoyData", 16);                       // Это мы публикуем структуру которую сформировали по данным с джойстика

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
	tfMapOdom.transform.rotation = tf::createQuaternionMsgFromYaw(DEG2RAD(0)); // Из градусов в радианы далле подладить под своё представление
	tfBroadcaster.sendTransform(tfMapOdom);									   // Публикация системы преобразования из odom в map Тут динамически, а статически выглядит так   <node pkg="tf" type="static_transform_publisher" name="static_map_odom_tf" args="0 0 0 0 0 0 map odom 100" /> <!--http://wiki.ros.org/tf-->
}
void CTopic::transformWheel(SPose pose_) // Публикуем системы трансормаций из одних систем координат в другие
{
	geometry_msgs::TransformStamped tfOdomWheel;
	tfOdomWheel.header.stamp = ros_time;
	tfOdomWheel.header.frame_id = "odom";
	tfOdomWheel.child_frame_id = "wheel";
	tfOdomWheel.transform.translation.x = pose_.x;
	tfOdomWheel.transform.translation.y = pose_.y;
	tfOdomWheel.transform.translation.z = 0.1;
	tfOdomWheel.transform.rotation = tf::createQuaternionMsgFromYaw(-pose_.th); // Из градусов в радианы далее подладить под своё представление
	tfBroadcaster.sendTransform(tfOdomWheel);									// Публикация системы преобразования из odom в map Тут динамически, а статически выглядит так   <node pkg="tf" type="static_transform_publisher" name="static_map_odom_tf" args="0 0 0 0 0 0 map odom 100" /> <!--http://wiki.ros.org/tf-->
}
void CTopic::transformUnited(SPose pose_) // Публикуем системы трансормаций из одних систем координат в другие
{
	geometry_msgs::TransformStamped tfOdomUnited;
	tfOdomUnited.header.stamp = ros_time;
	tfOdomUnited.header.frame_id = "odom";
	tfOdomUnited.child_frame_id = "united";
	tfOdomUnited.transform.translation.x = pose_.x;
	tfOdomUnited.transform.translation.y = pose_.y;
	tfOdomUnited.transform.translation.z = 0.1;
	tfOdomUnited.transform.rotation = tf::createQuaternionMsgFromYaw(-pose_.th); // Из градусов в радианы далее подладить под своё представление
	tfBroadcaster.sendTransform(tfOdomUnited);									 // Публикация системы преобразования из odom в map Тут динамически, а статически выглядит так   <node pkg="tf" type="static_transform_publisher" name="static_map_odom_tf" args="0 0 0 0 0 0 map odom 100" /> <!--http://wiki.ros.org/tf-->
}
void CTopic::transformMpu(SPose pose_) // Публикуем системы трансормаций из одних систем координат в другие
{
	geometry_msgs::TransformStamped tfOdomMpu;
	tfOdomMpu.header.stamp = ros_time;
	tfOdomMpu.header.frame_id = "odom";
	tfOdomMpu.child_frame_id = "mpu";
	tfOdomMpu.transform.translation.x = pose_.x;
	tfOdomMpu.transform.translation.y = pose_.y;
	tfOdomMpu.transform.translation.z = 0.2;
	tfOdomMpu.transform.rotation = tf::createQuaternionMsgFromYaw(-pose_.th); // Из градусов в радианы далее подладить под своё представление
	tfBroadcaster.sendTransform(tfOdomMpu);									  // Публикация системы преобразования из odom в map Тут динамически, а статически выглядит так   <node pkg="tf" type="static_transform_publisher" name="static_map_odom_tf" args="0 0 0 0 0 0 map odom 100" /> <!--http://wiki.ros.org/tf-->
}
// Публикация данных разобранных из джойстика
// void CTopic::publicationControlDriver(pb_msgs::SControlDriver data_)
// {
//     pub_ControlDriver.publish(data_);
// }

void CTopic::publishOdomWheel()
{
	transformWheel(odomWheel.pose);				   // Публиация системы трансформации
	odomWheel_msg.header.stamp = ros::Time::now(); // Время ROS
	odomWheel_msg.header.frame_id = "odom";		   // Поза в этом сообщении должна быть указана в системе координат, заданной header.frame_id.
	// set the position
	odomWheel_msg.pose.pose.position.x = odomWheel.pose.x;
	odomWheel_msg.pose.pose.position.y = odomWheel.pose.y;
	geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(-odomWheel.pose.th);
	odomWheel_msg.pose.pose.orientation = quat;
	// set the velocity
	odomWheel_msg.child_frame_id = "odom"; // Поворот в этом сообщении должен быть указан в системе координат, заданной child_frame_id
	odomWheel_msg.twist.twist.linear.x = odomWheel.twist.vx;
	odomWheel_msg.twist.twist.linear.y = odomWheel.twist.vy;
	odomWheel_msg.twist.twist.angular.z = odomWheel.twist.vth;
	publish_OdomWheel.publish(odomWheel_msg); // Публикация полученных данных
}
void CTopic::publishOdomUnited()
{
	transformUnited(odomUnited.pose);				// Публиация системы трансформации
	odomUnited_msg.header.stamp = ros::Time::now(); // Время ROS
	odomUnited_msg.header.frame_id = "odom";		// Поза в этом сообщении должна быть указана в системе координат, заданной header.frame_id.
	// set the position
	odomUnited_msg.pose.pose.position.x = odomUnited.pose.x;
	odomUnited_msg.pose.pose.position.y = odomUnited.pose.y;
	geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(-odomUnited.pose.th);
	odomUnited_msg.pose.pose.orientation = quat;
	// set the velocity
	odomUnited_msg.child_frame_id = "odom"; // Поворот в этом сообщении должен быть указан в системе координат, заданной child_frame_id
	odomUnited_msg.twist.twist.linear.x = odomUnited.twist.vx;
	odomUnited_msg.twist.twist.linear.y = odomUnited.twist.vy;
	odomUnited_msg.twist.twist.angular.z = odomUnited.twist.vth;
	publish_OdomUnited.publish(odomUnited_msg); // Публикация полученных данных
}
void CTopic::publishOdomMpu()
{
	transformMpu(odomMpu.pose);
	odomMpu_msg.header.stamp = ros::Time::now(); // Время ROS
	odomMpu_msg.header.frame_id = "odom";		 // Поза в этом сообщении должна быть указана в системе координат, заданной header.frame_id.
	// set the position
	odomMpu_msg.pose.pose.position.x = odomMpu.pose.x;
	odomMpu_msg.pose.pose.position.y = odomMpu.pose.y;
	geometry_msgs::Quaternion quat = tf::createQuaternionMsgFromYaw(odomMpu.pose.th);
	odomMpu_msg.pose.pose.orientation = quat;
	// set the velocity
	odomMpu_msg.child_frame_id = "odom"; // Поворот в этом сообщении должен быть указан в системе координат, заданной child_frame_id
	odomMpu_msg.twist.twist.linear.x = odomMpu.twist.vx;
	odomMpu_msg.twist.twist.linear.y = odomMpu.twist.vy;
	odomMpu_msg.twist.twist.angular.z = odomMpu.twist.vth;
	publish_OdomMpu.publish(odomMpu_msg); // Публикация полученных данных
}

// Сбор данных по результатам обмена по шине SPI по обоим контроллерам
void CTopic::processingSPI()
{
	spi_msg.header.stamp = ros::Time::now();

	spi_msg.ModulData.all = Modul2Data.spi.all; // Собираем для публикации данные о результатах обмена полученных из Modul о том как он принял по SPI данные отправленные Data
	spi_msg.ModulData.bed = Modul2Data.spi.bed;

	spi_msg.DataModul.all = data_modul_all; // Собираем для публикации данные о результатах обмена из Data о том как он принял по SPI данные отправленные Modul
	spi_msg.DataModul.bed = data_modul_bed;
	//-------------------------------------------------------------------------------------
	spi_msg.DriverData.all = Driver2Data.spi.all; // Собираем для публикации данные о результатах обмена полученных из Modul о том как он принял по SPI данные отправленные Data
	spi_msg.DriverData.bed = Driver2Data.spi.bed;

	spi_msg.DataDriver.all = data_driver_all; // Собираем для публикации данные о результатах обмена из Data о том как он принял по SPI данные отправленные Modul
	spi_msg.DataDriver.bed = data_driver_bed;
	//-------------------------------------------------------------------------------------
	spi_msg.PrintData.all = Print2Data.spi.all; // Собираем для публикации данные о результатах обмена полученных из Modul о том как он принял по SPI данные отправленные Data
	spi_msg.PrintData.bed = Print2Data.spi.bed;

	spi_msg.DataPrint.all = data_print_all; // Собираем для публикации данные о результатах обмена из Data о том как он принял по SPI данные отправленные Modul
	spi_msg.DataPrint.bed = data_print_bed;
	//-------------------------------------------------------------------------------------

	publish_Spi.publish(spi_msg); // Публикация собранных данных по обмену по шине SPI
}

// Копирование полученных данных в структуру для публикации в топике
void CTopic::processing_Driver2Data()
{
	// Копируем полученные по SPI данные в сообщение которое потом опубликуем
	Driver2Data_msg.header.stamp = ros::Time::now();

	Driver2Data_msg.id = Driver2Data.id;

	Driver2Data_msg.motor.status = Driver2Data.motor.status;
	Driver2Data_msg.motor.rpsEncodL = Driver2Data.motor.rpsEncodL;
	Driver2Data_msg.motor.rpsEncodL = Driver2Data.motor.rpsEncodR;

	Driver2Data_msg.mpu.status = Driver2Data.bno055.status;
	Driver2Data_msg.mpu.angleEuler.roll = Driver2Data.bno055.angleEuler.x;
	Driver2Data_msg.mpu.angleEuler.pitch = Driver2Data.bno055.angleEuler.y;
	Driver2Data_msg.mpu.angleEuler.yaw = Driver2Data.bno055.angleEuler.z;

	Driver2Data_msg.laserL.status = Driver2Data.laserL.status;
	Driver2Data_msg.laserL.distance = Driver2Data.laserL.distance;

	Driver2Data_msg.laserR.status = Driver2Data.laserR.status;
	Driver2Data_msg.laserR.distance = Driver2Data.laserR.distance;

	Driver2Data_msg.uzi.status = Driver2Data.uzi.status;
	Driver2Data_msg.uzi.distance = Driver2Data.uzi.distance;

	publish_Driver2Data.publish(Driver2Data_msg); // Публикация полученных данных
}

// Обработка полученных данных и копирование их для публикации в топике
void CTopic::processing_Modul2Data()
{
	Modul2Data_msg.header.stamp = ros::Time::now();
	Modul2Data_msg.id = Modul2Data.id;
	Modul2Data_msg.pinMotorEn = Modul2Data.pinMotorEn; // Стутус пина управления драйвером моторов, включен драйвер или нет
	Modul2Data_msg.statusDataLaser = Modul2Data.statusDataLaser;

	for (int i = 0; i < 4; i++)
	{
		Modul2Data_msg.motor[i].status = Modul2Data.motor[i].status;		   //
		Modul2Data_msg.motor[i].position = Modul2Data.motor[i].position;	   //
		Modul2Data_msg.motor[i].destination = Modul2Data.motor[i].destination; //

		Modul2Data_msg.laser[i].status = Modul2Data.laser[i].status;	 //
		Modul2Data_msg.laser[i].distance = Modul2Data.laser[i].distance; //
		Modul2Data_msg.laser[i].signalQuality = Modul2Data.laser[i].signalQuality; //
		Modul2Data_msg.laser[i].angle = Modul2Data.laser[i].angle;		 //
		Modul2Data_msg.laser[i].numPillar = Modul2Data.laser[i].numPillar;		 //

		Modul2Data_msg.micric[i] = Modul2Data.micric[i]; // Состояние концевиков
	}
	publish_Modul2Data.publish(Modul2Data_msg); // Публикация полученных данных
}

// Обработка полученных данных и копирование их для публикации в топике
void CTopic::processing_Print2Data()
{
	// Нечего публиковать SPI публикую отдельно
}

#endif