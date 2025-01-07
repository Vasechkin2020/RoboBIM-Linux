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
	void processingSPI();		   // Сбор данных по результатам обмена по шине SPI по обоим контроллерам
	void processing_Modul2Data();  // Обработка полученных данных и копирование их для публикации в топике
	void processing_Print2Data();  // Обработка полученных данных и копирование их для публикации в топике
	void processing_Driver2Data(); // Обработка полученных данных и копирование их для публикации в топике

	void processing_SetSpeed(SControl &control_); // Копирование данных скорости в структуру и публикация в топик

private:
	ros::NodeHandle _nh;
	//--------------------------------- ПУБЛИКАЦИЯ В ТОПИКИ -------------------------------------------------
	pb_msgs::Struct_Driver2Data Driver2Data_msg;														 // Это структуры которые мы заполняем и потом публикуем
	ros::Publisher publish_Driver2Data = _nh.advertise<pb_msgs::Struct_Driver2Data>("pbData/Driver", 3); // Это мы публикуем структуру которую получили с драйвера

	pb_msgs::Struct_Modul2Data Modul2Data_msg;														  // Это структуры которые мы заполняем и потом публикуем
	ros::Publisher publish_Modul2Data = _nh.advertise<pb_msgs::Struct_Modul2Data>("pbData/Modul", 3); // Это мы создаем публикатор и определяем название топика в рос

	pb_msgs::Struct_Info_SPI spi_msg;													   // Это структуры которые мы заполняем и потом публикуем
	ros::Publisher publish_Spi = _nh.advertise<pb_msgs::Struct_Info_SPI>("pbData/Spi", 3); // Это мы создаем публикатор и определяем название топика в рос

	pb_msgs::SSetSpeed setSpeed_msg;													 // Это структуры которые мы заполняем и потом публикуем
	ros::Publisher publish_Speed = _nh.advertise<pb_msgs::SSetSpeed>("pbData/Speed", 3); // Это мы создаем публикатор и определяем название топика в рос

	// ros::Publisher pub_JoyData = _nh.advertise<pb_msgs::SJoy>("pbInfo/JoyData", 16);                       // Это мы публикуем структуру которую сформировали по данным с джойстика

	ros::Time ros_time; // Время ROS
};

CTopic::CTopic(/* args */)
{
}

CTopic::~CTopic()
{
}
// Публикация данных разобранных из джойстика
// void CTopic::publicationControlDriver(pb_msgs::SControlDriver data_)
// {
//     pub_ControlDriver.publish(data_);
// }

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

// Копирование данных скорости в структуру и публикация в топик
void CTopic::processing_SetSpeed(SControl &control_)
{
	// Копируем полученные по SPI данные в сообщение которое потом опубликуем
	setSpeed_msg.header.stamp = ros::Time::now();
	setSpeed_msg.id = Data2Driver.id;
	setSpeed_msg.speedL = control_.speedL;
	setSpeed_msg.speedR = control_.speedR;
	publish_Speed.publish(setSpeed_msg); // Публикация полученных данных
}
// Копирование полученных данных в структуру для публикации в топике
void CTopic::processing_Driver2Data()
{
	// Копируем полученные по SPI данные в сообщение которое потом опубликуем
	Driver2Data_msg.header.stamp = ros::Time::now();

	Driver2Data_msg.id = Driver2Data.id;

	Driver2Data_msg.motor.status = Driver2Data.motor.status;
	Driver2Data_msg.motor.rpsEncodL = Driver2Data.motor.rpsEncodL;
	Driver2Data_msg.motor.rpsEncodR = Driver2Data.motor.rpsEncodR;

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
	
	Modul2Data_msg.mpu.status = Modul2Data.bno055.status;
	Modul2Data_msg.mpu.rate = Modul2Data.bno055.rate;
	
	Modul2Data_msg.mpu.angleEuler.roll = Modul2Data.bno055.angleEuler.x;
	Modul2Data_msg.mpu.angleEuler.pitch = Modul2Data.bno055.angleEuler.y;
	Modul2Data_msg.mpu.angleEuler.yaw = Modul2Data.bno055.angleEuler.z;
	
	Modul2Data_msg.mpu.linear.x = Modul2Data.bno055.linear.x;
	Modul2Data_msg.mpu.linear.y = Modul2Data.bno055.linear.y;
	Modul2Data_msg.mpu.linear.z = Modul2Data.bno055.linear.z;

	for (int i = 0; i < 4; i++)
	{
		Modul2Data_msg.motor[i].status = Modul2Data.motor[i].status;		   //
		Modul2Data_msg.motor[i].position = Modul2Data.motor[i].position;	   //
		Modul2Data_msg.motor[i].destination = Modul2Data.motor[i].destination; //

		Modul2Data_msg.laser[i].status = Modul2Data.laser[i].status;					  //
		Modul2Data_msg.laser[i].distance = Modul2Data.laser[i].distance + offSetLaser[i]; // Прибавляем поправочные значения полученные при калибровке
		Modul2Data_msg.laser[i].signalQuality = Modul2Data.laser[i].signalQuality;		  //
		Modul2Data_msg.laser[i].angle = Modul2Data.laser[i].angle;						  //
		Modul2Data_msg.laser[i].time = Modul2Data.laser[i].time;						  //
		Modul2Data_msg.laser[i].numPillar = Modul2Data.laser[i].numPillar;				  //
		Modul2Data_msg.laser[i].rate = Modul2Data.laser[i].rate;						  //

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