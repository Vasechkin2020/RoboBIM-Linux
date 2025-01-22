#ifndef CODE_H
#define CODE_H

// #include "pillar.h"
//**************************** ОБЬЯВЛЕНИЕ ПРОЦЕДУР **********************************
// void callback_Driver(pb_msgs::Struct_Data2Print msg); //

// pb_msgs::SControlDriver speedCorrect(pb_msgs::SDriver2Data Driver2Data_msg_, pb_msgs::SControlDriver Data2Driver_); // Корректировка скорости движения в зависимости от датчиков растояния перед
// void collectCommand(); // //Функция формирования команды для нижнего уровня на основе всех полученных данных, датчиков и анализа ситуации

// **********************************************************************************
// void callback_Driver(pb_msgs::Struct_Data2Print msg)
// {
// 	msg_Driver2Data = msg; // Пишнм в свою переменную пришедшее сообщение и потом его обрабатываем в основном цикле
// 	flag_msgPrint = true;
// }

// void callback_Speed(pb_msgs::SSetSpeed msg)
// {
// 	msg_Speed = msg; // Пишнм в свою переменную пришедшее сообщение и потом его обрабатываем в основном цикле
// 	flag_msgSpeed = true;
// }


// Заполнение маасива команд
void initCommandArray()
{
	commandArray[0].mode = 1;
	commandArray[0].status = 0;
	commandArray[0].position = 0.05;
	commandArray[0].velocity = 0.05;
	commandArray[0].torque = 0.05;
    commandArray[0].duration = 20000;

	commandArray[1].mode = 1;
	commandArray[1].status = 1;
	commandArray[1].position = 0.05;
	commandArray[1].velocity = 0.05;
	commandArray[1].torque = 0.05;
    commandArray[0].duration = 20000;
    
    commandArray[2].mode = 1;
	commandArray[2].status = 0;
	commandArray[2].position = 0.05;
	commandArray[2].velocity = 0.05;
	commandArray[2].torque = 0.05;
    commandArray[0].duration = 20000;

	commandArray[3].mode = 1;
	commandArray[3].status = 0;
	commandArray[3].position = 0.05;
	commandArray[3].velocity = 0.05;
	commandArray[3].torque = 0.05;
    commandArray[0].duration = 20000;
}


#endif
