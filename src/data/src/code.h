#ifndef CODE_H
#define CODE_H

//Функция возращает максимальный размер из 2 структур
uint16_t getMax_size_Struct(uint16_t stru1_, uint16_t stru2_)
{
	uint16_t ret = 0;

	if (stru1_ > stru2_)
	{
		ret = stru1_;
	}
	else
	{
		ret = stru2_;
	}
	return ret += 1; // + 1 байт Для контрольной суммы
}

//Настройка светодиодов
void set_PIN_Led()
{
	//pinMode(PIN_LED_GREEN, OUTPUT); // Зеленый светодиод
	pinMode(PIN_LED_BLUE, OUTPUT);	// Красный светодиод
	//pullUpDnControl(PIN_LED_GREEN, PUD_DOWN);
	pullUpDnControl(PIN_LED_BLUE, PUD_DOWN);

	//digitalWrite(PIN_LED_GREEN, 1);
	digitalWrite(PIN_LED_BLUE, 1);
	delay(1000);
	//digitalWrite(PIN_LED_GREEN, 0);
	digitalWrite(PIN_LED_BLUE, 0);
}

//Функция мигания светодиодом в осномном цикле что программа не зависла и работает
void Led_Blink(int led_, unsigned long time_)
{
	static unsigned long led_time = 0;
	static bool led_status = 0;
	if ((millis() - led_time) > time_)
	{
		led_status = 1 - led_status;
		digitalWrite(led_, led_status);
		led_time = millis();
		ROS_INFO("%s led_time = %i.", NN, led_time);
	}
}

//Функция записи в нужные места данных одометрии в tf и в odom
void setOdomToTf(ros::NodeHandle nh_, tf::TransformBroadcaster odom_broadcaster_, ros::Time current_time_)
{
	//Пример кода взят отсюда
	// http://library.isr.ist.utl.pt/docs/roswiki/navigation(2f)Tutorials(2f)RobotSetup(2f)Odom.html

	// current_time_ = ros::Time::now(); // Получаем текущее время в ROS

	// //since all odometry is 6DOF we'll need a quaternion created from yaw
	// // получаем из моего направления куда смотрит робот кватернион
	// geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(Body.odom_enc.th);
	// //first, we'll publish the transform over tf
	// geometry_msgs::TransformStamped odom_trans;
	// odom_trans.header.stamp = current_time_;
	// odom_trans.header.frame_id = "odom";
	// odom_trans.child_frame_id = "base_link";

	// odom_trans.transform.translation.x = Body.odom_enc.x;
	// odom_trans.transform.translation.y = Body.odom_enc.x;
	// odom_trans.transform.translation.z = 0.0;
	// odom_trans.transform.rotation = odom_quat;

	// //send the transform
	// odom_broadcaster_.sendTransform(odom_trans);

	// //next, we'll publish the odometry message over ROS

	// odom.header.stamp = current_time_;
	// odom.header.frame_id = "odom";

	// //set the position
	// odom.pose.pose.position.x = Body.odom_enc.x;
	// odom.pose.pose.position.y = Body.odom_enc.y;
	// odom.pose.pose.position.z = 0.0;
	// odom.pose.pose.orientation = odom_quat;

	// //set the velocity
	// odom.child_frame_id = "base_link";
	// odom.twist.twist.linear.x = Body.odom_enc.vel_x;
	// odom.twist.twist.linear.y = Body.odom_enc.vel_y;
	// odom.twist.twist.angular.z = Body.odom_enc.vel_th;
}

//Инициализация канала шины SPI
void init_SPI(int channel_, int speed_)
{
	uint8_t errSpi = 0; //Ошибка при инициализации шины SPI
	ROS_INFO("%s Init SPI start... ", NN);

	if ((errSpi = wiringPiSPISetup(channel_, speed_)) < 0) //Инициализация канало 0 это чип селект 0
	{
		ROS_ERROR("%s Can't open the SPI bus 0: %s\n", NN, strerror(errno));
		ROS_ERROR("%s errSpi: %s\n", NN, errSpi);
		exit(EXIT_FAILURE);
	}
	else
	{
		ROS_INFO("%s SPI ok! \n", NN);
	}
}

// Обратный вызов при опросе топика Control
void message_callback_Command(const my_msgs::Command &msg)
{
	msg_head_receive = msg; // Копируем структуру в глобальную переменную для дальнейшей работы с ней.
	// ROS_INFO("message_callback_Command.");
}

void transOdom() // Перенес то что было на главном файле
{
	//     current_time = ros::Time::now(); // Получаем текущее время в ROS

	//     //since all odometry is 6DOF we'll need a quaternion created from yaw
	//     // получаем из моего направления куда смотрит робот кватернион
	//     geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(Body.odom_enc.th);
	//     //first, we'll publish the transform over tf
	//     geometry_msgs::TransformStamped odom_trans;
	//     odom_trans.header.stamp = current_time;
	//     odom_trans.header.frame_id = "odom";
	//     odom_trans.child_frame_id = "base_link";

	//     odom_trans.transform.translation.x = Body.odom_enc.x;
	//     odom_trans.transform.translation.y = Body.odom_enc.y;
	//     odom_trans.transform.translation.z = 0.0;
	//     odom_trans.transform.rotation = odom_quat;

	//     //send the transform
	//     odom_broadcaster.sendTransform(odom_trans);

	//     //next, we'll publish the odometry message over ROS

	//     odom.header.stamp = current_time;
	//     odom.header.frame_id = "odom";

	//     //set the position
	//     odom.pose.pose.position.x = Body.odom_enc.x;
	//     odom.pose.pose.position.y = Body.odom_enc.y;
	//     odom.pose.pose.position.z = 0.0;
	//     odom.pose.pose.orientation = odom_quat;

	//     //set the velocity
	//     odom.child_frame_id = "base_link";
	//     odom.twist.twist.linear.x = Body.odom_enc.vel_x;
	//     odom.twist.twist.linear.y = Body.odom_enc.vel_y;
	//     odom.twist.twist.angular.z = Body.odom_enc.vel_th;

	//     odom_pub.publish(odom); //publish the message
}

#endif