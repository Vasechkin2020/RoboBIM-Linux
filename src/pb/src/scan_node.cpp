
#include "genStruct.h" // Тут все общие структуры. Истользуются и Data и Main и Head
// #include "scan_code/config.h"
// #include "scan_code/topic.h" // Файл для функций для формирования топиков в нужном виде и формате
#include "scan_code/code.h"

#include "logi.h" //Класс для моего формата логов
AsyncFileLogger logi("/home/pi/RoboBIM-Linux/src/pb/log/", "pose_node");

int main(int argc, char **argv)
{
    logi.log("***  scan_node *** ver. 1.01 *** printBIM.ru 29/11/25 *** 2025 ***\n");
    logi.log("------------------------------------------------------------------\n");

    ros::init(argc, argv, "scan_node");
    ros::NodeHandle nh;
    ros::Rate rate(100);        // Частота в Герцах - задержка
    // CTopic topic; // Экземпляр класса для всех публикуемых топиков

    // log4cxx::MDC::put("node", "|scan_node|"); 
    
    //----------------------------- ПОДПИСКИ НА ТОПИКИ -------
    // ros::Subscriber subscriber_Speed = nh.subscribe<pb_msgs::SSetSpeed>("pbData/Speed", 1000, callback_Speed);
	// ros::Publisher subscriber_Print = nh.advertise<pb_msgs::Struct_Data2Print>("pb/Data/Print", 3); // Это мы создаем публикатор и определяем название топика в рос

    ros::Duration(1).sleep(); // Подождем пока все обьявится и инициализируется внутри ROS
   
    ROS_WARN("End Setup. Start loop.\n");
    while (ros::ok())   
    {
        ros::spinOnce(); // Опрашиваем ядро ROS и по этой команде наши срабатывают колбеки. Нужно только для подписки на топики

        rate.sleep(); // Интеллектуальная задержка на указанную частоту
    }
    printf("--- Scan_node STOP \n");
    return 0;
}
