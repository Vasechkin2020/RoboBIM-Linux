
// #include "genStruct.h" // Тут все общие структуры. Истользуются и Data и Main и Head
// #include "scan_code/config.h"
// #include "scan_code/topic.h" // Файл для функций для формирования топиков в нужном виде и формате

#include "logi.h" //Класс для моего формата логов
AsyncFileLogger logi("/home/pi/RoboBIM-Linux/src/pb/log/", "scan_node");

// #include "scan_code/pillarScanNode.h"
#include "scan_code/pillarScan.h"

int main(int argc, char **argv)
{
    logi.log("***  scan_node *** ver. 1.01 *** printBIM.ru 29/11/25 *** 2025 ***\n");
    logi.log("------------------------------------------------------------------\n");

    ros::init(argc, argv, "scan_pillar_node");

    PillarScanNode node;
    node.init(); // Старт

    ros::Rate rate(20); // 20 Гц

    logi.log("Starting main loop...\n");

    while (ros::ok())
    {
        ros::spinOnce();
        node.process(); // Обработка каждого скана
        rate.sleep();
    }

    logi.log("Node finished execution gracefully.\n");
    return 0;

    printf("--- Scan_node STOP \n");
    return 0;
}
