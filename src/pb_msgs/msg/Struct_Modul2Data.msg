std_msgs/Header header
uint32  id             # id сообщения пришедшее от Modul
SMotorSend[4] motor     # Структура мотора
SLaserSend[4] laser     # Структура дальномеров
uint32[4] micric       # Данные по концевикам
uint32 pinMotorEn      # Стутус пина управления драйвером моторов, включен драйвер или нет
uint32 statusDataLaser # Статус обновления данных с лазерных датчиков
