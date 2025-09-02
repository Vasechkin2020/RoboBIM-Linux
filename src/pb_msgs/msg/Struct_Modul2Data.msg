std_msgs/Header header
uint32  id             # id сообщения пришедшее от Modul
uint32 pinMotorEn      # Стутус пина управления драйвером моторов, включен драйвер или нет
SMotorSend[4] motor     # Структура мотора
SLaserSend[4] laser     # Структура дальномеров
uint32[4] micric       # Данные по концевикам
SMpu bno               # Данные с датчика MPU BNO
SMpu icm               # Данные с датчика MPU ICM
