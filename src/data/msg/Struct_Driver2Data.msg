#// Структура принимаемых данных от контроллера Driver в Data

	uint32 id                       # id команды
	Struct_StatusDriver status
	Struct_Car car                  #
	Struct_Encoder motorLeft
	Struct_Encoder motorRight
	Struct_IMU bno055               # Данные с датчика BNO055
	Struct_Sensor lazer1
	Struct_Sensor lazer2
	Struct_Sensor uzi1
	uint32 cheksum                  # Контрольная сумма данных в структуре