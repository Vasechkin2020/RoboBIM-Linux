#// Структура отправляемых данных от Data к контроллеру Driver

	uint32 id                   # Номер команды по порядку
	Struct_Control control      # Структура управления машиной
	Struct_Servo servo1         # Управление сервомотором
	Struct_Servo servo2         # Управление сервомотором
	Struct_Led led              # Управление светодиодами
	uint32 cheksum              # Контрольная сумма данных в структуре