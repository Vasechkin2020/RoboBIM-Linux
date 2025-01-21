  uint32 mode               # Текущий режим работы  0 - Выполняем команды по status 1- посылаем на CAN данные по position, velocity, torque
  uint32 status             # Текущий режим работы 0 - не печатем, 1 печатаем
  uint32 intensity          # ИНтенсивность печати. Сколько раз прыскаем на 1 мм
  float32 speed             #
  float32 position          #
  float32 velocity          #
  float32 torque            #