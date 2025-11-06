SMwu vx          # Скорости линейная по х всех типов
SMwu vth         # Скорости угловая всех типов
SMwu dt          # Время между измерениями
SMwu track       # Пройденное расстояние
SMwu yaw         # Угол поворота, куда смотрим

float32 offsetYaw    # 
float32 offsetX    # 

float32 raw_accel  # Сырые данные
float32 raw_gyro  # Сырые данные
float32 real_accel  # Данные на которых фактически считаем
float32 real_gyro  # Данные на которых фактически считаем

float32 a_lin_X    # после слияния

float32 roll    # 
float32 pitch    # 

float32 complX     # офсет во время стоянки
float32 complYaw    # после слияния
float32 linRaw      # Сырые данные
float32 flagAccel     # даные после применения bias
float32 g_x    # после слияния
float32 a_lin_odom    # после слияния
float32 fused_accel    # после слияния
float32 odomVth    # после слияния
float32 fused_yaw    # после слияния
