# Структура топика с моими данными получаемыми от Iot и пбуликуемые в топик

  uint32  id                      # id сообщения 
  Bme  bme                        # Данные с датчика BMP280
  Power ina_iot                   # Данные с датчика INA219 Iot
  DataTimeMy timeIot              # Дата время с датчика
  float32 gaz_data 	              # Данные с первого датчика газа
  float32 lux                     # Данные с дачика освещенности
  uint32 status_wifi_Iot          # Статус сети вайфай
  uint32 status_movement          # Статус Движения 
  string ssid_wifi                # Статус Движения 
  string password_wifi            # Статус Движения 
  uint32 obmen_all                # Число обменов данными
  uint32 obmen_bed                # Число обменов с ошибками

