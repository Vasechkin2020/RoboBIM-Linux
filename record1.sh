#rostopic list
#chmod +x record1.sh
rosbag record /scan /pbData/Modul /pbControl/ControlDriver /pbData/Speed /pbData/Driver /pbData/Spi  -O go1-25Hz.bag --duration=120
rosbag record /scan /pbData/Modul /pbData/Speed /pbPas/ControlModul -O src/pb/bag/lidarOnPlace4.bag --duration=60
rosbag record /scan /pbData/Modul /pbData/Speed -O src/pb/bag/lidarOnPlace5.bag --duration=60