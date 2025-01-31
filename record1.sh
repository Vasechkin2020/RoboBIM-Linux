#rostopic list
#chmod +x record1.sh
rosbag record /scan /pbData/Modul /pbControl/ControlDriver /pbData/Speed /pbData/Driver /pbData/Spi  -O go1-25Hz.bag --duration=120
rosbag record /scan /pbData/Modul -O src/pb/bag/lidarOnPlace2.bag --duration=10