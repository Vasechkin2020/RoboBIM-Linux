#rostopic list
#chmod +x record1.sh
 rosbag record /scan /pbData/Modul /pbControl/ControlDriver /pbData/Speed /pbData/Driver /pbData/Spi  -O go1-25Hz.bag --duration=120