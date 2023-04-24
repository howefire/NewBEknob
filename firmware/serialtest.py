import serial
import json
import time
# 打开串口
ser = serial.Serial('COM15', 921600)
# 发送 JSON 数据
data ={
	"mode":"angle",	
	"type":"set",
	"data":{
		"value":7.2647
	}
}
json_data = json.dumps(data).encode('utf-8')
print(json_data)
ser.write(json_data)

#接收 JSON 数据
while True:
    if ser.in_waiting:
        json_data = ser.readline().strip().decode('utf-8')
        #data = json.loads(json_data)
        print(json_data)
    # break
