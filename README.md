# RFID 125K EM4305 读写模块

## 硬件 TTL 连接线序
从 J1 引脚开始，分别为 GND、VCC、*RX* 和 *TX*，依次连接到 TTL 的 GND、VCC、*TX* 和 *RX* 。
![EM4305-Writer.jpg](https://raw.githubusercontent.com/alphameow/em4305/master/EM4305-Writer.jpg "EM4305 读写模块")

## 命令说明
*  em4305 -r

    读取 EM4305 ID 卡内容。执行命令后，将 ID 卡放在线圈上， 若读取正常则会显示 PAGE 1、4、5、6、7 和 8 的内容。

*  em4305 -1 number

    将十进制 number 写入 EM4305 ID 卡的PAGE 5 和 6 。

*  em4305 -1 number1 -2 number2

    将十进制 number1 写入 EM4305 ID 卡的PAGE 5 和 6 ， 将十进制 number2 写入 EM4305 ID 卡的PAGE 7 和 8 ，这样就可以将两个 ID 号写到同一个卡上。



