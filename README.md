# raw_can
a can bus controller implied by cpu.<br>
Now is run on a freescale(NXP) K60 chip, system clock is 96Mhz.<br>
And the can bus can run at bitrate 20Kb.<br>
Becareful most CAN transceiver have a low bitrate limite, such as 20Kb.<br>
# use
It can be used to understand the principle of the CAN bus. <br>
It can be used in some applications that are cost sensitive and are not high for transmission rates, such as replacing some of the original RS485 based applications. <br>
It can be used as data transmission between multiple CPU on board, using open - out output, and not suitable for CAN transceiver. <br>
A custom protocol based on CAN protocol can be used to communicate, increasing the complexity of the system and improving the difficulty of solving the problem. <br>

# 软件CAN控制器
这是一个软件实现的CAN控制器。<br>
当前是运行在飞思卡尔K60处理器上，时钟96MHz。<br>
CAN速率运行在20Kb（理论上最高可以到40Kb）。<br>
需要注意的是，很多CAN收发器有一个最低速率的限制，比如tja1050限制最低速率为20Kb。<br>
# 用处
可以用来深入理解CAN总线的原理。<br>
可以用在一些对成本敏感，对传输速率要求不高的应用中，比如替换一些原来基于RS485的应用中。<br>
可以用作板上多CPU之间的数据传输，使用开漏输出，不适用CAN收发器。<br>
可以基于CAN协议自定义协议用于通讯，增加系统的复杂度，提高破解难度。<br>

