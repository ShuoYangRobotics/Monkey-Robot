# Monkey-Robot

A monkey robot designed for CMU course 24-775

## File Description

- GM6020_demo 	: The sample code to control GM6020 motor
- Doc			: The folder for all the documents
- MonkeyMatlab	: Simualtion for 3-Link Monkey

## DMA Read Strategy

DMA定长读取错位问题
每次产生dma接收中断，

1. 看长度够不够14（既然产生中断了肯定是够的）
2. 看首位是不是AA，不是的话往下找，直到找到AA或者到尾巴退出（记住找的指针）
3. 找到首位AA，看长度够不够14，不够退出
4. 找到首位AA且长度够14，check crc，不对的话头指针++退出
5. 如果checksum过了，读包，把头指针移动到包尾+1处（DMA将从这里开始写）
6. 以上的头指针移动完后，把头指针前的数据清空，把数据往前挪


## TODO

 - [x] Read through demo code and control two motor  position through PWM
 - [x] Add DMA Uart
 - [ ] Understand how SPI and CAN works

