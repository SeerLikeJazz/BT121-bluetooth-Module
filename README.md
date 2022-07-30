## 系统  
从机：BT121-A Bluetooth Dual Mode Module + stm32F411CEU6  
主机：BT121-A + FT232RL  

### 备注
长时间传输，连接稳定性；验证  
是否丢包，丢包率；  
发送的策略，重发  
Trigger如何插入  
上位机如何修改  


### 22.07.30
- 从机DMA发送

### 22.07.27  
主机  
2001010100  reset  
200204010200  start server  
200a0400f4a96aab78cc00020111  dumo_cmd_bt_rfcomm_open  
从机  
发送速率保持70k字节/s，测试12小时以上  

