# 串口连接状态：由 serial_bridge 以 1 Hz 发布
bool connected          # 当前串口是否已成功连接
string port             # 串口设备路径，如 /dev/ttyACM0
uint32 reconnect_count  # 自节点启动以来重连成功次数
