# MotorHigh.msg - 25x高速反馈数据
uint8 motor_id
uint16 can_id
float32 rpm          # 转速 (RPM)
float32 current      # 电流 (A)
int32 pulse          # 脉冲数