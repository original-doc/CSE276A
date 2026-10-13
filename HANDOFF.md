# 交接文档(CSE 276A HW1)

> 交接时间:2026-10-13 · 作业截止:**2026-10-16(周五,剩 3 天)**
> 机器人:Rubik Pi 3(Ubuntu 24.04.5 + ROS 2 Jazzy,Python 3.12.3)

## 1. 仓库状态(已核实,截至 2026-10-13)

| 分支 | 最新 commit | 内容 |
|---|---|---|
| `main` | `3747767` | 队友的标定框架 + 地毯地面标定数据 + HW1 材料 |
| `dev-yuanshengqi` | `8c184ac` | 我的传感器实验文档、脚本、数据、参数 CSV(本文件也在此分支) |

**main 上队友的成果**:
- `robot_control/robot_control/calibrate.py` — 标定 CLI(imu_diag / straight / arc / rotate / deadband 五种测试),输出 `config/floors/<floor>.yaml` + `config/diag/*.csv`
- `robot_control/robot_control/heading.py` — 陀螺仪航向 + **ZUPT**(零速修正),已放弃板子 yaw(和我们结论一致)
- `robot_control/robot_control/kinematics.py` — 运动学模型,参数:linear_scale / trim / track_eff(0.172)/ min_cmd / rot_cmd(0.3)/ rot_rate_ccw / rot_rate_cw / rot_coast_* / rot_deadband / gyro_scale
- `robot_control/test/test_heading.py`、`test_kinematics.py` — 单元测试
- `HW1/imu_diag.md` — 标定操作教程(tmux 双窗格:motor_control + calibrate)
- `HW1/waypoints.txt`、`hw1-2026.pdf` — 作业数据
- 地毯地面已标定(`config/floors/carpet.yaml`),**木地板参数还是默认值 0——没填**

**dev-yuanshengqi 上我的成果**(`calibration/` 目录):
- `README.md` — 完整实验链文档(设计逻辑 + 结果 + 代码函数说明)
- `scripts/` — 6 个实验脚本(mag_cal_sample / mag_cal_fit / turn_analyze / gyro_bias_test / gyro_turn_test2 / motor_cal2)
- `data/` — 8 组原始数据(3 圈标定、4 次半圈、快转组合、电机实转)
- `calibration_data.csv` — **参数卡数据**(队友也复制了一份到 main 的 `HW1/wooden_floor_calibration_data.csv`)

## 2. 我的传感器结论(全部有数据支撑)

1. **板子 yaw 不可用**:电机运行时被磁干扰;快转后阻尼融合跟不上(90° 只报 15°);慢转 360° 能跟上(+394°)——队友已转向陀螺仪方案,方向正确
2. **自标定磁力计航向**(公式见下):静止时 ±5~8° 准,无弹回;但**电机停转后必须等 2 秒**才能读(剩磁恢复,0.5 秒时读数连方向都错)
3. **转向参数(木地板自习室)**:死区 ~0.2;ω(0.3) 逆 73°/s、顺 95°/s;ω(0.4) 逆 177°/s、顺 188°/s;脉冲响应表(0.1~0.8s × 2 档 × 2 方向,24 个数据点)在 CSV 里
4. **gz 积分**:转弯 ≥1 秒时 ±10° 准;0.5 秒快转失效(板子有效采样率只有 4.5Hz)
5. 标定参数:`cx=-14.5 cy=-15.0 sx=20.5 sy=22.0`,`heading = -atan2((my-cy)/sy, (mx-cx)/sx)`(逆时针为正);**环境相关,换房间需重标定**(水平转一圈 20 秒)
6. 90° 验证三轮(数据量少,仅参考):误差 +24.5/-7.5/-20°;180° 四轮:2~12°;360° 三轮:1.6~15.5°

## 3. 待办清单(按优先级)

1. **【最急】waypoint 跟随跑通 + 录视频 + 报告**(周五截止)——队友框架已就绪,缺:把木地板参数填进 kinematics(用我方数据:rot_rate_ccw=73/rot_rate_cw=95 @0.3 档、rot_deadband≈0.2、gyro_scale 待队友的 imu_diag 拟合)、写 waypoint follower 节点
2. **木地板标定补全**:队友的 calibrate.py 有 straight/arc 测试(需卷尺量距离)——我们没卷尺所以 E4 直线速度一直欠着;有卷尺后直接跑队友的工具
3. **实验 G(直线跑偏率 + 单侧 pivot)** 被 WiFi 中断未完成(不需要卷尺,2 分钟)
4. **机器人反复重启之谜**(未解决):充电器插着仍重启(10-11 日共 7+ 次),多发生在转动/搬动机器人之后——首要嫌疑是充电线/电池接头在机身转动时被拉扯。**下次实验盯着电源灯看有没有闪灭**
5. 本地 Mac 的 ROS 容器环境:OrbStack 已卸载,Docker Desktop 安装未完成(需 sudo 的步骤没跑)——如需本地容器,重装 Docker Desktop 后按 `~/Desktop/cse276a/team-env/docker/Dockerfile` 重建

## 4. 机器人操作要点(重要,新 agent 必读)

**访问方式**:
- `ssh rubikpi`(mDNS,推荐)/ `ssh rubikpi-ip`(直连 IP,DHCP 会变);密码登录已配公钥
- 串口救援脚本:`~/Desktop/cse276a/scripts/serial_console.py`(登录+执行命令)、`grub_boot.py`(硬断电后过 GRUB 菜单);串口设备 `/dev/cu.usbmodem58C60621731`,115200

**网络**:
- WiFi 配置:宿舍 MySpectrumWiFiC7-2G(优先 100)+ 手机热点 Kissshot(优先 100,密码已存);队友的 MySpectrumWiFi68-5G 已禁自动连接
- iPhone 热点坑:**离开"个人热点"设置页后 SSID 停止广播**(Mac 走 iCloud 不受影响,机器人会掉)——连接前必须把设置页开着 + "允许其他人加入"开启
- 热点闪断频繁,SSH 长会话建议 nohup 脱离

**关机**:`sudo poweroff` 后板子会**自动重启**(PMIC 不保持断电)——要在关机日志走完的几秒内关电池开关;错过了就直接硬断电,GRUB 菜单 30 秒自动过

**串口取数**:
- 串口捕获会丢**输出开头**(约前 50 秒)——用固定窗口原始读取,或 `tac` 倒序输出让丢失段落在已知部分
- 机器人 /tmp 重启即清空,长存的东西放家目录;串口长命令 >1000 字符会损坏,传文件走 scp

**传感器操作铁律**:读航向前电机必须停转 ≥2 秒;机器人水平放置(mz 波动 >10 说明倾斜,2D 标定会劣化)

## 5. 关键路径速查

```
Mac:
  ~/Projects/CSE276A/           仓库(dev-yuanshengqi 分支 = 我方;main = 队友)
  ~/Desktop/cse276a/            课程文件夹:作业 PDF、SETUP_PLAN.md、scripts/(全部实验脚本)
  ~/Desktop/cse276a/calibration_data.csv   参数卡 CSV(与仓库副本同步)
机器人:
  ~/ros2_ws/                    ROS 工作区(队友已 build robot_control)
  /dev/ttyUSB0                  电机板串口(IMU+电机命令,JSON 协议)
  {"T":126} 请求 IMU / {"T":1,"L":x,"R":x} 电机命令(±0.5)
```
