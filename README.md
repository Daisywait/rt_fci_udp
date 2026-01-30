rt_fci_udp
===========

面向 FR3 的最小硬实时桥接：
- libfranka 1 kHz 控制回路（CartesianVelocity）
- UDP 输入：最新的 Twist 速度（vx,vy,vz,wx,wy,wz）
- UDP 输出：末端位姿矩阵（O_T_EE，4x4 列主序）

构建（在已安装 libfranka 的 asus 上）
-------------------------------------
mkdir -p build
cd build
cmake ..
make -j

运行（在 asus 上）
-----------------
默认会读取当前目录下的 rt_fci_udp.conf；若在 build 目录运行，会自动尝试 ../rt_fci_udp.conf（无需传参）：
./rt_fci_udp

手动指定配置文件：
./rt_fci_udp --config /home/enine/rt_fci_udp/rt_fci_udp.conf

默认值：
- cmd_bind_ip = 0.0.0.0
- cmd_port = 5005
- pose_target_ip = 10.0.0.2
- pose_port = 5006
- pose_hz = 200
- cmd_timeout_ms = 200

示例：
./rt_fci_udp
./rt_fci_udp --config /home/enine/rt_fci_udp/rt_fci_udp.conf

直连示例（asus 10.0.0.1 -> enine 10.0.0.2）：
使用配置文件 rt_fci_udp.conf（默认已是 10.0.0.2）。

enine 侧启动（VR + UDP 节点）
---------------------------
ros2 launch vr_teleop_twist vr_teleop_udp.launch.py

UDP 包格式（主机字节序，预期 x86_64）
------------------------------------
CmdPacket（enine -> asus 发送）
- double v[6]  // 基坐标系下的 vx,vy,vz,wx,wy,wz
- uint64 t_ns  // 可选的发送端时间戳

PosePacket（asus -> enine 发送）
- double T[16] // franka::RobotState::O_T_EE，列主序 4x4
- uint64 t_ns  // 发送端时间戳（steady_clock）

备注
----
- 1 kHz 回调只读取最新指令，不会阻塞。
- 在 cmd_timeout_ms 内未收到指令时，速度设为零。
- 末端位姿矩阵的平移分量位于索引 12,13,14（列主序）。
