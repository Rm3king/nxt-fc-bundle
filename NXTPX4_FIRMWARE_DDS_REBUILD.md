# NxtPX4 固件重编与 DDS 话题裁剪需求

本文记录下一步为了导出高频角速度反馈、裁剪 uXRCE-DDS 带宽、并重新烧录 NxtPX4v2 固件的完整流程。

当前硬件和软件假设：

- 飞控：NxtPX4v2
- PX4 target：`hkust_nxt-dual`
- PX4 端串口：`/dev/ttyS3`
- NX 端串口：`/dev/ttyTHS3`
- uXRCE-DDS 串口波特率：`921600`
- ROS_DOMAIN_ID / UXRCE_DDS_DOM_ID：`0`
- 当前 ROS2 端已经使用 NxtPX4 fork 对应的 `px4_msgs`

## 1. 为什么要重编固件

目前 `/fmu/out/vehicle_attitude` 在 NX 侧约 `100 Hz`，链路稳定；但是 `/fmu/out/vehicle_angular_velocity`
没有导出。后续几何控制、扰动观测、机械臂反作用补偿、强化学习状态反馈都需要角速度，因此需要在 PX4
固件里的 uXRCE-DDS topic 清单中打开：

```text
/fmu/out/vehicle_angular_velocity
```

额外做过一次诊断：把 uXRCE-DDS 串口从 `921600` 临时拉到 `2000000` 后，`/fmu/out/vehicle_attitude`
仍稳定卡在约 `100 Hz`，没有明显升高。源码实查后确认：当前 NxtPX4 fork 的
`dds_topics.h.em` 对所有 `/fmu/out/*` 统一调用：

```cpp
orb_set_interval(fds[idx].fd, UXRCE_DEFAULT_POLL_RATE);
```

其中 `UXRCE_DEFAULT_POLL_RATE = 10`，即所有 DDS out topic 默认被订阅层限制到 `10 ms / 100 Hz`。
所以单纯提高串口波特率不会让 `/fmu/out/vehicle_attitude` 超过约 `100 Hz`。

本仓库已改成从 `dds_topics.yaml` 读取每个 topic 的 `rate_limit`，生成对应的 `orb_set_interval`
毫秒间隔；未写 `rate_limit` 的 topic 继续沿用 10 ms 默认值。需要重点检查和维护：

```text
PX4-Autopilot/src/modules/uxrce_dds_client/dds_topics.yaml
```

以及编译生成的：

```text
build/*/src/modules/uxrce_dds_client/dds_topics.h
```

同时为了避免串口带宽被无关 topic 占满，需要裁剪低优先级 topic，并给高频 topic 加 `rate_limit`。
这版固件按“一次覆盖姿态、角速度、位置速度、遥控、安全状态、动捕输入”来做，不再为了早期姿态测试
临时关闭位置/速度数据，避免后面接动捕和位置环时再次重编。

目标频率：

```text
vehicle_attitude            200 Hz
vehicle_angular_velocity    200 Hz
vehicle_local_position      50 Hz，链路有余量再升 80 Hz
vehicle_odometry            50 Hz，链路有余量再升 80 Hz
vehicle_status              1-5 Hz
manual_control_setpoint     20-50 Hz
vehicle_control_mode        10-50 Hz
battery_status              1-5 Hz
failsafe_flags              5-20 Hz
timesync_status             5-20 Hz
vehicle_command_ack         事件型，保留
```

四旋翼 + 机械臂后续要做 DOB / INDI / MPC / RL，并且 Offboard 接管到 thrust + body-rate 层，
姿态四元数和机体系角速度建议优先按 `200 Hz` 打开。`150 Hz` 可以作为链路压力过大时的保守下限；
`100 Hz` 更适合慢速 RL 状态观测或上层位置/速度环，不建议作为角速度闭环反馈的长期目标。

## 2. 编译放在哪里做

优先建议在笔记本 Ubuntu 上做：

- 下载源码、装 PX4 toolchain、编译、烧录都更舒服
- 磁盘和散热余量更大
- 编译失败排查更方便

NX 也可以编译，但要注意：

- 确保剩余磁盘空间足够，建议至少 `20 GB`
- 编译时接电源，避免降频
- 第一次安装 toolchain 会比较慢
- 不要在飞控/NX 正在跑实机链路时编译，避免影响调试

结论：如果只是赶进度，可以先在 NX 上试；如果要长期维护固件，建议笔记本上固定一份 PX4 编译环境。

## 3. 烧录前必须保存参数

烧固件前一定先保存当前 QGC/PX4 参数。至少保存两份：

1. QGroundControl 参数导出文件
2. 关键参数截图或文本记录

QGC 保存路径：

```text
Vehicle Setup -> Parameters -> Tools -> Save to file
```

建议文件命名：

```text
NxtPX4v2_params_before_dds_rebuild_YYYYMMDD.params
```

必须重点核对并记录：

```text
Airframe / Mixer
传感器校准
遥控器校准
飞行模式开关
电机顺序和方向
电池/电源参数
UXRCE_DDS_CFG
UXRCE_DDS_DOM_ID
UXRCE_DDS_KEY
SER_TEL*_BAUD
MAV_*_CONFIG
COM_OF_LOSS_T
COM_OBL_RC_ACT
COM_RC_IN_MODE
COM_RC_OVERRIDE
EKF2_* 外部视觉相关参数
```

烧录后不要立刻装桨测试，先恢复/核对参数，再做无桨检查。

## 4. 获取源码

最安全原则：尽量使用当前飞控固件对应的同一个 NxtPX4/PX4 源码分支。

你之前飞控信息里出现过：

```text
HW arch: HKUST_NXT_DUAL
PX4 git-branch: wmy_test
PX4 git-hash: 04c30103...
PX4 version: 1.14.0
```

如果能拿到这个 `wmy_test / 04c30103...` 对应源码，优先用它。这样对 airframe、board config、uORB
message 定义最稳。

如果拿不到，就使用公开 Nxt-FC 作为基线：

```bash
cd ~/my_data
git clone --recursive https://github.com/HKUST-Aerial-Robotics/Nxt-FC.git
cd Nxt-FC
git submodule update --init --recursive
cd PX4-Autopilot
git rev-parse --short HEAD
git branch --show-current
```

确认 target 存在：

```bash
make list_config_targets | grep hkust
```

期望能看到类似：

```text
hkust_nxt-dual
```

如果源码 hash/branch 和当前飞控不一致，烧录前要接受一个事实：这不只是 DDS 清单变化，还可能带来
board config、驱动、默认参数或 message 定义差异。烧录后必须完整回归测试。

## 5. 安装 PX4 编译环境

在 PX4 源码上安装 toolchain：

```bash
cd ~/my_data/Nxt-FC/PX4-Autopilot
bash Tools/setup/ubuntu.sh
```

安装完成后重启电脑或重新登录 shell。

如果只在笔记本编译 NuttX 固件，不需要仿真工具，可以尝试：

```bash
bash Tools/setup/ubuntu.sh --no-sim-tools
```

如果脚本失败，先不要硬改系统，记录报错再处理。

## 6. 修改 DDS topic 清单

文件位置通常是：

```text
PX4-Autopilot/src/modules/uxrce_dds_client/dds_topics.yaml
```

先搜索相关 topic：

```bash
cd ~/my_data/Nxt-FC/PX4-Autopilot
rg -n "vehicle_attitude|vehicle_angular_velocity|vehicle_odometry|vehicle_local_position|sensor_combined|battery_status|manual_control_setpoint|failsafe_flags" \
  src/modules/uxrce_dds_client/dds_topics.yaml
```

编译后也检查生成代码，确认限制真的进入固件：

```bash
rg -n "vehicle_attitude|vehicle_angular_velocity|rate_limit|interval|deadline|SubscriptionInterval" \
  build/*/src/modules/uxrce_dds_client/dds_topics.h
```

需求如下。

必须保留/打开。下面频率是第一版已落地配置，实际以 `921600` 串口稳定性为准：

```yaml
publications:
  - topic: /fmu/out/vehicle_attitude
    type: px4_msgs::msg::VehicleAttitude
    rate_limit: 200.

  - topic: /fmu/out/vehicle_angular_velocity
    type: px4_msgs::msg::VehicleAngularVelocity
    rate_limit: 200.

  - topic: /fmu/out/vehicle_local_position
    type: px4_msgs::msg::VehicleLocalPosition
    rate_limit: 50.

  - topic: /fmu/out/vehicle_odometry
    type: px4_msgs::msg::VehicleOdometry
    rate_limit: 50.

  - topic: /fmu/out/vehicle_status
    type: px4_msgs::msg::VehicleStatus
    rate_limit: 5.

  - topic: /fmu/out/vehicle_control_mode
    type: px4_msgs::msg::VehicleControlMode
    rate_limit: 20.

  - topic: /fmu/out/manual_control_setpoint
    type: px4_msgs::msg::ManualControlSetpoint
    rate_limit: 50.

  - topic: /fmu/out/battery_status
    type: px4_msgs::msg::BatteryStatus
    rate_limit: 5.

  - topic: /fmu/out/failsafe_flags
    type: px4_msgs::msg::FailsafeFlags
    rate_limit: 10.

  - topic: /fmu/out/timesync_status
    type: px4_msgs::msg::TimesyncStatus
    rate_limit: 10.

  - topic: /fmu/out/vehicle_command_ack
    type: px4_msgs::msg::VehicleCommandAck
```

如果 `vehicle_local_position` 和 `vehicle_odometry` 同时 80 Hz 导致串口压力过大，优先降频：

```text
vehicle_local_position  50 Hz
vehicle_odometry        50 Hz
```

本次源码改动后，离线生成检查得到的关键间隔应为：

```text
/fmu/out/vehicle_attitude            5 ms   # 200 Hz
/fmu/out/vehicle_angular_velocity    5 ms   # 200 Hz
/fmu/out/manual_control_setpoint     20 ms  # 50 Hz
/fmu/out/vehicle_local_position      20 ms  # 50 Hz
/fmu/out/vehicle_odometry            20 ms  # 50 Hz
/fmu/out/vehicle_control_mode        50 ms  # 20 Hz
/fmu/out/failsafe_flags              100 ms # 10 Hz
/fmu/out/timesync_status             100 ms # 10 Hz
/fmu/out/battery_status              200 ms # 5 Hz
/fmu/out/vehicle_status              200 ms # 5 Hz
```

不建议直接裁掉它们，因为本项目一定会接动捕，位置/速度状态后面是位置环和 RL 的基础输入。

需要保留的输入 topic：

```yaml
subscriptions:
  - topic: /fmu/in/offboard_control_mode
    type: px4_msgs::msg::OffboardControlMode

  - topic: /fmu/in/vehicle_rates_setpoint
    type: px4_msgs::msg::VehicleRatesSetpoint

  - topic: /fmu/in/vehicle_command
    type: px4_msgs::msg::VehicleCommand

  - topic: /fmu/in/vehicle_visual_odometry
    type: px4_msgs::msg::VehicleOdometry
```

可关闭或低频保留的低优先级 out topic：

```text
position_setpoint_triplet
estimator_status_flags
sensor_combined
actuator_motors
actuator_servos
```

说明：

- `vehicle_local_position` 和 `vehicle_odometry` 本版直接保留，避免接动捕后重编。
- `sensor_combined` 如果不做原始 IMU 分析，可以先关或低频保留；角速度优先用 `vehicle_angular_velocity`。
- `rate_limit` 是最大 DDS 发布频率限制，不会改变 PX4 内部 EKF/控制环频率。
- 不要裁掉 `/fmu/in/vehicle_rates_setpoint`，这是当前 NX 控制链路的核心输入。
- 不要裁掉 `/fmu/out/manual_control_setpoint`，这是 RC/拨杆/人工接管逻辑的重要输入。

## 7. 编译固件

```bash
cd ~/my_data/Nxt-FC/PX4-Autopilot
make hkust_nxt-dual
```

编译成功后，在 `build/` 下查找固件：

```bash
find build -maxdepth 3 -type f \( -name "*.px4" -o -name "*.bin" \)
```

常见产物类似：

```text
build/hkust_nxt-dual_default/hkust_nxt-dual_default.px4
```

实际路径以本机 build 输出为准。

## 8. 烧录固件

推荐用 QGroundControl 烧录自定义固件：

```text
QGC -> Vehicle Setup -> Firmware -> Advanced settings -> Custom firmware file
```

选择刚刚编译出的 `.px4` 文件。

烧录后：

1. 重启飞控
2. 连接 QGC
3. 检查 airframe/传感器/RC/模式/电机配置是否仍正确
4. 如果参数丢失，用前面保存的 `.params` 文件恢复
5. 重启飞控

## 9. 同步 ROS2 端 px4_msgs

只要重新编译/烧录了 fork PX4，就必须确认 ROS2 端 `px4_msgs` 与固件里的 `msg/` 一致。
否则会再次出现：

```text
payload size ... > history payload size ...
```

同步方式：

```bash
cd ~/my_data/aerial_manipulation/onboard_ws_cpp
cp -a src/px4_msgs/msg src/px4_msgs/msg_backup_before_dds_rebuild

rm -f src/px4_msgs/msg/*.msg
cp ~/my_data/Nxt-FC/PX4-Autopilot/msg/*.msg src/px4_msgs/msg/

if [ -d ~/my_data/Nxt-FC/PX4-Autopilot/msg/versioned ]; then
  cp ~/my_data/Nxt-FC/PX4-Autopilot/msg/versioned/*.msg src/px4_msgs/msg/
fi

colcon build --symlink-install --packages-select px4_msgs am_core_cpp am_px4_bridge_cpp am_controllers_cpp am_bringup_cpp
source install/setup.bash
```

如果 PX4 源码里有 `srv/` 定义，也同步：

```bash
rm -f src/px4_msgs/srv/*.srv
cp ~/my_data/Nxt-FC/PX4-Autopilot/srv/*.srv src/px4_msgs/srv/
```

## 10. 烧录后链路验证

PX4 MAVLink Console：

```sh
uxrce_dds_client stop
uxrce_dds_client start -t serial -d /dev/ttyS3 -b 921600
```

NX：

```bash
cd ~/my_data/aerial_manipulation/onboard_ws_cpp
source env.sh
scripts/start_agent.sh /dev/ttyTHS3 921600 2
```

确认 topic：

```bash
ros2 topic list | grep /fmu/out/vehicle_angular_velocity
ros2 topic echo /fmu/out/vehicle_angular_velocity --once
ros2 topic hz /fmu/out/vehicle_attitude
ros2 topic hz /fmu/out/vehicle_angular_velocity
ros2 topic hz /fmu/out/vehicle_local_position
ros2 topic hz /fmu/out/vehicle_odometry
ros2 topic echo /fmu/out/manual_control_setpoint --once
```

期望：

```text
vehicle_attitude            接近并稳定在 180-200 Hz
vehicle_angular_velocity    接近并稳定在 180-200 Hz
vehicle_local_position      稳定 45-50 Hz
vehicle_odometry            稳定 45-50 Hz
manual_control_setpoint     能看到 RC 输入
没有 payload size mismatch
没有长时间 0.5-1.0 s 堵塞尖峰
```

再跑只读检查：

```bash
python3 scripts/preflight_check.py
python3 scripts/watch_attitude.py
```

确认角速度稳定后，ROS2 几何姿态控制器可以打开角速度反馈：

```yaml
# onboard_ws_cpp/src/am_bringup_cpp/config/geometric_attitude.yaml
use_angular_velocity_feedback: true
```

然后重新启动 bringup：

```bash
ros2 launch am_bringup_cpp bringup.launch.py enable_arm:=false enable_mocap:=false \
  enable_controller:=true controller_type:=geometric_attitude
```

观察是否不再出现 `angular velocity stale`：

```bash
ros2 topic echo /am/control/body_rate_sp
```

然后做无桨 Offboard 检查：

```bash
python3 scripts/offboard_rate_pulse_test.py
```

最后才允许做无桨解锁小油门：

```bash
python3 scripts/offboard_rate_pulse_test.py --props-removed --arm --thrust 0.03
```

## 11. 回滚策略

如果新固件出现任一问题：

- `/fmu/out/*` 不稳定
- topic 类型 mismatch
- 无法进入 Offboard
- RC/模式/解锁行为异常
- 电机顺序、方向、混控异常

立刻停止实机测试。回滚方式：

1. 用 QGC 重新烧回之前稳定固件
2. 恢复烧录前保存的 `.params`
3. 重新跑无桨电机顺序和 Offboard smoke test

## 12. 对接结论

这次固件重编的最小目标不是改控制器，而是改 PX4 uXRCE-DDS 出口：

```text
打开 vehicle_angular_velocity
保留 vehicle_local_position / vehicle_odometry，限制到 50-100 Hz
保留 manual_control_setpoint 给 RC/拨杆/接管逻辑
限制 attitude/angular_velocity 到 200 Hz
裁剪低优先级高频 out topic
保持 body-rate Offboard 输入 topic 不变
保持 vehicle_visual_odometry 输入 topic 不变，给动捕接入使用
重新同步 ROS2 px4_msgs
```

完成后，NX 侧就可以同时拿到姿态和角速度，继续做几何控制、扰动观测、机械臂补偿和 RL 状态反馈。
