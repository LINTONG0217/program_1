# 全车 UWB / 主控配置清单

适用对象：

- 辅助车 A：固定基站 0
- 辅助车 B：固定基站 1
- 主车：UWB 标签 + 主控板
- 从车：UWB 标签 + 主控板

当前方案基于：

- 两基站 + 车载 IMU / 编码器连续选解
- 场地尺寸 `3.2m x 2.4m`
- 基站布置在场地下边两端
- 小车从基站连线同一侧启动

## 1. DW3000 AT 配置

统一参数：

- 信道：`0`，即图里 `CH9`
- 速率：`1`，即 `6.8M`
- 组号：`1`

### 辅助车 A / 基站 0

```text
AT+SETCFG=0,1,0,1,1
```

含义：

- `ID=0`
- `Role=1`，基站
- `CH=0`
- `Rate=1`
- `Group=1`

### 辅助车 B / 基站 1

```text
AT+SETCFG=1,1,0,1,1
```

含义：

- `ID=1`
- `Role=1`，基站
- `CH=0`
- `Rate=1`
- `Group=1`

### 主车 / 标签

```text
AT+SETCFG=2,0,0,1,1
```

含义：

- `ID=2`
- `Role=0`，标签
- `CH=0`
- `Rate=1`
- `Group=1`

### 从车 / 标签

```text
AT+SETCFG=3,0,0,1,1
```

含义：

- `ID=3`
- `Role=0`，标签
- `CH=0`
- `Rate=1`
- `Group=1`

注意：

- 每台模块配置后还要执行厂家手册里的“保存配置”指令
- 如果你决定改组号或信道，4 台设备必须一起改

## 2. 基站摆位

推荐摆位：

- 基站 0：`(0.0, 0.0)`
- 基站 1：`(3.2, 0.0)`

这对应场地下边一条边。

小车启动要求：

- 主车和从车都从基站连线的同一侧启动
- 当前测试默认假定小车在基站连线“上方”，也就是 `y > 0`
- 车头建议统一朝向场地 `+x`

## 3. 主车主控配置

主车烧录时，`Module/config.py` 里至少要确认这些值：

```python
APP_PROFILE = "test_uwb_two_anchor"   # 定位测试时
# APP_PROFILE = "competition"         # 比赛时改回

UWB_POSE_ENABLE = True
UWB_UART_ID = 5
UWB_BAUDRATE = 115200
UWB_TX_PIN = "D20"
UWB_RX_PIN = "D21"

UWB_ANCHOR_0_ID = 0
UWB_ANCHOR_0_X_M = 0.0
UWB_ANCHOR_0_Y_M = 0.0
UWB_ANCHOR_1_ID = 1
UWB_ANCHOR_1_X_M = 3.2
UWB_ANCHOR_1_Y_M = 0.0

UWB_TWO_ANCHOR_PREFERRED_SIDE = 1
UWB_TWO_ANCHOR_INIT_ENABLE = True
UWB_TWO_ANCHOR_INIT_X_M = 0.25
UWB_TWO_ANCHOR_INIT_Y_M = 0.25
UWB_TWO_ANCHOR_INIT_YAW_DEG = 0.0
```

如果模块需要轮询才返回距离，再加：

```python
UWB_RANGE_POLL_ENABLE = True
UWB_RANGE_CMD_TEMPLATE = "AT+DISTANCE={id}\r\n"
```

如果模块会主动连续上报距离，则保持：

```python
UWB_RANGE_POLL_ENABLE = False
```

## 4. 从车主控配置

### 从车做定位测试时

从车单独验证定位时，推荐和主车保持同样的两基站测试配置：

```python
APP_PROFILE = "test_uwb_two_anchor"

UWB_POSE_ENABLE = True
UWB_UART_ID = 5
UWB_BAUDRATE = 115200
UWB_TX_PIN = "D20"
UWB_RX_PIN = "D21"

UWB_ANCHOR_0_ID = 0
UWB_ANCHOR_0_X_M = 0.0
UWB_ANCHOR_0_Y_M = 0.0
UWB_ANCHOR_1_ID = 1
UWB_ANCHOR_1_X_M = 3.2
UWB_ANCHOR_1_Y_M = 0.0

UWB_TWO_ANCHOR_PREFERRED_SIDE = 1
UWB_TWO_ANCHOR_INIT_ENABLE = True
UWB_TWO_ANCHOR_INIT_X_M = 0.45
UWB_TWO_ANCHOR_INIT_Y_M = 0.25
UWB_TWO_ANCHOR_INIT_YAW_DEG = 0.0
```

建议把从车初始 `x` 和主车错开一点，方便两台车同时上电时看输出。

### 从车进协同时

```python
APP_PROFILE = "dual_slave"
COMPETITION_DUAL_ENABLE = True
UWB_POSE_ENABLE = True
```

其余 UWB 锚点坐标保持和主车一致。

## 5. 主车进协同时

```python
APP_PROFILE = "competition"
COMPETITION_DUAL_ENABLE = True
UWB_POSE_ENABLE = True
```

其余 UWB 锚点坐标保持和从车一致。

## 6. 角色分工总结

### 辅助车 A

- 只需要做基站
- 不运行主控定位业务
- DW3000 角色：`Anchor`

### 辅助车 B

- 只需要做基站
- 不运行主控定位业务
- DW3000 角色：`Anchor`

### 主车

- DW3000 角色：`Tag`
- 测试定位时：`APP_PROFILE = "test_uwb_two_anchor"`
- 比赛时：`APP_PROFILE = "competition"`

### 从车

- DW3000 角色：`Tag`
- 测试定位时：`APP_PROFILE = "test_uwb_two_anchor"`
- 协同时：`APP_PROFILE = "dual_slave"`

## 7. 上车检查顺序

1. 先给 4 个 DW3000 模块写入正确的 `AT+SETCFG`
2. 确认 2 个基站固定摆位不动
3. 主车单独跑 `test_uwb_two_anchor`
4. 从车单独跑 `test_uwb_two_anchor`
5. 两车都能稳定出现 `[2A FIX]` 后，再切回 `competition` / `dual_slave`

## 8. 现阶段限制

- 两基站方案依赖 IMU、编码器和连续运动约束
- 它更适合“连续跟踪修正”，不是完全无先验的绝对定位
- 如果现场经常跳镜像解，优先补第 3 个固定基站
