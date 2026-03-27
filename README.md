# droncan

将原有的 **CAN 原始帧接收** 改为基于 **DroneCAN 协议** 的消息接收。

---

## 背景

| 方式 | 文件 | 说明 |
|---|---|---|
| 旧：CAN 原始帧接收 | `can_receiver.py` | 使用 `python-can` 直接读取原始 CAN 帧，不解析协议层 |
| 新：DroneCAN 接收 | `dronecan_receiver.py` | 使用 `dronecan` 库接收并解码 DroneCAN（UAVCAN v0）消息 |

DroneCAN（原名 UAVCAN v0）是专为无人机 CAN 总线设计的轻量级协议，在原始 CAN 层之上提供：

- 节点寻址（Node ID 1–127）
- 消息帧化与多帧传输重组
- 数据类型编码（DSDL）
- 标准消息集（NodeStatus、ESCStatus 等）

---

## 快速开始

### 安装依赖

```bash
pip install -r requirements.txt
```

### 运行 DroneCAN 接收器

```bash
python dronecan_receiver.py
```

默认监听 `vcan0`（socketcan），节点 ID 为 127。可通过修改 `receive_dronecan_messages()` 的参数自定义接口、总线类型和节点 ID。

### 运行旧版 CAN 原始帧接收器（供对比参考）

```bash
python can_receiver.py
```

---

## 文件说明

| 文件 | 说明 |
|---|---|
| `dronecan_receiver.py` | **新**：DroneCAN 消息接收，订阅 `NodeStatus` 和 `ESCStatus` |
| `can_receiver.py` | **旧**：原始 CAN 帧接收（仅供对比参考） |
| `requirements.txt` | Python 依赖（`dronecan`、`python-can`） |

