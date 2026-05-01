# uart_vision_receiver

- 层级：Module
- 功能：串口视觉数据接收与标准化
- 代码映射：`vision.py`

## 协议

接收端支持两种“每帧一行”的 UART 协议：

- 新协议（推荐）：`SP|<json>|<crc16_hex>\n`
  - `SP`：帧头
  - `<json>`：UTF-8 JSON 文本
  - `<crc16_hex>`：对 `<json>` 字节计算 CRC16（CRC-16/IBM/ARC, init=0xFFFF, poly=0xA001），4 位十六进制
- 旧协议（兼容）：`<json>\n`

当 CRC 校验失败时，本帧会被丢弃，不更新 last_frame。
