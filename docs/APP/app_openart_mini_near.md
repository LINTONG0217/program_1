# app_openart_mini_near

用途：OpenArt mini 近距离识别脚本，通过串口把识别到的物体标签发给主控。

对应脚本：

- `openart/openart_near_ball.py`

串口输出格式：

- `label=red,conf=0.93,cx=81,cy=64,pixels=2210`
- `label=green,conf=0.88,cx=77,cy=60,pixels=1840`
- `none`

主控侧兼容：

- `APP/test_uwb_two_anchor_localization.py`
- `Module/openart_mini_receiver.py` 中的 `OpenArtMiniReceiver`

建议流程：

1. 先只运行 OpenArt 脚本，确认串口持续输出 `label=...` 或 `none`。
2. 再打开主控侧 `OPENART_MINI_ENABLE = True`。
3. 保证主控与 OpenArt mini 波特率一致，默认 `115200`。
4. 按现场颜色重新标定 `THRESHOLDS`。

脚本内可调参数：

- `ROI`：近距离识别区域，默认只看画面中下部。
- `PIXELS_THRESHOLD` / `AREA_THRESHOLD`：过滤小噪声。
- `STABLE_FRAMES`：连续多少帧相同标签才发串口。
- `SEND_INTERVAL_MS`：串口发送节流。

联调建议：

- 如果总发 `none`，先扩大 `ROI` 和颜色阈值。
- 如果误检多，先增大 `PIXELS_THRESHOLD`、`AREA_THRESHOLD`，再提高 `STABLE_FRAMES`。
- 如果颜色漂移明显，优先固定光照并关闭自动白平衡、自动增益。
