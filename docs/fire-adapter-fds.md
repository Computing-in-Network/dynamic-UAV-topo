# Fire Adapter（FDS 输入）使用说明

## 功能

- 离线模式（`mode=offline`）：读取单文件 JSON/CSV。
- 流式模式（`mode=stream`）：轮询目录中的新增文件，支持断点续读与坏文件跳过。

## 运行（离线）

```bash
./scripts/fire_adapter_fds_start.sh \
  data/fds_samples/fds_fire_snapshot.sample.json \
  auto \
  1.0 \
  /env/fire_state \
  offline
```

## 运行（流式）

```bash
./scripts/fire_adapter_fds_start.sh \
  data/fds_samples/fds_fire_snapshot.sample.json \
  auto \
  2.0 \
  /env/fire_state \
  stream \
  data/fds_samples/stream \
  '*.json' \
  /tmp/fire_adapter_fds_checkpoint.json
```

参数：

1. `input_path`：离线输入文件（流式模式下仅占位）
2. `input_format`：`auto|json|csv`
3. `publish_hz`：发布频率（Hz）
4. `output_topic`：输出话题（默认 `/env/fire_state`）
5. `mode`：`offline|stream`
6. `stream_dir`：流式目录
7. `stream_glob`：文件匹配模式
8. `checkpoint_file`：断点文件路径

## 验证

```bash
source /opt/ros/humble/setup.bash
source ros2_ws/install/setup.bash
ros2 topic echo /env/fire_state --once
```

流式验证建议：

1. 先启动流式模式。
2. 向 `stream_dir` 新增 `*.json` 文件。
3. 观察话题在 2 秒内更新。
4. 终止后重启，确认不会重复消费已处理文件（依赖 checkpoint）。

## 常见故障

- `input file not found`：离线输入路径不存在。
- `stream_dir not found`：流式目录不存在。
- `unsupported input_format/mode`：参数值非法。
