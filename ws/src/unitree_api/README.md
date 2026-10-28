# unitree_api

Go2/Go2W 高层运动接口（sport）的请求与响应消息。

字段按 `~/prj/unitree_sdk2/include/unitree/robot/internal/internal_idl_decl/*.hpp`（unitree_sdk2，2026-07-09）逐一定义。
ROS 2 会把 `unitree_api/msg/Request` 映射为 DDS 类型 `unitree_api::msg::dds_::Request_`，
话题 `/api/sport/request` 映射为 DDS 话题 `rt/api/sport/request`，与 SDK 和 Go2W 主控一致。

| 话题 | 类型 | 方向 |
|---|---|---|
| `/api/sport/request` | `unitree_api/msg/Request` | 发给主控 |
| `/api/sport/response` | `unitree_api/msg/Response` | 主控回复 |

常用 api_id（`sport_api.hpp`）：Damp 1001、BalanceStand 1002、StopMove 1003、Move 1008（parameter 为 `{"x": vx, "y": vy, "z": vyaw}`）。
