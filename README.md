# waypoint_manager

ROS 2 (Humble) 向けのシンプルなウェイポイントフォロワーです。`waypoint_editor` と共通の CSV 形式で定義したウェイポイント列に沿ってロボットをナビゲートします。Nav2 などのナビゲーションスタックが出力する速度指令を監視しつつ、`goal_pose` トピックへ次の目標姿勢を順番に送信します。

## パッケージ構成

| パッケージ | 説明 |
|-----------|------|
| `waypoint_manager` | メインノード (`waypoint_manager`) |
| `waypoint_manager_msgs` | ウェイポイント定義用カスタムメッセージ |

## 動作概要

1. 起動時に CSV ファイルからウェイポイントを読み込む
2. 最初のウェイポイントを `goal_pose` に publish
3. 100 ms 周期のタイマーでロボット位置（`map` フレーム）を監視
4. 現在のウェイポイント半径内に入ると、次のウェイポイントへ目標を更新
5. `robot_wait: true` のウェイポイントでは、到達後に `cmd_vel` をゼロにして停止を維持
6. `/restart_waypoint_manager` サービスで先頭ウェイポイントから再開

```
  CSV waypoints (waypoint_editor 共通形式)
        |
        v
  waypoint_manager ----goal_pose----> Nav2 / ナビゲータ
        ^                                        |
        |                                        v
   TF (map<-base)                          cmd_vel
        |                                        |
        +-------- cmd_vel_sub <------------------+
        |
        v
     cmd_vel (sim_cmd_vel 等へ remap)
```

## 依存関係

- ROS 2 Humble
- `rclcpp`, `geometry_msgs`, `std_srvs`
- `nav2_util`, `tf2_ros`
- `waypoint_manager_msgs`

## ビルド

```bash
cd ~/cursor_ws/waypint_ws   # ワークスペースルート
source /opt/ros/humble/setup.bash
colcon build --packages-select waypoint_manager_msgs waypoint_manager
source install/setup.bash
```

## 起動

```bash
ros2 launch waypoint_manager waypoint_manager.launch.xml
```

別のウェイポイントファイルを使う場合:

```bash
ros2 launch waypoint_manager waypoint_manager.launch.xml \
  waypoint_csv:=/path/to/waypoint.csv
```

パラメータのみ指定して起動する場合:

```bash
ros2 run waypoint_manager waypoint_manager --ros-args \
  -p waypoint_csv_path:=$(ros2 pkg prefix waypoint_manager)/share/waypoint_manager/config/waypoint_test.csv \
  -p waypoint_radius:=0.5
```

## パラメータ

| 名前 | 型 | デフォルト | 説明 |
|------|-----|-----------|------|
| `waypoint_csv_path` | string | `waypoint.csv` | ウェイポイント定義 CSV のパス |
| `waypoint_radius` | double | `0.5` | 各ウェイポイントの到達判定半径（m）。CSV で個別指定がない場合に使用 |

## トピック

| トピック | 型 | 方向 | 説明 |
|---------|-----|------|------|
| `goal_pose` | `geometry_msgs/PoseStamped` | publish | 現在のナビゲーション目標 |
| `cmd_vel_sub` | `geometry_msgs/Twist` | subscribe | ナビゲーションスタックからの速度指令（入力） |
| `cmd_vel` | `geometry_msgs/Twist` | publish | ロボットへ送る速度指令（`robot_wait` 時はゼロ） |

launch ファイルでは次の remap が設定されています:

- `cmd_vel_sub` → `/cmd_vel`（Nav2 の出力を受信）
- `cmd_vel` → `/sim_cmd_vel`（シミュレータ等へ送信）

## サービス

| サービス | 型 | 説明 |
|---------|-----|------|
| `/restart_waypoint_manager` | `std_srvs/Trigger` | ウェイポイント列を先頭から再開し、待機状態を解除 |

```bash
ros2 service call /restart_waypoint_manager std_srvs/srv/Trigger
```

## ウェイポイント CSV 形式（`waypoint_editor` 共通）

```csv
id,pose_x,pose_y,pose_z,rot_x,rot_y,rot_z,rot_w,waypoint_radius,robot_wait,command,
0,6.13,3.99,0,0,0,0.707,0.707,0.5,false,,
1,4.68,6.62,0,0,0,0.383,0.924,0.5,true,,
2,2.19,7.05,0,0,0,0.0,1.0,0.5,false,wait,
```

### フィールド説明

- `id`: ウェイポイント識別子（ログ表示用）
- `pose_x/y/z`, `rot_x/y/z/w`: `map` フレーム上の位置とクォータニオン
- `waypoint_radius`: 到達判定半径 [m]（省略時はパラメータ `waypoint_radius`）
- `robot_wait`: 到達後にロボットを停止させるか（`true`/`false`）。`true` のとき `/restart_waypoint_manager` で再開
- `command`: `waypoint_editor` 用の任意コマンド列（本ノードでは未使用）

`waypoint_radius` / `robot_wait` 列が無い従来形式の CSV も読み込めます（半径はパラメータ値、`robot_wait` は `false`）。

サンプルファイル:

- `waypoint_manager/config/waypoint_test.csv` — 3 点のテスト経路
- `waypoint_manager/config/waypoint.csv` — 6 点の本番用経路

### 旧 YAML からの変換

以前の YAML コンフィグは次のスクリプトで CSV に変換できます（`python3-yaml` / PyYAML が必要）:

```bash
# ワークスペース内のスクリプトを直接実行
./src/waypoint_manager/waypoint_manager/scripts/yaml_to_csv.sh /path/to/waypoint.yaml

# 出力先を指定
./src/waypoint_manager/waypoint_manager/scripts/yaml_to_csv.sh \
  -o /path/to/waypoint.csv /path/to/waypoint.yaml

# 複数ファイル（各入力と同名の .csv を生成）
./src/waypoint_manager/waypoint_manager/scripts/yaml_to_csv.sh -r 0.5 a.yaml b.yaml

# インストール後
ros2 run waypoint_manager yaml_to_csv.sh /path/to/waypoint.yaml
```

変換内容:

- `position` + `euler_angle.z` → `pose_*` / `rot_*`（yaw からクォータニオンへ）
- `functions.variable_waypoint_radius` → `waypoint_radius`（未指定時は `-r` の値）
- `robot_wait` → `robot_wait`（未指定時は `false`）

## 必要条件（実行環境）

- `map` → `base_link`（または `base_footprint`）の TF が配信されていること
- ナビゲーションスタックが `goal_pose` を受け取り、`cmd_vel` を出力すること

TF が無い場合、位置取得に失敗しウェイポイント到達判定は行われません（ログスパムは抑止済み）。

## ライセンス

Apache-2.0

## メンテナ

Keitaro Nakamura (numerugon487@gmail.com)
