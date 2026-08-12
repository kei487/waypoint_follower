# simple_waypoint_follower

ROS 2 (Humble) 向けのシンプルなウェイポイントフォロワーです。YAML で定義したウェイポイント列に沿ってロボットをナビゲートします。Nav2 などのナビゲーションスタックが出力する速度指令を監視しつつ、`goal_pose` トピックへ次の目標姿勢を順番に送信します。
WayponitをCSVで管理するバージョン

## パッケージ構成

| パッケージ | 説明 |
|-----------|------|
| `simple_waypoint_follower` | メインノード (`simple_wf`) |
| `simple_waypoint_follower_msgs` | ウェイポイント定義用カスタムメッセージ |

## 動作概要

1. 起動時に YAML ファイルからウェイポイントを読み込む
2. 最初のウェイポイントを `goal_pose` に publish
3. 100 ms 周期のタイマーでロボット位置（`map` フレーム）を監視
4. 現在のウェイポイント半径内に入ると、次のウェイポイントへ目標を更新
5. `robot_wait: true` のウェイポイントでは、到達後に `cmd_vel` をゼロにして停止を維持
6. `/restart_waypoint_follower` サービスで先頭ウェイポイントから再開

```
  YAML waypoints
        |
        v
  simple_waypoint_follower ----goal_pose----> Nav2 / ナビゲータ
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
- `nav2_util`, `tf2_ros`, `yaml-cpp`
- `simple_waypoint_follower_msgs`

## ビルド

```bash
cd ~/cursor_ws/waypint_ws   # ワークスペースルート
source /opt/ros/humble/setup.bash
colcon build --packages-select simple_waypoint_follower_msgs simple_waypoint_follower
source install/setup.bash
```

## 起動

```bash
ros2 launch simple_waypoint_follower simple_wf.launch.xml
```

別のウェイポイントファイルを使う場合:

```bash
ros2 launch simple_waypoint_follower simple_wf.launch.xml \
  waypoint_yaml:=/path/to/waypoint.yaml
```

パラメータのみ指定して起動する場合:

```bash
ros2 run simple_waypoint_follower simple_wf --ros-args \
  -p waypoint_yaml_path:=$(ros2 pkg prefix simple_waypoint_follower)/share/simple_waypoint_follower/config/waypoint_test.yaml \
  -p waypoint_radius:=0.5
```

## パラメータ

| 名前 | 型 | デフォルト | 説明 |
|------|-----|-----------|------|
| `waypoint_yaml_path` | string | `waypoint.yaml` | ウェイポイント定義 YAML のパス |
| `waypoint_radius` | double | `0.5` | 各ウェイポイントの到達判定半径（m）。YAML で個別指定がない場合に使用 |

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
| `/restart_waypoint_follower` | `std_srvs/Trigger` | ウェイポイント列を先頭から再開し、待機状態を解除 |

```bash
ros2 service call /restart_waypoint_follower std_srvs/srv/Trigger
```

## ウェイポイント YAML 形式

```yaml
waypoints:
  - id: 1
    position:
      x: 6.13
      y: 3.99
    euler_angle:
      z: 1.57          # yaw [rad]
    functions:         # 省略可
      - function: variable_waypoint_radius
        waypoint_radius: 0.5
    robot_wait: false  # 省略時は false。true で到達後に停止

  - id: 2
    position:
      x: 4.68
      y: 6.62
    euler_angle:
      z: 0.785
    robot_wait: true
```

### フィールド説明

- `id`: ウェイポイント識別子（ログ表示用）
- `position.x/y`: `map` フレーム上の位置 [m]
- `euler_angle.z`: 向き [rad]（yaw のみ使用）
- `functions`: オプション。`variable_waypoint_radius` で到達半径を個別指定
- `robot_wait`: 到達後にロボットを停止させるか。`true` のとき `/restart_waypoint_follower` で再開

サンプルファイル:

- `simple_waypoint_follower/config/waypoint_test.yaml` — 3 点のテスト経路
- `simple_waypoint_follower/config/waypoint.yaml` — 6 点の本番用経路

## 必要条件（実行環境）

- `map` → `base_link`（または `base_footprint`）の TF が配信されていること
- ナビゲーションスタックが `goal_pose` を受け取り、`cmd_vel` を出力すること

TF が無い場合、位置取得に失敗しウェイポイント到達判定は行われません（エラーログが `nav2_util` から出力されます）。

## デバッグ時の修正内容（2025-06）

以下の問題を修正済みです:

1. **ウェイポイント index のずれ** — 初期 `waypoint_id_` が `1` のため先頭ウェイポイントをスキップしていた → `0` から開始
2. **未初期化メンバ変数** — `_is_robot_wait`, `get_robot_pose_` をコンストラクタ前に初期化
3. **YAML パース** — `robot_wait` 未指定時の例外、`functions` 未指定時の半径未設定を修正
4. **ゴール再送のスパム** — 停止中に 100 ms ごとに goal を再送していた → 到達圏外かつ待機中でない場合のみ再送
5. **待機中の誤再送** — `robot_wait` 中も goal を再送していた → 待機フラグを考慮
6. **ログ過多** — ループ・cmd_vel コールバックの INFO ログを整理
7. **launch remap** — Nav2 の `/cmd_vel` を `cmd_vel_sub` へ正しく接続
8. **restart サービス** — 先頭ウェイポイントへのリセットと goal 再送を追加
9. **TF 未取得時のログスパム** — `lookupTransform` の例外を握りつぶし、TF 未接続時にエラーログが連続出力されないよう改善

## ライセンス

Apache-2.0

## メンテナ

Keitaro Nakamura (numerugon487@gmail.com)
