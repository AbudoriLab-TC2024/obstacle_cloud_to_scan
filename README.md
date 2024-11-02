# obstacle-cloud-to-scan
3次元点群から障害物を検出し、scanトピックに変換してPublishするROS2パッケージです。

## 概要

obstacle-cloud-to-scanは、LiDARセンサーからの3D点群データを処理し、ロボットにとって衝突したくない物体をLaserScanに反映し出力するROS2パッケージです。3D LiDARを使用することで、机のような細い足の障害物も適切に障害物として認識し、坂道は障害物として認識せず通行可能であると判断することができます。ナビゲーションやマッピングシステムに活用することができます。

## 特徴

### 従来の2D LiDARの課題

- ロボットに設置したスキャンラインに必ずしも障害物が存在するとは限らない（例: 机やパイロン）。
- 上り坂などでは、本来進行可能な坂道も障害物として認識してしまう。
- ロボットが乗り越え可能な段差かどうかの判断ができない。

### obstacle-cloud-to-scanでの改善

- 3D点群を使用して、ロボットの大きさに応じて衝突する可能性のある物体をスキャンに反映。
- ロボットが登坂可能な角度の坂道は障害物として認識せず、通行可能と判断。
- ロボットが乗り越え可能な些細な段差は障害物として認識しない。

## ディレクトリ構造
```
obstacle-cloud-to-scan/
  ├── CMakeLists.txt
  ├── package.xml
  ├── include/
  │   └── obstacle_cloud_to_scan/
  │       └── obstacle_cloud_to_scan_node.hpp
  ├── src/
  │   └── obstacle_cloud_to_scan_node.cpp
  ├── config/
  │   └── params.yaml
  ├── launch/
  │   └── obstacle_cloud_to_scan.launch.py
  └── README.md
```

## インストール

このリポジトリをROS2ワークスペースにクローンし、依存パッケージをインストールしてからビルドします:

```sh
# リポジトリをクローン
cd ~/ros2_ws/src
git clone https://github.com/AbudoriLab-TC2024/obstacle-cloud-to-scan.git

# 依存パッケージをインストール
sudo apt-get update
sudo apt-get install libpcl-dev ros-<ros_distro>-pcl-ros ros-<ros_distro>-pointcloud-to-laserscan

# ビルド
cd ~/ros2_ws
colcon build --packages-select obstacle-cloud-to-scan
```

## 使用方法

obstacle-cloud-to-scanノードを起動します:

```sh
ros2 launch obstacle-cloud-to-scan obstacle_cloud_to_scan.launch.py
```

### パラメータ

ノードのパラメータは `config/params.yaml` に定義されています。以下の内容を含みます:

| パラメータ名               | 説明                                          | デフォルト値                  |
| -------------------- | ------------------------------------------- | ----------------------- |
| `input_topic`        | 点群データの入力トピック名                               | /livox_cloud_in       |
| `output_topic`       | フィルタリング後の点群データの出力トピック名                      | `/filtered_point_cloud` |
| `laser_scan_topic`   | LaserScanメッセージの出力トピック名                      | `/scan`                 |
| `max_slope_angle`     | ロボットが登坂可能な角度の最大値 (度)                  | `5.0`                   |
| `voxel_leaf_size`    | ボクセルグリッドダウンサンプリングのリーフサイズ                    | `0.1`                   |
| `max_distance`       | 点群処理の最大距離                                   | `10.0`                  |
| `min_distance`       | 点群処理の最小距離                                   | `0.1`                   |
| `robot_box_size`     | ロボット自身を表す点を除去するためのバウンディングボックスのサイズ (X, Y, Z) | `[1.0, 1.0, 0.5]`       |
| `robot_box_position` | 自己フィルタリングに使用するバウンディングボックスの位置 (X, Y, Z)      | `[0.0, 0.0, 0.25]`      |
| `scan_angle_min`     | LaserScan出力の最小角度                            | `-3.14`                 |
| `scan_angle_max`     | LaserScan出力の最大角度                            | `3.14`                  |
| `scan_range_min`     | LaserScanの最小範囲                              | `0.0`                   |
| `scan_range_max`     | LaserScanの最大範囲                              | `10.0`                  |
| `use_gpu`            | 点群処理にGPUまたはCPUを選択するブールパラメータ                 | `true`                  |

## トピック

- **入力**: `/point_cloud_in` (`sensor_msgs/PointCloud2`) - 3D LiDARから点群データを受け取ります。
- **出力**:
  - `/filtered_point_cloud` (`sensor_msgs/PointCloud2`) - 障害物のみを含むフィルタリング後の点群データをパブリッシュします。
  - `/scan` (`sensor_msgs/LaserScan`) - フィルタリングされた点群から生成された2D LaserScanメッセージをパブリッシュします。


