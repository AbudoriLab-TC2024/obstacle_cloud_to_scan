# obstacle-cloud-to-scan
3次元点群から障害物を検出し、scanトピックに変換してPublishするROS2パッケージです。

## 概要

obstacle-cloud-to-scanは、LiDARセンサーからの3D点群データを処理し、ロボットにとって衝突したくない物体をLaserScanに反映し出力するROS2パッケージです。3D LiDARを使用することで、机のような細い足の障害物も適切に障害物として認識し、坂道は障害物として認識せず通行可能であると判断することができます。ナビゲーションやマッピングシステムに活用することができます。

![demo](https://github.com/user-attachments/assets/5d84b072-c780-40a0-92e0-30cc47587650)

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
├── LICENSE
├── README.md
├── config
│   └── params.yaml
├── include
│   └── obstacle_cloud_to_scan
│       ├── obstacle_cloud_to_scan.hpp
│       └── pcl_functions.hpp
├── launch
│   ├── filter_obstacle_cloud.launch.py
│   └── obstacle_cloud_to_scan.launch.py
├── package.xml
└── src
    ├── obstacle_cloud_to_scan.cpp
    └── pcl_functions.cpp

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

obstacle-cloud-to-scanノードを起動します。このとき、pointcloud_to_laserscanも同時に起動します。

```sh
ros2 launch obstacle-cloud-to-scan obstacle_cloud_to_scan.launch.py
```

もし、obstacle-cloud-to-scanノード単体で起動したいときはは以下のコマンドを実行します。（pointcloud_to_laserscanを起動しない）
```sh
ros2 launch obstacle-cloud-to-scan filter_obstacle_cloud.launch.py
```

### パラメータ

obstacle_cloud_to_scan ノードのパラメータ

| パラメータ名          | 型        | 説明                                               | デフォルト値    |
|----------------------|-----------|----------------------------------------------------|-----------------|
| `input_topic`        | `string`  | LiDARデータの入力トピック名                        | `/livox/lidar`  |
| `output_topic`       | `string`  | フィルタリングされたPointCloudの出力トピック名    | `/cloud_in`     |
| `voxel_leaf_size`    | `double`  | ボクセルフィルタの葉サイズ（メートル単位）          | `0.05`          |
| `robot_box_size`     | `array`   | ロボット周囲のバウンディングボックスのサイズ（[x, y, z]） | `[0.9, 0.8, 1.0]` |
| `robot_box_position` | `array`   | バウンディングボックスの位置（[x, y, z]）          | `[0.0, 0.0, 0.0]` |
| `max_slope_angle`    | `double`  | 検出する最大傾斜角度（度単位）                      | `25.0`          |
| `use_gpu`            | `bool`    | GPUを使用して処理を行うかどうか                      | `False`         |


pointcloud_to_laserscan ノードのパラメータ

| パラメータ名         | 型        | 説明                                               | デフォルト値  |
|---------------------|-----------|----------------------------------------------------|---------------|
| `target_frame`      | `string`  | LaserScanデータのフレーム名                        | `base_link`   |
| `transform_tolerance` | `double` | トランスフォームの許容時間（秒）                    | `0.01`        |
| `min_height`        | `double`  | フィルタリングするPointCloudの最小高さ（メートル）    | `-1.0`        |
| `max_height`        | `double`  | フィルタリングするPointCloudの最大高さ（メートル）    | `2.0`         |
| `angle_min`         | `double`  | スキャンの開始角度（ラジアン）                       | `-1.5708`     |
| `angle_max`         | `double`  | スキャンの終了角度（ラジアン）                       | `1.5708`      |
| `angle_increment`   | `double`  | スキャンの角度増分（ラジアン）                       | `0.0174`      |
| `scan_time`         | `double`  | スキャンの時間（秒）                                 | `0.1`         |
| `range_min`         | `double`  | レンジの最小値（メートル）                            | `0.1`         |
| `range_max`         | `double`  | レンジの最大値（メートル）                            | `40.0`        |
| `use_inf`           | `bool`    | 無限大を使用して無効なデータポイントを表現するかどうか  | `True`        |
| `inf_epsilon`       | `double`  | 無限大の補正値                                       | `1.0`         |


## トピック

- **入力**: `/point_cloud_in` (`sensor_msgs/PointCloud2`) - 3D LiDARから点群データを受け取ります。
- **出力**:
  - `/filtered_point_cloud` (`sensor_msgs/PointCloud2`) - 障害物のみを含むフィルタリング後の点群データをパブリッシュします。
  - `/scan` (`sensor_msgs/LaserScan`) - フィルタリングされた点群から生成された2D LaserScanメッセージをパブリッシュします。


