# Requirements and TODO

## 現在の実装状況

### 穴検知機能
- 基本的な穴検知機能は実装済み
- 静的な地面平面（z=0）を使用した光線交点計算による穴判定
- 問題点：ロボットの傾きに対応できず、誤検知・検知漏れが発生

---

## TODO: 穴検知機能の強化（動的地面平面推定）

### 目的
ロボットが傾きながら移動する環境でも、正確に穴を検知できるようにする

### 背景
- 運用環境：土の段差がある不整地と整地の中間的な環境
- ロボットの傾き：±5度程度
- 検知対象：深さ50cm以上、幅50cm〜2m程度の穴、階段
- 現在の問題：静的なz=0平面を使用しているため、ロボット移動時に意味のない処理結果になる

### 実装方針
**案A（採用）：既存の地面推定結果を活用**
- PMF/法線推定の地面除去で得られた地面点を活用
- Rolling window内の地面点からRANSACで平面推定
- 毎フレーム推定（重ければ後で調整）

### 実装内容

#### 1. 新規パラメータ追加
```yaml
# 地面平面推定パラメータ
ground_plane_rolling_window_x: 4.0       # 前方距離 [m]
ground_plane_rolling_window_y: 6.0       # 横幅 [m]
ground_plane_ransac_distance_threshold: 0.05  # RANSAC閾値 [m]
ground_plane_ransac_max_iterations: 100  # RANSAC反復回数
hole_detection_height_buffer: 0.1        # 地面より高い点のスキップバッファ [m]
```

#### 2. 新規関数追加（pcl_functions.hpp/cpp）

**Rolling window範囲フィルタ**
```cpp
pcl::PointCloud<pcl::PointXYZ>::Ptr filterRollingWindow(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    double window_x,  // 前方距離
    double window_y,  // 横幅（±window_y/2）
    rclcpp::Logger logger);
```

**RANSAC地面平面推定**
```cpp
bool estimateGroundPlaneRANSAC(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &ground_cloud,
    GroundPlane &plane,
    double distance_threshold,
    int max_iterations,
    rclcpp::Logger logger);
```

**PMFから地面点を抽出**
```cpp
pcl::PointCloud<pcl::PointXYZ>::Ptr extractGroundPointsFromPMF(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    rclcpp::Logger logger,
    int max_window_size,
    double slope,
    double initial_distance,
    double max_distance,
    double cell_size);
```

**法線ベースで地面点を抽出**
```cpp
pcl::PointCloud<pcl::PointXYZ>::Ptr extractGroundPointsFromNormals(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    const pcl::PointCloud<pcl::Normal>::Ptr &normals,
    double max_slope_angle,
    rclcpp::Logger logger);
```

**高さチェック付き穴検知（既存関数の改善）**
```cpp
pcl::PointCloud<pcl::PointXYZ>::Ptr detectHolesBasicWithHeightCheck(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
    const pcl::PointXYZ &lidar_origin,
    const GroundPlane &ground_plane,
    double ground_tolerance,
    double height_buffer,  // 新規：地面より高い点のスキップバッファ
    rclcpp::Logger logger);
```

#### 3. obstacle_cloud_to_scan.cpp 修正箇所

**pointCloudCallback()の地面除去処理部分**
- PMF/NORMALの両方で地面点を取得
- Rolling window内の地面点をフィルタ
- RANSAC平面推定を実行
- 推定した平面を穴検知に使用

#### 4. ヘッダファイル・launchファイル修正
- obstacle_cloud_to_scan.hpp: パラメータメンバ変数追加
- obstacle_cloud_to_scan.launch.py: パラメータデフォルト値設定

### 実装上の注意点

1. **地面点取得の統一化**
   - PMF/NORMAL両方で地面点を取得可能にする
   - 選択されている地面推定アルゴリズムで処理
   - PMFは既にground_indicesを計算しているので再利用

2. **シンプルな実装**
   - 地面より高い点のスキップは `if (point.z > intersection.z + height_buffer)` でシンプルに実装
   - 後で全点探索に変更しやすいようコメント追加
   - 点群のブレを考慮してバッファを設ける

3. **デバッグログ**
   - 各段階で点数をログ出力
   - RANSAC推定した平面パラメータをログ出力

### 期待される効果
- ✅ ロボットの傾きに自動対応
- ✅ 壁/障害物の誤検知を削減
- ✅ 直近の地面状況をrolling windowで反映
- ✅ PMF/NORMAL両アルゴリズム対応
- ✅ パラメータで調整可能
- ✅ 計算効率：地面点のみでRANSAC実行

### 実装ステップ

1. [ ] 新ブランチ作成（feature/dynamic-ground-plane-estimation）
2. [ ] pcl_functions.hpp - 関数宣言追加
3. [ ] pcl_functions.cpp - 新規関数実装
   - [ ] filterRollingWindow()
   - [ ] estimateGroundPlaneRANSAC()
   - [ ] extractGroundPointsFromPMF()
   - [ ] extractGroundPointsFromNormals()
   - [ ] detectHolesBasicWithHeightCheck()
4. [ ] obstacle_cloud_to_scan.hpp - パラメータメンバ変数追加
5. [ ] obstacle_cloud_to_scan.cpp - パラメータ宣言/取得、コールバック修正
6. [ ] launch/obstacle_cloud_to_scan.launch.py - パラメータデフォルト値設定
7. [ ] ビルド＆テスト
8. [ ] README.md更新（新パラメータ追加）

---

## 参考情報

### ±5度の傾きが影響する箇所
1. **RANSAC平面推定の許容誤差**
   - 5度傾くと、1m先で約87mmのずれ
   - RANSACのinlier判定閾値（distance_threshold: 0.05m）で対応

2. **穴判定の閾値**
   - hole_ground_tolerance: 0.05m
   - 傾斜により判定精度が変わる可能性

3. **地面平面の法線方向**
   - z=0平面の法線(0,0,1)から最大5度傾く
   - RANSACで実際の地面方向を推定して対応

### 既存実装の問題点
1. **静的なz=0平面**：ロボットが傾いても地面平面がz=0固定
2. **光線交点判定の限界**：地面より高い障害物も穴として誤検知される可能性
3. **base_link座標系での処理**：実際の地面との関係が考慮されていない
