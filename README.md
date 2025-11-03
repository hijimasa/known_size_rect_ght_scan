# known_size_rect_ght_scan

## 概要

このパッケージは、2D LiDARセンサ（`sensor_msgs/LaserScan`）から得られる点群データを用い、**一般化ハフ変換（Generalized Hough Transform, GHT）**によって**既知のサイズの長方形**を検出するROS 2ノードです。

検出した長方形の位置・姿勢を可視化マーカー（RViz）およびPoseとして出力します。

---

## 一般化ハフ変換（GHT）とは？

### 通常のハフ変換

通常のハフ変換は、画像中の**直線**や**円**などの単純な形状を検出する手法です。例えば、直線を検出する場合、各エッジ点から考えられるすべての直線パラメータ（傾き・切片など）に「投票」を行い、最も多くの投票を集めた箇所が直線であると判定します。

### 一般化ハフ変換（GHT）

一般化ハフ変換は、**任意の形状**を検出できるように拡張した手法です。通常のハフ変換が「直線や円」に限定されるのに対し、GHTは以下のような特徴があります。

- **形状モデルを事前に定義**：検出したい形状（今回は長方形）の「輪郭点」と「中心からのベクトル（Δ）」および「法線方向」を記録した**R-table（参照テーブル）**を作成します。
- **法線方向を利用した投票**：点群データから各点の法線方向（壁面の向き）を推定し、それに対応するR-tableのΔベクトルを参照して「形状の中心がどこにあるか」に投票します。
- **累積空間でのピーク検出**：多くの点が一致する位置（投票数が多い位置）が、形状の中心位置として検出されます。

---

## このパッケージでのGHTの適用方法

### 1. **長方形モデルの構築（R-table）**

検出対象は**既知のサイズ（幅W×高さH）の長方形**です。

- 長方形の4辺上に均等にサンプル点を配置します（パラメータ: `samples_per_edge`）。
- 各サンプル点について、以下を記録します：
  - 法線方向 φ（壁面の向き）
  - 中心からのベクトル Δ = (中心座標) - (サンプル点座標)
  
これらの情報を**角度ビン**（`angle_bins`個）に分類してR-tableとして保存します。

**コード例**（`ght_utils.hpp`）:
```cpp
inline RTable build_rtable_rect(float W, float H, int angle_bins=60, int samples_per_edge=32)
```

### 2. **LiDAR点群からの法線推定**

LiDARの各ビームから得られる点について、隣接点との差分（中央差分法）を用いて**接線ベクトル**を計算し、それに直交する**法線ベクトル**を求めます。

- **法線推定ウィンドウ**（`normal_window`）：前後数ビームを使って法線を計算
- **ジャンプ閾値**（`jump_threshold`）：大きな距離変化がある箇所（角や遮蔽端）は除外

これにより、各点に「位置(x, y)」と「法線角度 φ'」が付与されます。

### 3. **累積空間（Accumulator）への投票**

累積空間は **(x, y, θ)** の3次元空間です：

- **(x, y)**：長方形の中心位置候補
- **θ**：長方形の姿勢（回転角度）

各点について、以下の手順で投票を行います：

1. 姿勢 θ を仮定する（0 から π まで `theta_bins` 個に分割）
2. 点の法線角 φ' とモデルの姿勢 θ から、相対角度 φ = φ' - θ を計算
3. R-tableから相対角度 φ に対応するΔベクトルを取得
4. Δベクトルを姿勢 θ で回転させ、点の位置に加算して「中心位置候補」を得る
5. その中心位置候補 (x, y, θ) の累積空間のセルに+1票を投じる

**コード例**（`known_size_rect_ght_scan_node.cpp`）:
```cpp
for (const auto &q : pts) {
  for (int k = 0; k < theta_bins_; k++) {
    float th = (float)M_PI * k / theta_bins_;
    float rel = q.phi - th; // 相対角度
    int b = abin_of(rel, angle_bins_);
    const auto &deltas = rotated_cell_offsets_[k][b];
    // ... 投票処理 ...
  }
}
```

### 4. **ピーク検出（NMS）**

累積空間の中で、投票数が局所的に最大となる位置を**Non-Maximum Suppression (NMS)** によって検出します。

- 3次元的に近傍セル（xy方向：`nms_xy_cells`, θ方向：`nms_theta_bins`）と比較
- 自身が最大値であれば候補として保存
- 投票数が多い順に上位K個（`topk_candidates`）を選択

### 5. **候補の評価とスコアリング**

各候補について、実際に長方形を当てはめて「どれだけ点群がその辺に沿っているか」を評価します。

- 長方形の4辺それぞれについて、辺の近傍（`edge_distance_thresh`以内）に存在する点の数をカウント
- 辺の両端（`edge_margin`）は除外して、中央部分のみを評価
- 合計点数が最も高い候補を「検出結果」として採用

**スコア閾値**（`min_edge_points`）以下の場合は検出失敗とします。

### 6. **結果の出力**

検出された長方形について以下を出力します：

- **可視化マーカー**（RViz）：
  - 長方形の輪郭（緑色のLINE_STRIP）
  - 中心位置（赤い球）
  - 姿勢を示す矢印（青い矢印）
- **Poseメッセージ**（`geometry_msgs/PoseStamped`）：
  - 中心位置と回転角度（quaternion）

---

## サイズを既知にすることによる次元削減

### 通常のGHTの問題点

一般的なGHTでは、**形状のスケール（大きさ）も未知パラメータ**として扱うため、累積空間が **(x, y, θ, スケール)** の**4次元**になります。

- **計算コスト**：累積空間のサイズが膨大になり、メモリと計算時間が爆発的に増加
- **ピーク検出の困難さ**：次元が増えると、ノイズの影響を受けやすく、正しいピークを見つけにくくなる

### 既知サイズによる次元削減

このパッケージでは、**長方形のサイズ（幅W×高さH）が既知**であるという前提を利用しています。

- 累積空間は **(x, y, θ)** の**3次元**のみ
- スケールパラメータを探索する必要がないため、**計算量が大幅に削減**
- **リアルタイム処理**が可能になる

**具体的な効果**：

- 累積空間のサイズが1次元分削減される（例：100×100×90 ≈ 90万セル vs 100×100×90×20 = 1800万セル）
- 投票回数が減少（スケールごとに繰り返す必要がない）
- メモリ使用量とCPU負荷が軽減

**事前計算の活用**：

サイズが既知であるため、R-tableと回転後のセルオフセットを**起動時に一度だけ計算**し、以降は再利用できます。

```cpp
rt_ = build_rtable_rect((float)known_size_rect_w_, (float)known_size_rect_h_, 
                        angle_bins_, samples_per_edge_);
rotated_cell_offsets_ = precompute_rotated_cell_offsets(rt_, theta_bins_, (float)dp_);
```

---

## ノードの使い方

### ビルド

```bash
cd ~/irlab_ws/Unity_ROS2_sample/colcon_ws
colcon build --packages-select known_size_rect_ght_scan
source install/setup.bash
```

### 実行

launchファイルを使用：

```bash
ros2 launch known_size_rect_ght_scan known_size_rect_ght_scan.launch.py
```

または直接ノードを起動：

```bash
ros2 run known_size_rect_ght_scan known_size_rect_ght_scan_node --ros-args \
  -p scan_topic:=/diffbot/lidar_link/scan \
  -p known_size_rect_width:=8.0 \
  -p known_size_rect_height:=12.0
```

### RVizでの可視化

RVizで以下のトピックを追加してください：

- **MarkerArray**: `/known_size_rect_markers`（長方形の輪郭・中心・姿勢矢印）
- **LaserScan**: `/diffbot/lidar_link/scan`（元のLiDARデータ）

---

## パラメータ一覧

### 基本パラメータ

| パラメータ名 | 型 | デフォルト値 | 説明 |
|---|---|---|---|
| `scan_topic` | string | `/scan` | 購読するLaserScanトピック名 |
| `known_size_rect_width` | double | 5.0 | 検出対象の長方形の幅 [m] |
| `known_size_rect_height` | double | 4.0 | 検出対象の長方形の高さ [m] |
| `max_range` | double | 30.0 | 使用する最大測距範囲 [m] |

### GHTアルゴリズムパラメータ

| パラメータ名 | 型 | デフォルト値 | 説明 |
|---|---|---|---|
| `accum_grid_res` | double | 0.10 | 累積空間の解像度（1セル当たりのメートル数） |
| `theta_bins` | int | 90 | 姿勢角θの分割数（0～π） |
| `angle_bins` | int | 60 | R-tableの法線角φの分割数 |
| `samples_per_edge` | int | 24 | 長方形の各辺に配置するサンプル点の数 |

### 点群処理パラメータ

| パラメータ名 | 型 | デフォルト値 | 説明 |
|---|---|---|---|
| `beam_step` | int | 1 | ビームの間引き（1=全ビーム使用） |
| `normal_window` | int | 3 | 法線推定に使う前後のビーム数 |
| `jump_threshold` | double | 0.5 | 不連続エッジ判定の距離閾値 [m] |

### ピーク検出・評価パラメータ

| パラメータ名 | 型 | デフォルト値 | 説明 |
|---|---|---|---|
| `nms_xy_cells` | int | 2 | NMSのxy方向窓サイズ（セル数） |
| `nms_theta_bins` | int | 2 | NMSのθ方向窓サイズ（ビン数） |
| `topk_candidates` | int | 5 | 評価する上位候補数 |
| `edge_distance_thresh` | double | 0.08 | 辺に属すると判定する距離閾値 [m] |
| `edge_margin` | double | 0.10 | 辺の両端から除外する範囲 [m] |
| `min_edge_points` | int | 50 | 検出に必要な最小辺点数 |

---

## 出力トピック

| トピック名 | 型 | 説明 |
|---|---|---|
| `/known_size_rect_markers` | `visualization_msgs/MarkerArray` | 検出された長方形の可視化マーカー |
| `/known_size_rect_pose` | `geometry_msgs/PoseStamped` | 長方形の中心位置と姿勢 |

---

## アルゴリズムの流れ（まとめ）

1. **R-table構築**：既知サイズの長方形モデルから、法線角とΔベクトルの対応表を作成
2. **法線推定**：LiDAR点群から各点の法線方向を計算
3. **投票**：各点・各姿勢について、R-tableを参照して累積空間に投票
4. **ピーク検出**：NMSで投票数が多い位置・姿勢を候補として抽出
5. **スコアリング**：各候補に長方形を当てはめ、辺近傍の点数で評価
6. **出力**：最も高スコアの候補を検出結果として可視化・パブリッシュ

---

## 参考文献

- Ballard, D. H. (1981). "Generalizing the Hough transform to detect arbitrary shapes." Pattern Recognition, 13(2), 111-122.
- GHTの詳細については、コンピュータビジョンの教科書や論文を参照してください。

---

## ライセンス

MIT License

---

## メンテナ

- Masaaki HIJIKATA (hijimasa@gmail.com)

---

## トラブルシューティング

### 検出されない場合

- `max_range`が小さすぎないか確認
- `known_size_rect_width/height`が実際の環境と一致しているか確認
- `min_edge_points`の閾値を下げてみる
- RVizでLiDARデータを確認し、長方形の輪郭が明瞭に取得できているか確認

### 誤検出が多い場合

- `edge_distance_thresh`を小さくして厳密に評価
- `min_edge_points`を上げて閾値を厳しくする
- `nms_xy_cells`や`nms_theta_bins`を大きくして近接ピークを抑制

### 処理が重い場合

- `beam_step`を2以上にして点群を間引く
- `theta_bins`を減らして姿勢の探索を粗くする
- `accum_grid_res`を大きくして累積空間の解像度を下げる

---

このパッケージを使って、LiDARセンサから既知サイズの長方形を効率的に検出しましょう！
