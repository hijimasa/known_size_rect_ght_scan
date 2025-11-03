#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "known_size_rect_ght_scan/ght_utils.hpp"
#include <numeric>

class KnownSizeRectGHTNode : public rclcpp::Node
{
public:
  KnownSizeRectGHTNode() : Node("known_size_rect_ght_scan_node")
  {
    // 基本パラメータ
    scan_topic_ = declare_parameter<std::string>("scan_topic", "/scan");
    known_size_rect_w_ = declare_parameter("known_size_rect_width", 5.0);
    known_size_rect_h_ = declare_parameter("known_size_rect_height", 4.0);
    size_tol_ratio_ = declare_parameter("size_tol_ratio", 0.08);   // 検証時の幅/高さ許容
    dp_ = declare_parameter("accum_grid_res", 0.10);               // [m/セル]
    theta_bins_ = declare_parameter("theta_bins", 90);             // [0,pi)
    angle_bins_ = declare_parameter("angle_bins", 60);             // R-tableの角度ビン
    samples_per_edge_ = declare_parameter("samples_per_edge", 24); // R-tableサンプル密度
    beam_step_ = declare_parameter("beam_step", 1);                // ビーム間引き
    normal_win_ = declare_parameter("normal_window", 3);           // 法線推定の隣接幅（ビーム数）
    jump_thresh_ = declare_parameter("jump_threshold", 0.5);       // 法線推定での両側差分[ m ]
    max_range_clip_ = declare_parameter("max_range", 30.0);
    // 投票・ピーク/評価
    nms_xy_ = declare_parameter("nms_xy_cells", 2);   // NMS窓（xy）
    nms_th_ = declare_parameter("nms_theta_bins", 2); // NMS窓（θ）
    topk_ = declare_parameter("topk_candidates", 5);
    edge_dist_th_ = declare_parameter("edge_distance_thresh", 0.08); // 辺距離[ m ]
    edge_margin_ = declare_parameter("edge_margin", 0.10);           // 端の除外[ m ]
    min_edge_points_ = declare_parameter("min_edge_points", 50);     // 採用閾値
    frame_override_ = declare_parameter<std::string>("frame_id", "");

    pub_mk_ = create_publisher<visualization_msgs::msg::MarkerArray>("known_size_rect_markers", 1);
    pub_pose_ = create_publisher<geometry_msgs::msg::PoseStamped>("known_size_rect_pose", 1);

    sub_ = create_subscription<sensor_msgs::msg::LaserScan>(
        scan_topic_, rclcpp::SensorDataQoS(),
        std::bind(&KnownSizeRectGHTNode::onScan, this, std::placeholders::_1));

    // モデルR-tableと回転済みセルオフセットを準備（dpが既知なので起動時に前計算）
    rt_ = build_rtable_rect((float)known_size_rect_w_, (float)known_size_rect_h_, angle_bins_, samples_per_edge_);
    // rotated_cell_offsets_[theta][bin] -> dx,dy (セル単位)
    rotated_cell_offsets_ = precompute_rotated_cell_offsets(rt_, theta_bins_, (float)dp_);

    RCLCPP_INFO(get_logger(), "GHT rect node ready. scan=%s W=%.2f H=%.2f dp=%.2f theta_bins=%d",
                scan_topic_.c_str(), known_size_rect_w_, known_size_rect_h_, dp_, theta_bins_);
  }

private:
  void onScan(const sensor_msgs::msg::LaserScan::SharedPtr msg)
  {
    const std::string frame = frame_override_.empty() ? msg->header.frame_id : frame_override_;

    // ---- 1) Scan → XY配列（全ビーム）と有効フラグ ----
    const int N = (int)msg->ranges.size();
    const float a0 = msg->angle_min;
    const float da = msg->angle_increment;
    const float rmin = std::max(0.0f, msg->range_min);
    const float rmax = std::min((float)max_range_clip_, msg->range_max > 0 ? msg->range_max : (float)max_range_clip_);

    std::vector<Pt2> p_all(N);
    std::vector<char> valid(N, 0);
    float minx = 1e9f, miny = 1e9f, maxx = -1e9f, maxy = -1e9f;
    for (int i = 0; i < N; i++)
    {
      float r = msg->ranges[i];
      if (std::isfinite(r) && r >= rmin && r <= rmax)
      {
        float th = a0 + i * da;
        float x = r * std::cos(th), y = r * std::sin(th);
        p_all[i] = Pt2{x, y};
        valid[i] = 1;
        minx = std::min(minx, x);
        maxx = std::max(maxx, x);
        miny = std::min(miny, y);
        maxy = std::max(maxy, y);
      }
    }
    // 可視領域を少し拡張
    float pad = 1.0f;
    minx -= pad;
    miny -= pad;
    maxx += pad;
    maxy += pad;
    if (!(minx < maxx && miny < maxy))
    {
      publishClear(frame, msg->header.stamp);
      return;
    }

    // ---- 2) 法線角 phi' の推定（中央差分） + 投票対象点の抽出 ----
    struct Pn
    {
      Pt2 p;
      float phi;
    };
    std::vector<Pn> pts;
    pts.reserve(N / beam_step_ + 1);
    for (int i = normal_win_; i < N - normal_win_; i += std::max(1, beam_step_))
    {
      if (!valid[i])
        continue;
      int L = i - normal_win_, R = i + normal_win_;
      if (!valid[L] || !valid[R])
        continue;
      // 角の不連続/遮蔽端の抑制
      float dl = std::hypot(p_all[i].x - p_all[L].x, p_all[i].y - p_all[L].y);
      float dr = std::hypot(p_all[R].x - p_all[i].x, p_all[R].y - p_all[i].y);
      if (dl > jump_thresh_ || dr > jump_thresh_)
        continue;

      float tx = p_all[R].x - p_all[L].x;
      float ty = p_all[R].y - p_all[L].y;
      float tl = std::hypot(tx, ty);
      if (tl < 1e-6f)
        continue;
      tx /= tl;
      ty /= tl;                       // 接線
      float nx = -ty, ny = tx;        // 法線
      float phi = std::atan2(ny, nx); // (-pi,pi] → [0,pi)
      phi = wrap_pi(phi);
      pts.push_back({p_all[i], phi});
    }

    if (pts.size() < 80)
    {
      publishClear(frame, msg->header.stamp);
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "Too few oriented points: %zu", pts.size());
      return;
    }

    // ---- 3) 累積空間 (x,y,theta) を用意 ----
    const int Wc = std::max(1, (int)std::ceil((maxx - minx) / dp_));
    const int Hc = std::max(1, (int)std::ceil((maxy - miny) / dp_));
    // acc[k][y][x] フラット化: k*(Hc*Wc) + y*Wc + x
    const size_t slice = (size_t)Hc * (size_t)Wc;
    std::vector<uint32_t> acc((size_t)theta_bins_ * slice, 0u);

    auto cell_of = [&](const Pt2 &p, int &xi, int &yi) -> bool
    {
      xi = (int)std::lround((p.x - minx) / dp_);
      yi = (int)std::lround((p.y - miny) / dp_);
      return (xi >= 0 && xi < Wc && yi >= 0 && yi < Hc);
    };

    // ---- 4) 投票 ----
    for (const auto &q : pts)
    {
      for (int k = 0; k < theta_bins_; k++)
      {
        float th = (float)M_PI * k / theta_bins_;
        // モデル側角度ビン b = phi' - theta
        float rel = q.phi - th;
        rel = wrap_pi(rel);
        int b = abin_of(rel, angle_bins_);
        const auto &deltas = rotated_cell_offsets_[k][b]; // セル単位のΔ
        // q のセル座標
        int qx, qy;
        if (!cell_of(q.p, qx, qy))
          continue;
        uint32_t *base = &acc[(size_t)k * slice];
        for (const auto &d : deltas)
        {
          int xi = qx + d.x;
          int yi = qy + d.y;
          if ((unsigned)xi < (unsigned)Wc && (unsigned)yi < (unsigned)Hc)
          {
            base[(size_t)yi * Wc + (size_t)xi] += 1u;
          }
        }
      }
    }

    // ---- 5) 3D NMSでピーク候補を収集 ----
    struct Node
    {
      uint32_t v;
      int xi, yi, k;
    };
    std::vector<Node> nodes;
    nodes.reserve(1024);
    auto at = [&](int x, int y, int k) -> uint32_t
    {
      return acc[(size_t)k * slice + (size_t)y * Wc + (size_t)x];
    };
    for (int k = 0; k < theta_bins_; k++)
    {
      for (int y = nms_xy_; y < Hc - nms_xy_; ++y)
      {
        for (int x = nms_xy_; x < Wc - nms_xy_; ++x)
        {
          uint32_t v = at(x, y, k);
          if (!v)
            continue;
          bool ok = true;
          for (int dk = -nms_th_; dk <= nms_th_ && ok; ++dk)
          {
            int kk = k + dk;
            if (kk < 0)
              kk += theta_bins_;
            if (kk >= theta_bins_)
              kk -= theta_bins_;
            for (int dy = -nms_xy_; dy <= nms_xy_ && ok; ++dy)
            {
              for (int dx = -nms_xy_; dx <= nms_xy_; ++dx)
              {
                if (dx == 0 && dy == 0 && dk == 0)
                  continue;
                if (at(x + dx, y + dy, kk) > v)
                  ok = false;
              }
            }
          }
          if (ok)
            nodes.push_back({v, x, y, k});
        }
      }
    }
    std::sort(nodes.begin(), nodes.end(),
              [](const Node &a, const Node &b)
              { return a.v > b.v; });
    if ((int)nodes.size() > topk_)
      nodes.resize(topk_);

    if (nodes.empty())
    {
      publishClear(frame, msg->header.stamp);
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "No peaks in accumulator.");
      return;
    }

    // ---- 6) 候補評価（辺距離でスコア）→最良を選択 ----
    struct Cand
    {
      float cx, cy, th;
      float score;
    };
    Cand best{0, 0, 0, -1};
    auto score_rect = [&](float cx, float cy, float th) -> float
    {
      // 四隅
      float c = std::cos(th), s = std::sin(th);
      auto rot = [&](float x, float y)
      { return Pt2{c * x - s * y, s * x + c * y}; };
      Pt2 o = {cx, cy};
      Pt2 e1 = rot(+known_size_rect_w_ / 2, 0), e2 = rot(0, +known_size_rect_h_ / 2);
      Pt2 v0 = {o.x + e1.x + e2.x, o.y + e1.y + e2.y};
      Pt2 v1 = {o.x - e1.x + e2.x, o.y - e1.y + e2.y};
      Pt2 v2 = {o.x - e1.x - e2.x, o.y - e1.y - e2.y};
      Pt2 v3 = {o.x + e1.x - e2.x, o.y + e1.y - e2.y};

      auto acc_edge = [&](const Pt2 &s, const Pt2 &e)
      {
        float ex = e.x - s.x, ey = e.y - s.y;
        float L = std::max(1e-3f, std::hypot(ex, ey));
        float ux = ex / L, uy = ey / L;
        float nx = -uy, ny = ux;
        int cnt = 0;
        for (const auto &q : pts)
        {
          float d = std::fabs(nx * (q.p.x - s.x) + ny * (q.p.y - s.y));
          if (d > edge_dist_th_)
            continue;
          float t = ux * (q.p.x - s.x) + uy * (q.p.y - s.y);
          if (t < edge_margin_ || t > (L - edge_margin_))
            continue;
          cnt++;
        }
        return cnt;
      };
      int cnt = 0;
      cnt += acc_edge(v0, v1);
      cnt += acc_edge(v1, v2);
      cnt += acc_edge(v2, v3);
      cnt += acc_edge(v3, v0);
      return (float)cnt;
    };

    for (const auto &nd : nodes)
    {
      float cx = minx + nd.xi * dp_;
      float cy = miny + nd.yi * dp_;
      float th = (float)M_PI * nd.k / theta_bins_;
      float sc = score_rect(cx, cy, th);
      if (sc > best.score)
      {
        best = Cand{cx, cy, th, sc};
      }
    }

    if (best.score < min_edge_points_)
    {
      publishClear(frame, msg->header.stamp);
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 3000, "Low rectangle score: %.0f", best.score);
      return;
    }

    // ---- 7) RViz描画 & Pose出力 ----
    publishResult(best, frame, msg->header.stamp);
  }

  void publishResult(const auto &c, const std::string &frame, const rclcpp::Time &stamp)
  {
    visualization_msgs::msg::MarkerArray ma;

    // 四隅
    float cth = std::cos(c.th), sth = std::sin(c.th);
    auto rot = [&](float x, float y)
    { return Pt2{cth * x - sth * y, sth * x + cth * y}; };
    Pt2 o{c.cx, c.cy};
    Pt2 e1 = rot((float)known_size_rect_w_ / 2, 0), e2 = rot(0, (float)known_size_rect_h_ / 2);
    Pt2 v0 = {o.x + e1.x + e2.x, o.y + e1.y + e2.y};
    Pt2 v1 = {o.x - e1.x + e2.x, o.y - e1.y + e2.y};
    Pt2 v2 = {o.x - e1.x - e2.x, o.y - e1.y - e2.y};
    Pt2 v3 = {o.x + e1.x - e2.x, o.y + e1.y - e2.y};

    auto pt = [](const Pt2 &p)
    { geometry_msgs::msg::Point q; q.x=p.x; q.y=p.y; q.z=0.0; return q; };

    visualization_msgs::msg::Marker poly;
    poly.header.frame_id = frame;
    poly.header.stamp = stamp;
    poly.ns = "known_size_rect";
    poly.id = 0;
    poly.type = visualization_msgs::msg::Marker::LINE_STRIP;
    poly.action = visualization_msgs::msg::Marker::ADD;
    poly.scale.x = 0.05;
    poly.color.r = 0.1f;
    poly.color.g = 0.9f;
    poly.color.b = 0.1f;
    poly.color.a = 1.0f;
    poly.points.push_back(pt(v0));
    poly.points.push_back(pt(v1));
    poly.points.push_back(pt(v2));
    poly.points.push_back(pt(v3));
    poly.points.push_back(pt(v0));
    ma.markers.push_back(poly);

    visualization_msgs::msg::Marker ctr;
    ctr.header.frame_id = frame;
    ctr.header.stamp = stamp;
    ctr.ns = "known_size_rect";
    ctr.id = 1;
    ctr.type = visualization_msgs::msg::Marker::SPHERE;
    ctr.action = visualization_msgs::msg::Marker::ADD;
    ctr.scale.x = ctr.scale.y = ctr.scale.z = 0.15;
    ctr.color.r = 0.9f;
    ctr.color.g = 0.2f;
    ctr.color.b = 0.2f;
    ctr.color.a = 0.9f;
    ctr.pose.position.x = c.cx;
    ctr.pose.position.y = c.cy;
    ctr.pose.position.z = 0.0;
    ma.markers.push_back(ctr);

    visualization_msgs::msg::Marker arr;
    arr.header.frame_id = frame;
    arr.header.stamp = stamp;
    arr.ns = "known_size_rect";
    arr.id = 2;
    arr.type = visualization_msgs::msg::Marker::ARROW;
    arr.action = visualization_msgs::msg::Marker::ADD;
    arr.scale.x = 0.5;
    arr.scale.y = 0.1;
    arr.scale.z = 0.1;
    arr.color.r = 0.2f;
    arr.color.g = 0.4f;
    arr.color.b = 1.0f;
    arr.color.a = 0.9f;
    {
      geometry_msgs::msg::Point p0, p1;
      p0.x = c.cx;
      p0.y = c.cy;
      p0.z = 0.0;
      double L = std::max(known_size_rect_w_, known_size_rect_h_) * 0.5;
      p1 = p0;
      p1.x += L * std::cos(c.th);
      p1.y += L * std::sin(c.th);
      arr.points.push_back(p0);
      arr.points.push_back(p1);
    }
    ma.markers.push_back(arr);

    pub_mk_->publish(ma);

    geometry_msgs::msg::PoseStamped pose;
    pose.header.frame_id = frame;
    pose.header.stamp = stamp;
    pose.pose.position.x = c.cx;
    pose.pose.position.y = c.cy;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = std::sin(c.th * 0.5);
    pose.pose.orientation.w = std::cos(c.th * 0.5);
    pub_pose_->publish(pose);
  }

  void publishClear(const std::string &frame, const rclcpp::Time &stamp)
  {
    visualization_msgs::msg::MarkerArray ma;
    for (int id = 0; id < 3; ++id)
    {
      visualization_msgs::msg::Marker m;
      m.header.frame_id = frame;
      m.header.stamp = stamp;
      m.ns = "known_size_rect";
      m.id = id;
      m.action = visualization_msgs::msg::Marker::DELETE;
      ma.markers.push_back(m);
    }
    pub_mk_->publish(ma);
  }

private:
  // params
  std::string scan_topic_, frame_override_;
  int theta_bins_, angle_bins_, samples_per_edge_, beam_step_, normal_win_;
  double known_size_rect_w_, known_size_rect_h_, size_tol_ratio_, dp_, jump_thresh_, max_range_clip_;
  int nms_xy_, nms_th_, topk_;
  double edge_dist_th_, edge_margin_;
  int min_edge_points_;

  // model
  RTable rt_;
  std::vector<std::vector<std::vector<Pt2i>>> rotated_cell_offsets_;

  // io
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr sub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_mk_;
  rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pub_pose_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KnownSizeRectGHTNode>());
  rclcpp::shutdown();
  return 0;
}
