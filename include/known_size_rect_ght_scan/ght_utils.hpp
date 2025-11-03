#pragma once
#include <vector>
#include <cmath>
#include <algorithm>
#include <cstdint>

struct Pt2 { float x, y; };
struct Pt2i { int x, y; };

inline float wrap_pi(float a){ a = std::fmod(a,(float)M_PI); return (a<0)?(a+(float)M_PI):a; }

struct RTable {
  // angle_bins 個のビン。各ビンに Δ（中心-境界点）群（メートル）を保持
  std::vector<std::vector<Pt2>> bins; // size = angle_bins
  int angle_bins{60};
};

inline int abin_of(float phi, int K){
  // phi in [0,pi)
  float t = phi / (float)M_PI * K;
  int b = (int)std::floor(t);
  if (b < 0) b = 0;
  if (b >= K) b = K-1;
  return b;
}

inline RTable build_rtable_rect(float W, float H, int angle_bins=60, int samples_per_edge=32){
  RTable rt; rt.angle_bins = angle_bins; rt.bins.assign(angle_bins, {});
  auto push = [&](float x, float y, float phi){
    int b = abin_of(phi, angle_bins);
    rt.bins[b].push_back(Pt2{-x, -y}); // Δ = center(0,0) - p
  };
  // 辺上のサンプリング
  for (int i=0;i<samples_per_edge;i++){
    float t = (samples_per_edge<=1)?0.f : (float)i/(samples_per_edge-1);
    float x = -W*0.5f + W*t;
    // 上下（法線は±y → phi=pi/2）
    push(x, -H*0.5f, (float)M_PI*0.5f);
    push(x, +H*0.5f, (float)M_PI*0.5f);
  }
  for (int i=0;i<samples_per_edge;i++){
    float t = (samples_per_edge<=1)?0.f : (float)i/(samples_per_edge-1);
    float y = -H*0.5f + H*t;
    // 左右（法線は±x → phi=0）
    push(-W*0.5f, y, 0.0f);
    push(+W*0.5f, y, 0.0f);
  }
  return rt;
}

inline void rot2(float th, float x, float y, float& ox, float& oy){
  float c=std::cos(th), s=std::sin(th);
  ox = c*x - s*y; oy = s*x + c*y;
}

inline std::vector<std::vector<std::vector<Pt2i>>>
precompute_rotated_cell_offsets(const RTable& rt, int theta_bins, float dp){
  // 角度θごと・角度ビンbごとに、Δを回して dp で割って丸めたセルオフセットを保存
  std::vector<std::vector<std::vector<Pt2i>>> out(theta_bins);
  for (int k=0;k<theta_bins;k++){
    float th = (float)M_PI * k / theta_bins;
    out[k].resize(rt.angle_bins);
    for (int b=0;b<rt.angle_bins;b++){
      const auto& vec = rt.bins[b];
      auto& vv = out[k][b];
      vv.reserve(vec.size());
      for (auto& d: vec){
        float rx, ry; rot2(th, d.x, d.y, rx, ry);
        int dx = (int)std::lround(rx / dp);
        int dy = (int)std::lround(ry / dp);
        vv.push_back(Pt2i{dx,dy});
      }
    }
  }
  return out;
}

