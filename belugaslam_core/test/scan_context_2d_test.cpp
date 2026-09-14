#include "belugaslam_core/scan_context_2d.hpp"
#include <cmath>
#include <iostream>
#include <vector>

namespace {
struct Grid {
  int width_=160, height_=160; double resolution_=0.1, ox_=-8.0, oy_=-8.0;
  std::vector<float> data_=std::vector<float>(static_cast<std::size_t>(width_*height_), -1.0F);
  int width() const { return width_; } int height() const { return height_; }
  double resolution() const { return resolution_; } double origin_x() const { return ox_; } double origin_y() const { return oy_; }
  float at(int x,int y) const { return data_[static_cast<std::size_t>(y*width_+x)]; }
  void occupied(double x,double y) {
    const int gx=static_cast<int>(std::floor((x-ox_)/resolution_));
    const int gy=static_cast<int>(std::floor((y-oy_)/resolution_));
    if(gx>=0&&gx<width_&&gy>=0&&gy<height_) data_[static_cast<std::size_t>(gy*width_+gx)]=1.0F;
  }
};
void wall(Grid& g,double angle) {
  for(double r=1.0;r<7.0;r+=0.05) g.occupied(r*std::cos(angle),r*std::sin(angle));
}
int fail(const char* m){std::cerr<<m<<'\n';return 1;}
}
int main(){
  Grid a,b,c;
  wall(a,0.25); wall(a,1.10); wall(a,-2.20);
  const double rotation=0.8;
  wall(b,0.25-rotation); wall(b,1.10-rotation); wall(b,-2.20-rotation);
  wall(c,-0.7); wall(c,2.4);
  const auto da=belugaslam::make_scan_context_2d(a,20,60,8.0);
  const auto db=belugaslam::make_scan_context_2d(b,20,60,8.0);
  const auto dc=belugaslam::make_scan_context_2d(c,20,60,8.0);
  const auto same=belugaslam::match_scan_context_2d(da,db,1);
  const auto different=belugaslam::match_scan_context_2d(da,dc,1);
  if(!da.valid()||!db.valid()) return fail("descriptor invalid");
  if(same.distance>=different.distance) return fail("rotated revisit did not outrank different place");
  if(std::abs(std::remainder(same.yaw-rotation,2.0*3.14159265358979323846))>0.15)
    return fail("yaw shift estimate is wrong");
  if(std::abs(belugaslam::scan_context_ring_key_distance(da,db))>0.03)
    return fail("ring key is not approximately rotation invariant");
  return 0;
}
