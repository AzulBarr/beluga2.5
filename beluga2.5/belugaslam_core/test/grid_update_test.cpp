#include "belugaslam_core/grid_update.hpp"
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <random>
#include <stdexcept>
#include <string>

using Cells = std::vector<float>;
using Endpoints = std::vector<std::pair<int, int>>;

// Independent copy of the previous allocating traversal and sort/dedup update.
static Endpoints old_ray(int x0, int y0, int x1, int y1, int width, int height) {
  Endpoints line;
  int dx = std::abs(x1 - x0), dy = std::abs(y1 - y0);
  int sx = x0 < x1 ? 1 : -1, sy = y0 < y1 ? 1 : -1;
  int err = dx - dy;
  while (true) {
    if (x0 == x1 && y0 == y1) break;
    if (x0 >= 0 && x0 < width && y0 >= 0 && y0 < height) line.emplace_back(x0, y0);
    int e2 = 2 * err;
    if (e2 > -dy) { err -= dy; x0 += sx; }
    if (e2 < dx) { err += dx; y0 += sy; }
  }
  return line;
}
static void legacy(Cells& cells, int width, int height, int ox, int oy,
                   const Endpoints& endpoints, std::vector<int>& hits, std::vector<int>& misses) {
  hits.clear(); misses.clear();
  for (const auto& [x, y] : endpoints) {
    if (x >= 0 && x < width && y >= 0 && y < height) hits.push_back(y * width + x);
    for (const auto& [rx, ry] : old_ray(ox, oy, x, y, width, height)) {
      const int index = ry * width + rx;
      if (index != oy * width + ox) misses.push_back(index);
    }
  }
  for (auto* indices : {&hits, &misses}) {
    std::sort(indices->begin(), indices->end());
    indices->erase(std::unique(indices->begin(), indices->end()), indices->end());
  }
  for (int index : hits) cells[index] = std::min(cells[index] + 1.2F, 5.0F);
  for (int index : misses) if (!std::binary_search(hits.begin(), hits.end(), index))
    cells[index] = std::max(cells[index] - 0.2F, -5.0F);
}
static void require(bool condition, const char* message) {
  if (!condition) throw std::runtime_error(message);
}
int main(int argc, char** argv) {
  belugaslam::ScanCellUpdates scratch;
  std::vector<int> hits, misses, old_hits, old_misses;
  std::mt19937 rng(42);
  std::size_t comparisons = 0;
  // Includes clipped rays, zero-length rays, all octants, repeated hits, hit/miss
  // crossings, differing grid shapes, repeated scans and saturated probabilities.
  for (int trial = 0; trial < 600; ++trial) {
    const int w = 20 + static_cast<int>(rng() % 90), h = 20 + static_cast<int>(rng() % 90);
    const int ox = static_cast<int>(rng() % w), oy = static_cast<int>(rng() % h);
    Endpoints endpoints{{ox, oy}, {ox + 1, oy}, {ox + 4, oy}, {ox + 4, oy}};
    for (int i = 0; i < 80; ++i) endpoints.emplace_back(static_cast<int>(rng() % (w + 20)) - 10,
        static_cast<int>(rng() % (h + 20)) - 10);
    Cells expected(static_cast<std::size_t>(w * h)), actual;
    for (auto& cell : expected) cell = static_cast<float>(static_cast<int>(rng() % 101) - 50) / 10.0F;
    actual = expected;
    for (int scan = 0; scan < 4; ++scan) {
      scratch.begin(actual.size()); scratch.endpoints = endpoints;
      legacy(expected, w, h, ox, oy, endpoints, old_hits, old_misses);
      belugaslam::apply_scan_cells(actual, w, h, ox, oy, 1.2F, -0.2F, 5.0F, scratch, hits, misses);
      require(actual == expected, "grid differs from previous insertion"); ++comparisons;
    }
  }
  std::cout << comparisons << " complete-grid equivalence checks passed\n";
  if (argc > 1 && std::string(argv[1]) == "--benchmark") {
    constexpr int w = 600, h = 600, count = 250;
    Endpoints endpoints;
    for (int i = 0; i < 1080; ++i) {
      const double angle = i * 6.283185307179586 / 1080;
      const double radius = 180 + 35 * std::sin(7 * angle);
      endpoints.emplace_back(300 + static_cast<int>(radius * std::cos(angle)),
                            300 + static_cast<int>(radius * std::sin(angle)));
    }
    Cells old_cells(w * h), new_cells(w * h);
    const auto start = std::chrono::steady_clock::now();
    for (int i = 0; i < count; ++i) legacy(old_cells, w, h, 300, 300, endpoints, old_hits, old_misses);
    const auto middle = std::chrono::steady_clock::now();
    for (int i = 0; i < count; ++i) {
      scratch.begin(new_cells.size()); scratch.endpoints = endpoints;
      belugaslam::apply_scan_cells(new_cells, w, h, 300, 300, 1.2F, -0.2F, 5.0F, scratch, hits, misses);
    }
    const auto end = std::chrono::steady_clock::now();
    require(old_cells == new_cells, "benchmark grids differ");
    const double old_ms = std::chrono::duration<double, std::milli>(middle - start).count() / count;
    const double new_ms = std::chrono::duration<double, std::milli>(end - middle).count() / count;
    std::cout << "Synthetic 1080-ray, 600x600 grid: legacy " << old_ms << " ms/scan; optimized "
              << new_ms << " ms/scan; ratio " << old_ms / new_ms << "x\n";
  }
}
