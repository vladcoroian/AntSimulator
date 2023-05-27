#pragma once
#include <iostream>
#include <cmath>
#include <SFML/Graphics.hpp>
#include "editor/GUI/container.hpp"
#include "editor/GUI/button.hpp"

#include <vector>
#include <algorithm>

namespace edtr {

struct Room {
  int id;
  sf::Vector2i pos;
  int32_t width;
  int32_t height;
  int vx, vy;
  bool close_to_x_boundary = false;
  bool close_to_y_boundary = false;

  bool isOverlapping(const Room& other) const {
    const auto& pos1 = pos;
    const auto& pos2 = other.pos;
    const auto& size1 = sf::Vector2i{width, height};
    const auto& size2 = sf::Vector2i{other.width, other.height};
    int left1 = pos1.x - size1.x / 2;
    int right1 = pos1.x + size1.x / 2;
    int top1 = pos1.y - size1.y / 2;
    int bottom1 = pos1.y + size1.y / 2;

    int left2 = pos2.x - size2.x / 2;
    int right2 = pos2.x + size2.x / 2;
    int top2 = pos2.y - size2.y / 2;
    int bottom2 = pos2.y + size2.y / 2;

    if (left1 > right2 || left2 > right1) return false;
    if (top1 > bottom2 || top2 > bottom1) return false;

    return true;
  }

  sf::Vector2i distance(const Room& ret2) const {
    auto ret1 = *this;
    sf::Vector2i translation{0, 0};
    double overlapX = 1 + std::min(ret1.pos.x + ret1.width / 2, ret2.pos.x + ret2.width / 2) -
                      std::max(ret1.pos.x - ret1.width / 2, ret2.pos.x - ret2.width / 2);
    double overlapY = 1 + std::min(ret1.pos.y + ret1.height / 2, ret2.pos.y + ret2.height / 2) -
                      std::max(ret1.pos.y - ret1.height / 2, ret2.pos.y - ret2.height / 2);

    if (overlapX < 0.0 || overlapY < 0.0) {
      // The rectangles do not overlap
      return translation;
    }

    if (overlapX < overlapY && !ret1.close_to_x_boundary || ret1.close_to_y_boundary) {
      // The rectangles overlap more in the x-direction
      translation.x = ret1.pos.x < ret2.pos.x ? -overlapX : overlapX;
    } else {
      // The rectangles overlap more in the y-direction
      translation.y = ret1.pos.y < ret2.pos.y ? -overlapY : overlapY;
    }

    return translation;
  }

  double distanceTo(const Room& other) const {
    return std::sqrt((pos.x - other.pos.x) * (pos.x - other.pos.x) +
                     (pos.y - other.pos.y) * (pos.y - other.pos.y));
  }

  bool operator==(const Room& other) const {
    return pos == other.pos && width == other.width && height == other.height;
  }

  bool operator!=(const Room& other) const { return !(*this == other); }
};
}  // namespace edtr