#pragma once
#include <iostream>
#include <cmath>
#include "editor/GUI/container.hpp"
#include "editor/GUI/button.hpp"

#include <vector>
#include <algorithm>

namespace edtr {

struct Room {
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
    double overlapX = 1 + std::min(ret1.pos.x + ret1.width / 2, ret2.pos.x + ret2.width / 2) - std::max(ret1.pos.x - ret1.width/2, ret2.pos.x - ret2.width/2);
    double overlapY = 1 + std::min(ret1.pos.y + ret1.height / 2, ret2.pos.y + ret2.height / 2) - std::max(ret1.pos.y - ret1.height/2, ret2.pos.y - ret2.height/2);

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

  bool operator==(const Room& other) const {
    return pos == other.pos && width == other.width && height == other.height;
  }

  bool operator!=(const Room& other) const { return !(*this == other); }
};

struct MapGenerator : public GUI::NamedContainer {
  Simulation& simulation;
  ControlState& control_state;
  int32_t brush_size = 1;
  SPtr<ToolOption> generate_button;
  SPtr<ToolOption> reset_map_button;
  std::vector<Room> rooms;

  explicit MapGenerator(Simulation& sim, ControlState& control_state_)
      : GUI::NamedContainer("Map Generator",
                            Container::Orientation::Horizontal),
        simulation(sim),
        control_state(control_state_) {
    root->setHeight(30.0f, GUI::Size::Fixed);
    generate_button =
        create<ToolOption>("Generate", [this]() { this->createMap(); });
    reset_map_button = create<ToolOption>(
        "Reset Map", [this]() { this->simulation.world.resetMap(); rooms.clear(); });
    generate_button->color = sf::Color(200, 255, 200);
    reset_map_button->color = sf::Color(255, 200, 200);
    // Add items
    addItem(generate_button);
    addItem(reset_map_button);
  }

  sf::Vector2i getRandomPointInCircle(int32_t radius) {
    const float t = 2.0f * M_PI * RNGf::getUnder(1.0f);
    const float u = RNGf::getUnder(1.0f) + RNGf::getUnder(1.0f);
    const float r = u > 1.0f ? 2.0f - u : u;
    const auto roundm = [](float x, int32_t m) {
      return (int32_t) (m * std::round(x / m));
    };
    const auto x = roundm(radius * r * std::cos(t), simulation.world.map.cell_size);
    const auto y = roundm(radius * r * std::sin(t), simulation.world.map.cell_size);
    return sf::Vector2i{(int32_t) simulation.world.map.width / 2 + x, (int32_t) simulation.world.map.height / 2 + y};
  }

  bool anyRoomOverlap(std::vector<Room> rooms) {
    for (const auto& room1 : rooms) {
      for (const auto& room2 : rooms) {
        if (room1 != room2 && room1.isOverlapping(room2)) {
          return true;
        }
      }
    }
    return false;
  }

  sf::Vector2f steeringBehaviourSeparation(Room agent, std::vector<Room> rooms) {
    sf::Vector2f total_force = sf::Vector2f{0.0f, 0.0f};
    int32_t neighbours_count = 0;
    for (const auto& room : rooms) {
      if (room != agent) {
        if (agent.isOverlapping(room)) {
          const auto push_force = agent.distance(room);
          total_force.x += push_force.x;
          total_force.y += push_force.y;
          neighbours_count++;
        }
      }
      if (neighbours_count > 5) {
        break;
      }
    }

    if (neighbours_count == 0) {
      return sf::Vector2f{0.0f, 0.0f};
    }

    total_force.x /= neighbours_count;
    total_force.y /= neighbours_count;
    return total_force;
  }



  std::vector<Room> getRooms() {
    
    for (int32_t i(0); i < 60; ++i) {
      const float radius = RNGf::getUnder(1.0f) * 150.0f;
      const auto point = getRandomPointInCircle(radius);
      const int room_width = std::round(RNGf::getUnder(1.0f) * 20.0f + 5.0f) * 2 + 1;
      const int room_height = std::round(RNGf::getUnder(1.0f) * 20.0f + 5.0f) * 2 + 1;
      rooms.push_back({point, room_width, room_height});
    }

    return rooms;
  }

  void makeRoomsNonOverlapping(std::vector<Room>& rooms) {
    const int boundary = 10;
    int count = 0;
    while (anyRoomOverlap(rooms)) {
      for (auto& room : rooms) {
        auto force = steeringBehaviourSeparation(room, rooms);
        room.pos.x = std::max(room.width / 2 + 1 + boundary, std::min(to<int32_t>(simulation.world.map.width - 1 - room.width / 2 - 1 - boundary), room.pos.x + (int32_t) force.x));
        room.pos.y = std::max(room.height / 2 + 1 + boundary, std::min(to<int32_t>(simulation.world.map.height - 1 - room.height / 2 - 1 - boundary), room.pos.y + (int32_t) force.y));
        room.close_to_x_boundary = room.pos.x == room.width / 2 + 1 + boundary || room.pos.x == to<int32_t>(simulation.world.map.width - 1 - room.width / 2 - 1 - boundary);
        room.close_to_y_boundary = room.pos.y == room.height / 2 + 1 + boundary || room.pos.y == to<int32_t>(simulation.world.map.height - 1 - room.height / 2 - 1 - boundary);
      }
      count++;
      // TODO: try to fix this (probably check whether already at boundary and if yes change direction)
      if (count > 100000) {
        std::cout << "Could not generate map count = " << count << std::endl; 
        break;
      }
    }
  }

  void smoothMap() {
    for (int32_t x(1); x < simulation.world.map.width - 1; ++x) {
      for (int32_t y(1); y < simulation.world.map.height - 1; ++y) {
        int32_t wall_count = 0;
        for (int32_t i(-1); i <= 1; ++i) {
          for (int32_t j(-1); j <= 1; ++j) {
            if (simulation.world.map.get(sf::Vector2i{x + i, y + j}).wall == 1) {
              wall_count++;
            }
          }
        }
        if (wall_count > 4) {
          simulation.world.addWall(sf::Vector2i{x, y});
        } else if (wall_count < 4) {
          simulation.world.removeWall(sf::Vector2i{x, y});
        }
      }
    }
  }

  void createMap() {
    rooms.clear();
    simulation.world.resetMap();
    simulation.world.fillWithWalls();
    std::vector<Room> rooms = getRooms();
    makeRoomsNonOverlapping(rooms);
    // find mean height and width and perform thresholding to only keep rooms bigger than mean
    int32_t mean_width = 0;
    int32_t mean_height = 0;
    for (const auto& room : rooms) {
      mean_width += room.width;
      mean_height += room.height;
    }
    mean_width /= rooms.size();
    mean_height /= rooms.size();
    // store rooms bigger than mean in main_room vector
    std::vector<Room> main_rooms;
    for (const auto& room : rooms) {
      if (room.width > mean_width * 1.1 && room.height > mean_height * 1.1) {
        main_rooms.push_back(room);
      }
    }

    for (const auto& room : rooms) {
      const auto& point = room.pos;
      const int room_width = room.width;
      const int room_height = room.height;
      const int32_t min_x = std::max(1, point.x - room_width / 2);
      const int32_t max_x = std::min(to<int32_t>(simulation.world.map.width - 1),
                                    point.x + room_width / 2 + 1);
      const int32_t min_y = std::max(1, point.y - room_height / 2);
      const int32_t max_y = std::min(to<int32_t>(simulation.world.map.height - 1),
                                    point.y + room_height / 2 + 1);
      for (int32_t x(min_x); x < max_x; ++x) {
        for (int32_t y(min_y); y < max_y; ++y) {
          simulation.world.removeWall(sf::Vector2i{x, y});
          // place food if at the border to see the rectangle
          // if (x == min_x || x == max_x - 1 || y == min_y || y == max_y - 1) {
          //   simulation.world.addFoodAt(sf::Vector2i{x, y}, 2);
          // }
        }
      }
    }

    for (int32_t x(1); x < simulation.world.map.width - 1; x+= 1) {
      for (int32_t y(1); y < simulation.world.map.height - 1; y+= 1) {
        if (RNGf::getUnder(1.0f) < 0.3f) {
          applyBrush(sf::Vector2f{RNGf::getUnder(Conf::WORLD_WIDTH),
                          RNGf::getUnder(Conf::WORLD_HEIGHT)},
              [this](int32_t x, int32_t y) {
                  simulation.world.addWall(sf::Vector2i{x, y});
              });
        }
      }
    }

    for (int i = 0; i < 5; ++i) {
      smoothMap();
    }

    simulation.distance_field_builder.requestUpdate();
  }

  void createMap2() {
    // randomly initialize the world so that exactly 0.65 of the cells are walls
    simulation.world.resetMap();
    // mark edges as walls 
    for (int32_t x(0); x < simulation.world.map.width; ++x) {
      simulation.world.addWall(sf::Vector2i{x, 0});
      simulation.world.addWall(sf::Vector2i{x, simulation.world.map.height - 1});
    }
    for (int32_t y(0); y < simulation.world.map.height; ++y) {
      simulation.world.addWall(sf::Vector2i{0, y});
      simulation.world.addWall(sf::Vector2i{simulation.world.map.width - 1, y});
    }

    for (int32_t x(1); x < simulation.world.map.width - 1; x += 3) {
      for (int32_t y(1); y < simulation.world.map.height - 1; y += 3) {
        if (RNGf::getUnder(1.0f) < 0.30f) {
          applyBrush(sf::Vector2f{RNGf::getUnder(Conf::WORLD_WIDTH),
                          RNGf::getUnder(Conf::WORLD_HEIGHT)},
              [this](int32_t x, int32_t y) {
                  simulation.world.addWall(sf::Vector2i{x, y});
              });
        }
      }
    }

    for (int i = 0; i < 3; ++i) {
      smoothMap();
    }
    simulation.distance_field_builder.requestUpdate();
  }

  template <typename TCallback>
  void applyBrush(sf::Vector2f position, TCallback&& callback) {
    const auto x = to<int32_t>(position.x) / simulation.world.map.cell_size;
    const auto y = to<int32_t>(position.y) / simulation.world.map.cell_size;

    // const int32_t min_x = std::max(1, x - brush_size);
    // const int32_t max_x = std::min(to<int32_t>(simulation.world.map.width - 1),
    //                                x + brush_size + 1);
    // const int32_t min_y = std::max(1, y - brush_size);
    // const int32_t max_y = std::min(to<int32_t>(simulation.world.map.height - 1),
    //                                y + brush_size + 1);

    // for (int32_t px(min_x); px < max_x; ++px) {
    //   for (int32_t py(min_y); py < max_y; ++py) {
    //     callback(px, py);
    //   }
    // }
    callback(x, y);
  }
};

}  // namespace edtr
