#pragma once
#include <iostream>
#include <cmath>
#include <SFML/Graphics.hpp>
#include "editor/GUI/container.hpp"
#include "editor/GUI/button.hpp"
#include "editor/map_generator/room.hpp"
#include "delaunay.hpp"
#include <vector>
#include <algorithm>
#include <queue>

namespace edtr {

struct MapGenerator : public GUI::NamedContainer {
  Simulation& simulation;
  ControlState& control_state;
  int32_t brush_size = 1;
  SPtr<ToolOption> generate_button;
  SPtr<ToolOption> reset_map_button;
  std::vector<Room> rooms_;

  explicit MapGenerator(Simulation& sim, ControlState& control_state_)
      : GUI::NamedContainer("Map Generator", Container::Orientation::Horizontal),
        simulation(sim),
        control_state(control_state_) {
    root->setHeight(30.0f, GUI::Size::Fixed);
    generate_button = create<ToolOption>("Generate", [this]() { this->createMap(); });
    reset_map_button = create<ToolOption>("Reset Map", [this]() {
      this->simulation.world.resetMap();
      rooms_.clear();
    });
    generate_button->color = sf::Color(200, 255, 200);
    reset_map_button->color = sf::Color(255, 200, 200);
    // Add items
    addItem(generate_button);
    addItem(reset_map_button);
  }

  sf::Vector2i getRandomPointInCircle(int32_t radius) {
    const double t = 2.0f * M_PI * RNGf::getUnder(1.0f);
    const double u = RNGf::getUnder(1.0f) + RNGf::getUnder(1.0f);
    const double r = u > 1.0f ? 2.0f - u : u;
    const auto roundm = [](float x, int32_t m) { return (int32_t)(m * std::round(x / m)); };
    const auto x = roundm(radius * r * std::cos(t), simulation.world.map.cell_size);
    const auto y = roundm(radius * r * std::sin(t), simulation.world.map.cell_size);
    return sf::Vector2i{(int32_t)simulation.world.map.width / 2 + x,
                        (int32_t)simulation.world.map.height / 2 + y};
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
      rooms_.push_back({i, point, room_width, room_height});
    }

    return rooms_;
  }

  void makeRoomsNonOverlapping(std::vector<Room>& rooms) {
    const int boundary = 10;
    int count = 0;
    while (anyRoomOverlap(rooms)) {
      for (auto& room : rooms) {
        auto force = steeringBehaviourSeparation(room, rooms);
        room.pos.x = std::max(
            room.width / 2 + 1 + boundary,
            std::min(to<int32_t>(simulation.world.map.width - 1 - room.width / 2 - 1 - boundary),
                     room.pos.x + (int32_t)force.x));
        room.pos.y = std::max(
            room.height / 2 + 1 + boundary,
            std::min(to<int32_t>(simulation.world.map.height - 1 - room.height / 2 - 1 - boundary),
                     room.pos.y + (int32_t)force.y));
        room.close_to_x_boundary = room.pos.x == room.width / 2 + 1 + boundary ||
                                   room.pos.x == to<int32_t>(simulation.world.map.width - 1 -
                                                             room.width / 2 - 1 - boundary);
        room.close_to_y_boundary = room.pos.y == room.height / 2 + 1 + boundary ||
                                   room.pos.y == to<int32_t>(simulation.world.map.height - 1 -
                                                             room.height / 2 - 1 - boundary);
      }
      count++;
      // TODO: try to fix this (probably check whether already at boundary and
      // if yes change direction)
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

  std::vector<std::tuple<int, int, double>> delaunayTriangulation(std::vector<Room>& rooms) {
    std::vector<std::tuple<int, int, double>> edges;
    std::vector<double> coords;
    for (const auto& room : rooms) {
      coords.push_back(room.pos.x);
      coords.push_back(room.pos.y);
    }
    delaunator::Delaunator d(coords);
    for (std::size_t i = 0; i < d.triangles.size(); i += 3) {
      int first_room = rooms[d.triangles[i]].id;
      int second_room = rooms[d.triangles[i + 1]].id;
      int third_room = rooms[d.triangles[i + 2]].id;
      edges.emplace_back(first_room, second_room, rooms[first_room].distanceTo(rooms[second_room]));
      edges.emplace_back(second_room, first_room, rooms[first_room].distanceTo(rooms[second_room]));
      edges.emplace_back(second_room, third_room, rooms[second_room].distanceTo(rooms[third_room]));
      edges.emplace_back(third_room, second_room, rooms[second_room].distanceTo(rooms[third_room]));
      edges.emplace_back(third_room, first_room, rooms[third_room].distanceTo(rooms[first_room]));
      edges.emplace_back(first_room, third_room, rooms[third_room].distanceTo(rooms[first_room]));
    }
    return edges;
  }

  std::vector<std::tuple<int, int, double>> min_spanning_tree(
      const std::vector<std::tuple<int, int, double>>& graph, uint32_t size) {
    std::vector<std::tuple<int, int, double>> mst;
    std::vector<bool> visited(size, false);

    std::priority_queue<std::tuple<int, int, double>, std::vector<std::tuple<int, int, double>>,
                        std::greater<>>
        pq;
    const int first_room = std::get<0>(graph[0]);
    pq.emplace(first_room, first_room, 0.0);
    while (!pq.empty()) {
      auto edge = pq.top();
      pq.pop();
      if (visited[std::get<1>(edge)]) {
        continue;
      }
      visited[std::get<1>(edge)] = true;
      mst.push_back(edge);
      for (const auto& e : graph) {
        if (std::get<0>(e) == std::get<1>(edge) && !visited[std::get<1>(e)]) {
          pq.push(e);
        }
      }
    }
    return mst;
  }

  void placeRoom(const Room& room) {
    const auto& point = room.pos;
    const int room_width = room.width;
    const int room_height = room.height;
    const int32_t min_x = std::max(1, point.x - room_width / 2);
    const int32_t max_x =
        std::min(to<int32_t>(simulation.world.map.width - 1), point.x + room_width / 2 + 1);
    const int32_t min_y = std::max(1, point.y - room_height / 2);
    const int32_t max_y =
        std::min(to<int32_t>(simulation.world.map.height - 1), point.y + room_height / 2 + 1);
    for (int32_t x(min_x); x < max_x; ++x) {
      for (int32_t y(min_y); y < max_y; ++y) {
        simulation.world.removeWall(sf::Vector2i{x, y});
      }
    }
  }

  void placeHallway(const Room& source, const Room& target) {
    const auto& source_point = source.pos;
    const auto& target_point = target.pos;
    // midpoint between source_point and target_point
    const auto midpoint = (source_point + target_point) / 2;

    const bool x_in_source = midpoint.x >= source_point.x - source.width / 2 &&
                             midpoint.x <= source_point.x + source.width / 2;
    const bool y_in_source = midpoint.y >= source_point.y - source.height / 2 &&
                             midpoint.y <= source_point.y + source.height / 2;
    const bool x_in_target = midpoint.x >= target_point.x - target.width / 2 &&
                             midpoint.x <= target_point.x + target.width / 2;
    const bool y_in_target = midpoint.y >= target_point.y - target.height / 2 &&
                             midpoint.y <= target_point.y + target.height / 2;
    if (x_in_source && x_in_target) {
      // create vertical corridor
      const int32_t min_y = std::min(source_point.y, target_point.y);
      const int32_t max_y = std::max(source_point.y, target_point.y);
      for (int32_t y(min_y); y < max_y; ++y) {
        applyIntegerBrush(sf::Vector2i{midpoint.x, y}, [this](int32_t x, int32_t y) {
          simulation.world.removeWall(sf::Vector2i{x, y});
        });
      }
      return;
    }

    if (y_in_source && y_in_target) {
      // create horizontal corridor
      const int32_t min_x = std::min(source_point.x, target_point.x);
      const int32_t max_x = std::max(source_point.x, target_point.x);
      for (int32_t x(min_x); x < max_x; ++x) {
        applyIntegerBrush(sf::Vector2i{x, midpoint.y}, [this](int32_t x, int32_t y) {
          simulation.world.removeWall(sf::Vector2i{x, y});
        });
      }
      return;
    }

    // create L-shaped corridor from source_point to target_point
    if (source_point.x < target_point.x) {
      for (int32_t x = source_point.x; x <= target_point.x; ++x) {
        applyIntegerBrush(sf::Vector2i{x, source_point.y}, [this](int32_t x, int32_t y) {
          simulation.world.removeWall(sf::Vector2i{x, y});
        });
      }
      // I am at (target_point.x, source_point.y) and want to move to (target_point.x,
      // target_point.y)
      const int32_t min_y = std::min(source_point.y, target_point.y);
      const int32_t max_y = std::max(source_point.y, target_point.y);
      for (int32_t y(min_y); y < max_y; ++y) {
        applyIntegerBrush(sf::Vector2i{target_point.x, y}, [this](int32_t x, int32_t y) {
          simulation.world.removeWall(sf::Vector2i{x, y});
        });
      }
    } else {
      for (int32_t x = target_point.x; x <= source_point.x; ++x) {
        applyIntegerBrush(sf::Vector2i{x, target_point.y}, [this](int32_t x, int32_t y) {
          simulation.world.removeWall(sf::Vector2i{x, y});
        });
      }
      // I am at (source_point.x, target_point.y) and want to move to (source_point.x,
      // source_point.y)
      const int32_t min_y = std::min(source_point.y, target_point.y);
      const int32_t max_y = std::max(source_point.y, target_point.y);
      for (int32_t y = min_y; y < max_y; ++y) {
        applyIntegerBrush(sf::Vector2i{source_point.x, y}, [this](int32_t x, int32_t y) {
          simulation.world.removeWall(sf::Vector2i{x, y});
        });
      }
    }
  }

  void createMap() {
    rooms_.clear();
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
      if (room.width > mean_width && room.height > mean_height) {
        main_rooms.push_back(room);
      }
    }

    std::cout << "Number of rooms: " << main_rooms.size() << std::endl;
    // Delauany triangulation for main rooms and MST
    std::vector<std::tuple<int, int, double>> graph;
    std::cout << graph.size() << std::endl;
    graph = delaunayTriangulation(main_rooms);
    std::cout << graph.size() << std::endl;
    std::vector<std::tuple<int, int, double>> mst = min_spanning_tree(graph, rooms.size());

    for (const auto& room : main_rooms) {
      placeRoom(room);
    }

    // create corridors
    for (const auto& edge : mst) {
      const auto& source = rooms[std::get<0>(edge)];
      const auto& target = rooms[std::get<1>(edge)];
      if (source.isOverlapping(target)) {
        continue;
      }
      placeHallway(source, target);
    }

    // for (int32_t x(1); x < simulation.world.map.width - 1; x += 1) {
    //   for (int32_t y(1); y < simulation.world.map.height - 1; y += 1) {
    //     if (RNGf::getUnder(1.0f) < 0.3f) {
    //       applyBrush(
    //           sf::Vector2f{RNGf::getUnder(Conf::WORLD_WIDTH),
    //           RNGf::getUnder(Conf::WORLD_HEIGHT)}, [this](int32_t x, int32_t y) {
    //             simulation.world.addWall(sf::Vector2i{x, y});
    //           });
    //     }
    //   }
    // }

    // for (int i = 0; i < 5; ++i) {
    //   smoothMap();
    // }

    simulation.distance_field_builder.requestUpdate();
  }

  template <typename TCallback>
  void applyBrush(sf::Vector2f position, TCallback&& callback) {
    const auto x = to<int32_t>(position.x) / simulation.world.map.cell_size;
    const auto y = to<int32_t>(position.y) / simulation.world.map.cell_size;
    const int32_t min_x = std::max(1, x - brush_size);
    const int32_t max_x = std::min(to<int32_t>(simulation.world.map.width - 1), x + brush_size + 1);
    const int32_t min_y = std::max(1, y - brush_size);
    const int32_t max_y =
        std::min(to<int32_t>(simulation.world.map.height - 1), y + brush_size + 1);

    for (int32_t px(min_x); px < max_x; ++px) {
      for (int32_t py(min_y); py < max_y; ++py) {
        callback(px, py);
      }
    }
    // callback(x, y);
  }

  template <typename TCallback>
  void applyIntegerBrush(sf::Vector2i position, TCallback&& callback) {
    const auto x = to<int32_t>(position.x);
    const auto y = to<int32_t>(position.y);
    const int32_t min_x = std::max(1, x - brush_size);
    const int32_t max_x = std::min(to<int32_t>(simulation.world.map.width - 1), x + brush_size + 1);
    const int32_t min_y = std::max(1, y - brush_size);
    const int32_t max_y =
        std::min(to<int32_t>(simulation.world.map.height - 1), y + brush_size + 1);

    for (int32_t px(min_x); px < max_x; ++px) {
      for (int32_t py(min_y); py < max_y; ++py) {
        callback(px, py);
      }
    }
    // callback(x, y);
  }
};

}  // namespace edtr
