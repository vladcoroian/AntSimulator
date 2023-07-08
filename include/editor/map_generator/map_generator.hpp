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
  int32_t brush_size = 15;
  float fill_percentage = 0.45f;
  SPtr<ToolOption> generate_button;
  SPtr<ToolOption> reset_map_button;
  std::vector<Room> rooms_;
  int32_t simulation_width = Conf::WORLD_WIDTH / simulation.world.map.cell_size;
  int32_t simulation_height = Conf::WORLD_HEIGHT / simulation.world.map.cell_size;

  explicit MapGenerator(Simulation& sim, ControlState& control_state_)
      : GUI::NamedContainer("Map Generator", Container::Orientation::Vertical),
        simulation(sim),
        control_state(control_state_) {
    generate_button = create<ToolOption>("Generate", [this]() { this->createMap(); });
    reset_map_button = create<ToolOption>("Reset Map", [this]() {
      this->simulation.world.resetMap();
      rooms_.clear();
    });
    generate_button->color = sf::Color(200, 255, 200);
    reset_map_button->color = sf::Color(255, 200, 200);

    auto map_width_setter =
        create<GUI::NamedContainer>("Map Width", GUI::Container::Orientation::Horizontal);
    auto map_width_picker = create<SliderLabel>(270.0);
    auto map_width_set_button = create<GUI::Button>("Set", [this, map_width_picker]() {
      int new_width = (int)map_width_picker->getValue();
      simulation_width = new_width;
      std::cout << "New width: " << simulation_width << std::endl;
    });
    map_width_set_button->setWidth(30.0f);
    map_width_set_button->setHeight(30.0f);
    map_width_setter->addItem(map_width_picker);
    map_width_setter->addItem(map_width_set_button);

    auto map_height_setter =
        create<GUI::NamedContainer>("Map Height", GUI::Container::Orientation::Horizontal);
    auto map_height_picker = create<SliderLabel>(270.0);
    auto map_height_set_button = create<GUI::Button>("Set", [this, map_height_picker]() {
      int new_height = (int)map_height_picker->getValue();
      simulation_height = new_height;
    });
    map_height_set_button->setWidth(30.0f);
    map_height_set_button->setHeight(30.0f);
    map_height_setter->addItem(map_height_picker);
    map_height_setter->addItem(map_height_set_button);

    auto fill_percentage_setter =
        create<GUI::NamedContainer>("Fill Percentage", GUI::Container::Orientation::Horizontal);
    auto fill_percentage_picker = create<SliderLabel>(100.0);
    auto fill_percentage_set_button = create<GUI::Button>("Set", [this, fill_percentage_picker]() {
      float new_fill_percentage = fill_percentage_picker->getValue();
      fill_percentage = new_fill_percentage / 100;
      std::cout << "New fill percentage: " << fill_percentage << std::endl;
    });
    fill_percentage_set_button->setWidth(30.0f);
    fill_percentage_set_button->setHeight(30.0f);
    fill_percentage_setter->addItem(fill_percentage_picker);
    fill_percentage_setter->addItem(fill_percentage_set_button);

    // Add items

    auto buttons = create<GUI::Container>(GUI::Container::Orientation::Horizontal);
    buttons->size.y = 50.0f;
    buttons->size_type.y = GUI::Size::Fixed;
    buttons->addItem(generate_button);
    buttons->addItem(reset_map_button);
    addItem(buttons);
    addItem(map_width_setter);
    addItem(map_height_setter);
    addItem(fill_percentage_setter);
  }

  void createMap() { createMap2(); }

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

  sf::Vector2i getRandomPointInEllipse(int32_t radius_x, int32_t radius_y, int32_t min_radius = 0) {
    const double t = 2.0f * M_PI * RNGf::getUnder(1.0f);
    const double u = RNGf::getUnder(1.0f) + RNGf::getUnder(1.0f);
    const double r = u > 1.0f ? 2.0f - u : u;
    const auto roundm = [](float x, int32_t m) { return (int32_t)(m * std::round(x / m)); };
    const auto x = roundm(radius_x * r * std::cos(t), simulation.world.map.cell_size);
    const auto y = roundm(radius_y * r * std::sin(t), simulation.world.map.cell_size);
    return sf::Vector2i{(int32_t)simulation.world.map.width / 2 + x + min_radius,
                        (int32_t)simulation.world.map.height / 2 + y + min_radius};
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
    for (int32_t i(0); i < 100; ++i) {
      const int32_t max_width = 30;
      const int32_t boundary = 10;
      const auto point = getRandomPointInEllipse(
          Conf::WORLD_WIDTH / simulation.world.map.cell_size - 3 * max_width,
          Conf::WORLD_HEIGHT / simulation.world.map.cell_size - 3 * max_width, max_width);
      const int room_width = std::round(RNGf::getUnder(1.0f) * max_width + 1.0f) * 2 + 1;
      const int room_height = std::round(RNGf::getUnder(1.0f) * max_width + 1.0f) * 2 + 1;
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

  void placeHallway(const Room& source, const Room& target, std::vector<Room>& hallways) {
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
    const int32_t hallways_size = hallways.size();
    if (x_in_source && x_in_target) {
      // create vertical corridor
      const int32_t min_y = std::min(source_point.y, target_point.y);
      const int32_t max_y = std::max(source_point.y, target_point.y);
      for (int32_t y(min_y); y < max_y; ++y) {
        hallways.push_back({hallways_size, sf::Vector2i{midpoint.x, midpoint.y}, brush_size * 2 + 1,
                            max_y - min_y + 2});
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
        hallways.push_back({hallways_size, sf::Vector2i{midpoint.x, midpoint.y}, max_x - min_x + 2,
                            brush_size * 2 + 1});
        applyIntegerBrush(sf::Vector2i{x, midpoint.y}, [this](int32_t x, int32_t y) {
          simulation.world.removeWall(sf::Vector2i{x, y});
        });
      }
      return;
    }

    // create L-shaped corridor from source_point to target_point
    if (source_point.x < target_point.x) {
      for (int32_t x = source_point.x; x <= target_point.x; ++x) {
        hallways.push_back({hallways_size,
                            sf::Vector2i{(source_point.x + target_point.x) / 2, source_point.y},
                            target_point.x - source_point.x + 2, 2 * brush_size + 1});
        applyIntegerBrush(sf::Vector2i{x, source_point.y}, [this](int32_t x, int32_t y) {
          simulation.world.removeWall(sf::Vector2i{x, y});
        });
      }
      // Move from (target_point.x, source_point.y) to (target_point.x, target_point.y)
      const int32_t min_y = std::min(source_point.y, target_point.y);
      const int32_t max_y = std::max(source_point.y, target_point.y);
      for (int32_t y(min_y); y < max_y; ++y) {
        hallways.push_back({hallways_size, sf::Vector2i{target_point.x, (max_y + min_y) / 2},
                            2 * brush_size + 1, max_y - min_y + 2});
        applyIntegerBrush(sf::Vector2i{target_point.x, y}, [this](int32_t x, int32_t y) {
          simulation.world.removeWall(sf::Vector2i{x, y});
        });
      }
    } else {
      for (int32_t x = target_point.x; x <= source_point.x; ++x) {
        hallways.push_back({hallways_size,
                            sf::Vector2i{(source_point.x + target_point.x) / 2, target_point.y},
                            source_point.x - target_point.x + 2, 2 * brush_size + 1});
        applyIntegerBrush(sf::Vector2i{x, target_point.y}, [this](int32_t x, int32_t y) {
          simulation.world.removeWall(sf::Vector2i{x, y});
        });
      }
      // Move from (source_point.x, target_point.y) to (source_point.x, source_point.y)
      const int32_t min_y = std::min(source_point.y, target_point.y);
      const int32_t max_y = std::max(source_point.y, target_point.y);
      for (int32_t y = min_y; y < max_y; ++y) {
        hallways.push_back({hallways_size, sf::Vector2i{source_point.x, (max_y + min_y) / 2},
                            2 * brush_size + 1, max_y - min_y + 2});
        applyIntegerBrush(sf::Vector2i{source_point.x, y}, [this](int32_t x, int32_t y) {
          simulation.world.removeWall(sf::Vector2i{x, y});
        });
      }
    }
  }

  void createMap2() {
    rooms_.clear();
    simulation.world.resetMap();
    simulation.world.fillWithWalls();
    rooms_.push_back({0,
                      sf::Vector2i{simulation.world.map.width / 2, simulation.world.map.height / 2},
                      simulation_width, simulation_height});
    std::cout << simulation_height << " " << simulation_width << std::endl;

    for (const auto& room : rooms_) {
      placeRoom(room);
    }
    simulation.main_rooms = rooms_;


    for (int32_t x(1); x < simulation.world.map.width - 1; x += 1) {
      for (int32_t y(1); y < simulation.world.map.height - 1; y += 1) {
      }
    }

    for (int32_t x(1); x < simulation.world.map.width - 1; x += 1) {
      for (int32_t y(1); y < simulation.world.map.height - 1; y += 1) {
        if (RNGf::getUnder(1.0f) < fill_percentage) {
          applyBrush(
              sf::Vector2f{RNGf::getUnder(Conf::WORLD_WIDTH), RNGf::getUnder(Conf::WORLD_HEIGHT)},
              [this](int32_t x, int32_t y) {
                simulation.world.addWall(sf::Vector2i{x, y});
              });
        }
      }
    }

    for (int i = 0; i < 10; ++i) {
      smoothMap();
    }

    simulation.distance_field_builder.requestUpdate();

    simulation.distance_field_builder.requestUpdate();
  }

  void createMap1() {
    // generate dungeon using tinykeep algorithm
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
      if (room.width > mean_width * 1.1 && room.height > mean_height * 1.1) {
        main_rooms.push_back(room);
      }
    }

    std::cout << "Number of rooms: " << main_rooms.size() << std::endl;
    // Delauany triangulation for main rooms and MST
    std::vector<std::tuple<int, int, double>> graph;
    graph = delaunayTriangulation(main_rooms);
    std::vector<std::tuple<int, int, double>> mst = min_spanning_tree(graph, rooms.size());

    // create corridors
    std::vector<Room> hallways;
    for (const auto& edge : mst) {
      const auto& source = rooms[std::get<0>(edge)];
      const auto& target = rooms[std::get<1>(edge)];
      if (source.isOverlapping(target)) {
        continue;
      }
      placeHallway(source, target, hallways);
    }

    // check all hallways and if a hallway overlaps with a room, add the room to main_rooms
    for (const auto& hallway : hallways) {
      for (const auto& room : rooms) {
        if (hallway.isOverlapping(room)) {
          main_rooms.push_back(room);
        }
      }
    }

    for (const auto& room : main_rooms) {
      placeRoom(room);
    }
    simulation.main_rooms = main_rooms;

    for (int32_t x(1); x < simulation.world.map.width - 1; x += 1) {
      for (int32_t y(1); y < simulation.world.map.height - 1; y += 1) {
        // if (x, y) is in a hallway then skip it
        bool in_hallway = false;
        for (const auto& hallway : hallways) {
          if (hallway.isOverlapping({0, sf::Vector2i{x, y}, 1, 1})) {
            in_hallway = true;
            break;
          }
        }
        if (in_hallway) {
          continue;
        }
        if (RNGf::getUnder(1.0f) < fill_percentage) {
          applyBrush(
              sf::Vector2f{RNGf::getUnder(Conf::WORLD_WIDTH), RNGf::getUnder(Conf::WORLD_HEIGHT)},
              [this](int32_t x, int32_t y) {
                simulation.world.addWall(sf::Vector2i{x, y});
              });
        }
      }
    }

    for (int i = 0; i < 10; ++i) {
      smoothMap();
    }

    simulation.distance_field_builder.requestUpdate();
  }

  template <typename TCallback>
  void applyBrush(sf::Vector2f position, TCallback&& callback) {
    const auto x = to<int32_t>(position.x) / simulation.world.map.cell_size;
    const auto y = to<int32_t>(position.y) / simulation.world.map.cell_size;
    callback(x, y);
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
  }
};

}  // namespace edtr
