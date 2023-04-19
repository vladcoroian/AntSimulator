#pragma once
#include <iostream>
#include "editor/GUI/container.hpp"
#include "editor/GUI/button.hpp"

namespace edtr {

struct MapGenerator : public GUI::NamedContainer {
  Simulation& simulation;
  ControlState& control_state;
  int32_t brush_size = 3;
  SPtr<ToolOption> generate_button;
  SPtr<ToolOption> reset_map_button;

  explicit MapGenerator(Simulation& sim, ControlState& control_state_)
      : GUI::NamedContainer("Map Generator",
                            Container::Orientation::Horizontal),
        simulation(sim),
        control_state(control_state_) {
    root->setHeight(30.0f, GUI::Size::Fixed);
    generate_button =
        create<ToolOption>("Generate", [this]() { this->createMap(); });
    reset_map_button = create<ToolOption>(
        "Reset Map", [this]() { this->simulation.world.resetMap(); });
    generate_button->color = sf::Color(200, 255, 200);
    reset_map_button->color = sf::Color(255, 200, 200);
    // Add items
    addItem(generate_button);
    addItem(reset_map_button);
  }

  void createMap() {
    simulation.world.resetMap();
    for (int32_t i(0); i < 500; ++i) {
      bool is_food = RNGf::getUnder(1.0f) < 0.05f;
      applyBrush(sf::Vector2f{RNGf::getUnder(Conf::WORLD_WIDTH),
                              RNGf::getUnder(Conf::WORLD_HEIGHT)},
                 [this, is_food](int32_t x, int32_t y) {
                   if (is_food) {
                     simulation.world.addFoodAt(sf::Vector2i{x, y}, 2);
                   } else {
                     simulation.world.addWall(sf::Vector2i{x, y});
                   }
                 });
    }

    simulation.distance_field_builder.requestUpdate();
  }

  template <typename TCallback>
  void applyBrush(sf::Vector2f position, TCallback&& callback) {
    const auto x = to<int32_t>(position.x) / simulation.world.map.cell_size;
    const auto y = to<int32_t>(position.y) / simulation.world.map.cell_size;

    const int32_t min_x = std::max(1, x - brush_size);
    const int32_t max_x = std::min(to<int32_t>(simulation.world.map.width - 1),
                                   x + brush_size + 1);
    const int32_t min_y = std::max(1, y - brush_size);
    const int32_t max_y = std::min(to<int32_t>(simulation.world.map.height - 1),
                                   y + brush_size + 1);

    for (int32_t px(min_x); px < max_x; ++px) {
      for (int32_t py(min_y); py < max_y; ++py) {
        callback(px, py);
      }
    }
  }
};

}  // namespace edtr
