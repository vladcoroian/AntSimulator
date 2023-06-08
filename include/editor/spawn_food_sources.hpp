#pragma once
#include "GUI/button.hpp"
#include "editor/color_picker/color_picker.hpp"
#include "editor/GUI/named_container.hpp"
#include "editor/GUI/toggle.hpp"
#include "common/dynamic_blur.hpp"

namespace edtr {

struct SpawnFoodSource : public GUI::NamedContainer {
  Simulation& simulation;
  SPtr<SliderLabel> spawn_food_sources_slider;
  SPtr<SliderLabel> food_quantity_slider;
  SPtr<SliderLabel> window_size_slider;

  explicit SpawnFoodSource(Simulation& sim)
      : GUI::NamedContainer("Spawn food sources", GUI::Container::Orientation::Vertical),
        simulation(sim) {
    auto food_source_number_setter = create<GUI::NamedContainer>(
        "Number of food sources", GUI::Container::Orientation::Vertical);
    spawn_food_sources_slider = create<SliderLabel>(10.0f);
    auto food_quantity_setter =
        create<GUI::NamedContainer>("Food quantity", GUI::Container::Orientation::Vertical);
    food_quantity_slider = create<SliderLabel>(100.0f);
    auto window_size_setter =
        create<GUI::NamedContainer>("Food window size", GUI::Container::Orientation::Vertical);
    window_size_slider = create<SliderLabel>(10.0f);
    auto spawn_food_sources_button = create<ToolOption>("Spawn", [this]() { spawnFoodSources(); });
    auto remove_food_sources_button =
        create<ToolOption>("Remove all food", [this]() { simulation.world.removeAllFood(); });

    food_source_number_setter->addItem(spawn_food_sources_slider);

    auto buttons = create<GUI::Container>(GUI::Container::Orientation::Horizontal);
    buttons->size.y = 50.0f;
    buttons->size_type.y = GUI::Size::Fixed;
    buttons->addItem(spawn_food_sources_button);
    buttons->addItem(remove_food_sources_button);
    addItem(buttons);
    spawn_food_sources_button->color = sf::Color(200, 255, 200);
    remove_food_sources_button->color = sf::Color(255, 200, 200);
    addItem(food_source_number_setter);
    food_quantity_setter->addItem(food_quantity_slider);
    addItem(food_quantity_setter);
    window_size_setter->addItem(window_size_slider);
    addItem(window_size_setter);
    header->addItem(create<GUI::EmptyItem>());

    auto tools_toggle = create<GUI::Toggle>();
    tools_toggle->color_on = {240, 180, 0};
    tools_toggle->setState(false);
    watch(tools_toggle, [this, tools_toggle] {
      if (tools_toggle->state) {
        showRoot();
      } else {
        hideRoot();
      }
    });
    hideRoot();
    header->addItem(tools_toggle);
  }

  void spawnFood(int32_t food_quantity, int32_t window_size, int32_t food_sources) {
    bool placedFood = false;
    int brush_size = window_size / 2;
    while (!placedFood) {
      const float x = RNGf::getUnder(simulation.world.map.width);
      const float y = RNGf::getUnder(simulation.world.map.height);
      int emptyCells = 0;
      for (int dx = -brush_size; dx <= brush_size; ++dx) {
        for (int dy = -brush_size; dy <= brush_size; ++dy) {
          auto pos = sf::Vector2i(x + dx, y + dy);
          if (!simulation.world.map.checkCoords(pos)) {
            continue;
          }
          const auto& cell = simulation.world.map.get(pos);
          if (!cell.wall && cell.food == 0) {
            ++emptyCells;
          }
        }
      }
      if (emptyCells >= brush_size * brush_size * 3 / 4) {
        placedFood = true;
        for (int dx = -brush_size; dx <= brush_size; ++dx) {
          for (int dy = -brush_size; dy <= brush_size; ++dy) {
            auto pos = sf::Vector2i(x + dx, y + dy);
            if (!simulation.world.map.checkCoords(pos)) {
              continue;
            }
            auto& cell = simulation.world.map.get(pos);
            if (!cell.wall && cell.food == 0) {
              cell.food = food_quantity;
            }
          }
        }
      } else {
        std::cout << "Resampling..." << std::endl;
      }
    }
  }

  void spawnFoodSources() {
    auto food_quantity = (int32_t)food_quantity_slider->getValue();
    auto window_size = (int32_t)window_size_slider->getValue();
    auto food_sources = (int32_t)spawn_food_sources_slider->getValue();
    for (int i = 0; i < food_sources; ++i) {
      spawnFood(food_quantity, window_size, food_sources);
    }
  }
};

}  // namespace edtr
