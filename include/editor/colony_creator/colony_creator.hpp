#pragma once
#include "editor/GUI/container.hpp"
#include "editor/GUI/button.hpp"
#include "colony_tool.hpp"
#include "simulation/simulation.hpp"

namespace edtr {

struct ColonyCreator : public GUI::NamedContainer {
  Simulation& simulation;
  ControlState& control_state;
  uint32_t colonies_count = 0;
  int32_t last_selected = -1;
  float max_autonomy = 300.0f;
  int32_t colony_size = 1000;

  explicit ColonyCreator(Simulation& sim, ControlState& control_state_,
                         float max_autonomy_ = 300.0f, int32_t colony_size_ = 1000)
      : GUI::NamedContainer("Colonies", Container::Orientation::Vertical),
        simulation(sim),
        control_state(control_state_),
        max_autonomy(max_autonomy_),
        colony_size(colony_size_) {
    root->size_type.y = GUI::Size::FitContent;
    auto add_button = create<GUI::Button>("Add", [this]() { this->createColony(); });
    auto add_random_button = create<GUI::Button>("Spawn colony", [this]() {
      // select random room and place colony in center
      if (!simulation.main_rooms.empty()) {
        const int index = RNGf::getUnder(simulation.main_rooms.size());
        const auto& room = simulation.main_rooms[index];
        this->createColony(room.pos.x * simulation.world.map.cell_size,
                           room.pos.y * simulation.world.map.cell_size);
      } else {
        std::cout << "No room to place colony in" << std::endl;
      }
    });
    add_button->setWidth(50.0f, GUI::Size::Fixed);
    add_button->setHeight(35.0f, GUI::Size::Fixed);

    add_random_button->setWidth(160.0f, GUI::Size::Fixed);
    add_random_button->setHeight(35.0f, GUI::Size::Fixed);
    header->addItem(create<GUI::EmptyItem>());
    header->addItem(add_button);
    header->addItem(create<GUI::EmptyItem>());
    header->addItem(add_random_button);
  }

  void createColony(float x = 50.0f, float y = 50.0f) {
    if (this->colonies_count < Conf::MAX_COLONIES_COUNT) {
      auto new_colony = simulation.createColony(x, y, max_autonomy, colony_size);
      auto colony_tool = create<ColonyTool>(new_colony, control_state);
      colony_tool->on_select = [this](int8_t id) { select(id); };
      // Add the new item to this
      this->addItem(colony_tool);
      ++this->colonies_count;

      // Set the correct callback for the remove button
      colony_tool->top_zone->getByName<GUI::Button>("remove")->click_callback = [this,
                                                                                 colony_tool]() {
        simulation.removeColony(colony_tool->colony->id);
        this->removeItem(colony_tool);
        --this->colonies_count;
      };
    }
  }

  void select(int32_t id) {
    int32_t selected = -1;
    if (id != last_selected) {
      selected = id;
    }
    last_selected = selected;
    this->simulation.world.renderer.selected_colony = selected;

    for (const auto& item : root->sub_items) {
      auto tool = std::dynamic_pointer_cast<ColonyTool>(item);
      if (tool) {
        tool->selected = tool->colony->id == selected;
      }
    }
  }

  void setMaxAutonomy(float new_max_autonomy) { max_autonomy = new_max_autonomy; }

  void setColonySize(int32_t new_colony_size) { colony_size = new_colony_size; }
};

}  // namespace edtr
