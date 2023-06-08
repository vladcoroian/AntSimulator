#pragma once
#include "editor/tool_selector.hpp"
#include "editor/GUI/toggle.hpp"

namespace edtr {

struct SimulationManager : public GUI::NamedContainer {
  enum class State {
    Play,
    Pause,
  };

  State current_state = State::Pause;

  SPtr<ToolOption> tool_pause;
  SPtr<ToolOption> tool_play;
  SPtr<GUI::NamedToggle> tool_speed;
  SPtr<SliderLabel> seed_picker;

  SimulationManager()
      : GUI::NamedContainer("Simulation Manager", Container::Orientation::Vertical),
        current_state(State::Pause) {
    size_type.x = GUI::Size::FitContent;
    size_type.y = GUI::Size::FitContent;
    root->size_type.x = GUI::Size::FitContent;
    root->size_type.y = GUI::Size::FitContent;

    tool_pause = create<ToolOption>("PAUSE", [this]() {
      current_state = State::Pause;
      select(State::Pause);
    });
    tool_pause->color = {255, 255, 230};
    tool_pause->setWidth(100.0f);

    tool_play = create<ToolOption>("PLAY", [this]() {
      current_state = State::Play;
      select(State::Play);
    });
    tool_play->color = {230, 255, 230};
    tool_play->setWidth(100.0f);

    tool_speed = create<GUI::NamedToggle>("Full Speed");
    tool_speed->setWidth(100.0f);
    watch(tool_speed, [this] { notifyChanged(); });

    // Named container with a slider and a button next to it, when button is pressed the seed is set
    // to the slider value
    auto seed_setter = create<GUI::NamedContainer>("Seed", GUI::Container::Orientation::Horizontal);
    seed_picker = create<SliderLabel>(1000.0);
    seed_setter->setWidth(500.0f);
    seed_picker->setWidth(430.0f);
    auto seed_set_button = create<GUI::Button>("Set", [this]() {
      int new_seed = (int)seed_picker->getValue();
      std::cout << "Setting seed to " << new_seed << std::endl;
      RNGf::initialize(new_seed);
    });
    seed_set_button->setHeight(30.0f);
    seed_setter->addItem(seed_picker);
    seed_setter->addItem(seed_set_button);

    // Add items
    auto buttons = create<GUI::Container>(GUI::Container::Orientation::Horizontal);
    buttons->size.x = 400.0f;
    buttons->size_type.x = GUI::Size::FitContent;
    buttons->size_type.y = GUI::Size::FitContent;
    buttons->addItem(tool_pause);
    buttons->addItem(tool_play);
    buttons->addItem(tool_speed);
    addItem(buttons);
    addItem(seed_setter);
    // Default selection
    select(State::Pause);
  }

  void reset() {
    tool_pause->reset();
    tool_play->reset();
  }

  void select(State option) {
    reset();
    current_state = option;
    switch (option) {
      case State::Pause:
        tool_pause->select();
        break;
      case State::Play:
        tool_play->select();
        break;
    }
    notifyChanged();
  }

  void startSimulation() { select(State::Play); }
};

}  // namespace edtr
