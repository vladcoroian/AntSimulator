#pragma once
#include "editor/tool_selector.hpp"
#include "editor/GUI/toggle.hpp"

namespace edtr {

struct FoodChart : public GUI::Item {
  Simulation& simulation;
  ControlState& control_state;
  Graphic chart;
  uint32_t frame_count = 0;
  SPtr<Item> chart_zone;

  explicit FoodChart(Simulation& sim, ControlState& control_state_)
      : chart(1000, {}, {}), simulation(sim), control_state(control_state_) {
    size_type.x = GUI::Size::Auto;
    size_type.y = GUI::Size::Auto;
    setHeight(100.0f);
  }

  void onSizeChange() override {
    chart.width = size.x;
    chart.height = size.y;
  }

  void onPositionChange() override {
    chart.x = position.x;
    chart.y = position.y;
  }

  void update() override {
    if (control_state.updating) {
      if (frame_count % 10 == 0) {
        chart.addValue(to<float>(simulation.world.map.food_count));
      }
      ++frame_count;
    }
  }

  void render(sf::RenderTarget& target) override {
    chart.color = {0, 150, 0};
    GUI::Item::draw(target, chart);
  }
};

struct FoodStats : public GUI::NamedContainer {
  SPtr<GUI::TextLabel> food_label;
  Simulation& simulation;
  ControlState& control_state;
  SPtr<FoodChart> food_chart;

  FoodStats(Simulation& sim, ControlState& control_state)
      : GUI::NamedContainer("Food left"), simulation(sim), control_state(control_state) {
    header->addItem(create<GUI::EmptyItem>());

    food_label = create<GUI::TextLabel>("", 22);
    header->addItem(food_label);
    root->size_type = {GUI::Size::Auto, GUI::Size::Auto};
    setHeight(160.0f);
    food_chart = create<FoodChart>(simulation, control_state);
    GUI::NamedContainer::addItem(food_chart);
  }

  void update() override { food_label->setText(toStr(simulation.world.map.food_count)); }

  void reset() { food_chart->chart.reset(); }
};

struct SimulationManager : public GUI::NamedContainer {
  enum class State {
    Play,
    Pause,
  };

  State current_state = State::Pause;
  Simulation& simulation;
  ControlState& control_state;
  SPtr<FoodStats> food_chart;

  SPtr<ToolOption> tool_pause;
  SPtr<ToolOption> tool_play;
  SPtr<GUI::NamedToggle> tool_speed;
  SPtr<SliderLabel> seed_picker;

  SimulationManager(Simulation& sim, ControlState& control_state_, SPtr<ColonyCreator> colonies)
      : GUI::NamedContainer("Simulation Manager", Container::Orientation::Vertical),
        simulation(sim),
        control_state(control_state_),
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

    food_chart = create<FoodStats>(simulation, control_state);

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
    addItem(colonies);
    addItem(food_chart);
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

  void startSimulation() {
    select(State::Play);
    food_chart->reset();
  }
};

}  // namespace edtr
