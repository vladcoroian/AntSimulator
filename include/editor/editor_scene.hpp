#pragma once
#include "GUI/scene.hpp"
#include "toolbox.hpp"
#include "editor/color_picker/color_picker.hpp"
#include "color_saver.hpp"
#include "GUI/utils.hpp"
#include "set_color_button.hpp"
#include "GUI/named_container.hpp"
#include "tool_selector.hpp"
#include "editor/colony_creator/colony_creator.hpp"
#include "simulation/world/world.hpp"
#include "render/renderer.hpp"
#include "simulation/config.hpp"
#include "editor/world_view.hpp"
#include "simulation_manager/simulation_manager.hpp"
#include "display_options/display_options.hpp"
#include "map_generator/map_generator.hpp"
#include "spawn_food_sources.hpp"

namespace edtr {

struct EditorScene : public GUI::Scene {
  using Ptr = std::shared_ptr<EditorScene>;

  Simulation& simulation;
  ControlState control_state;

  SPtr<Toolbox> toolbox;
  SPtr<WorldView> renderer;
  SPtr<ToolSelector> tool_selector;
  SPtr<DisplayOption> display_controls;
  SPtr<MapGenerator> map_generator;
  SPtr<ColonyCreator> colonies;
  SPtr<SimulationManager> simulation_manager;
  SPtr<SpawnFoodSource> spawn_food_sources;
  sf::Vector2i colony_location = {-1, -1};

  explicit EditorScene(sf::RenderWindow& window, Simulation& sim)
      : GUI::Scene(window), control_state(sim), simulation(sim) {
    root.padding = 20.0f;

    Conf::loadTextures();
    initialize();
  }

  ~EditorScene() {
    std::cout << "Exiting, clean resources" << std::endl;
    Conf::freeTextures();
  }

  void initialize() {
    const sf::Vector2u window_size = window.getSize();

    renderer = create<WorldView>(toVector2f(window_size), simulation, control_state);

    toolbox = create<Toolbox>(sf::Vector2f{500.0f, to<float>(window_size.y)},
                              sf::Vector2f{root.padding, root.padding});
    // Add display options
    display_controls = create<DisplayOption>(control_state);
    watch(display_controls, [this]() { updateRenderOptions(); });
    toolbox->addItem(display_controls);

    initializeMapTools();

    initializeColonyTools();

    addItem(renderer);
    addItem(toolbox, "Toolbox");
    initializeSimulationManager();
    addFullSimulationButton();
  }

  void initializeMapTools() {
    // Add map edition tools
    auto tools = create<GUI::NamedContainer>("Edit Map", GUI::Container::Orientation::Vertical);
    tools->header->addItem(create<GUI::EmptyItem>());
    tools->hideRoot();
    tools->root->spacing = 0.0f;
    auto tools_toggle = create<GUI::Toggle>();
    tools_toggle->color_on = {240, 180, 0};
    tools->watch(tools_toggle, [this, tools_toggle, tools]() {
      if (tools_toggle->state) {
        tools->showRoot();
        this->tool_selector->setEditMode(tools_toggle->state);
      } else {
        tools->hideRoot();
        this->tool_selector->resetCallback();
      }
    });
    control_state.request_edits_off = [tools_toggle] { tools_toggle->setState(false); };

    tools->header->addItem(tools_toggle);
    toolbox->addItem(tools);

    tool_selector = create<ToolSelector>(control_state, simulation);
    tools->addItem(tool_selector);
    auto brush_size =
        create<GUI::NamedContainer>("Brush Size", GUI::Container::Orientation::Vertical);
    auto slider = create<SliderLabel>(10.0f);
    watch(slider, [this, slider]() { setBrushSize(slider->getValue()); });
    setBrushSize(slider->getValue());
    brush_size->addItem(slider);
    tools->addItem(brush_size);
    map_generator = create<MapGenerator>(simulation, control_state);
    toolbox->addItem(map_generator);

    spawn_food_sources = create<SpawnFoodSource>(simulation);
    toolbox->addItem(spawn_food_sources);
  }

  void initializeColonyTools() {
    auto colony_settings =
        create<GUI::NamedContainer>("Colony settings", GUI::Container::Orientation::Vertical);
    auto max_autonomy_setter =
        create<GUI::NamedContainer>("Max autonomy", GUI::Container::Orientation::Vertical);
    auto colony_size_setter =
        create<GUI::NamedContainer>("Initial colony size", GUI::Container::Orientation::Vertical);
    auto autonomy_slider = create<SliderLabel>(500.0f);
    auto colony_size_slider = create<SliderLabel>(2000.0f);

    max_autonomy_setter->addItem(autonomy_slider);
    colony_settings->addItem(max_autonomy_setter);
    colony_size_setter->addItem(colony_size_slider);
    colony_settings->addItem(colony_size_setter);

    auto tools_toggle = create<GUI::Toggle>();
    tools_toggle->color_on = {240, 180, 0};
    tools_toggle->setState(false);
    watch(tools_toggle, [this, tools_toggle, colony_settings] {
      if (tools_toggle->state) {
        colony_settings->showRoot();
      } else {
        colony_settings->hideRoot();
      }
    });
    colony_settings->header->addItem(create<GUI::EmptyItem>());
    colony_settings->header->addItem(tools_toggle);
    colony_settings->hideRoot();

    colonies = create<ColonyCreator>(simulation, control_state, autonomy_slider->getValue());
    watch(autonomy_slider,
          [this, autonomy_slider]() { colonies->setMaxAutonomy(autonomy_slider->getValue()); });
    watch(colony_size_slider, [this, colony_size_slider]() {
      colonies->setColonySize(colony_size_slider->getValue());
    });
    toolbox->addItem(colony_settings);
  }

  void initializeSimulationManager() {
    simulation_manager = create<SimulationManager>(simulation, control_state, colonies);
    watch(simulation_manager, [this]() {
      this->renderer->current_time_state = simulation_manager->current_state;
      this->control_state.updating =
          simulation_manager->current_state == SimulationManager::State::Play;
      if (simulation_manager->tool_speed->getState()) {
        this->window.setFramerateLimit(400);
      } else {
        this->window.setFramerateLimit(60);
      }
    });
    addItem(simulation_manager, "", GUI::Alignment::Right);
  }

  void addFullSimulationButton() {
    auto full_simulation_container =
        create<GUI::NamedContainer>("Full Simulation", GUI::Container::Orientation::Horizontal);
    auto regenerate_map_toggle = create<GUI::NamedToggle>("Keep same map");
    regenerate_map_toggle->setState(false);

    auto full_simulation = create<ToolOption>("Full Simulation", [this, regenerate_map_toggle]() {
      // generate map, spawn colonies, spawn food
      simulation.world.removeAllFood();
      colonies->removeAllColonies();
      if (!regenerate_map_toggle->getState()) {
        simulation.world.resetMap();
        map_generator->createMap();
        colonies->createRandomColony(colony_location);
      } else {
        colony_location = colonies->createRandomColony();
      }
      spawn_food_sources->spawnFoodSources();
      simulation_manager->startSimulation();
    });
    full_simulation->color = {255, 200, 0};
    full_simulation->setWidth(200.0f);
    full_simulation->setHeight(50.0f);

    full_simulation_container->addItem(full_simulation);
    full_simulation_container->addItem(regenerate_map_toggle);
    simulation_manager->addItem(full_simulation_container);
  }

  void updateRenderOptions() const {
    renderer->simulation.renderer.render_ants = display_controls->draw_ants;
    renderer->simulation.world.renderer.draw_markers = display_controls->draw_markers;
    renderer->simulation.world.renderer.draw_density = display_controls->draw_density;
  }

  void setBrushSize(float size) {
    const auto brush_size = to<int32_t>(size);
    tool_selector->brush_size = brush_size;
    control_state.brush_radius = to<float>(brush_size);
  }

  void onSizeChange() override {
    renderer->size = root.size;
    renderer->simulation.renderer.vp_handler.state.center = root.size * 0.5f * Conf::GUI_SCALE;
    // This is to update mouse_position
    simulation.renderer.vp_handler.wheelZoom(0);
  }
};

}  // namespace edtr
