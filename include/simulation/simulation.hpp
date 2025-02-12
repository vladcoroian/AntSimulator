#pragma once
#include "world/world.hpp"
#include "colony/colony.hpp"
#include "config.hpp"
#include "common/viewport_handler.hpp"
#include "common/event_manager.hpp"
#include "event_state.hpp"
#include "render/renderer.hpp"
#include "simulation/world/map_loader.hpp"
#include "simulation/ant/fight_system.hpp"
#include "world/async_distance_field_builder.hpp"
#include "../editor/map_generator/room.hpp"

struct Simulation {
  civ::Vector<Colony> colonies;
  World world;
  // Render
  Renderer renderer;
  EventSate ev_state;
  FightSystem fight_system;
  sf::Clock clock;
  AsyncDistanceFieldBuilder distance_field_builder;
  std::vector<edtr::Room> main_rooms;
  std::ofstream log_file;
  int iteration;
  float max_liberty_coef = 0.01f;

  explicit Simulation(sf::Window& window, float max_liberty_coef = 0.01)
      : world(Conf::WORLD_WIDTH, Conf::WORLD_HEIGHT),
        renderer(),
        distance_field_builder(world.map),
        max_liberty_coef(max_liberty_coef) {
    distance_field_builder.requestUpdate();
    iteration = 0;
    log_file.open("out/log.csv");
    log_file.clear();
    log_file << "time,food,colony_size,positions" << std::endl;
  }

  void loadMap(const std::string& map_filename) {
    MapLoader::loadMap(world, map_filename);
    distance_field_builder.requestUpdate();
  }

  civ::Ref<Colony> createColony(float colony_x, float colony_y, float max_autonomy,
                                int32_t max_ants_count = Conf::ANTS_COUNT) {
    // Create the colony object
    const civ::ID colony_id =
        colonies.emplace_back(colony_x, colony_y, max_ants_count, max_autonomy, max_liberty_coef);
    auto colony_ref = colonies.getRef(colony_id);
    Colony& colony = *colony_ref;
    colony.initialize(to<uint8_t>(colony_id));
    // Create colony markers
    createColonyMarkers(colony);
    // Register it for the renderer
    renderer.addColony(colony_ref);
    world.renderer.colonies_color.emplace_back();
    return colony_ref;
  }

  void update(float dt) {
    if (!ev_state.pause) {
      // Mark ants with no more time left as dead
      removeDeadAnts();
      // Update world cells (markers, density, walls)
      world.update(dt);
      // First perform position update and grid registration
      for (Colony& colony : colonies) {
        if (colony.color_changed) {
          world.renderer.colonies_color[colony.id] = colony.ants_color;
        }
        if (colony.position_changed) {
          updateColonyPosition(colony);
        }
      }
      for (Colony& colony : colonies) {
        colony.genericAntsUpdate(dt, world);
      }
      // Then update objectives and world sampling
      for (Colony& colony : colonies) {
        colony.update(dt, world);
      }
      // Search for fights
      fight_system.checkForFights(colonies, world);
      // Update stats
      renderer.updateColoniesStats(dt);
      // Update logs
      iteration++;
      logStep();
    }
  }

  void logStep() {
    if (log_file.is_open()) {
      for (Colony& colony : colonies) {
        log_file << iteration << "," << this->world.map.food_count << "," << colony.ants.size()
                 << ",";
        if (iteration % 10 == 1) {
          log_file << "\"";
          for (Ant& ant : colony.ants) {
            // print position of each ant
            log_file << "(" << ant.position.x << "-" << ant.position.y << ");";
          }
          log_file << "\"";
        }
        log_file << std::endl;
      }
    }
  }

  void removeDeadAnts() {
    // Mark old ants as dead
    for (Colony& colony : colonies) {
      const uint32_t killed = colony.killWeakAnts(world);
      if (killed) {
        const uint32_t initial_size = to<int32_t>(colony.ants.size());
        renderer.colonies[colony.id].cleanVAs(initial_size - killed, initial_size);
      }
    }
    // Remove them
    for (Colony& colony : colonies) {
      colony.removeDeadAnts();
    }
  }

  void updateColonyPosition(Colony& colony) {
    colony.position_changed = false;
    world.clearMarkers(colony.id);
    // Create colony markers
    createColonyMarkers(colony);
  }

  void createColonyMarkers(Colony& colony) {
    for (uint32_t i(0); i < 64; ++i) {
      float angle = float(i) / 64.0f * (2.0f * PI);
      world.addMarker(
          colony.base.position + 0.9f * colony.base.radius * sf::Vector2f(cos(angle), sin(angle)),
          Mode::ToHome, 10.0f, true);
    }
  }

  void render(sf::RenderTarget& target) { renderer.render(world, target); }

  void removeColony(uint8_t colony_id) {
    for (Colony& c : colonies) {
      c.stopFightsWith(colony_id);
    }
    colonies.erase(colony_id);
    renderer.colonies.erase(colony_id);
    world.renderer.colonies_color.erase(colony_id);
    world.clearMarkers(colony_id);
  }
};
