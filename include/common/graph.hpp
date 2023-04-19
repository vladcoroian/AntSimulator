#pragma once
#include <SFML/Graphics.hpp>

struct Graphic : public sf::Drawable {
  mutable sf::VertexArray va;
  std::vector<float> values;
  float max_value;
  float width;
  float height;
  float x, y;

  sf::Color color;

  uint32_t current_index;
  bool full;

  float last_value = 0.0f;

  Graphic(uint32_t values_count, sf::Vector2f size, sf::Vector2f position)
      : va(sf::TriangleStrip, values_count * 2),
        values(values_count, 0.0f),
        max_value(0.0f),
        width(size.x),
        height(size.y),
        x(position.x),
        y(position.y),
        color(sf::Color::White),
        current_index(0),
        full(false) {}

  void addValue(float value) {
    last_value = value;
    const uint64_t size = values.size();
    values[(current_index++) % size] = value;
    max_value = std::max(max_value, value);
    if (current_index == size) {
      full = true;
    }
  }

  void next() {
    ++current_index;
    if (current_index == values.size()) {
      full = true;
      current_index = 0;
    }
  }

  void setLastValue(float value) {
    const uint64_t size = values.size();
    values[current_index % size] = value;
    max_value = std::max(max_value, value);
  }

  void draw(sf::RenderTarget& target, sf::RenderStates states) const override {
    const uint64_t size = values.size();
    const float bw = width / float(size);
    const float hf = height / max_value;
    for (uint64_t i(0); i < size; ++i) {
      const uint64_t index = full ? (current_index + 1 + i) % size : i;
      const float bh = values[index] * hf;
      va[2 * i + 0].position = sf::Vector2f(x + i * bw, y + height);
      va[2 * i + 1].position = sf::Vector2f(x + i * bw, y + height - bh);

      va[2 * i + 0].color = color;
      va[2 * i + 1].color = color;
    }

    target.draw(va, states);
  }
};
