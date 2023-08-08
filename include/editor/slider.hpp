#include "GUI/item.hpp"

namespace edtr {

struct Slider : public GUI::Item {
  float current_ratio;
  float min_value;
  float max_value;

  Slider(float max_value_, float min_value_ = 0.0f, sf::Vector2f size_ = {},
         sf::Vector2f position_ = {}, float initial_ratio = 0.5f)
      : GUI::Item(size_, position_),
        current_ratio(initial_ratio),
        min_value(min_value_),
        max_value(max_value_) {
    const float slider_height = 20.0f;
    setHeight(slider_height);
    padding = 3.0f;
  }

  float getValue() const { return min_value + (max_value - min_value) * current_ratio; }

  void setCursorPosition(float x) {
    const float width = size.x - 2.0f * padding;
    const float max_x = size.x - padding;
    current_ratio = (clamp(x, padding, max_x) - padding) / width;
    notifyChanged();
  }

  void onClick(sf::Vector2f mouse_position, sf::Mouse::Button) override {
    setCursorPosition(mouse_position.x);
  }

  void onMouseMove(sf::Vector2f mouse_position) override {
    if (clicking) {
      setCursorPosition(mouse_position.x);
    }
  }

  void render(sf::RenderTarget& target) override {
    const sf::Color color(100, 100, 100);
    // Bar
    const float bar_width = size.x - 2.0f * padding;
    const float bar_height = 2.0f;
    sf::RectangleShape bar(sf::Vector2f(bar_width, bar_height));
    bar.setFillColor(color);
    bar.setOrigin(0.0f, bar_height * 0.5f);
    bar.setPosition(position + sf::Vector2f(padding, size.y * 0.5f));
    // Cursor
    const float cursor_radius = 5.0f;
    sf::CircleShape cursor(cursor_radius);
    cursor.setOrigin(cursor_radius, cursor_radius);
    cursor.setOutlineThickness(2.0f);
    cursor.setOutlineColor(color);
    cursor.setPosition(position + sf::Vector2f(padding + bar_width * current_ratio, size.y * 0.5f));
    // Draw things
    draw(target, bar);
    draw(target, cursor);
  }
};

struct SliderLabel : public GUI::Container {
  SPtr<Slider> slider;
  SPtr<GUI::TextLabel> label;
  int precision = 0;

  SliderLabel(float max_value_, float min_value_ = 0.0f, sf::Vector2f size_ = {},
              sf::Vector2f position_ = {}, float initial_ratio = 0.5f, int precision = 0)
      : GUI::Container(GUI::Container::Orientation::Horizontal, size_, position_),
        precision(precision) {
    size_type.y = GUI::Size::FitContent;

    slider = create<Slider>(max_value_, min_value_, sf::Vector2f{}, sf::Vector2f{}, initial_ratio);
    label = create<GUI::TextLabel>("", 22);
    label->setWidth(20.0f);
    label->auto_size_update = false;

    addItem(label);
    addItem(slider);

    watch(slider, [this]() { updateLabel(); });
    updateLabel();
  }

  std::string formatWithPrecision(float value) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(precision) << value;
    return oss.str();
  }

  void updateLabel() {
    label->setText(formatWithPrecision(slider->getValue()));
    notifyChanged();
  }

  float getValue() const { return slider->getValue(); }

  void render(sf::RenderTarget& target) override {
    const float angle_radius = 5.0f;
    GUI::RoundedRectangle background(size, position, angle_radius);
    background.setFillColor({200, 200, 200});
    GUI::Item::draw(target, background);
  }
};

}  // namespace edtr
