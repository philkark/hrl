//ros
#include <ros/ros.h>

//local
#include "rviz_ros_communication_plugin/category.h"

namespace rviz_ros_communication_plugin
{

//////////////////////////////////////////////////////////////////

//////////////////// PUBLIC //////////////////////////////////////

//////////////////////////////////////////////////////////////////

Category::Category(const std::string &name) :
    name(name)
{
  scrollArea = new QScrollArea;
  scrollArea->setVerticalScrollBarPolicy(Qt::ScrollBarAsNeeded);
  scrollArea->setWidgetResizable(true);
  QWidget* scrollWidget = new QWidget(scrollArea);
  scrollArea->setWidget(scrollWidget);
  layout = new QVBoxLayout(scrollWidget);
  layout->setAlignment(Qt::AlignmentFlag::AlignTop);

  QPalette palette;
  palette.setColor(QPalette::Background, Qt::white);
  scrollWidget->setPalette(palette);
}

QString Category::getName() const
{
  return QString::fromStdString(name);
}

QScrollArea* Category::getWidget() const
{
  return scrollArea;
}

void Category::addHeadLine(const std::string &name, const int fontSize)
{
  QLabel* label = new QLabel(QString::fromStdString(name));
  label->setAlignment(Qt::AlignCenter);
  label->setContentsMargins(0, 10, 0, 4);

  QFont font;
  font.setPointSize(fontSize);
  label->setFont(font);
  layout->addWidget(label);
}

void Category::addInfoText(const std::string &topic)
{
  infoTexts.push_back(new InfoText(topic));

  layout->addWidget(infoTexts.back()->getLabel());
}

void Category::addSlider(DoubleSlider* slider)
{
  doubleSliders.push_back(slider);
  layout->addLayout(slider->getLayout());
}

void Category::addSlider(IntSlider* slider)
{
  intSliders.push_back(slider);
  layout->addLayout(slider->getLayout());
}

void Category::addButtons(const std::vector<AbstractButton*> &buttons)
{
  QHBoxLayout* layoutTmp = new QHBoxLayout;

  for (int i = 0; i < buttons.size(); ++i)
  {
    layoutTmp->addWidget(buttons[i]->getButton());

    switch (buttons[i]->getType())
    {
      case AbstractButton::Toggle:
        toggleButtons.push_back(reinterpret_cast<ToggleButton*>(buttons[i]));
        break;
      case AbstractButton::Menu:
        menuButtons.push_back(reinterpret_cast<MenuButton*>(buttons[i]));
        break;
    }
  }

  layout->addLayout(layoutTmp);
}

void Category::addTextField(TextField* textField)
{
  textFields.push_back(textField);
  layout->addLayout(textField->getLayout());
}

void Category::finalizeLayout()
{
  adjustLabelWidths();
}

int Category::getIntSliderSettingsCount() const
{
  return intSliders.size();
}

std::vector<int> Category::getIntSliderSettings() const
{
  std::vector<int> vec(intSliders.size());

  for (int i = 0; i < vec.size(); ++i)
    vec[i] = intSliders[i]->getValue();

  return vec;
}

void Category::setIntSliderSettings(const std::vector<int> &settings)
{
  for (int i = 0; i < settings.size(); ++i)
    intSliders[i]->setValue(settings[i]);
}

int Category::getDoubleSliderSettingsCount() const
{
  return doubleSliders.size();
}

std::vector<double> Category::getDoubleSliderSettings() const
{
  std::vector<double> vec(doubleSliders.size());

  for (int i = 0; i < vec.size(); ++i)
    vec[i] = doubleSliders[i]->getValue();

  return vec;
}

void Category::setDoubleSliderSettings(const std::vector<double> &settings)
{
  for (int i = 0; i < doubleSliders.size(); ++i)
    doubleSliders[i]->setValue(settings[i]);
}

int Category::getTextFieldSettingsCount() const
{
  return textFields.size();
}

std::vector<QString> Category::getTextFieldSettings() const
{
  std::vector<QString> vec(textFields.size());

  for (int i = 0; i < vec.size(); ++i)
    vec[i] = textFields[i]->getContent();

  return vec;
}

void Category::setTextFieldSettings(const std::vector<QString> &settings)
{
  for (int i = 0; i < textFields.size(); ++i)
    textFields[i]->setContent(settings[i]);
}

int Category::getMenuButtonSettingsCount() const
{
  return menuButtons.size();
}

std::vector<int> Category::getMenuButtonSettings() const
{
  std::vector<int> vec(menuButtons.size());

  for (int i = 0; i < menuButtons.size(); ++i)
    vec[i] = menuButtons[i]->getIndex();

  return vec;
}

void Category::setMenuButtonSettings(const std::vector<int> &settings)
{
  for (int i = 0; i < menuButtons.size(); ++i)
    menuButtons[i]->setIndex(settings[i]);
}

//////////////////////////////////////////////////////////////////

//////////////////// PRIVATE /////////////////////////////////////

//////////////////////////////////////////////////////////////////

void Category::adjustLabelWidths()
{
  int width = 0;
  for (int i = 0; i < doubleSliders.size(); ++i)
  {
    int widthTmp = doubleSliders[i]->getLabelWidth();
    if (widthTmp > width)
      width = widthTmp;
  }
  for (int i = 0; i < intSliders.size(); ++i)
  {
    int widthTmp = intSliders[i]->getLabelWidth();
    if (widthTmp > width)
      width = widthTmp;
  }
  for (int i = 0; i < textFields.size(); ++i)
  {
    int widthTmp = textFields[i]->getLabelWidth();
    if (widthTmp > width)
      width = widthTmp;
  }

  for (int i = 0; i < doubleSliders.size(); ++i)
    doubleSliders[i]->setLabelWidth(width);
  for (int i = 0; i < intSliders.size(); ++i)
    intSliders[i]->setLabelWidth(width);
  for (int i = 0; i < textFields.size(); ++i)
    textFields[i]->setLabelWidth(width);
}

} // namespace rviz_ros_communication_plugin
