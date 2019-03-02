#include <tools_std/color_scale.h>

// *********************************************************************

// ************************** Color ************************************

// *********************************************************************

ColorScale::ColorScale(Mode mode)
{
  switch (mode)
  {
    case Mode::MGray:
      initializeScaleGray();
      break;
    case Mode::MColor:
      initializeScaleColor();
      break;
    default:
      initializeScaleColor();
      break;
  }
}

const Color &ColorScale::getColor(Real value) const
{
  if (value < 0.0)
    value = 0.0;
  else if (value > 1.0)
    value = 1.0;

  return colors[(UInt)(value * ((Real)colors.size() - 1))];
}

void ColorScale::initializeScaleGray()
{
  colors.resize(256);
  for (UInt i = 0; i < 256; ++i)
  {
    colors[i].r = colors[i].g = colors[i].b = i / 255.0;
  }
}

void ColorScale::initializeScaleColor()
{
  colors.resize(256);
  for (UInt i = 0; i < 256; ++i)
  {
    colors[i].b = 0.0;
    colors[i].r = (255 - i) / 255.0;
    colors[i].g = i / 255.0;

    if (colors[i].g > 1.0)
      colors[i].g = 1.0;
    else if (colors[i].g < 0.0)
      colors[i].g = 0.0;

    if (colors[i].r > 1.0)
      colors[i].r = 1.0;
    else if (colors[i].r < 0.0)
      colors[i].r = 0.0;
  }
}
