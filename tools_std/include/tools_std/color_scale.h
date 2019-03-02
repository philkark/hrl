#ifndef TOOLS_STD_COLOR_SCALE_H_
#define TOOLS_STD_COLOR_SCALE_H_

#include <math_std/common.h>
#include <vector>

struct Color
{
  Real r;
  Real g;
  Real b;
};

class ColorScale
{
public:
  enum Mode
  {
    MGray,
    MColor
  };

  ColorScale(Mode mode);

  const Color &getColor(Real value) const;

private:
  std::vector<Color> colors;

  void initializeScaleGray();

  void initializeScaleColor();
};

#endif // TOOLS_STD_COLOR_SCALE_H_
