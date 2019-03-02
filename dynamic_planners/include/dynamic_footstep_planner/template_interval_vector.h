#ifndef DYNAMIC_PLANNERS_TEMPLATE_INVERVAL_VECTOR_H_
#define DYNAMIC_PLANNERS_TEMPLATE_INVERVAL_VECTOR_H_

#include "math_std/common.h"

template<typename T>
  struct IntervalVector
  {
    Real minimum;
    Real maximum;
    Real resolution;
    Real resolutionRecip;

    std::vector<T> elements;

    IntervalVector()
    {
      resize(0.0, 0.0, 1.0);
    }

    IntervalVector(const Real &minimum, const Real &maximum, const Real &resolution)
    {
      resize(minimum, maximum, resolution);
    }

    T &operator[](const UInt &index)
    {
      return elements[index];
    }

    const T &operator[](const UInt &index) const
    {
      return elements[index];
    }

    const UInt size() const
    {
      return elements.size();
    }

    void resize(const Real &minimum, const Real &maximum, const Real &resolution)
    {
      elements.clear();
      this->maximum = maximum;
      this->minimum = minimum;
      this->resolution = resolution;
      resolutionRecip = 1.0 / resolution;

      UInt vectorSize = 1;
      if (maximum > minimum)
        vectorSize = (UInt)(std::ceil((maximum - minimum) * resolutionRecip));

      elements.resize(vectorSize);

    }

    void resize(const Real &minimum, const Real &maximum, const Real &resolution, const T &initializer)
    {
      elements.clear();
      this->maximum = maximum;
      this->minimum = minimum;
      this->resolution = resolution;
      resolutionRecip = 1.0 / resolution;

      UInt vectorSize = 1;
      if (maximum > minimum)
        vectorSize = (UInt)(std::ceil((maximum - minimum) * resolutionRecip));
      elements.resize(vectorSize, initializer);
    }

    void add(const T &element)
    {
      maximum += resolution;
      elements.push_back(element);
    }

    void setMinimum(const Real &minimum)
    {
      this->minimum = minimum;
      if (maximum < minimum)
        maximum = minimum;

      UInt vectorSize = 1;
      if (maximum > minimum)
        vectorSize = (UInt)(std::ceil((maximum - minimum) / resolution));
      elements.resize(vectorSize);
    }

    void setMaximum(const Real &maximum)
    {
      this->maximum = maximum;
      if (minimum > maximum)
        minimum = maximum;

      UInt vectorSize = 1;
      if (maximum > minimum)
        vectorSize = (UInt)(std::ceil((maximum - minimum) / resolution));
      elements.resize(vectorSize);
    }

    T &get(const Real &position)
    {
      Int index = (Int)((position - minimum) * resolutionRecip);
      if (index < 0)
        return elements[0];
      else if (index > elements.size() - 1)
        return elements.back();
      else
        return elements[index];
    }

    const T &get(const Real &position) const
    {
      Int index = (Int)((position - minimum) * resolutionRecip);
      if (index < 0)
        return elements[0];
      else if (index > elements.size() - 1)
        return elements.back();
      else
        return elements[index];
    }

    const UInt getIndex(const Real &position) const
    {
      Int index = (Int)((position - minimum) * resolutionRecip);
      if (index < 0)
        return 0;
      else if (index > elements.size() - 1)
        return elements.size() - 1;
      else
        return (UInt)index;
    }

    const Real getPosition(const UInt &index) const
    {
      return minimum + resolution * (0.5 + index);
    }

    const Real getPositionCenter(const Real &position) const
    {
      return getPosition(getIndex(position));
    }

    const bool inRange(const Real &position) const
    {
      bool isInRange = !(position < minimum || position > maximum);
      return isInRange;
    }
  };

#endif // DYNAMIC_PLANNERS_TEMPLATE_INVERVAL_VECTOR_H_
