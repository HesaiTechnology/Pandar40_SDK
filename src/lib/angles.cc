#include <math.h>

namespace angles
{
  double from_degrees(double degrees)
  {
    return degrees * M_PI / 180;
  }
}