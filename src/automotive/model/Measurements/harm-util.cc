#include "harm-util.h"

#include <cmath>

namespace ns3 {

double
appUtil_pairwiseHarm (double m1, double v1, double m2, double v2)
{
  double totalMass = m1 + m2;
  if (totalMass <= 0.0)
    return 0.0;
  return (m2 / totalMass) * std::abs (v1 - v2);
}

} // namespace ns3
