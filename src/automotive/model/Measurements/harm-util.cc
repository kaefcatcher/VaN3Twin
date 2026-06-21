#include "harm-util.h"

#include <cmath>

namespace ns3 {

HarmMetric
ParseHarmMetric (const std::string &name)
{
  if (name == "kinetic_energy")
    return HarmMetric::kKineticEnergy;
  // "deltaV" and anything unrecognized map to the project default.
  return HarmMetric::kDeltaV;
}

std::string
HarmMetricName (HarmMetric metric)
{
  switch (metric)
    {
    case HarmMetric::kKineticEnergy:
      return "kinetic_energy";
    case HarmMetric::kDeltaV:
    default:
      return "deltaV";
    }
}

double
appUtil_harmFromVrel (HarmMetric metric, double m_self, double m_other,
                      double v_rel)
{
  double total_mass = m_self + m_other;
  if (total_mass <= 0.0)
    return 0.0;
  if (v_rel < 0.0)
    v_rel = 0.0;
  switch (metric)
    {
    case HarmMetric::kKineticEnergy:
      {
        double mu = (m_self * m_other) / total_mass;
        return 0.5 * mu * v_rel * v_rel;
      }
    case HarmMetric::kDeltaV:
    default:
      return (m_other / total_mass) * v_rel;
    }
}

double
appUtil_harm (HarmMetric metric, double m_self, double v_self, double m_other,
              double v_other)
{
  return appUtil_harmFromVrel (metric, m_self, m_other,
                               std::abs (v_self - v_other));
}

double
appUtil_pairwiseHarm (double m1, double v1, double m2, double v2)
{
  return appUtil_harm (HarmMetric::kDeltaV, m1, v1, m2, v2);
}

} // namespace ns3
