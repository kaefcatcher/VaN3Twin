#ifndef HARM_UTIL_H
#define HARM_UTIL_H

#include <string>

namespace ns3 {

// Canonical harm metric shared by the optimizer (CalculateHarm /
// CalculateOptimalDeceleration), the time-sampled HarmLogger, and the
// per-message MSGLOG harm. Fix 1 of the remediation plan requires the
// quantity the optimizer minimizes to be EXACTLY the quantity the figures
// plot, so every harm number in the project routes through one function.
//
//   kDeltaV : paper formula (3), H = m_other / (m_self + m_other) * |v_rel|.
//             Units: kg-normalized m/s. This is the project default because
//             every committed figure / CSV "harm" column reports it.
//   kKineticEnergy : reduced-mass collision kinetic energy, ½ * mu * v_rel²
//             with mu = m_self*m_other/(m_self+m_other). Units: joules.
enum class HarmMetric
{
  kDeltaV,
  kKineticEnergy
};

// Parse a config/attribute string into a HarmMetric. Accepts "deltaV"
// (default) and "kinetic_energy"; any other value falls back to kDeltaV.
HarmMetric ParseHarmMetric (const std::string &name);

// Short human-readable name for a HarmMetric (the inverse of
// ParseHarmMetric), used in log lines and CSV provenance.
std::string HarmMetricName (HarmMetric metric);

/**
 * Canonical pairwise harm under the selected metric.
 *
 * Returns the harm that vehicle "self" (mass m_self, speed v_self) suffers
 * in a hypothetical collision with vehicle "other" (mass m_other, speed
 * v_other), evaluated from the relative speed |v_self - v_other|:
 *   - kDeltaV         -> m_other / (m_self + m_other) * |v_self - v_other|
 *   - kKineticEnergy  -> ½ * mu * (v_self - v_other)²,  mu = reduced mass
 *
 * If the two masses sum to a non-positive value the function returns 0.
 */
double appUtil_harm (HarmMetric metric, double m_self, double v_self,
                     double m_other, double v_other);

/**
 * Canonical pairwise harm from an already-resolved relative speed v_rel
 * (>= 0). The optimizer's collision solver produces v_rel at the collision
 * instant, so it calls this overload to score exactly the same metric the
 * loggers report. v_rel < 0 is treated as 0.
 */
double appUtil_harmFromVrel (HarmMetric metric, double m_self,
                             double m_other, double v_rel);

/**
 * Backward-compatible alias for the paper-formula-(3) harm
 *   H = m2 / (m1 + m2) * |v1 - v2|.
 * Kept so existing call sites / tests stay valid; equivalent to
 * appUtil_harm(HarmMetric::kDeltaV, m1, v1, m2, v2).
 */
double appUtil_pairwiseHarm (double m1, double v1, double m2, double v2);

} // namespace ns3

#endif /* HARM_UTIL_H */
