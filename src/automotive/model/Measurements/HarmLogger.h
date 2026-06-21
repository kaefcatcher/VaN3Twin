#ifndef HARM_LOGGER_H
#define HARM_LOGGER_H

#include "ns3/event-id.h"
#include "ns3/ptr.h"
#include "ns3/simple-ref-count.h"
#include "ns3/traci-client.h"

#include "harm-util.h"

#include <fstream>
#include <string>

namespace ns3 {

/**
 * Time-sampled pairwise HARM logger (ground-truth, collision-aware).
 *
 * Once per period this class pulls the SUMO vehicle list via TraCI and, for
 * every unordered pair (i, j) with i < j, writes one CSV row of the harm the
 * pair would suffer in a collision — but ONLY for pairs that are physically
 * able to collide:
 *
 *   1. centres within radiusMeters of each other,
 *   2. travelling in roughly the same direction (heading difference below
 *      headingTolDeg) — Fix 3: drops oncoming / cross-traffic pairs,
 *   3. genuinely closing: the rear vehicle is faster than the one ahead along
 *      the line of approach — Fix 3: drops separating / co-moving pairs.
 *
 * The harm value uses the project's CANONICAL metric (Fix 1: the same metric
 * the optimizer minimizes), and each vehicle's mass is read per-vehicle from
 * its SUMO vType (Fix 5) rather than a single constant. The CSV is:
 *
 *     time,car1ID,car2ID,harm,energy
 *
 * where `harm` is the selected metric and `energy` is the reduced-mass kinetic
 * energy of the closing collision (kept for backward-compatible plots).
 *
 * The metric reads SUMO ground truth (positions, speeds) directly — it does
 * not depend on whether any V2X message was received, so it isolates the
 * algorithm's safety effect from PRR / channel confounders.
 *
 * One instance per simulation. Construct with the TraCI client, start via
 * Start() *after* Simulator has been set up, and the destructor closes the
 * file. Cancellation on Simulator::Stop is automatic via the EventId.
 */
class HarmLogger : public SimpleRefCount<HarmLogger>
{
public:
  HarmLogger (Ptr<TraciClient> traci,
              const std::string &outputFile,
              double periodSeconds,
              double radiusMeters,
              HarmMetric metric = HarmMetric::kDeltaV,
              double defaultMassKg = 1500.0,
              double headingTolDeg = 30.0);
  ~HarmLogger ();

  /** Schedule the first tick. Idempotent. */
  void Start ();
  void Stop ();

private:
  void Tick ();
  /** Per-vehicle mass from the SUMO "mass" vType param (Fix 5); falls back to
      the default mass when TraCI has no usable value. */
  double VehicleMass (const std::string &id);

  Ptr<TraciClient> m_traci;
  std::string m_output_file;
  double m_period_s;
  double m_radius_m;
  HarmMetric m_metric;
  double m_default_mass_kg;
  double m_heading_tol_deg;

  std::ofstream m_ofstream;
  EventId m_tick_event;
  bool m_started;
};

} // namespace ns3

#endif /* HARM_LOGGER_H */
