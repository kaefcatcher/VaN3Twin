#ifndef HARM_LOGGER_H
#define HARM_LOGGER_H

#include "ns3/event-id.h"
#include "ns3/ptr.h"
#include "ns3/simple-ref-count.h"
#include "ns3/traci-client.h"

#include <fstream>
#include <string>

namespace ns3 {

/**
 * Time-sampled pairwise HARM logger.
 *
 * Once per period this class pulls the SUMO vehicle list via TraCI,
 * computes appUtil_pairwiseHarm(mi, vi, mj, vj) for every unordered
 * pair (i, j) with i < j whose centres are within radius_m of each
 * other, and writes one row per pair to a single CSV file:
 *
 *     time,car1ID,car2ID,harm
 *
 * The metric reads SUMO ground truth (positions, speeds) directly —
 * it does not depend on whether any V2X message was received, so it
 * isolates the algorithm's safety effect from PRR / channel
 * confounders.
 *
 * One instance per simulation. Construct with the TraCI client, start
 * via Start() *after* Simulator has been set up (the first tick must
 * fire after SUMO has stepped once), and the destructor closes the
 * file. Cancellation on Simulator::Stop is automatic via the EventId.
 */
class HarmLogger : public SimpleRefCount<HarmLogger>
{
public:
  HarmLogger (Ptr<TraciClient> traci,
              const std::string &outputFile,
              double periodSeconds,
              double radiusMeters,
              double defaultMassKg = 1500.0);
  ~HarmLogger ();

  /** Schedule the first tick. Idempotent. */
  void Start ();
  void Stop ();

private:
  void Tick ();

  Ptr<TraciClient> m_traci;
  std::string m_output_file;
  double m_period_s;
  double m_radius_m;
  double m_default_mass_kg;

  std::ofstream m_ofstream;
  EventId m_tick_event;
  bool m_started;
};

} // namespace ns3

#endif /* HARM_LOGGER_H */
