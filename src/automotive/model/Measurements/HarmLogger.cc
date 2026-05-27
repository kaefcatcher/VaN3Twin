#include "HarmLogger.h"

#include "harm-util.h"

#include "ns3/log.h"
#include "ns3/simulator.h"

#include <cmath>
#include <vector>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("HarmLogger");

HarmLogger::HarmLogger (Ptr<TraciClient> traci,
                        const std::string &outputFile,
                        double periodSeconds,
                        double radiusMeters,
                        double defaultMassKg)
    : m_traci (traci),
      m_output_file (outputFile),
      m_period_s (periodSeconds),
      m_radius_m (radiusMeters),
      m_default_mass_kg (defaultMassKg),
      m_started (false)
{
}

HarmLogger::~HarmLogger ()
{
  Stop ();
  if (m_ofstream.is_open ())
    m_ofstream.close ();
}

void
HarmLogger::Start ()
{
  if (m_started)
    return;
  m_started = true;

  m_ofstream.open (m_output_file, std::ofstream::trunc);
  if (!m_ofstream.is_open ())
    {
      NS_LOG_ERROR ("HarmLogger: cannot open " << m_output_file);
      return;
    }
  // CSV schema:
  //   time     simulator time (s)
  //   car1ID   SUMO id of vehicle i (alphabetically first of the pair)
  //   car2ID   SUMO id of vehicle j
  //   harm     m2/(m1+m2) * |v1-v2|        — paper formula 3 (momentum-diff)
  //   gap      Euclidean centre distance (m)
  //   ttc      gap / |v1-v2| (s) if speeds differ, else 1e6 (capped)
  //   energy   ½·μ·|v1-v2|²      (J)        — kinetic energy of a perfectly
  //                                            inelastic collision at this Δv,
  //                                            where μ = m1·m2/(m1+m2)
  // The richer columns are needed because pairwise `harm` alone doesn't
  // distinguish "two cars moving apart" from "two cars about to crash".
  // For collision-risk analysis use min(ttc) and max(energy) per pair.
  m_ofstream << "time,car1ID,car2ID,harm,gap,ttc,energy" << std::endl;

  // First tick fires one period in the future so SUMO has already
  // stepped at least once and getIDList() is non-empty in the typical
  // scenario.
  m_tick_event =
      Simulator::Schedule (Seconds (m_period_s), &HarmLogger::Tick, this);
}

void
HarmLogger::Stop ()
{
  Simulator::Cancel (m_tick_event);
}

void
HarmLogger::Tick ()
{
  if (!m_ofstream.is_open ())
    return;

  std::vector<std::string> ids;
  try
    {
      ids = m_traci->TraCIAPI::vehicle.getIDList ();
    }
  catch (...)
    {
      // SUMO may be tearing down at the very end of the simulation.
      return;
    }

  // Snapshot positions and speeds once so we don't pay TraCI cost N²
  // times per pair.
  struct Snap
  {
    std::string id;
    double x;
    double y;
    double speed;
  };
  std::vector<Snap> snap;
  snap.reserve (ids.size ());
  for (const auto &id : ids)
    {
      Snap s;
      s.id = id;
      try
        {
          auto pos = m_traci->TraCIAPI::vehicle.getPosition (id);
          s.x = pos.x;
          s.y = pos.y;
          s.speed = m_traci->TraCIAPI::vehicle.getSpeed (id);
        }
      catch (...)
        {
          continue;
        }
      snap.push_back (s);
    }

  const double t = Simulator::Now ().GetSeconds ();
  const double m = m_default_mass_kg;
  const double mu = (m * m) / (m + m); // = m/2 with equal masses
  const double TTC_CAP = 1e6;          // 1e6 s = effectively "no closure"
  for (size_t i = 0; i < snap.size (); ++i)
    {
      for (size_t j = i + 1; j < snap.size (); ++j)
        {
          double dx = snap[i].x - snap[j].x;
          double dy = snap[i].y - snap[j].y;
          double gap = std::sqrt (dx * dx + dy * dy);
          if (gap > m_radius_m)
            continue;
          double dv = std::abs (snap[i].speed - snap[j].speed);
          double harm = appUtil_pairwiseHarm (m, snap[i].speed,
                                              m, snap[j].speed);
          double ttc = (dv > 1e-6) ? (gap / dv) : TTC_CAP;
          double energy = 0.5 * mu * dv * dv;
          m_ofstream << t << ',' << snap[i].id << ',' << snap[j].id << ','
                     << harm << ',' << gap << ',' << ttc << ',' << energy
                     << '\n';
        }
    }
  m_ofstream.flush ();

  m_tick_event =
      Simulator::Schedule (Seconds (m_period_s), &HarmLogger::Tick, this);
}

} // namespace ns3
