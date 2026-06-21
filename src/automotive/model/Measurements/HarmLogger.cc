#include "HarmLogger.h"

#include "harm-util.h"

#include "ns3/log.h"
#include "ns3/simulator.h"

#include <cmath>
#include <string>
#include <vector>

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("HarmLogger");

namespace {

// Smallest absolute difference between two compass headings in [0,360) deg.
double
HeadingDiffDeg (double a, double b)
{
  double d = std::fabs (a - b);
  while (d > 360.0)
    d -= 360.0;
  if (d > 180.0)
    d = 360.0 - d;
  return d;
}

} // namespace

HarmLogger::HarmLogger (Ptr<TraciClient> traci,
                        const std::string &outputFile,
                        double periodSeconds,
                        double radiusMeters,
                        HarmMetric metric,
                        double defaultMassKg,
                        double headingTolDeg)
    : m_traci (traci),
      m_output_file (outputFile),
      m_period_s (periodSeconds),
      m_radius_m (radiusMeters),
      m_metric (metric),
      m_default_mass_kg (defaultMassKg),
      m_heading_tol_deg (headingTolDeg),
      m_started (false)
{
}

HarmLogger::~HarmLogger ()
{
  Stop ();
  if (m_ofstream.is_open ())
    m_ofstream.close ();
}

double
HarmLogger::VehicleMass (const std::string &id)
{
  try
    {
      std::string raw = m_traci->TraCIAPI::vehicle.getParameter (id, "mass");
      if (!raw.empty ())
        {
          double m = std::stod (raw);
          if (m > 0.0)
            return m;
        }
    }
  catch (...)
    {
      // fall through to default
    }
  return m_default_mass_kg;
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
  m_ofstream << "time,car1ID,car2ID,harm,energy" << std::endl;

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
      return;
    }

  struct Snap
  {
    std::string id;
    double x;
    double y;
    double speed;
    double heading; // degrees
    double mass;    // kg, per-vehicle (Fix 5)
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
          s.heading = m_traci->TraCIAPI::vehicle.getAngle (id);
          s.mass = VehicleMass (id);
        }
      catch (...)
        {
          continue;
        }
      snap.push_back (s);
    }

  const double t = Simulator::Now ().GetSeconds ();
  for (size_t i = 0; i < snap.size (); ++i)
    {
      for (size_t j = i + 1; j < snap.size (); ++j)
        {
          double dx = snap[i].x - snap[j].x;
          double dy = snap[i].y - snap[j].y;
          double gap = std::sqrt (dx * dx + dy * dy);
          // Fix 3 gate (1): pairing radius.
          if (gap > m_radius_m || gap < 1e-6)
            continue;
          // Fix 3 gate (2): same travel direction (drops oncoming / cross).
          if (HeadingDiffDeg (snap[i].heading, snap[j].heading) > m_heading_tol_deg)
            continue;

          // Fix 3 gate (3): genuinely closing. Identify which vehicle is behind
          // along the shared direction of travel and require it to be faster.
          // SUMO angle is degrees clockwise from north; unit travel vector is
          // (sin θ, cos θ). The vehicle that is "behind" sees the other ahead
          // along +travel, i.e. the displacement (other - self) projects
          // positively on its travel vector.
          double th = snap[i].heading * M_PI / 180.0;
          double ux = std::sin (th);
          double uy = std::cos (th);
          // displacement from i to j projected on travel direction:
          double proj = (snap[j].x - snap[i].x) * ux + (snap[j].y - snap[i].y) * uy;

          double v_behind, v_ahead;
          double m_behind, m_ahead;
          if (proj > 0.0)
            {
              // j is ahead of i: i is the (rear) follower.
              v_behind = snap[i].speed;
              v_ahead = snap[j].speed;
              m_behind = snap[i].mass;
              m_ahead = snap[j].mass;
            }
          else
            {
              // i is ahead of j: j is the (rear) follower.
              v_behind = snap[j].speed;
              v_ahead = snap[i].speed;
              m_behind = snap[j].mass;
              m_ahead = snap[i].mass;
            }
          double v_rel = v_behind - v_ahead;
          if (v_rel <= 0.0)
            continue; // separating or co-moving -> no harm logged

          // Fix 1: harm from the canonical metric; Fix 5: per-vehicle masses.
          double harm =
              appUtil_harmFromVrel (m_metric, m_behind, m_ahead, v_rel);
          double total = m_behind + m_ahead;
          double mu = (total > 0.0) ? (m_behind * m_ahead) / total : 0.0;
          double energy = 0.5 * mu * v_rel * v_rel;
          m_ofstream << t << ',' << snap[i].id << ',' << snap[j].id << ','
                     << harm << ',' << energy << '\n';
        }
    }
  m_ofstream.flush ();

  m_tick_event =
      Simulator::Schedule (Seconds (m_period_s), &HarmLogger::Tick, this);
}

} // namespace ns3
