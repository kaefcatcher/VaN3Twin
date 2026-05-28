/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation;
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 * Created by:
 *  Marco Malinverno, Politecnico di Torino (marco.malinverno1@gmail.com)
 *  Francesco Raviglione, Politecnico di Torino (francescorav.es483@gmail.com)
 *  Carlos Mateo Risma Carletti, Politecnico di Torino (carlosrisma@gmail.com)
*/

#include "emergencyVehicleAlert.h"

#include "ns3/CAM.h"
#include "ns3/DENM.h"
#include "ns3/socket.h"
#include "ns3/network-module.h"
#include "ns3/gn-utils.h"
#include "ns3/harm-util.h"

#define DEG_2_RAD(val) ((val) *M_PI / 180.0)

namespace ns3 {

NS_LOG_COMPONENT_DEFINE ("emergencyVehicleAlert");

NS_OBJECT_ENSURE_REGISTERED (emergencyVehicleAlert);

// Function to compute the distance between two objects, given their Lon/Lat
double
appUtil_haversineDist (double lat_a, double lon_a, double lat_b, double lon_b)
{
  // 12742000 is the mean Earth radius (6371 km) * 2 * 1000 (to convert from km to m)
  return 12742000.0 *
         asin (sqrt (sin (DEG_2_RAD (lat_b - lat_a) / 2) * sin (DEG_2_RAD (lat_b - lat_a) / 2) +
                     cos (DEG_2_RAD (lat_a)) * cos (DEG_2_RAD (lat_b)) *
                         sin (DEG_2_RAD (lon_b - lon_a) / 2) *
                         sin (DEG_2_RAD (lon_b - lon_a) / 2)));
}

// appUtil_pairwiseHarm now lives in ns3/harm-util.h so both the
// application and HarmLogger share one definition.

// Function to compute the absolute difference between two angles (angles must be between -180 and 180)
double
appUtil_angDiff (double ang1, double ang2)
{
  double angDiff;
  angDiff = ang1 - ang2;

  if (angDiff > 180)
    {
      angDiff -= 360;
    }
  else if (angDiff < -180)
    {
      angDiff += 360;
    }
  return std::abs (angDiff);
}

TypeId
emergencyVehicleAlert::GetTypeId (void)
{
  static TypeId tid =
      TypeId ("ns3::emergencyVehicleAlert")
          .SetParent<Application> ()
          .SetGroupName ("Applications")
          .AddConstructor<emergencyVehicleAlert> ()
          .AddAttribute ("RealTime", "To compute properly timestamps", BooleanValue (false),
                         MakeBooleanAccessor (&emergencyVehicleAlert::m_real_time),
                         MakeBooleanChecker ())
          .AddAttribute ("IpAddr", "IpAddr", Ipv4AddressValue ("10.0.0.1"),
                         MakeIpv4AddressAccessor (&emergencyVehicleAlert::m_ipAddress),
                         MakeIpv4AddressChecker ())
          .AddAttribute (
              "PrintSummary", "To print summary at the end of simulation", BooleanValue (false),
              MakeBooleanAccessor (&emergencyVehicleAlert::m_print_summary), MakeBooleanChecker ())
          .AddAttribute ("CSV", "CSV log name", StringValue (),
                         MakeStringAccessor (&emergencyVehicleAlert::m_csv_name),
                         MakeStringChecker ())
          .AddAttribute ("Model", "Physical and MAC layer communication model", StringValue (""),
                         MakeStringAccessor (&emergencyVehicleAlert::m_model), MakeStringChecker ())
          .AddAttribute ("Client", "TraCI client for SUMO", PointerValue (0),
                         MakePointerAccessor (&emergencyVehicleAlert::m_client),
                         MakePointerChecker<TraciClient> ())
          .AddAttribute (
              "MetricSupervisor",
              "Metric Supervisor to compute metrics according to 3GPP TR36.885 V14.0.0 page 70",
              PointerValue (0), MakePointerAccessor (&emergencyVehicleAlert::m_metric_supervisor),
              MakePointerChecker<MetricSupervisor> ())
          .AddAttribute (
              "SendCAM", "To enable/disable the transmission of CAM messages", BooleanValue (true),
              MakeBooleanAccessor (&emergencyVehicleAlert::m_send_cam), MakeBooleanChecker ())
          .AddAttribute (
              "SendCPM", "To enable/disable the transmission of CPM messages", BooleanValue (true),
              MakeBooleanAccessor (&emergencyVehicleAlert::m_send_cpm), MakeBooleanChecker ())
          .AddAttribute (
              "SendDENM", "To enable/disable DENM event triggering (hard brake, collision risk). "
                          "Set false to produce a CAM-only baseline for comparison.",
              BooleanValue (true),
              MakeBooleanAccessor (&emergencyVehicleAlert::m_send_denm), MakeBooleanChecker ())
          .AddAttribute (
              "HardBrakeThreshold",
              "Acceleration threshold (m/s^2) below which hard braking is detected",
              DoubleValue (-4.0),
              MakeDoubleAccessor (&emergencyVehicleAlert::m_hard_brake_threshold),
              MakeDoubleChecker<double> ())
          .AddAttribute (
              "CollisionRiskDistance",
              "Distance threshold (m) for collision risk detection with leading vehicle",
              DoubleValue (50.0),
              MakeDoubleAccessor (&emergencyVehicleAlert::m_collision_risk_distance),
              MakeDoubleChecker<double> ())
          .AddAttribute (
              "EventCheckInterval",
              "Interval (s) for periodic event detection checks",
              DoubleValue (0.1),
              MakeDoubleAccessor (&emergencyVehicleAlert::m_event_check_interval),
              MakeDoubleChecker<double> ())
          .AddAttribute (
              "EthicalBraking",
              "Enable ethical V2X braking fields (maxDeceleration, brakingStartTime)",
              BooleanValue (false),
              MakeBooleanAccessor (&emergencyVehicleAlert::m_ethical_braking_enabled),
              MakeBooleanChecker ())
          .AddAttribute (
              "VehicleMass",
              "Vehicle mass in kg for DENM alacarte container",
              DoubleValue (1500.0),
              MakeDoubleAccessor (&emergencyVehicleAlert::m_vehicle_mass),
              MakeDoubleChecker<double> ())
          .AddAttribute (
              "CooperativeDetection",
              "Enable cooperative ethical braking algorithm based on harm minimization",
              BooleanValue (false),
              MakeBooleanAccessor (&emergencyVehicleAlert::m_cooperative_detection_enabled),
              MakeBooleanChecker ())
          .AddAttribute (
              "SigmaMode",
              "How V2's decision-time budget σ is determined: 'computed' (default — derive "
              "from CalculateDecisionBudget so H_{1,2}=0), 'fixed' (use FixedSigma directly), "
              "or 'scaled' (multiply the computed value by FixedSigma).",
              StringValue ("computed"),
              MakeStringAccessor (&emergencyVehicleAlert::m_sigma_mode),
              MakeStringChecker ())
          .AddAttribute (
              "FixedSigma",
              "Override or scale for σ. Interpreted per SigmaMode. Unit: seconds (mode=fixed) "
              "or unitless multiplier (mode=scaled). Ignored when SigmaMode='computed'.",
              DoubleValue (0.5),
              MakeDoubleAccessor (&emergencyVehicleAlert::m_fixed_sigma),
              MakeDoubleChecker<double> ())
          .AddAttribute (
              "SpeedDropThreshold",
              "Speed drop (m/s) within a 1 s window that triggers a slowVehicle (cause 26) DENM. "
              "Single-vehicle trigger, fires independently of any leader.",
              DoubleValue (3.0),
              MakeDoubleAccessor (&emergencyVehicleAlert::m_speed_drop_threshold),
              MakeDoubleChecker<double> ())
          .AddAttribute (
              "StationarySpeed",
              "Speed (m/s) below which a previously-moving vehicle is considered stationary. "
              "Triggers a stationaryVehicle (cause 94) DENM once per stop event.",
              DoubleValue (1.0),
              MakeDoubleAccessor (&emergencyVehicleAlert::m_stationary_speed),
              MakeDoubleChecker<double> ())
          .AddAttribute (
              "WasMovingSpeed",
              "Speed (m/s) the vehicle must have exceeded at some point before a stationary "
              "report is allowed. Prevents reporting a never-moved vehicle as stationary.",
              DoubleValue (5.0),
              MakeDoubleAccessor (&emergencyVehicleAlert::m_was_moving_speed),
              MakeDoubleChecker<double> ())
          .AddAttribute (
              "IncludeEthicalAlacarte",
              "If true, populate the custom alacarte extension fields (ethicalMaxDeceleration, "
              "ethicalBrakingStartTime, ethicalVehicleMass) on DENMs sent when CooperativeDetection "
              "or EthicalBraking is on. Set false to fall back to a strict ETSI EN 302 637-3 "
              "alacarte container — use this to bisect UPER encoder failures.",
              BooleanValue (false),
              MakeBooleanAccessor (&emergencyVehicleAlert::m_include_ethical_alacarte),
              MakeBooleanChecker ())
          .AddAttribute (
              "ChainBrakeFraction",
              "Fraction of the vehicle's max deceleration to apply in V3's chain / rear "
              "cooperative branch. 1.0 = paper-strict (V3 brakes at max). Values <1 trade "
              "collision-prevention margin for keeping V3's brake in sync with V2's softer "
              "optimal_a2 (avoids the V2↔V3 transient closing-gap).",
              DoubleValue (1.0),
              MakeDoubleAccessor (&emergencyVehicleAlert::m_chain_brake_fraction),
              MakeDoubleChecker<double> ());
  return tid;
}

emergencyVehicleAlert::emergencyVehicleAlert ()
{
  NS_LOG_FUNCTION (this);
  m_client = nullptr;
  m_print_summary = true;
  m_already_print = false;
  m_send_cam = true;

  m_denm_sent = 0;
  m_cam_received = 0;
  m_cpm_received = 0;
  m_denm_received = 0;
  m_denm_intertime = 0;

  m_distance_threshold = 75;
  m_heading_threshold = 45;

  m_is_event_active = false;
  m_active_action_id = {};
  m_active_detection_time_ms = 0;
  m_hard_brake_threshold = -4.0;
  m_collision_risk_distance = 50.0;
  m_event_check_interval = 0.1;
  m_vehicle_mass = 1500.0;
  m_ethical_braking_enabled = false;
  m_cooperative_detection_enabled = false;
  m_include_ethical_alacarte = false;
  m_chain_brake_fraction = 1.0;
  m_send_denm = true;
  m_sigma_mode = "computed";
  m_fixed_sigma = 0.5;
  m_speed_drop_threshold = 3.0;
  m_stationary_speed = 1.0;
  m_was_moving_speed = 5.0;
  for (int i = 0; i < SPEED_WINDOW_N; ++i)
    m_speed_window[i] = 0.0;
  m_speed_window_pos = 0;
  m_speed_window_full = false;
  m_has_been_moving = false;
  m_stationary_already_reported = false;
}

emergencyVehicleAlert::~emergencyVehicleAlert ()
{
  NS_LOG_FUNCTION (this);
}

void
emergencyVehicleAlert::DoDispose (void)
{
  NS_LOG_FUNCTION (this);
  Application::DoDispose ();
}

void
emergencyVehicleAlert::StartApplication (void)
{
  NS_LOG_FUNCTION (this);

  /* Save the vehicles informations */
  m_id = m_client->GetVehicleId (this->GetNode ());
  m_type = m_client->TraCIAPI::vehicle.getVehicleClass (m_id);
  m_max_speed = m_client->TraCIAPI::vehicle.getMaxSpeed (m_id);

  VDP *traci_vdp = new VDPTraCI (m_client, m_id);

  //Create LDM and sensor object
  m_LDM = CreateObject<LDM> ();
  m_LDM->setStationID (m_id);
  m_LDM->setTraCIclient (m_client);
  m_LDM->setVDP (traci_vdp);

  m_sensor = CreateObject<SUMOSensor> ();
  m_sensor->setStationID (m_id);
  m_sensor->setTraCIclient (m_client);
  m_sensor->setVDP (traci_vdp);
  m_sensor->setLDM (m_LDM);

  // Create new BTP and GeoNet objects and set them in DENBasicService and CABasicService
  m_btp = CreateObject<btp> ();
  m_geoNet = CreateObject<GeoNet> ();

  if (m_metric_supervisor != nullptr)
    {
      m_geoNet->setMetricSupervisor (m_metric_supervisor);
    }

  m_btp->setGeoNet (m_geoNet);
  m_denService.setBTP (m_btp);
  m_caService.setBTP (m_btp);
  m_cpService.setBTP (m_btp);
  m_caService.setLDM (m_LDM);
  m_cpService.setLDM (m_LDM);

  /* Create the Sockets for TX and RX */
  TypeId tid;
  if (m_model == "80211p")
    tid = TypeId::LookupByName ("ns3::PacketSocketFactory");
  else if (m_model == "cv2x" || m_model == "nrv2x")
    tid = TypeId::LookupByName ("ns3::UdpSocketFactory");
  else
    NS_FATAL_ERROR (
        "No communication model set - check simulation script - valid models: '80211p' or 'lte'");
  m_socket = Socket::CreateSocket (GetNode (), tid);

  if (m_model == "80211p")
    {
      /* Bind the socket to local address */
      PacketSocketAddress local = getGNAddress (GetNode ()->GetDevice (0)->GetIfIndex (),
                                                GetNode ()->GetDevice (0)->GetAddress ());
      if (m_socket->Bind (local) == -1)
        {
          NS_FATAL_ERROR ("Failed to bind client socket for BTP + GeoNetworking (802.11p)");
        }
      // Set the socketAddress for broadcast
      PacketSocketAddress remote = getGNAddress (GetNode ()->GetDevice (0)->GetIfIndex (),
                                                 GetNode ()->GetDevice (0)->GetBroadcast ());
      m_socket->Connect (remote);
    }
  else // m_model=="cv2x"
    {
      if (m_socket->Bind (InetSocketAddress (Ipv4Address::GetAny (), 19)) == -1)
        {
          NS_FATAL_ERROR ("Failed to bind client socket for C-V2X");
        }
      m_socket->Connect (InetSocketAddress (m_ipAddress, 19));
    }

  /* Set Station Type in DENBasicService */
  StationType_t stationtype;
  if (m_type == "passenger")
    stationtype = StationType_passengerCar;
  else if (m_type == "emergency")
    {
      stationtype = StationType_specialVehicle;
      m_LDM->enablePolygons ();
    }
  else
    stationtype = StationType_unknown;

  libsumo::TraCIColor connected;
  connected.r = 0;
  connected.g = 225;
  connected.b = 255;
  connected.a = 255;
  m_client->TraCIAPI::vehicle.setColor (m_id, connected);

  /* Set sockets, callback and station properties in DENBasicService */
  m_denService.setSocketTx (m_socket);
  m_denService.setSocketRx (m_socket);
  m_denService.setStationProperties (std::stol (m_id.substr (3)), (long) stationtype);
  m_denService.addDENRxCallbackExtended (std::bind (&emergencyVehicleAlert::receiveDENMExtended, this,
                                                    std::placeholders::_1, std::placeholders::_2,
                                                    std::placeholders::_3, std::placeholders::_4,
                                                    std::placeholders::_5));
  m_denService.setRealTime (m_real_time);

  /* Set sockets, callback, station properties and TraCI VDP in CABasicService */
  m_caService.setSocketTx (m_socket);
  m_caService.setSocketRx (m_socket);
  m_caService.setStationProperties (std::stol (m_id.substr (3)), (long) stationtype);
  m_caService.addCARxCallbackExtended (std::bind (&emergencyVehicleAlert::receiveCAMExtended, this,
                                                  std::placeholders::_1, std::placeholders::_2,
                                                  std::placeholders::_3, std::placeholders::_4,
                                                  std::placeholders::_5));
  m_caService.setRealTime (m_real_time);

  /* Set sockets, callback, station properties and TraCI VDP in CPBasicService */
  m_cpService.setSocketTx (m_socket);
  m_cpService.setSocketRx (m_socket);
  m_cpService.setStationProperties (std::stol (m_id.substr (3)), (long) stationtype);
  m_cpService.addCPRxCallback (std::bind (&emergencyVehicleAlert::receiveCPM, this,
                                          std::placeholders::_1, std::placeholders::_2));
  m_cpService.setRealTime (m_real_time);
  m_cpService.setTraCIclient (m_client);

  /* Set TraCI VDP for GeoNet object */
  m_caService.setVDP (traci_vdp);
  m_denService.setVDP (traci_vdp);
  m_cpService.setVDP (traci_vdp);

  /* Schedule CAM dissemination */
  if (m_send_cam == true)
    {
      Ptr<UniformRandomVariable> desync_rvar = CreateObject<UniformRandomVariable> ();
      desync_rvar->SetAttribute ("Min", DoubleValue (0.0));
      desync_rvar->SetAttribute ("Max", DoubleValue (1.0));
      double desync = desync_rvar->GetValue ();

      m_caService.startCamDissemination (desync);
    }

  /* Schedule CPM dissemination */
  if (m_send_cpm == true)
    {
      m_cpService.startCpmDissemination ();
    }

  if (!m_csv_name.empty ())
    {
      m_csv_ofstream_cam.open (m_csv_name + "-" + m_id + "-CAM.csv", std::ofstream::trunc);
      m_csv_ofstream_cam
          << "messageId,camId,timestamp,latitude,longitude,heading,speed,acceleration" << std::endl;
    }

  /* Open unified message log CSV */
  if (!m_csv_name.empty ())
    {
      m_csv_ofstream_msglog.open (m_csv_name + "-" + m_id + "-MSGLOG.csv", std::ofstream::trunc);
      m_csv_ofstream_msglog
          << "timestamp,senderStationId,receiverStationId,messageType,"
          << "decoded,distance_m,harm,sinr,rsrp,lossType" << std::endl;
    }

  /* Initialize cooperative braking state and CSV */
  m_coopBraking = CooperativeBrakingState{};
  if (m_cooperative_detection_enabled && !m_csv_name.empty ())
    {
      m_csv_ofstream_coop.open (m_csv_name + "-" + m_id + "-COOP.csv", std::ofstream::trunc);
      m_csv_ofstream_coop
          << "timestamp,vehicleId,role,causeCode,senderStationId,"
          << "harm12,harm23,harmTotal,deceleration,sigma,rearDenmInSigma,"
          << "sigmaMode,fixedSigmaAttr"
          << std::endl;
    }

  /* Cooperative summary counters */
  m_coop_optimal_count = 0;
  m_coop_suboptimal_count = 0;
  m_coop_sigma_sum = 0.0;
  m_coop_decel_sum = 0.0;

  /* Schedule periodic event detection */
  m_event_check_ev = Simulator::Schedule (Seconds (m_event_check_interval),
                                           &emergencyVehicleAlert::CheckForEvents, this);

  // Always-on startup line so the user can see the app actually started
  // for this vehicle, regardless of log-level configuration.
  std::cout << "[EVA-START " << Simulator::Now ().GetSeconds () << "s] " << m_id
            << " send_denm=" << m_send_denm
            << " coop=" << m_cooperative_detection_enabled
            << " sigma_mode=" << m_sigma_mode
            << " fixed_sigma=" << m_fixed_sigma
            << " hbt=" << m_hard_brake_threshold
            << " crd=" << m_collision_risk_distance
            << " drop_thr=" << m_speed_drop_threshold
            << " stationary_thr=" << m_stationary_speed
            << std::endl;
}

void
emergencyVehicleAlert::StopApplication ()
{
  NS_LOG_FUNCTION (this);
  Simulator::Cancel (m_speed_ev);
  Simulator::Cancel (m_send_cam_ev);
  Simulator::Cancel (m_update_denm_ev);
  Simulator::Cancel (m_event_check_ev);
  Simulator::Cancel (m_coopBraking.decisionTimerEvent);

  if (m_csv_ofstream_coop.is_open ())
    m_csv_ofstream_coop.close ();
  if (m_csv_ofstream_msglog.is_open ())
    m_csv_ofstream_msglog.close ();

  uint64_t cam_sent, cpm_sent;

  if (!m_csv_name.empty ())
    {
      m_csv_ofstream_cam.close ();
    }

  cam_sent = m_caService.terminateDissemination ();
  cpm_sent = m_cpService.terminateDissemination ();
  m_denService.cleanup ();
  m_LDM->cleanup ();
  m_sensor->cleanup ();

  if (m_print_summary && !m_already_print)
    {
      std::cout << "INFO-" << m_id << ",CAM-SENT:" << cam_sent << ",CAM-RECEIVED:" << m_cam_received
                << ",CPM-SENT: " << cpm_sent << ",CPM-RECEIVED: " << m_cpm_received
                << "DENM-RECEIVED: " << m_denm_received << ",DENM-SENT: " << m_denm_sent
                << std::endl;

      if (m_cooperative_detection_enabled)
        {
          uint64_t coop_total = m_coop_optimal_count + m_coop_suboptimal_count;
          double avg_sigma = coop_total > 0 ? m_coop_sigma_sum / coop_total : 0.0;
          double avg_decel = coop_total > 0 ? m_coop_decel_sum / coop_total : 0.0;
          std::cout << "COOP-" << m_id
                    << ",MODE:" << m_sigma_mode
                    << ",FIXED-SIGMA:" << m_fixed_sigma
                    << ",MIDDLE-DECISIONS:" << coop_total
                    << ",REAR-IN-SIGMA:" << m_coop_optimal_count
                    << ",SIGMA-TIMEOUT:" << m_coop_suboptimal_count
                    << ",AVG-SIGMA:" << avg_sigma
                    << ",AVG-DECEL:" << avg_decel
                    << std::endl;
        }
      m_already_print = true;
    }
}

void
emergencyVehicleAlert::StopApplicationNow ()
{
  NS_LOG_FUNCTION (this);
  StopApplication ();
}

void
emergencyVehicleAlert::CheckForEvents ()
{
  // Read kinematic state once per tick.
  double my_speed = 0.0;
  double my_accel = 0.0;
  try
    {
      my_speed = m_client->TraCIAPI::vehicle.getSpeed (m_id);
      my_accel = m_client->TraCIAPI::vehicle.getAcceleration (m_id);
    }
  catch (...)
    {
      // SUMO may not have stepped yet; skip this tick.
      m_event_check_ev = Simulator::Schedule (Seconds (m_event_check_interval),
                                               &emergencyVehicleAlert::CheckForEvents, this);
      return;
    }

  // Push into the 1 s rolling speed window.
  m_speed_window[m_speed_window_pos] = my_speed;
  m_speed_window_pos = (m_speed_window_pos + 1) % SPEED_WINDOW_N;
  if (m_speed_window_pos == 0)
    m_speed_window_full = true;

  if (my_speed > m_was_moving_speed)
    m_has_been_moving = true;

  // One-per-second state snapshot at NS_LOG_INFO. Gated, so silent
  // unless the emergencyVehicleAlert log component is on.
  static thread_local uint32_t s_tick = 0;
  if ((++s_tick % 10) == 0)
    {
      NS_LOG_INFO ("[" << Simulator::Now ().GetSeconds () << "s] " << m_id
                   << " STATE active=" << m_is_event_active
                   << " v=" << my_speed << " a=" << my_accel
                   << " hbt=" << m_hard_brake_threshold);
    }

  if (m_send_denm && !m_is_event_active)
    {
      long cause = -1, subcause = 0;
      const char *reason = "";

      if (DetectHardBraking ())
        {
          cause = 99;  // dangerousSituation
          subcause = 1; // emergencyElectronicBrakeEngaged
          reason = "HARD-BRAKE";
        }
      else if (DetectSpeedDrop (my_speed))
        {
          cause = 26;  // slowVehicle (loosest cause for "significant decel")
          subcause = 0;
          reason = "SPEED-DROP";
        }
      else if (DetectStationary (my_speed))
        {
          cause = 94;  // stationaryVehicle
          subcause = 0;
          reason = "STATIONARY";
        }
      else if (DetectCollisionRisk ())
        {
          cause = 97;  // collisionRisk
          subcause = 0;
          reason = "COLLISION-RISK";
        }

      if (cause >= 0)
        {
          std::cout << "[EVA-FIRE " << Simulator::Now ().GetSeconds () << "s] " << m_id
                    << " " << reason << " cause=" << cause << "/" << subcause
                    << " v=" << my_speed << " a=" << my_accel << std::endl;
          TriggerDenm (cause, subcause);
          m_is_event_active = true;
        }
    }
  else if (m_send_denm && m_is_event_active)
    {
      // Termination: vehicle no longer in a hard-brake AND no collision risk
      // AND not stationary (or stationary already reported and we've moved
      // again). Be conservative — better to leave a DENM active than to
      // flap.
      bool not_braking = my_accel > m_hard_brake_threshold;
      bool not_at_risk = !DetectCollisionRisk ();
      bool not_stopped = my_speed > m_stationary_speed * 2.0;
      if (not_braking && not_at_risk && not_stopped)
        {
          TerminateDenm ();
          m_is_event_active = false;
          m_stationary_already_reported = false;
          NS_LOG_INFO ("[" << Simulator::Now ().GetSeconds () << "] Vehicle " << m_id
                           << " event ended, DENM terminated");
        }
    }

  // Clean up expired forwarding entries
  CleanupForwardingTable ();

  // Reschedule
  m_event_check_ev = Simulator::Schedule (Seconds (m_event_check_interval),
                                           &emergencyVehicleAlert::CheckForEvents, this);
}

bool
emergencyVehicleAlert::DetectSpeedDrop (double current_speed)
{
  if (!m_speed_window_full)
    return false;
  // Speed one second ago is at the slot we're about to overwrite next,
  // i.e. m_speed_window_pos. (Window is 10 entries at 100 ms each.)
  double one_sec_ago = m_speed_window[m_speed_window_pos];
  if (one_sec_ago - current_speed >= m_speed_drop_threshold)
    return true;
  return false;
}

bool
emergencyVehicleAlert::DetectStationary (double current_speed)
{
  if (m_stationary_already_reported)
    return false;
  if (!m_has_been_moving)
    return false;
  if (current_speed >= m_stationary_speed)
    return false;
  m_stationary_already_reported = true;
  return true;
}

bool
emergencyVehicleAlert::DetectHardBraking ()
{
  double accel = m_client->TraCIAPI::vehicle.getAcceleration (m_id);
  return accel < m_hard_brake_threshold;
}

bool
emergencyVehicleAlert::DetectCollisionRisk ()
{
  // Use TraCI getLeader to get leading vehicle info
  auto leader = m_client->TraCIAPI::vehicle.getLeader (m_id, 100.0);
  if (leader.first.empty ())
    return false;

  double gap = leader.second; // gap in meters
  if (gap < 0)
    return false;

  double my_speed = m_client->TraCIAPI::vehicle.getSpeed (m_id);

  // Get leader speed from neighbor table or TraCI
  double leader_speed = 0.0;
  // Try to get leader's station ID from neighbor table
  for (const auto &neighbor : m_neighborTable)
    {
      // Simple heuristic: match by proximity if we can
      // The leader ID from TraCI is a SUMO ID, not a stationId
      // So we just use the gap and closing speed
    }

  // Use TraCI to get leader speed directly
  try
    {
      leader_speed = m_client->TraCIAPI::vehicle.getSpeed (leader.first);
    }
  catch (...)
    {
      return false;
    }

  double closing_speed = my_speed - leader_speed;

  // Collision risk: gap is small AND closing speed is positive (approaching)
  if (gap < m_collision_risk_distance && closing_speed > 0.1)
    {
      // Time to collision estimate
      double ttc = gap / closing_speed;
      if (ttc < 10.0) // Less than 10 seconds to collision
        return true;
    }

  return false;
}

void
emergencyVehicleAlert::CleanupForwardingTable ()
{
  uint64_t now_us = Simulator::Now ().GetMicroSeconds ();
  auto it = m_forwardingTable.begin ();
  while (it != m_forwardingTable.end ())
    {
      // Remove entries older than 30 seconds (default validity)
      if (now_us - it->second.receiveTime_us > 30000000)
        {
          it = m_forwardingTable.erase (it);
        }
      else
        {
          ++it;
        }
    }
}

void
emergencyVehicleAlert::receiveCAM (asn1cpp::Seq<CAM> cam, Address from)
{
  m_cam_received++;

  // Extract sender info for neighbor table
  unsigned long sender_id = (unsigned long) asn1cpp::getField (cam->header.stationId, long);
  NeighborState neighbor;
  neighbor.stationId = sender_id;
  neighbor.latitude =
      asn1cpp::getField (cam->cam.camParameters.basicContainer.referencePosition.latitude, double) /
      DOT_ONE_MICRO;
  neighbor.longitude =
      asn1cpp::getField (cam->cam.camParameters.basicContainer.referencePosition.longitude,
                         double) /
      DOT_ONE_MICRO;
  neighbor.speed =
      asn1cpp::getField (cam->cam.camParameters.highFrequencyContainer.choice
                             .basicVehicleContainerHighFrequency.speed.speedValue,
                         double) /
      CENTI;
  neighbor.heading =
      asn1cpp::getField (cam->cam.camParameters.highFrequencyContainer.choice
                             .basicVehicleContainerHighFrequency.heading.headingValue,
                         double) /
      DECI;
  neighbor.stationType =
      asn1cpp::getField (cam->cam.camParameters.basicContainer.stationType, long);
  neighbor.lastUpdate = Simulator::Now ().GetMicroSeconds ();

  m_neighborTable[sender_id] = neighbor;

  /* If the CAM is received from an emergency vehicle, and the host vehicle is a "passenger" car, then process the CAM */
  if (neighbor.stationType == StationType_specialVehicle && m_type != "emergency")
    {
      libsumo::TraCIPosition pos = m_client->TraCIAPI::vehicle.getPosition (m_id);
      pos = m_client->TraCIAPI::simulation.convertXYtoLonLat (pos.x, pos.y);

      if (appUtil_haversineDist (pos.y, pos.x, neighbor.latitude, neighbor.longitude) <
              m_distance_threshold &&
          appUtil_angDiff (m_client->TraCIAPI::vehicle.getAngle (m_id), neighbor.heading) <
              m_heading_threshold)
        {
          if (m_client->TraCIAPI::vehicle.getLaneIndex (m_id) == 0)
            {
              m_client->TraCIAPI::vehicle.changeLane (m_id, 0, 3);
              m_client->TraCIAPI::vehicle.setMaxSpeed (m_id, m_max_speed * 0.5);
              libsumo::TraCIColor orange;
              orange.r = 232;
              orange.g = 126;
              orange.b = 4;
              orange.a = 255;
              m_client->TraCIAPI::vehicle.setColor (m_id, orange);

              Simulator::Remove (m_speed_ev);
              m_speed_ev =
                  Simulator::Schedule (Seconds (3.0), &emergencyVehicleAlert::SetMaxSpeed, this);
            }
          else
            {
              m_client->TraCIAPI::vehicle.changeLane (m_id, 0, 3);
              m_client->TraCIAPI::vehicle.setMaxSpeed (m_id, m_max_speed * 1.5);
              libsumo::TraCIColor green;
              green.r = 0;
              green.g = 128;
              green.b = 80;
              green.a = 255;
              m_client->TraCIAPI::vehicle.setColor (m_id, green);

              Simulator::Remove (m_speed_ev);
              m_speed_ev =
                  Simulator::Schedule (Seconds (3.0), &emergencyVehicleAlert::SetMaxSpeed, this);
            }
        }
    }

  if (!m_csv_name.empty ())
    {
      // messageId,camId,timestamp,latitude,longitude,heading,speed,acceleration
      m_csv_ofstream_cam << cam->header.messageId << "," << cam->header.stationId << ",";
      m_csv_ofstream_cam << cam->cam.generationDeltaTime << ","
                         << asn1cpp::getField (
                                cam->cam.camParameters.basicContainer.referencePosition.latitude,
                                double) /
                                DOT_ONE_MICRO
                         << ",";
      m_csv_ofstream_cam << asn1cpp::getField (
                                cam->cam.camParameters.basicContainer.referencePosition.longitude,
                                double) /
                                DOT_ONE_MICRO
                         << ",";
      m_csv_ofstream_cam
          << asn1cpp::getField (cam->cam.camParameters.highFrequencyContainer.choice
                                    .basicVehicleContainerHighFrequency.heading.headingValue,
                                double) /
                 DECI
          << ","
          << asn1cpp::getField (cam->cam.camParameters.highFrequencyContainer.choice
                                    .basicVehicleContainerHighFrequency.speed.speedValue,
                                double) /
                 CENTI
          << ",";
      m_csv_ofstream_cam << asn1cpp::getField (cam->cam.camParameters.highFrequencyContainer.choice
                                                   .basicVehicleContainerHighFrequency
                                                   .longitudinalAcceleration.value,
                                               double) /
                                DECI
                         << std::endl;
    }
}

/* =====================================================================
 *  Extended CAM callback — delegates to receiveCAM(), then logs to MSGLOG
 * ===================================================================== */
void
emergencyVehicleAlert::receiveCAMExtended (asn1cpp::Seq<CAM> cam, Address from,
                                           StationId_t rxStationId,
                                           StationType_t rxStationType,
                                           SignalInfo sigInfo)
{
  // Delegate all existing CAM processing logic
  receiveCAM (cam, from);

  // Unified message logging
  if (m_csv_name.empty ())
    return;

  unsigned long sender_id = (unsigned long) asn1cpp::getField (cam->header.stationId, long);

  // Get receiver position
  libsumo::TraCIPosition my_pos = m_client->TraCIAPI::vehicle.getPosition (m_id);
  my_pos = m_client->TraCIAPI::simulation.convertXYtoLonLat (my_pos.x, my_pos.y);

  // Get sender position from neighbor table (just updated by receiveCAM)
  double sender_lat = 0.0, sender_lon = 0.0, sender_speed = 0.0;
  auto nb_it = m_neighborTable.find (sender_id);
  if (nb_it != m_neighborTable.end ())
    {
      sender_lat = nb_it->second.latitude;
      sender_lon = nb_it->second.longitude;
      sender_speed = nb_it->second.speed;
    }

  double distance = appUtil_haversineDist (my_pos.y, my_pos.x, sender_lat, sender_lon);

  // Receiver speed from SUMO
  double receiver_speed = m_client->TraCIAPI::vehicle.getSpeed (m_id);

  // Pairwise HARM: H = m2/(m1+m2) * |v_sender - v_receiver|   (paper formula 3)
  double harm = appUtil_pairwiseHarm (m_vehicle_mass, sender_speed,
                                      m_vehicle_mass, receiver_speed);

  LogMessageToCSV (sender_id, "CAM", distance, harm, sigInfo);
}

/* =====================================================================
 *  Extended DENM callback — delegates to receiveDENM(), then logs to MSGLOG
 * ===================================================================== */
void
emergencyVehicleAlert::receiveDENMExtended (denData denm, Address from,
                                            unsigned long rxStationId,
                                            long rxStationType,
                                            SignalInfo sigInfo)
{
  // Delegate all existing DENM processing logic
  receiveDENM (denm, from);

  // Unified message logging
  if (m_csv_name.empty ())
    return;

  // Extract sender station ID
  ActionID_t action_id = denm.getDenmActionID ();
  unsigned long sender_station_id = action_id.originatingStationId;

  // Skip self-messages (consistent with receiveDENM)
  unsigned long my_station_id = std::stol (m_id.substr (3));
  if (sender_station_id == my_station_id)
    return;

  // Get receiver position
  libsumo::TraCIPosition my_pos = m_client->TraCIAPI::vehicle.getPosition (m_id);
  my_pos = m_client->TraCIAPI::simulation.convertXYtoLonLat (my_pos.x, my_pos.y);

  // Get sender/event position from DENM management container
  double event_lat = static_cast<double> (denm.getDenmMgmtLatitude ()) / DOT_ONE_MICRO;
  double event_lon = static_cast<double> (denm.getDenmMgmtLongitude ()) / DOT_ONE_MICRO;

  double distance = appUtil_haversineDist (my_pos.y, my_pos.x, event_lat, event_lon);

  // Receiver speed from SUMO
  double receiver_speed = m_client->TraCIAPI::vehicle.getSpeed (m_id);

  // Sender speed: prefer neighbor table (from CAMs), fall back to DENM eventSpeed
  double sender_speed = 0.0;
  auto nb_it = m_neighborTable.find (sender_station_id);
  if (nb_it != m_neighborTable.end ())
    {
      sender_speed = nb_it->second.speed;
    }
  else if (denm.isDenmLocationDataSet ())
    {
      DENDataItem<denData::denDataLocation> loc_data = denm.getDenmLocationData_asn_types ();
      denData::denDataLocation loc = loc_data.getData ();
      if (loc.eventSpeed.isAvailable ())
        sender_speed = static_cast<double> (loc.eventSpeed.getData ().getValue ()) / CENTI;
    }

  // Pairwise HARM: H = m2/(m1+m2) * |v_sender - v_receiver|   (paper formula 3)
  double harm = appUtil_pairwiseHarm (m_vehicle_mass, sender_speed,
                                      m_vehicle_mass, receiver_speed);

  LogMessageToCSV (sender_station_id, "DENM", distance, harm, sigInfo);
}

/* =====================================================================
 *  CalculatePairwiseHarm — paper formula (3): H = m2/(m1+m2) * |delta_v|
 * ===================================================================== */
double
emergencyVehicleAlert::CalculatePairwiseHarm (double m1, double v1, double m2, double v2)
{
  return appUtil_pairwiseHarm (m1, v1, m2, v2);
}

/* =====================================================================
 *  LogMessageToCSV — write a single row to the unified MSGLOG CSV
 * ===================================================================== */
void
emergencyVehicleAlert::LogMessageToCSV (unsigned long senderStationId,
                                        const std::string &msgType,
                                        double distance, double harm,
                                        const SignalInfo &sigInfo)
{
  if (!m_csv_ofstream_msglog.is_open ())
    return;

  unsigned long my_station_id = std::stol (m_id.substr (3));

  m_csv_ofstream_msglog << Simulator::Now ().GetSeconds () << ","
                        << senderStationId << ","
                        << my_station_id << ","
                        << msgType << ","
                        << "Successfully" << ","
                        << distance << ","
                        << harm << ",";

  // SINR
  if (std::isnan (sigInfo.sinr))
    m_csv_ofstream_msglog << "NaN";
  else
    m_csv_ofstream_msglog << sigInfo.sinr;
  m_csv_ofstream_msglog << ",";

  // RSRP
  if (std::isnan (sigInfo.rsrp))
    m_csv_ofstream_msglog << "NaN";
  else
    m_csv_ofstream_msglog << sigInfo.rsrp;

  m_csv_ofstream_msglog << ",N/A" << std::endl;
}

void
emergencyVehicleAlert::receiveCPMV1 (asn1cpp::Seq<CPMV1> cpm, Address from)
{
  /* Implement CPM strategy here */
  m_cpm_received++;
  (void) from;
  std::cout << "[" << Simulator::Now ().GetSeconds () << "] " << m_id
            << " received a new CPMv1 from vehicle "
            << asn1cpp::getField (cpm->header.stationId, long) << " with "
            << asn1cpp::getField (cpm->cpm.cpmParameters.numberOfPerceivedObjects, long)
            << " perceived objects." << std::endl;
  //For every PO inside the CPM, if any
  bool POs_ok;
  auto PObjects = asn1cpp::getSeqOpt (cpm->cpm.cpmParameters.perceivedObjectContainer,
                                      PerceivedObjectContainer, &POs_ok);
  if (POs_ok)
    {
      int PObjects_size =
          asn1cpp::sequenceof::getSize (cpm->cpm.cpmParameters.perceivedObjectContainer);
      for (int i = 0; i < PObjects_size; i++)
        {
          LDM::returnedVehicleData_t PO_data;
          auto PO_seq = asn1cpp::makeSeq (PerceivedObjectV1);
          PO_seq = asn1cpp::sequenceof::getSeq (cpm->cpm.cpmParameters.perceivedObjectContainer,
                                                PerceivedObjectV1, i);
          //If PO is already in local copy of vLDM
          if (m_LDM->lookup (asn1cpp::getField (PO_seq->objectID, long), PO_data) == LDM::LDM_OK)
            {
              //Add the new perception to the LDM
              std::vector<long> associatedCVs = PO_data.vehData.associatedCVs.getData ();
              if (std::find (associatedCVs.begin (), associatedCVs.end (),
                             asn1cpp::getField (cpm->header.stationId, long)) ==
                  associatedCVs.end ())
                associatedCVs.push_back (asn1cpp::getField (cpm->header.stationId, long));
              PO_data.vehData.associatedCVs = OptionalDataItem<std::vector<long>> (associatedCVs);
              m_LDM->insert (PO_data.vehData);
            }
          else
            {
              //Translate CPM data to LDM format
              m_LDM->insert (translateCPMV1data (cpm, i));
            }
        }
    }
}

vehicleData_t
emergencyVehicleAlert::translateCPMV1data (asn1cpp::Seq<CPMV1> cpm, int objectIndex)
{
  vehicleData_t retval;
  auto PO_seq = asn1cpp::makeSeq (PerceivedObjectV1);
  using namespace boost::geometry::strategy::transform;
  PO_seq = asn1cpp::sequenceof::getSeq (cpm->cpm.cpmParameters.perceivedObjectContainer,
                                        PerceivedObjectV1, objectIndex);
  retval.detected = true;
  retval.stationID = asn1cpp::getField (PO_seq->objectID, long);
  retval.ID = std::to_string (retval.stationID);
  retval.vehicleLength = asn1cpp::getField (PO_seq->planarObjectDimension1->value, long);
  retval.vehicleWidth = asn1cpp::getField (PO_seq->planarObjectDimension2->value, long);
  retval.heading = asn1cpp::getField (cpm->cpm.cpmParameters.stationDataContainer->choice
                                          .originatingVehicleContainer.heading.headingValue,
                                      double) /
                       10 +
                   asn1cpp::getField (PO_seq->yawAngle->value, double) / 10;
  if (retval.heading > 360.0)
    retval.heading -= 360.0;

  retval.speed_ms = (double) (asn1cpp::getField (cpm->cpm.cpmParameters.stationDataContainer->choice
                                                     .originatingVehicleContainer.speed.speedValue,
                                                 long) +
                              asn1cpp::getField (PO_seq->xSpeed.value, long)) /
                    CENTI;

  double fromLon =
      asn1cpp::getField (cpm->cpm.cpmParameters.managementContainer.referencePosition.longitude,
                         double) /
      DOT_ONE_MICRO;
  double fromLat =
      asn1cpp::getField (cpm->cpm.cpmParameters.managementContainer.referencePosition.latitude,
                         double) /
      DOT_ONE_MICRO;

  libsumo::TraCIPosition objectPosition =
      m_client->TraCIAPI::simulation.convertLonLattoXY (fromLon, fromLat);

  point_type objPoint (asn1cpp::getField (PO_seq->xDistance.value, double) / CENTI,
                       asn1cpp::getField (PO_seq->yDistance.value, double) / CENTI);
  double fromAngle = asn1cpp::getField (cpm->cpm.cpmParameters.stationDataContainer->choice
                                            .originatingVehicleContainer.heading.headingValue,
                                        double) /
                     10;
  rotate_transformer<boost::geometry::degree, double, 2, 2> rotate (fromAngle - 90);
  boost::geometry::transform (objPoint, objPoint,
                              rotate); // Transform points to the reference (x,y) axises
  objectPosition.x += boost::geometry::get<0> (objPoint);
  objectPosition.y += boost::geometry::get<1> (objPoint);

  libsumo::TraCIPosition objectPosition2 = objectPosition;
  objectPosition =
      m_client->TraCIAPI::simulation.convertXYtoLonLat (objectPosition.x, objectPosition.y);

  retval.lon = objectPosition.x;
  retval.lat = objectPosition.y;

  point_type speedPoint (asn1cpp::getField (PO_seq->xSpeed.value, double) / CENTI,
                         asn1cpp::getField (PO_seq->ySpeed.value, double) / CENTI);
  boost::geometry::transform (speedPoint, speedPoint,
                              rotate); // Transform points to the reference (x,y) axises
  retval.speed_ms = asn1cpp::getField (cpm->cpm.cpmParameters.stationDataContainer->choice
                                           .originatingVehicleContainer.speed.speedValue,
                                       double) /
                        CENTI +
                    boost::geometry::get<0> (speedPoint);

  retval.camTimestamp = asn1cpp::getField (cpm->cpm.generationDeltaTime, long);
  retval.timestamp_us = Simulator::Now ().GetMicroSeconds () -
                        (asn1cpp::getField (PO_seq->timeOfMeasurement, long) * 1000);
  retval.stationType = StationType_passengerCar;
  retval.perceivedBy.setData (asn1cpp::getField (cpm->header.stationId, long));
  retval.confidence = asn1cpp::getField (PO_seq->objectConfidence, long);
  return retval;
}

void
emergencyVehicleAlert::receiveDENM (denData denm, Address from)
{
  // Extract action ID for sender identification
  ActionID_t action_id = denm.getDenmActionID ();
  unsigned long sender_station_id = action_id.originatingStationId;
  long sequence_number = action_id.sequenceNumber;

  // Don't count or process our own DENMs (they loop back through the
  // multicast group).
  unsigned long my_station_id = std::stol (m_id.substr (3));
  if (sender_station_id == my_station_id)
    return;

  m_denm_received++;
  std::cout << "[EVA-DENM " << Simulator::Now ().GetSeconds () << "s] " << m_id
            << " RECEIVE actionId=(" << sender_station_id << "," << sequence_number
            << ")" << std::endl;

  // Extract position information (converted from 0.1 micro-degrees to degrees)
  long latitude_raw = denm.getDenmMgmtLatitude ();
  long longitude_raw = denm.getDenmMgmtLongitude ();
  double latitude_degrees = static_cast<double> (latitude_raw) / DOT_ONE_MICRO;
  double longitude_degrees = static_cast<double> (longitude_raw) / DOT_ONE_MICRO;

  // Extract timing information
  long detection_time = denm.getDenmMgmtDetectionTime ();

  // Extract cause code from situation container if available
  long cause_code = -1;
  long sub_cause_code = -1;
  if (denm.isDenmSituationDataSet ())
    {
      DENDataItem<denData::denDataSituation> situation_data =
          denm.getDenmSituationData_asn_types ();
      denData::denDataSituation situation = situation_data.getData ();
      cause_code = situation.causeCode;
      sub_cause_code = situation.subCauseCode;
    }

  NS_LOG_INFO ("[" << Simulator::Now ().GetSeconds () << "] Vehicle " << m_id
                   << " received DENM from station " << sender_station_id
                   << " (Seq: " << sequence_number << ", Cause: " << cause_code << ")");

  // Extract speed from location container
  double event_speed = 0.0;
  if (denm.isDenmLocationDataSet ())
    {
      DENDataItem<denData::denDataLocation> location_data = denm.getDenmLocationData_asn_types ();
      denData::denDataLocation location = location_data.getData ();
      if (location.eventSpeed.isAvailable ())
        {
          event_speed = static_cast<double> (location.eventSpeed.getData ().getValue ()) / CENTI;
        }
    }

  // Extract custom ethical V2X fields from alacarte container
  double max_deceleration = 0.0;
  long braking_start_time = 0;
  if (denm.isDenmAlacarteDataSet ())
    {
      DENDataItem<denData::denDataAlacarte> alacarte_data = denm.getDenmAlacarteData_asn_types ();
      denData::denDataAlacarte alacarte = alacarte_data.getData ();
      if (alacarte.maxDeceleration.isAvailable ())
        max_deceleration = alacarte.maxDeceleration.getData ();
      if (alacarte.brakingStartTime.isAvailable ())
        braking_start_time = alacarte.brakingStartTime.getData ();
    }

  // --- Cooperative Ethical Braking Algorithm ---
  // Trigger on any cause that indicates a braking-related event. The
  // ETSI alacarte ethical extensions (max_deceleration etc.) are
  // optional: when present, HandleCooperativeDenm uses them; when
  // absent, it falls back to TraCI-derived defaults for the sender's
  // deceleration and mass. Gating cooperative on
  // max_deceleration > 0 used to silently skip the algorithm whenever
  // include_ethical_alacarte was off, which is the default.
  if (m_cooperative_detection_enabled &&
      (cause_code == 99 || cause_code == 97 || cause_code == 26 ||
       cause_code == 94))
    {
      HandleCooperativeDenm (denm, sender_station_id);
    }

  // --- DENM Forwarding Logic ---
  std::pair<unsigned long, long> fw_key =
      std::make_pair (sender_station_id, sequence_number);

  auto fw_it = m_forwardingTable.find (fw_key);
  if (fw_it != m_forwardingTable.end ())
    {
      // Already have this DENM — check if detection time is newer
      if (detection_time <= fw_it->second.detectionTime)
        return; // Duplicate or older, discard
      // Update with newer data
      fw_it->second.denmData = denm;
      fw_it->second.detectionTime = detection_time;
      fw_it->second.eventLat = latitude_degrees;
      fw_it->second.eventLon = longitude_degrees;
      fw_it->second.receiveTime_us = Simulator::Now ().GetMicroSeconds ();
    }
  else
    {
      // New DENM — add to forwarding table
      ForwardingEntry entry;
      entry.denmData = denm;
      entry.actionId.originatingStationID = sender_station_id;
      entry.actionId.sequenceNumber = sequence_number;
      entry.detectionTime = detection_time;
      entry.eventLat = latitude_degrees;
      entry.eventLon = longitude_degrees;
      entry.receiveTime_us = Simulator::Now ().GetMicroSeconds ();
      entry.lastForwardTime_us = 0;
      entry.forwardCount = 0;
      m_forwardingTable[fw_key] = entry;
      fw_it = m_forwardingTable.find (fw_key);
    }

  // Check if we are within relevance area and should forward
  libsumo::TraCIPosition my_pos = m_client->TraCIAPI::vehicle.getPosition (m_id);
  my_pos = m_client->TraCIAPI::simulation.convertXYtoLonLat (my_pos.x, my_pos.y);

  double dist_to_event =
      appUtil_haversineDist (my_pos.y, my_pos.x, latitude_degrees, longitude_degrees);

  // Forward if within 500m relevance area and forward count not exceeded
  if (dist_to_event < 500.0 && fw_it->second.forwardCount < MAX_FORWARD_COUNT)
    {
      // Gate on time since the *last forward*, not since the last receive.
      // The previous code reset receiveTime_us on every receive, so this
      // condition was always 0 after the first forward and the count never
      // climbed past 1 even when MAX_FORWARD_COUNT allowed more.
      uint64_t now_us = Simulator::Now ().GetMicroSeconds ();
      if (fw_it->second.lastForwardTime_us == 0 ||
          (now_us - fw_it->second.lastForwardTime_us) >= MIN_FORWARD_INTERVAL_US)
        {
          DEN_ActionID_t fwd_action_id;
          fwd_action_id.originatingStationID = sender_station_id;
          fwd_action_id.sequenceNumber = sequence_number;

          // The forwarded packet's GBC header carries the GeoArea from
          // DENBasicService::m_geoArea. Without this re-set, the forwarder
          // would stamp the packet with its own last event area (or
          // uninitialized bytes if it has never triggered a DENM), and every
          // receiver would discard the packet at the isInsideGeoArea check.
          // Anchor the area on the originator's event location with the
          // cause-code-dependent radius used by TriggerDenm.
          GeoArea_t fwdGeoArea;
          fwdGeoArea.posLat = (long) (latitude_degrees * DOT_ONE_MICRO);
          fwdGeoArea.posLong = (long) (longitude_degrees * DOT_ONE_MICRO);
          fwdGeoArea.distA = (cause_code == 99) ? 200 : 300;
          fwdGeoArea.distB = 0;
          fwdGeoArea.angle = 0;
          fwdGeoArea.shape = CIRCULAR;
          m_denService.setGeoArea (fwdGeoArea);

          DENBasicService_error_t fwd_retval =
              m_denService.forwardDENM (denm, fwd_action_id);

          if (fwd_retval == DENM_NO_ERROR)
            {
              fw_it->second.forwardCount++;
              fw_it->second.lastForwardTime_us = now_us;
              NS_LOG_INFO ("[" << Simulator::Now ().GetSeconds () << "] Vehicle " << m_id
                               << " forwarded DENM from station " << sender_station_id
                               << " (forward #" << fw_it->second.forwardCount << ")");
            }
        }
    }
}

void
emergencyVehicleAlert::TriggerDenm (long causeCode, long subCauseCode)
{
  denData data;
  DEN_ActionID_t actionid;
  DENBasicService_error_t trigger_retval;

  NS_LOG_INFO ("[" << Simulator::Now ().GetSeconds () << "s] Vehicle " << m_id
               << " TRIGGER cause=" << causeCode << "/" << subCauseCode);

  // Get real coordinates from SUMO
  libsumo::TraCIPosition posXY = m_client->TraCIAPI::vehicle.getPosition (m_id);
  libsumo::TraCIPosition pos =
      m_client->TraCIAPI::simulation.convertXYtoLonLat (posXY.x, posXY.y);

  // Detection time: the time at which the event was originally detected.
  // Capture it once here and reuse it through UpdateDenm/TerminateDenm so the
  // DENM keeps a stable detection-time anchor across its lifecycle, per
  // ETSI EN 302 637-3.
  m_active_detection_time_ms = compute_timestampIts (m_real_time);

  // Set management container mandatory fields with real coordinates (0.1 microdegrees)
  data.setDenmMandatoryFields (m_active_detection_time_ms,
                               (long) (pos.y * DOT_ONE_MICRO),
                               (long) (pos.x * DOT_ONE_MICRO));

  // Set validity duration (30 seconds for safety events)
  data.setValidityDuration (30);

  // Repetition is driven by the application's UpdateDenm scheduler. Leaving
  // service-side repetition on (setDenmRepetition(duration, interval)) makes
  // every interval fire twice — once from T_RepetitionStop in DENBasicService,
  // once from appDENM_update inside UpdateDenm — because both rearm the same
  // timer. Explicitly disable service-side repetition here.
  data.setDenmRepetition (0, 0);

  // Set situation container
  denData::denDataSituation situation;
  situation.informationQuality = 1; // Based on vehicle sensor data
  situation.causeCode = causeCode;
  situation.subCauseCode = subCauseCode;
  data.setDenmSituationData_asn_types (situation);

  // Set location container — speed in cm/s, heading in 0.1 degrees
  double speed_ms = m_client->TraCIAPI::vehicle.getSpeed (m_id);
  long speed_cm_s = (long) (speed_ms * CENTI);
  {
    // Build full location container with event speed AND a mandatory trace.
    // LocationContainer.traces is Traces ::= SEQUENCE SIZE(1..7) OF PathHistory,
    // so at least one trace with one path point is required for valid UPER encoding.
    denData::denDataLocation location;
    DENValueConfidence<long, long> speedConf (speed_cm_s,
                                               SpeedConfidenceV1_equalOrWithinOneCentimeterPerSec);
    location.eventSpeed = DENDataItem<DENValueConfidence<long, long>> (speedConf);

    DEN_PathPoint_t originPt = {};
    originPt.pathPosition.deltaLatitude = 0;
    originPt.pathPosition.deltaLongitude = 0;
    originPt.pathPosition.deltaAltitude = 0;
    location.traces.push_back ({originPt});

    data.setDenmLocationData_asn_types (location);
  }

  // Set alacarte container
  data.setDenmAlacarteVehicleMass ((long) m_vehicle_mass);
  // ASN.1 LanePosition is the lane *index* (-1..14: offTheRoad, hardShoulder,
  // innermost driving lane, …, outerHardShoulder), NOT the longitudinal offset.
  // TraCI's getLanePosition returns metres-along-lane (was overflowing the
  // (-1..14) constraint and killing the UPER encode with failed_type=LanePosition).
  // Clamp the lane index defensively in case TraCI returns >14 (e.g. on
  // multi-lane motorway scenarios beyond ETSI's enumerated range).
  {
    long laneIdx = (long) m_client->TraCIAPI::vehicle.getLaneIndex (m_id);
    if (laneIdx < -1) laneIdx = -1;
    if (laneIdx > 14) laneIdx = 14;
    data.setDenmAlacarteLanePosition (laneIdx);
  }

  // The ethical extension fields (ethicalMaxDeceleration,
  // ethicalBrakingStartTime, ethicalVehicleMass) are *custom*
  // additions to the alacarte container — not in the original ETSI
  // EN 302 637-3 spec — added by hand to the generated ASN.1 C
  // descriptor. If the asn1c PER extension wiring is incomplete, the
  // UPER encoder rejects the whole DENM. IncludeEthicalAlacarte
  // defaults off so basic DENMs always encode; turn it on once the
  // extension is known good.
  if ((m_ethical_braking_enabled || m_cooperative_detection_enabled)
      && m_include_ethical_alacarte)
    {
      double max_decel = m_client->TraCIAPI::vehicle.getDecel (m_id);
      data.setDenmAlacarteMaxDeceleration (max_decel);
      data.setDenmAlacarteBrakingStartTime (compute_timestampIts (m_real_time));
    }

  // Set GeoArea — circular area around the event
  GeoArea_t geoArea;
  geoArea.posLong = (long) (pos.x * DOT_ONE_MICRO);
  geoArea.posLat = (long) (pos.y * DOT_ONE_MICRO);
  // Hard braking: 200m radius, collision risk: 300m radius
  geoArea.distA = (causeCode == 99) ? 200 : 300;
  geoArea.distB = 0;
  geoArea.angle = 0;
  geoArea.shape = CIRCULAR;
  m_denService.setGeoArea (geoArea);

  trigger_retval = m_denService.appDENM_trigger (data, actionid);

  if (trigger_retval != DENM_NO_ERROR)
    {
      std::cout << "[EVA-DENM " << Simulator::Now ().GetSeconds () << "s] " << m_id
                << " TRIGGER FAILED err=" << trigger_retval
                << " cause=" << causeCode << "/" << subCauseCode << std::endl;
      NS_LOG_ERROR ("Cannot trigger DENM. Error code: " << trigger_retval);
    }
  else
    {
      m_denm_sent++;
      m_active_action_id = actionid;
      std::cout << "[EVA-DENM " << Simulator::Now ().GetSeconds () << "s] " << m_id
                << " TRIGGER OK actionId=(" << actionid.originatingStationID
                << "," << actionid.sequenceNumber << ") cause=" << causeCode
                << "/" << subCauseCode << std::endl;

      try
        {
          libsumo::TraCIColor red;
          red.r = 255;
          red.g = 0;
          red.b = 0;
          red.a = 255;
          m_client->TraCIAPI::vehicle.setColor (m_id, red);

          // V1's "a1 = a_max" is enforced externally by the force_brake
          // scheduler in the NR scenario. We deliberately do NOT call
          // ApplyCooperativeBraking on the originator here — stacking a
          // second slowDown on the same vehicle in the same simulator
          // step desyncs TraCI. The middle / rear branches of
          // HandleCooperativeDenm still call ApplyCooperativeBraking on
          // *their own* vehicle, which is safe.
          m_update_denm_ev =
              Simulator::Schedule (MilliSeconds (500),
                                   &emergencyVehicleAlert::UpdateDenm, this, actionid);
        }
      catch (const std::exception &e)
        {
          NS_LOG_ERROR ("TriggerDenm post-encode exception: " << e.what ());
        }
      catch (...)
        {
          NS_LOG_ERROR ("TriggerDenm post-encode exception (unknown)");
        }
    }
}

void
emergencyVehicleAlert::UpdateDenm (DEN_ActionID actionid)
{
  denData data;
  DENBasicService_error_t trigger_retval;

  NS_LOG_INFO ("[" << Simulator::Now ().GetSeconds () << "s] Vehicle " << m_id
               << " UPDATE seq=" << actionid.sequenceNumber);

  // Get updated coordinates from SUMO
  libsumo::TraCIPosition pos = m_client->TraCIAPI::vehicle.getPosition (m_id);
  pos = m_client->TraCIAPI::simulation.convertXYtoLonLat (pos.x, pos.y);

  // Set management container mandatory fields with updated position.
  // Detection time stays anchored to the original trigger time per
  // ETSI EN 302 637-3; only position and reference time advance.
  data.setDenmMandatoryFields (actionid.originatingStationID, actionid.sequenceNumber,
                               m_active_detection_time_ms,
                               (double) (pos.y * DOT_ONE_MICRO),
                               (double) (pos.x * DOT_ONE_MICRO));

  data.setValidityDuration (30);
  // App-driven repetition: see TriggerDenm comment.
  data.setDenmRepetition (0, 0);

  // Update speed
  double speed_ms = m_client->TraCIAPI::vehicle.getSpeed (m_id);
  long speed_cm_s = (long) (speed_ms * CENTI);
  {
    denData::denDataLocation location;
    DENValueConfidence<long, long> speedConf (speed_cm_s,
                                               SpeedConfidenceV1_equalOrWithinOneCentimeterPerSec);
    location.eventSpeed = DENDataItem<DENValueConfidence<long, long>> (speedConf);

    DEN_PathPoint_t originPt = {};
    originPt.pathPosition.deltaLatitude = 0;
    originPt.pathPosition.deltaLongitude = 0;
    originPt.pathPosition.deltaAltitude = 0;
    location.traces.push_back ({originPt});

    data.setDenmLocationData_asn_types (location);
  }

  // Update alacarte
  data.setDenmAlacarteVehicleMass ((long) m_vehicle_mass);
  // ASN.1 LanePosition is the lane *index* (-1..14: offTheRoad, hardShoulder,
  // innermost driving lane, …, outerHardShoulder), NOT the longitudinal offset.
  // TraCI's getLanePosition returns metres-along-lane (was overflowing the
  // (-1..14) constraint and killing the UPER encode with failed_type=LanePosition).
  // Clamp the lane index defensively in case TraCI returns >14 (e.g. on
  // multi-lane motorway scenarios beyond ETSI's enumerated range).
  {
    long laneIdx = (long) m_client->TraCIAPI::vehicle.getLaneIndex (m_id);
    if (laneIdx < -1) laneIdx = -1;
    if (laneIdx > 14) laneIdx = 14;
    data.setDenmAlacarteLanePosition (laneIdx);
  }

  if ((m_ethical_braking_enabled || m_cooperative_detection_enabled)
      && m_include_ethical_alacarte)
    {
      double max_decel = m_client->TraCIAPI::vehicle.getDecel (m_id);
      data.setDenmAlacarteMaxDeceleration (max_decel);
    }

  // Update GeoArea
  GeoArea_t geoArea;
  geoArea.posLong = (long) (pos.x * DOT_ONE_MICRO);
  geoArea.posLat = (long) (pos.y * DOT_ONE_MICRO);
  geoArea.distA = 200;
  geoArea.distB = 0;
  geoArea.angle = 0;
  geoArea.shape = CIRCULAR;
  m_denService.setGeoArea (geoArea);

  trigger_retval = m_denService.appDENM_update (data, actionid);

  if (trigger_retval != DENM_NO_ERROR)
    {
      NS_LOG_ERROR ("Cannot update DENM. Error code: " << trigger_retval);
    }
  else
    {
      m_denm_sent++;
    }

  // Continue updating if event is still active
  if (m_is_event_active)
    {
      m_update_denm_ev =
          Simulator::Schedule (MilliSeconds (500), &emergencyVehicleAlert::UpdateDenm, this,
                               actionid);
    }
}

void
emergencyVehicleAlert::TerminateDenm ()
{
  if (!m_is_event_active)
    return;

  NS_LOG_INFO ("[" << Simulator::Now ().GetSeconds () << "s] Vehicle " << m_id
               << " TERMINATE seq=" << m_active_action_id.sequenceNumber);

  Simulator::Cancel (m_update_denm_ev);

  denData data;
  // Set mandatory fields for termination
  libsumo::TraCIPosition pos = m_client->TraCIAPI::vehicle.getPosition (m_id);
  pos = m_client->TraCIAPI::simulation.convertXYtoLonLat (pos.x, pos.y);

  data.setDenmMandatoryFields (m_active_action_id.originatingStationID,
                               m_active_action_id.sequenceNumber,
                               m_active_detection_time_ms,
                               (double) (pos.y * DOT_ONE_MICRO),
                               (double) (pos.x * DOT_ONE_MICRO));

  data.setValidityDuration (30);

  DENBasicService_error_t term_retval =
      m_denService.appDENM_termination (data, m_active_action_id);

  if (term_retval != DENM_NO_ERROR)
    {
      NS_LOG_ERROR ("Cannot terminate DENM. Error code: " << term_retval);
    }
  else
    {
      m_denm_sent++;
    }

  // Restore vehicle color
  libsumo::TraCIColor connected;
  connected.r = 0;
  connected.g = 225;
  connected.b = 255;
  connected.a = 255;
  m_client->TraCIAPI::vehicle.setColor (m_id, connected);
}

void
emergencyVehicleAlert::SetMaxSpeed ()
{
  libsumo::TraCIColor normal;
  normal.r = 0;
  normal.g = 225;
  normal.b = 255;
  normal.a = 255;
  m_client->TraCIAPI::vehicle.setColor (m_id, normal);
  m_client->TraCIAPI::vehicle.setMaxSpeed (m_id, m_max_speed);
}

void
emergencyVehicleAlert::receiveCPM (asn1cpp::Seq<CollectivePerceptionMessage> cpm, Address from)
{
  /* Implement CPM strategy here */
  m_cpm_received++;
  (void) from;
  //For every PO inside the CPM, if any
  bool POs_ok;
  //auto wrappedContainer = asn1cpp::makeSeq(WrappedCpmContainer);
  int wrappedContainer_size = asn1cpp::sequenceof::getSize (cpm->payload.cpmContainers);
  for (int i = 0; i < wrappedContainer_size; i++)
    {
      auto wrappedContainer =
          asn1cpp::sequenceof::getSeq (cpm->payload.cpmContainers, WrappedCpmContainer, i);
      WrappedCpmContainer__containerData_PR present = asn1cpp::getField (
          wrappedContainer->containerData.present, WrappedCpmContainer__containerData_PR);
      if (present == WrappedCpmContainer__containerData_PR_PerceivedObjectContainer)
        {
          auto POcontainer =
              asn1cpp::getSeq (wrappedContainer->containerData.choice.PerceivedObjectContainer,
                               PerceivedObjectContainer);
          int PObjects_size = asn1cpp::sequenceof::getSize (POcontainer->perceivedObjects);
          std::cout << "[" << Simulator::Now ().GetSeconds () << "] " << m_id
                    << " received a new CPMv2 from "
                    << asn1cpp::getField (cpm->header.stationId, long) << " with " << PObjects_size
                    << " perceived objects." << std::endl;
          for (int j = 0; j < PObjects_size; j++)
            {
              LDM::returnedVehicleData_t PO_data;
              auto PO_seq = asn1cpp::makeSeq (PerceivedObject);
              PO_seq =
                  asn1cpp::sequenceof::getSeq (POcontainer->perceivedObjects, PerceivedObject, j);
              //If PO is already in local copy of vLDM
              if (m_LDM->lookup (asn1cpp::getField (PO_seq->objectId, long), PO_data) ==
                  LDM::LDM_OK)
                {
                  //Add the new perception to the LDM
                  std::vector<long> associatedCVs = PO_data.vehData.associatedCVs.getData ();
                  if (std::find (associatedCVs.begin (), associatedCVs.end (),
                                 asn1cpp::getField (cpm->header.stationId, long)) ==
                      associatedCVs.end ())
                    associatedCVs.push_back (asn1cpp::getField (cpm->header.stationId, long));
                  PO_data.vehData.associatedCVs =
                      OptionalDataItem<std::vector<long>> (associatedCVs);
                  m_LDM->insert (PO_data.vehData);
                }
              else
                {
                  //Translate CPM data to LDM format
                  m_LDM->insert (translateCPMdata (cpm, PO_seq, j));
                }
            }
        }
    }
}

vehicleData_t
emergencyVehicleAlert::translateCPMdata (asn1cpp::Seq<CollectivePerceptionMessage> cpm,
                                         asn1cpp::Seq<PerceivedObject> object, int objectIndex)
{
  vehicleData_t retval;
  retval.detected = true;
  retval.stationID = asn1cpp::getField (object->objectId, long);
  retval.ID = std::to_string (retval.stationID);
  retval.vehicleLength = asn1cpp::getField (object->objectDimensionX->value, long);
  retval.vehicleWidth = asn1cpp::getField (object->objectDimensionY->value, long);
  retval.heading = asn1cpp::getField (object->angles->zAngle.value, double) / DECI;
  retval.xSpeedAbs.setData (
      asn1cpp::getField (object->velocity->choice.cartesianVelocity.xVelocity.value, long));
  retval.xSpeedAbs.setData (
      asn1cpp::getField (object->velocity->choice.cartesianVelocity.yVelocity.value, long));
  retval.speed_ms =
      (sqrt (pow (retval.xSpeedAbs.getData (), 2) + pow (retval.ySpeedAbs.getData (), 2))) / CENTI;

  libsumo::TraCIPosition fromPosition = m_client->TraCIAPI::simulation.convertLonLattoXY (
      asn1cpp::getField (cpm->payload.managementContainer.referencePosition.longitude, double) /
          DOT_ONE_MICRO,
      asn1cpp::getField (cpm->payload.managementContainer.referencePosition.latitude, double) /
          DOT_ONE_MICRO);
  libsumo::TraCIPosition objectPosition = fromPosition;
  objectPosition.x += asn1cpp::getField (object->position.xCoordinate.value, long) / CENTI;
  objectPosition.y += asn1cpp::getField (object->position.yCoordinate.value, long) / CENTI;
  objectPosition =
      m_client->TraCIAPI::simulation.convertXYtoLonLat (objectPosition.x, objectPosition.y);
  retval.lon = objectPosition.x;
  retval.lat = objectPosition.y;

  retval.camTimestamp = asn1cpp::getField (cpm->payload.managementContainer.referenceTime, long);
  retval.timestamp_us = Simulator::Now ().GetMicroSeconds () -
                        (asn1cpp::getField (object->measurementDeltaTime, long) * 1000);
  retval.stationType = StationType_passengerCar;
  retval.perceivedBy.setData (asn1cpp::getField (cpm->header.stationId, long));

  return retval;
}

/* =====================================================================
 *  Cooperative Ethical Braking Algorithm
 *  Based on: "Ethical 5G NR-V2X sidelink communication protocol"
 *            (Rolich et al., VTC 2024)
 * ===================================================================== */

double
emergencyVehicleAlert::CalculateHarm (double m_follower, double v_follower, double a_follower,
                                      double m_ahead, double v_ahead, double a_ahead, double gap)
{
  if (gap < 0)
    gap = 0;
  if (v_follower <= v_ahead && a_follower >= a_ahead)
    return 0.0; // follower is slower and braking harder — no collision

  // Handle gap=0 instant collision: vehicles already at same position
  // and follower is faster → collision at t=0
  if (gap <= 0 && v_follower > v_ahead)
    {
      double v_rel = v_follower - v_ahead;
      double total_mass = m_follower + m_ahead;
      if (total_mass <= 0)
        return 0.0;
      double m_reduced = (m_follower * m_ahead) / total_mass;
      return 0.5 * m_reduced * v_rel * v_rel;
    }

  // Time each vehicle needs to stop
  double t_stop_ahead = (a_ahead > 0) ? v_ahead / a_ahead : 1e9;
  double t_stop_follower = (a_follower > 0) ? v_follower / a_follower : 1e9;

  // Distance each vehicle travels until stop (using v^2/(2a) = 0.5*v*t_stop)
  double d_ahead_stop = (a_ahead > 0) ? v_ahead * v_ahead / (2.0 * a_ahead) : 1e18;
  double d_follower_stop = (a_follower > 0) ? v_follower * v_follower / (2.0 * a_follower) : 1e18;

  // If follower stops before closing the gap — no collision
  if (d_follower_stop <= gap + d_ahead_stop)
    return 0.0;

  // Find collision time by solving piecewise kinematics
  // Phase 1: both moving, t in [0, min(t_stop_ahead, t_stop_follower)]
  double t_phase1_end = std::min (t_stop_ahead, t_stop_follower);

  // gap(t) = gap + (v_ahead - v_follower)*t + 0.5*(a_follower - a_ahead)*t^2
  // Solve gap(t) = 0: A*t^2 + B*t + C = 0
  double A = 0.5 * (a_follower - a_ahead);
  double B = v_ahead - v_follower;
  double C = gap;

  double t_collision = -1;

  if (std::abs (A) > 1e-9)
    {
      double discriminant = B * B - 4 * A * C;
      if (discriminant >= 0)
        {
          double sqrt_disc = std::sqrt (discriminant);
          double t1 = (-B - sqrt_disc) / (2 * A);
          double t2 = (-B + sqrt_disc) / (2 * A);
          if (t1 > 1e-12 && t1 <= t_phase1_end)
            t_collision = t1;
          else if (t2 > 1e-12 && t2 <= t_phase1_end)
            t_collision = t2;
        }
    }
  else if (std::abs (B) > 1e-9)
    {
      double t = -C / B;
      if (t > 1e-12 && t <= t_phase1_end)
        t_collision = t;
    }

  // Phase 2: ahead stopped, follower still moving
  if (t_collision < 0 && t_stop_ahead < t_stop_follower)
    {
      double gap_at_phase2_start = gap + v_ahead * t_stop_ahead - 0.5 * a_ahead * t_stop_ahead * t_stop_ahead
                                   - (v_follower * t_stop_ahead - 0.5 * a_follower * t_stop_ahead * t_stop_ahead);
      double v_follower_phase2 = v_follower - a_follower * t_stop_ahead;

      if (v_follower_phase2 > 0 && gap_at_phase2_start > 0)
        {
          // gap(dt) = gap_at_phase2_start - v_follower_phase2 * dt + 0.5 * a_follower * dt^2
          // Solve: A2*dt^2 + B2*dt + C2 = 0
          double A2 = 0.5 * a_follower;
          double B2 = -v_follower_phase2;
          double C2 = gap_at_phase2_start;
          double dt_max = t_stop_follower - t_stop_ahead;

          if (std::abs (A2) > 1e-9)
            {
              double disc2 = B2 * B2 - 4 * A2 * C2;
              if (disc2 >= 0)
                {
                  double sqrt_disc2 = std::sqrt (disc2);
                  double dt1 = (-B2 - sqrt_disc2) / (2 * A2);
                  double dt2 = (-B2 + sqrt_disc2) / (2 * A2);
                  if (dt1 > 1e-12 && dt1 <= dt_max)
                    t_collision = t_stop_ahead + dt1;
                  else if (dt2 > 1e-12 && dt2 <= dt_max)
                    t_collision = t_stop_ahead + dt2;
                }
            }
          else if (std::abs (B2) > 1e-9)
            {
              // Linear case: a_follower ≈ 0, follower coasts at constant speed
              double dt = -C2 / B2;
              if (dt > 1e-12 && dt <= dt_max)
                t_collision = t_stop_ahead + dt;
            }
        }
    }

  if (t_collision < 0)
    return 0.0; // No collision

  // Compute relative velocity at collision
  double v_f_at_tc = std::max (0.0, v_follower - a_follower * t_collision);
  double v_a_at_tc = std::max (0.0, v_ahead - a_ahead * t_collision);
  double v_rel = v_f_at_tc - v_a_at_tc;

  if (v_rel <= 0)
    return 0.0;

  double total_mass = m_follower + m_ahead;
  if (total_mass <= 0)
    return 0.0;
  double m_reduced = (m_follower * m_ahead) / total_mass;
  return 0.5 * m_reduced * v_rel * v_rel;
}

double
emergencyVehicleAlert::CalculateDecisionBudget (double v_self, double a_max_self,
                                                double v_ahead, double a_ahead, double gap)
{
  // During sigma: self at constant v_self, ahead braking at a_ahead
  // After sigma: self brakes at a_max_self
  // Requirement: self stops before reaching ahead's final position
  //
  // ahead's stopping distance from now: d_ahead = v_ahead^2 / (2*a_ahead)
  // During sigma, gap changes: gap(sigma) = gap + (v_ahead - v_self)*sigma - 0.5*a_ahead*sigma^2
  //   (ahead decelerates, self at constant speed)
  // After sigma, self needs: d_self = v_self^2 / (2*a_max_self)
  // ahead still travels: d_ahead_remaining = depends on phase
  //
  // Simplified: require gap(sigma) >= d_self - d_ahead_remaining
  // For H_{self,ahead} = 0: self must stop before hitting ahead

  if (a_max_self <= 0 || v_self <= 0)
    return 0.0;

  double d_self_brake = v_self * v_self / (2.0 * a_max_self);

  // ahead's remaining stopping distance as function of sigma
  // v_ahead(sigma) = max(0, v_ahead - a_ahead*sigma)
  // t_stop_ahead = v_ahead / a_ahead

  double t_stop_ahead = (a_ahead > 0) ? v_ahead / a_ahead : 0;

  // Binary search for max sigma where no collision occurs
  double sigma_lo = 0.0;
  double sigma_hi = 5.0; // max 5 seconds
  double sigma = 0.0;

  for (int i = 0; i < 50; i++)
    {
      double sigma_mid = (sigma_lo + sigma_hi) / 2.0;

      // Gap at sigma_mid (ahead braking, self at constant speed)
      double t_eff = std::min (sigma_mid, t_stop_ahead);
      double d_ahead_during_sigma = v_ahead * t_eff - 0.5 * a_ahead * t_eff * t_eff;
      double d_self_during_sigma = v_self * sigma_mid;
      double gap_at_sigma = gap + d_ahead_during_sigma - d_self_during_sigma;

      // Ahead's remaining stopping distance after sigma
      double v_ahead_at_sigma = std::max (0.0, v_ahead - a_ahead * sigma_mid);
      double d_ahead_remaining = v_ahead_at_sigma * v_ahead_at_sigma / (2.0 * a_ahead + 1e-9);

      // Self needs d_self_brake to stop; ahead still travels d_ahead_remaining
      double margin = gap_at_sigma + d_ahead_remaining - d_self_brake;

      if (margin >= 0)
        {
          sigma = sigma_mid;
          sigma_lo = sigma_mid;
        }
      else
        {
          sigma_hi = sigma_mid;
        }
    }

  return std::max (0.0, sigma);
}

double
emergencyVehicleAlert::CalculateOptimalDeceleration (double v1, double a1, double m1,
                                                     double v2, double m2, double a2_max,
                                                     double v3, double a3, double m3,
                                                     double gap12, double gap23)
{
  double best_a2 = a2_max;
  double min_harm = std::numeric_limits<double>::max ();

  // Sweep a2 from 0.1 to a2_max in 0.1 m/s^2 steps
  for (double a2 = 0.1; a2 <= a2_max + 0.05; a2 += 0.1)
    {
      double clamped_a2 = std::min (a2, a2_max);
      // H_{1,2}: vehicle 2 follows vehicle 1
      double h12 = CalculateHarm (m2, v2, clamped_a2, m1, v1, a1, gap12);
      // H_{2,3}: vehicle 3 follows vehicle 2
      double h23 = CalculateHarm (m3, v3, a3, m2, v2, clamped_a2, gap23);
      double h_total = h12 + h23;

      if (h_total < min_harm)
        {
          min_harm = h_total;
          best_a2 = clamped_a2;
        }
    }

  return best_a2;
}

void
emergencyVehicleAlert::HandleCooperativeDenm (denData &denm, unsigned long senderStationId)
{
  NS_LOG_INFO ("[" << Simulator::Now ().GetSeconds () << "s] " << m_id
               << " HandleCooperativeDenm sender=" << senderStationId);

  // Don't re-enter if already actively processing
  if (m_coopBraking.active && m_coopBraking.decisionMade)
    return;

  try
    {

  unsigned long my_station_id = std::stol (m_id.substr (3));

  // Extract originator from DENM action ID
  ActionID_t action_id = denm.getDenmActionID ();
  unsigned long originator_id = action_id.originatingStationId;

  // Get own kinematic state from SUMO
  double my_speed = m_client->TraCIAPI::vehicle.getSpeed (m_id);
  double my_max_decel = m_client->TraCIAPI::vehicle.getDecel (m_id);
  double my_heading = m_client->TraCIAPI::vehicle.getAngle (m_id);

  // Extract sender's data from DENM alacarte. The ethical-extension
  // fields (maxDeceleration, vehicleMass) are not part of the
  // baseline ETSI alacarte and are gated behind include_ethical_alacarte
  // on the sender; when off, we use defensible defaults so the algo
  // still runs.
  double sender_max_decel = 0.0;     // 0 ⇒ "unknown"; replaced below
  double sender_mass = m_vehicle_mass;
  if (denm.isDenmAlacarteDataSet ())
    {
      auto alacarte = denm.getDenmAlacarteData_asn_types ().getData ();
      if (alacarte.maxDeceleration.isAvailable ())
        sender_max_decel = alacarte.maxDeceleration.getData ();
      if (alacarte.vehicleMass.isAvailable ())
        sender_mass = static_cast<double> (alacarte.vehicleMass.getData ());
    }
  if (sender_max_decel <= 0.0)
    {
      // No ethical alacarte: assume sender's max deceleration equals our
      // own. Reasonable for a fleet of similar vehicles; for mixed
      // fleets the sender would carry its mass+decel in the alacarte.
      sender_max_decel = my_max_decel;
    }

  // Get sender speed from DENM location container
  double sender_speed = 0.0;
  if (denm.isDenmLocationDataSet ())
    {
      auto loc = denm.getDenmLocationData_asn_types ().getData ();
      if (loc.eventSpeed.isAvailable ())
        sender_speed = static_cast<double> (loc.eventSpeed.getData ().getValue ()) / CENTI;
    }

  // Get leader (vehicle ahead) info from TraCI
  auto leader = m_client->TraCIAPI::vehicle.getLeader (m_id, 200.0);
  bool has_leader = !leader.first.empty () && leader.second >= 0;
  double gap_to_leader = has_leader ? leader.second : 0;
  double leader_speed = 0;
  double leader_max_decel = 7.5; // default
  unsigned long leader_station_id = 0;

  if (has_leader)
    {
      try
        {
          leader_speed = m_client->TraCIAPI::vehicle.getSpeed (leader.first);
          leader_max_decel = m_client->TraCIAPI::vehicle.getDecel (leader.first);
          leader_station_id = std::stol (leader.first.substr (3));
        }
      catch (...)
        {
          has_leader = false;
        }
    }

  // Find follower: closest neighbor behind me with similar heading
  bool has_follower = false;
  double gap_to_follower = 0;
  double follower_speed = 0;
  double follower_max_decel = 7.5;
  double follower_mass = 1500.0;
  unsigned long follower_station_id = 0;

  libsumo::TraCIPosition my_pos = m_client->TraCIAPI::vehicle.getPosition (m_id);
  my_pos = m_client->TraCIAPI::simulation.convertXYtoLonLat (my_pos.x, my_pos.y);

  double min_behind_dist = std::numeric_limits<double>::max ();
  for (const auto &neighbor : m_neighborTable)
    {
      // Skip if heading differs by more than 30 degrees
      if (appUtil_angDiff (my_heading, neighbor.second.heading) > 30.0)
        continue;

      double dist = appUtil_haversineDist (my_pos.y, my_pos.x,
                                           neighbor.second.latitude, neighbor.second.longitude);

      // Determine if neighbor is behind: compute bearing from me to neighbor
      double dLon = neighbor.second.longitude - my_pos.x;
      double dLat = neighbor.second.latitude - my_pos.y;
      double bearing = std::atan2 (dLon, dLat) * 180.0 / M_PI;
      if (bearing < 0)
        bearing += 360.0;

      // "Behind" means bearing is roughly opposite to my heading
      double heading_to_neighbor_diff = appUtil_angDiff (bearing, my_heading);
      if (heading_to_neighbor_diff > 90.0 && dist < min_behind_dist && dist < 200.0)
        {
          min_behind_dist = dist;
          has_follower = true;
          gap_to_follower = dist;
          follower_speed = neighbor.second.speed;
          follower_station_id = neighbor.second.stationId;
        }
    }

  // Extract cause code for logging
  long cause_code = -1;
  if (denm.isDenmSituationDataSet ())
    {
      auto sit = denm.getDenmSituationData_asn_types ().getData ();
      cause_code = sit.causeCode;
    }

  // Determine role based on position relative to the braking originator
  bool originator_is_leader = (has_leader && leader_station_id == originator_id);
  bool sender_is_behind = (senderStationId == follower_station_id);

  // === CASE: I'm already waiting for rear DENM and this is the rear DENM ===
  if (m_coopBraking.active && !m_coopBraking.decisionMade && sender_is_behind)
    {
      m_coopBraking.rearDenmReceived = true;
      Simulator::Cancel (m_coopBraking.decisionTimerEvent);

      // Recalculate with updated rear vehicle info
      double v1 = leader_speed;
      double a1 = sender_max_decel > 0 ? sender_max_decel : leader_max_decel;
      double m1_mass = sender_mass;
      double v3 = follower_speed;
      double a3 = follower_max_decel;
      double m3_mass = follower_mass;

      // Use originator's max decel if available from the DENM we received earlier
      if (originator_is_leader && sender_max_decel > 0)
        a1 = sender_max_decel;

      double optimal_a2 = CalculateOptimalDeceleration (
          v1, a1, m1_mass, my_speed, m_vehicle_mass, my_max_decel,
          v3, a3, m3_mass, gap_to_leader, gap_to_follower);

      double h12 = CalculateHarm (m_vehicle_mass, my_speed, optimal_a2,
                                  m1_mass, v1, a1, gap_to_leader);
      double h23 = CalculateHarm (m3_mass, v3, a3,
                                  m_vehicle_mass, my_speed, optimal_a2, gap_to_follower);

      NS_LOG_INFO ("[" << Simulator::Now ().GetSeconds () << "] Vehicle " << m_id
                       << " COOPERATIVE: rear DENM received within sigma, optimal a2*="
                       << optimal_a2 << " m/s^2, H12=" << h12 << " H23=" << h23
                       << " Htotal=" << h12 + h23);

      LogCooperativeDecision ("middle", cause_code, senderStationId,
                              h12, h23, h12 + h23, optimal_a2, m_coopBraking.sigma, true);

      ApplyCooperativeBraking (optimal_a2, true);
      return;
    }

  // === CASE: Middle vehicle — originator is ahead, I have a follower ===
  if (originator_is_leader && has_follower && !m_coopBraking.active)
    {
      m_coopBraking.active = true;
      m_coopBraking.originatorStationId = originator_id;
      m_coopBraking.decisionMade = false;
      m_coopBraking.rearDenmReceived = false;

      // Snapshot leader / follower identity so the σ-timeout branch can
      // re-run the optimization with up-to-date neighbor-table CAM data
      // instead of using the closed-form suboptimal value.
      m_coopBraking.followerStationId = follower_station_id;
      m_coopBraking.leaderStationId = leader_station_id;
      m_coopBraking.leaderMass = sender_mass;

      double a_ahead = sender_max_decel > 0 ? sender_max_decel : leader_max_decel;
      m_coopBraking.leaderAheadDecel = a_ahead;

      // Decision time budget σ. Default is the closed-form computed value
      // (sweep-friendly override via attributes for parameter studies).
      double sigma_computed =
          CalculateDecisionBudget (my_speed, my_max_decel, leader_speed, a_ahead, gap_to_leader);
      double sigma;
      if (m_sigma_mode == "fixed")
        sigma = std::max (0.0, m_fixed_sigma);
      else if (m_sigma_mode == "scaled")
        sigma = std::max (0.0, sigma_computed * m_fixed_sigma);
      else
        sigma = sigma_computed;
      m_coopBraking.sigma = sigma;

      // Calculate suboptimal deceleration (guarantees H_{self,ahead} = 0)
      double subopt_a2 = (gap_to_leader > 0)
          ? my_speed * my_speed / (2.0 * gap_to_leader + leader_speed * leader_speed / (a_ahead + 1e-9))
          : my_max_decel;
      subopt_a2 = std::min (subopt_a2, my_max_decel);
      subopt_a2 = std::max (subopt_a2, 0.1);
      m_coopBraking.suboptimalDecel = subopt_a2;

      NS_LOG_INFO ("[" << Simulator::Now ().GetSeconds () << "] Vehicle " << m_id
                       << " COOPERATIVE: middle vehicle role, sigma=" << sigma
                       << "s, suboptimal_a2=" << subopt_a2
                       << " m/s^2, waiting for rear DENM from station " << follower_station_id);

      // Paper algorithm 2 step 3: vehicle 2 generates its own DENM to inform
      // vehicle 3 that the cooperative protocol has begun. Use sub-cause 2
      // to mark this as a cooperative relay (sub-cause 1 = primary brake).
      // Skip if we already have an active DENM of our own — CheckForEvents
      // will handle the lifecycle.
      if (!m_is_event_active)
        {
          TriggerDenm (99, 2);
          m_is_event_active = true;
        }

      // Schedule decision timeout
      m_coopBraking.decisionTimerEvent = Simulator::Schedule (
          Seconds (sigma), &emergencyVehicleAlert::CooperativeDecisionTimeout, this);
      return;
    }

  // === CASE: Middle vehicle — originator is ahead, no follower (I'm effectively rear) ===
  // === CASE: Rear vehicle — no follower behind me ===
  if ((originator_is_leader || senderStationId == originator_id) && !has_follower && !m_coopBraking.active)
    {
      m_coopBraking.active = true;
      m_coopBraking.originatorStationId = originator_id;
      m_coopBraking.decisionMade = true;

      NS_LOG_INFO ("[" << Simulator::Now ().GetSeconds () << "] Vehicle " << m_id
                       << " COOPERATIVE: rear vehicle role, braking at max decel="
                       << my_max_decel << " m/s^2");

      // Scale V3's brake down so it doesn't out-brake V2's softer
      // optimal_a2 and close the V2↔V3 gap. m_chain_brake_fraction
      // defaults to 0.7; 1.0 == paper-strict behaviour.
      double chain_decel = std::max (0.1, my_max_decel * m_chain_brake_fraction);

      double h_ahead = CalculateHarm (m_vehicle_mass, my_speed, chain_decel,
                                      sender_mass, sender_speed, sender_max_decel, gap_to_leader);

      LogCooperativeDecision ("rear", cause_code, senderStationId,
                              h_ahead, 0.0, h_ahead, chain_decel, 0.0, false);

      // Paper algorithm 3 step 4: vehicle 3 generates a DENM toward vehicle 2
      // so the σ-budget loop can close. Sub-cause 2 marks it as a cooperative
      // relay.
      if (!m_is_event_active)
        {
          TriggerDenm (99, 2);
          m_is_event_active = true;
        }

      ApplyCooperativeBraking (chain_decel, false);
      return;
    }

  // === CASE: Received forwarded DENM about someone further ahead ===
  if (!originator_is_leader && !m_coopBraking.active)
    {
      // I'm further back in the chain — treat as rear vehicle: brake at
      // m_chain_brake_fraction × max_decel (see attribute doc).
      m_coopBraking.active = true;
      m_coopBraking.originatorStationId = originator_id;
      m_coopBraking.decisionMade = true;

      double chain_decel = std::max (0.1, my_max_decel * m_chain_brake_fraction);

      NS_LOG_INFO ("[" << Simulator::Now ().GetSeconds () << "] Vehicle " << m_id
                       << " COOPERATIVE: chain vehicle, braking at " << chain_decel
                       << " m/s^2 (fraction=" << m_chain_brake_fraction << ")");

      double h_ahead = has_leader ?
          CalculateHarm (m_vehicle_mass, my_speed, chain_decel,
                         1500.0, leader_speed, leader_max_decel, gap_to_leader) : 0.0;

      LogCooperativeDecision ("chain", cause_code, senderStationId,
                              h_ahead, 0.0, h_ahead, chain_decel, 0.0, false);

      if (!m_is_event_active)
        {
          TriggerDenm (99, 2);
          m_is_event_active = true;
        }

      ApplyCooperativeBraking (chain_decel, false);
    }

    }
  catch (const std::exception &e)
    {
      NS_LOG_ERROR ("[" << Simulator::Now ().GetSeconds () << "s] " << m_id
                    << " HandleCooperativeDenm exception: " << e.what ());
    }
  catch (...)
    {
      NS_LOG_ERROR ("[" << Simulator::Now ().GetSeconds () << "s] " << m_id
                    << " HandleCooperativeDenm exception (unknown)");
    }
}

void
emergencyVehicleAlert::CooperativeDecisionTimeout ()
{
  if (m_coopBraking.decisionMade)
    return;

  // Paper algorithm 2 lines 11-12: σ expired without a fresh rear DENM,
  // so recompute H_total = H_{1,2} + H_{2,3} using whatever V3 state we
  // have from the last received CAM and pick the a2 that minimizes it.
  // Falling back to the closed-form suboptimal (the previous behaviour)
  // skips the optimization entirely and makes the σ knob meaningless.

  double my_speed = m_client->TraCIAPI::vehicle.getSpeed (m_id);
  double my_max_decel = m_client->TraCIAPI::vehicle.getDecel (m_id);

  // Leader info: prefer the snapshot taken when we entered the wait
  // (sender_max_decel etc. came from the DENM alacarte). For position
  // and current speed we re-query TraCI so the gap is fresh.
  auto leader = m_client->TraCIAPI::vehicle.getLeader (m_id, 200.0);
  double leader_speed = 0.0;
  double gap_to_leader = 0.0;
  double leader_max_decel = m_coopBraking.leaderAheadDecel > 0
                                ? m_coopBraking.leaderAheadDecel : 7.5;
  double leader_mass = m_coopBraking.leaderMass;
  if (!leader.first.empty () && leader.second >= 0)
    {
      try
        {
          leader_speed = m_client->TraCIAPI::vehicle.getSpeed (leader.first);
          gap_to_leader = leader.second;
        }
      catch (...)
        {
        }
    }

  // Follower info: use last neighbor-table entry (i.e. last CAM) for
  // the station id we noted when entering the wait. If we lost track,
  // assume no follower (gap=0, speed=0) — degenerates to the
  // single-pair optimization.
  double follower_speed = 0.0;
  double follower_max_decel = 7.5;
  double follower_mass = 1500.0;
  double gap_to_follower = 0.0;
  if (m_coopBraking.followerStationId != 0)
    {
      auto it = m_neighborTable.find (m_coopBraking.followerStationId);
      if (it != m_neighborTable.end ())
        {
          follower_speed = it->second.speed;

          libsumo::TraCIPosition my_pos =
              m_client->TraCIAPI::vehicle.getPosition (m_id);
          my_pos = m_client->TraCIAPI::simulation.convertXYtoLonLat (my_pos.x, my_pos.y);
          gap_to_follower = appUtil_haversineDist (my_pos.y, my_pos.x,
                                                    it->second.latitude,
                                                    it->second.longitude);
        }
    }

  double optimal_a2 = CalculateOptimalDeceleration (
      leader_speed, leader_max_decel, leader_mass,
      my_speed, m_vehicle_mass, my_max_decel,
      follower_speed, follower_max_decel, follower_mass,
      gap_to_leader, gap_to_follower);

  double h12 = CalculateHarm (m_vehicle_mass, my_speed, optimal_a2,
                              leader_mass, leader_speed, leader_max_decel,
                              gap_to_leader);
  double h23 = (gap_to_follower > 0)
                   ? CalculateHarm (follower_mass, follower_speed,
                                    follower_max_decel, m_vehicle_mass,
                                    my_speed, optimal_a2, gap_to_follower)
                   : 0.0;

  NS_LOG_INFO ("[" << Simulator::Now ().GetSeconds () << "] Vehicle " << m_id
                   << " COOPERATIVE: sigma timeout, optimization on last-CAM data"
                   << " optimal_a2=" << optimal_a2
                   << " H12=" << h12 << " H23=" << h23
                   << " Htotal=" << h12 + h23);

  // isOptimal=false here because the decision was made without a fresh
  // rear DENM; the COOP CSV's isOptimal column still flags "rear DENM
  // arrived in σ" semantics. The optimal_a2 magnitude itself reflects
  // the joint H_total minimization.
  LogCooperativeDecision ("middle", -1, m_coopBraking.originatorStationId,
                          h12, h23, h12 + h23, optimal_a2,
                          m_coopBraking.sigma, false);

  ApplyCooperativeBraking (optimal_a2, false);
}

void
emergencyVehicleAlert::ApplyCooperativeBraking (double deceleration, bool isOptimal)
{
  m_coopBraking.decisionMade = true;
  m_coopBraking.appliedDeceleration = deceleration;

  double current_speed = m_client->TraCIAPI::vehicle.getSpeed (m_id);
  if (current_speed > 0 && deceleration > 0)
    {
      double duration = current_speed / deceleration;
      m_client->TraCIAPI::vehicle.slowDown (m_id, 0.0, duration);
    }

  // Visual feedback: yellow for optimal, orange for suboptimal
  libsumo::TraCIColor color;
  if (isOptimal)
    {
      color.r = 255; color.g = 255; color.b = 0; color.a = 255; // yellow
    }
  else
    {
      color.r = 255; color.g = 165; color.b = 0; color.a = 255; // orange
    }
  m_client->TraCIAPI::vehicle.setColor (m_id, color);
}

void
emergencyVehicleAlert::LogCooperativeDecision (const std::string &role, long causeCode,
                                               unsigned long senderStationId, double harm12,
                                               double harm23, double harmTotal,
                                               double deceleration, double sigma,
                                               bool rearDenmInSigma)
{
  // Counters for the end-of-run summary. Count only "middle" decisions —
  // chain / rear branches don't exercise the σ-budget loop.
  if (role == "middle")
    {
      if (rearDenmInSigma)
        m_coop_optimal_count++;
      else
        m_coop_suboptimal_count++;
      m_coop_sigma_sum += sigma;
      m_coop_decel_sum += deceleration;
    }

  if (m_csv_ofstream_coop.is_open ())
    {
      m_csv_ofstream_coop << Simulator::Now ().GetSeconds () << ","
                          << m_id << ","
                          << role << ","
                          << causeCode << ","
                          << senderStationId << ","
                          << harm12 << ","
                          << harm23 << ","
                          << harmTotal << ","
                          << deceleration << ","
                          << sigma << ","
                          << (rearDenmInSigma ? 1 : 0) << ","
                          << m_sigma_mode << ","
                          << m_fixed_sigma << std::endl;
    }
}

} // namespace ns3