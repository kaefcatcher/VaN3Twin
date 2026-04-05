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
              "HardBrakeThreshold",
              "Acceleration threshold (m/s^2) below which hard braking is detected",
              DoubleValue (-4.0),
              MakeDoubleAccessor (&emergencyVehicleAlert::m_hard_brake_threshold),
              MakeDoubleChecker<double> ())
          .AddAttribute (
              "CollisionRiskDistance",
              "Distance threshold (m) for collision risk detection with leading vehicle",
              DoubleValue (20.0),
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

  m_prev_speed = 0.0;
  m_is_event_active = false;
  m_active_action_id = {};
  m_hard_brake_threshold = -4.0;
  m_collision_risk_distance = 20.0;
  m_event_check_interval = 0.1;
  m_vehicle_mass = 1500.0;
  m_ethical_braking_enabled = false;
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
  m_denService.addDENRxCallback (std::bind (&emergencyVehicleAlert::receiveDENM, this,
                                            std::placeholders::_1, std::placeholders::_2));
  m_denService.setRealTime (m_real_time);

  /* Set sockets, callback, station properties and TraCI VDP in CABasicService */
  m_caService.setSocketTx (m_socket);
  m_caService.setSocketRx (m_socket);
  m_caService.setStationProperties (std::stol (m_id.substr (3)), (long) stationtype);
  m_caService.addCARxCallback (std::bind (&emergencyVehicleAlert::receiveCAM, this,
                                          std::placeholders::_1, std::placeholders::_2));
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

  /* Initialize previous speed and schedule periodic event detection */
  m_prev_speed = m_client->TraCIAPI::vehicle.getSpeed (m_id);
  m_event_check_ev = Simulator::Schedule (Seconds (m_event_check_interval),
                                           &emergencyVehicleAlert::CheckForEvents, this);
}

void
emergencyVehicleAlert::StopApplication ()
{
  NS_LOG_FUNCTION (this);
  Simulator::Cancel (m_speed_ev);
  Simulator::Cancel (m_send_cam_ev);
  Simulator::Cancel (m_update_denm_ev);
  Simulator::Cancel (m_event_check_ev);

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
  double current_speed = m_client->TraCIAPI::vehicle.getSpeed (m_id);
  double acceleration = (current_speed - m_prev_speed) / m_event_check_interval;
  m_prev_speed = current_speed;

  if (!m_is_event_active)
    {
      if (DetectHardBraking ())
        {
          // causeCode=99 (dangerousSituation), subCauseCode=1 (emergencyElectronicBrakeEngaged)
          TriggerDenm (99, 1);
          m_is_event_active = true;

          NS_LOG_INFO ("[" << Simulator::Now ().GetSeconds () << "] Vehicle " << m_id
                           << " detected HARD BRAKING (accel=" << acceleration << " m/s^2)");
        }
      else if (DetectCollisionRisk ())
        {
          // causeCode=97 (collisionRisk), subCauseCode=0 (unavailable)
          TriggerDenm (97, 0);
          m_is_event_active = true;

          NS_LOG_INFO ("[" << Simulator::Now ().GetSeconds () << "] Vehicle " << m_id
                           << " detected COLLISION RISK");
        }
    }
  else
    {
      // Check if event has ended: no longer braking hard and no collision risk
      double accel = m_client->TraCIAPI::vehicle.getAcceleration (m_id);
      if (accel > m_hard_brake_threshold && !DetectCollisionRisk ())
        {
          TerminateDenm ();
          m_is_event_active = false;

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
  if (gap < m_collision_risk_distance && closing_speed > 1.0)
    {
      // Time to collision estimate
      double ttc = gap / closing_speed;
      if (ttc < 3.0) // Less than 3 seconds to collision
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
  m_denm_received++;

  // Extract action ID for sender identification
  ActionID_t action_id = denm.getDenmActionID ();
  unsigned long sender_station_id = action_id.originatingStationId;
  long sequence_number = action_id.sequenceNumber;

  // Don't process our own DENMs
  unsigned long my_station_id = std::stol (m_id.substr (3));
  if (sender_station_id == my_station_id)
    return;

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
      // Check minimum forwarding interval
      uint64_t now_us = Simulator::Now ().GetMicroSeconds ();
      if (fw_it->second.forwardCount == 0 ||
          (now_us - fw_it->second.receiveTime_us) >= MIN_FORWARD_INTERVAL_US)
        {
          DEN_ActionID_t fwd_action_id;
          fwd_action_id.originatingStationID = sender_station_id;
          fwd_action_id.sequenceNumber = sequence_number;

          DENBasicService_error_t fwd_retval =
              m_denService.forwardDENM (denm, fwd_action_id);

          if (fwd_retval == DENM_NO_ERROR)
            {
              fw_it->second.forwardCount++;
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

  // Get real coordinates from SUMO
  libsumo::TraCIPosition pos = m_client->TraCIAPI::vehicle.getPosition (m_id);
  pos = m_client->TraCIAPI::simulation.convertXYtoLonLat (pos.x, pos.y);

  // Set management container mandatory fields with real coordinates (0.1 microdegrees)
  data.setDenmMandatoryFields (compute_timestampIts (m_real_time),
                               (long) (pos.y * DOT_ONE_MICRO),
                               (long) (pos.x * DOT_ONE_MICRO));

  // Set validity duration (30 seconds for safety events)
  data.setValidityDuration (30);

  // Set repetition parameters (500ms interval, 30s duration)
  data.setDenmRepetition (30000, 500);

  // Set situation container
  denData::denDataSituation situation;
  situation.informationQuality = 1; // Based on vehicle sensor data
  situation.causeCode = causeCode;
  situation.subCauseCode = subCauseCode;
  data.setDenmSituationData_asn_types (situation);

  // Set location container — speed in cm/s, heading in 0.1 degrees
  double speed_ms = m_client->TraCIAPI::vehicle.getSpeed (m_id);
  long speed_cm_s = (long) (speed_ms * CENTI);
  data.setDenmLocationEventSpeed (speed_cm_s, SpeedConfidence_equalOrWithinOneCentimeterPerSec);

  // Set alacarte container
  data.setDenmAlacarteVehicleMass ((long) m_vehicle_mass);
  data.setDenmAlacarteLanePosition ((long) m_client->TraCIAPI::vehicle.getLanePosition (m_id));

  // Set ethical V2X custom fields if enabled
  if (m_ethical_braking_enabled)
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
      NS_LOG_ERROR ("Cannot trigger DENM. Error code: " << trigger_retval);
    }
  else
    {
      m_denm_sent++;
      m_active_action_id = actionid;

      // Change vehicle color to red to indicate event
      libsumo::TraCIColor red;
      red.r = 255;
      red.g = 0;
      red.b = 0;
      red.a = 255;
      m_client->TraCIAPI::vehicle.setColor (m_id, red);

      // Schedule periodic updates
      m_update_denm_ev =
          Simulator::Schedule (MilliSeconds (500), &emergencyVehicleAlert::UpdateDenm, this,
                               actionid);
    }
}

void
emergencyVehicleAlert::UpdateDenm (DEN_ActionID actionid)
{
  denData data;
  DENBasicService_error_t trigger_retval;

  // Get updated coordinates from SUMO
  libsumo::TraCIPosition pos = m_client->TraCIAPI::vehicle.getPosition (m_id);
  pos = m_client->TraCIAPI::simulation.convertXYtoLonLat (pos.x, pos.y);

  // Set management container mandatory fields with updated position
  data.setDenmMandatoryFields (actionid.originatingStationID, actionid.sequenceNumber,
                               compute_timestampIts (m_real_time),
                               (double) (pos.y * DOT_ONE_MICRO),
                               (double) (pos.x * DOT_ONE_MICRO));

  data.setValidityDuration (30);
  data.setDenmRepetition (30000, 500);

  // Update speed
  double speed_ms = m_client->TraCIAPI::vehicle.getSpeed (m_id);
  long speed_cm_s = (long) (speed_ms * CENTI);
  data.setDenmLocationEventSpeed (speed_cm_s, SpeedConfidence_equalOrWithinOneCentimeterPerSec);

  // Update alacarte
  data.setDenmAlacarteVehicleMass ((long) m_vehicle_mass);
  data.setDenmAlacarteLanePosition ((long) m_client->TraCIAPI::vehicle.getLanePosition (m_id));

  if (m_ethical_braking_enabled)
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

  Simulator::Cancel (m_update_denm_ev);

  denData data;
  // Set mandatory fields for termination
  libsumo::TraCIPosition pos = m_client->TraCIAPI::vehicle.getPosition (m_id);
  pos = m_client->TraCIAPI::simulation.convertXYtoLonLat (pos.x, pos.y);

  data.setDenmMandatoryFields (m_active_action_id.originatingStationID,
                               m_active_action_id.sequenceNumber,
                               compute_timestampIts (m_real_time),
                               (double) (pos.y * DOT_ONE_MICRO),
                               (double) (pos.x * DOT_ONE_MICRO));

  data.setValidityDuration (30);

  DENBasicService_error_t term_retval =
      m_denService.appDENM_termination (data, m_active_action_id);

  if (term_retval != DENM_NO_ERROR)
    {
      NS_LOG_ERROR ("Cannot terminate DENM. Error code: " << term_retval);
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

} // namespace ns3
