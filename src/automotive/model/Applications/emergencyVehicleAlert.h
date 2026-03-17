#ifndef EMERGENCYVEHICLEALERT_H
#define EMERGENCYVEHICLEALERT_H

#include "ns3/MetricSupervisor.h"

#include "ns3/application.h"
#include "ns3/asn_utils.h"

#include <unordered_map>

#include "ns3/denBasicService.h"
#include "ns3/caBasicService.h"
#include "ns3/cpBasicService.h"
#include "ns3/cpBasicService_v1.h"
#include "ns3/vdpTraci.h"
#include "ns3/socket.h"

#include "ns3/sumo-sensor.h"
#include "ns3/LDM.h"
#include "ns3/traci-client.h"
namespace ns3 {

class emergencyVehicleAlert : public Application
{
public:
  static TypeId GetTypeId (void);

  emergencyVehicleAlert ();

  virtual ~emergencyVehicleAlert ();

  void StopApplicationNow ();

  struct NeighborState
  {
    unsigned long stationId;
    double latitude;  // degrees
    double longitude; // degrees
    double speed;     // m/s
    double heading;   // degrees
    long stationType;
    uint64_t lastUpdate; // microseconds (simulation time)
  };

  struct ForwardingEntry
  {
    denData denmData;
    DEN_ActionID_t actionId;
    long detectionTime;
    double eventLat;  // degrees
    double eventLon;  // degrees
    uint64_t receiveTime_us;
    int forwardCount;
  };

  // void receiveCAM (CAM_t *cam, Address from);
  void receiveCAM (asn1cpp::Seq<CAM> cam, Address from);
  void receiveDENM (denData denm, Address from);
  void receiveCPM (asn1cpp::Seq<CollectivePerceptionMessage> cpm, Address from);
  void receiveCPMV1 (asn1cpp::Seq<CPMV1> cpm, Address from);

protected:
  virtual void DoDispose (void);

private:
  DENBasicService m_denService; //!< DEN Basic Service object
  CABasicService m_caService; //!< CA Basic Service object
  CPBasicService m_cpService; //!< CP Basic Service object
  CPBasicServiceV1 m_cpService_v1; //!< CP Basic Service object version 1 (for CPMv1)
  Ptr<btp> m_btp; //! BTP object
  Ptr<GeoNet> m_geoNet; //! GeoNetworking Object
  Ptr<SUMOSensor> m_sensor;
  Ptr<LDM> m_LDM; //! LDM object
  Ipv4Address m_ipAddress; //!< C-V2X self IP address (set by 'v2v-cv2x.cc')
  Ptr<Socket> m_socket; //!< Socket TX/RX for everything
  std::string m_model; //!< Communication Model (possible values: 80211p and cv2x)

  void UpdateDenm (DEN_ActionID actionid);
  void TriggerDenm (long causeCode, long subCauseCode);
  void TerminateDenm ();
  void SetMaxSpeed ();

  void CheckForEvents ();
  bool DetectHardBraking ();
  bool DetectCollisionRisk ();
  void CleanupForwardingTable ();

  vehicleData_t translateCPMV1data (asn1cpp::Seq<CPMV1> cpm, int objectIndex);
  vehicleData_t translateCPMdata (asn1cpp::Seq<CollectivePerceptionMessage> cpm,
                                  asn1cpp::Seq<PerceivedObject> object, int objectIndex);

  virtual void StartApplication (void);
  virtual void StopApplication (void);

  double m_distance_threshold;
  double m_heading_threshold;

  Ptr<TraciClient> m_client; //!< TraCI client
  std::string m_id; //!< vehicle id
  std::string m_type; //!< vehicle type
  double m_max_speed; //!< To save initial veh max speed
  double m_denm_intertime; //!< Time between two consecutives DENMs
  bool m_print_summary; //!< To print a small summary when vehicle leaves the simulation
  bool m_already_print; //!< To avoid printing two summaries
  bool m_real_time; //!< To decide wheter to use realtime scheduler
  std::string m_csv_name; //!< CSV log file name
  std::ofstream m_csv_ofstream_cam; //!< CSV log stream (CAM), created using m_csv_name

  /* Counters */
  int m_cam_received;
  int m_cpm_received;
  int m_denm_sent;
  int m_denm_received;

  EventId m_speed_ev; //!< Event to change the vehicle speed
  EventId m_send_denm_ev; //!< Event to send the DENM
  EventId m_send_cam_ev; //!< Event to send the CAM
  EventId m_update_denm_ev; //!< Event to update the DENM
  EventId m_event_check_ev; //!< Event for periodic event detection

  bool m_send_cam;
  bool m_send_cpm;

  /* Event detection */
  double m_prev_speed;
  bool m_is_event_active;
  DEN_ActionID_t m_active_action_id;
  double m_hard_brake_threshold;     // m/s², default -4.0
  double m_collision_risk_distance;  // m, default 20.0
  double m_event_check_interval;     // s, default 0.1
  double m_vehicle_mass;             // kg, default 1500.0
  bool m_ethical_braking_enabled;

  /* Neighbor tracking (filled from received CAMs) */
  std::unordered_map<unsigned long, NeighborState> m_neighborTable;

  /* DENM forwarding table */
  std::map<std::pair<unsigned long, long>, ForwardingEntry> m_forwardingTable;
  static const int MAX_FORWARD_COUNT = 3;
  static const uint64_t MIN_FORWARD_INTERVAL_US = 500000; // 500 ms in microseconds

  Ptr<MetricSupervisor> m_metric_supervisor = nullptr;
};

} // namespace ns3

#endif /* EMERGENCYVEHICLEALERT_H */
