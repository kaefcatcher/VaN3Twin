/* -*-  Mode: C++; c-file-style: "gnu"; indent-tabs-mode:nil; -*- */
/*
 *   Copyright (c) 2020 Centre Tecnologic de Telecomunicacions de Catalunya (CTTC)
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License version 2 as
 *   published by the Free Software Foundation;
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the Free Software
 *   Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include "ns3/carla-module.h"
//#include "ns3/automotive-module.h"
#include "ns3/emergencyVehicleAlert-helper.h"
#include "ns3/emergencyVehicleAlert.h"
#include "ns3/traci-module.h"
#include "ns3/config-store.h"
#include "ns3/network-module.h"
#include "ns3/internet-module.h"
#include "ns3/applications-module.h"
#include "ns3/mobility-module.h"
#include "ns3/point-to-point-module.h"
#include "ns3/nr-module.h"
#include "ns3/lte-module.h"
#include "ns3/stats-module.h"
#include "ns3/config-store-module.h"
#include "ns3/log.h"
#include "ns3/antenna-module.h"
#include <iomanip>
#include "ns3/sumo_xml_parser.h"
#include "ns3/vehicle-visualizer-module.h"
#include "ns3/MetricSupervisor.h"
#include "ns3/HarmLogger.h"
#include "json.hpp"
#include <fstream>
#include <vector>
#include <sstream>
#include <set>
#include <algorithm>

#include <unistd.h>
#include "ns3/core-module.h"


using namespace ns3;
using json = nlohmann::json;

#include <fstream>
// #include <nlohmann/json.hpp>

using json = nlohmann::json;

NS_LOG_COMPONENT_DEFINE("v2v-nrv2x");

/**
 * \brief Get sidelink bitmap from string
 * \param slBitMapString The sidelink bitmap string
 * \param slBitMapVector The vector passed to store the converted sidelink bitmap
 */
 
 
void
GetSlBitmapFromString (std::string slBitMapString, std::vector <std::bitset<1> > &slBitMapVector)
{
  static std::unordered_map<std::string, uint8_t> lookupTable =
  {
    { "0", 0 },
    { "1", 1 },
  };

  std::stringstream ss (slBitMapString);
  std::string token;
  std::vector<std::string> extracted;

  while (std::getline (ss, token, '|'))
    {
      extracted.push_back (token);
    }

  for (const auto & v : extracted)
    {
      if (lookupTable.find (v) == lookupTable.end ())
        {
          NS_FATAL_ERROR ("Bit type " << v << " not valid. Valid values are: 0 and 1");
        }
      slBitMapVector.push_back (lookupTable[v] & 0x01);
    }
}


int
main (int argc, char *argv[])
{

  json config;
  std::ifstream f("src/automotive/examples/config.json");

  if (f.is_open()) {
      f >> config;
      std::cout << "Config loaded: " << config.dump() << std::endl;
  } else {
      std::cout << "ERROR: cannot open config.json" << std::endl;
  }

  // std::string sumo_folder = "src/automotive/examples/sumo_files_v2v_map/";
  // std::string mob_trace = "cars.rou.xml";
  // std::string sumo_config ="src/automotive/examples/sumo_files_v2v_map/map.sumo.cfg";
  std::string sumo_folder = config.value("sumo_folder", std::string(""));
  std::string mob_trace = config.value("mob_trace", std::string(""));
  std::string sumo_config = config.value("sumo_config", std::string(""));

  /*** 0.a App Options ***/
  bool verbose = true;
  bool realtime = config.value("realtime", false);
  bool sumo_gui = config.value("sumo_gui", true);
  double sumo_updates = config.value("sumo_updates", 0.01);
  std::string csv_name = config.value("csv_log", std::string("mylog"));
  std::string csv_name_cumulative = config.value("csv_name_cumulative", std::string("results"));
  std::string sumo_netstate_file_name;
  bool vehicle_vis = config.value("vehicle_vis", false);

  int numberOfNodes;
  uint32_t nodeCounter = 0;

  double penetrationRate = 1.0;
  bool cooperativeDetection = true;
  bool sendDenm = true;
  uint32_t denmCopies = 1;          // copies per triggered DENM (>=1); >1 enables a reliability burst
  double denmCopySpacingMs = 20.0;  // spacing [ms] between consecutive DENM copies
  std::string sigmaMode = "computed";   // "computed" | "fixed" | "scaled"
  double fixedSigma = 0.5;              // seconds (fixed) or multiplier (scaled)
  std::string harmLogFile = "harm_log.csv";
  double harmLogPeriodS = 0.1;
  double harmLogRadiusM = 150.0;

  // Forced emergency brake of a designated vehicle, used to guarantee a
  // hard-brake event in scenarios where SUMO's planned <stop> would
  // otherwise decelerate too smoothly to fire the HardBrakeThreshold gate.
  // forceBrakeTime <= 0 disables the feature.
  double forceBrakeTime = 15.0;            // seconds of simulation time
  std::string forceBrakeVehicles = "veh0";   // SUMO vehicle id to brake
  double forceBrakeDuration = 1.0;          // seconds to reach speed 0
  double forceBrakeTargetSpeed = 0.0;       // m/s
  double forceBrakePosition = 2500.0;

  // Single-vehicle DENM trigger thresholds (cause 26 / 94). Lenient
  // defaults so any meaningful brake event fires a DENM.
  double speedDropThreshold = 3.0;          // m/s drop in 1 s
  double stationarySpeed = 1.0;             // m/s
  double wasMovingSpeed = 5.0;              // m/s, hysteresis floor

  // Custom alacarte extension fields. Off by default — they're a
  // suspect for UPER encode failures.
  bool includeEthicalAlacarte = false;

  // Fraction of max_decel that V3 applies in the chain / rear cooperative
  // branch. 1.0 = paper-strict; reduce to keep V3 from out-braking V2.
  double chainBrakeFraction = 1.0;

  xmlDocPtr rou_xml_file;
  double m_baseline_prr = config.value("m_baseline_prr", 150.0);
  bool m_metric_sup = config.value("m_metric_sup", false);


  // Simulation parameters.
  double simTime = config.value("simTime", 100.0);
  //Sidelink bearers activation time
  Time slBearersActivationTime = Seconds (2.0);


  // NR parameters. We will take the input from the command line, and then we
  // will pass them inside the NR module.
  double centralFrequencyBandSl = 5.89e9; // band n47  TDD //Here band is analogous to channel
  uint16_t bandwidthBandSl = 400;
  double txPower = config.value("tx_power", 23.0); //dBm
  std::string tddPattern = "UL|UL|UL|UL|UL|UL|UL|UL|UL|UL|";
  std::string slBitMap = "1|1|1|1|1|1|1|1|1|1";
  uint16_t numerologyBwpSl = 2;
  uint16_t slSensingWindow = 100; // T0 in ms
  uint16_t slSelectionWindow = 5; // T2min
  uint16_t slSubchannelSize = config.value("slSubchannelSize", 10);
  uint16_t slMaxNumPerReserve = config.value("slMaxNumPerReserve", 3);
  double slProbResourceKeep = config.value("slProbResourceKeep", 0.0);
  uint16_t slMaxTxTransNumPssch = 5;
  uint16_t reservationPeriod = config.value("reservationPeriod", 20); // in ms
  bool enableSensing = false;
  uint16_t t1 = config.value("t1", 2);
  uint16_t t2 = config.value("t2", 81);
  int slThresPsschRsrp = -128;
  bool enableChannelRandomness = false;
  uint16_t channelUpdatePeriod = 500; //ms
  uint8_t mcs = config.value("mcs", 14);

  // Channel Busy Ratio (CBR) measurement knobs (see docs/nr_v2x_cbr_analysis.md).
  double cbrWindowMs = config.value("cbr_window_ms", 100.0); // CBR measurement window [ms]
  double cbrAlpha    = config.value("cbr_alpha", 0.5);       // EMA smoothing factor for CBR, in [0,1]
  bool cbrEnabled    = config.value("cbr_enabled", true);    // measure + print CBR when the MetricSupervisor is on

  // NR Mode-2 resource (re)selection counter. 0 keeps the standard random
  // counter of TS 38.214; a value > 0 forces a fixed SL reselection counter,
  // so reselection_counter = 1 yields fully dynamic (per-transmission) resource
  // selection, while larger values emulate semi-persistent scheduling (SPS).
  int reselCounter = config.value("reselection_counter", 0);

  // Master RNG seed. Drives BOTH the SUMO mobility RNG (SumoSeed) and the ns-3
  // radio RNG run (RngRun), so repeating a config under several seeds yields
  // independent replications for confidence intervals / box plots. "sumo_seed"
  // is accepted as a backward-compatible alias.
  int seed = config.value("seed", config.value("sumo_seed", 10));

  // === JSON CONFIG PARSER ===
  try
  {
    std::ifstream f("src/automotive/examples/config.json");
    json config = json::parse(f);

    realtime          = config.value("realtime", realtime);
    sumo_gui          = config.value("sumo_gui", sumo_gui);
    sumo_updates      = config.value("sumo_updates", sumo_updates);
    sumo_folder       = config.value("sumo_folder", sumo_folder);
    mob_trace         = config.value("mob_trace", mob_trace);
    sumo_config       = config.value("sumo_config", sumo_config);
    vehicle_vis       = config.value("vehicle_visualizer", vehicle_vis);
    penetrationRate   = config.value("penetrationRate", penetrationRate);
    simTime           = config.value("sim_time", simTime);

    txPower           = config.value("tx_power", txPower);
    slSubchannelSize  = config.value("sizeSubchannel", slSubchannelSize);
    slMaxNumPerReserve= config.value("numSubchannel", slMaxNumPerReserve);
    t1                = config.value("T1", t1);
    t2                = config.value("T2", t2);
    mcs               = config.value("mcs", mcs);
    reservationPeriod = config.value("pRsvp", reservationPeriod);
    slProbResourceKeep= config.value("probResourceKeep", slProbResourceKeep);
    m_baseline_prr    = config.value("baseline", m_baseline_prr);
    m_metric_sup      = config.value("metric_supervisor", m_metric_sup);
    sendDenm          = config.value("send_denm", sendDenm);
    denmCopies        = config.value("denm_copies", denmCopies);
    denmCopySpacingMs = config.value("denm_copy_spacing_ms", denmCopySpacingMs);
    cooperativeDetection = config.value("cooperative_detection", cooperativeDetection);
    sigmaMode         = config.value("sigma_mode", sigmaMode);
    fixedSigma        = config.value("fixed_sigma", fixedSigma);
    harmLogFile       = config.value("harm_log_file", harmLogFile);
    harmLogPeriodS    = config.value("harm_log_period_s", harmLogPeriodS);
    harmLogRadiusM    = config.value("harm_log_radius_m", harmLogRadiusM);

    cbrWindowMs       = config.value("cbr_window_ms", cbrWindowMs);
    cbrAlpha          = config.value("cbr_alpha", cbrAlpha);
    cbrEnabled        = config.value("cbr_enabled", cbrEnabled);
    reselCounter      = config.value("reselection_counter", reselCounter);
    seed              = config.value("seed", config.value("sumo_seed", seed));

    forceBrakeTime         = config.value("force_brake_time", forceBrakeTime);
    forceBrakeVehicles      = config.value("force_brake_vehicles", forceBrakeVehicles);
    forceBrakeDuration     = config.value("force_brake_duration", forceBrakeDuration);
    forceBrakeTargetSpeed  = config.value("force_brake_target_speed", forceBrakeTargetSpeed);
    forceBrakePosition     = config.value("force_brake_position", forceBrakePosition);

    speedDropThreshold     = config.value("speed_drop_threshold", speedDropThreshold);
    stationarySpeed        = config.value("stationary_speed", stationarySpeed);
    wasMovingSpeed         = config.value("was_moving_speed", wasMovingSpeed);
    includeEthicalAlacarte = config.value("include_ethical_alacarte", includeEthicalAlacarte);
    chainBrakeFraction     = config.value("chain_brake_fraction", chainBrakeFraction);

    NS_LOG_INFO("Configuration loaded from JSON");
  }
  catch (const std::exception& e)
  {
    NS_FATAL_ERROR("Error parsing config.json: " << e.what());
  }

  std::vector<std::string> brakeVehicles;
  std::stringstream ss(forceBrakeVehicles);
  std::string veh;

  while (std::getline(ss, veh, ','))
  {
      if (!veh.empty())
      {
          brakeVehicles.push_back(veh);
      }
  }

  /*
   * From here, we instruct the ns3::CommandLine class of all the input parameters
   * that we may accept as input, as well as their description, and the storage
   * variable.
   */
  CommandLine cmd;

  /* Cmd Line option for application */
  cmd.AddValue ("realtime", "Use the realtime scheduler or not", realtime);
  cmd.AddValue ("sumo-gui", "Use SUMO gui or not", sumo_gui);
  cmd.AddValue ("sumo-updates", "SUMO granularity", sumo_updates);
  cmd.AddValue ("sumo-folder","Position of sumo config files",sumo_folder);
  cmd.AddValue ("mob-trace", "Name of the mobility trace file", mob_trace);
  cmd.AddValue ("sumo-config", "Location and name of SUMO configuration file", sumo_config);
  cmd.AddValue ("csv-log", "Name of the CSV log file", csv_name);
  cmd.AddValue ("vehicle-visualizer", "Activate the web-based vehicle visualizer for ms-van3t", vehicle_vis);
  cmd.AddValue ("csv-log-cumulative", "Name of the CSV log file for the cumulative (average) PRR and latency data", csv_name_cumulative);
  cmd.AddValue ("netstate-dump-file", "Name of the SUMO netstate-dump file containing the vehicle-related information throughout the whole simulation", sumo_netstate_file_name);
  cmd.AddValue ("baseline", "Baseline for PRR calculation", m_baseline_prr);
  cmd.AddValue ("met-sup","Use the Metric supervisor or not",m_metric_sup);
  cmd.AddValue ("penetrationRate", "Rate of vehicles equipped with wireless communication devices", penetrationRate);
  cmd.AddValue ("cooperative-detection", "Enable cooperative ethical braking algorithm", cooperativeDetection);
  cmd.AddValue ("send-denm", "Enable DENM event triggering; set false for a CAM-only baseline", sendDenm);
  cmd.AddValue ("sigma-mode", "How V2 picks decision-time budget σ: 'computed', 'fixed', or 'scaled'", sigmaMode);
  cmd.AddValue ("fixed-sigma", "σ override; meaning depends on sigma-mode (seconds or multiplier)", fixedSigma);
  cmd.AddValue ("harm-log-file", "Output CSV path for the time-sampled pairwise HARM log", harmLogFile);
  cmd.AddValue ("harm-log-period-s", "Sampling period (s) of the HARM log", harmLogPeriodS);
  cmd.AddValue ("harm-log-radius-m", "Pairing radius (m) for the HARM log", harmLogRadiusM);
  cmd.AddValue ("force-brake-time", "Simulation time (s) at which to force a TraCI brake. <= 0 disables.", forceBrakeTime);
  cmd.AddValue ("force-brake-vehicle", "SUMO vehicle id to brake", forceBrakeVehicles);
  cmd.AddValue ("force-brake-duration", "Duration (s) over which the forced brake completes", forceBrakeDuration);
  cmd.AddValue ("force-brake-target-speed", "Target speed (m/s) of the forced brake (0 = full stop)", forceBrakeTargetSpeed);
  cmd.AddValue ("speed-drop-threshold", "Speed drop (m/s) within 1 s that triggers slowVehicle DENM", speedDropThreshold);
  cmd.AddValue ("stationary-speed", "Speed (m/s) below which vehicle is considered stopped", stationarySpeed);
  cmd.AddValue ("was-moving-speed", "Speed (m/s) the vehicle must have exceeded before stationary fires", wasMovingSpeed);
  cmd.AddValue ("include-ethical-alacarte", "Include custom ethical extension fields in DENM alacarte (off by default while bisecting UPER failures)", includeEthicalAlacarte);
  cmd.AddValue ("chain-brake-fraction", "Fraction of max_decel V3 uses in the chain/rear branch (1.0 = paper-strict, 0.7 = default)", chainBrakeFraction);

  cmd.AddValue ("simTime",
                "Simulation time in seconds",
                simTime);
  cmd.AddValue ("slBearerActivationTime",
                "Sidelik bearer activation time in seconds",
                slBearersActivationTime);
  cmd.AddValue ("centralFrequencyBandSl",
                "The central frequency to be used for Sidelink band/channel",
                centralFrequencyBandSl);
  cmd.AddValue ("bandwidthBandSl",
                "The system bandwidth to be used for Sidelink",
                bandwidthBandSl);
  cmd.AddValue ("txPower",
                "total tx power in dBm",
                txPower);
  cmd.AddValue ("tddPattern",
                "The TDD pattern string",
                tddPattern);
  cmd.AddValue ("slBitMap",
                "The Sidelink bitmap string",
                slBitMap);
  cmd.AddValue ("numerologyBwpSl",
                "The numerology to be used in Sidelink bandwidth part",
                numerologyBwpSl);
  cmd.AddValue ("slSensingWindow",
                "The Sidelink sensing window length in ms",
                slSensingWindow);
  cmd.AddValue ("slSelectionWindow",
                "The parameter which decides the minimum Sidelink selection "
                "window length in physical slots. T2min = slSelectionWindow * 2^numerology",
                slSelectionWindow);
  cmd.AddValue ("slSubchannelSize",
                "The Sidelink subchannel size in RBs",
                slSubchannelSize);
  cmd.AddValue ("slMaxNumPerReserve",
                "The parameter which indicates the maximum number of reserved "
                "PSCCH/PSSCH resources that can be indicated by an SCI.",
                slMaxNumPerReserve);
  cmd.AddValue ("slProbResourceKeep",
                "The parameter which indicates the probability with which the "
                "UE keeps the current resource when the resource reselection"
                "counter reaches zero.",
                slProbResourceKeep);
  cmd.AddValue ("slMaxTxTransNumPssch",
                "The parameter which indicates the maximum transmission number "
                "(including new transmission and retransmission) for PSSCH.",
                slMaxTxTransNumPssch);
  cmd.AddValue ("ReservationPeriod",
                "The resource reservation period in ms",
                reservationPeriod);
  cmd.AddValue ("enableSensing",
                "If true, it enables the sensing based resource selection for "
                "SL, otherwise, no sensing is applied",
                enableSensing);
  cmd.AddValue ("t1",
                "The start of the selection window in physical slots, "
                "accounting for physical layer processing delay",
                t1);
  cmd.AddValue ("t2",
                "The end of the selection window in physical slots",
                t2);
  cmd.AddValue ("slThresPsschRsrp",
                "A threshold in dBm used for sensing based UE autonomous resource selection",
                slThresPsschRsrp);
  cmd.AddValue ("enableChannelRandomness",
                "Enable shadowing and channel updates",
                enableChannelRandomness);
  cmd.AddValue ("channelUpdatePeriod",
                "The channel update period in ms",
                channelUpdatePeriod);
  cmd.AddValue ("mcs",
                "The MCS to used for sidelink",
                mcs);


  // Parse the command line
  cmd.Parse (argc, argv);

  // Seed the ns-3 radio RNG run from the master seed (set before any RNG draws /
  // AssignStreams). Combined with SumoSeed below, this makes each seed an
  // independent replication of both mobility and the NR sidelink stochastics.
  RngSeedManager::SetSeed (1);
  RngSeedManager::SetRun (static_cast<uint64_t> (seed));

  if (verbose)
    {
      LogComponentEnable ("v2v-nrv2x", LOG_LEVEL_INFO);
      LogComponentEnable ("CABasicService", LOG_LEVEL_INFO);
      LogComponentEnable ("DENBasicService", LOG_LEVEL_INFO);
      LogComponentEnable ("emergencyVehicleAlert", LOG_LEVEL_INFO);
    }

  /*
   * Check if the frequency is in the allowed range.
   * If you need to add other checks, here is the best position to put them.
   */
  NS_ABORT_IF (centralFrequencyBandSl > 6e9);

  /*
   * Default values for the simulation.
   */
  Config::SetDefault ("ns3::LteRlcUm::MaxTxBufferSize", UintegerValue (999999999));


  /* Use the realtime scheduler of ns3 */
  if(realtime)
      GlobalValue::Bind ("SimulatorImplementationType", StringValue ("ns3::RealtimeSimulatorImpl"));

  /***  Read from the mob_trace the number of vehicles that will be created.
   *       The number of vehicles is directly parsed from the rou.xml file, looking at all
   *       the valid XML elements of type <vehicle>
  ***/
  NS_LOG_INFO("Reading the .rou file...");
  std::string path = sumo_folder + mob_trace;

  /* Load the .rou.xml document */
  xmlInitParser();
  rou_xml_file = xmlParseFile(path.c_str ());
  if (rou_xml_file == NULL)
    {
      NS_FATAL_ERROR("Error: unable to parse the specified XML file: "<<path);
    }
  numberOfNodes = XML_rou_count_vehicles(rou_xml_file);

  xmlFreeDoc(rou_xml_file);
  xmlCleanupParser();

  if(numberOfNodes==-1)
    {
      NS_FATAL_ERROR("Fatal error: cannot gather the number of vehicles from the specified XML file: "<<path<<". Please check if it is a correct SUMO file.");
    }
  NS_LOG_INFO("The .rou file has been read: " << numberOfNodes << " vehicles will be present in the simulation.");
  /*
   * Create a NodeContainer for all the UEs
   */
  NodeContainer allSlUesContainer;
  allSlUesContainer.Create(numberOfNodes);

  /*
   * Assign mobility to the UEs.
   */
  MobilityHelper mobility;
  mobility.Install (allSlUesContainer);

  /*
   * Setup the NR module. We create the various helpers needed for the
   * NR simulation:
   * - EpcHelper, which will setup the core network
   * - NrHelper, which takes care of creating and connecting the various
   * part of the NR stack
   */
  Ptr<NrPointToPointEpcHelper> epcHelper = CreateObject<NrPointToPointEpcHelper> ();
  Ptr<NrHelper> nrHelper = CreateObject<NrHelper> ();

  // Put the pointers inside nrHelper
  nrHelper->SetEpcHelper (epcHelper);

  /*
   * Spectrum division. We create one operational band, containing
   * one component carrier, and a single bandwidth part
   * centered at the frequency specified by the input parameters.
   * We will use the StreetCanyon channel modeling.
   */
  BandwidthPartInfoPtrVector allBwps;
  CcBwpCreator ccBwpCreator;
  const uint8_t numCcPerBand = 1;

  /* Create the configuration for the CcBwpHelper. SimpleOperationBandConf
   * creates a single BWP per CC
   */
  CcBwpCreator::SimpleOperationBandConf bandConfSl (centralFrequencyBandSl, bandwidthBandSl, numCcPerBand, BandwidthPartInfo::V2V_Highway);
  //CcBwpCreator::SimpleOperationBandConf bandConfSl (centralFrequencyBandSl, bandwidthBandSl, numCcPerBand, BandwidthPartInfo::CV2X_UrbanMicrocell);

  // By using the configuration created, it is time to make the operation bands
  OperationBandInfo bandSl = ccBwpCreator.CreateOperationBandContiguousCc (bandConfSl);

  /*
   * The configured spectrum division is:
   * ------------Band1--------------
   * ------------CC1----------------
   * ------------BwpSl--------------
   */
  if (enableChannelRandomness)
    {
      Config::SetDefault ("ns3::ThreeGppChannelModel::UpdatePeriod", TimeValue (MilliSeconds (channelUpdatePeriod)));
      nrHelper->SetChannelConditionModelAttribute ("UpdatePeriod", TimeValue (MilliSeconds (channelUpdatePeriod)));
      nrHelper->SetPathlossAttribute ("ShadowingEnabled", BooleanValue (true));
    }
  else
    {
      Config::SetDefault ("ns3::ThreeGppChannelModel::UpdatePeriod", TimeValue (MilliSeconds (0)));
      nrHelper->SetChannelConditionModelAttribute ("UpdatePeriod", TimeValue (MilliSeconds (0)));
      nrHelper->SetPathlossAttribute ("ShadowingEnabled", BooleanValue (false));
    }

  /*
   * Initialize channel and pathloss, plus other things inside bandSl. If needed,
   * the band configuration can be done manually, but we leave it for more
   * sophisticated examples. For the moment, this method will take care
   * of all the spectrum initialization needs.
   */
  nrHelper->InitializeOperationBand (&bandSl);
  allBwps = CcBwpCreator::GetAllBwps ({bandSl});


  /*
   * Now, we can setup the attributes. We can have three kind of attributes:
   */

  /*
   * Antennas for all the UEs
   * We are not using beamforming in SL, rather we are using
   * quasi-omnidirectional transmission and reception, which is the default
   * configuration of the beams.
   *
   * Following attribute would be common for all the UEs
   */
  nrHelper->SetUeAntennaAttribute ("NumRows", UintegerValue (1));  //following parameter has no impact at the moment because:
  nrHelper->SetUeAntennaAttribute ("NumColumns", UintegerValue (2));
  nrHelper->SetUeAntennaAttribute ("AntennaElement", PointerValue (CreateObject<IsotropicAntennaModel> ()));

  nrHelper->SetUePhyAttribute ("TxPower", DoubleValue (txPower));

  nrHelper->SetUeMacAttribute ("EnableSensing", BooleanValue (enableSensing));
  nrHelper->SetUeMacAttribute ("T1", UintegerValue (static_cast<uint8_t> (t1)));
  nrHelper->SetUeMacAttribute ("T2", UintegerValue (t2));
  nrHelper->SetUeMacAttribute ("ActivePoolId", UintegerValue (0));
  nrHelper->SetUeMacAttribute ("ReservationPeriod", TimeValue (MilliSeconds (reservationPeriod)));
  nrHelper->SetUeMacAttribute ("NumSidelinkProcess", UintegerValue (4));
  nrHelper->SetUeMacAttribute ("EnableBlindReTx", BooleanValue (true));
  nrHelper->SetUeMacAttribute ("SlThresPsschRsrp", IntegerValue (slThresPsschRsrp));

  // Force a fixed SL resource (re)selection counter when requested. 0 (default)
  // leaves the standard random counter of TS 38.214; reselection_counter = 1
  // gives dynamic per-transmission reselection, larger values approximate SPS.
  if (reselCounter > 0)
    {
      uint16_t clamped = static_cast<uint16_t> (std::min (reselCounter, 255));
      nrHelper->SetUeMacAttribute ("SlFixedReselectionCounter",
                                   UintegerValue (clamped));
    }

  uint8_t bwpIdForGbrMcptt = 0;

  nrHelper->SetBwpManagerTypeId (TypeId::LookupByName ("ns3::NrSlBwpManagerUe"));
  nrHelper->SetUeBwpManagerAlgorithmAttribute ("GBR_MC_PUSH_TO_TALK", UintegerValue (bwpIdForGbrMcptt));

  std::set<uint8_t> bwpIdContainer;
  bwpIdContainer.insert (bwpIdForGbrMcptt);

  /*
   * We have configured the attributes we needed. Now, install and get the pointers
   * to the NetDevices, which contains all the NR stack:
   */
  NetDeviceContainer allSlUesNetDeviceContainer = nrHelper->InstallUeDevice (allSlUesContainer, allBwps);

  // When all the configuration is done, explicitly call UpdateConfig ()
  for (auto it = allSlUesNetDeviceContainer.Begin (); it != allSlUesNetDeviceContainer.End (); ++it)
    {
      DynamicCast<NrUeNetDevice> (*it)->UpdateConfig ();
    }

  /*
   * Configure Sidelink. We create the following helpers needed for the
   * NR Sidelink, i.e., V2X simulation:
   * - NrSlHelper, which will configure the UEs protocol stack to be ready to
   *   perform Sidelink related procedures.
   * - EpcHelper, which takes care of triggering the call to EpcUeNas class
   *   to establish the NR Sidelink bearer(s). We note that, at this stage
   *   just communicate the pointer of already instantiated EpcHelper object,
   *   which is the same pointer communicated to the NrHelper above.
   */
  Ptr<NrSlHelper> nrSlHelper = CreateObject <NrSlHelper> ();
  // Put the pointers inside NrSlHelper
  nrSlHelper->SetEpcHelper (epcHelper);

  /*
   * Set the SL error model and AMC
   * Error model type: ns3::NrEesmCcT1, ns3::NrEesmCcT2, ns3::NrEesmIrT1,
   *                   ns3::NrEesmIrT2, ns3::NrLteMiErrorModel
   * AMC type: NrAmc::ShannonModel or NrAmc::ErrorModel
   */
  std::string errorModel = "ns3::NrLteMiErrorModel";
  nrSlHelper->SetSlErrorModel (errorModel);
  nrSlHelper->SetUeSlAmcAttribute ("AmcModel", EnumValue (NrAmc::ErrorModel));

  /*
   * Set the SL scheduler attributes
   * In this example we use NrSlUeMacSchedulerSimple scheduler, which uses
   * fix MCS value
   */
  nrSlHelper->SetNrSlSchedulerTypeId (NrSlUeMacSchedulerSimple::GetTypeId());
  nrSlHelper->SetUeSlSchedulerAttribute ("FixNrSlMcs", BooleanValue (true));
  nrSlHelper->SetUeSlSchedulerAttribute ("InitialNrSlMcs", UintegerValue (mcs));

  /*
   * Very important method to configure UE protocol stack, i.e., it would
   * configure all the SAPs among the layers, setup callbacks, configure
   * error model, configure AMC, and configure ChunkProcessor in Interference
   * API.
   */
  nrSlHelper->PrepareUeForSidelink (allSlUesNetDeviceContainer, bwpIdContainer);


  /*
   * Start preparing for all the sub Structs/RRC Information Element (IEs)
   * of LteRrcSap::SidelinkPreconfigNr. This is the main structure, which would
   * hold all the pre-configuration related to Sidelink.
   */

  //SlResourcePoolNr IE
  LteRrcSap::SlResourcePoolNr slResourcePoolNr;
  //get it from pool factory
  Ptr<NrSlCommPreconfigResourcePoolFactory> ptrFactory = Create<NrSlCommPreconfigResourcePoolFactory> ();
  /*
   * Above pool factory is created to help the users of the simulator to create
   * a pool with valid default configuration. Please have a look at the
   * constructor of NrSlCommPreconfigResourcePoolFactory class.
   *
   * In the following, we show how one could change those default pool parameter
   * values as per the need.
   */
  std::vector <std::bitset<1> > slBitMapVector;
  GetSlBitmapFromString (slBitMap, slBitMapVector);
  NS_ABORT_MSG_IF (slBitMapVector.empty (), "GetSlBitmapFromString failed to generate SL bitmap");
  ptrFactory->SetSlTimeResources (slBitMapVector);
  ptrFactory->SetSlSensingWindow (slSensingWindow); // T0 in ms
  ptrFactory->SetSlSelectionWindow (slSelectionWindow);
  ptrFactory->SetSlFreqResourcePscch (10); // PSCCH RBs
  ptrFactory->SetSlSubchannelSize (slSubchannelSize);
  ptrFactory->SetSlMaxNumPerReserve (slMaxNumPerReserve);
  //Once parameters are configured, we can create the pool
  LteRrcSap::SlResourcePoolNr pool = ptrFactory->CreatePool ();
  slResourcePoolNr = pool;

  //Configure the SlResourcePoolConfigNr IE, which hold a pool and its id
  LteRrcSap::SlResourcePoolConfigNr slresoPoolConfigNr;
  slresoPoolConfigNr.haveSlResourcePoolConfigNr = true;
  //Pool id, ranges from 0 to 15
  uint16_t poolId = 0;
  LteRrcSap::SlResourcePoolIdNr slResourcePoolIdNr;
  slResourcePoolIdNr.id = poolId;
  slresoPoolConfigNr.slResourcePoolId = slResourcePoolIdNr;
  slresoPoolConfigNr.slResourcePool = slResourcePoolNr;

  //Configure the SlBwpPoolConfigCommonNr IE, which hold an array of pools
  LteRrcSap::SlBwpPoolConfigCommonNr slBwpPoolConfigCommonNr;
  //Array for pools, we insert the pool in the array as per its poolId
  slBwpPoolConfigCommonNr.slTxPoolSelectedNormal [slResourcePoolIdNr.id] = slresoPoolConfigNr;

  //Configure the BWP IE
  LteRrcSap::Bwp bwp;
  bwp.numerology = numerologyBwpSl;
  bwp.symbolsPerSlots = 14;
  bwp.rbPerRbg = 1;
  bwp.bandwidth = bandwidthBandSl;

  //Configure the SlBwpGeneric IE
  LteRrcSap::SlBwpGeneric slBwpGeneric;
  slBwpGeneric.bwp = bwp;
  slBwpGeneric.slLengthSymbols = LteRrcSap::GetSlLengthSymbolsEnum (14);
  slBwpGeneric.slStartSymbol = LteRrcSap::GetSlStartSymbolEnum (0);

  //Configure the SlBwpConfigCommonNr IE
  LteRrcSap::SlBwpConfigCommonNr slBwpConfigCommonNr;
  slBwpConfigCommonNr.haveSlBwpGeneric = true;
  slBwpConfigCommonNr.slBwpGeneric = slBwpGeneric;
  slBwpConfigCommonNr.haveSlBwpPoolConfigCommonNr = true;
  slBwpConfigCommonNr.slBwpPoolConfigCommonNr = slBwpPoolConfigCommonNr;

  //Configure the SlFreqConfigCommonNr IE, which hold the array to store
  //the configuration of all Sidelink BWP (s).
  LteRrcSap::SlFreqConfigCommonNr slFreConfigCommonNr;
  //Array for BWPs. Here we will iterate over the BWPs, which
  //we want to use for SL.
  for (const auto &it:bwpIdContainer)
    {
      // it is the BWP id
      slFreConfigCommonNr.slBwpList [it] = slBwpConfigCommonNr;
    }

  //Configure the TddUlDlConfigCommon IE
  LteRrcSap::TddUlDlConfigCommon tddUlDlConfigCommon;
  tddUlDlConfigCommon.tddPattern = tddPattern;

  //Configure the SlPreconfigGeneralNr IE
  LteRrcSap::SlPreconfigGeneralNr slPreconfigGeneralNr;
  slPreconfigGeneralNr.slTddConfig = tddUlDlConfigCommon;

  //Configure the SlUeSelectedConfig IE
  LteRrcSap::SlUeSelectedConfig slUeSelectedPreConfig;
  NS_ABORT_MSG_UNLESS (slProbResourceKeep <= 1.0, "slProbResourceKeep value must be between 0 and 1");
  slUeSelectedPreConfig.slProbResourceKeep = slProbResourceKeep;
  //Configure the SlPsschTxParameters IE
  LteRrcSap::SlPsschTxParameters psschParams;
  psschParams.slMaxTxTransNumPssch = static_cast<uint8_t> (slMaxTxTransNumPssch);
  //Configure the SlPsschTxConfigList IE
  LteRrcSap::SlPsschTxConfigList pscchTxConfigList;
  pscchTxConfigList.slPsschTxParameters [0] = psschParams;
  slUeSelectedPreConfig.slPsschTxConfigList = pscchTxConfigList;

  /*
   * Finally, configure the SidelinkPreconfigNr. This is the main structure
   * that needs to be communicated to NrSlUeRrc class
   */
  LteRrcSap::SidelinkPreconfigNr slPreConfigNr;
  slPreConfigNr.slPreconfigGeneral = slPreconfigGeneralNr;
  slPreConfigNr.slUeSelectedPreConfig = slUeSelectedPreConfig;
  slPreConfigNr.slPreconfigFreqInfoList [0] = slFreConfigCommonNr;

  //Communicate the above pre-configuration to the NrSlHelper
  nrSlHelper->InstallNrSlPreConfiguration (allSlUesNetDeviceContainer, slPreConfigNr);

  /****************************** End SL Configuration ***********************/

  /*
   * Fix the random streams
   */
  int64_t stream = 1;
  stream += nrHelper->AssignStreams (allSlUesNetDeviceContainer, stream);
  stream += nrSlHelper->AssignStreams (allSlUesNetDeviceContainer, stream);

  /*
   * if enableOneTxPerLane is true:
   *
   * Divide the UEs in transmitting UEs and receiving UEs. Each lane can
   * have only odd number of UEs, and on each lane middle UE would
   * be the transmitter.
   *
   * else:
   *
   * All the UEs can transmit and receive
   */
  NodeContainer txSlUes;
  NodeContainer rxSlUes;
  NetDeviceContainer txSlUesNetDevice;
  NetDeviceContainer rxSlUesNetDevice;
  txSlUes.Add (allSlUesContainer);
  rxSlUes.Add (allSlUesContainer);
  txSlUesNetDevice.Add (allSlUesNetDeviceContainer);
  rxSlUesNetDevice.Add (allSlUesNetDeviceContainer);

  /*
   * Configure the IP stack, and activate NR Sidelink bearer (s) as per the
   * configured time.
   *
   * This example supports IPV4 and IPV6
   */

  InternetStackHelper internet;
  internet.Install (allSlUesContainer);
  uint32_t dstL2Id = 255;
  Ipv4Address groupAddress4 ("225.0.0.0");     //use multicast address as destination

  Address remoteAddress;
  Address localAddress;
  uint16_t port = 8000;
  Ptr<LteSlTft> tft;

  Ipv4InterfaceContainer ueIpIface;
  ueIpIface = epcHelper->AssignUeIpv4Address (allSlUesNetDeviceContainer);

  // set the default gateway for the UE
  Ipv4StaticRoutingHelper ipv4RoutingHelper;
  for (uint32_t u = 0; u < allSlUesContainer.GetN (); ++u)
    {
      Ptr<Node> ueNode = allSlUesContainer.Get (u);
      // Set the default gateway for the UE
      Ptr<Ipv4StaticRouting> ueStaticRouting = ipv4RoutingHelper.GetStaticRouting (ueNode->GetObject<Ipv4> ());
      ueStaticRouting->SetDefaultRoute (epcHelper->GetUeDefaultGatewayAddress (), 1);
    }
  remoteAddress = InetSocketAddress (groupAddress4, port);
  localAddress = InetSocketAddress (Ipv4Address::GetAny (), port);

  tft = Create<LteSlTft> (LteSlTft::Direction::TRANSMIT, LteSlTft::CommType::GroupCast, groupAddress4, dstL2Id);
  //Set Sidelink bearers
  nrSlHelper->ActivateNrSlBearer (slBearersActivationTime, allSlUesNetDeviceContainer, tft);

  tft = Create<LteSlTft> (LteSlTft::Direction::RECEIVE, LteSlTft::CommType::GroupCast, groupAddress4, dstL2Id);
  //Set Sidelink bearers
  nrSlHelper->ActivateNrSlBearer (slBearersActivationTime, allSlUesNetDeviceContainer, tft);

  // enable log component
  //LogComponentEnable("NrUeMac", LOG_LEVEL_INFO);

  /*** 6. Setup Traci and start SUMO ***/
  Ptr<TraciClient> sumoClient = CreateObject<TraciClient> ();
  sumoClient->SetAttribute ("SumoConfigPath", StringValue (sumo_config));
  sumoClient->SetAttribute ("SumoBinaryPath", StringValue (""));    // use system installation of sumo
  sumoClient->SetAttribute ("SynchInterval", TimeValue (Seconds (sumo_updates)));
  sumoClient->SetAttribute ("StartTime", TimeValue (Seconds (0.0)));
  sumoClient->SetAttribute ("SumoGUI", BooleanValue (sumo_gui));
  sumoClient->SetAttribute ("SumoPort", UintegerValue (3400));
  sumoClient->SetAttribute ("PenetrationRate", DoubleValue (penetrationRate));
  sumoClient->SetAttribute ("SumoLogFile", BooleanValue (false));
  sumoClient->SetAttribute ("SumoStepLog", BooleanValue (false));
  sumoClient->SetAttribute ("SumoSeed", IntegerValue (seed));

  std::string sumo_additional_options = "--verbose true";

  if(sumo_netstate_file_name!="")
  {
    sumo_additional_options += " --netstate-dump " + sumo_netstate_file_name;
  }

  sumoClient->SetAttribute ("SumoAdditionalCmdOptions", StringValue (sumo_additional_options));
  sumoClient->SetAttribute ("SumoWaitForSocket", TimeValue (Seconds (1.0)));

  /* Create and setup the web-based vehicle visualizer of ms-van3t */
  vehicleVisualizer vehicleVisObj;
  Ptr<vehicleVisualizer> vehicleVis = &vehicleVisObj;
  if (vehicle_vis)
  {
      vehicleVis->startServer();
      vehicleVis->connectToServer ();
      sumoClient->SetAttribute ("VehicleVisualizer", PointerValue (vehicleVis));
  }

  Ptr<MetricSupervisor> metSup = NULL;
  MetricSupervisor prrSupObj(m_baseline_prr);
  if(m_metric_sup)
    {
      metSup = &prrSupObj;
      metSup->setTraCIClient(sumoClient);

      /* Enable Channel Busy Ratio (CBR) measurement over the NR sidelink.
       * The MetricSupervisor subscribes to each UE's ChannelOccupied trace and
       * forms a per-node exponential-moving-average CBR over cbrWindowMs.
       * See docs/nr_v2x_cbr_analysis.md for the analytical model. */
      if (cbrEnabled)
        {
          metSup->setChannelTechnology ("Nr");
          metSup->setCBRWindowValue (cbrWindowMs);
          metSup->setCBRAlphaValue (cbrAlpha);
          metSup->setSimulationTimeValue (simTime);
          metSup->setNodeContainer (allSlUesContainer);
          metSup->startCheckCBR ();
        }
    }

  /*** 7. Setup interface and application for dynamic nodes ***/
  emergencyVehicleAlertHelper EmergencyVehicleAlertHelper;
  EmergencyVehicleAlertHelper.SetAttribute ("Client", PointerValue (sumoClient));
  EmergencyVehicleAlertHelper.SetAttribute ("RealTime", BooleanValue(realtime));
  EmergencyVehicleAlertHelper.SetAttribute ("PrintSummary", BooleanValue (true));
  EmergencyVehicleAlertHelper.SetAttribute ("CSV", StringValue(csv_name));
  EmergencyVehicleAlertHelper.SetAttribute ("Model", StringValue ("nrv2x"));
  EmergencyVehicleAlertHelper.SetAttribute ("MetricSupervisor", PointerValue (metSup));

  if (cooperativeDetection)
    {
      EmergencyVehicleAlertHelper.SetAttribute ("CooperativeDetection", BooleanValue (true));
      EmergencyVehicleAlertHelper.SetAttribute ("EthicalBraking", BooleanValue (true));
    }

  EmergencyVehicleAlertHelper.SetAttribute ("SendDENM", BooleanValue (sendDenm));
  EmergencyVehicleAlertHelper.SetAttribute ("DENMCopies", UintegerValue (denmCopies));
  EmergencyVehicleAlertHelper.SetAttribute ("DENMCopySpacingMs", DoubleValue (denmCopySpacingMs));
  EmergencyVehicleAlertHelper.SetAttribute ("SigmaMode", StringValue (sigmaMode));
  EmergencyVehicleAlertHelper.SetAttribute ("FixedSigma", DoubleValue (fixedSigma));
  EmergencyVehicleAlertHelper.SetAttribute ("SpeedDropThreshold", DoubleValue (speedDropThreshold));
  EmergencyVehicleAlertHelper.SetAttribute ("StationarySpeed", DoubleValue (stationarySpeed));
  EmergencyVehicleAlertHelper.SetAttribute ("WasMovingSpeed", DoubleValue (wasMovingSpeed));
  EmergencyVehicleAlertHelper.SetAttribute ("IncludeEthicalAlacarte", BooleanValue (includeEthicalAlacarte));
  EmergencyVehicleAlertHelper.SetAttribute ("ChainBrakeFraction", DoubleValue (chainBrakeFraction));

  /* callback function for node creation */
  int i=0;
  STARTUP_FCN setupNewWifiNode = [&] (std::string vehicleID,TraciClient::StationTypeTraCI_t stationType) -> Ptr<Node>
    {
      if (nodeCounter >= allSlUesContainer.GetN())
        NS_FATAL_ERROR("Node Pool empty!: " << nodeCounter << " nodes created.");

      Ptr<Node> includedNode = allSlUesContainer.Get(nodeCounter);
      ++nodeCounter; // increment counter for next node

      /* Install Application */
      EmergencyVehicleAlertHelper.SetAttribute ("IpAddr", Ipv4AddressValue(groupAddress4));
      i++;

      //ApplicationContainer CAMSenderApp = CamSenderHelper.Install (includedNode);
      ApplicationContainer AppSample = EmergencyVehicleAlertHelper.Install (includedNode);

      AppSample.Start (Seconds (0.0));
      AppSample.Stop (Seconds(simTime) - Simulator::Now () - Seconds (0.1));

      return includedNode;
    };

  /* callback function for node shutdown */
  SHUTDOWN_FCN shutdownWifiNode = [] (Ptr<Node> exNode,std::string vehicleID)
    {
      /* stop all applications */
      Ptr<emergencyVehicleAlert> appSample_ = exNode->GetApplication(0)->GetObject<emergencyVehicleAlert>();

      if(appSample_)
        appSample_->StopApplicationNow();

       /* set position outside communication range */
      Ptr<ConstantPositionMobilityModel> mob = exNode->GetObject<ConstantPositionMobilityModel>();
      mob->SetPosition(Vector(-1000.0+(rand()%25),320.0+(rand()%25),250.0)); // rand() for visualization purposes

      /* NOTE: further actions could be required for a safe shut down! */
    };

  /* start traci client with given function pointers */
  sumoClient->SumoSetup (setupNewWifiNode, shutdownWifiNode);

  /* Time-sampled pairwise HARM log over SUMO ground truth. Independent of
   * the V2X stack, so it isolates the algorithm's effect from PRR / loss.
   */
  Ptr<HarmLogger> harmLogger =
      Create<HarmLogger> (sumoClient, harmLogFile, harmLogPeriodS, harmLogRadiusM);
  harmLogger->Start ();

  /* Forced emergency brake. SUMO's planned <stop> tends to use a smooth
   * deceleration profile that doesn't reliably cross the -4 m/s²
   * HardBrakeThreshold the application uses. Schedule an explicit
   * slowDown via TraCI so the brake event is well-defined and the test
   * is repeatable.
   *
   * Set force_brake_time to 0 (or negative) to disable. Useful when
   * sweeping with a SUMO scenario that already produces the event.
   */
  std::set<std::string> brakedVehicles;

  std::function<void()> checkBrake;

  checkBrake = [&]()
  {
      for (const auto& v : brakeVehicles)
      {
          if (brakedVehicles.count(v))
          {
              continue;
          }

          try
          {
              double pos =
                  sumoClient->TraCIAPI::vehicle.getLanePosition(v);

              if (pos >= forceBrakePosition)
              {
                  NS_LOG_UNCOND(
                      "[" << Simulator::Now().GetSeconds()
                      << "s] FORCED BRAKE at "
                      << pos << " m: "
                      << v);

                  sumoClient->TraCIAPI::vehicle.slowDown(
                      v,
                      forceBrakeTargetSpeed,
                      forceBrakeDuration);

                  sumoClient->TraCIAPI::vehicle.setMaxSpeed(
                      v,
                      std::max(forceBrakeTargetSpeed, 0.001));

                  brakedVehicles.insert(v);
              }
          }
          catch (...)
          {
              // машина ещё не появилась
          }
      }

      Simulator::Schedule(
          Seconds(1.0),
          checkBrake);
  };

  Simulator::Schedule(
      Seconds(1.0),
      checkBrake);
  /*** 8. Start Simulation ***/
  Simulator::Stop (Seconds(simTime));

  Simulator::Run ();
  Simulator::Destroy ();

  if(m_metric_sup)
    {
      // Network-average CBR over the whole run (-1 if CBR measurement was off).
      double avgCBR = cbrEnabled ? metSup->getAverageCBROverall () : -1.0;

      if(csv_name_cumulative!="")
      {
        std::ofstream csv_cum_ofstream;
        std::string full_csv_name = csv_name_cumulative + ".csv";

        if(access(full_csv_name.c_str(),F_OK)!=-1)
        {
          // The file already exists
          csv_cum_ofstream.open(full_csv_name,std::ofstream::out | std::ofstream::app);
        }
        else
        {
          // The file does not exist yet
          csv_cum_ofstream.open(full_csv_name);
          csv_cum_ofstream << "current_txpower_dBm,avg_PRR,avg_latency_ms,avg_CBR" << std::endl;
        }

        csv_cum_ofstream << txPower << "," << metSup->getAveragePRR_overall () << "," << metSup->getAverageLatency_overall () << "," << avgCBR << std::endl;
      }
      std::cout << "Average PRR: " << metSup->getAveragePRR_overall () << std::endl;
      std::cout << "Average latency (ms): " << metSup->getAverageLatency_overall () << std::endl;
      std::cout << "Average CBR: " << avgCBR << std::endl;

      /* Per-vehicle, per-message-type PRR / latency statistics, e.g.
       * "veh1 CAM reception ratio, veh1 DENM reception ratio, ...".
       * Written to <csv_log>_prr_per_vehicle_messagetype.csv and echoed to stdout. */
      const auto &prrTable = metSup->getAveragePRR_per_vehicle_per_messagetype ();
      const auto &txTable  = metSup->getNumberTx_per_vehicle_per_messagetype ();
      const auto &latTable = metSup->getAverageLatency_per_vehicle_per_messagetype ();

      std::string perveh_csv = (csv_name.empty () ? std::string ("run") : csv_name)
                               + "_prr_per_vehicle_messagetype.csv";
      std::ofstream perveh_ofs (perveh_csv, std::ofstream::trunc);
      perveh_ofs << "vehicle,message_type,n_tx,avg_prr,avg_latency_ms" << std::endl;

      std::cout << "Per-vehicle, per-message-type PRR:" << std::endl;
      for (const auto &vehEntry : prrTable)
        {
          uint64_t veh = vehEntry.first;
          for (const auto &mtEntry : vehEntry.second)
            {
              auto mt = mtEntry.first;
              double prr = mtEntry.second;

              uint64_t ntx = 0;
              auto txVehIt = txTable.find (veh);
              if (txVehIt != txTable.end ())
                {
                  auto txMtIt = txVehIt->second.find (mt);
                  if (txMtIt != txVehIt->second.end ()) ntx = txMtIt->second;
                }

              double lat = 0.0;
              auto latVehIt = latTable.find (veh);
              if (latVehIt != latTable.end ())
                {
                  auto latMtIt = latVehIt->second.find (mt);
                  if (latMtIt != latVehIt->second.end ()) lat = latMtIt->second;
                }

              std::string mtStr = MetricSupervisor::messageTypeToString (mt);
              perveh_ofs << "veh" << veh << "," << mtStr << "," << ntx << ","
                         << prr << "," << lat << std::endl;
              std::cout << "  veh" << veh << " " << mtStr << " reception ratio: "
                        << prr << " (tx=" << ntx << ", latency=" << lat << " ms)"
                        << std::endl;
            }
        }
      perveh_ofs.close ();
    }


  return 0;
}


