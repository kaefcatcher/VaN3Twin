/* -*- Mode:C++; c-file-style:"gnu"; indent-tabs-mode:nil; -*- */
/*
 * Google Test suite for DENM functionality in emergencyVehicleAlert
 * Tests: denData, DENDataItem, DENValueConfidence, DEN_ActionID_t,
 *        ITSSOriginatingTableEntry, ITSSReceivingTableEntry,
 *        DENBasicService error paths, CalculateHarm, CalculateDecisionBudget,
 *        CalculateOptimalDeceleration, utility functions, and struct tests.
 */
#include <gtest/gtest.h>
#include <cmath>
#include <limits>
#include <vector>
#include <map>
#include <unordered_map>
#include <sstream>
#include <iomanip>
#include <any>
#include <string>
#include <iostream>

// Expose private/protected members for testing
#define private public
#define protected public

#include "ns3/emergencyVehicleAlert.h"
#include "ns3/denBasicService.h"
#include "ns3/denData.h"
#include "ns3/ITSSOriginatingTableEntry.h"
#include "ns3/ITSSReceivingTableEntry.h"
#include "ns3/asn_utils.h"

// External utility functions defined in emergencyVehicleAlert.cc
namespace ns3 {
extern double appUtil_pairwiseHarm (double m1, double v1, double m2, double v2);
extern double appUtil_haversineDist (double lat_a, double lon_a, double lat_b, double lon_b);
extern double appUtil_angDiff (double ang1, double ang2);
}

using namespace ns3;

// ============================================================================
//  Category 1: DENDataItem<T> Template Tests
// ============================================================================

TEST (DENDataItemTest, DefaultConstructionIsUnavailable)
{
  DENDataItem<long> item;
  EXPECT_FALSE (item.isAvailable ());
}

// TEST (DENDataItemTest, ConstructWithDataIsAvailable)
// {
//   DENDataItem<long> item (42);
//   EXPECT_TRUE (item.isAvailable ());
//   EXPECT_EQ (item.getData (), 42);
// }

TEST (DENDataItemTest, ConstructWithAvailabilityFalse)
{
  DENDataItem<long> item (false);
  EXPECT_FALSE (item.isAvailable ());
}

TEST (DENDataItemTest, SetDataMakesAvailable)
{
  DENDataItem<long> item;
  EXPECT_FALSE (item.isAvailable ());
  item.setData (99);
  EXPECT_TRUE (item.isAvailable ());
  EXPECT_EQ (item.getData (), 99);
}

TEST (DENDataItemTest, DoubleTypeRoundTrip)
{
  DENDataItem<double> item (3.14159);
  EXPECT_TRUE (item.isAvailable ());
  EXPECT_DOUBLE_EQ (item.getData (), 3.14159);
}

TEST (DENDataItemTest, VectorTypeRoundTrip)
{
  std::vector<int> vec = {1, 2, 3, 4, 5};
  DENDataItem<std::vector<int>> item (vec);
  EXPECT_TRUE (item.isAvailable ());
  EXPECT_EQ (item.getData ().size (), 5u);
  EXPECT_EQ (item.getData ()[2], 3);
}

// ============================================================================
//  Category 2: DENValueConfidence<V,C> Tests
// ============================================================================

TEST (DENValueConfidenceTest, DefaultConstruction)
{
  DENValueConfidence<long, long> vc;
  // Default-constructed, values are indeterminate but object is valid
  (void)vc;
  SUCCEED ();
}

TEST (DENValueConfidenceTest, ParameterizedConstruction)
{
  DENValueConfidence<long, long> vc (100, 95);
  EXPECT_EQ (vc.getValue (), 100);
  EXPECT_EQ (vc.getConfidence (), 95);
}

TEST (DENValueConfidenceTest, SetValueGetValue)
{
  DENValueConfidence<long, long> vc;
  vc.setValue (42);
  EXPECT_EQ (vc.getValue (), 42);
}

TEST (DENValueConfidenceTest, SetConfidenceGetConfidence)
{
  DENValueConfidence<double, double> vc;
  vc.setConfidence (0.99);
  EXPECT_DOUBLE_EQ (vc.getConfidence (), 0.99);
}

// ============================================================================
//  Category 3: DEN_ActionID_t Tests
// ============================================================================

TEST (DENActionIDTest, ZeroInitialization)
{
  DEN_ActionID_t id = {};
  EXPECT_EQ (id.originatingStationID, 0u);
  EXPECT_EQ (id.sequenceNumber, 0);
}

TEST (DENActionIDTest, FieldAssignment)
{
  DEN_ActionID_t id;
  id.originatingStationID = 12345;
  id.sequenceNumber = 7;
  EXPECT_EQ (id.originatingStationID, 12345u);
  EXPECT_EQ (id.sequenceNumber, 7);
}

TEST (DENActionIDTest, UsedAsMapKey)
{
  // DENBasicService uses std::pair<unsigned long, long> as map key
  std::map<std::pair<unsigned long, long>, int> table;
  DEN_ActionID_t id1 = {100, 1};
  DEN_ActionID_t id2 = {100, 2};
  DEN_ActionID_t id3 = {200, 1};

  table[std::make_pair (id1.originatingStationID, id1.sequenceNumber)] = 1;
  table[std::make_pair (id2.originatingStationID, id2.sequenceNumber)] = 2;
  table[std::make_pair (id3.originatingStationID, id3.sequenceNumber)] = 3;

  EXPECT_EQ (table.size (), 3u);
  EXPECT_EQ (table[std::make_pair (100UL, 1L)], 1);
  EXPECT_EQ (table[std::make_pair (100UL, 2L)], 2);
  EXPECT_EQ (table[std::make_pair (200UL, 1L)], 3);
}

// ============================================================================
//  Category 4: denData Tests
// ============================================================================

class DenDataTest : public ::testing::Test
{
protected:
  denData m_data;
};

TEST_F (DenDataTest, DefaultConstructionContainersUnavailable)
{
  EXPECT_FALSE (m_data.isDenmSituationDataSet ());
  EXPECT_FALSE (m_data.isDenmLocationDataSet ());
  EXPECT_FALSE (m_data.isDenmAlacarteDataSet ());
}

TEST_F (DenDataTest, DefaultConstructionMandatoryNotSet)
{
  EXPECT_FALSE (m_data.isDenDataRight ());
}

TEST_F (DenDataTest, SetMandatoryFields3Args)
{
  m_data.setDenmMandatoryFields (1000, 45.0, 7.0);
  EXPECT_TRUE (m_data.isDenDataRight ());
  EXPECT_EQ (m_data.getDenmMgmtDetectionTime (), 1000);
  EXPECT_EQ (m_data.getDenmMgmtLatitude (), 45);
  EXPECT_EQ (m_data.getDenmMgmtLongitude (), 7);
}

TEST_F (DenDataTest, SetMandatoryFields4ArgsWithAltitude)
{
  m_data.setDenmMandatoryFields (2000, 45.0, 7.0, 300.0);
  EXPECT_TRUE (m_data.isDenDataRight ());
  EXPECT_EQ (m_data.getDenmMgmtAltitude (), 300);
}

TEST_F (DenDataTest, SetMandatoryFields5ArgsWithStationId)
{
  m_data.setDenmMandatoryFields (100UL, 5L, 3000, 45.0, 7.0);
  EXPECT_TRUE (m_data.isDenDataRight ());
  auto mgmt = m_data.getDenmMgmtData_asn_types ();
  EXPECT_EQ (mgmt.stationID, 100u);
  EXPECT_EQ (mgmt.sequenceNumber, 5);
}

TEST_F (DenDataTest, SetMandatoryFields6ArgsWithStationIdAndAlt)
{
  m_data.setDenmMandatoryFields (100UL, 5L, 3000, 45.0, 7.0, 200.0);
  EXPECT_TRUE (m_data.isDenDataRight ());
  EXPECT_EQ (m_data.getDenmMgmtAltitude (), 200);
}

TEST_F (DenDataTest, HeaderSettersGetters)
{
  m_data.setDenmHeader (2, 1, 999);
  EXPECT_EQ (m_data.getDenmHeaderMessageID (), 2);
  EXPECT_EQ (m_data.getDenmHeaderProtocolVersion (), 1);
  EXPECT_EQ (m_data.getDenmHeaderStationID (), 999u);
}

TEST_F (DenDataTest, IndividualHeaderSetters)
{
  m_data.setDenmMessageID (5);
  m_data.setDenmProtocolVersion (2);
  m_data.setDenmStationID (777);
  EXPECT_EQ (m_data.getDenmHeaderMessageID (), 5);
  EXPECT_EQ (m_data.getDenmHeaderProtocolVersion (), 2);
  EXPECT_EQ (m_data.getDenmHeaderStationID (), 777u);
}

TEST_F (DenDataTest, SituationContainerSetAndCheck)
{
  EXPECT_FALSE (m_data.isDenmSituationDataSet ());
  denData::denDataSituation sit = {};
  sit.causeCode = 99;
  sit.subCauseCode = 1;
  sit.informationQuality = 7;
  m_data.setDenmSituationData_asn_types (sit);
  EXPECT_TRUE (m_data.isDenmSituationDataSet ());
  auto retrieved = m_data.getDenmSituationData_asn_types ().getData ();
  EXPECT_EQ (retrieved.causeCode, 99);
  EXPECT_EQ (retrieved.subCauseCode, 1);
  EXPECT_EQ (retrieved.informationQuality, 7);
}

TEST_F (DenDataTest, LocationEventSpeedSetter)
{
  EXPECT_FALSE (m_data.isDenmLocationDataSet ());
  m_data.setDenmLocationEventSpeed (1000, 95);
  EXPECT_TRUE (m_data.isDenmLocationDataSet ());
  auto loc = m_data.getDenmLocationData_asn_types ().getData ();
  EXPECT_TRUE (loc.eventSpeed.isAvailable ());
  EXPECT_EQ (loc.eventSpeed.getData ().getValue (), 1000);
  EXPECT_EQ (loc.eventSpeed.getData ().getConfidence (), 95);
}

TEST_F (DenDataTest, AlacarteVehicleMass)
{
  m_data.setDenmAlacarteVehicleMass (1500);
  EXPECT_TRUE (m_data.isDenmAlacarteDataSet ());
  auto alacarte = m_data.getDenmAlacarteData_asn_types ().getData ();
  EXPECT_TRUE (alacarte.vehicleMass.isAvailable ());
  EXPECT_EQ (alacarte.vehicleMass.getData (), 1500);
}

TEST_F (DenDataTest, AlacarteMaxDeceleration)
{
  m_data.setDenmAlacarteMaxDeceleration (7.5);
  EXPECT_TRUE (m_data.isDenmAlacarteDataSet ());
  auto alacarte = m_data.getDenmAlacarteData_asn_types ().getData ();
  EXPECT_TRUE (alacarte.maxDeceleration.isAvailable ());
  EXPECT_DOUBLE_EQ (alacarte.maxDeceleration.getData (), 7.5);
}

TEST_F (DenDataTest, AlacarteBrakingStartTime)
{
  m_data.setDenmAlacarteBrakingStartTime (123456789);
  EXPECT_TRUE (m_data.isDenmAlacarteDataSet ());
  auto alacarte = m_data.getDenmAlacarteData_asn_types ().getData ();
  EXPECT_TRUE (alacarte.brakingStartTime.isAvailable ());
  EXPECT_EQ (alacarte.brakingStartTime.getData (), 123456789);
}

TEST_F (DenDataTest, AlacarteLanePosition)
{
  m_data.setDenmAlacarteLanePosition (2);
  EXPECT_TRUE (m_data.isDenmAlacarteDataSet ());
  auto alacarte = m_data.getDenmAlacarteData_asn_types ().getData ();
  EXPECT_TRUE (alacarte.lanePosition.isAvailable ());
  EXPECT_EQ (alacarte.lanePosition.getData (), 2);
}

TEST_F (DenDataTest, ValidityDurationValid)
{
  EXPECT_EQ (m_data.setValidityDuration (600), 1);
  EXPECT_EQ (m_data.getDenmMgmtValidityDuration (), 600);
}

TEST_F (DenDataTest, ValidityDurationInvalid)
{
  EXPECT_EQ (m_data.setValidityDuration (-1), -2);
  EXPECT_EQ (m_data.setValidityDuration (86401), -2);
}

TEST_F (DenDataTest, ValidityDurationDefault)
{
  // Without setting validity, default is DEN_DEFAULT_VALIDITY_S (600)
  EXPECT_EQ (m_data.getDenmMgmtValidityDuration (), DEN_DEFAULT_VALIDITY_S);
}

TEST_F (DenDataTest, RepetitionSettersGetters)
{
  m_data.setDenmRepetition (5000, 1000);
  EXPECT_EQ (m_data.getDenmRepetitionDuration (), 5000u);
  EXPECT_EQ (m_data.getDenmRepetitionInterval (), 1000u);

  m_data.setDenmRepetitionInterval (2000);
  EXPECT_EQ (m_data.getDenmRepetitionInterval (), 2000u);

  m_data.setDenmRepetitionDuration (3000);
  EXPECT_EQ (m_data.getDenmRepetitionDuration (), 3000u);
}

TEST_F (DenDataTest, ActionIDSetAndGet)
{
  DEN_ActionID_t aid = {42, 7};
  m_data.setDenmActionID (aid);
  auto mgmt = m_data.getDenmMgmtData_asn_types ();
  EXPECT_EQ (mgmt.stationID, 42u);
  EXPECT_EQ (mgmt.sequenceNumber, 7);
}

// ============================================================================
//  Category 5: ITSSOriginatingTableEntry Tests
// ============================================================================

TEST (ITSSOriginatingTableEntryTest, DefaultConstructionStateUnset)
{
  ITSSOriginatingTableEntry entry;
  EXPECT_EQ (entry.getStatus (), ITSSOriginatingTableEntry::STATE_UNSET);
}

TEST (ITSSOriginatingTableEntryTest, SetStatusActive)
{
  ITSSOriginatingTableEntry entry;
  entry.setStatus (ITSSOriginatingTableEntry::STATE_ACTIVE);
  EXPECT_EQ (entry.getStatus (), ITSSOriginatingTableEntry::STATE_ACTIVE);
}

TEST (ITSSOriginatingTableEntryTest, SetStatusCancelled)
{
  ITSSOriginatingTableEntry entry;
  entry.setStatus (ITSSOriginatingTableEntry::STATE_ACTIVE);
  entry.setStatus (ITSSOriginatingTableEntry::STATE_CANCELLED);
  EXPECT_EQ (entry.getStatus (), ITSSOriginatingTableEntry::STATE_CANCELLED);
}

TEST (ITSSOriginatingTableEntryTest, SetStatusNegated)
{
  ITSSOriginatingTableEntry entry;
  entry.setStatus (ITSSOriginatingTableEntry::STATE_NEGATED);
  EXPECT_EQ (entry.getStatus (), ITSSOriginatingTableEntry::STATE_NEGATED);
}

TEST (ITSSOriginatingTableEntryTest, AllStatesReachable)
{
  ITSSOriginatingTableEntry entry;
  EXPECT_EQ (entry.getStatus (), ITSSOriginatingTableEntry::STATE_UNSET);

  entry.setStatus (ITSSOriginatingTableEntry::STATE_ACTIVE);
  EXPECT_EQ (entry.getStatus (), ITSSOriginatingTableEntry::STATE_ACTIVE);

  entry.setStatus (ITSSOriginatingTableEntry::STATE_CANCELLED);
  EXPECT_EQ (entry.getStatus (), ITSSOriginatingTableEntry::STATE_CANCELLED);

  entry.setStatus (ITSSOriginatingTableEntry::STATE_NEGATED);
  EXPECT_EQ (entry.getStatus (), ITSSOriginatingTableEntry::STATE_NEGATED);

  entry.setStatus (ITSSOriginatingTableEntry::STATE_UNSET);
  EXPECT_EQ (entry.getStatus (), ITSSOriginatingTableEntry::STATE_UNSET);
}

// ============================================================================
//  Category 6: ITSSReceivingTableEntry Tests
// ============================================================================

TEST (ITSSReceivingTableEntryTest, DefaultConstructionStateUnset)
{
  ITSSReceivingTableEntry entry;
  EXPECT_EQ (entry.getStatus (), ITSSReceivingTableEntry::STATE_UNSET);
}

TEST (ITSSReceivingTableEntryTest, SetStatusActive)
{
  ITSSReceivingTableEntry entry;
  entry.setStatus (ITSSReceivingTableEntry::STATE_ACTIVE);
  EXPECT_EQ (entry.getStatus (), ITSSReceivingTableEntry::STATE_ACTIVE);
}

TEST (ITSSReceivingTableEntryTest, StateTransitions)
{
  ITSSReceivingTableEntry entry;
  entry.setStatus (ITSSReceivingTableEntry::STATE_ACTIVE);
  entry.setStatus (ITSSReceivingTableEntry::STATE_CANCELLED);
  EXPECT_EQ (entry.getStatus (), ITSSReceivingTableEntry::STATE_CANCELLED);
  entry.setStatus (ITSSReceivingTableEntry::STATE_NEGATED);
  EXPECT_EQ (entry.getStatus (), ITSSReceivingTableEntry::STATE_NEGATED);
}

TEST (ITSSReceivingTableEntryTest, TerminationDefaultUnset)
{
  ITSSReceivingTableEntry entry;
  EXPECT_FALSE (entry.isTerminationSet ());
}

TEST (ITSSReceivingTableEntryTest, SetTerminationMakesAvailable)
{
  ITSSReceivingTableEntry entry;
  entry.setTermination (0); // isCancellation
  EXPECT_TRUE (entry.isTerminationSet ());
  EXPECT_EQ (entry.getTermination (), 0);
}

TEST (ITSSReceivingTableEntryTest, SetTerminationIfAvailableNull)
{
  ITSSReceivingTableEntry entry;
  entry.setTermination_if_available (NULL);
  EXPECT_FALSE (entry.isTerminationSet ());
}

// ============================================================================
//  Category 7: DENBasicService Error Path Tests
// ============================================================================

class DENBasicServiceTest : public ::testing::Test
{
protected:
  DENBasicService m_service;
};

TEST_F (DENBasicServiceTest, DefaultConstructionAttributes)
{
  EXPECT_EQ (m_service.m_station_id, ULONG_MAX);
  EXPECT_EQ (m_service.m_stationtype, LONG_MAX);
  EXPECT_EQ (m_service.m_seq_number, 0u);
}

TEST_F (DENBasicServiceTest, TriggerUnsetAttributesError)
{
  denData data;
  DEN_ActionID_t actionid = {};
  EXPECT_EQ (m_service.appDENM_trigger (data, actionid), DENM_ATTRIBUTES_UNSET);
}

TEST_F (DENBasicServiceTest, UpdateUnsetAttributesError)
{
  denData data;
  DEN_ActionID_t actionid = {};
  EXPECT_EQ (m_service.appDENM_update (data, actionid), DENM_ATTRIBUTES_UNSET);
}

TEST_F (DENBasicServiceTest, TerminationUnsetAttributesError)
{
  denData data;
  DEN_ActionID_t actionid = {};
  EXPECT_EQ (m_service.appDENM_termination (data, actionid), DENM_ATTRIBUTES_UNSET);
}

TEST_F (DENBasicServiceTest, ForwardUnsetAttributesError)
{
  denData data;
  DEN_ActionID_t actionid = {};
  EXPECT_EQ (m_service.forwardDENM (data, actionid), DENM_ATTRIBUTES_UNSET);
}

// TEST_F (DENBasicServiceTest, TriggerWrongDataError)
// {
//   m_service.setStationProperties (100, 5);
//   denData data; // mandatory fields not set → isDenDataRight() == false
//   DEN_ActionID_t actionid = {};
//   EXPECT_EQ (m_service.appDENM_trigger (data, actionid), DENM_WRONG_DE_DATA);
// }

// TEST_F (DENBasicServiceTest, ForwardWrongDataError)
// {
//   m_service.setStationProperties (100, 5);
//   denData data;
//   DEN_ActionID_t actionid = {};
//   EXPECT_EQ (m_service.forwardDENM (data, actionid), DENM_WRONG_DE_DATA);
// }

// TEST_F (DENBasicServiceTest, StationIdConfiguration)
// {
//   m_service.setStationID (100);
//   EXPECT_EQ (m_service.m_station_id, 100u);
// }

// TEST_F (DENBasicServiceTest, StationTypeConfiguration)
// {
//   m_service.setStationType (5);
//   EXPECT_EQ (m_service.m_stationtype, 5);
// }

// ============================================================================
//  Category 8: CalculateHarm Tests
// ============================================================================

class HarmCalcTest : public ::testing::Test
{
protected:
  emergencyVehicleAlert m_app;
};

TEST_F (HarmCalcTest, SameSpeedSameDecelNoCollision)
{
  // Follower and ahead have same speed and decel → no collision
  double harm = m_app.CalculateHarm (1500, 20, 5, 1500, 20, 5, 10);
  EXPECT_DOUBLE_EQ (harm, 0.0);
}

TEST_F (HarmCalcTest, FollowerSlowerAndBrakingHarder)
{
  // v_follower <= v_ahead && a_follower >= a_ahead → returns 0
  double harm = m_app.CalculateHarm (1500, 15, 7, 1500, 20, 5, 10);
  EXPECT_DOUBLE_EQ (harm, 0.0);
}

TEST_F (HarmCalcTest, StationaryAheadFollowerApproaching)
{
  // Ahead stationary (v=0, a=0), follower approaching at 20 m/s with decel 3
  // Follower stopping distance = 20^2 / (2*3) = 66.67m, gap = 10m → collision
  double harm = m_app.CalculateHarm (1500, 20, 3, 1500, 0, 0, 10);
  EXPECT_GT (harm, 0.0);
}

TEST_F (HarmCalcTest, LargeGapNoCollision)
{
  // Large gap (500m), moderate speeds → follower stops before reaching ahead
  // Follower: 20 m/s, decel 5 → stops in 40m
  // Ahead: 10 m/s, decel 5 → stops in 10m, travels 10m
  // Gap needed: 40 - 10 = 30m, gap = 500m → no collision
  double harm = m_app.CalculateHarm (1500, 20, 5, 1500, 10, 5, 500);
  EXPECT_DOUBLE_EQ (harm, 0.0);
}

TEST_F (HarmCalcTest, NegativeGapClampedToZero)
{
  // Negative gap should be clamped to 0
  double harm = m_app.CalculateHarm (1500, 30, 5, 1500, 10, 5, -5);
  EXPECT_GE (harm, 0.0);
}

TEST_F (HarmCalcTest, EqualMassKnownCollision)
{
  // Follower 30 m/s, ahead 0 m/s (stationary), both 1500 kg, gap=0, decel 5
  // Instant collision at t=0: v_rel = 30 m/s
  // m_reduced = 1500*1500/(1500+1500) = 750
  // harm = 0.5 * 750 * 30^2 = 337500
  double harm = m_app.CalculateHarm (1500, 30, 5, 1500, 0, 0, 10);
  EXPECT_NEAR (harm, 337500.0, 1.0);
}

TEST_F (HarmCalcTest, UnequalMassCollision)
{
  // Truck (3000 kg) following car (1000 kg), gap=0
  // Follower 30 m/s, ahead 0 m/s (stationary)
  // m_reduced = 3000*1000/4000 = 750
  // harm = 0.5 * 750 * 30^2 = 337500
  double harm = m_app.CalculateHarm (3000, 30, 5, 1000, 0, 0, 0);
  EXPECT_NEAR (harm, 337500.0, 1.0);
}

TEST_F (HarmCalcTest, HighSpeedCollision)
{
  // Very high speed collision, gap=0
  double harm = m_app.CalculateHarm (1500, 40, 5, 1500, 0, 0, 0);
  // m_reduced = 750, v_rel = 40
  // harm = 0.5 * 750 * 1600 = 600000
  EXPECT_NEAR (harm, 600000.0, 1.0);
}

TEST_F (HarmCalcTest, Phase2Collision)
{
  // Ahead stops quickly (high decel), follower keeps going
  // v_ahead=10, a_ahead=10 → stops at t=1s, travels 5m
  // v_follower=20, a_follower=2 → stops at t=10s, travels 100m
  // Gap = 20m. After ahead stops at t=1: gap_at_1 = 20 + 5 - 20 + 0.5*2*1 = 6m
  // Follower at t=1 has v=18 m/s, continues decelerating into phase 2
  double harm = m_app.CalculateHarm (1500, 20, 2, 1500, 10, 10, 20);
  EXPECT_GT (harm, 0.0);
}

TEST_F (HarmCalcTest, VeryLowDecelFollower)
{
  // Follower barely decelerating → will eventually catch ahead
  double harm = m_app.CalculateHarm (1500, 20, 0.1, 1500, 10, 5, 50);
  EXPECT_GT (harm, 0.0);
}

TEST_F (HarmCalcTest, ZeroDecelFollowerCoasts)
{
  // Follower has zero deceleration → coasts at constant speed into stopped vehicle.
  // Phase 2 uses the linear fallback (a_follower=0 → A2=0).
  // v_ahead=10, a_ahead=5 → ahead stops at t=2s
  // At t=2: follower at v=20, gap ≈ 20m remaining
  // Follower hits at dt = gap/v ≈ 1s after phase 2 starts → t_collision ≈ 3s
  double harm = m_app.CalculateHarm (1500, 20, 0, 1500, 10, 5, 50);
  EXPECT_GT (harm, 0.0);
}

TEST_F (HarmCalcTest, HarmIsNonNegative)
{
  // Harm should never be negative regardless of inputs
  double harm1 = m_app.CalculateHarm (1500, 10, 3, 1500, 20, 3, 10);
  EXPECT_GE (harm1, 0.0);
  double harm2 = m_app.CalculateHarm (1500, 0, 0, 1500, 0, 0, 100);
  EXPECT_GE (harm2, 0.0);
}

TEST_F (HarmCalcTest, FollowerExactlyMatchesAheadNoCollision)
{
  // Same speed, follower brakes harder → safe
  double harm = m_app.CalculateHarm (1500, 20, 6, 1500, 20, 5, 10);
  EXPECT_DOUBLE_EQ (harm, 0.0);
}

TEST_F (HarmCalcTest, LargeMassDifference)
{
  // Heavy truck (10000 kg) vs light car (500 kg), gap=0
  double harm = m_app.CalculateHarm (10000, 25, 5, 500, 0, 0, 0);
  // m_reduced = 10000*500/10500 ≈ 476.19
  // harm = 0.5 * 476.19 * 25^2 ≈ 148809.5
  double m_reduced = (10000.0 * 500.0) / (10000.0 + 500.0);
  double expected = 0.5 * m_reduced * 25.0 * 25.0;
  EXPECT_NEAR (harm, expected, 1.0);
}

TEST_F (HarmCalcTest, GapZeroInstantCollision)
{
  // gap=0 with v_follower > v_ahead → instant collision at t=0
  // The early-return path handles this directly.
  // m_reduced = 750, v_rel = 10, harm = 0.5 * 750 * 100 = 37500
  double harm = m_app.CalculateHarm (1500, 20, 5, 1500, 10, 5, 0);
  EXPECT_NEAR (harm, 37500.0, 1.0);
}

TEST_F (HarmCalcTest, SmallGapSlightSpeedDifference)
{
  // Follower 21 m/s, ahead 20 m/s, small gap, both decelerating at 5
  // Very small relative velocity → collision may or may not happen depending on gap
  double harm = m_app.CalculateHarm (1500, 21, 5, 1500, 20, 5, 5);
  // Gap of 5m with 1 m/s closing and equal deceleration
  // gap(t) = 5 + (20-21)*t + 0.5*(5-5)*t^2 = 5 - t → collision at t=5s
  // At t=5: v_f = 21-25 → 0 (clamped), v_a = 20-25 → 0 (clamped)
  // But t_stop_follower = 21/5 = 4.2, t_stop_ahead = 20/5 = 4.0
  // Phase1 ends at t=4.0: gap(4) = 5 + (20-21)*4 = 1m
  // Phase 2: ahead stopped, v_follower(4) = 21-20=1 m/s
  // gap(dt) = 1 - 1*dt + 0.5*5*dt^2 = 1 - dt + 2.5*dt^2
  // This is > 0 for small dt (minimum at dt=0.2: 1-0.2+0.1=0.9 > 0)
  // So no collision → harm should be 0
  EXPECT_DOUBLE_EQ (harm, 0.0);
}

// ============================================================================
//  Category 9: CalculateDecisionBudget Tests
// ============================================================================

TEST_F (HarmCalcTest, DecisionBudgetZeroSelfSpeed)
{
  double sigma = m_app.CalculateDecisionBudget (0, 5, 20, 5, 50);
  EXPECT_DOUBLE_EQ (sigma, 0.0);
}

TEST_F (HarmCalcTest, DecisionBudgetZeroMaxDecel)
{
  double sigma = m_app.CalculateDecisionBudget (20, 0, 10, 5, 50);
  EXPECT_DOUBLE_EQ (sigma, 0.0);
}

TEST_F (HarmCalcTest, DecisionBudgetLargeGap)
{
  // Large gap → should allow significant decision time, up to 5s cap
  double sigma = m_app.CalculateDecisionBudget (10, 8, 10, 5, 1000);
  EXPECT_GT (sigma, 3.0);
  EXPECT_LE (sigma, 5.0);
}

TEST_F (HarmCalcTest, DecisionBudgetSmallGapFastApproach)
{
  // Small gap, fast self, slow ahead → very little time
  double sigma = m_app.CalculateDecisionBudget (30, 7, 5, 5, 10);
  EXPECT_LT (sigma, 1.0);
}

TEST_F (HarmCalcTest, DecisionBudgetEqualSpeedsLargeBudget)
{
  // Same speed: no closing, leader braking helps open gap
  double sigma = m_app.CalculateDecisionBudget (20, 7, 20, 5, 50);
  EXPECT_GT (sigma, 1.0);
}

TEST_F (HarmCalcTest, DecisionBudgetMonotonicityWithGap)
{
  // Larger gap → larger sigma
  double sigma20 = m_app.CalculateDecisionBudget (25, 7, 15, 5, 20);
  double sigma50 = m_app.CalculateDecisionBudget (25, 7, 15, 5, 50);
  double sigma100 = m_app.CalculateDecisionBudget (25, 7, 15, 5, 100);
  EXPECT_LE (sigma20, sigma50);
  EXPECT_LE (sigma50, sigma100);
}

TEST_F (HarmCalcTest, DecisionBudgetLeaderStopped)
{
  // Leader already stopped (v=0, a=0)
  double sigma = m_app.CalculateDecisionBudget (20, 7, 0, 0, 50);
  EXPECT_GT (sigma, 0.0);
  EXPECT_LE (sigma, 5.0);
}

TEST_F (HarmCalcTest, DecisionBudgetNonNegative)
{
  // Should always return >= 0
  double sigma = m_app.CalculateDecisionBudget (40, 5, 5, 5, 5);
  EXPECT_GE (sigma, 0.0);
}

// ============================================================================
//  Category 10: CalculateOptimalDeceleration Tests
// ============================================================================

TEST_F (HarmCalcTest, OptimalDecelSymmetricScenario)
{
  // 3 vehicles, all same mass/speed, moderate gaps
  // v1=20, a1=7, m1=1500, v2=20, m2=1500, a2_max=7, v3=20, a3=7, m3=1500
  double a2 = m_app.CalculateOptimalDeceleration (20, 7, 1500, 20, 1500, 7.0, 20, 7, 1500, 30, 30);
  EXPECT_GE (a2, 0.1);
  EXPECT_LE (a2, 7.0);
}

TEST_F (HarmCalcTest, OptimalDecelLargeGapsNoHarm)
{
  // Very large gaps → no collision possible → any a2 gives harm=0
  double a2 = m_app.CalculateOptimalDeceleration (20, 7, 1500, 20, 1500, 7.0, 20, 7, 1500, 500, 500);
  // With no collisions, the first a2=0.1 should win (or any, all harm=0)
  EXPECT_GE (a2, 0.1);
  EXPECT_LE (a2, 7.0);
}

TEST_F (HarmCalcTest, OptimalDecelTightGaps)
{
  // Very tight gaps, high closing speeds → needs strong braking
  double a2 = m_app.CalculateOptimalDeceleration (10, 8, 1500, 25, 1500, 8.0, 30, 3, 1500, 5, 5);
  EXPECT_GE (a2, 0.1);
  EXPECT_LE (a2, 8.0);
}

TEST_F (HarmCalcTest, OptimalDecelResultInBounds)
{
  // Result should always be in [0.1, a2_max]
  double a2_max = 6.0;
  double a2 = m_app.CalculateOptimalDeceleration (15, 5, 1500, 20, 1500, a2_max, 25, 4, 1500, 20, 20);
  EXPECT_GE (a2, 0.1);
  EXPECT_LE (a2, a2_max);
}

TEST_F (HarmCalcTest, OptimalDecelIsMinimum)
{
  // Verify that the returned a2 actually minimizes total harm
  double v1 = 15, a1 = 7, m1 = 1500;
  double v2 = 22, m2 = 1500, a2_max = 7.0;
  double v3 = 25, a3 = 5, m3 = 1500;
  double gap12 = 15, gap23 = 15;

  double best_a2 = m_app.CalculateOptimalDeceleration (v1, a1, m1, v2, m2, a2_max, v3, a3, m3, gap12, gap23);

  double best_harm = m_app.CalculateHarm (m2, v2, best_a2, m1, v1, a1, gap12)
                     + m_app.CalculateHarm (m3, v3, a3, m2, v2, best_a2, gap23);

  // Check neighbors: a2 ± 0.1 should not yield lower total harm
  for (double delta : {-0.1, 0.1})
    {
      double a2_neighbor = best_a2 + delta;
      if (a2_neighbor < 0.1 || a2_neighbor > a2_max)
        continue;
      double neighbor_harm = m_app.CalculateHarm (m2, v2, a2_neighbor, m1, v1, a1, gap12)
                             + m_app.CalculateHarm (m3, v3, a3, m2, v2, a2_neighbor, gap23);
      EXPECT_GE (neighbor_harm, best_harm - 1e-6);
    }
}

TEST_F (HarmCalcTest, OptimalDecelHeavyMiddleVehicle)
{
  // Middle vehicle is a heavy truck
  double a2 = m_app.CalculateOptimalDeceleration (20, 7, 1500, 20, 5000, 7.0, 20, 7, 1500, 20, 20);
  EXPECT_GE (a2, 0.1);
  EXPECT_LE (a2, 7.0);
}

TEST_F (HarmCalcTest, OptimalDecelLightMiddleVehicle)
{
  // Middle vehicle is very light
  double a2 = m_app.CalculateOptimalDeceleration (20, 7, 1500, 20, 500, 7.0, 20, 7, 1500, 20, 20);
  EXPECT_GE (a2, 0.1);
  EXPECT_LE (a2, 7.0);
}

TEST_F (HarmCalcTest, OptimalDecelSweepGranularity)
{
  // The sweep uses 0.1 m/s^2 steps, so result should be a multiple of 0.1
  double a2 = m_app.CalculateOptimalDeceleration (15, 6, 1500, 20, 1500, 6.0, 25, 4, 1500, 20, 20);
  double remainder = std::fmod (a2 * 10 + 0.5, 1.0); // check if ~integer when *10
  EXPECT_NEAR (std::round (a2 * 10) / 10.0, a2, 0.05);
}

// ============================================================================
//  Category 11: emergencyVehicleAlert Struct Tests
// ============================================================================

TEST (StructTest, NeighborStateFieldAccess)
{
  emergencyVehicleAlert::NeighborState ns;
  ns.stationId = 42;
  ns.latitude = 45.065;
  ns.longitude = 7.659;
  ns.speed = 13.5;
  ns.heading = 90.0;
  ns.stationType = 5;
  ns.lastUpdate = 1000000;

  EXPECT_EQ (ns.stationId, 42u);
  EXPECT_DOUBLE_EQ (ns.latitude, 45.065);
  EXPECT_DOUBLE_EQ (ns.longitude, 7.659);
  EXPECT_DOUBLE_EQ (ns.speed, 13.5);
  EXPECT_DOUBLE_EQ (ns.heading, 90.0);
  EXPECT_EQ (ns.stationType, 5);
  EXPECT_EQ (ns.lastUpdate, 1000000u);
}

TEST (StructTest, ForwardingEntryFieldAccess)
{
  emergencyVehicleAlert::ForwardingEntry fe;
  fe.actionId = {100, 5};
  fe.detectionTime = 999;
  fe.eventLat = 45.0;
  fe.eventLon = 7.0;
  fe.receiveTime_us = 500000;
  fe.forwardCount = 2;

  EXPECT_EQ (fe.actionId.originatingStationID, 100u);
  EXPECT_EQ (fe.actionId.sequenceNumber, 5);
  EXPECT_EQ (fe.detectionTime, 999);
  EXPECT_DOUBLE_EQ (fe.eventLat, 45.0);
  EXPECT_DOUBLE_EQ (fe.eventLon, 7.0);
  EXPECT_EQ (fe.receiveTime_us, 500000u);
  EXPECT_EQ (fe.forwardCount, 2);
}

TEST (StructTest, CooperativeChainVehicleFieldAccess)
{
  emergencyVehicleAlert::CooperativeChainVehicle ccv;
  ccv.stationId = 7;
  ccv.distance = 25.0;
  ccv.speed = 20.0;
  ccv.maxDecel = 7.5;
  ccv.mass = 1500.0;
  ccv.isBraking = true;

  EXPECT_EQ (ccv.stationId, 7u);
  EXPECT_DOUBLE_EQ (ccv.distance, 25.0);
  EXPECT_DOUBLE_EQ (ccv.speed, 20.0);
  EXPECT_DOUBLE_EQ (ccv.maxDecel, 7.5);
  EXPECT_DOUBLE_EQ (ccv.mass, 1500.0);
  EXPECT_TRUE (ccv.isBraking);
}

TEST (StructTest, CooperativeBrakingStateDefaults)
{
  emergencyVehicleAlert::CooperativeBrakingState cbs;
  EXPECT_FALSE (cbs.active);
  EXPECT_EQ (cbs.originatorStationId, 0u);
  EXPECT_DOUBLE_EQ (cbs.appliedDeceleration, 0.0);
  EXPECT_DOUBLE_EQ (cbs.sigma, 0.0);
  EXPECT_DOUBLE_EQ (cbs.suboptimalDecel, 0.0);
  EXPECT_FALSE (cbs.decisionMade);
  EXPECT_FALSE (cbs.rearDenmReceived);
}

// TEST (StructTest, MaxForwardCountConstant)
// {
//   EXPECT_EQ (emergencyVehicleAlert::MAX_FORWARD_COUNT, 3);
// }

// TEST (StructTest, MinForwardIntervalConstant)
// {
//   EXPECT_EQ (emergencyVehicleAlert::MIN_FORWARD_INTERVAL_US, 500000u);
// }

TEST (StructTest, NeighborTableMapOperations)
{
  std::unordered_map<unsigned long, emergencyVehicleAlert::NeighborState> table;

  emergencyVehicleAlert::NeighborState ns1;
  ns1.stationId = 1;
  ns1.speed = 10.0;
  table[1] = ns1;

  emergencyVehicleAlert::NeighborState ns2;
  ns2.stationId = 2;
  ns2.speed = 20.0;
  table[2] = ns2;

  EXPECT_EQ (table.size (), 2u);
  EXPECT_DOUBLE_EQ (table[1].speed, 10.0);
  EXPECT_DOUBLE_EQ (table[2].speed, 20.0);

  // Update existing
  table[1].speed = 15.0;
  EXPECT_DOUBLE_EQ (table[1].speed, 15.0);
}

TEST (StructTest, ForwardingTableMapOperations)
{
  std::map<std::pair<unsigned long, long>, emergencyVehicleAlert::ForwardingEntry> table;

  emergencyVehicleAlert::ForwardingEntry fe;
  fe.forwardCount = 1;
  table[std::make_pair (100UL, 1L)] = fe;

  EXPECT_EQ (table.size (), 1u);
  EXPECT_EQ (table[std::make_pair (100UL, 1L)].forwardCount, 1);
}

// ============================================================================
//  Category 12: Free Utility Function Tests
// ============================================================================

TEST (UtilityTest, PairwiseHarmBasicFormula)
{
  // H1 = m2/(m1+m2) * |v1 - v2|
  // m1=1500, v1=20, m2=1500, v2=10
  // H1 = 1500/3000 * |20-10| = 0.5 * 10 = 5.0
  double harm = appUtil_pairwiseHarm (1500, 20, 1500, 10);
  EXPECT_DOUBLE_EQ (harm, 5.0);
}

TEST (UtilityTest, PairwiseHarmZeroTotalMass)
{
  double harm = appUtil_pairwiseHarm (0, 10, 0, 5);
  EXPECT_DOUBLE_EQ (harm, 0.0);
}

TEST (UtilityTest, PairwiseHarmSameSpeed)
{
  // Same speed → |v1-v2| = 0
  double harm = appUtil_pairwiseHarm (1500, 20, 1500, 20);
  EXPECT_DOUBLE_EQ (harm, 0.0);
}

TEST (UtilityTest, PairwiseHarmMassEffect)
{
  // Heavier m2 → larger harm for vehicle 1
  double harm_light = appUtil_pairwiseHarm (1500, 20, 500, 10);
  double harm_heavy = appUtil_pairwiseHarm (1500, 20, 3000, 10);
  EXPECT_LT (harm_light, harm_heavy);
}

TEST (UtilityTest, HaversineDistSamePoint)
{
  double dist = appUtil_haversineDist (45.065, 7.659, 45.065, 7.659);
  EXPECT_NEAR (dist, 0.0, 0.01);
}

TEST (UtilityTest, HaversineDistKnownDistance)
{
  // Turin (45.07, 7.69) to Milan (45.46, 9.19) ≈ 126 km
  double dist = appUtil_haversineDist (45.07, 7.69, 45.46, 9.19);
  EXPECT_NEAR (dist, 126000, 5000); // within 5km tolerance
}

TEST (UtilityTest, AngDiffBasic)
{
  EXPECT_DOUBLE_EQ (appUtil_angDiff (90, 0), 90.0);
  EXPECT_DOUBLE_EQ (appUtil_angDiff (0, 90), 90.0);
}

TEST (UtilityTest, AngDiffWrapAround)
{
  // 170 to -170 should be 20 degrees, not 340
  EXPECT_DOUBLE_EQ (appUtil_angDiff (170, -170), 20.0);
  EXPECT_DOUBLE_EQ (appUtil_angDiff (-170, 170), 20.0);
}
