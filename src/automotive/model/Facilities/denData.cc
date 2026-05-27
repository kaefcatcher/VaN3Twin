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
*/

#include "denData.h"

#include <iostream>

#include "ns3/Seq.hpp"
#include "ns3/Getter.hpp"
#include "ns3/Setter.hpp"
#include "ns3/Encoding.hpp"
#include "ns3/SetOf.hpp"
#include "ns3/SequenceOf.hpp"

namespace ns3 {

denData::denData ()
{
  denDataAlacarte alacarte = {};
  // m_management optional fields initialization
  m_management.termination = DENDataItem<long> (false);
  m_management.relevanceDistance = DENDataItem<long> (false);
  m_management.relevanceTrafficDirection = DENDataItem<long> (false);
  m_management.validityDuration = DENDataItem<long> (false);
  m_management.transmissionInterval = DENDataItem<long> (false);

  // m_situation optional fields initialization
  m_situation = DENDataItem<denDataSituation> ();

  // m_location optional fields initialization
  m_location = DENDataItem<denDataLocation> ();

  // m_alacarte optional fields initialization
  m_alacarte = DENDataItem<denDataAlacarte> ();

  // Set m_internals isMandatorySet to false during the object creation
  m_internals.isMandatorySet = false;
  // Initialize the repetition parameters to 0
  m_internals.repetitionDuration = 0;
  m_internals.repetitionInterval = 0;
}

ActionID_t
denData::getDenmActionID ()
{
  ActionID retval;
  retval.originatingStationId = m_management.stationID;
  retval.sequenceNumber = m_management.sequenceNumber;
  return retval;
}
void
denData::setDenmMandatoryFields (long detectionTime_ms, double latReference_deg,
                                 double longReference_deg)
{
  m_management.detectionTime = detectionTime_ms;

  // ASN.1 ranges (0.1 µdeg):
  //   Latitude  : -900_000_000 .. 900_000_001 (last == unavailable)
  //   Longitude : -1800_000_000 .. 1800_000_001 (last == unavailable)
  // SUMO scenes built without a real geo projection (projParameter="!")
  // make TraCI::convertXYtoLonLat return raw metres, which after the
  // application's "* DOT_ONE_MICRO" scale lands far above 1.8·10⁹ and
  // causes the entire DENM UPER encoding to fail (DENM_ASN1_UPER_ENC_ERROR).
  // Clamp here so the encoding always succeeds and print a warning on
  // stderr the first time we have to clamp — that points the operator at
  // the underlying SUMO-projection bug instead of letting it stay silent.
  const long LAT_MAX  =  900000000L;
  const long LAT_MIN  = -900000000L;
  const long LON_MAX  =  1800000000L;
  const long LON_MIN  = -1800000000L;

  long lat = (long) latReference_deg;
  long lon = (long) longReference_deg;

  static bool warned = false;
  if (lat > LAT_MAX || lat < LAT_MIN || lon > LON_MAX || lon < LON_MIN)
    {
      if (!warned)
        {
          std::cerr << "denData: lat/lon outside ASN.1 range, clamping. "
                       "lat_in=" << lat << " lon_in=" << lon
                    << ". Most likely the SUMO net.xml lacks a geo projection "
                       "(projParameter=\"!\"); fix the .net.xml to make the "
                       "encoded DENM coordinates correct rather than clamped."
                    << std::endl;
          warned = true;
        }
      if (lat > LAT_MAX) lat = LAT_MAX;
      if (lat < LAT_MIN) lat = LAT_MIN;
      if (lon > LON_MAX) lon = LON_MAX;
      if (lon < LON_MIN) lon = LON_MIN;
    }

  m_management.latitude = lat;
  m_management.longitude = lon;
  m_management.altitude.setValue (AltitudeValue_unavailable);
  m_management.altitude.setConfidence (AltitudeConfidence_unavailable);
  m_management.posConfidenceEllipse.semiMajorConfidence = SemiAxisLength_unavailable;
  m_management.posConfidenceEllipse.semiMinorConfidence = SemiAxisLength_unavailable;
  m_management.posConfidenceEllipse.semiMajorOrientation = HeadingValue_unavailable;

  m_internals.isMandatorySet = true;
}

void
denData::setDenmMandatoryFields (long detectionTime_ms, double latReference_deg,
                                 double longReference_deg, double altitude_m)
{

  setDenmMandatoryFields (detectionTime_ms, latReference_deg, longReference_deg);
  m_management.altitude.setValue (altitude_m);
}

void
denData::setDenmMandatoryFields_asn_types (TimestampIts_t detectionTime,
                                           ReferencePosition_t eventPosition)
{
  asn_INTEGER2long (&detectionTime, &m_management.detectionTime);
  m_management.longitude = (long) eventPosition.longitude;
  m_management.latitude = (long) eventPosition.latitude;
  m_management.posConfidenceEllipse.semiMajorConfidence =
      eventPosition.positionConfidenceEllipse.semiMajorConfidence;
  m_management.posConfidenceEllipse.semiMinorConfidence =
      eventPosition.positionConfidenceEllipse.semiMinorConfidence;
  m_management.posConfidenceEllipse.semiMajorOrientation =
      eventPosition.positionConfidenceEllipse.semiMajorOrientation;
  m_management.altitude.setValue (eventPosition.altitude.altitudeValue);
  m_management.altitude.setConfidence (eventPosition.altitude.altitudeConfidence);

  m_internals.isMandatorySet = true;
}

void
denData::setDenmMandatoryFields (unsigned long originatingStationID, long sequenceNumber,
                                 long detectionTime_ms, double latReference_deg,
                                 double longReference_deg)
{
  setDenmMandatoryFields (detectionTime_ms, latReference_deg, longReference_deg);
  m_management.stationID = originatingStationID;
  m_management.sequenceNumber = sequenceNumber;
}

void
denData::setDenmMandatoryFields (unsigned long originatingStationID, long sequenceNumber,
                                 long detectionTime_ms, double latReference_deg,
                                 double longReference_deg, double altitude_m)
{
  setDenmMandatoryFields (detectionTime_ms, latReference_deg, longReference_deg, altitude_m);
  m_management.stationID = originatingStationID;
  m_management.sequenceNumber = sequenceNumber;
}

void
denData::setDenmMandatoryFields_asn_types (ActionID_t actionID, TimestampIts_t detectionTime,
                                           ReferencePosition_t eventPosition)
{
  setDenmMandatoryFields_asn_types (detectionTime, eventPosition);
  m_management.stationID = (long) actionID.originatingStationId;
  m_management.sequenceNumber = (long) actionID.sequenceNumber;
}

void
denData::setDenmHeader (long messageID, long protocolVersion, unsigned long stationID)
{
  m_header.messageID = messageID;
  m_header.protocolVersion = protocolVersion;
  m_header.stationID = stationID;
}

void
denData::setDenmActionID (DEN_ActionID_t actionID)
{
  m_management.stationID = actionID.originatingStationID;
  m_management.sequenceNumber = actionID.sequenceNumber;
}

int
denData::setValidityDuration (long validityDuration_s)
{

  if (validityDuration_s < 0 || validityDuration_s > 86400)
    {
      return -2;
    }

  m_management.validityDuration = DENDataItem<long> (validityDuration_s);

  return 1;
}

INTEGER_t
denData::asnTimeConvert (long time)
{
  INTEGER_t value;
  memset (&value, 0, sizeof (value));
  asn_long2INTEGER (&value, time);
  return value;
}

/* Integrity check method */
bool
denData::isDenDataRight ()
{
  if (m_internals.isMandatorySet == false)
    {
      return false;
    }
  return true;
}

void
denData::setDenmAlacarteVehicleMass (long vehicle_mass)
{
  if (m_alacarte.isAvailable ())
    {
      denDataAlacarte alacarte = m_alacarte.getData ();
      alacarte.vehicleMass = DENDataItem<long> (vehicle_mass);
      m_alacarte.setData (alacarte);
    }
  else
    {
      denDataAlacarte alacarte;
      alacarte.vehicleMass = DENDataItem<long> (vehicle_mass);
      m_alacarte.setData (alacarte);
    }
}

void
denData::setDenmAlacarteMaxDeceleration (double max_deceleration)
{
  if (m_alacarte.isAvailable ())
    {
      denDataAlacarte alacarte = m_alacarte.getData ();
      alacarte.maxDeceleration = DENDataItem<double> (max_deceleration);
      m_alacarte.setData (alacarte);
    }
  else
    {
      denDataAlacarte alacarte;
      alacarte.maxDeceleration = DENDataItem<double> (max_deceleration);
      m_alacarte.setData (alacarte);
    }
}

void
denData::setDenmAlacarteBrakingStartTime (long braking_start_time)
{
  if (m_alacarte.isAvailable ())
    {
      denDataAlacarte alacarte = m_alacarte.getData ();
      alacarte.brakingStartTime = DENDataItem<long> (braking_start_time);
      m_alacarte.setData (alacarte);
    }
  else
    {
      denDataAlacarte alacarte;
      alacarte.brakingStartTime = DENDataItem<long> (braking_start_time);
      m_alacarte.setData (alacarte);
    }
}

void
denData::setDenmLocationEventSpeed (long vehicle_speed, long confidence)
{
  if (m_location.isAvailable ())
    {
      denDataLocation location = m_location.getData ();
      DENValueConfidence<long, long> speedConf (vehicle_speed, confidence);
      location.eventSpeed = DENDataItem<DENValueConfidence<long, long>> (speedConf);
      m_location.setData (location);
    }
  else
    {
      denDataLocation location;
      DENValueConfidence<long, long> speedConf (vehicle_speed, confidence);
      location.eventSpeed = DENDataItem<DENValueConfidence<long, long>> (speedConf);
      m_location.setData (location);
    }
}

void
denData::setDenmAlacarteLanePosition (long lane_position)
{
  if (m_alacarte.isAvailable ())
    {
      denDataAlacarte alacarte = m_alacarte.getData ();
      alacarte.lanePosition = DENDataItem<long> (lane_position);
      m_alacarte.setData (alacarte);
    }
  else
    {
      denDataAlacarte alacarte;
      alacarte.lanePosition = DENDataItem<long> (lane_position);
      m_alacarte.setData (alacarte);
    }
}

} // namespace ns3
