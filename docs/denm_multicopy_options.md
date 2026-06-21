# Sending several copies of a DENM in the NR‑V2X simulator

*Exploration note for the `v2v-emergencyVehicleAlert-nrv2x` scenario.*

**Question.** When a DENM is triggered (hard brake, collision risk, …), can we send
*several copies* of the message instead of one, to raise the probability the alert is
delivered correctly?

**Answer.** Yes — there are **three independent layers** at which copies/redundancy can
be added, and the simulator already supports all three. They are complementary and can
be combined. This note documents each, with code references, and describes the
configurable option that was added (`denm_copies` / `denm_copy_spacing_ms`).

---

## TL;DR

| Layer | Mechanism | How many copies | Already there? | How to configure |
|-------|-----------|-----------------|----------------|------------------|
| **PHY** | NR SL blind re‑transmission | 1 + up to 4 (per TB) | yes | `slMaxTxTransNumPssch` (≤5), `EnableBlindReTx` |
| **Facilities** | ETSI DEN service `T_Repetition` | configurable burst | yes (now wired) | **`denm_copies`, `denm_copy_spacing_ms`** |
| **Application** | extra `appDENM_update` sends | configurable | possible (not recommended) | — |

The recommended, standards‑compliant way to “send N copies of a triggered DENM” is the
**facilities‑layer repetition**, now exposed as `denm_copies`.

---

## 1. PHY layer — NR sidelink blind re‑transmission

NR‑V2X Mode 2 can transmit each transport block more than once *at the PHY*, without the
facilities or application layer knowing. This is the native 3GPP reliability tool.

- Configured in
  [`v2v-emergencyVehicleAlert-nrv2x.cc`](../src/automotive/examples/v2v-emergencyVehicleAlert-nrv2x.cc):
  `slMaxTxTransNumPssch = 5` (max transmissions per PSSCH, i.e. 1 + 4 blind re‑Tx) and
  `nrHelper->SetUeMacAttribute("EnableBlindReTx", BooleanValue(true))`.
- The scheduler ([`NrSlUeMacSchedulerSimple`](../src/nr/model/nr-sl-ue-mac-scheduler-simple.cc))
  reserves up to `slMaxNumPerReserve = 3` resources per SCI for these copies.

**Pros:** lowest latency (copies within the same selection window), transparent to the
app, and the receiver soft‑combines them. **Cons:** applies to *all* traffic (CAMs too),
multiplies channel load (see the CBR note — each extra copy multiplies the per‑vehicle
CBR contribution), and is not DENM‑specific. It is the right knob for *blanket* link
reliability, not for *“make this one emergency message extra reliable.”*

---

## 2. Facilities layer — ETSI DEN service repetition  ← **the recommended option**

ETSI **EN 302 637‑3** defines, for the originating DEN basic service, two timers:

- **T_Repetition** — every `repetitionInterval` ms, re‑send the *same* cached DENM.
- **T_RepetitionDuration** — stop repeating after `repetitionDuration` ms.

Both are fully implemented by the active service
([`DENBasicService`](../src/automotive/model/Facilities/denBasicService.cc)):

- `appDENM_trigger` arms the timers when `repetitionDuration > 0 && repetitionInterval > 0`
  (denBasicService.cc:632–636).
- `T_RepetitionStop` re‑transmits the cached encoded DENM via BTP/GeoNetworking and
  re‑arms itself (denBasicService.cc:1253–1284) until `T_RepetitionDurationStop` cancels
  it (denBasicService.cc:1245–1250).
- The repetition parameters live in `denData`
  (`setDenmRepetition(duration, interval)` — [`denData.h`](../src/automotive/model/Facilities/denData.h):252).

Because the repetition re‑sends the **same packet**, every copy is a genuine duplicate of
the alert, and — importantly — each copy still flows through the GeoNet TX hook, so the
`MetricSupervisor` counts it in the per‑vehicle, per‑message‑type DENM Tx/PRR statistics.

### Why it was previously disabled

The application drives its own DENM lifecycle: after a trigger it re‑sends an
**update** every 500 ms via `UpdateDenm`
([`emergencyVehicleAlert.cc`](../src/automotive/model/Applications/emergencyVehicleAlert.cc)).
The original code therefore called `setDenmRepetition(0, 0)` to avoid a *double‑fire*:
if an `appDENM_update` carried non‑zero repetition parameters it would **re‑arm** the
T_Repetition timer (denBasicService.cc:741–745), stacking on top of the timer already
armed at trigger.

### What was changed

`denm_copies > 1` now arms the repetition **only on the initial trigger**, for a short
burst, while every `UpdateDenm` keeps passing `setDenmRepetition(0, 0)` — so the timer is
never re‑armed and there is no double‑fire. In
[`TriggerDenm`](../src/automotive/model/Applications/emergencyVehicleAlert.cc):

```cpp
if (m_denm_copies > 1)
  {
    long interval = (long) m_denm_copy_spacing_ms;
    long duration = (long) ((m_denm_copies - 1) * m_denm_copy_spacing_ms
                            + m_denm_copy_spacing_ms / 2.0);   // half‑interval margin
    data.setDenmRepetition (duration, interval);              // N‑1 repeats + the original = N copies
  }
else
  {
    data.setDenmRepetition (0, 0);
  }
```

So `denm_copies = N` produces the initial transmission plus `N‑1` repetitions spaced
`denm_copy_spacing_ms` apart — **N copies** within a burst of `(N‑1)·spacing` ms.

---

## 3. Application layer — manual extra sends (considered, not used)

One could schedule extra `appDENM_update` calls right after the trigger to emit copies.
This was **rejected** because:

- `UpdateDenm` re‑schedules itself every 500 ms (`m_update_denm_ev`), so reusing it for a
  burst would spawn parallel self‑rescheduling chains (runaway updates).
- A dedicated copy method would duplicate ~70 lines of DENM‑building code.
- It would re‑encode the DENM each time (different reference time) rather than re‑sending
  an identical copy, which is *not* what the standard repetition does.

The facilities‑layer mechanism (§2) achieves the same goal correctly and with far less
code, so it is preferred.

---

## 4. How to use it

### Config keys ([`config.json`](../src/automotive/examples/config.json))

| Key | Default | Meaning |
|-----|---------|---------|
| `denm_copies` | `1` | copies per triggered DENM (`1` = single send) |
| `denm_copy_spacing_ms` | `20.0` | ms between consecutive copies |

Example — send **4 copies** of every DENM, 25 ms apart (burst finishes ~75 ms after the
trigger, well inside the 500 ms update cadence):

```json
{
  "denm_copies": 4,
  "denm_copy_spacing_ms": 25.0
}
```

### Equivalent ns‑3 attributes

`emergencyVehicleAlert` exposes `DENMCopies` (Uinteger, ≥1) and `DENMCopySpacingMs`
(Double), forwarded by `emergencyVehicleAlertHelper` and set from the config in the
example.

### Guidance

- Keep `(denm_copies − 1) × denm_copy_spacing_ms` **below 500 ms** so the burst does not
  overlap the periodic `UpdateDenm`.
- More copies ⇒ higher DENM delivery probability **and** higher channel load. Validate
  against CBR (see [`nr_v2x_cbr_analysis.md`](nr_v2x_cbr_analysis.md)); each copy adds to
  the offered load just like a re‑transmission.
- Inspect the effect with the per‑vehicle, per‑message‑type PRR output
  (`*_prr_per_vehicle_messagetype.csv`): the DENM `n_tx` rises by ×`denm_copies` and the
  DENM reception ratio should improve under loss.

---

## 5. Summary

To send several copies of a triggered DENM, use **`denm_copies`** (facilities‑layer ETSI
repetition). It is standards‑compliant, re‑sends identical copies, is counted by the
metric supervisor, and composes with PHY blind re‑transmission (`slMaxTxTransNumPssch`)
for an additional, lower‑layer redundancy if needed.
