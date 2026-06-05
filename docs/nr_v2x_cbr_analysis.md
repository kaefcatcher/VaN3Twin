# Channel Busy Ratio (CBR) of the NR‑V2X Sidelink Channel — Analytical Model and Parameter Rationale

*Companion note for the `v2v-emergencyVehicleAlert-nrv2x` scenario (ms‑van3t / VaN3Twin).*

This document derives, from first principles, the Channel Busy Ratio (CBR) of the
NR‑V2X PC5 sidelink channel configured in
[`src/automotive/examples/v2v-emergencyVehicleAlert-nrv2x.cc`](../src/automotive/examples/v2v-emergencyVehicleAlert-nrv2x.cc),
explains **why each radio parameter was chosen**, and shows the worked calculation
that links the configured resource grid to a target load (e.g. *CBR ≈ 80 %* at a
given vehicle density).

All numerical inputs are taken directly from the simulator configuration
([`config.json`](../src/automotive/examples/config.json)) and the NR module
source, so the model can be checked against the runtime CBR that the
`MetricSupervisor` now prints at the end of a run (see
[§8](#8-measuring-cbr-inside-the-simulator)).

---

## 1. Notation and references

| Symbol | Meaning | Unit |
|--------|---------|------|
| $\mu$ | OFDM numerology | – |
| $\Delta f$ | sub‑carrier spacing (SCS) | kHz |
| $T_\text{slot}$ | slot duration | s |
| $N_\text{RB}$ | usable resource blocks in the bandwidth part | RB |
| $K$ | sub‑channel size | RB |
| $N_\text{sch}$ | number of sub‑channels in the pool | – |
| $r$ | per‑vehicle message generation rate (CAM) | Hz |
| $k$ | transmissions per transport block (1 + blind re‑Tx) | – |
| $b$ | sub‑channels occupied by one transmission | sub‑ch |
| $N$ | transmitters whose S‑RSSI is above the CBR threshold (“in‑range”) | veh |
| $\rho_\ell$ | linear vehicle density | veh/km |
| $\rho_A$ | areal vehicle density | veh/km² |

**Normative references**

- 3GPP **TS 38.215** §5.1.30 — definition of CBR for NR sidelink.
- 3GPP **TS 38.214** §8.1.4 — sidelink S‑RSSI and CBR measurement window.
- 3GPP **TS 38.211 / 38.101‑1** — numerology, RB count vs channel bandwidth.
- ETSI **EN 302 637‑2** — CAM generation rules ($T_\text{GenCam}\in[100,1000]\text{ ms}$).
- ETSI **TR 103 766 / TS 103 574** — congestion control and CBR operating points for ITS‑G5 / NR‑V2X.

---

## 2. The configured resource grid

The example fixes the PC5 physical layer as follows (file references in brackets):

| Parameter | Value | Source |
|-----------|-------|--------|
| Central frequency | 5.89 GHz, band **n47** (ITS), **TDD** | `centralFrequencyBandSl` |
| Channel bandwidth | `bandwidthBandSl = 400` → **40 MHz** | `bandwidthBandSl` |
| Numerology | $\mu = 2$ | `numerologyBwpSl` |
| TDD pattern | `UL|UL|UL|UL|UL|UL|UL|UL|UL|UL` (all UL) | `tddPattern` |
| SL time bitmap | `1|1|1|1|1|1|1|1|1|1` (all slots are SL) | `slBitMap` |
| Sub‑channel size | $K = 10$ RB | `slSubchannelSize` |
| Max reservations / SCI | 3 | `slMaxNumPerReserve` |
| Max Tx per PSSCH | 5 (blind re‑Tx enabled) | `slMaxTxTransNumPssch` |
| Reservation period | 20 ms | `reservationPeriod` |
| MCS | 14 (16‑QAM) | `mcs` |
| Tx power | 23 dBm | `tx_power` |
| RB overhead | 0.04 (NR default) | `NrGnbPhy::RbOverhead` |

### 2.1 Sub‑carrier spacing and slot duration

For numerology $\mu$, 3GPP TS 38.211 gives

$$\Delta f = 15 \cdot 2^{\mu}\ \text{kHz}, \qquad
T_\text{slot} = \frac{1\,\text{ms}}{2^{\mu}}.$$

With $\mu = 2$:

$$\Delta f = 15 \cdot 2^{2} = \boxed{60\ \text{kHz}}, \qquad
T_\text{slot} = \frac{1}{4} = \boxed{0.25\ \text{ms} = 250\ \mu s}.$$

A slot still carries 14 OFDM symbols, so each symbol lasts $\approx 250/14 = 17.9\ \mu s$.

### 2.2 Resource blocks and sub‑channels

One RB is 12 sub‑carriers, hence

$$W_\text{RB} = 12 \cdot \Delta f = 12 \cdot 60\ \text{kHz} = 720\ \text{kHz}.$$

The NR PHY converts channel bandwidth to RBs with a 4 % guard overhead
([`NrPhy::DoUpdateRbNum`](../src/nr/model/nr-phy.cc), `m_rbOh = 0.04`):

$$N_\text{RB} = \left\lfloor \frac{B_\text{ch}\,(1-\text{oh})}{W_\text{RB}} \right\rfloor
= \left\lfloor \frac{40\ \text{MHz}\cdot 0.96}{0.72\ \text{MHz}} \right\rfloor
= \left\lfloor \frac{38.4}{0.72} \right\rfloor = \left\lfloor 53.3 \right\rfloor = \boxed{53\ \text{RB}}.$$

The number of sub‑channels in the pool follows
([`NrSlUeMac::GetTotalSubCh`](../src/nr/model/nr-sl-ue-mac.cc)):

$$N_\text{sch} = \left\lfloor \frac{N_\text{RB}}{K} \right\rfloor
= \left\lfloor \frac{53}{10} \right\rfloor = \boxed{5\ \text{sub‑channels}}.$$

Each sub‑channel is $K\cdot W_\text{RB} = 10 \cdot 0.72 = 7.2$ MHz wide.

### 2.3 Slot availability under the TDD/SL bitmap

The TDD pattern is **all‑UL** and the SL time bitmap is **all‑ones**, so every
physical slot of the band is usable for sidelink. The slot supply rate is

$$R_\text{slot} = \frac{1}{T_\text{slot}} = \frac{1}{0.25\ \text{ms}} = \boxed{4000\ \text{slots/s}}.$$

The **total resource supply** is therefore

$$\Phi = N_\text{sch}\cdot R_\text{slot} = 5 \cdot 4000 = \boxed{20{,}000\ \text{sub‑channel‑slots per second}}.$$

This single number — 20 000 sub‑channel‑slots/s — is the capacity against which all
channel load is measured.

---

## 3. Definition of CBR used here

Per **TS 38.215 §5.1.30**, the CBR measured in slot $n$ on an NR‑V2X resource pool is

> the portion of sub‑channels in the resource pool whose **S‑RSSI** measured over a
> window $[\,n-100,\ n-1\,]$ slots exceeds a (pre‑)configured threshold.

Two consequences matter for the model:

1. **CBR is a fraction of sub‑channel occupancy**, not of raw time. The denominator
   is $N_\text{sch}$ sub‑channels over the 100‑slot window, i.e.
   $N_\text{sch}\cdot 100$ sub‑channel‑slots. With $\mu=2$ the window spans
   $100\cdot T_\text{slot} = 25$ ms.
2. **Only transmitters above the energy threshold count.** A far‑away vehicle whose
   received power is below threshold does not raise CBR. We capture this with $N$,
   the number of *in‑range* transmitters (those above the S‑RSSI threshold at the
   measuring vehicle).

This is exactly what the `MetricSupervisor` reproduces in simulation: it integrates
the per‑node `ChannelOccupied` durations of the NR spectrum PHY over a configurable
window and divides by the window length
([`MetricSupervisor::checkCBR`](../src/automotive/model/Measurements/MetricSupervisor.cc)).

---

## 4. Analytical occupancy model

Consider a CBR window of $W$ slots. The **supply** of resources in the window is

$$S = N_\text{sch}\cdot W \quad [\text{sub‑channel‑slots}].$$

Each in‑range vehicle generates messages at rate $r$ (Hz). Over the window of duration
$W\,T_\text{slot}$ seconds it emits, on average, $r\,W\,T_\text{slot}$ transport blocks;
each TB is sent $k$ times (one transmission plus $k-1$ blind re‑transmissions), and
each transmission occupies $b$ contiguous sub‑channels in one slot. The **demand**
from $N$ in‑range vehicles is

$$D = N \cdot r \cdot (W\,T_\text{slot}) \cdot k \cdot b \quad [\text{sub‑channel‑slots}].$$

The window length $W$ cancels, giving a compact closed form:

$$\boxed{\ \text{CBR} \;=\; \min\!\left(1,\ \frac{D}{S}\right)
\;=\; \min\!\left(1,\ \frac{N\, r\, k\, b\, T_\text{slot}}{N_\text{sch}}\right)\ }$$

Equivalently, in terms of the per‑second capacity $\Phi = N_\text{sch}/T_\text{slot}$:

$$\text{CBR} = \min\!\left(1,\ \frac{N\,r\,k\,b}{\Phi}\right).$$

> **Reading of the model.** CBR is the offered channel load normalised by capacity.
> It rises linearly with the number of in‑range transmitters $N$, the message rate
> $r$, the re‑transmission multiplicity $k$, and the per‑message sub‑channel footprint
> $b$; it falls with more sub‑channels $N_\text{sch}$ or shorter slots (larger $\mu$).
> The model is a **first‑order, collision‑agnostic upper bound**: it counts every
> transmitted resource as “busy” and ignores the small reduction from two vehicles
> picking the *same* resource (a collision occupies one sub‑channel‑slot, not two).
> Around the design point (CBR ≤ 0.8) collisions are rare under sensing, so the bound
> is tight to within a few percent.

---

## 5. Per‑message footprint $b$ at MCS 14

The footprint $b$ depends on the CAM size and on how many bits one sub‑channel carries.

At **MCS 14** (NR MCS index table 1, TS 38.214 Table 5.1.3.1‑1): modulation order
$Q_m = 4$ (16‑QAM), target code rate $R = 553/1024 = 0.540$, spectral efficiency
$\eta = 2.16$ bit/RE.

A 10‑RB sub‑channel over one slot offers, after removing the AGC symbol, the guard
symbol, the PSCCH region and PSSCH DM‑RS, roughly

$$N_\text{RE} \approx \underbrace{10\cdot 12}_{120\ \text{sc}} \times \underbrace{9}_{\text{PSSCH sym}} \times \underbrace{0.75}_{\text{DM‑RS/SCI‑2 loss}} \approx 810\ \text{RE}.$$

Hence one sub‑channel carries about

$$N_\text{RE}\cdot \eta \approx 810 \cdot 2.16 \approx 1750\ \text{bit} \approx 220\ \text{byte}.$$

Therefore:

| CAM type | Size | Footprint $b$ |
|----------|------|---------------|
| Plain CAM (no security) | ~150–220 B | **1 sub‑channel** |
| Secured CAM (ETSI 1609.2 / 103 097 envelope) | ~300–400 B | **2 sub‑channels** |

We adopt $b = 2$ (secured CAM) as the representative working point and report $b=1$
as a sensitivity case.

---

## 6. Worked example: density for CBR ≈ 80 %

**Representative operating point** (worst‑case CAM cadence, one blind re‑Tx, secured CAM):

$$r = 10\ \text{Hz}\ (T_\text{GenCam}=100\text{ ms}),\quad k = 2,\quad b = 2,\quad
N_\text{sch}=5,\quad T_\text{slot}=0.25\ \text{ms}.$$

Plugging into the boxed formula:

$$\text{CBR} = \frac{N\cdot 10 \cdot 2 \cdot 2 \cdot (0.25\times 10^{-3})}{5}
= \frac{N\cdot 0.01}{5} = 0.002\,N.$$

So each in‑range transmitter contributes **0.2 %** of CBR, and

$$\text{CBR} = 0.80 \;\Longleftrightarrow\; N = \frac{0.80}{0.002} = \boxed{400\ \text{vehicles in range}}.$$

Equivalently, the channel saturates (CBR = 1) at $N = 500$, so $80\,\%$ is exactly
$400/500$ of saturation — a deliberate safety margin below congestion.

### 6.1 From “vehicles in range” to vehicle density

The S‑RSSI sensing radius $R_\text{cs}$ at 23 dBm in the 5.9 GHz highway channel is
large because CBR is an *energy* measurement (it does not require successful
decoding). Taking a representative one‑sided sensing radius $R_\text{cs} \approx 1\
\text{km}$ on the straight 6‑lane motorway of the `highway` scenario
([`highway.net.xml`](../src/automotive/examples/highway/highway.net.xml): 5 km, 3+3
lanes, 3.2 m lane width), the in‑range set spans a road strip of length
$2R_\text{cs} = 2\ \text{km}$:

$$N = \rho_\ell \cdot 2R_\text{cs} \;\Rightarrow\;
\rho_\ell = \frac{N}{2R_\text{cs}} = \frac{400}{2\ \text{km}} = \boxed{200\ \text{veh/km}}
\;=\; \frac{200}{6} \approx \boxed{33\ \text{veh/km/lane}}.$$

Expressed as an **areal** density over the $19.2\,\text{m}$‑wide carriageway strip
($A = 2\,\text{km}\times 19.2\,\text{m} = 0.0384\ \text{km}^2$):

$$\rho_A = \frac{N}{A} = \frac{400}{0.0384\ \text{km}^2} \approx \boxed{1.0\times 10^{4}\ \text{veh/km}^2}.$$

> **Caveat on “veh/km²”.** A motorway is a thin strip, so the per‑km² figure is large
> and somewhat artificial; the physically meaningful densities are the **linear**
> ones (≈ 200 veh/km, ≈ 33 veh/km/lane), which correspond to heavy but realistic
> rush‑hour congestion at 120 km/h. The areal figure is reported only because the
> task asks for it and to make the strip‑geometry assumption explicit.

### 6.2 Cross‑check against the `highway` scenario

The shipped scenario inserts **698 vehicles** over 300 s at a 0.43 s headway on the
5 km road ([`highway700.rou.xml`](../src/automotive/examples/highway/highway700.rou.xml)).
With a ~150 s transit time at 33.3 m/s, the steady‑state population is on the order of
$300$–$500$ vehicles spread over 5 km, i.e. $\rho_\ell \approx 60$–$100$ veh/km
across all lanes. A vehicle near the centre of the road then sees on the order of
$N \approx \rho_\ell \cdot 2R_\text{cs} \approx 120$–$200$ in‑range transmitters,
predicting **CBR ≈ 25–40 %** at $r=10$ Hz — comfortably below the 0.8 congestion knee
and consistent with the design intent of leaving headroom for DENM bursts. Raising
the offered load (higher $r$, more re‑Tx $k$, or denser traffic) walks the channel up
toward the 80 % operating point analysed above.

---

## 7. Sensitivity and the rationale for each parameter

### 7.1 CBR sensitivity table

In‑range transmitters $N$ needed for a target CBR, under variations of $(r,k,b)$
(all with $N_\text{sch}=5$, $T_\text{slot}=0.25$ ms):

| $r$ [Hz] | $k$ | $b$ | per‑veh CBR | $N$ for 50 % | $N$ for 80 % | $N$ at saturation |
|---------:|----:|----:|------------:|-------------:|-------------:|------------------:|
| 10 | 2 | 2 | 0.20 % | 250 | **400** | 500 |
| 10 | 2 | 1 | 0.10 % | 500 | 800 | 1000 |
| 10 | 1 | 2 | 0.10 % | 500 | 800 | 1000 |
|  5 | 2 | 2 | 0.10 % | 500 | 800 | 1000 |
|  2 | 2 | 2 | 0.04 % | 1250 | 2000 | 2500 |
| 10 | 3 | 2 | 0.30 % | 167 | 267 | 333 |

The first row is the representative secured‑CAM, one‑re‑Tx case used in §6.

### 7.2 Why these values were chosen

- **Numerology $\mu = 2$ (60 kHz SCS, 0.25 ms slot).** Safety messaging is
  latency‑bound. A 0.25 ms slot makes the selection window $[T_1,T_2]$ (here
  $t_1=2,\ t_2=81$ slots) span only ~20 ms, keeping access delay well inside the
  100 ms CAM budget and the tens‑of‑ms DENM budget. A larger SCS also quadruples the
  slot supply versus $\mu=0$, **lowering CBR** for the same traffic — the model shows
  CBR $\propto T_\text{slot}$.
- **40 MHz channel → 5 sub‑channels of 10 RB.** 40 MHz is the canonical ITS NR‑V2X
  allocation. The sub‑channel size $K$ is a direct CBR lever: $N_\text{sch} =
  \lfloor N_\text{RB}/K\rfloor$, and CBR $\propto 1/N_\text{sch}$. $K=10$ RB yields
  $N_\text{sch}=5$, i.e. one secured CAM ($b=2$) consumes 40 % of the frequency grid
  in its slot — large enough that a handful of simultaneous transmitters fill the
  band (giving a meaningful CBR dynamic range to study), yet small enough that a CAM
  fits in 1–2 sub‑channels. Smaller $K$ (more, narrower sub‑channels) would dilute
  CBR and force multi‑sub‑channel CAMs; larger $K$ would coarsen the grid and inflate
  CBR.
- **All‑UL TDD pattern + all‑ones SL bitmap.** Mode‑2 sidelink reuses UL slots; making
  every slot available maximises $R_\text{slot}=4000$/s and removes DL/flexible‑slot
  puncturing from the CBR accounting, so the measured CBR reflects pure V2X load.
- **MCS 14 (16‑QAM, $\eta\approx2.16$).** A mid‑table MCS balances range against
  footprint: low enough to be decodable at the ~150 m PRR baseline used by the
  `MetricSupervisor` (`m_baseline_prr = 150 m`), high enough that a secured CAM fits
  in 2 sub‑channels rather than 3–4 (which would push the single‑vehicle CBR
  contribution from 0.2 % to 0.3–0.4 %, per the last row of the table).
- **Reservation period 20 ms, `slMaxNumPerReserve = 3`.** The SPS reservation period
  is the grant periodicity advertised in SCI‑1; it lets neighbours sense and avoid the
  reserved resource. It governs *collision avoidance*, not the offered load — the load
  is set by the CAM rate $r$. Keeping 20 ms (≤ the 100 ms CAM IPI) means a reservation
  comfortably covers each CAM while exposing the reservation early to sensing
  neighbours.
- **Blind re‑Tx (`slMaxTxTransNumPssch = 5`, $k\ge2$ effective).** Re‑transmissions
  trade channel load for reliability: each extra copy multiplies the per‑vehicle CBR
  contribution by $k$. This is the knob the DENM‑reliability study in
  [`docs/`](.) varies (see the multi‑copy DENM note), and it is why the headroom
  below 80 % matters — bursts of re‑transmitted DENMs must not tip the channel into
  congestion.
- **Tx power 23 dBm.** Sets the sensing radius $R_\text{cs}$ and thus $N$ for a given
  density. Higher power increases both communication range *and* the number of
  in‑range interferers, raising CBR — the §6.1 mapping is power‑dependent through
  $R_\text{cs}$.

---

## 8. Measuring CBR inside the simulator

The analytical CBR above can be validated at runtime. The `MetricSupervisor`
subscribes to each UE’s NR `ChannelOccupied` trace and forms an
exponential‑moving‑average CBR per node over a configurable window
([`MetricSupervisor::startCheckCBR` / `checkCBR`](../src/automotive/model/Measurements/MetricSupervisor.cc)).
The `v2v-emergencyVehicleAlert-nrv2x` example now enables this and prints the
network‑average CBR next to the average PRR at the end of the run, and appends it to
the cumulative results CSV (see the CBR‑output change in the example and
[`config.json`](../src/automotive/examples/config.json) keys `cbr_window_ms`,
`cbr_alpha`).

To compare model vs. simulation:

1. Set the traffic point ($r$ via `T_GenCam`, $k$ via re‑Tx, density via the `.rou`
   file) and read $N_\text{sch}=5$, $T_\text{slot}=0.25$ ms from this note.
2. Predict CBR $= 0.002\,N$ (representative point) for the number of in‑range
   vehicles $N$.
3. Run the scenario and read the printed **“Average CBR”**.

The two should agree to within the collision‑and‑sensing correction (a few percent)
up to the 0.8 knee; beyond it the analytical bound over‑predicts because the channel
saturates and the EMA flattens.

---

## 9. Summary

| Quantity | Value |
|----------|-------|
| SCS / slot | 60 kHz / 0.25 ms |
| Usable RBs | 53 |
| Sub‑channels $N_\text{sch}$ | 5 |
| Slot supply | 4000 slots/s |
| Capacity $\Phi$ | 20 000 sub‑ch‑slots/s |
| **CBR (closed form)** | $\min\!\big(1,\ N\,r\,k\,b\,T_\text{slot}/N_\text{sch}\big)$ |
| Representative per‑veh CBR | 0.2 % ($r{=}10,k{=}2,b{=}2$) |
| In‑range vehicles for **80 %** | **400** |
| Linear density for 80 % | ≈ 200 veh/km (≈ 33 veh/km/lane) |
| Areal density for 80 % | ≈ 1.0 × 10⁴ veh/km² (thin‑strip caveat) |

The parameter set places the design point safely below the congestion knee at
realistic motorway densities, with explicit, model‑traceable headroom for DENM
re‑transmission bursts.
