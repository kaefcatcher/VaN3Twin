# NR-V2X simulation campaign + academic analysis

One command runs the full set of "main" cooperative-braking simulations and produces a
presentation-ready figure set:

```bash
./run_campaign.sh
```

It (1) generates the highway density route files and all campaign configs, (2) runs the
two campaign groups via the existing `run_sweep.sh`, and (3) aggregates everything into
`figures/` (+ `campaign_summary.csv`) via `scripts/campaign_analyze.py`.

> **Prerequisites:** ns-3 built (`./ns3 configure && ./ns3 build`) and SUMO installed.
> Plots need `matplotlib` (`pip install matplotlib`); without it only the summary CSV is
> written. **This depends on the metrics added in `MetricSupervisor` / `NrUeMac` / the
> example** (CBR output, per-vehicle/per-message-type PRR, `SlFixedReselectionCounter`,
> `denm_copies`) — build first and run the smoke test below.

## What it runs (Structured / OFAT, multi-seed → 28 configs × seeds)

Every config is run once per **seed** in `SEEDS` (default `[1,2,3,4,5]` in
`scripts/gen_campaign.py`) — each seed sets a distinct `seed` that drives **both** the SUMO
mobility RNG and the ns-3 radio RNG, giving independent replications for box plots / error
bars. With 5 seeds: **28 configs → 140 runs**. `SEEDS` is the runtime multiplier; drop to
`[1,2,3]` for a faster first pass.

**Group A — scenario study** (`sweep_configs/campaign_main/`, 10 configs × seeds) — each
scenario with the algorithm OFF (`cooperative_detection=false`) and ON (`true`,
`sigma_mode=computed`), all at one baseline network point (SPS, pKeep 0, 1 DENM copy):

| scenario | map / route |
|----------|-------------|
| `basic` | `sumo_files_v2v_cooperative/cooperative.rou.xml` |
| `highway_low/mid/high` | generated `highway/highway_{low,mid,high}.rou.xml` |
| `moscow_large` | `moscow/routes1000.rou.xml` (both algo states — fixed: the old configs compared routes100 vs routes1000) |

**Group B — network study** (`sweep_configs/campaign_network/`, 18 configs × seeds) —
`highway_mid`, **both algo OFF and ON** (so the algorithm's benefit is measured under every
network setting), varying one axis at a time:
- scheduling / pKeep: `dynamic` (reselection counter 1) and SPS at pKeep 0.0/0.2/0.4/0.6/0.8;
- DENM copies: 1, 2, 3.

## Figures (`figures/`)

| file | shows |
|------|-------|
| `fig1_harm_by_scenario` | **box plots** of integrated HARM, no-algo vs algo, per scenario — headline |
| `fig2_harm_reduction_pct` | box plots of per-seed % HARM reduction the algorithm achieves |
| `fig3_network_metrics_by_scenario` | PRR / DENM-PRR / CBR per scenario (mean±std), algo vs no-algo |
| `fig4_harm_vs_cbr` | HARM and DENM-PRR vs **measured CBR** (highway low/mid/high), mean±std |
| `fig5_sched_pkeep` | PRR / CBR / HARM vs pKeep, **dynamic vs SPS, algo vs no-algo** |
| `fig6_denm_copies` | DENM-PRR and HARM vs number of DENM copies, algo vs no-algo |
| `fig7_harm_timeseries` | HARM(t), all seeds overlaid, algo vs no-algo, per scenario |
| `pairwise_harm_heatmap_*` | per-vehicle-pair HARM heatmap for selected runs |
| `campaign_summary.csv` | every run × every metric (one row per seed) |
| `campaign_summary_agg.csv` | per-config mean/std/n for every metric (for tables) |

Each figure is written as both PNG and PDF (vector, for slides/papers). Box plots / error
bars come from the multiple seeds; for more seeds (tighter boxes) bump `SEEDS` in
`scripts/gen_campaign.py`.

## Smoke test + tuning (do once, after the first build)

1. Run a single config and confirm the new outputs appear:
   ```bash
   cp sweep_configs/campaign_main/highway_mid_algo.json src/automotive/examples/config.json
   ./ns3 run v2v-emergencyVehicleAlert-nrv2x
   ```
   Expect `Average CBR: …`, a `Per-vehicle, per-message-type PRR:` block, and
   `run_prr_per_vehicle_messagetype.csv`.
2. **CBR levels** are nominal. Read the printed `Average CBR` for `highway_low/mid/high`
   and, if the spread isn't what you want, edit the insertion periods in
   `scripts/gen_highway_density.py` (`DENSITY_LEVELS`) and regenerate.
3. **Brake must fire** for the algorithm to show a benefit (the cooperative algorithm only
   acts on a hard-brake DENM). The forced brake is **position based**
   (`force_brake_position`; `force_brake_time` is ignored by the example). Highway brakes the
   lead platoon at 800 m with `simTime=40 s`; basic/moscow brake at 50 m. If HARM comes out
   identical with/without algo, the brake didn't fire — lower `force_brake_position` or raise
   `simTime` (knobs in `scripts/gen_campaign.py` / `gen_highway_density.py`).

## Re-running just the analysis

```bash
./run_campaign.sh --analyze-only     # rebuild figures from existing results/
./run_campaign.sh --skip-gen         # reuse configs, re-run sims + analyze
```
