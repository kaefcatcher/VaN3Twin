# Campaign re-run instructions (operator)

This branch (`task/add_statistics`) lands the remediation fixes for the ethical
cooperative-braking V2X work. The code changes are committed, but the **figures
and summary CSVs must be regenerated on the build machine** (this repo's commit
cannot run ns-3/SUMO). Follow these steps in order.

## 0. Set aside the unrelated ASN.1 working-tree changes (if present)

The working tree may carry ~35 uncommitted ASN.1-regeneration edits
(`StationId`→`StationID`, `ETSI-ITS-CDD`→`ITS-Container` module swap) under
`src/automotive/model/ASN1/**` and `emulation-support/AMQP-client/C-ITS/ASN1/**`.
They are **unrelated** to this work and are likely build-breaking if half-applied.
Stash them before building:

```bash
git stash push -m "asn1-regen WIP" -- \
  src/automotive/model/ASN1 emulation-support/AMQP-client/C-ITS/ASN1
git stash list   # confirm it's stashed
```

(Do **not** commit them as part of the campaign re-run.)

## 1. Build + run the DENM gtest suite

```bash
./ns3 configure --enable-tests --enable-examples
./ns3 build
./ns3 run denm-gtest          # or: ./test.py -s denm-gtest
```

New/updated assertions to expect green:
- **Fix 1** harm-metric agreement (`HarmMetricTest.*`): optimizer objective ==
  logger metric for both `deltaV` and `kinetic_energy`.
- **Fix 4** `EthicalAlacarteRoundTrip.UperEncodeDecodePreservesEthicalFields`:
  the ethical alacarte extension survives a strict UPER encode→decode.
- **Fix 5/7/14** `CoopModelTest.*`: weights, σ-wait, coasting, and restitution
  measurably move the chosen deceleration.
- **Fix 10** `DenmBurst.*`: copy count == request and the burst window stays
  below the 500 ms update cadence for copies ∈ {1,2,3}.
- The pre-existing `OptimalDecel*` bounds were relaxed from `>= 0.1` to `>= 0.0`
  because Fix 7 now allows coasting (`a2 = 0`).

## 2. Run the campaign

```bash
./run_campaign.sh                 # generate configs + run + analyze
# or piecewise:
#   ./run_campaign.sh --skip-gen      reuse configs, run + analyze
#   ./run_campaign.sh --analyze-only  rebuild figures from existing results
```

The campaign banner now reports the **true** run count (`#configs × #seeds`)
read from each `manifest.csv`, not the old stale "10 runs"/"9 runs".

The default sweep uses 5 seeds (`scripts/gen_campaign.py:SEEDS`). For a fast
apples-to-apples sanity pass first, set `SEEDS = [1, 2, 3]` and run a Group-A
subset, then restore 5 seeds for the headline figures.

## 3. What to commit afterward

Stage **only** the regenerated artifacts, by explicit path:

```bash
git add figures/                                  # fig1–fig7 + pairwise_harm_heatmap_*.png
git add figures/campaign_summary.csv figures/campaign_summary_agg.csv
git add sweep_configs/                            # only if you re-ran gen_campaign
git status                                         # verify the staged set
git commit -m "Regenerate campaign figures + summary CSVs after remediation"
```

The summary CSVs (`campaign_summary.csv`, `campaign_summary_agg.csv`) are the
compact, regenerable evidence behind the tables/figures — commit them so the
headline can be rebuilt without re-running the sims (Fix 8). The stale
`moscow_large` heatmaps were already deleted in the code commit.

## 4. What NOT to commit

- `presentation_plan.md`, `remediation_plan.md` (planning docs — keep untracked).
- The ASN.1 changes from step 0 (`git stash pop` them back into your working
  tree afterward if you still need them, but keep them out of this campaign
  commit).
- Anything under `docs/*` that is unrelated WIP.

## Fix status: fully implemented vs. stubbed-with-knobs

**Fully implemented (code + settings + tests where applicable):**
- Fix 1 — single canonical harm metric (`HarmMetric` attr / `harm_metric` key,
  default `deltaV`); optimizer, `HarmLogger`, and per-message harm all route
  through `appUtil_harmFromVrel`.
- Fix 2 — force-brake the originator only; no `setMaxSpeed`, no follower
  slowDown. `force_brake_count` (default 1) selects how many platoon leaders.
- Fix 3 — `HarmLogger` gates pairs on same-direction + closing geometry;
  radius and heading tolerance are configurable.
- Fix 4 — `include_ethical_alacarte` is turned **on** for algo runs by the
  generator; the receiver is driven from the transmitted mass/decel; UPER
  round-trip is unit-tested.
- Fix 5 — per-vehicle mass from the SUMO `mass` vType param (app + HarmLogger);
  ethical weights `EthicalWeightLead`/`EthicalWeightFollow` applied as Σ Wᵢ Hᵢ.
- Fix 6 — `suboptimalDecel` is now the explicit σ-timeout fallback when no
  follower state is available; its closed form accounts for the σ-coast.
- Fix 7 — optimizer advances leader/ego state by σ before scoring; sweep starts
  at `a2 = 0` (coasting allowed).
- Fix 8 — stale `moscow_large` heatmaps deleted; summary CSVs to be committed
  here (this step).
- Fix 10 — DENM burst geometry factored into pure, unit-tested helpers.
- Fix 11 — single scenario registry in `gen_campaign.SCENARIOS`; the analyzer
  derives order/labels from `manifest.csv`.
- Fix 12 — figures iterate manifest scenarios/axes; fig4 plots any CBR-swept
  family; an explicit warning prints for any scenario in no figure.
- Fix 13 — forced-brake set derived from the platoon via `force_brake_count`; a
  `organic_basic` template scenario fires the algorithm with no forced brake;
  `run_campaign.sh` reports the true run count. (See "new scenario in 3 steps".)
- Fix 14 (items 1–3) — coefficient of restitution `Restitution` drives
  post-collision velocities and the chain's **second** collision in `H_total`; a
  `SigmaMode="linkquality"` mode derives σ from `LinkPacketErrorRate` /
  `LinkSigmaGain`.

**Stubbed with knobs + TODO (scoped extensions, not yet modeled):**
- Fix 14 item 4 — `DENMMrcCombining` flag exposed; MRC/diversity-combining of
  DENM copies is documented as a TODO, not implemented.
- Fix 14 item 5 — `HeterogeneousTraffic` flag exposed; cross/oncoming-traffic
  role logic is a TODO. The `organic_basic` scenario is single-direction; add an
  opposite-direction route to exercise it once the role logic lands.

## New scenario in 3 steps

1. Drop the SUMO files (`*.net.xml`, `*.rou.xml`, `*.sumocfg`) under
   `src/automotive/examples/<your_scenario>/`.
2. Add one entry to `scripts/gen_campaign.py:SCENARIOS` with `sumo_folder` /
   `mob_trace` / `sumo_config` plus the presentation metadata `order`, `label`,
   `cbr_level`, `axis` (use `force_brake_count: 0` for an organic hazard).
3. `./run_campaign.sh` — it flows generator → manifest → figures with **no**
   analyzer edit (Fix 11/12). Confirm it appears in fig1/2/3/7 (and fig4 if you
   tagged it `axis: cbr_sweep`).
