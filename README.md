# Fuzzy-Adaptive Optimization for Economic Load Dispatch

This experiment studies economic load dispatch (ELD) on a six-unit thermal-generator system. It compares a standard genetic algorithm (GA), a Mamdani fuzzy-adaptive GA, and particle swarm optimization (PSO) with a fuzzy supervisor. The objective is to minimize fuel cost while meeting demand plus transmission losses and respecting generator constraints.

## Method

- Classical lambda iteration provides a reference for the convex case; `fmincon` independently checks that solution.
- Non-convex cases add ramp-rate limits and prohibited operating zones. An exhaustive search over convex sub-regions supplies the reference for those cases.
- Fuzzy supervisors adapt GA crossover/mutation behavior and PSO inertia based on population or swarm diversity and stagnation.
- Each of three methods is evaluated over 30 seeded runs, with a population of 30 and 200 generations. The script also evaluates parameter sensitivity and a load sweep.

## Case 3 results

Case 3 uses an 800 MW load with ramp-rate and prohibited-zone constraints. The reference optimum recorded by the experiment is **$9,539.7564/h**.

| Method | Best cost ($/h) | Mean cost ($/h) | Runs within $1/h of reference |
| --- | ---: | ---: | ---: |
| Standard GA | 9,539.7728 | 9,543.6703 | 15 / 30 |
| Fuzzy-adaptive GA | 9,539.7594 | 9,539.8512 | 30 / 30 |
| Fuzzy PSO | 9,539.7566 | 9,542.9771 | 17 / 30 |

The fuzzy-adaptive GA had the most consistent Case 3 results in this set of runs. This is a stochastic comparison over the recorded seeds and settings, not a guarantee for other parameter choices or systems.

## Requirements and run

Requires MATLAB with Optimization Toolbox and Fuzzy Logic Toolbox. The Simulink model builder also requires Simulink.

From the repository root, run:

```matlab
Exp6_FuzzyGA_ELD
build_Exp6_Simulink
```

The first script runs the optimization study and refreshes the result files and MATLAB figures. The second builds and simulates `Exp6_FuzzyGA_ELD_Sim.slx` using the saved study results, and exports the Simulink figures.

## Repository contents

- `Exp6_FuzzyGA_ELD.m` — experiment driver and statistical comparison.
- `eld6_data.m`, `eld_problem.m`, `eld_evaluate.m`, `eld_repair.m`, `lambda_dispatch.m`, `poz_exact.m` — system data, constraint handling, objective evaluation, and reference solvers.
- `build_fis.m`, `run_ga.m`, `run_fpso.m`, `ga_generation.m`, `pop_diversity.m` — fuzzy supervisors and optimization routines.
- `build_Exp6_Simulink.m`, `Exp6_FuzzyGA_ELD_Sim.slx` — Simulink model build and model.
- `Exp6_results.mat`, `matlab_results.json`, `simulink_results.json` — saved results.
- `figures_matlab/`, `figures_report/`, `figures_simulink/` — analysis, report, and model figures.

## Scope

The findings apply to the included IEEE six-unit test system, constraints, and algorithm settings. No software license is specified in this repository.
