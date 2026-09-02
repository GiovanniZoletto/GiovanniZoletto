# MSL-Like Mars Entry Model

This folder contains a fresh MATLAB implementation built around an **MSL / Mars 2020-like** entry vehicle, using public mission heritage where available and explicit engineering estimates where public data are incomplete.

## Fixed From Literature

- Mars reference radius `R_Mars = 3389.5 km`
  Source: [NASA Mars Facts](https://science.nasa.gov/mars/facts/)

- Mars gravitational parameter `mu_Mars = 4.2828e13 m^3/s^2`
  Source: [NASA Mars Facts](https://science.nasa.gov/mars/facts/)

- Mars atmosphere curve-fit form
  Source: [NASA Glenn Mars Atmosphere Model](https://www1.grc.nasa.gov/beginners-guide-to-aeronautics/mars-atmosphere-equation-metric/)

- MSL aeroshell diameter `D = 4.5 m`
  Source: [MSL Aerodynamic Challenges](https://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/20090025343.pdf)

- MSL hypersonic `L/D ≈ 0.24`
  Source: [MSL Aerodynamic Challenges](https://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/20090025343.pdf)

- MSL entry mass `m = 2804 kg`
  Source: comparison tables cited in [MSL EDL performance](https://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/20070016022.pdf) and related NASA tables

- Parachute diameter `21.5 m`
  Source: [NASA Perseverance parachute deploy page](https://www.nasa.gov/image-article/perseverance-deploys-its-parachute/)

- Parachute deploy condition reference `Mach ≈ 2.1`, altitude around `11 km`
  Source basis: [NASA Perseverance parachute deploy page](https://www.nasa.gov/image-article/perseverance-deploys-its-parachute/) and MSL heritage summaries

- PICA nominal density about `0.27 g/cm^3`
  Source: [NASA Low Density Ablators / PICA](https://www.nasa.gov/general/thermal-protection-materials-branch-low-density-ablators/)

## Estimated For Preliminary Modeling

- Nose radius `R_n = 1.125 m`
  Rationale: blunt MSL-like forebody estimate, set explicitly so heating depends on geometry.

- Initial entry state `v0 = 5600 m/s`, `gamma0 = -11 deg`, `h0 = 125 km`
  Rationale: reasonable MSL-like entry setup for a 3DOF educational/research model.

- Aerodynamic tables `Cd(M,alpha)` and `Cl(M,alpha)`
  Rationale: public NASA papers describe trends and performance targets, but do not publish the full flight database in a convenient table. The code uses **surrogate tables** consistent with:
  - 70 deg sphere-cone behavior
  - positive lift at nonzero alpha
  - hypersonic `L/D ≈ 0.24`
  Heritage basis:
  - [MSL Aerodynamic Challenges](https://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/20090025343.pdf)
  - [Phoenix Capsule Aerodynamics](https://ntrs.nasa.gov/archive/nasa/casi.ntrs.nasa.gov/20080034648.pdf)

- Prescribed alpha schedule `alpha(M)`
  Rationale: surrogate for active guidance/attitude control without adding 6DOF rotational dynamics.

- TPS properties `k`, `cp`, `epsilon`, `H_eff`, `T_abl_start`
  Rationale: public NASA pages clearly identify PICA heritage and density, but a compact open-source material card suitable for direct MATLAB use is not easily available in one place. These values are therefore preliminary **PICA-like engineering estimates**.

- Initial TPS thickness `thickness0 = 60 mm`
  Rationale: configurable first guess, left easy to replace with mission-specific data later.

- Equivalent inner-wall thermal mass
  Rationale: detailed wall-stack data are not public; the model uses an effective inner wall to predict `T_inner_wall`.

## Parameters Left Easy To Change

- vehicle mass
- vehicle diameter
- nose radius
- aerodynamic tables
- alpha schedule
- TPS thickness
- TPS material properties
- inner wall equivalent properties
- thermal limit `T_inner_wall,max`
- parachute parameters

## Physical Meaning Of `alpha(M)` In This Model

The code does **not** assume that Mach directly causes alpha to change by itself.

Instead:

- `gamma` is the direction of the velocity vector
- `alpha` is the angle between vehicle axis and freestream
- `alpha(M)` is a **prescribed control schedule**, meaning the vehicle is assumed to actively hold different attitudes in different Mach regimes

So here `alpha` changes with Mach because **the control system is assumed to command a different attitude as the vehicle slows down**.

A more complete model would compute `alpha` from rotational dynamics and pitching moment equilibrium:

- passive trim: `Cm(M, alpha) = 0`
- active control: `I * dω/dt = M_aero + M_RCS`

That higher-fidelity approach was intentionally not added yet to keep the model manageable.








Temperatura max 310 K
alpha max 5°
temperature ideali degli astronauti verso marte
Biocapsula su marte
