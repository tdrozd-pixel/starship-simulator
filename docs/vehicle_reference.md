# Vehicle reference: Starship V3

Public information as of Nov 2026. SpaceX internal data is not public; anything not from a public source is an estimate. Every vehicle number used in code lives in a parameters file with a comment giving its source (link, estimate, or derived).

- **Super Heavy:** 33 Raptor 3 engines; three grid fins (down from four on V2), each about 50% larger, mounted lower to reduce hot-staging heat, with actuators inside the fuel tank and catch points for the tower.
- **Ship:** six Raptor 3 engines (three sea-level, three vacuum), four flaps, hexagonal tile heat shield.
- **Integrated hot staging:** no disposable interstage; the Booster forward dome takes the Ship's plume directly (protected by tank pressure and steel shielding); interstage actuators retract after separation.
- **Fuel transfer tube:** redesigned so all 33 engines can light simultaneously.
- **Pad:** two separate quick disconnect systems instead of one.

## Numbers used by the sim

The hop configuration (Ship on its three sea-level Raptor 3s, partial propellant load) and every number it uses, with sources and estimates marked, are in `sim_vehicle/include/sim/vehicle/HopperParams.hpp`. Highlights:

- Raptor 3: 280 tf sea-level thrust and 350 s Isp from SpaceX's Aug 2024 performance card (SpaceX later said the 350 s is the vacuum Isp of the sea-level engine; some pages list 250 tf). Throttle 40 to 100%.
- Ship: about 52 m long, 9 m diameter, 1600 t propellant capacity. Dry mass is not published; the sim uses 150 t (estimates range from about 100 t to 166 t).
- Landing target: Starbase Pad 2 at 25.996603 N, 97.158251 W (Launch Library 2, third-party), placeholder for the tower catch point.
- Navigation: public sources describe GNSS plus inertial navigation, with tower-side radar reported for the catch. RTK-grade GPS is reported by one unverified source; the sim defaults to RTK and has a standalone preset.
