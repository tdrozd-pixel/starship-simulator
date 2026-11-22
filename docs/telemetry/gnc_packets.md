# GNC telemetry dictionary, version 1

The packets the flight computer downlinks for the GNC console. This is the contract between `flight_sw` (which builds the packets), the comms model, `sim_server` (which forwards them unchanged) and the console (which decodes them). It is the seed of the future XTCE database (build order item 48).

Everything here is something the vehicle knows: flight software state, its estimates, its commands and raw sensor readings. Never truth.

## Packet format

CCSDS Space Packet (CCSDS 133.0-B-2), telemetry, **big-endian** (network byte order).

### Primary header (6 bytes)

| Bits | Field | Value |
|---|---|---|
| 3 | Packet version number | 0 |
| 1 | Packet type | 0 (telemetry) |
| 1 | Secondary header flag | 1 |
| 11 | APID | see the table below |
| 2 | Sequence flags | 0b11 (unsegmented) |
| 14 | Packet sequence count | per APID, starts at 0, wraps 16383 -> 0 |
| 16 | Packet data length | (secondary header + user data length in bytes) - 1 |

### Secondary header (6 bytes): CUC time code

| Bytes | Field |
|---|---|
| 4 | coarse time: whole seconds of flight computer time (u32) |
| 2 | fine time: fraction of a second in units of 1/65536 s (u16) |

Flight computer time is `Hal::timeSec()`, seconds since the flight computer started (the sim start). Onboard time = coarse + fine / 65536.

### User data

Fields follow in the order listed, with no implicit padding (explicit `pad` bytes are zero). Types: `u8`, `u16`, `u32`, `f32` (IEEE 754 single), `f64` (IEEE 754 double), all big-endian. Angles in radians, SI units.

Total packet length = 12 + user data length. Every packet is at most 256 bytes.

## APIDs

| APID | Name | Rate | User data length |
|---|---|---|---|
| 0x100 (256) | FSW status | 10 Hz | 20 |
| 0x101 (257) | Mission config | 1 Hz | 36 |
| 0x110 (272) | Navigation | 50 Hz | 80 |
| 0x120 (288) | Guidance and control | 50 Hz | 84 |
| 0x130 (304) | Actuator feedback | 50 Hz | 36 |
| 0x140 (320) | GNSS | each new fix (10 Hz) | 56 |

Rates are of flight computer cycles at 100 Hz: 50 Hz packets go on even cycles, 10 Hz on every 10th cycle, 1 Hz on every 100th, starting with cycle 0. The GNSS packet goes on the first cycle that sees a new receiver solution.

### 0x100 FSW status

| Offset | Type | Field | Notes |
|---:|---|---|---|
| 0 | u8 | mode | `fsw::HopMode`: 0 Prelaunch, 1 Ignition, 2 Ascent, 3 Hover, 4 Descent, 5 FinalDescent, 6 Landed |
| 1 | u8 | flags | bit 0 engines on, bit 1 horizontal nav valid, bit 2 translating to target, bit 3 aligned over target, bit 4 GNSS ready (fix count >= required) |
| 2 | u16 | gnssFixCount | saturates at 65535 |
| 4 | u32 | cycleCount | flight computer cycles since start |
| 8 | f32 | timeInMode | [s] |
| 12 | f32 | propellantEstimate | [kg] |
| 16 | f32 | massEstimate | [kg] |

### 0x101 Mission config

| Offset | Type | Field | Notes |
|---:|---|---|---|
| 0 | f64 | targetLatitude | [rad] WGS-84 geodetic |
| 8 | f64 | targetLongitude | [rad] east positive |
| 16 | f64 | targetHeight | [m] above the WGS-84 ellipsoid |
| 24 | f32 | targetAltitude | [m] hop altitude |
| 28 | f32 | landingAlignTolerance | [m] |
| 32 | f32 | translateAltitude | [m] |

### 0x110 Navigation

Estimates are the flight software's; "measured" values are the latest raw sensor readings.

| Offset | Type | Field | Notes |
|---:|---|---|---|
| 0 | f32[4] | attitude w, x, y, z | estimated, body to local ENU |
| 16 | f32[3] | angularRate x, y, z | measured (IMU), body [rad/s] |
| 28 | f32[3] | specificForce x, y, z | measured (IMU), body [m/s^2] |
| 40 | f32 | altitude | estimated, base above ground [m] |
| 44 | f32 | verticalVelocity | estimated [m/s] |
| 48 | f32 | altimeterRange | measured slant range [m] |
| 52 | u8 | altimeterValid | 0 or 1 |
| 53 | u8[3] | pad | 0 |
| 56 | f32 | east | estimated, IMU point in the target's ENU frame [m] |
| 60 | f32 | north | [m] |
| 64 | f32 | velocityEast | [m/s] |
| 68 | f32 | velocityNorth | [m/s] |
| 72 | f32 | accelBiasEast | estimated apparent acceleration bias [m/s^2] |
| 76 | f32 | accelBiasNorth | [m/s^2] |

### 0x120 Guidance and control

| Offset | Type | Field | Notes |
|---:|---|---|---|
| 0 | f32 | altitudeSetpoint | [m] |
| 4 | f32 | velocitySetpoint | vertical [m/s] |
| 8 | f32 | eastSetpoint | horizontal target, target frame [m] |
| 12 | f32 | northSetpoint | [m] |
| 16 | f32 | eastAccelCommand | [m/s^2] |
| 20 | f32 | northAccelCommand | [m/s^2] |
| 24 | f32[3] | attitudeError x, y, z | body [rad] |
| 36 | engine[3] | per engine, 16 bytes each | engine i at offset 36 + 16 i |

Engine record (16 bytes): `u8 on`, `u8[3] pad`, `f32 throttleCommand` (fraction of rated flow), `f32 gimbalPitchCommand` [rad], `f32 gimbalYawCommand` [rad].

### 0x130 Actuator feedback

Measured gimbal positions from the gimbal position sensors (`GimbalFeedbackModel`).

| Offset | Type | Field |
|---:|---|---|
| 0 | engine[3] | per engine, 12 bytes each, engine i at offset 12 i |

Engine record (12 bytes): `u8 valid`, `u8[3] pad`, `f32 gimbalPitch` [rad], `f32 gimbalYaw` [rad].

### 0x140 GNSS

The latest receiver solution as the flight computer received it.

| Offset | Type | Field | Notes |
|---:|---|---|---|
| 0 | u8 | fixType | `fsw::GnssFixType`: 0 none, 1 standalone, 2 RTK fixed |
| 1 | u8 | satellitesUsed | |
| 2 | u8 | satellitesVisible | |
| 3 | u8 | pad | 0 |
| 4 | f64 | fixTime | time of validity, flight computer time [s] |
| 12 | f64 | latitude | [rad] WGS-84 geodetic, antenna |
| 20 | f64 | longitude | [rad] |
| 28 | f64 | height | [m] ellipsoidal |
| 36 | f32[3] | velocity east, north, up | at the antenna [m/s] |
| 48 | f32 | horizontalSigma | receiver-reported 1 sigma [m] |
| 52 | f32 | verticalSigma | [m] |

## Vehicle constants the console needs

Not downlinked (they are configuration, like an XTCE calibration): engine i sits at azimuth `90 deg + 120 deg * i` from body +x toward +y, 1.55 m from the axis; gimbal limit 15 deg. From `sim_vehicle/include/sim/vehicle/HopperParams.hpp` (estimates).
