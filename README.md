# phoenix-flight-computer
Flight computer firmware for Rocket Propulsion Lab's flagship liquid-propellant rocket **Phoenix**.

This system:
* Receives valve state commands from [Phoenix Ground Station](https://github.com/rocketproplab/phoenix-gs) over raw Ethernet 
* Actuates flow/pressurant/vent valves according to received commands.
* Samples pressure transducers (and other sensors in a modular fashion)
* Streams telemetry frames back to the ground over raw Ethernet.
* Is structured to allow easy/clean addition of new sensors and actuators as self-contained modules.

## Repo Structure 
```text
phoenix-flight-computer/
├── phoenix-flight-computer.ino     # Main entry: setup(), loop(), Ethernet RX/TX, module orchestration
├── src/
│   ├── w5500/                      # MACRAW driver wrapper for Wiznet W5500
│   ├── valve/                      # Valve abstraction & state machine
│   └── pt/                         # Pressure transducer abstraction & configuration
├── docs/
│   └── diagrams/                   # Valve / plumbing / system diagrams
├── LICENSE
└── README.md
```

## High Level Operation
On boot:
* Initialize serial for debugging (disabled for deployment).
* Initialize W5500 Ethernet Chip in MACRAW mode with according MAC address.
* Initialize all valves.
* Initialize all pressure transducers
On loop:
* Read Ethernet frames and update valve target state.
* Apply valve target state to valves.
* Aggregate sensor readings and transmit telemetry frame to ground station.

## Ethernet Communication Protocol
The Phoenix FC and ground station communicate using custom raw Ethernet frames over the W5500.

### MAC Addressing
Each device is assigned a unique unicast MAC:
```c
// Ground station
const uint8_t MAC_GROUND_STATION[6] = {0x02, 0x47, 0x53, 0x00, 0x00, 0x01}; // "GS"
// Relief valve controller (separate from main flight computer)
const uint8_t MAC_RELIEF_VALVE[6]   = {0x02, 0x52, 0x56, 0x00, 0x00, 0x02}; // "RV"
// Phoenix sensor/flight copmuter (this repo)
const uint8_t MAC_SENSOR_GIGA[6]    = {0x02, 0x53, 0x49, 0x00, 0x00, 0x04}; // "SI, Sensor Interface"
```
### Frame Type
There are two custom EtherTypes:
* Command frames (GS to SI/RV): `0x63e4`
* Telemetry frames (SI to GS): `0x8889`
### Generic Frame Layout
```text
+------------+------------+-----------+-----------------+
| Dst MAC(6) | Src MAC(6) | Type (2)  | Payload (N ...) |
+------------+------------+-----------+-----------------+
```

### Valve Command Frames (GS to SI)
**EtherType**: `0x63e4`

**Payload format**:

![My Image](docs/diagrams/valve_fmt.drawio.svg)

The mapping from bits to physical valves is defined in `src/valve/valve.h` via named masks.

### Telemetry Frames (SI to GS)
**EtherType**: `0x8889`

Telemetry is currently serialized as simple CSV ASCII string in payload.

**Payload format**:
```text
Payload: "PT0,PT1,...,PTn,LC1,LC2,LC3,TC1,TC2\r\n"
```
Where:
* `PT[i]` = engineering-value pressure in PSI
* `LC[i]` = load cell readings (currently set to 0 as placeholder)
* `TC[i]` = thermocouple readings (currently set to 0 as placeholder)

## Valve System
Valve control is encapsulated in `src/valve/`. Conceptually:
* Physical valves are abstracted to single bit in state.
* Module is responsible for enforcing proper voltage to said valves.

![Valve Diagram](docs/diagrams/valve.drawio.svg)

## Pressure Transducer System
Pressure transducers are encapsulated in `src/pt/`. Conceptually:
* Each PT has a unique engineering pressure range and output voltage range.
* The module holds static configurations of all PTs.
To add/modify existing PTs, edit the configuration in `src/pt/` and the main flight software loop automatically aggregates all readings.

## Launch Control - Dev Mode (Risk: High)
* This mode is intended for testing valve openings before fueling
* All valves are free to be switched on and off
* Be very sure of what you are doing when using this mode

![My Image](./docs/diagrams/phoenix-fc-launch-control-dev.svg)

## Launch Control - Fueling Mode (Risk: Medium)
* In fueling mode, all flow valves are locked in OFF mode
* all vent valves can be controlled by individual switch

![My Image](./docs/diagrams/phoenix-fc-launch-control-fueling.svg)

## Launch Control  - Launch Mode (Risk: Low)

### Buttons/Switches
* Abort (button): has highest priority, open vent valves for all
* Arm (switch): open/close pressurant valves
* Launch (button): open flow valves

![My Image](./docs/diagrams/phoenix-fc-launch-control-launch.svg)

# Physical Specification
## MCU
Arduino Mega 2560
## Ethernet
[W5500 Ethernet LAN Network Module](https://www.amazon.com/HiLetgo-Ethernet-Network-Support-Microcontroller/dp/B0CDWX9VQ5)