# Bill of Material

## Main Components

| Qty       | Component              | Description                               | Link                                                                                                       |
| --------- | ---------------------- | ----------------------------------------- | ---------------------------------------------------------------------------------------------------------- |
| 16        | FeeTech FT90M Servo    | 9G servo motor, PWM                       | [Alibaba](https://www.alibaba.com/product-detail/Feetech-FT90M-Metal-Gear-Digital-Core_1601043147164.html) |
| 1         | Pimoroni Servo2040     | RP2040 Servo Shell                        | [Adafruit](https://www.adafruit.com/product/5437)                                                          |
| 1         | 6V 5A Power Supply     | Alternatively use adjustable power supply | –                                                                                                          |
| As needed | DuraBraid fishing wire | 40lb                                      | [Amazon](https://www.amazon.com/SpiderWire-DuraBraid-Braid-Fishing-Line/dp/B0C69TLSHK)                     |
| 10        | Compression spring     |                                           | [McMaster](https://www.mcmaster.com/2006N111/)                                                             |
| As needed | 2mm zipties            |                                           | –                                                                                                          |
| As needed | 3mm OD PTFE Tube       |                                           | [Amazon](https://www.amazon.com/uxcell-Tubing-Printer-RepRap-16-4ft/dp/B07F64KQR1)                         |

*PWM Version, use SCS0009 and ESP32 from `V3` for SCS Bus builds.    
## Fasteners

### Shoulder Screws

| Qty | Specification                                                                                                   | Link                                            |
| --- | --------------------------------------------------------------------------------------------------------------- | ----------------------------------------------- |
| 15  | 3 mm Shoulder Diameter, 10 mm Shoulder Length, M2 x 0.4 mm Thread, Metric Low-Profile Precision Shoulder Screws | [McMaster](https://www.mcmaster.com/90323A608/) |
| 5   | 3 mm Shoulder Diameter, 6 mm Shoulder Length, M2 x 0.4 mm Thread, Metric Low-Profile Precision Shoulder Screws  | [McMaster](https://www.mcmaster.com/90323A605/) |
| 3   | 3 mm Shoulder Diameter, 5 mm Shoulder Length, M2 x 0.4 mm Thread, Metric Low-Profile Precision Shoulder Screws  | [McMaster](https://www.mcmaster.com/90323A604/) |
| 1   | 3 mm Shoulder Diameter, 6 mm Shoulder Length, M2 x 0.4 mm Thread, Metric Low-Profile Precision Shoulder Screws  | [McMaster](https://www.mcmaster.com/90323A612/) |
| 10  | 2.5 mm Shoulder Diameter, 2 mm Shoulder Length, M2 x 0.4 mm Thread, Slotted Precision Shoulder Screws           | [McMaster](https://www.mcmaster.com/97307A111/) |


### SHCS

| Qty | Specification               |
| --- | --------------------------- |
| 60  | M1.6 x 0.35 mm Thread, 4mm  |
| 40  | M2 x 0.4 mm Thread, 8mm     |
| 20  | M2 x 0.4 mm Thread, 12mm    |
| 10  | M2.5 x 0.45 mm Thread, 25mm |
| 4   | M3 x 0.5 mm Thread, 16mm    |

### Flat Head Screws

| Qty | Specification    |
| --- | ---------------- |
| 28  | M1.6 x 0.35, 2mm |

### Bearings

| Qty | Specification                                                                                             | Link                                           |
| --- | --------------------------------------------------------------------------------------------------------- | ---------------------------------------------- |
| 26  | Light Duty Dry-Running Flanged Sleeve Bearing Thermoplastic-Blend, for 3 mm Shaft Diameter, 5 mm Long     | [McMaster](https://www.mcmaster.com/2705T112/) |
| 1   | Ultra-Low-Friction Oil-Embedded Sleeve Bearing Flanged, High-Strength, 3mm Shaft Diameter, 6mm Housing ID | [McMaster](https://www.mcmaster.com/7119N113/) |
| 2   | Oil-Embedded 841 Bronze Sleeve Bearing for 3 mm Shaft Diameter and 5 mm Housing ID, 4 mm Long             | [McMaster](https://www.mcmaster.com/6658K412/) |

### Shims

| Qty       | Specification     |
| --------- | ----------------- |
| As needed | 1.7mm ID x 4mm OD |

For adjusting offsets on phalanx/thumb support linkages to prevent binding.

### Inserts

| Qty | Specification        | Link                                            |
| --- | -------------------- | ----------------------------------------------- |
| 10  | M2.5 Screw to expand | [McMaster](https://www.mcmaster.com/94510A370/) |

### Camera

| Qty | Specification  |
| --- | -------------- |
| 1   | RealSense D405 |

Wrist camera, optional

---

## 3D Printed Components

### PHALANX

| Qty | Component     |
| --- | ------------- |
| 4   | plx, distal   |
| 4   | plx, middle   |
| 4   | plx, proximal |
| 4   | plx, support  |
| 4   | plx, cover    |
| 4   | plx, knuckle  |
| 4   | plx, base     |

---

### THUMB

| Qty | Component     |
| --- | ------------- |
| 1   | tmb, base     |
| 1   | tmb, arm      |
| 1   | tmb, cmc      |
| 1   | tmb, mcp      |
| 1   | tmb, bridge   |
| 1   | tmb, proximal |
| 1   | tmb, cover    |
| 1   | plx, distal   |
| 1   | plx, middle   |
| 1   | plx, support  |
---

### WRIST

| Qty | Component     |
| --- | ------------- |
| 1   | palm, base    |
| 1   | palm, guard   |
| 1   | palm, coupler |

---

### SERVO SLEEVE

| Qty | Component                       |
| --- | ------------------------------- |
| 1   | sleeve, servo rack bottom       |
| 2   | sleeve, servo rack cap          |
| 1   | sleeve, servo rack recessed     |
| 2   | sleeve, servo rack top          |
| 1   | sleeve, servo single rack back  |
| 1   | sleeve, servo single rack front |
| 1   | sleeve, servo single rack top   |
| 1   | sleeve, base                    |
| 10  | sleeve, servo carriage          |

---

### PULLEY

| Qty | Component       |
| --- | --------------- |
| 15  | pulley, scs0009 |

---
While FDM works, SLA tends to yield better results for phalanx and thumb components.

## Assembly
Exploded views


Refer to CAD under `cad_asset` for assembly details.


---
If you're interested in contributing to the source CAD, please reach out to [me](wengmister@gmail.com).