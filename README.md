## ea-nassir 
A robot made to compete in the FTC decode season, built just to practise cad and coding skills =)

<img width="858" height="799" alt="Screenshot 2026-04-23 at 11 40 23 am" src="https://github.com/user-attachments/assets/315f9139-9762-4489-98d3-3a65d3f5d6b4" />


## why I made this

I wanted to attempt making a full FTC robot from scrach, on my own. despite my final design turning out very similar to widespread designs, I am happy just for building one of my own from scratch. I probably will not take part in any competitions, but I have been wanting to go through the whole process of building an FTC robot from start to finish, and the stasis hackathon finally gave me an excuse to do so.


## what there is

Currently, I have done the following:

CAD: The most complete, comprehensive part of the design, this consists of silicon molds for custom intake rollers, a drivetrain, 360 turret and PTO mechanism, all connected with custom CNC plates.

<img width="858" height="799" alt="Screenshot 2026-04-23 at 12 18 12 pm" src="https://github.com/user-attachments/assets/137bd4e0-e10d-4b81-a074-c82790fd101e" />


<img width="654" height="591" alt="Screenshot 2026-04-23 at 12 18 51 pm" src="https://github.com/user-attachments/assets/bb3c7d85-a491-4e1d-b186-c3fe40358069" />


<img width="654" height="591" alt="Screenshot 2026-04-23 at 12 19 32 pm" src="https://github.com/user-attachments/assets/80c4cb76-0aa8-4ff7-859e-e3cac3ef258f" />


<img width="654" height="703" alt="Screenshot 2026-04-23 at 12 31 41 pm" src="https://github.com/user-attachments/assets/e22b7073-8c81-444d-be19-44425f8e94e7" />


Coding: all set up and completed except for specific pedropathing parameters, will do when I build the physical robot (needs specific calibration)


<img width="1440" height="971" alt="Screenshot 2026-04-23 at 12 33 21 pm" src="https://github.com/user-attachments/assets/c1b0eda4-688a-4e03-b235-22bccc64967f" />

(note that code is in /TeamCode/src/main/java/org/firstinspires/ftc/teamcode)

BOM: created a (as comprehensive as I could remember) BOM

Wiring: also added the wiring diagram for all motors/servos/sensors/etc. - in WIRING.md

## BOM

(note that this may not be final. also, most of these parts already exist with me)

Electronics & Control

| # | Name | Item Code | Qty | Notes | Link |
|---|------|-----------|-----|-------|------|
| 1 | REV Control Hub | REV-31-1595 | 1 | Main robot controller | [Link](https://www.revrobotics.com/rev-31-1595/) |
| 2 | REV Expansion Hub | REV-31-1153 | 1 | Adds extra I/O ports | [Link](https://www.revrobotics.com/rev-31-1153/) |
| 3 | Axon MAX Servo | — | 2 | High-torque smart servo | [Link](https://axon-robotics.com/products/max) |
| 4 | GoBilda 5000 Series 12VDC Motor | 5000-0002-0001 | 2 | 12V DC motor, no encoder | [Link](https://www.gobilda.com/5000-series-12vdc-motor/) |
| 5 | GoBilda 5000 Series 12VDC Motor (REX Pinion Shaft) | 5000-0002-4008 | 1 | 12V DC motor with 8mm REX pinion shaft | [Link](https://www.gobilda.com/5000-series-12vdc-motor-with-8mm-rex-pinion-shaft/) |
| 6 | 5203 Series Yellow Jacket Planetary Gear Motor (13.7:1) | 5203-2402-0014 | 4 | 435 RPM, 8mm REX shaft, 3.3–5V encoder | [Link](https://www.gobilda.com/5203-series-yellow-jacket-planetary-gear-motor-13-7-1-ratio-24mm-length-8mm-rex-shaft-435-rpm-3-3-5v-encoder/) |
| 7 | 5203 Series Yellow Jacket Planetary Gear Motor (1:1) | 5203-242-0001 | 1 | 6000 RPM, 8mm REX shaft, 3.3–5V encoder | [Link](https://www.gobilda.com/5203-series-yellow-jacket-motor-1-1-ratio-24mm-length-8mm-rex-shaft-6000-rpm-3-3-5v-encoder/) |
| 8 | REV Through Bore Encoder | REV-11-3174 | 1 | Magnetic rotary encoder | [Link](https://www.revrobotics.com/rev-11-3174) |
| 9 | 6V Servo Power Injector (6 Channel, 8–15V Input) | 3125-0001-0001 | 1 | Powers servos independently from hubs | [Link](https://www.gobilda.com/6v-servo-power-injector-6-channel-8-15v-input/) |


 Sensors

| # | Name | Item Code | Qty | Notes | Link |
|---|------|-----------|-----|-------|------|
| 10 | Swyft Ranger Distance Sensor | — | 3 | Distance sensor | [Link](https://swyftrobotics.com/products/swyft-ranger-distance-sensor) |
| 11 | REV Magnetic Limit Switch Bundle | REV-31-1462 | 1 | Bundle includes REV-40-1467 switch | [Link](https://www.revrobotics.com/rev-31-1462/) |
| 12 | REV Magnet Limit Switch (individual) | REV-40-1467 (inside REV-31-1462) | 1 | Part of the bundle above | [Link](https://www.revrobotics.com/rev-31-1462/) |


Odometry & Navigation

| # | Name | Item Code | Qty | Notes | Link |
|---|------|-----------|-----|-------|------|
| 13 | GoBilda Pinpoint Odometry Pod (32mm wheel) | 3110-0001-0002 | 3 | 2 for drivetrain, 1 for other use | [Link](https://www.gobilda.com/4-bar-odometry-pod-32mm-wheel/) |


Drivetrain

| # | Name | Item Code | Qty | Notes | Link |
|---|------|-----------|-----|-------|------|
| 14 | Mecanum Wheel Set 96mm (70A, Bearing Supported) | 3213-3606-0002 | 4 (1 pack of 4) | Main drivetrain wheels | [Link](https://www.gobilda.com/96mm-mecanum-wheel-set-70a-durometer-bearing-supported-rollers/) |
| 15 | 3417 Series 5mm HTD Pinion Timing Belt Pulley (8mm REX, 16T) | 3417-4008-0016 | 4 | Drivetrain belt drive | [Link](https://www.gobilda.com/3417-series-5mm-htd-pitch-set-screw-pinion-timing-belt-pulley-8mm-rex-bore-16-tooth/) |
| 16 | 5203 Yellow Jacket Motor — see Electronics | — | 4 | Listed under Electronics (#6) | — |


Turret

| # | Name | Item Code | Qty | Notes | Link |
|---|------|-----------|-----|-------|------|
| 16 | KA070CPO Thin Section Bearing | KA070CPO | 1 | 177.8×190.5×6.35mm | [Link](https://zhongzhengbearing.en.made-in-china.com/product/PxOUjEkKZXhu/China-Thin-Section-Bearing-Ka070cpo-High-Quality-Bearing-Ka070cpo-Bearing-Size-177-8-190-5-6-35.html) |
| 17 | 2mm Pitch GT2 Pinion Timing Pulley (8mm REX, 20T) | 3422-4008-0020 | 3 | 2 for turret, 1 for other use | [Link](https://www.gobilda.com/2mm-pitch-gt2-pinion-timing-pulley-8mm-rex-bore-20-tooth/) |
| 18 | 2mm Pitch GT2 Hub Mount Timing Belt Pulley (14mm Bore, 60T) | 3421-0014-0060 | 1 | Large driven pulley | [Link](https://www.gobilda.com/2mm-pitch-gt2-hub-mount-timing-belt-pulley-14mm-bore-60-tooth/) |
| 19 | GT2 Timing Belts for Turret | — | 2 | 2mm pitch GT2 belts | [Link](https://ar.aliexpress.com/i/32860465902.html) |
| 20 | 2302 Series Aluminum Mod 0.8 Hub Mount Gear (14mm Bore, 60T) | 2302-0014-0060 | 2 | Turret gear drive | [Link](https://www.gobilda.com/2302-series-aluminum-mod-0-8-hub-mount-gear-14mm-bore-60-tooth/) |
| 21 | 2303 Series Steel Mod 0.8 Pinion Gear (8mm REX, 20T) | 2303-4008-0020 | 1 | Pinion for turret gear drive | [Link](https://www.gobilda.com/2303-series-steel-mod-0-8-pinion-gear-8mm-rex-bore-20-tooth/) |


Wheels & Rollers

| # | Name | Item Code | Qty | Notes | Link |
|---|------|-----------|-----|-------|------|
| 22 | 3614 Series Rhino Wheel (14mm Bore, 96mm, 30A) | 3614-0014-0096 | 1 | Soft rubber wheel | [Link](https://www.gobilda.com/3614-series-rhino-wheel-14mm-bore-96mm-diameter-30a-durometer/) |
| 23 | 3613 Series Gecko Wheel (14mm Bore, 96mm) | 3613-0014-0096 | 1 | High-grip gecko wheel | [Link](https://www.gobilda.com/3613-series-gecko-wheel-14mm-bore-96mm-diameter/) |
| 24 | 3613 Series Gecko Wheel (14mm Bore, 72mm) | 3613-0014-0072 | 2 | Smaller gecko wheel variant | [Link](https://www.gobilda.com/3613-series-gecko-wheel-14mm-bore-72mm-diameter/) |


Bearings

| # | Name | Item Code | Qty | Notes | Link |
|---|------|-----------|-----|-------|------|
| 25 | 1611 Series Flanged Ball Bearing (8mm REX ID × 14mm OD, 5mm) | 1611-0514-4008 | 15 | 4 for turret, 11 for drivetrain | [Link](https://www.gobilda.com/1611-series-flanged-ball-bearing-8mm-rex-id-x-14mm-od-5mm-thickness-2-pack/) |
| 26 | Miniature Flange Bearings Assortment (MF52ZZ–MF148ZZ) | — | 10 | Various small flanged bearings | [Link](https://www.temu.com/ae/10pcs-mf52zz-mf62zz-mf63zz-mf74zz-mf83zz-mf84zz-to-mf148zz-miniature-flange-bearing-thin-wall-metal-shielded-flanged-bearings-g-601101431047091.html) |


Hubs & Shafts

| # | Name | Item Code | Qty | Notes | Link |
|---|------|-----------|-----|-------|------|
| 27 | 1310 Series Hyper Hub (8mm REX Bore) | 1310-0016-4008 | 5 | 1 for turret, 4 for drivetrain | [Link](https://www.gobilda.com/1310-series-hyper-hub-8mm-rex-bore/) |
| 28 | 1309 Series Sonic Hub (8mm REX Bore) | 1309-0016-4008 | 8 | 1 for turret, 7 for drivetrain | [Link](https://www.gobilda.com/1309-series-sonic-hub-8mm-rex-bore/) |
| 29 | 1906 Series Lightweight Servo Hub (25T Spline, 32mm) | 1906-0025-0032 | 3 | 2 for turret/arm, 1 for drivetrain | [Link](https://www.gobilda.com/1906-series-lightweight-servo-hub-25-tooth-spline-32mm-diameter/) |
| 30 | 8mm REX Shaft with E-Clip (Stainless, 192mm) | 2106-4008-1920 | 1 | Main shaft for turret/arm assembly | [Link](https://www.gobilda.com/8mm-rex-shaft-with-e-clip-stainless-steel-192mm-length/) |


Structural Components

| # | Name | Item Code | Qty | Notes | Link |
|---|------|-----------|-----|-------|------|
| 31 | 1119 Series Shaft Beam (7-Hole, 56mm) | 1119-0007-0056 | 16 | 2 for turret, 14 for drivetrain | [Link](https://www.gobilda.com/1119-series-shaft-beam-7-hole-56mm-length/) |
| 32 | 1121 Series Low-Side U-Channel (9-Hole, 240mm) | 1121-0009-0240 | 2 | Drivetrain frame channel | [Link](https://www.gobilda.com/1121-series-low-side-u-channel-9-hole-240mm-length/) |
| 33 | 1205 Series Dual Block Mount (1.5) | 1205-0001-0005 | 10 | 4 for turret, 2 from turret pack, 4 for drivetrain | [Link](https://www.gobilda.com/1205-series-dual-block-mount-1-5-2-pack/) |


Standoffs

| # | Name | Item Code | Qty | Notes | Link |
|---|------|-----------|-----|-------|------|
| 34 | 1516 Series 8mm REX Standoff (192mm) | 1516-4008-1920 | 5 | 4 for turret, 1 for drivetrain | [Link](https://www.gobilda.com/1516-series-8mm-rex-standoff-m4-x-0-7mm-threads-192mm-length-4-pack/) |
| 35 | 1516 Series 8mm REX Standoff (288mm) | 1516-4008-2880 | 3 | Drivetrain frame | [Link](https://www.gobilda.com/1516-series-8mm-rex-standoff-m4-x-0-7mm-threads-288mm-length-4-pack/) |
| 36 | 1501 Series M4 × 0.7mm Standoff (26mm) | 1501-0006-0260 | 16 | Drivetrain spacers | [Link](https://www.gobilda.com/1501-series-m4-x-0-7mm-standoff-6mm-od-26mm-length-4-pack/) |
| 37 | 1501 Series M4 × 0.7mm Standoff (16mm) | 1501-0006-0160 | 12 | Drivetrain spacers | [Link](https://www.gobilda.com/1501-series-m4-x-0-7mm-standoff-6mm-od-16mm-length-4-pack/) |
| 38 | 1501 Series M4 × 0.7mm Standoff (46mm) | 1501-0006-0460 | 12 | Drivetrain spacers | [Link](https://www.gobilda.com/1501-series-m4-x-0-7mm-standoff-6mm-od-46mm-length-4-pack/) |


Fasteners

| # | Name | Item Code | Qty | Notes | Link |
|---|------|-----------|-----|-------|------|
| 39 | Zinc-Plated Steel Socket Head Screw M3 × 0.5mm × 12mm | 2800-0003-0012 | 8 | 6-pack | [Link](https://www.gobilda.com/zinc-plated-steel-socket-head-screw-m3-x-0-5mm-12mm-length-6-pack/) |
| 40 | Hex Socket Head Cap Screw M3 × 0.5mm × 5mm | — | 16 | Generic M3×5 cap screws | — |
| 41 | Zinc-Plated Steel Button Head Screw M4 × 0.7mm × 60mm (25-Pack) | 2802-0004-0060 | 1 pack | Long M4 bolts | [Link](https://www.gobilda.com/2802-series-zinc-plated-steel-button-head-screw-m4-x-0-7mm-60mm-length-25-pack/) |


Consumables & Misc

| # | Name | Item Code | Qty | Notes | Link |
|---|------|-----------|-----|-------|------|
| 41 | Loctite Threadlocker Blue 242 (6mL) | 2922-0001-0001 | 2 | Thread locking compound | [Link](https://www.gobilda.com/loctite-threadlocker-blue-242-6ml/) |
