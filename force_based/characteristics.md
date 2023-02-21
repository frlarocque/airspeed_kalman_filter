# Characteristics of aircraft

## Questions
1. What is the airfoil for the main wing?
  MH32 modified, 15% thickness 5% camber
2. What is the airfoil for the Stab V, Stab H?
  NACA 0015
3. What range of motion can the Stab H move?
  37 deg down, 10 deg up
4. What is the expected cruise speed of the vehicle? Max speed?
  16 m/s cruise, about 20 m/s max
5. What is the expected transition speed of the vehicle? (was the wing skew schedule already calculated for the V3?)
  transition between 12-15 m/s


## Wing
|Parameter  |Value  |Unit|
| --------- |------ |---|
| Aifoil    |Mh32 modified |- |
| Wingspan  |1.797  | m |
| Tip chord |0.178  | m |
| Root chord|0.3    | m |
| MAC       | 0.244 | m |
| Surface   | 0.4388| m^2|
| Aspect Ratio|7.359 | - |
| Taper     | 0.593 | - |
| Sweep     | 0     |deg |



## Stab H
|Parameter  |Value  |Unit|
| --------- |------ |---|
| Aifoil    |NACA 0015 |- |
| Wingspan  |0.8  | m |
| Tip chord |0.175  | m |
| Root chord|0.200  | m |
| MAC       |0.1878  | m |
| Surface   |0.150 | m^2|
| Aspect Ratio|4.26 | - |
| Taper     |0.875  | - |
| Sweep     | 0     |deg |
| LE to CG  |0.696 |m |


## Stab V
|Parameter  |Value  |Unit|
| --------- |------ |---|
| Aifoil    |NACA 0015 |- |
| Wingspan  |0.251  | m |
| Tip chord |0.132  | m |
| Root chord|0.175  | m |
| MAC       |0.155  | m |
| Surface   |0.0388 | m^2|
| Aspect Ratio|1.62 | - |
| Taper     |0.754  | - |
| Sweep     | 0     |deg |
| LE to CG  |0.696 |m |


taper = c_t/c_r
MAC = 2/3*c_r*(1+taper+taper^2)/)(1+taper)
AR = b^2/A

## Reynolds
At sea level, standard conditions

### Wing
@10 m/s: 167 000
@16 m/s: 267 000
@20 m/s: 334 000

### Stab H
@10 m/s: 129 000
@16 m/s: 207 000
@20 m/s: 259 000

### Stab V
@10 m/s: 106 000
@16 m/s: 170 000
@20 m/s: 212 000

## Performance XFLR5

### MH32 modified
CL_0 = 0.35
CL_10 = 1.154
CL_alpha =  0.0804 1/deg or 4.60658 1/rad

### NACA 0015
CL_0 = 0
CL_10 = 0.656
CL_alpha = 0.0656 1/deg or 3.7586 1/rad

## Performance Wind Tunnel Characteristics

### Aircraft without pusher prop (LP1)
CL_0 = 0.36
CL_alpha = 3.10
0.5*rho*S = 0.4161 --> S = 0.68

### Aircraft without wing and pusher (LP3)
CL_0 = 0.4
CL_alpha = 4.48
S = 0.09

### Without Wing, hover motor props (LP4)
#### Forward flight (Skew = 90 deg)
CL_0 = -0.56
CL_alpha = 19.94
S = 0.02

Force/Airspeed at 15 deg angle of attack: 0.0533 N / (m/s)^2

#### Quad Mode (Skew = 0 deg)
CL_0 = -0.27
CL_alpha = 11.29
S = 0.03

Force/Airspeed at 15 deg angle of attack: 0.056 N / (m/s)^2

#### All skew angles
CL_0 = -0.6580
CL_alpha = 26.4910
S = 0.0138

### Aircraft without pusher and hover motors props (LP6)

#### Forward flight (Skew = 90 deg)
CL_0 = 0.3641
CL_alpha = 4.0547
S = 0.5486

Force/Airspeed at 15 deg angle of attack: 0.47 N / (m/s)^2

#### Quad Mode (Skew = 0 deg)
CL_0 = 0.17
CL_alpha = 14.83
S = 0.06
