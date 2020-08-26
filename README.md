# Bicylocomputer
Cycle computer made from:
- 4.3" Sharp memory LCD 
- Arduino Nano 33 BLE.
- 3.3V voltage regulator

Connects to Gadgetbridge on Android to display notifications, upload fitness data.

Features working:
- Gadgetbridge connection and notifications
- BLE Speed Sensor (address hard coded)
- BLE Cadence Sensor (address hard coded)
- Compass
- All derived data from above sensors
- LCD UI

Total Features:
- works in sunlight
- low power (can run for months off button cell)
- BLE connection to smartphone
- saves smartphone battery
- connects to speed/cadence BLE sensors
- displays:
- Clock, date?
- Tempature
- Altitude (from GPS or pressure sensor?)
- Current/average speed
- Current/averate cadence
- Total distance since ride start
- Total time since ride start
- Cumulative ascent (daily?) - consider using GPS to calibrate altiude initially
- Cumulative descent (daily?)
- Estimated gear ratio (or gear if chainring # teeth known).  Gear ratio/cadence to be constant before changing number.
- Song title+artist (if playing)
- Txts, IMs
- Maps directions (potentially)
- Acceleration
- Gradient (determine from 3D acceleration?)
- Compass direction
