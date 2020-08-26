# Bicylocomputer
Cycle computer made from:
- 4.3" Sharp memory LCD 
- Arduino Nano 33 BLE.
- 3.3V voltage regulator (button cell or li-ion can be used)

Connects to Gadgetbridge on Android to display notifications, upload fitness data.

Features working
- LCD UI
  - Sunlight visible
- Gadgetbridge connection
  - Notifications from many apps
  - Current song title, artist
  - Time synchronisation
- BLE Speed Sensor (address hard coded)
  - Current speed
  - Average speed
  - Total distance
  - Total moving time
- BLE Cadence Sensor (address hard coded)
  - Current cadence
  - Average cadence
  - Total pedal revolutions
  - Total pedalling time
- Compass
  - Graphical arrow
  - Bearing
- Time
- Temperature
- Humidity


Features to come:
- Altitude (from GPS or pressure sensor?)
- Cumulative ascent (daily?) - consider using GPS to calibrate altiude initially
- Cumulative descent (daily?)
- Estimated gear ratio (or gear if chainring # teeth known).  Gear ratio/cadence to be constant before changing number.
- Maps directions (potentially)
- Acceleration
- Gradient (determine from 3D acceleration?)
