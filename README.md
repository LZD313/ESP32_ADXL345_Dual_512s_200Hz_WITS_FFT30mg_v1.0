============================================================
ESP32 ADXL345 Dual Sensor Vibration Monitor
WITS Output Version
============================================================

1. OVERVIEW

This project implements a dual vibration monitoring system using:

- ESP32
- 2 × ADXL345 accelerometers (I2C)
- FFT dominant frequency detection
- WITS formatted serial output

The system measures:

- X, Y, Z acceleration (mg)
- Total RMS (including gravity)
- AC RMS (gravity removed)
- Dominant frequency (Hz, divided by 2)

Data is transmitted once per second.

------------------------------------------------------------

2. HARDWARE CONFIGURATION

Microcontroller:
ESP32

Sensors:
2 × ADXL345

I2C addresses:
Sensor 1: 0x53  (SDO connected to GND)
Sensor 2: 0x1D  (SDO connected to VCC)

Default ESP32 I2C pins:
SDA = GPIO21
SCL = GPIO22

------------------------------------------------------------

3. SAMPLING PARAMETERS

Sampling frequency: 200 Hz

FFT window size: 512 samples

Time window length:
512 / 200 = 2.56 seconds

WITS transmission interval:
1 second

FFT frequency search band:
Minimum frequency: 5 Hz
Maximum frequency: 80 Hz

AC RMS threshold for FFT:
30 mg

Frequency resolution:
Frequency resolution = Sampling Frequency / Number of Samples
200 / 512 = approximately 0.39 Hz

------------------------------------------------------------

4. SIGNAL PROCESSING LOGIC

Step 1 – Acceleration magnitude

For each sample:
Magnitude = sqrt( X^2 + Y^2 + Z^2 )

Step 2 – RMS calculation

Total RMS:
RMS calculated from magnitude (includes gravity).

AC RMS:
Mean magnitude is subtracted first, then RMS is calculated.
This removes the DC component (gravity).

Step 3 – FFT

- Hamming window is applied
- DC component is removed before FFT
- Dominant frequency is searched only between 5 Hz and 80 Hz
- Final reported frequency = Dominant Frequency / 2

------------------------------------------------------------

============================================================
WITS OUTPUT FORMAT
============================================================

Data is sent once per second in the following exact format:

&&
2711<X1>
2712<Y1>
2713<Z1>
2714<RMS1>
2715<RMSAC1>
2716<FREQ1>
2811<X2>
2812<Y2>
2813<Z2>
2814<RMS2>
2815<RMSAC2>
2816<FREQ2>
!!

IMPORTANT:

Each record is printed on a NEW LINE.

There are NO spaces between record number and value.

Example (real numbers):

&&
271112
271215
271318
271423
271520
271614
281111
281213
281317
281422
281519
281614
!!

------------------------------------------------------------

RECORD MEANING

2711 / 2811  Last absolute X value (mg)
2712 / 2812  Last absolute Y value (mg)
2713 / 2813  Last absolute Z value (mg)
2714 / 2814  Total RMS (mg)
2715 / 2815  AC RMS (mg)
2716 / 2816  Dominant frequency (Hz, divided by 2)

============================================================
------------------------------------------------------------

6. NOISE BEHAVIOR

At rest, AC RMS may show small values (typically 5–10 mg).
This is normal sensor noise and quantization noise from ADXL345.

Frequency is calculated only if:

AC RMS >= 30 mg

This prevents frequency detection from pure noise.

------------------------------------------------------------

7. LIBRARY REQUIREMENT

Required Arduino library:

arduinoFFT by Enrique Condes

Include in sketch:

#include <arduinoFFT.h>

------------------------------------------------------------

8. PERFORMANCE NOTES

- Stable frequency detection between 4 Hz and 30 Hz confirmed
- Frequency resolution approximately 0.39 Hz
- Suitable for rotating machinery monitoring
- Low CPU load on ESP32

------------------------------------------------------------

Version: v1.0
Platform: ESP32
Sampling: 200 Hz
Window: 512 samples
Output: WITS over Serial

============================================================
