#include <Wire.h>
#include <arduinoFFT.h>
#include <math.h>

// ======================= ADXL345 registrai =========================
#define ADXL345_ADDR_1   0x53   // sensorius 1 (SDO -> GND)
#define ADXL345_ADDR_2   0x1D   // sensorius 2 (SDO -> VCC)
#define REG_POWER_CTL    0x2D
#define REG_DATA_FORMAT  0x31
#define REG_BW_RATE      0x2C
#define REG_DATAX0       0x32

// ADXL345 ~3.9 mg/LSB (+-2g režimu)
const float ADXL_LSB_TO_MG = 3.9f;

// ==================== Mėginių ėmimas / FFT =========================
// ~3 s langas: 512 mėginių / 200 Hz ≈ 2.56 s
const uint16_t MAX_SAMPLES        = 512;      // žiedinio buferio dydis
const double   SAMPLE_FREQ        = 200.0;    // Hz
const uint32_t SAMPLE_INTERVAL_US = (uint32_t)(1000000.0 / SAMPLE_FREQ);

// FFT min AC RMS slenkstis (mg) – žemiau to dažnio neskaičiuojam
const int FFT_MIN_AC_RMS_MG = 30;

// Dažnių filtras FFT’ui (Hz)
const double FREQ_MIN = 5.0;
const double FREQ_MAX = 80.0;

// WITS kadrą siunčiam kas 1 s
const uint32_t WITS_PERIOD_MS = 1000;

// FFT buferiai
double vReal[MAX_SAMPLES];
double vImag[MAX_SAMPLES];

// ArduinoFFT objektas (naujos bibliotekos API)
ArduinoFFT<double> FFT(vReal, vImag, MAX_SAMPLES, SAMPLE_FREQ);

// ==================== Buferiai sensoriams ==========================
// Sensorius 1
int16_t  x1Buf[MAX_SAMPLES];
int16_t  y1Buf[MAX_SAMPLES];
int16_t  z1Buf[MAX_SAMPLES];
uint32_t t1Buf[MAX_SAMPLES];
int idx1   = 0;
int count1 = 0;

// Sensorius 2
int16_t  x2Buf[MAX_SAMPLES];
int16_t  y2Buf[MAX_SAMPLES];
int16_t  z2Buf[MAX_SAMPLES];
uint32_t t2Buf[MAX_SAMPLES];
int idx2   = 0;
int count2 = 0;

// Būklės flag'ai
bool has1 = false;
bool has2 = false;

int lastFreq1 = 0;
int lastFreq2 = 0;

// ========================= Pagalbinės fn ===========================
void writeRegister(uint8_t addr, uint8_t reg, uint8_t value) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(value);
  Wire.endTransmission();
}

bool initADXL345(uint8_t addr) {
  // BW_RATE – 0x0B = 200 Hz
  writeRegister(addr, REG_BW_RATE, 0x0B);

  // DATA_FORMAT – 0x08 = FULL_RES, +-2g
  writeRegister(addr, REG_DATA_FORMAT, 0x08);

  // POWER_CTL – 0x08 = Measurement mode
  writeRegister(addr, REG_POWER_CTL, 0x08);

  delay(10);

  // Paprastas "ping" – pabandysim perskaityti vieną registrą
  Wire.beginTransmission(addr);
  Wire.write(REG_POWER_CTL);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }
  Wire.requestFrom((int)addr, 1);
  if (Wire.available() < 1) {
    return false;
  }
  (void)Wire.read();
  return true;
}

bool readAccelRaw(uint8_t addr, int16_t &x, int16_t &y, int16_t &z) {
  Wire.beginTransmission(addr);
  Wire.write(REG_DATAX0);
  if (Wire.endTransmission(false) != 0) {
    return false;
  }

  Wire.requestFrom((int)addr, 6);
  uint32_t start = millis();
  while (Wire.available() < 6) {
    if (millis() - start > 10) return false; // timeout
  }

  uint8_t x0 = Wire.read();
  uint8_t x1 = Wire.read();
  uint8_t y0 = Wire.read();
  uint8_t y1 = Wire.read();
  uint8_t z0 = Wire.read();
  uint8_t z1 = Wire.read();

  x = (int16_t)((x1 << 8) | x0);
  y = (int16_t)((y1 << 8) | y0);
  z = (int16_t)((z1 << 8) | z0);

  return true;
}

// ======================== RMS skaičiavimas =========================
// xBuf, yBuf, zBuf – mg vienetais
// idx – write pointeris (į šitą dar NERA įrašyta, paskutinis yra idx-1)
// count – kiek realiai užpildyta (<= MAX_SAMPLES)
// rms      – bendras RMS (mg)
// rmsAC    – RMS be DC (gravitacijos) (mg)
void computeRMS(
  int16_t *xBuf, int16_t *yBuf, int16_t *zBuf,
  int idx, int count,
  int &rms, int &rmsAC
) {
  if (count <= 0) {
    rms = 0;
    rmsAC = 0;
    return;
  }

  int n = count;
  if (n > MAX_SAMPLES) n = MAX_SAMPLES;

  // 1. Apskaičiuojam vektoriaus modulį ir jo vidurkį
  double sum = 0.0;
  for (int k = 0; k < n; k++) {
    int i = idx - 1 - k;
    if (i < 0) i += MAX_SAMPLES;

    double x = xBuf[i];
    double y = yBuf[i];
    double z = zBuf[i];
    double mag = sqrt(x * x + y * y + z * z);
    sum += mag;
  }
  double mean = sum / (double)n;

  // 2. RMS (total) ir RMS AC
  double sumSq   = 0.0;
  double sumSqAC = 0.0;

  for (int k = 0; k < n; k++) {
    int i = idx - 1 - k;
    if (i < 0) i += MAX_SAMPLES;

    double x = xBuf[i];
    double y = yBuf[i];
    double z = zBuf[i];
    double mag = sqrt(x * x + y * y + z * z);

    double v   = mag;
    double vAC = mag - mean;

    sumSq   += v * v;
    sumSqAC += vAC * vAC;
  }

  double rmsVal   = sqrt(sumSq   / (double)n);
  double rmsACVal = sqrt(sumSqAC / (double)n);

  rms   = (int)round(rmsVal);
  rmsAC = (int)round(rmsACVal);
}

// ====================== FFT dažnio skaičiavimas ====================
// Grąžina DOMINUOJANTĮ dažnį, padalintą iš 2 (Hz, sveikas),
// ribotą FREQ_MIN..FREQ_MAX (ant pradinio spektro).
// Jei nieko gero neranda – grąžina lastFreq (histerezė).
int computeFreqFFT(
  int16_t *xBuf, int16_t *yBuf, int16_t *zBuf,
  uint32_t *tBuf, int idx, int count,
  int lastFreq
) {
  (void)tBuf; // tBuf šitoje versijoje nenaudojam, bet paliekam signatūrai

  if (count < 8) {
    return 0;
  }

  int n = count;
  if (n > MAX_SAMPLES) n = MAX_SAMPLES;

  // Į FFT imame paskutinius n mėginių
  double sum = 0.0;
  for (int k = 0; k < n; k++) {
    int i = idx - 1 - k;
    if (i < 0) i += MAX_SAMPLES;

    double x = xBuf[i];
    double y = yBuf[i];
    double z = zBuf[i];
    double mag = sqrt(x * x + y * y + z * z);
    vReal[k] = mag;
    vImag[k] = 0.0;
    sum += mag;
  }
  double mean = sum / (double)n;

  // Nuimam DC
  for (int k = 0; k < n; k++) {
    vReal[k] -= mean;
  }

  // Jei n < MAX_SAMPLES, likusį masyvą užpildom nuliais
  for (int k = n; k < MAX_SAMPLES; k++) {
    vReal[k] = 0.0;
    vImag[k] = 0.0;
  }

  // Naujos ArduinoFFT API:
  FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
  FFT.compute(FFT_FORWARD);
  FFT.complexToMagnitude();

  double bestAmp  = 0.0;
  double bestFreq = 0.0;

  // tikrinam tik iki Nyquist (MAX_SAMPLES/2)
  for (int i = 1; i < MAX_SAMPLES / 2; i++) {
    double freq = (i * SAMPLE_FREQ) / (double)MAX_SAMPLES;
    if (freq < FREQ_MIN || freq > FREQ_MAX) {
      continue;
    }
    double amp = vReal[i];
    if (amp > bestAmp) {
      bestAmp  = amp;
      bestFreq = freq;
    }
  }

  if (bestAmp <= 0.0 || bestFreq <= 0.0) {
    // nieko gero neradom – laikomės ankstesnio
    return lastFreq;
  }

  // ČIA PADAROM korekciją: grąžinam per pusę mažesnį dažnį
  double freqOut = bestFreq / 2.0;
  return (int)round(freqOut);
}

// =========================== setup/loop ============================
void setup() {
  Serial.begin(115200);
  // ESP32 default: SDA=21, SCL=22 (jei tavo dev board’e kitaip – naudok Wire.begin(SDA, SCL))
  Wire.begin();

  has1 = initADXL345(ADXL345_ADDR_1);
  has2 = initADXL345(ADXL345_ADDR_2);

  delay(100);
}

void loop() {
  static uint32_t lastSampleUs = 0;
  static uint32_t lastWitsMs   = 0;

  uint32_t nowUs = micros();
  uint32_t nowMs = millis();

  // ===================== Mėginių ėmimas ============================
  if (nowUs - lastSampleUs >= SAMPLE_INTERVAL_US) {
    lastSampleUs += SAMPLE_INTERVAL_US;

    int16_t x, y, z;

    // Sensorius 1
    if (has1) {
      if (readAccelRaw(ADXL345_ADDR_1, x, y, z)) {
        // į mg
        x1Buf[idx1] = (int16_t)round(x * ADXL_LSB_TO_MG);
        y1Buf[idx1] = (int16_t)round(y * ADXL_LSB_TO_MG);
        z1Buf[idx1] = (int16_t)round(z * ADXL_LSB_TO_MG);
        t1Buf[idx1] = nowUs;

        idx1++;
        if (idx1 >= MAX_SAMPLES) idx1 = 0;
        if (count1 < MAX_SAMPLES) count1++;
      } else {
        has1 = false;
      }
    }

    // Sensorius 2
    if (has2) {
      if (readAccelRaw(ADXL345_ADDR_2, x, y, z)) {
        x2Buf[idx2] = (int16_t)round(x * ADXL_LSB_TO_MG);
        y2Buf[idx2] = (int16_t)round(y * ADXL_LSB_TO_MG);
        z2Buf[idx2] = (int16_t)round(z * ADXL_LSB_TO_MG);
        t2Buf[idx2] = nowUs;

        idx2++;
        if (idx2 >= MAX_SAMPLES) idx2 = 0;
        if (count2 < MAX_SAMPLES) count2++;
      } else {
        has2 = false;
      }
    }
  }

  // ======================= WITS išvedimas ==========================
  if (nowMs - lastWitsMs >= WITS_PERIOD_MS) {
    lastWitsMs = nowMs;

    Serial.println("&&");

    // =====================================================
    //                SENSORIUS 1 → 2711–2716
    // =====================================================
    if (has1 && count1 > 0) {
      int i = idx1 - 1;
      if (i < 0) i = MAX_SAMPLES - 1;
      int X = abs(x1Buf[i]);
      int Y = abs(y1Buf[i]);
      int Z = abs(z1Buf[i]);

      int rms, rmsAC;
      computeRMS(x1Buf, y1Buf, z1Buf, idx1, count1, rms, rmsAC);

      int freq = 0;
      if (rmsAC >= FFT_MIN_AC_RMS_MG) {
        freq = computeFreqFFT(x1Buf, y1Buf, z1Buf, t1Buf, idx1, count1, lastFreq1);
      }
      lastFreq1 = freq;

      Serial.print("2711"); Serial.println(X);
      Serial.print("2712"); Serial.println(Y);
      Serial.print("2713"); Serial.println(Z);
      Serial.print("2714"); Serial.println(rms);
      Serial.print("2715"); Serial.println(rmsAC);
      Serial.print("2716"); Serial.println(freq);
    }

    // =====================================================
    //                SENSORIUS 2 → 2811–2816
    // =====================================================
    if (has2 && count2 > 0) {
      int i = idx2 - 1;
      if (i < 0) i = MAX_SAMPLES - 1;
      int X = abs(x2Buf[i]);
      int Y = abs(y2Buf[i]);
      int Z = abs(z2Buf[i]);

      int rms, rmsAC;
      computeRMS(x2Buf, y2Buf, z2Buf, idx2, count2, rms, rmsAC);

      int freq = 0;
      if (rmsAC >= FFT_MIN_AC_RMS_MG) {
        freq = computeFreqFFT(x2Buf, y2Buf, z2Buf, t2Buf, idx2, count2, lastFreq2);
      }
      lastFreq2 = freq;

      Serial.print("2811"); Serial.println(X);
      Serial.print("2812"); Serial.println(Y);
      Serial.print("2813"); Serial.println(Z);
      Serial.print("2814"); Serial.println(rms);
      Serial.print("2815"); Serial.println(rmsAC);
      Serial.print("2816"); Serial.println(freq);
    }

    Serial.println("!!");
  }
}