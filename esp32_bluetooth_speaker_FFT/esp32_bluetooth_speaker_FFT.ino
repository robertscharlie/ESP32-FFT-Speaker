/*
  ESP32 Bluetooth Speaker — BT phase, with live FFT spectrum display
  - A2DP Bluetooth sink, audio out via internal DAC (GPIO25 = L, GPIO26 = R)
  - SSD1306 OLED (I2C on GPIO14/27): connection status, play/pause,
    scrolling track info, live 16-band FFT spectrum
  - Buttons: volume up/down, play/pause, mode (reserved for AM radio switch)

  Libraries required:
    - "ESP32-A2DP" by pschatzmann (manual ZIP install)
    - "arduino-audio-tools" by pschatzmann (manual ZIP install)
    - "Adafruit SSD1306" (Library Manager)
    - "Adafruit GFX Library" (Library Manager)
    - "arduinoFFT" by kosme (Library Manager)
*/

#include "AudioTools.h"
#include "BluetoothA2DPSink.h"
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <arduinoFFT.h>

// Forward declaration: Arduino auto-generates function prototypes right
// after the includes, before struct Button is defined below.
struct Button;

// ---------- Pins ----------
#define I2C_SDA      14
#define I2C_SCL      27
#define BTN_VOL_UP   32
#define BTN_VOL_DN   33
#define BTN_PLAY     12
#define BTN_MODE     13

#define SCREEN_WIDTH  128
#define SCREEN_HEIGHT 64
#define OLED_ADDR     0x3C

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);
AnalogAudioStream out;           // drives internal DAC: GPIO25 (L) / GPIO26 (R)
BluetoothA2DPSink a2dp_sink(out);

// ---------- Shared state (set in callbacks, read in loop) ----------
volatile bool btConnected = false;
volatile bool isPlaying   = false;
String trackTitle  = "";
String trackArtist = "";
int volume = 50; // 0-100, software-tracked for display purposes

// Bluetooth glyph, 8x16 pixels (the classic bind-rune shape)
static const unsigned char PROGMEM bt_icon[] = {
  0x18, 0x1C, 0x1E, 0x96, 0xD8, 0x70, 0x20, 0x20,
  0x20, 0x20, 0x70, 0xD8, 0x96, 0x1E, 0x1C, 0x18
};

// Scrolling state for track titles too long to fit on screen
int scrollPos = 0;
unsigned long lastScrollTime = 0;
const unsigned long SCROLL_INTERVAL_MS = 350;
const int VISIBLE_CHARS = 21; // approx characters that fit at text size 1, 128px wide

// ---------- FFT spectrum analyzer ----------
#define FFT_SAMPLES 256          // must be a power of 2
#define SAMPLING_FREQUENCY 44100 // matches the A2DP PCM stream's rate
#define NUM_BANDS 16             // how many bars are drawn on screen

double vReal[FFT_SAMPLES];
double vImag[FFT_SAMPLES];
ArduinoFFT<double> FFT = ArduinoFFT<double>(vReal, vImag, FFT_SAMPLES, SAMPLING_FREQUENCY);

// Filled by the audio callback (runs on the Bluetooth stack's own task),
// consumed by loop() — kept as a simple flag-guarded buffer rather than a
// full mutex, since only one side writes at a time by design (see below).
volatile bool fftBufferReady = false;
int fftWriteIndex = 0;
int16_t fftCaptureBuffer[FFT_SAMPLES];

float bandValues[NUM_BANDS] = {0}; // current bar heights (0-100)

// Called from the Bluetooth stack's task context — kept deliberately light
// (just copying samples) so it never delays audio playback itself.
void read_data_stream(const uint8_t *data, uint32_t length) {
  if (fftBufferReady) return; // loop() hasn't consumed the last buffer yet — skip this packet
  int16_t *samples = (int16_t *)data;
  uint32_t sampleCount = length / 2; // 16-bit samples
  // PCM here is interleaved stereo (L, R, L, R, ...) — take the left
  // channel only, which is plenty for a visual spectrum display.
  for (uint32_t i = 0; i < sampleCount && !fftBufferReady; i += 2) {
    fftCaptureBuffer[fftWriteIndex++] = samples[i];
    if (fftWriteIndex >= FFT_SAMPLES) {
      fftWriteIndex = 0;
      fftBufferReady = true;
    }
  }
}

// Groups the FFT's linear bins into NUM_BANDS log-spaced bands (each
// roughly an octave). Each band auto-calibrates against its own recent
// quietest/loudest level in dB (snaps instantly to a new extreme, relaxes
// back slowly otherwise) so bars stay scaled to whatever's playing instead
// of a fixed window that clips on loud songs or sits flat on quiet ones.
// Bar heights get no smoothing — set straight from each frame's reading.
float bandFloorDb[NUM_BANDS] = {0};
float bandCeilDb[NUM_BANDS]  = {0};

void computeBands() {
  int usableBins = FFT_SAMPLES / 2; // upper half of the FFT output is redundant for real input

  // Skip bin 0 (DC) and bin 1: the 256-sample window is too short to
  // resolve anything below ~170Hz, so content down there is mostly leakage.
  const int minBin = 2;

  // How fast the floor/ceiling relax back when the signal isn't setting a
  // new extreme — in dB per FFT block (roughly every ~12ms).
  const float RANGE_RELEASE = 0.05f;
  // Never let the scale collapse to (near) zero width.
  const float MIN_SPAN_DB = 6.0f;

  for (int band = 0; band < NUM_BANDS; band++) {
    int startBin = minBin + (int)pow((float)usableBins, (float)band / NUM_BANDS);
    int endBin   = minBin + (int)pow((float)usableBins, (float)(band + 1) / NUM_BANDS);
    if (endBin <= startBin) endBin = startBin + 1;
    if (endBin > usableBins) endBin = usableBins;

    double peak = 0;
    for (int bin = startBin; bin < endBin; bin++) {
      if (vReal[bin] > peak) peak = vReal[bin];
    }
    float db = (peak > 0) ? 20.0 * log10(peak) : 0;

    if (db > bandCeilDb[band]) {
      bandCeilDb[band] = db;             // new loudest moment: snap up now
    } else {
      bandCeilDb[band] -= RANGE_RELEASE; // otherwise ease back down
    }

    if (db < bandFloorDb[band]) {
      bandFloorDb[band] = db;            // new quietest moment: snap down now
    } else {
      bandFloorDb[band] += RANGE_RELEASE; // otherwise ease back up
    }

    if (bandCeilDb[band] - bandFloorDb[band] < MIN_SPAN_DB) {
      bandCeilDb[band] = bandFloorDb[band] + MIN_SPAN_DB;
    }

    float normalized = (db - bandFloorDb[band]) / (bandCeilDb[band] - bandFloorDb[band]) * 100.0;
    bandValues[band] = constrain(normalized, 0, 100);
  }
}

// ---------- Button debounce ----------
struct Button {
  uint8_t pin;
  bool lastReading;
  bool stableState;
  unsigned long lastChangeTime;
};

// Idle state is LOW with pull-down wiring (pressed = HIGH)
Button btnVolUp = {BTN_VOL_UP, LOW, LOW, 0};
Button btnVolDn = {BTN_VOL_DN, LOW, LOW, 0};
Button btnPlay  = {BTN_PLAY,   LOW, LOW, 0};
Button btnMode  = {BTN_MODE,   LOW, LOW, 0};

const unsigned long DEBOUNCE_MS = 40;

// Returns true exactly once, on the press transition (LOW -> HIGH), after debounce
bool checkButtonPressed(Button &b) {
  bool reading = digitalRead(b.pin);
  if (reading != b.lastReading) {
    b.lastChangeTime = millis();
  }
  bool pressedEvent = false;
  if ((millis() - b.lastChangeTime) > DEBOUNCE_MS) {
    if (reading != b.stableState) {
      b.stableState = reading;
      if (b.stableState == HIGH) {
        pressedEvent = true;
      }
    }
  }
  b.lastReading = reading;
  return pressedEvent;
}

// ---------- A2DP callbacks ----------
void connection_state_changed(esp_a2d_connection_state_t state, void *ptr) {
  btConnected = (state == ESP_A2D_CONNECTION_STATE_CONNECTED);
}

void avrc_rn_playstatus_callback(esp_avrc_playback_stat_t playback) {
  isPlaying = (playback == ESP_AVRC_PLAYBACK_PLAYING);
}

void avrc_metadata_callback(uint8_t id, const uint8_t *text) {
  switch (id) {
    case ESP_AVRC_MD_ATTR_TITLE:
      trackTitle = String((char *)text);
      break;
    case ESP_AVRC_MD_ATTR_ARTIST:
      trackArtist = String((char *)text);
      break;
  }
}

// ---------- Display ----------
void updateDisplay() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // --- Header row: BT glyph + status text, play/pause icon ---
  if (btConnected) {
    display.drawBitmap(0, 0, bt_icon, 8, 16, SSD1306_WHITE);
    display.setCursor(12, 4);
    display.print("Connected");
  } else {
    display.setCursor(0, 4);
    display.print("No BT connection");
  }

  int iconX = SCREEN_WIDTH - 12;
  if (isPlaying) {
    display.fillTriangle(iconX, 2, iconX, 14, iconX + 10, 8, SSD1306_WHITE);
  } else {
    display.fillRect(iconX, 2, 3, 12, SSD1306_WHITE);
    display.fillRect(iconX + 7, 2, 3, 12, SSD1306_WHITE);
  }

  display.drawLine(0, 17, SCREEN_WIDTH, 17, SSD1306_WHITE);

  // --- Track title + artist combined onto one scrolling line, to leave
  //     more screen height for the spectrum below ---
  String titleToShow = trackTitle.length() ? trackTitle : "No track";
  if (trackArtist.length()) titleToShow += " - " + trackArtist;
  display.setCursor(0, 21);
  if ((int)titleToShow.length() > VISIBLE_CHARS) {
    if (millis() - lastScrollTime > SCROLL_INTERVAL_MS) {
      scrollPos = (scrollPos + 1) % (titleToShow.length() + 3);
      lastScrollTime = millis();
    }
    String padded = titleToShow + "   " + titleToShow;
    display.print(padded.substring(scrollPos, scrollPos + VISIBLE_CHARS));
  } else {
    display.print(titleToShow);
  }

  display.drawLine(0, 30, SCREEN_WIDTH, 30, SSD1306_WHITE);

  // --- FFT spectrum bars: rounded pill bars that grow outward from a
  //     center line instead of sitting on the bottom ---
  int barsTop = 32, barsBottom = 60;
  int barsHeight = barsBottom - barsTop;
  int midY = (barsTop + barsBottom) / 2;
  int barWidth = SCREEN_WIDTH / NUM_BANDS;

  for (int i = 0; i < NUM_BANDS; i++) {
    int x = i * barWidth;
    int w = (barWidth - 2) / 2;             // about half as thin as before
    int barX = x + (barWidth - w) / 2;      // keep it centered in its slot
    int h = (int)(bandValues[i] / 100.0f * barsHeight);

    if (h > 0) {
      int r = min(2, h / 2); // rounded ends; shrinks for short bars so it never looks odd
      display.fillRoundRect(barX, midY - h / 2, w, h, r, SSD1306_WHITE);
    }
  }

  // --- Thin volume indicator, bottom edge ---
  int volWidth = map(volume, 0, 100, 0, SCREEN_WIDTH);
  display.fillRect(0, 62, volWidth, 2, SSD1306_WHITE);

  display.display();
}

void setup() {
  Serial.begin(115200);

  // Plain INPUT here, not INPUT_PULLUP — these buttons use external
  // 10k pull-down resistors, so the ESP32's internal pull-up isn't needed.
  pinMode(BTN_VOL_UP, INPUT);
  pinMode(BTN_VOL_DN, INPUT);
  pinMode(BTN_PLAY,   INPUT);
  pinMode(BTN_MODE,   INPUT);

  Wire.begin(I2C_SDA, I2C_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("OLED init failed - check wiring/address");
  }
  display.clearDisplay();
  display.display();

  // Start the DAC explicitly with the exact format A2DP streams (rather
  // than trusting defaultConfig()) so it's never a mismatch guess.
  auto dac_cfg = out.defaultConfig();
  dac_cfg.sample_rate = 44100;
  dac_cfg.channels = 2;
  dac_cfg.bits_per_sample = 16;
  out.begin(dac_cfg);

  a2dp_sink.set_on_connection_state_changed(connection_state_changed);
  a2dp_sink.set_avrc_rn_playstatus_callback(avrc_rn_playstatus_callback);
  a2dp_sink.set_avrc_metadata_callback(avrc_metadata_callback);
  a2dp_sink.set_avrc_metadata_attribute_mask(ESP_AVRC_MD_ATTR_TITLE | ESP_AVRC_MD_ATTR_ARTIST);
  a2dp_sink.set_stream_reader(read_data_stream); // taps PCM audio for the FFT, alongside normal playback

  a2dp_sink.start("ESP32 Speaker");
}

void loop() {
  if (checkButtonPressed(btnVolUp)) {
    volume = min(100, volume + 10);
    a2dp_sink.set_volume(map(volume, 0, 100, 0, 127));
  }
  if (checkButtonPressed(btnVolDn)) {
    volume = max(0, volume - 10);
    a2dp_sink.set_volume(map(volume, 0, 100, 0, 127));
  }
  if (checkButtonPressed(btnPlay)) {
    if (isPlaying) {
      a2dp_sink.pause();
    } else {
      a2dp_sink.play();
    }
  }
  if (checkButtonPressed(btnMode)) {
    // Placeholder — AM/BT source switching isn't wired yet.
    // Once you add the 74HC4066 (or your chosen alternative), toggle its
    // control GPIO(s) here.
    Serial.println("Mode button pressed - AM switching not implemented yet");
  }

  // Run the FFT whenever a fresh buffer of samples has been captured
  if (fftBufferReady) {
    double mean = 0; // remove DC bias before windowing
    for (int i = 0; i < FFT_SAMPLES; i++) mean += fftCaptureBuffer[i];
    mean /= FFT_SAMPLES;
    for (int i = 0; i < FFT_SAMPLES; i++) {
      vReal[i] = (double)fftCaptureBuffer[i] - mean;
      vImag[i] = 0.0;
    }
    fftBufferReady = false; // release the capture buffer back to the audio callback

    FFT.windowing(FFTWindow::Hamming, FFTDirection::Forward);
    FFT.compute(FFTDirection::Forward);
    FFT.complexToMagnitude();
    computeBands();
  }

  static unsigned long lastDisplayUpdate = 0;
  if (millis() - lastDisplayUpdate > 50) {
    updateDisplay();
    lastDisplayUpdate = millis();
  }
}
