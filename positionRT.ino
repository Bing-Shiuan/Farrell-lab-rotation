/*
  Real-time port of your Python logic to Arduino.

  Inputs:
    chA  -> A0
    chB  -> A1
    hall -> A2

  Notes:
    - Uses analogRead() + thresholding, matching your Python (chA > th, chB > th, hall < hall_th).
    - Quadrature update matches:
        chA_plus_minus = (chA*2)-1
        chB_diff = chB - prevB
        chA_prod = chA_plus_minus * chB_diff
        chB_plus_minus = (chB*2)-1
        chA_diff = chA - prevA
        chB_prod = -chB_plus_minus * chA_diff
        position += (chA_prod + chB_prod)

    - "lin2circ" simplified to: offset = pos - pos_at_last_hall_rising_edge
      and loop_len is measured between hall rising edges.
*/

const uint8_t PIN_CHA  = A1;
const uint8_t PIN_CHB  = A0;
const uint8_t PIN_HALL = A2;

// === Thresholds (tune these) ===
const int ENC_TH  = 1000;  // like your Encoder2pos(..., th=1000)
const int HALL_TH = 500;   // set this to your hall_th

// === Scaling from your return line ===
// return (position/4) * (44.8/256)
const float CIRCUMFERENCE_CM = 44.8f;
const float PULSES_PER_REV   = 256.0f;
const float CM_PER_COUNT = (CIRCUMFERENCE_CM / PULSES_PER_REV) / 4.0f;

// State for encoder "diff" logic
int prevA = 0;  // previous binary state (0/1)
int prevB = 0;

// Integrated position in "counts" (same units as your Python 'position' before scaling)
long posCount = 0;

// Hall edge detection
int prevHall = 0;          // previous hall binary (0/1)
bool haveHallRef = false;  // have we seen first rising edge yet?
long lastHallPos = 0;      // posCount at last hall rising edge

// Loop length tracking
long lastLoopLenCounts = 0;

unsigned long lastPrintMs = 0;
const unsigned long PRINT_EVERY_MS = 20; // ~50 Hz printing

static inline int binEnc(int analogValue) {
  return (analogValue > ENC_TH) ? 1 : 0;
}

// Python: hall_binary = np.where(hall < hall_th, 1, 0)
static inline int binHall(int analogValue) {
  return (analogValue < HALL_TH) ? 1 : 0;
}

void setup() {
  Serial.begin(115200);

  pinMode(PIN_CHA, INPUT);
  pinMode(PIN_CHB, INPUT);
  pinMode(PIN_HALL, INPUT);

  // Initialize previous states from first reads
  prevA = binEnc(analogRead(PIN_CHA));
  prevB = binEnc(analogRead(PIN_CHB));
  prevHall = binHall(analogRead(PIN_HALL));

  // Optional header (comment out if using Serial Plotter)
  // Serial.println("pos_cm\toffset_cm\tlooplen_cm\thall");
}

void loop() {
  // --- Read + threshold ---
  int a = binEnc(analogRead(PIN_CHA));   // chA binary
  int b = binEnc(analogRead(PIN_CHB));   // chB binary
  int h = binHall(analogRead(PIN_HALL)); // hall binary

  // --- Encoder2pos step (diff-based) ---
  int chA_plus_minus = (a * 2) - 1;  // 0->-1, 1->+1
  int chB_diff = b - prevB;          // -1, 0, +1
  int chA_prod = chA_plus_minus * chB_diff;

  int chB_plus_minus = (b * 2) - 1;
  int chA_diff = a - prevA;
  int chB_prod = -chB_plus_minus * chA_diff;

  posCount += (long)(chA_prod + chB_prod);

  prevA = a;
  prevB = b;

  // --- lin2circ: detect hall rising edge (0->1) ---
  int hall_def = h - prevHall;
  if (hall_def == 1) {
    if (!haveHallRef) {
      haveHallRef = true;
      lastHallPos = posCount;
      lastLoopLenCounts = 0;
    } else {
      lastLoopLenCounts = posCount - lastHallPos; // loop length in counts
      lastHallPos = posCount;
    }
  }
  prevHall = h;

  // --- Compute outputs ---
  float pos_cm = posCount * CM_PER_COUNT;

  long offsetCounts;
  if (!haveHallRef) {
    // Python case: if no loop_start, pos_offset1 = pos
    offsetCounts = posCount;
  } else {
    // offset within current loop
    offsetCounts = posCount - lastHallPos;
  }
  float offset_cm = offsetCounts * CM_PER_COUNT;
  float looplen_cm = lastLoopLenCounts * CM_PER_COUNT;

  // --- Print periodically ---
  unsigned long now = millis();
  if (now - lastPrintMs >= PRINT_EVERY_MS) {
    lastPrintMs = now;

    // Format suited for Serial Plotter: 3 signals + hall state
    Serial.print(pos_cm, 6);
    Serial.print('\t');
    Serial.print(offset_cm, 6);
    Serial.print('\t');
    Serial.print(analogRead(PIN_HALL), 6);
    Serial.print('\t');
    Serial.println(h);
  }
}
