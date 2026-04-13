// door_monitor.ino — TF-Luna door state monitor for Arduino Nano ESP32
// Reads TF-Luna LiDAR via UART1, reports state changes as JSON over USB serial.
// Supports serial commands: "Status" (query state), "Ping" (health check).

#include <Arduino.h>

// ── Pin configuration (Arduino Nano ESP32 Dx macros) ──────────────────────
#define LIDAR_RX_PIN  D5   // GPIO8 — receives data FROM TF-Luna TXD
#define LIDAR_TX_PIN  D6   // GPIO9 — transmits data TO TF-Luna RXD

// ── Hysteresis thresholds (centimeters) ───────────────────────────────────
#define CLOSED_THRESHOLD_CM  150
#define OPEN_THRESHOLD_CM    170

// ── Debounce: require new state to persist this long before confirming ────
#define DEBOUNCE_MS  5000

// ── Minimum valid distance — ignore readings below this ───────────────────
#define MIN_VALID_DISTANCE_CM  120

// ── Minimum signal strength for a valid reading ───────────────────────────
#define MIN_SIGNAL_STRENGTH  100

// ── Door state enumeration ────────────────────────────────────────────────
enum DoorState : uint8_t {
  DOOR_UNKNOWN = 0,
  DOOR_CLOSED  = 1,
  DOOR_OPEN    = 2
};

// ── Global state ──────────────────────────────────────────────────────────
DoorState confirmedState   = DOOR_UNKNOWN;
DoorState pendingState     = DOOR_UNKNOWN;
unsigned long pendingStart = 0;
int lastValidDistance      = 0;
int lastSignalStrength     = 0;
unsigned long bootTime     = 0;

// ── Debug distance printing ───────────────────────────────────────────────
unsigned long lastPrintMs  = 0;

// ── Forward declarations ──────────────────────────────────────────────────
bool      readTFLuna(HardwareSerial &ser, int &dist, int &strength, float &tempC);
DoorState evaluateHysteresis(int distanceCm);
void      updateDoorState(int distanceCm, int strength);
void      emitStateChange(DoorState newState, int distanceCm, int strength);
const char* stateToString(DoorState s);
void      processSerialCommands();

// ══════════════════════════════════════════════════════════════════════════
void setup() {
  // USB serial — for JSON output to host PC
  Serial.begin(115200);
  while (!Serial && millis() < 3000) { /* wait up to 3 s for USB CDC */ }

  // UART1 — connected to TF-Luna at 115200 8N1
  Serial1.begin(115200, SERIAL_8N1, LIDAR_RX_PIN, LIDAR_TX_PIN);

  bootTime = millis();
  Serial.println("{\"event\":\"boot\",\"message\":\"Door monitor started\"}");
}

// ══════════════════════════════════════════════════════════════════════════
void loop() {
  // Check for incoming commands from host
  processSerialCommands();

  int dist      = 0;
  int strength  = 0;
  float tempC   = 0.0;

  // Process all available frames (TF-Luna sends at 100 Hz = every 10 ms)
  while (readTFLuna(Serial1, dist, strength, tempC)) {
    // Reject low-confidence readings
    if (strength < MIN_SIGNAL_STRENGTH || strength == 65535) {
      continue;
    }
    // Ignore readings closer than minimum valid distance
    if (dist < MIN_VALID_DISTANCE_CM) {
      continue;
    }

    lastValidDistance   = dist;
    lastSignalStrength = strength;
    updateDoorState(dist, strength);

    // Debug: print distance every 500 ms for calibration
    // Comment out once thresholds are calibrated
    // if (millis() - lastPrintMs >= 500) {
    //  lastPrintMs = millis();
    //  Serial.printf("{\"event\":\"debug\",\"distance_cm\":%d,\"signal_strength\":%d}\r\n", dist, strength);
    // }
  }
}

// ── TF-Luna UART parser (non-blocking state machine) ─────────────────────
bool readTFLuna(HardwareSerial &ser, int &dist, int &strength, float &tempC) {
  static uint8_t buf[9];
  static uint8_t idx = 0;

  while (ser.available()) {
    uint8_t b = ser.read();

    if (idx == 0) {
      if (b == 0x59) buf[idx++] = b;
    }
    else if (idx == 1) {
      if (b == 0x59) {
        buf[idx++] = b;
      } else {
        idx = 0;
      }
    }
    else {
      buf[idx++] = b;
      if (idx == 9) {
        idx = 0;

        uint8_t checksum = 0;
        for (uint8_t i = 0; i < 8; i++) checksum += buf[i];
        if (checksum != buf[8]) {
          continue;
        }

        dist     = buf[2] | (buf[3] << 8);
        strength = buf[4] | (buf[5] << 8);
        int16_t rawTemp = buf[6] | (buf[7] << 8);
        tempC    = rawTemp / 8.0f - 256.0f;
        return true;
      }
    }
  }
  return false;
}

// ── Hysteresis evaluation ─────────────────────────────────────────────────
DoorState evaluateHysteresis(int distanceCm) {
  if (distanceCm <= CLOSED_THRESHOLD_CM) return DOOR_CLOSED;
  if (distanceCm >= OPEN_THRESHOLD_CM)   return DOOR_OPEN;
  return confirmedState;
}

// ── Debounced state update ────────────────────────────────────────────────
void updateDoorState(int distanceCm, int strength) {
  DoorState rawState = evaluateHysteresis(distanceCm);

  if (rawState != confirmedState) {
    if (rawState != pendingState) {
      pendingState = rawState;
      pendingStart = millis();
    }
    else if (millis() - pendingStart >= DEBOUNCE_MS) {
      confirmedState = rawState;
      emitStateChange(confirmedState, distanceCm, strength);
    }
  } else {
    pendingState = confirmedState;
  }
}

// ── Emit JSON state-change event over USB serial ──────────────────────────
void emitStateChange(DoorState newState, int distanceCm, int strength) {
  char json[192];
  snprintf(json, sizeof(json),
    "{\"event\":\"state_change\","
     "\"state\":\"%s\","
     "\"distance_cm\":%d,"
     "\"signal_strength\":%d,"
     "\"uptime_ms\":%lu}\r\n",
    stateToString(newState),
    distanceCm,
    strength,
    millis() - bootTime
  );
  Serial.print(json);
}

// ── State enum to string ──────────────────────────────────────────────────
const char* stateToString(DoorState s) {
  switch (s) {
    case DOOR_CLOSED:  return "closed";
    case DOOR_OPEN:    return "open";
    default:           return "unknown";
  }
}

// ── Process incoming USB serial commands ──────────────────────────────────
void processSerialCommands() {
  static char cmdBuf[32];
  static uint8_t cmdIdx = 0;

  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (cmdIdx > 0) {
        cmdBuf[cmdIdx] = '\0';

        // Normalize to lowercase for comparison
        for (uint8_t i = 0; i < cmdIdx; i++) {
          cmdBuf[i] = tolower(cmdBuf[i]);
        }

        if (strcmp(cmdBuf, "status") == 0) {
          char json[192];
          snprintf(json, sizeof(json),
            "{\"event\":\"status_query\","
             "\"state\":\"%s\","
             "\"distance_cm\":%d,"
             "\"signal_strength\":%d,"
             "\"uptime_ms\":%lu}\r\n",
            stateToString(confirmedState),
            lastValidDistance,
            lastSignalStrength,
            millis() - bootTime
          );
          Serial.print(json);

        } else if (strcmp(cmdBuf, "ping") == 0) {
          Serial.print("{\"event\":\"pong\"}\r\n");

        } else {
          Serial.printf("{\"event\":\"error\",\"message\":\"unknown command: %s\"}\r\n", cmdBuf);
        }

        cmdIdx = 0;
      }
    } else if (cmdIdx < sizeof(cmdBuf) - 1) {
      cmdBuf[cmdIdx++] = c;
    }
  }
}
