// ---------------------------------------------------------------------------
// F-4 Phantom / J79 scale-model lighting simulator
//
// Drives LEDs that stand in for a J79 turbojet's afterburner glow (red/yellow
// "engine cans") plus the aircraft's exterior lights (nav, beacons, strobes,
// landing light, cockpit glow, radome/refuel light). A single push button
// on FLIGHT_BUTTON kicks off one full unattended flight sequence — engine
// start, taxi, takeoff, cruise, a simulated combat/damage event, return, and
// shutdown — with the button also acting as an abort switch mid-sequence.
// ---------------------------------------------------------------------------

// --- Engine "can" LEDs: red/yellow simulate afterburner glow, blue is the
//     compressor/turbine shimmer at the exhaust core. Left/right = engine 1/2.
const byte LEFT_RED_PIN      = 9;
const byte RIGHT_RED_PIN     = 10;
const byte LEFT_YELLOW_PIN   = 5;
const byte RIGHT_YELLOW_PIN  = 3;
const byte BLUE_PIN          = 11;

// --- Airframe lighting
const byte NAV_PIN           = 4;   // navigation (position) lights
const byte JOINUP_PIN        = 8;   // formation join-up light
const byte RED_BEACON_PIN    = 2;   // rotating anti-collision beacon (fake PWM via beaconWrite)
const byte WHITE_BEACON_PIN  = 12;  // upper/lower white anti-collision strobe
const byte DORSAL_STROBE_PIN = A1;  // dorsal (top fuselage) strobe
const byte INTAKE_STROBE_PIN = A2;  // intake-mounted strobe
const byte COCKPIT_PIN       = A3;  // cockpit interior glow
const byte LANDING_PIN       = 6;   // landing/taxi light
const byte RADOME_PIN        = A5;  // radome/refuel probe light

// --- Control input
const byte FLIGHT_BUTTON     = A4;        // start/abort push button (active LOW, INPUT_PULLUP)
const unsigned long DEBOUNCE_MS = 35;     // settle time used by pressed()

// --- Engine glow brightness ranges (0-255 PWM). "IDLE" is ground idle power,
//     "MIL" is full military power. Ranges are randomly sampled each rotation
//     step to fake flicker; BLING values are the peak used during startup.
const byte RED_IDLE_MIN       = 50;
const byte RED_IDLE_MAX       = 150;
const byte RED_MIL_MIN        = 150;
const byte RED_MIL_MAX        = 255;
const byte RED_BLING_MAX      = 255;

const byte YELLOW_IDLE_MIN    = 50;
const byte YELLOW_IDLE_MAX    = 150;
const byte YELLOW_MIL_MIN     = 150;
const byte YELLOW_MIL_MAX     = 255;
const byte YELLOW_BLING_MAX   = 255;

const byte BLUE_IDLE_MIN  = 0;
const byte BLUE_IDLE_MAX  = 5;
const byte BLUE_MIL_MIN   = 0;
const byte BLUE_MIL_MAX   = 8;
const byte BLUE_BLING_MAX     = 100;

const byte LANDING_TAXI       = 10;   // dim landing light for taxi
const byte LANDING_FULL       = 255;  // full brightness for takeoff/approach

// --- Effect timing
const unsigned long ROTATION_MS         = 30;   // engine glow update interval (fake compressor rotation)
const unsigned long BLUE_FLICKER_MS     = 120;  // blue core shimmer update interval
const unsigned long STARTUP_STEP_MS     = 22;   // delay per brightness step during engine light-off ramps
const unsigned long SHUTDOWN_STEP_MS    = 150;  // delay per brightness step during engine spool-down ramps

const unsigned long BEACON_STEP_MS      = 25;   // slower updates
const unsigned long STROBE_FLASH_MS     = 25;   // how long a strobe stays lit per flash
const unsigned long STROBE_PERIOD_MS    = 1000; // time between strobe flashes
const unsigned long STROBE_OFFSET_MS    = 400;  // dorsal/intake strobes are offset so they don't flash together

const int STARTUP_FLICKER_COUNT         = 140;  // number of random flicker frames after engine 2 lights off
const unsigned long STARTUP_FLICKER_MIN = 20;
const unsigned long STARTUP_FLICKER_MAX = 70;
const int SHUTDOWN_FLICKER_COUNT        = 80;   // number of random flicker frames during engine shutdown

// --- Flight-sequence phase durations (real elapsed time, in ms)
const unsigned long TAXI_OUT_MS         = 60000UL;
const unsigned long TAKEOFF_MS          = 40000UL;
const unsigned long JOINUP_CLIMB_MS     = 15000UL;
const unsigned long FLIGHT_MS           = 120000UL;
const unsigned long COMBAT_MS           = 120000UL;
const unsigned long DAMAGE_CHECK_MS     = 30000UL;
const unsigned long RETURN_JOINUP_MS    = 10000UL;
const unsigned long RETURN_MS           = 90000UL;
const unsigned long APPROACH_MS         = 60000UL;
const unsigned long TAXI_IN_MS          = 40000UL;
const unsigned long MIL_POWER_TEST_MS   = 60000UL;
const unsigned long CHECKLIST_STEP_MS   = 5000UL;  // pause between pre-flight checklist light steps

// --- Random formation join-up light behavior (used during combat/damage phases)
const unsigned long JOINUP_MIN_MS       = 8000UL;   // shortest gap before next join-up flash
const unsigned long JOINUP_MAX_MS       = 20000UL;  // longest gap before next join-up flash
const unsigned long JOINUP_FLASH_MS     = 2500UL;   // how long the join-up light stays on per flash

// --- Timing state (last-updated timestamps, millis())
unsigned long lastRotation   = 0;
unsigned long lastBlue       = 0;
unsigned long lastBeaconStep = 0;
unsigned long lastDorsal     = 0;
unsigned long lastIntake     = 0;
unsigned long nextJoinup     = 0;
unsigned long joinupStart    = 0;

// Phase accumulators driving the sine-wave engine glow; left/right start out
// of phase with each other so the two engines don't pulse in lockstep.
float leftAngle  = 0;
float rightAngle = PI / 3;

// Rotating beacon software-PWM state (see beaconWrite/updateBeacon)
int  beaconLevel  = 0;
bool beaconRising = true;
bool beaconActive = false;

bool running        = false;  // true while a flight sequence is in progress
bool dorsalOn       = false;  // dorsal strobe currently lit
bool intakeOn       = false;  // intake strobe currently lit
bool joinupActive   = false;  // join-up light currently lit
bool abortRequested = false;  // set when the flight button is pressed mid-sequence

enum EngineMode { ENG_OFF, ENG_IDLE, ENG_MIL };
EngineMode engMode = ENG_OFF;

// Software PWM for the red beacon: RED_BEACON_PIN is a plain digital pin
// (no hardware PWM on that pin), so brightness is faked with a single short
// on/off burst proportional to `level` (0-255). Called repeatedly from
// updateBeacon() as beaconLevel ramps up and down, producing the rotating
// anti-collision beacon's fade-in/fade-out pulse.
void beaconWrite(int level) {
  if (level <= 0) {
    digitalWrite(RED_BEACON_PIN, LOW);
    return;
  }
  if (level >= 255) {
    digitalWrite(RED_BEACON_PIN, HIGH);
    return;
  }
  int onTime  = level / 10;
  int offTime = (255 - level) / 10;
  if (onTime  < 1) onTime  = 1;
  if (offTime < 1) offTime = 1;
  digitalWrite(RED_BEACON_PIN, HIGH);
  delayMicroseconds(onTime * 40);
  digitalWrite(RED_BEACON_PIN, LOW);
  delayMicroseconds(offTime * 40);
}

// Debounced button read. Blocks until the button is released once a press
// is confirmed, so callers get one clean "pressed" event per physical press.
bool pressed(byte pin) {
  if (digitalRead(pin) == LOW) {
    delay(DEBOUNCE_MS);
    if (digitalRead(pin) == LOW) {
      while (digitalRead(pin) == LOW) {}
      delay(DEBOUNCE_MS);
      return true;
    }
  }
  return false;
}

// Extinguishes every light and resets all animation state back to the
// aircraft's fully-shutdown condition. Used at the very end of a flight
// sequence and after an aborted one.
void allOff() {
  analogWrite(LEFT_RED_PIN,     0);
  analogWrite(RIGHT_RED_PIN,    0);
  analogWrite(LEFT_YELLOW_PIN,  0);
  analogWrite(RIGHT_YELLOW_PIN, 0);
  analogWrite(BLUE_PIN,         0);
  digitalWrite(NAV_PIN,          LOW);
  digitalWrite(JOINUP_PIN,       LOW);
  digitalWrite(RED_BEACON_PIN,   LOW);
  digitalWrite(WHITE_BEACON_PIN, LOW);
  digitalWrite(DORSAL_STROBE_PIN,LOW);
  digitalWrite(INTAKE_STROBE_PIN,LOW);
  digitalWrite(COCKPIT_PIN,      LOW);
  analogWrite(LANDING_PIN,       0);
  digitalWrite(RADOME_PIN,       LOW);
  beaconActive = false;
  beaconLevel  = 0;
  beaconRising = true;
  dorsalOn     = false;
  intakeOn     = false;
  joinupActive = false;
  engMode      = ENG_OFF;
}

void setEngineIdle() { engMode = ENG_IDLE; }
void setEngineMil()  { engMode = ENG_MIL;  }

// Continuously animates the running engines' glow. Called from every wait
// loop while an engine is on. Red/yellow brightness for each engine follows
// a sine wave (leftAngle/rightAngle advance at slightly different rates so
// the two engines drift in and out of phase) with random jitter layered on
// top for a flicker effect; yellow runs in anti-phase with red (0.5 - 0.5*s
// vs 0.5 + 0.5*s) so each can's core brightens as its edge dims. The blue
// core shimmer is updated independently on its own, slower cadence.
void updateEngines() {
  unsigned long now = millis();
  if (engMode == ENG_OFF) return;

  if (now - lastRotation >= ROTATION_MS) {
    lastRotation = now;

    leftAngle  += 0.15;
    rightAngle += 0.13;

    if (leftAngle  > 2 * PI) leftAngle  -= 2 * PI;
    if (rightAngle > 2 * PI) rightAngle -= 2 * PI;

    float ls = sin(leftAngle);
    float rs = sin(rightAngle);

    byte rMin = (engMode == ENG_MIL) ? RED_MIL_MIN    : RED_IDLE_MIN;
    byte rMax = (engMode == ENG_MIL) ? RED_MIL_MAX    : RED_IDLE_MAX;
    byte yMin = (engMode == ENG_MIL) ? YELLOW_MIL_MIN : YELLOW_IDLE_MIN;
    byte yMax = (engMode == ENG_MIL) ? YELLOW_MIL_MAX : YELLOW_IDLE_MAX;

    int leftRed     = constrain(rMin + (int)((rMax - rMin) * (0.5 + 0.5 * ls)) + random(-5, 6), 0, 255);
    int leftYellow  = constrain(yMin + (int)((yMax - yMin) * (0.5 - 0.5 * ls)) + random(-4, 5), 0, 255);
    int rightRed    = constrain(rMin + (int)((rMax - rMin) * (0.5 + 0.5 * rs)) + random(-5, 6), 0, 255);
    int rightYellow = constrain(yMin + (int)((yMax - yMin) * (0.5 - 0.5 * rs)) + random(-4, 5), 0, 255);

    analogWrite(LEFT_RED_PIN,     leftRed);
    analogWrite(LEFT_YELLOW_PIN,  leftYellow);
    analogWrite(RIGHT_RED_PIN,    rightRed);
    analogWrite(RIGHT_YELLOW_PIN, rightYellow);
  }

  if (now - lastBlue >= BLUE_FLICKER_MS) {
    lastBlue = now;
    byte bMin = (engMode == ENG_MIL) ? BLUE_MIL_MIN : BLUE_IDLE_MIN;
    byte bMax = (engMode == ENG_MIL) ? BLUE_MIL_MAX : BLUE_IDLE_MAX;
    analogWrite(BLUE_PIN, random(bMin, bMax + 1));
  }
}

// Ramps the red anti-collision beacon up and down (via beaconWrite) in
// triangle-wave fashion whenever beaconActive is set. No-ops (and forces the
// pin low) when the beacon has been switched off.
void updateBeacon() {
  if (!beaconActive) {
    digitalWrite(RED_BEACON_PIN, LOW);
    return;
  }

  unsigned long now = millis();

  if (now - lastBeaconStep >= BEACON_STEP_MS) {
    lastBeaconStep = now;

    if (beaconRising) {
      beaconLevel += 4;   // slower rise
      if (beaconLevel >= 255) {
        beaconLevel  = 255;
        beaconRising = false;
      }
    } else {
      beaconLevel -= 4;   // same speed as rise — symmetric
      if (beaconLevel <= 0) {
        beaconLevel  = 0;
        beaconRising = true;
      }
    }

    beaconWrite(beaconLevel);
  }
}

// Toggles the dorsal and intake strobes on their own independent
// STROBE_PERIOD_MS/STROBE_FLASH_MS cycles. Their start times are offset
// (see STROBE_OFFSET_MS) so the two strobes flash at different moments
// instead of in unison.
void updateStrobes() {
  unsigned long now = millis();

  if (!dorsalOn && now - lastDorsal >= STROBE_PERIOD_MS) {
    digitalWrite(DORSAL_STROBE_PIN, HIGH);
    lastDorsal = now;
    dorsalOn   = true;
  }
  if (dorsalOn && now - lastDorsal >= STROBE_FLASH_MS) {
    digitalWrite(DORSAL_STROBE_PIN, LOW);
    dorsalOn = false;
  }

  if (!intakeOn && now - lastIntake >= STROBE_PERIOD_MS) {
    digitalWrite(INTAKE_STROBE_PIN, HIGH);
    lastIntake = now;
    intakeOn   = true;
  }
  if (intakeOn && now - lastIntake >= STROBE_FLASH_MS) {
    digitalWrite(INTAKE_STROBE_PIN, LOW);
    intakeOn = false;
  }
}

// Randomly flashes the formation join-up light at intervals between
// JOINUP_MIN_MS and JOINUP_MAX_MS, each flash lasting JOINUP_FLASH_MS, with
// a quick dorsal-strobe blink at the start of each flash. Only used during
// the combat and damage-check phases to simulate a wingman signaling.
void updateJoinup() {
  unsigned long now = millis();

  if (!joinupActive && now >= nextJoinup) {
    joinupActive = true;
    joinupStart  = now;
    digitalWrite(JOINUP_PIN, HIGH);
    digitalWrite(DORSAL_STROBE_PIN, HIGH);
    delay(STROBE_FLASH_MS);
    digitalWrite(DORSAL_STROBE_PIN, LOW);
  }

  if (joinupActive && now - joinupStart >= JOINUP_FLASH_MS) {
    joinupActive = false;
    digitalWrite(JOINUP_PIN, LOW);
    nextJoinup = now + random(JOINUP_MIN_MS, JOINUP_MAX_MS);
  }
}

// Busy-waits for `ms` while keeping the engine glow, beacon, and strobe
// animations running. Doubles as the abort check for every phase: if the
// flight button is pressed during the wait, it sets abortRequested and
// returns false immediately so the calling phase can bail out early.
bool waitMs(unsigned long ms) {
  unsigned long start = millis();
  while (millis() - start < ms) {
    updateEngines();
    updateBeacon();
    updateStrobes();
    if (pressed(FLIGHT_BUTTON)) {
      abortRequested = true;
      return false;
    }
  }
  return true;
}

// Randomly snaps every light to an on/off or PWM-random state for one
// frame. Used during the damage-check phase to simulate battle-damage
// electrical faults ("everything's glitching").
void allSystemsFlicker() {
  digitalWrite(COCKPIT_PIN,      random(0, 2) ? HIGH : LOW);
  digitalWrite(NAV_PIN,          random(0, 2) ? HIGH : LOW);
  digitalWrite(JOINUP_PIN,       random(0, 2) ? HIGH : LOW);
  digitalWrite(WHITE_BEACON_PIN, random(0, 2) ? HIGH : LOW);
  digitalWrite(RED_BEACON_PIN,   random(0, 2) ? HIGH : LOW);
  digitalWrite(DORSAL_STROBE_PIN,random(0, 2) ? HIGH : LOW);
  digitalWrite(INTAKE_STROBE_PIN,random(0, 2) ? HIGH : LOW);
  digitalWrite(RADOME_PIN,       random(0, 2) ? HIGH : LOW);
  analogWrite(RIGHT_RED_PIN,    random(0, RED_MIL_MAX + 1));
  analogWrite(RIGHT_YELLOW_PIN, random(0, YELLOW_MIL_MAX + 1));
  analogWrite(BLUE_PIN,         random(0, BLUE_MIL_MAX + 1));
}

// Kills every light except the cockpit glow (forced on) — the "total
// electrical failure, emergency power only" look that follows the
// flicker effect in the damage-check phase.
void allSystemsOut() {
  digitalWrite(NAV_PIN,          LOW);
  digitalWrite(JOINUP_PIN,       LOW);
  digitalWrite(WHITE_BEACON_PIN, LOW);
  digitalWrite(RED_BEACON_PIN,   LOW);
  digitalWrite(DORSAL_STROBE_PIN,LOW);
  digitalWrite(INTAKE_STROBE_PIN,LOW);
  digitalWrite(RADOME_PIN,       LOW);
  analogWrite(RIGHT_RED_PIN,    0);
  analogWrite(RIGHT_YELLOW_PIN, 0);
  analogWrite(BLUE_PIN,         0);
  beaconActive = false;
  beaconLevel  = 0;
  digitalWrite(COCKPIT_PIN,     HIGH);
}

// Cockpit power-up: flickering cockpit light settles on steady, then the
// checklist brings up radome, nav, and white beacon lights one at a time
// with a pause between each (simulating the pilot's pre-start walk-through).
void phaseStartup() {
  for (int i = 0; i < 8; i++) {
    digitalWrite(COCKPIT_PIN, HIGH);
    delay(random(30, 100));
    digitalWrite(COCKPIT_PIN, LOW);
    delay(random(20, 80));
  }
  digitalWrite(COCKPIT_PIN, HIGH);
  if (!waitMs(CHECKLIST_STEP_MS)) return;

  digitalWrite(RADOME_PIN, HIGH);
  if (!waitMs(CHECKLIST_STEP_MS)) return;

  digitalWrite(NAV_PIN, HIGH);
  if (!waitMs(CHECKLIST_STEP_MS)) return;

  digitalWrite(WHITE_BEACON_PIN, HIGH);
  lastDorsal = millis();
  lastIntake = millis() + STROBE_OFFSET_MS;
  if (!waitMs(CHECKLIST_STEP_MS)) return;
}

// Pilot's exterior lights check: join-up light and landing light both
// brought up full for 5s, then switched back off (along with nav/beacon)
// to confirm they work before engine start.
void phasePreflightLightsCheck() {
  digitalWrite(JOINUP_PIN, HIGH);
  analogWrite(LANDING_PIN, LANDING_FULL);
  if (!waitMs(5000)) return;
  digitalWrite(JOINUP_PIN,       LOW);
  digitalWrite(WHITE_BEACON_PIN, LOW);
  digitalWrite(NAV_PIN,          LOW);
  analogWrite(LANDING_PIN,       0);
  if (!waitMs(2000)) return;
}

// Switches on the rotating red anti-collision beacon ahead of engine start.
void phaseBeaconOn() {
  beaconActive   = true;
  beaconLevel    = 0;
  beaconRising   = true;
  lastBeaconStep = millis();
  if (!waitMs(3000)) return;
}

// Left engine (#1) light-off sequence: starter spins the blue core up to a
// steady shimmer, then the red can ramps to full "bling" brightness (fuel
// ignition), then yellow ramps up while red eases back down toward its
// resting glow — mimicking the bright ignition flash settling into a
// steady idle flame. Each ramp checks abortRequested so a button press can
// interrupt mid-startup.
void phaseEngine1Start() {
  for (int level = 0; level <= 60; level++) {
    analogWrite(BLUE_PIN, constrain(level + random(-3, 4), 0, 255));
    delay(STARTUP_STEP_MS);
    if (abortRequested) return;
  }
  delay(200);
  for (int level = 0; level <= RED_BLING_MAX; level++) {
    analogWrite(LEFT_RED_PIN, constrain(level + random(-8, 9), 0, 255));
    analogWrite(BLUE_PIN,     constrain(50 + random(-15, 16), 0, 255));
    delay(STARTUP_STEP_MS);
    if (abortRequested) return;
  }
  delay(200);
  for (int level = 0; level <= YELLOW_BLING_MAX; level++) {
    analogWrite(LEFT_YELLOW_PIN, constrain(level + random(-8, 9), 0, 255));
    analogWrite(LEFT_RED_PIN,    constrain(RED_BLING_MAX - (level / 3) + random(-5, 6), 0, 255));
    delay(STARTUP_STEP_MS);
    if (abortRequested) return;
  }
  if (!waitMs(3000)) return;
}

// Right engine (#2) light-off: same blue/red/yellow ignition ramp as engine
// 1, followed by a shared randomized flicker burst on both engines settling
// into idle, then engine mode switches to ENG_IDLE so updateEngines() takes
// over the ongoing glow. Finishes by bringing up nav and white beacon lights.
void phaseEngine2Start() {
  for (int level = 0; level <= 60; level++) {
    analogWrite(BLUE_PIN, constrain(level + random(-3, 4), 0, 255));
    delay(STARTUP_STEP_MS);
    if (abortRequested) return;
  }
  delay(200);
  for (int level = 0; level <= RED_BLING_MAX; level++) {
    analogWrite(RIGHT_RED_PIN, constrain(level + random(-8, 9), 0, 255));
    analogWrite(BLUE_PIN,      constrain(50 + random(-15, 16), 0, 255));
    delay(STARTUP_STEP_MS);
    if (abortRequested) return;
  }
  delay(200);
  for (int level = 0; level <= YELLOW_BLING_MAX; level++) {
    analogWrite(RIGHT_YELLOW_PIN, constrain(level + random(-8, 9), 0, 255));
    analogWrite(RIGHT_RED_PIN,    constrain(RED_BLING_MAX - (level / 3) + random(-5, 6), 0, 255));
    delay(STARTUP_STEP_MS);
    if (abortRequested) return;
  }
  for (int i = 0; i < STARTUP_FLICKER_COUNT; i++) {
    analogWrite(LEFT_RED_PIN,     random(RED_IDLE_MAX,    RED_BLING_MAX + 1));
    analogWrite(RIGHT_RED_PIN,    random(RED_IDLE_MAX,    RED_BLING_MAX + 1));
    analogWrite(LEFT_YELLOW_PIN,  random(YELLOW_IDLE_MAX, YELLOW_BLING_MAX + 1));
    analogWrite(RIGHT_YELLOW_PIN, random(YELLOW_IDLE_MAX, YELLOW_BLING_MAX + 1));
    analogWrite(BLUE_PIN,         random(BLUE_IDLE_MIN,   BLUE_BLING_MAX + 1));
    delay(random(STARTUP_FLICKER_MIN, STARTUP_FLICKER_MAX + 1));
    if (abortRequested) return;
  }
  setEngineIdle();
  lastRotation = millis();
  lastBlue     = millis();

  if (!waitMs(CHECKLIST_STEP_MS)) return;
  digitalWrite(NAV_PIN, HIGH);
  if (!waitMs(CHECKLIST_STEP_MS)) return;
  digitalWrite(WHITE_BEACON_PIN, HIGH);
  lastDorsal = millis();
  lastIntake = millis() + STROBE_OFFSET_MS;
}

// Ground run-up: engines pushed to military power briefly, then back to idle.
void phaseMilPowerTest() {
  setEngineMil();
  if (!waitMs(MIL_POWER_TEST_MS)) return;
  setEngineIdle();
  if (!waitMs(3000)) return;
}

// Taxiing to the runway with the landing light at its dim taxi setting.
void phaseTaxiOut() {
  analogWrite(LANDING_PIN, LANDING_TAXI);
  if (!waitMs(TAXI_OUT_MS)) return;
}

// Takeoff roll at full landing-light brightness and military engine power;
// once airborne the landing light is switched off and the join-up light
// comes on for the climb-out.
void phaseTakeoff() {
  analogWrite(LANDING_PIN, LANDING_FULL);
  setEngineMil();
  if (!waitMs(TAKEOFF_MS)) return;
  analogWrite(LANDING_PIN, 0);
  digitalWrite(JOINUP_PIN, HIGH);
  if (!waitMs(JOINUP_CLIMB_MS)) return;
  digitalWrite(JOINUP_PIN, LOW);
}

// Level cruise flight at military power.
void phaseFlight() {
  setEngineMil();
  if (!waitMs(FLIGHT_MS)) return;
}

// Combat engagement: nav/beacon lights doused for a low-visibility profile
// while engines keep running and the join-up light flashes intermittently
// (updateJoinup) to simulate wingman signaling.
void phaseCombat() {
  digitalWrite(NAV_PIN,          LOW);
  digitalWrite(JOINUP_PIN,       LOW);
  digitalWrite(WHITE_BEACON_PIN, LOW);
  beaconActive = false;
  beaconLevel  = 0;
  digitalWrite(RED_BEACON_PIN,   LOW);
  nextJoinup   = millis() + random(JOINUP_MIN_MS, JOINUP_MAX_MS);
  joinupActive = false;

  unsigned long combatStart = millis();
  while (millis() - combatStart < COMBAT_MS) {
    updateEngines();
    updateJoinup();
    if (pressed(FLIGHT_BUTTON)) {
      abortRequested = true;
      return;
    }
  }
}

// Simulated battle damage and recovery: left engine sputters and cuts out,
// every system flickers/fails, goes fully dark, then each system is brought
// back online one at a time (cockpit, radome, nav, beacon, white beacon)
// with a flicker-then-hold restart, and finally the left engine is
// re-ignited to military power while join-up flashing resumes.
void phaseDamageCheck() {
  // Left engine sputtering: alternate between a high random flare and a
  // near-zero dip to simulate the engine losing and regaining fuel flow.
  for (int i = 0; i < 8; i++) {
    analogWrite(LEFT_RED_PIN,    random(RED_MIL_MAX - 20, RED_MIL_MAX + 1));
    analogWrite(LEFT_YELLOW_PIN, random(YELLOW_MIL_MAX - 20, YELLOW_MIL_MAX + 1));
    delay(random(20, 60));
    analogWrite(LEFT_RED_PIN,    random(0, 30));
    analogWrite(LEFT_YELLOW_PIN, random(0, 20));
    delay(random(20, 60));
  }

  analogWrite(LEFT_RED_PIN,    0);
  analogWrite(LEFT_YELLOW_PIN, 0);
  delay(300);

  // Random electrical-fault flicker across all systems for 5-10s, then a
  // total blackout (cockpit-only emergency lighting) for 5s.
  unsigned long flickerStart    = millis();
  unsigned long flickerDuration = random(5000, 10001);
  while (millis() - flickerStart < flickerDuration) {
    allSystemsFlicker();
    delay(random(30, 100));
  }

  allSystemsOut();
  delay(5000);

  // Systems reboot sequence: cockpit lighting flickers back on and holds.
  for (int i = 0; i < 10; i++) {
    digitalWrite(COCKPIT_PIN, HIGH);
    delay(random(50, 150));
    digitalWrite(COCKPIT_PIN, LOW);
    delay(random(30, 100));
  }
  digitalWrite(COCKPIT_PIN, HIGH);
  delay(CHECKLIST_STEP_MS);

  // Radome light reboot.
  for (int i = 0; i < 5; i++) {
    digitalWrite(RADOME_PIN, HIGH);
    delay(random(40, 100));
    digitalWrite(RADOME_PIN, LOW);
    delay(random(30, 80));
  }
  digitalWrite(RADOME_PIN, HIGH);
  delay(CHECKLIST_STEP_MS);

  // Nav light reboot.
  for (int i = 0; i < 6; i++) {
    digitalWrite(NAV_PIN, HIGH);
    delay(random(50, 120));
    digitalWrite(NAV_PIN, LOW);
    delay(random(30, 80));
  }
  digitalWrite(NAV_PIN, HIGH);
  delay(CHECKLIST_STEP_MS);

  // Red beacon reboot (no flicker step — just re-enabled).
  beaconActive   = true;
  beaconLevel    = 0;
  beaconRising   = true;
  lastBeaconStep = millis();
  delay(CHECKLIST_STEP_MS);

  // White beacon reboot, then dorsal/intake strobes are re-armed.
  for (int i = 0; i < 4; i++) {
    digitalWrite(WHITE_BEACON_PIN, HIGH);
    delay(random(40, 100));
    digitalWrite(WHITE_BEACON_PIN, LOW);
    delay(random(30, 80));
  }
  digitalWrite(WHITE_BEACON_PIN, HIGH);
  lastDorsal = millis();
  lastIntake = millis() + STROBE_OFFSET_MS;
  delay(CHECKLIST_STEP_MS);

  // Left engine re-light directly to military power (right engine was
  // never damaged and is left running throughout via updateEngines()).
  setEngineMil();
  lastRotation = millis();
  lastBlue     = millis();
  delay(500);

  for (int level = 0; level <= 40; level++) {
    analogWrite(BLUE_PIN, constrain(level + random(-3, 4), 0, 255));
    delay(STARTUP_STEP_MS);
    if (abortRequested) return;
  }
  delay(200);
  for (int level = 0; level <= RED_MIL_MAX; level++) {
    analogWrite(LEFT_RED_PIN, constrain(level + random(-8, 9), 0, 255));
    delay(STARTUP_STEP_MS);
    if (abortRequested) return;
  }
  delay(200);
  for (int level = 0; level <= YELLOW_MIL_MAX; level++) {
    analogWrite(LEFT_YELLOW_PIN, constrain(level + random(-8, 9), 0, 255));
    delay(STARTUP_STEP_MS);
    if (abortRequested) return;
  }

  nextJoinup   = millis() + 2000;
  joinupActive = false;

  unsigned long damageStart = millis();
  while (millis() - damageStart < DAMAGE_CHECK_MS) {
    updateEngines();
    updateBeacon();
    updateStrobes();
    updateJoinup();
    if (pressed(FLIGHT_BUTTON)) {
      abortRequested = true;
      return;
    }
  }
  digitalWrite(JOINUP_PIN, LOW);
}

// Egress from the combat area back toward base: nav and white beacon lights
// back on, with a brief join-up light flash at the start of the leg.
void phaseReturn() {
  digitalWrite(NAV_PIN,          HIGH);
  digitalWrite(WHITE_BEACON_PIN, HIGH);
  lastDorsal = millis();
  lastIntake = millis() + STROBE_OFFSET_MS;

  digitalWrite(JOINUP_PIN, HIGH);
  if (!waitMs(RETURN_JOINUP_MS)) return;
  digitalWrite(JOINUP_PIN, LOW);

  if (!waitMs(RETURN_MS - RETURN_JOINUP_MS)) return;
}

// Final approach and landing: landing light at full, engines throttled back
// to idle for the descent.
void phaseApproach() {
  analogWrite(LANDING_PIN, LANDING_FULL);
  setEngineIdle();
  if (!waitMs(APPROACH_MS)) return;
}

// Taxi back to parking: landing light dims to taxi setting, then join-up,
// landing, white beacon, and nav lights are switched off one at a time
// (with a checklist pause between each) as the aircraft parks.
void phaseTaxiIn() {
  analogWrite(LANDING_PIN, LANDING_TAXI);
  if (!waitMs(TAXI_IN_MS)) return;

  digitalWrite(JOINUP_PIN,       LOW);
  delay(CHECKLIST_STEP_MS);
  analogWrite(LANDING_PIN,       0);
  delay(CHECKLIST_STEP_MS);
  digitalWrite(WHITE_BEACON_PIN, LOW);
  delay(CHECKLIST_STEP_MS);
  digitalWrite(NAV_PIN,          LOW);
}

// Full engine shutdown and cooldown: idle glow ramps down to off, cabin/
// radome lights switch off, then a 20s "hot start" red glow lingers at idle
// brightness (residual heat) before fading to zero, followed by a random
// dying-ember flicker on both cans and the blue core, and finally a slow
// 10s afterglow fade on the red cans down to fully dark and cold.
void phaseShutdown() {
  engMode = ENG_OFF;

  // Fade yellow/red cans down from idle level to off.
  for (int level = YELLOW_IDLE_MIN; level >= 0; level--) {
    analogWrite(LEFT_YELLOW_PIN,  constrain(level + random(-2, 3), 0, 255));
    analogWrite(RIGHT_YELLOW_PIN, constrain(level + random(-2, 3), 0, 255));
    analogWrite(LEFT_RED_PIN,     constrain(RED_IDLE_MIN + random(-3, 4), 0, 255));
    analogWrite(RIGHT_RED_PIN,    constrain(RED_IDLE_MIN + random(-3, 4), 0, 255));
    delay(SHUTDOWN_STEP_MS);
  }

  beaconActive = false;
  beaconLevel  = 0;
  digitalWrite(RED_BEACON_PIN, LOW);
  delay(CHECKLIST_STEP_MS);

  digitalWrite(COCKPIT_PIN, LOW);
  delay(CHECKLIST_STEP_MS);

  digitalWrite(RADOME_PIN, LOW);

  // 20s "hot start" glow: residual heat holds the red cans near idle
  // brightness even though the engines are already off.
  unsigned long hotStart = millis();
  while (millis() - hotStart < 20000) {
    analogWrite(LEFT_RED_PIN,  constrain(RED_IDLE_MIN + random(-3, 4), 0, 255));
    analogWrite(RIGHT_RED_PIN, constrain(RED_IDLE_MIN + random(-3, 4), 0, 255));
    delay(100);
  }

  // Fade the hot-start glow the rest of the way to zero.
  for (int level = RED_IDLE_MIN; level >= 0; level--) {
    analogWrite(LEFT_RED_PIN,  constrain(level + random(-3, 4), 0, 255));
    analogWrite(RIGHT_RED_PIN, constrain(level + random(-3, 4), 0, 255));
    delay(SHUTDOWN_STEP_MS);
  }

  // Dying-ember flicker across both cans and the blue core.
  for (int i = 0; i < SHUTDOWN_FLICKER_COUNT; i++) {
    analogWrite(LEFT_RED_PIN,     random(0, 20));
    analogWrite(RIGHT_RED_PIN,    random(0, 20));
    analogWrite(LEFT_YELLOW_PIN,  random(0, 10));
    analogWrite(RIGHT_YELLOW_PIN, random(0, 10));
    analogWrite(BLUE_PIN,         random(0, BLUE_IDLE_MAX + 1));
    delay(random(STARTUP_FLICKER_MIN, STARTUP_FLICKER_MAX + 1));
  }

  for (int level = BLUE_IDLE_MIN; level >= 0; level--) {
    analogWrite(BLUE_PIN, constrain(level + random(-2, 3), 0, 255));
    delay(SHUTDOWN_STEP_MS);
  }

  analogWrite(LEFT_YELLOW_PIN,  0);
  analogWrite(RIGHT_YELLOW_PIN, 0);
  analogWrite(BLUE_PIN,         0);

  // Final 10s linear afterglow fade of the red cans down to fully cold.
  unsigned long afterglowStart = millis();
  while (millis() - afterglowStart < 10000) {
    byte afterglowLevel = map(millis() - afterglowStart, 0, 10000, RED_IDLE_MIN, 0);
    analogWrite(LEFT_RED_PIN,  constrain(afterglowLevel + random(-2, 3), 0, 255));
    analogWrite(RIGHT_RED_PIN, constrain(afterglowLevel + random(-2, 3), 0, 255));
    delay(100);
  }

  analogWrite(LEFT_RED_PIN,  0);
  analogWrite(RIGHT_RED_PIN, 0);
}

// Runs the full flight profile phase by phase, in order. Every phase checks
// abortRequested (set by a mid-sequence flight-button press inside waitMs
// or the phases' own button polling); if a phase aborts, the sequence jumps
// straight to phaseShutdown() and allOff() so the aircraft always ends in a
// safe, fully-off state rather than stopping mid-animation.
void runFlightSequence() {
  running         = true;
  abortRequested  = false;

  phaseStartup();              if (abortRequested) { phaseShutdown(); allOff(); running = false; return; }
  phasePreflightLightsCheck(); if (abortRequested) { phaseShutdown(); allOff(); running = false; return; }
  phaseBeaconOn();             if (abortRequested) { phaseShutdown(); allOff(); running = false; return; }
  phaseEngine1Start();         if (abortRequested) { phaseShutdown(); allOff(); running = false; return; }
  phaseEngine2Start();         if (abortRequested) { phaseShutdown(); allOff(); running = false; return; }
  phaseMilPowerTest();         if (abortRequested) { phaseShutdown(); allOff(); running = false; return; }
  phaseTaxiOut();              if (abortRequested) { phaseShutdown(); allOff(); running = false; return; }
  phaseTakeoff();              if (abortRequested) { phaseShutdown(); allOff(); running = false; return; }
  phaseFlight();               if (abortRequested) { phaseShutdown(); allOff(); running = false; return; }
  phaseCombat();               if (abortRequested) { phaseShutdown(); allOff(); running = false; return; }
  phaseDamageCheck();          if (abortRequested) { phaseShutdown(); allOff(); running = false; return; }
  phaseReturn();               if (abortRequested) { phaseShutdown(); allOff(); running = false; return; }
  phaseApproach();             if (abortRequested) { phaseShutdown(); allOff(); running = false; return; }
  phaseTaxiIn();               if (abortRequested) { phaseShutdown(); allOff(); running = false; return; }
  phaseShutdown();

  allOff();
  running = false;
}

void setup() {
  // Configure every light pin as an output and the flight button as a
  // pulled-up input (button reads LOW when pressed).
  pinMode(LEFT_RED_PIN,      OUTPUT);
  pinMode(RIGHT_RED_PIN,     OUTPUT);
  pinMode(LEFT_YELLOW_PIN,   OUTPUT);
  pinMode(RIGHT_YELLOW_PIN,  OUTPUT);
  pinMode(BLUE_PIN,          OUTPUT);
  pinMode(NAV_PIN,           OUTPUT);
  pinMode(JOINUP_PIN,        OUTPUT);
  pinMode(RED_BEACON_PIN,    OUTPUT);
  pinMode(WHITE_BEACON_PIN,  OUTPUT);
  pinMode(DORSAL_STROBE_PIN, OUTPUT);
  pinMode(INTAKE_STROBE_PIN, OUTPUT);
  pinMode(COCKPIT_PIN,       OUTPUT);
  pinMode(LANDING_PIN,       OUTPUT);
  pinMode(RADOME_PIN,        OUTPUT);
  pinMode(FLIGHT_BUTTON,     INPUT_PULLUP);

  allOff();

  // Seed the PRNG from noise on a floating analog pin so flicker patterns
  // vary between power-ups instead of repeating identically every run.
  randomSeed(analogRead(A0));
}

// A button press starts one complete flight sequence; presses while a
// sequence is already running are instead treated as the abort signal
// inside waitMs()/the phase functions themselves.
void loop() {
  if (pressed(FLIGHT_BUTTON) && !running) {
    runFlightSequence();
  }
}