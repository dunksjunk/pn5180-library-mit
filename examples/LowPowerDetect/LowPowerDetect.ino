// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Rich Dunkel
//
// Low-power card detection (LPCD) with the PN5180. Reference test bed:
//   ESP32-D0WD-V3 dev board + generic PN5180 module
//   SCK 18, MISO 19, MOSI 23, NSS 16, BUSY 5, RST 17, IRQ 15.
// The IRQ pin is REQUIRED here: while LPCD runs no SPI instruction may be
// sent (DS Table 18) - any NSS activity would even end LPCD (DS 11.8.10) -
// so the wake can only be seen on the IRQ line (LPCD_IRQ is non-maskable,
// DS 11.3.2.2). With PIN_IRQ = -1, Lpcd::start() returns Unsupported.
// GPIO 15 is an ESP32 strapping pin (MTDO): a low level at reset only
// silences the ROM boot log. The PN5180 IRQ idles inactive (the Basic
// example prints the polarity from EEPROM IRQ_PIN_CONFIG, DS Table 51).
//
// Flow:
//   setup: reset, IRQ pin self-test, LPCD EEPROM settings written only where
//          they differ (the line "LPCD config: N EEPROM bytes written" shows
//          0 from the second boot on - EEPROM has limited write endurance).
//   loop:  "LPCD armed" -> chip in LPCD; every 10 s without a wake
//          "LPCD waiting 10 s"; on a wake one line
//            LPCD wake: card detected (after 1234 ms)
//          or "LPCD wake: other (IRQ_STATUS 0x...)", then one ISO 14443A
//          activation and one ISO 15693 inventory print the UIDs found,
//          and LPCD is armed again.
//
// Calibration: when LPCD starts, the PN5180 measures a reference AGC value
// with the field on and whatever is on the antenna at that moment (auto
// calibration with the EEPROM gear, DS 11.8.10 Figure 40). Each cycle it
// wakes only if (reference AGC - measured AGC) > LPCD_THRESHOLD, i.e. when
// the antenna is loaded MORE than at calibration time. So LPCD detects a
// CHANGE, not presence: cards lying on the antenna when it is armed are part
// of the reference and do not wake it; adding a card (or metal) does. The
// data sheet documents only that direction (Figure 40). Measured on the
// reference test bed (three stacked cards, 2026-11-08): removing the top
// card from the stack woke LPCD as well, as did a hand near the antenna, so
// treat any wake as "something changed" and poll for cards. After every wake
// this sketch re-arms with the cards then present as the new reference.

#include <SPI.h>

#include <PN5180.h>
#include <PN5180ArduinoHal.h>

namespace {

constexpr int8_t PIN_SCK = 18;
constexpr int8_t PIN_MISO = 19;
constexpr int8_t PIN_MOSI = 23;
constexpr int8_t PIN_NSS = 16;
constexpr int8_t PIN_BUSY = 5;
constexpr int8_t PIN_RST = 17;
constexpr int8_t PIN_IRQ = 15;  // required for LPCD (-1 -> Unsupported)

// 1 MHz: well below the 7 Mbit/s maximum (DS 11.4.1).
constexpr uint32_t SPI_CLOCK_HZ = 1000000u;

// Standby time between two detection cycles, 1..1023 ms (DS Table 21 says
// 1..2690, but DS Figure 40 shows values above 0x3FF replaced by 0x3FF).
constexpr uint16_t WAKEUP_COUNTER_MS = 500;
// AGC drop that counts as "card added" (EEPROM 0x37, DS Table 73). Larger =
// less sensitive / fewer false wakes. Application choice, not from the DS.
constexpr uint8_t LPCD_THRESHOLD = 0x20;
constexpr uint32_t STATUS_EVERY_MS = 10000;
// Pause after a wake before reading: LPCD reacts within one cycle, often
// while the card (and the hand holding it) is still moving. Measured on the
// test bed: the first 14443A activation right after a hand-made wake often
// got no answer, a read a few hundred ms later always worked. Application
// choice, not from the DS.
constexpr uint32_t SETTLE_AFTER_WAKE_MS = 300;

const pn5180::ArduinoPins kPins = {PIN_SCK, PIN_MISO, PIN_MOSI, PIN_NSS,
                                   PIN_BUSY, PIN_RST, PIN_IRQ};

pn5180::ArduinoHal hal(SPI, kPins, SPI_CLOCK_HZ);
pn5180::Pn5180 nfc(hal);
pn5180::Lpcd lpcd(nfc);
pn5180::Iso14443a readerA(nfc);
pn5180::Iso15693 readerV(nfc);
bool ready = false;

void printHex2(uint8_t b) {
    if (b < 0x10) {
        Serial.print('0');
    }
    Serial.print(b, HEX);
}

void printUid(const char* label, const uint8_t* uid, size_t n) {
    Serial.print(label);
    for (size_t i = 0; i < n; ++i) {
        Serial.print(' ');
        printHex2(uid[i]);
    }
    Serial.println();
}

void error(const char* what, pn5180::Status s) {
    Serial.print("ERROR: ");
    Serial.print(what);
    Serial.print(": ");
    Serial.println(pn5180::statusToString(s));
}

bool initReader() {
    pn5180::Status s = nfc.hardReset();
    if (!pn5180::ok(s)) {
        error("hard reset", s);
        return false;
    }
    s = nfc.checkIrqPin();
    Serial.print("IRQ pin: ");
    Serial.println(pn5180::statusToString(s));
    if (!pn5180::ok(s)) {
        error("IRQ pin self-test", s);
        return false;
    }
    pn5180::LpcdSettings cfg;
    s = lpcd.readSettings(cfg);
    if (s == pn5180::Status::MalformedData) {
        // Reserved calibration mode in EEPROM 0x38 (DS Table 73); the other
        // fields are valid and configure() below overwrites the mode.
        Serial.println("LPCD EEPROM: reserved calibration mode, will be rewritten");
        s = pn5180::Status::Ok;
    }
    if (!pn5180::ok(s)) {
        error("read LPCD settings", s);
        return false;
    }
    Serial.print("LPCD EEPROM: gear ");
    Serial.print(cfg.gear);
    Serial.print(", field-on ");
    Serial.print(static_cast<uint32_t>(cfg.fieldOnTime) * 8u);  // x 8 us  DS Table 73
    Serial.print(" us, threshold 0x");
    printHex2(cfg.threshold);
    Serial.print(", mode ");
    Serial.println(cfg.calibration == pn5180::LpcdCalibration::Self ? "self" : "auto");

    // Keep the module's gear and field-on time; set threshold and auto
    // calibration. Only differing bytes are written.
    cfg.threshold = LPCD_THRESHOLD;
    cfg.calibration = pn5180::LpcdCalibration::Auto;
    size_t written = 0;
    s = lpcd.configure(cfg, written);
    Serial.print("LPCD config: ");
    Serial.print(static_cast<unsigned>(written));
    Serial.println(" EEPROM bytes written");
    if (!pn5180::ok(s)) {
        error("LPCD configure", s);
        return false;
    }
    return true;
}

// One ISO 14443A activation and one ISO 15693 inventory after a wake.
void readCards() {
    pn5180::Status s = readerA.fieldOn();
    if (pn5180::ok(s)) {
        pn5180::CardA a;
        s = readerA.activateWithRetry(a);
        if (pn5180::ok(s)) {
            printUid("  ISO14443A UID:", a.uid, a.uidLen);
            (void)readerA.halt();
        } else if (s != pn5180::Status::NoResponse) {
            Serial.print("  ISO14443A: ");
            Serial.println(pn5180::statusToString(s));
        }
    }
    const pn5180::Status off = readerA.fieldOff();
    if (!pn5180::ok(s) && pn5180::requiresRecovery(s)) {
        error("ISO14443A", s);
    }
    if (!pn5180::ok(off)) {
        error("RF off", off);
    }

    s = readerV.fieldOn();
    if (pn5180::ok(s)) {
        static pn5180::InventoryResult inv;
        s = readerV.inventory16(inv);
        for (uint8_t i = 0; i < inv.count; ++i) {
            printUid("  ISO15693 UID:", inv.cards[i].uid, sizeof(inv.cards[i].uid));
        }
    }
    const pn5180::Status offV = readerV.fieldOff();
    if (!pn5180::ok(s) && pn5180::requiresRecovery(s)) {
        error("ISO15693", s);
    }
    if (!pn5180::ok(offV)) {
        error("RF off", offV);
    }
}

}  // namespace

void setup() {
    Serial.begin(115200);
    delay(200);
    Serial.println();
    Serial.println("PN5180-MIT LowPowerDetect example");
    if (!pn5180::ok(nfc.begin())) {
        Serial.println("ERROR: HAL initialisation failed (check pin map / SPI clock).");
        return;
    }
    ready = initReader();
}

void loop() {
    if (!ready) {
        delay(5000);
        ready = initReader();
        return;
    }
    if (!lpcd.armed()) {
        const pn5180::Status s = lpcd.start(WAKEUP_COUNTER_MS);
        if (!pn5180::ok(s)) {
            error("LPCD start", s);
            ready = false;
            return;
        }
        Serial.println("LPCD armed");
    }
    pn5180::LpcdWake wake;
    const pn5180::Status s = lpcd.waitForWake(STATUS_EVERY_MS, wake);
    if (s == pn5180::Status::IrqTimeout) {
        Serial.println("LPCD waiting 10 s");
        return;
    }
    if (!pn5180::ok(s)) {
        error("LPCD wake", s);
        ready = false;
        return;
    }
    if (wake.reason == pn5180::LpcdWakeReason::CardDetected) {
        Serial.print("LPCD wake: card detected (after ");
        Serial.print(wake.elapsedMs);
        Serial.println(" ms)");
    } else {
        Serial.print("LPCD wake: other (IRQ_STATUS 0x");
        Serial.print(wake.irqStatus, HEX);
        Serial.println(')');
    }
    delay(SETTLE_AFTER_WAKE_MS);
    readCards();
}
