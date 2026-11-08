// SPDX-License-Identifier: MIT
// Copyright (c) 2026 Rich Dunkel
//
// Host tests: IRQ pin path and low-power card detection (included by test_core.cpp).

struct ActivationResult {
    Status st = Status::Ok;
    CardA card;
    int irqReads = 0;
    int frames = 0;
};

// One activation with the card present / muted, with or without the pin.
ActivationResult activateWith(bool wired, bool activeHigh, int stuck, int mute) {
    FakeCardA card;
    const uint8_t uid[4] = {0x04, 0xBB, 0x6A, 0x04};
    std::memcpy(card.uid, uid, 4);
    FakeHal hal;
    hal.card = &card;
    hal.irqWired = wired;
    hal.irqStuck = stuck;
    hal.muteA = mute;
    // Answers arrive after the wait has started, so the pin (not the error
    // check's IRQ_STATUS read) is what reports them.
    hal.replyDelayA = 3;
    hal.eeprom[spec::eeprom::IRQ_PIN_CONFIG] = activeHigh ? 0x01 : 0x00;
    Pn5180 nfc(hal);
    Iso14443a reader(nfc);
    ActivationResult r;
    EXPECT(nfc.begin() == Status::Ok);
    EXPECT(nfc.hardReset() == Status::Ok);
    EXPECT(nfc.irqPinInUse() == wired);
    EXPECT(reader.fieldOn() == Status::Ok);
    const int frames0 = hal.frames;
    r.st = reader.activate(r.card);
    r.frames = hal.frames - frames0;
    r.irqReads = hal.irqReads;
    EXPECT(hal.locks == 0);
    return r;
}

void testIrqPin() {
    // Pin path == polling path: same Status and card data, both polarities.
    for (int pol = 0; pol < 2; ++pol) {
        const ActivationResult poll = activateWith(false, pol == 1, -1, 0);
        const ActivationResult pin = activateWith(true, pol == 1, -1, 0);
        EXPECT(poll.st == Status::Ok && pin.st == Status::Ok);
        EXPECT(pin.card.uidLen == poll.card.uidLen &&
               std::memcmp(pin.card.uid, poll.card.uid, poll.card.uidLen) == 0);
        EXPECT(pin.card.sak == poll.card.sak && pin.card.atqa == poll.card.atqa);
        EXPECT(pin.irqReads > 0 && poll.irqReads == 0);
    }
    // No card: NoResponse on both paths, after the same timeout.
    {
        const ActivationResult poll = activateWith(false, false, -1, 1000);
        const ActivationResult pin = activateWith(true, false, -1, 1000);
        EXPECT(poll.st == Status::NoResponse && pin.st == Status::NoResponse);
        // The pin path does not poll IRQ_STATUS during the wait: fewer frames.
        EXPECT(pin.frames < poll.frames);
    }
    // Stuck "inactive" (active-low pin stuck high): a received frame is
    // found by the final IRQ_STATUS check -> IrqPinError, not NoResponse.
    EXPECT(activateWith(true, false, 1, 0).st == Status::IrqPinError);
    // Stuck "active" (active-low pin stuck low): degrades to polling, same result.
    EXPECT(activateWith(true, false, 0, 0).st == Status::Ok);
    // Timeout with a working pin and no card.
    EXPECT(activateWith(true, true, -1, 1000).st == Status::NoResponse);

    // Self-test.
    for (int stuck = -1; stuck <= 1; ++stuck) {
        FakeHal hal;
        hal.irqWired = true;
        hal.irqStuck = stuck;
        Pn5180 nfc(hal);
        EXPECT(nfc.begin() == Status::Ok);
        EXPECT(nfc.hardReset() == Status::Ok);
        EXPECT(nfc.irqPinConfigKnown() && !nfc.irqPinActiveHigh());
        const Status s = nfc.checkIrqPin();
        EXPECT(stuck < 0 ? s == Status::Ok : s == Status::IrqPinError);
        EXPECT(hal.regs[spec::reg::IRQ_ENABLE] == 0);         // restored
        EXPECT(hal.regs[spec::reg::TIMER2_CONFIG] == 0);      // timer stopped
        EXPECT((hal.regs[spec::reg::IRQ_STATUS] & spec::irq::TIMER2) == 0);
        EXPECT(hal.eepromBytesWritten == 0);                  // 0x1A only read
    }
    {
        FakeHal hal;  // no IRQ line
        Pn5180 nfc(hal);
        EXPECT(nfc.begin() == Status::Ok);
        EXPECT(nfc.hardReset() == Status::Ok);
        EXPECT(nfc.checkIrqPin() == Status::Unsupported);
        EXPECT(!nfc.irqPinInUse());
    }
}

// Fresh chip with the pin wired (active low unless `pinConfig` says
// otherwise), the field on and an optional Type A card.
struct PinBench {
    FakeCardA card;
    FakeHal hal;
    Pn5180 nfc{hal};
    Iso14443a reader{nfc};
    explicit PinBench(uint8_t pinConfig, bool withCard) {
        const uint8_t uid[4] = {0x04, 0xBB, 0x6A, 0x04};
        std::memcpy(card.uid, uid, 4);
        hal.card = withCard ? &card : nullptr;
        hal.irqWired = true;
        hal.eeprom[spec::eeprom::IRQ_PIN_CONFIG] = pinConfig;
        EXPECT(nfc.begin() == Status::Ok);
        EXPECT(nfc.hardReset() == Status::Ok);
        EXPECT(nfc.irqPinInUse());
        EXPECT(reader.fieldOn() == Status::Ok);
    }
    // WUPA as a 7-bit short frame (ISO/IEC 14443-3); returns the elapsed fake ms.
    uint32_t wupa(Status& st, size_t& rxLen, uint32_t timeoutMs) {
        const uint8_t tx = iso14443a::WUPA;
        uint8_t rx[16] = {};
        uint8_t bits = 0;
        const uint32_t t0 = hal.millis();
        st = nfc.transceive(&tx, 1, 7, rx, sizeof(rx), rxLen, bits, timeoutMs);
        return hal.millis() - t0;
    }
};

void testIrqReview() {
    // 1. Auto-clear-on-read (DS Table 51, 0x1A bit 1): RX_IRQ is already
    //    read (and cleared) by SEND_DATA's error check before the wait
    //    starts, so the pin never goes active. Must still be Ok, at once.
    for (int pass = 0; pass < 2; ++pass) {
        PinBench b(spec::eeprom::kIrqPinAutoClear, true);
        if (pass == 1) {
            CardA c;
            EXPECT(b.reader.activate(c) == Status::Ok);
        }
        Status st = Status::Ok;
        size_t n = 0;
        const uint32_t ms = b.wupa(st, n, 50);
        EXPECT(st == Status::Ok && n == 2);
        EXPECT(ms < 50);  // no timeout
        EXPECT(b.hal.locks == 0);
    }
    // 2. RX_IRQ arrives between the final pin sample and the IRQ_STATUS
    //    read: re-sampled pin is active -> Ok, not IrqPinError.
    {
        PinBench b(0x00, false);
        b.hal.rxOnStatusReadAt = b.hal.millis() + 100;  // after the pre-wait commands
        Status st = Status::IrqPinError;
        size_t n = 99;
        b.wupa(st, n, 200);
        EXPECT(st == Status::Ok && n == 0);
        EXPECT(b.hal.rxOnStatusReadAt == 0);  // the hook did fire
    }
    // 3. A user write to IRQ_ENABLE invalidates the setIrqEnable() cache.
    for (int how = 0; how < 3; ++how) {
        PinBench b(0x00, true);
        CardA c;
        EXPECT(b.reader.activate(c) == Status::Ok);  // cache: RX routed
        Status w = Status::ChipError;
        if (how == 0) w = b.nfc.writeRegister(spec::reg::IRQ_ENABLE, 0);
        if (how == 1) w = b.nfc.writeRegisterAndMask(spec::reg::IRQ_ENABLE, 0);
        if (how == 2) w = b.nfc.writeRegisterOrMask(spec::reg::IRQ_ENABLE, spec::irq::TIMER2);
        EXPECT(w == Status::Ok);
        Status st = Status::Ok;
        size_t n = 0;
        b.wupa(st, n, 50);
        EXPECT(st == Status::Ok && n == 2);
        EXPECT(b.hal.regs[spec::reg::IRQ_ENABLE] == spec::irq::RX);
    }
    // 8. Timer2 model: a wrong reload or a timer that never starts makes the
    //    self-test fail; the right reload passes only after time elapsed.
    for (int fault = 0; fault < 3; ++fault) {
        FakeHal hal;
        hal.irqWired = true;
        hal.timer2ReloadOffset = fault == 1 ? 1u : 0u;
        hal.timer2IgnoreStart = fault == 2;
        Pn5180 nfc(hal);
        EXPECT(nfc.begin() == Status::Ok);
        EXPECT(nfc.hardReset() == Status::Ok);
        const uint32_t t0 = hal.millis();
        const Status s = nfc.checkIrqPin();
        EXPECT(fault == 0 ? s == Status::Ok : s == Status::IrqTimeout);
        if (fault == 0) {
            EXPECT(hal.millis() - t0 >= 1);  // the IRQ needed fake time to appear
        }
        EXPECT(hal.regs[spec::reg::TIMER2_CONFIG] == 0);
        EXPECT(hal.regs[spec::reg::IRQ_ENABLE] == 0);
    }
}

void testLpcd() {
    FakeHal hal;
    hal.irqWired = true;
    hal.eeprom[spec::eeprom::LPCD_REFERENCE_VALUE] = 0xA8;     // RFU bits 7:4 set, gear 8
    hal.eeprom[spec::eeprom::LPCD_REFVAL_GPO_CONTROL] = 0x10;  // GPO bit 4 set, auto
    Pn5180 nfc(hal);
    Lpcd lpcd(nfc);
    EXPECT(nfc.begin() == Status::Ok);
    EXPECT(nfc.hardReset() == Status::Ok);

    LpcdSettings cfg;
    cfg.threshold = 0x20;
    cfg.fieldOnTime = 0x10;
    cfg.calibration = LpcdCalibration::Auto;
    cfg.gear = 5;
    size_t written = 99;
    EXPECT(lpcd.configure(cfg, written) == Status::Ok);
    EXPECT(written == 3);  // 0x34 gear, 0x36, 0x37; 0x38 already auto
    EXPECT(hal.eeprom[0x34] == 0xA5 && hal.eeprom[0x36] == 0x10 && hal.eeprom[0x37] == 0x20);
    EXPECT(hal.eeprom[0x38] == 0x10);
    EXPECT(!nfc.eepromWriteEnabled());  // guard restored
    const int before = hal.eepromBytesWritten;
    EXPECT(lpcd.configure(cfg, written) == Status::Ok);
    EXPECT(written == 0 && hal.eepromBytesWritten == before);  // second boot: no writes
    cfg.calibration = LpcdCalibration::Self;
    EXPECT(lpcd.configure(cfg, written) == Status::Ok && written == 1);
    EXPECT(hal.eeprom[0x38] == 0x11);
    LpcdSettings back;
    EXPECT(lpcd.readSettings(back) == Status::Ok);
    EXPECT(back.gear == 5 && back.threshold == 0x20 && back.fieldOnTime == 0x10 &&
           back.calibration == LpcdCalibration::Self);
    cfg.gear = 16;
    EXPECT(lpcd.configure(cfg, written) == Status::InvalidArgument && written == 0);
    cfg.gear = 5;
    cfg.calibration = LpcdCalibration::Auto;
    EXPECT(lpcd.configure(cfg, written) == Status::Ok && written == 1);

    // Reserved calibration modes 10b / 11b in EEPROM 0x38.  DS Table 73
    for (uint8_t reserved = 0x12; reserved <= 0x13; ++reserved) {
        hal.eeprom[0x38] = reserved;
        LpcdSettings rs;
        EXPECT(lpcd.readSettings(rs) == Status::MalformedData);
        cfg.calibration = reserved == 0x12 ? LpcdCalibration::Auto : LpcdCalibration::Self;
        EXPECT(lpcd.configure(cfg, written) == Status::Ok && written == 1);
        EXPECT(hal.eeprom[0x38] == (reserved == 0x12 ? 0x10 : 0x11));  // GPO bit kept
        EXPECT(lpcd.readSettings(rs) == Status::Ok && rs.calibration == cfg.calibration);
    }
    cfg.calibration = LpcdCalibration::Auto;
    EXPECT(lpcd.configure(cfg, written) == Status::Ok && written == 1);

    // Wake-up counter 1..1023.  DS Table 21, Figure 40
    EXPECT(lpcd.start(0) == Status::InvalidArgument);
    EXPECT(lpcd.start(1024) == Status::InvalidArgument);
    EXPECT(lpcd.start(2690) == Status::InvalidArgument);
    EXPECT(!lpcd.armed() && hal.lastSwitchModeLen == 0);
    EXPECT(lpcd.start(1) == Status::Ok);
    EXPECT(hal.lastSwitchMode[2] == 0x01 && hal.lastSwitchMode[3] == 0x00);
    EXPECT(lpcd.cancel() == Status::Ok);

    // Start: SWITCH_MODE frame 0B 01 <counter LSB first>, field switched
    // off before.  DS Table 18, 21, 38
    EXPECT(nfc.rfOn() == Status::Ok);
    const int rfOffs0 = hal.rfOffs;
    EXPECT(lpcd.start(1023) == Status::Ok);
    EXPECT(hal.rfOffs == rfOffs0 + 1);
    EXPECT((hal.regs[spec::reg::RF_STATUS] & spec::rfstatus::TX_RF_STATUS) == 0);
    EXPECT(lpcd.armed() && hal.lowPower);
    EXPECT(hal.lastSwitchModeLen == 4 && hal.lastSwitchMode[0] == 0x0B &&
           hal.lastSwitchMode[1] == 0x01 && hal.lastSwitchMode[2] == 0xFF &&
           hal.lastSwitchMode[3] == 0x03);
    // While armed: no instruction may be sent.
    uint32_t v = 0;
    EXPECT(nfc.readRegister(spec::reg::RF_STATUS, v) == Status::LowPowerActive);
    // Not through the raw transport either: no NSS activity.  DS 11.8.10
    {
        const uint8_t f[2] = {spec::cmd::READ_REGISTER, spec::reg::RF_STATUS};
        uint8_t r[4] = {};
        const int frames0 = hal.frames;
        EXPECT(nfc.transport().command(f, sizeof(f), r, sizeof(r)) == Status::LowPowerActive);
        EXPECT(nfc.transport().command(f, sizeof(f)) == Status::LowPowerActive);
        EXPECT(nfc.transport().commandNoCompletion(f, sizeof(f)) == Status::LowPowerActive);
        EXPECT(hal.frames == frames0 && hal.framesInLowPower == 0 && hal.lowPower);
    }
    LpcdWake wake;
    EXPECT(lpcd.waitForWake(50, wake) == Status::IrqTimeout);
    EXPECT(lpcd.armed() && hal.lowPower && hal.framesInLowPower == 0);
    // Card detected: LPCD_IRQ on the (non-maskable) pin.
    hal.lpcdWake();
    EXPECT(lpcd.waitForWake(50, wake) == Status::Ok);
    EXPECT(wake.reason == LpcdWakeReason::CardDetected);
    EXPECT((wake.irqStatus & spec::irq::LPCD) != 0);
    EXPECT(wake.elapsedMs >= 50);
    EXPECT(!lpcd.armed() && hal.framesInLowPower == 0);
    EXPECT(nfc.readRegister(spec::reg::RF_STATUS, v) == Status::Ok);  // usable again
    {
        const uint8_t f[2] = {spec::cmd::READ_REGISTER, spec::reg::RF_STATUS};
        uint8_t r[4] = {};
        EXPECT(nfc.transport().command(f, sizeof(f), r, sizeof(r)) == Status::Ok);
    }
    EXPECT(lpcd.waitForWake(10, wake) == Status::InvalidArgument);    // not armed

    // Other wake reason (e.g. GENERAL_ERROR, also non-maskable).
    EXPECT(lpcd.start(100) == Status::Ok);
    hal.lowPower = false;
    hal.regs[spec::reg::IRQ_STATUS] |= spec::irq::GENERAL_ERROR;
    EXPECT(lpcd.waitForWake(10, wake) == Status::Ok);
    EXPECT(wake.reason == LpcdWakeReason::Other);

    // Cancel = reset.
    EXPECT(lpcd.start(1) == Status::Ok);
    EXPECT(lpcd.cancel() == Status::Ok);
    EXPECT(!lpcd.armed() && !hal.lowPower && hal.framesInLowPower == 0);
    EXPECT(!nfc.transport().lowPowerGuard());  // reset releases the guard

    // Self calibration reads AGC_REF_CONFIG before SWITCH_MODE.
    cfg.calibration = LpcdCalibration::Self;
    EXPECT(lpcd.configure(cfg, written) == Status::Ok);
    EXPECT(lpcd.start(10) == Status::Ok && lpcd.armed());
    EXPECT(lpcd.cancel() == Status::Ok);

    // Stuck pin: refused before entering LPCD.
    hal.irqStuck = 0;
    EXPECT(lpcd.start(10) == Status::IrqPinError && !lpcd.armed());
    hal.irqStuck = -1;

    // Without an IRQ line LPCD is unsupported.
    FakeHal noPin;
    Pn5180 nfc2(noPin);
    Lpcd lpcd2(nfc2);
    EXPECT(nfc2.begin() == Status::Ok);
    EXPECT(nfc2.hardReset() == Status::Ok);
    EXPECT(lpcd2.start(100) == Status::Unsupported);
    EXPECT(noPin.lastSwitchModeLen == 0);
}
