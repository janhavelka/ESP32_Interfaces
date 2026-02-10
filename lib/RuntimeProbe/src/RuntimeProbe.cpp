#include "RuntimeProbe.h"

#include <ctype.h>
#include <errno.h>
#include <limits.h>
#include <string.h>
#include <Wire.h>
#include <SPI.h>
#include "soc/soc_caps.h"

#ifndef RUNTIMEPROBE_MAX_CONSOLE_BYTES_PER_UPDATE
#define RUNTIMEPROBE_MAX_CONSOLE_BYTES_PER_UPDATE 256
#endif

#ifndef RUNTIMEPROBE_MAX_UART_TX_BYTES_PER_UPDATE
#define RUNTIMEPROBE_MAX_UART_TX_BYTES_PER_UPDATE 256
#endif

#ifndef RUNTIMEPROBE_MAX_UART_RX_BYTES_PER_UPDATE
#define RUNTIMEPROBE_MAX_UART_RX_BYTES_PER_UPDATE 256
#endif

#ifndef RUNTIMEPROBE_MAX_UART_TX_BUDGET_BYTES
#define RUNTIMEPROBE_MAX_UART_TX_BUDGET_BYTES 4096
#endif

#ifndef RUNTIMEPROBE_I2C_MIN_FREQ_HZ
#define RUNTIMEPROBE_I2C_MIN_FREQ_HZ 1000
#endif

#ifndef RUNTIMEPROBE_I2C_MAX_FREQ_HZ
#define RUNTIMEPROBE_I2C_MAX_FREQ_HZ 1000000
#endif

#ifndef RUNTIMEPROBE_I2C_MIN_TIMEOUT_MS
#define RUNTIMEPROBE_I2C_MIN_TIMEOUT_MS 1
#endif

#ifndef RUNTIMEPROBE_I2C_MAX_TIMEOUT_MS
#define RUNTIMEPROBE_I2C_MAX_TIMEOUT_MS 1000
#endif

#ifndef RUNTIMEPROBE_JTAG_TDO_REPORT_INTERVAL_MS
#define RUNTIMEPROBE_JTAG_TDO_REPORT_INTERVAL_MS 100
#endif

static_assert(RUNTIMEPROBE_MAX_CONSOLE_BYTES_PER_UPDATE > 0,
              "RUNTIMEPROBE_MAX_CONSOLE_BYTES_PER_UPDATE must be > 0");
static_assert(RUNTIMEPROBE_MAX_UART_TX_BYTES_PER_UPDATE > 0,
              "RUNTIMEPROBE_MAX_UART_TX_BYTES_PER_UPDATE must be > 0");
static_assert(RUNTIMEPROBE_MAX_UART_RX_BYTES_PER_UPDATE > 0,
              "RUNTIMEPROBE_MAX_UART_RX_BYTES_PER_UPDATE must be > 0");
static_assert(RUNTIMEPROBE_MAX_UART_TX_BUDGET_BYTES > 0,
              "RUNTIMEPROBE_MAX_UART_TX_BUDGET_BYTES must be > 0");
static_assert(RUNTIMEPROBE_I2C_MIN_FREQ_HZ > 0, "RUNTIMEPROBE_I2C_MIN_FREQ_HZ must be > 0");
static_assert(RUNTIMEPROBE_I2C_MAX_FREQ_HZ >= RUNTIMEPROBE_I2C_MIN_FREQ_HZ,
              "RUNTIMEPROBE_I2C_MAX_FREQ_HZ must be >= RUNTIMEPROBE_I2C_MIN_FREQ_HZ");
static_assert(RUNTIMEPROBE_I2C_MIN_TIMEOUT_MS > 0, "RUNTIMEPROBE_I2C_MIN_TIMEOUT_MS must be > 0");
static_assert(RUNTIMEPROBE_I2C_MAX_TIMEOUT_MS >= RUNTIMEPROBE_I2C_MIN_TIMEOUT_MS,
              "RUNTIMEPROBE_I2C_MAX_TIMEOUT_MS must be >= RUNTIMEPROBE_I2C_MIN_TIMEOUT_MS");
static_assert(RUNTIMEPROBE_I2C_MAX_TIMEOUT_MS <= 0xFFFF,
              "RUNTIMEPROBE_I2C_MAX_TIMEOUT_MS must fit in uint16_t");
static_assert(RUNTIMEPROBE_JTAG_TDO_REPORT_INTERVAL_MS > 0,
              "RUNTIMEPROBE_JTAG_TDO_REPORT_INTERVAL_MS must be > 0");

static const size_t kMaxIoBytes = 128;
static const size_t kMaxConsoleBytesPerUpdate =
    static_cast<size_t>(RUNTIMEPROBE_MAX_CONSOLE_BYTES_PER_UPDATE);
static const size_t kMaxUartTxBytesPerUpdate =
    static_cast<size_t>(RUNTIMEPROBE_MAX_UART_TX_BYTES_PER_UPDATE);
static const size_t kMaxUartRxBytesPerUpdate =
    static_cast<size_t>(RUNTIMEPROBE_MAX_UART_RX_BYTES_PER_UPDATE);
static const uint64_t kMicrosPerSecond = 1000000ULL;
static const uint64_t kMaxUartTxBudget =
    static_cast<uint64_t>(RUNTIMEPROBE_MAX_UART_TX_BUDGET_BYTES) * kMicrosPerSecond;
static const uint32_t kMinI2cFreqHz = static_cast<uint32_t>(RUNTIMEPROBE_I2C_MIN_FREQ_HZ);
static const uint32_t kMaxI2cFreqHz = static_cast<uint32_t>(RUNTIMEPROBE_I2C_MAX_FREQ_HZ);
static const uint32_t kMinI2cTimeoutMs = static_cast<uint32_t>(RUNTIMEPROBE_I2C_MIN_TIMEOUT_MS);
static const uint32_t kMaxI2cTimeoutMs = static_cast<uint32_t>(RUNTIMEPROBE_I2C_MAX_TIMEOUT_MS);
static const uint32_t kJtagTdoReportIntervalMs =
    static_cast<uint32_t>(RUNTIMEPROBE_JTAG_TDO_REPORT_INTERVAL_MS);

static bool pinsAreUnique(const int *pins, size_t count, const char *context, char *err, size_t errLen) {
  for (size_t i = 0; i < count; ++i) {
    if (pins[i] < 0) {
      continue;
    }
    for (size_t j = i + 1; j < count; ++j) {
      if (pins[i] == pins[j]) {
        snprintf(err, errLen, "duplicate pin %d in %s config", pins[i], context);
        return false;
      }
    }
  }
  return true;
}

static void printHexLine(const char *prefix, const uint8_t *data, size_t len) {
  Serial.print(prefix);
  for (size_t i = 0; i < len; ++i) {
    Serial.printf(" %02X", data[i]);
  }
  Serial.println();
}

RuntimeProbe::PinRegistry::PinRegistry() { clear(); }

void RuntimeProbe::PinRegistry::clear() {
  for (int i = 0; i < kMaxPins; ++i) {
    entries_[i].claimed = false;
    entries_[i].purpose[0] = '\0';
  }
}

bool RuntimeProbe::PinRegistry::isTracked(int pin) const { return pin >= 0 && pin < kMaxPins; }

bool RuntimeProbe::PinRegistry::isClaimed(int pin) const {
  if (!isTracked(pin)) {
    return false;
  }
  return entries_[pin].claimed;
}

const char *RuntimeProbe::PinRegistry::purpose(int pin) const {
  if (!isTracked(pin) || !entries_[pin].claimed) {
    return "";
  }
  return entries_[pin].purpose;
}

bool RuntimeProbe::PinRegistry::claim(int pin, const char *purpose, char *err, size_t errLen) {
  if (!isTracked(pin)) {
    snprintf(err, errLen, "pin %d out of tracked range (0-%d)", pin, kMaxPins - 1);
    return false;
  }
  if (entries_[pin].claimed) {
    snprintf(err, errLen, "pin %d already claimed (%s)", pin, entries_[pin].purpose);
    return false;
  }
  entries_[pin].claimed = true;
  strncpy(entries_[pin].purpose, purpose, sizeof(entries_[pin].purpose) - 1);
  entries_[pin].purpose[sizeof(entries_[pin].purpose) - 1] = '\0';
  return true;
}

void RuntimeProbe::PinRegistry::release(int pin, bool setInput) {
  if (!isTracked(pin)) {
    return;
  }
  if (setInput) {
    pinMode(pin, INPUT);
  }
  entries_[pin].claimed = false;
  entries_[pin].purpose[0] = '\0';
}

void RuntimeProbe::PinRegistry::releaseAll(bool setInput) {
  for (int i = 0; i < kMaxPins; ++i) {
    if (!entries_[i].claimed) {
      continue;
    }
    if (setInput) {
      pinMode(i, INPUT);
    }
    entries_[i].claimed = false;
    entries_[i].purpose[0] = '\0';
  }
}

void RuntimeProbe::PinRegistry::printClaims() const {
  bool any = false;
  for (int i = 0; i < kMaxPins; ++i) {
    if (!entries_[i].claimed) {
      continue;
    }
    Serial.printf("  pin %d -> %s\n", i, entries_[i].purpose);
    any = true;
  }
  if (!any) {
    Serial.println("  (none)");
  }
}

const char *RuntimeProbe::modeToString(Mode mode) const {
  switch (mode) {
    case Mode::Off:
      return "off";
    case Mode::Gpio:
      return "gpio";
    case Mode::Uart:
      return "uart";
    case Mode::Jtag:
      return "jtag";
    case Mode::Spi:
      return "spi";
    case Mode::I2c:
      return "i2c";
    case Mode::Pwm:
      return "pwm";
  }
  return "unknown";
}

bool RuntimeProbe::validatePin(int pin, char *err, size_t errLen) const {
  if (pin < 0) {
    snprintf(err, errLen, "pin must be >= 0");
    return false;
  }
#if defined(SOC_GPIO_PIN_COUNT)
  if (pin >= static_cast<int>(SOC_GPIO_PIN_COUNT)) {
    snprintf(err, errLen, "pin %d outside SoC range (0-%d)", pin, static_cast<int>(SOC_GPIO_PIN_COUNT) - 1);
    return false;
  }
#endif
  if (pin >= PinRegistry::kMaxPins) {
    snprintf(err, errLen, "pin %d too large for registry", pin);
    return false;
  }
  return true;
}

bool RuntimeProbe::parseInt(const char *s, int *out) const {
  if (!s || !*s) {
    return false;
  }
  errno = 0;
  char *end = nullptr;
  long val = strtol(s, &end, 0);
  if (!end || end == s || *end != '\0' || errno == ERANGE) {
    return false;
  }
  if (val < INT_MIN || val > INT_MAX) {
    return false;
  }
  *out = static_cast<int>(val);
  return true;
}

bool RuntimeProbe::parseUint32(const char *s, uint32_t *out) const {
  if (!s || !*s) {
    return false;
  }
  if (s[0] == '-') {
    return false;
  }
  errno = 0;
  char *end = nullptr;
  unsigned long long val = strtoull(s, &end, 0);
  if (!end || end == s || *end != '\0' || errno == ERANGE || val > 0xFFFFFFFFULL) {
    return false;
  }
  *out = static_cast<uint32_t>(val);
  return true;
}

bool RuntimeProbe::parseByte(const char *s, uint8_t *out, char *err, size_t errLen) const {
  uint32_t val = 0;
  if (!s || !*s) {
    snprintf(err, errLen, "missing byte");
    return false;
  }
  if (!parseUint32(s, &val)) {
    snprintf(err, errLen, "invalid byte");
    return false;
  }
  if (val > 0xFF) {
    snprintf(err, errLen, "byte out of range");
    return false;
  }
  *out = static_cast<uint8_t>(val);
  return true;
}

bool RuntimeProbe::parseI2cAddress(const char *s, uint8_t *out, char *err, size_t errLen) const {
  uint32_t val = 0;
  if (!s || !*s) {
    snprintf(err, errLen, "missing i2c address");
    return false;
  }
  if (!parseUint32(s, &val)) {
    snprintf(err, errLen, "invalid i2c address");
    return false;
  }
  if (val > 0x7F) {
    snprintf(err, errLen, "i2c address out of range (0x00-0x7F)");
    return false;
  }
  *out = static_cast<uint8_t>(val);
  return true;
}

bool RuntimeProbe::defaultUartPins(uint8_t num, int *txPin, int *rxPin, char *err, size_t errLen) const {
  if (num == 0) {
#if defined(TX) && defined(RX)
    *txPin = TX;
    *rxPin = RX;
    return true;
#else
    snprintf(err, errLen, "default UART0 pins not defined");
    return false;
#endif
  }
  if (num == 1) {
#if defined(TX1) && defined(RX1)
    *txPin = TX1;
    *rxPin = RX1;
    return true;
#else
    snprintf(err, errLen, "default UART1 pins not defined");
    return false;
#endif
  }
  if (num == 2) {
#if defined(TX2) && defined(RX2)
    *txPin = TX2;
    *rxPin = RX2;
    return true;
#else
    snprintf(err, errLen, "default UART2 pins not defined");
    return false;
#endif
  }
  snprintf(err, errLen, "uart num must be 0, 1, or 2");
  return false;
}

bool RuntimeProbe::defaultI2cPins(int *sdaPin, int *sclPin, char *err, size_t errLen) const {
#if defined(SDA) && defined(SCL)
  *sdaPin = SDA;
  *sclPin = SCL;
  return true;
#else
  snprintf(err, errLen, "default I2C pins not defined");
  return false;
#endif
}

bool RuntimeProbe::defaultSpiPins(int *sckPin, int *misoPin, int *mosiPin, int *csPin,
                                  char *err, size_t errLen) const {
#if defined(SCK) && defined(MISO) && defined(MOSI)
  *sckPin = SCK;
  *misoPin = MISO;
  *mosiPin = MOSI;
#if defined(SS)
  *csPin = SS;
#else
  *csPin = -1;
#endif
  return true;
#else
  snprintf(err, errLen, "default SPI pins not defined");
  return false;
#endif
}

void RuntimeProbe::gpioReleasePin() {
  if (!gpio_.pinClaimed || gpio_.pin < 0) {
    gpio_.wave = GpioState::None;
    return;
  }
  if (gpio_.safe == GpioState::SafeInput) {
    pins_.release(gpio_.pin, true);
  } else {
    pinMode(gpio_.pin, OUTPUT);
    digitalWrite(gpio_.pin, gpio_.safe == GpioState::SafeHigh ? HIGH : LOW);
    pins_.release(gpio_.pin, false);
  }
  gpio_.pinClaimed = false;
  gpio_.wave = GpioState::None;
}

bool RuntimeProbe::gpioEnsureClaimed(char *err, size_t errLen) {
  if (gpio_.pin < 0) {
    snprintf(err, errLen, "gpio pin not set");
    return false;
  }
  if (!gpio_.pinClaimed) {
    if (!pins_.claim(gpio_.pin, "GPIO", err, errLen)) {
      return false;
    }
    gpio_.pinClaimed = true;
  }
  return true;
}

void RuntimeProbe::gpioUpdate() {
  if (gpio_.wave == GpioState::None || !gpio_.pinClaimed) {
    return;
  }
  const uint32_t now = micros();
  if (gpio_.wave == GpioState::Toggle) {
    if (static_cast<int32_t>(now - gpio_.nextEdgeUs) >= 0) {
      gpio_.level = !gpio_.level;
      digitalWrite(gpio_.pin, gpio_.level ? HIGH : LOW);
      gpio_.nextEdgeUs = now + gpio_.halfPeriodUs;
    }
    return;
  }
  if (gpio_.wave == GpioState::Pulse) {
    if (static_cast<int32_t>(now - gpio_.nextEdgeUs) >= 0) {
      gpio_.level = !gpio_.level;
      digitalWrite(gpio_.pin, gpio_.level ? HIGH : LOW);
      if (gpio_.level) {
        gpio_.nextEdgeUs = now + gpio_.pulseWidthUs;
      } else {
        gpio_.nextEdgeUs = now + (gpio_.pulsePeriodUs - gpio_.pulseWidthUs);
      }
    }
  }
}

HardwareSerial *RuntimeProbe::uartByNum(uint8_t num) {
  if (num == 0) {
#if defined(ARDUINO_USB_CDC_ON_BOOT) && ARDUINO_USB_CDC_ON_BOOT
    return &Serial0;
#else
    return &Serial;
#endif
  }
#if SOC_UART_NUM > 1
  if (num == 1) {
    return &Serial1;
  }
#endif
#if SOC_UART_NUM > 2
  if (num == 2) {
    return &Serial2;
  }
#endif
  return nullptr;
}

const char *RuntimeProbe::uartPatternName(UartState::Pattern pat) const {
  switch (pat) {
    case UartState::Pat55:
      return "55";
    case UartState::PatAA:
      return "aa";
    case UartState::PatRamp:
      return "ramp";
    case UartState::PatAscii:
      return "ascii";
  }
  return "unknown";
}

uint8_t RuntimeProbe::uartNextByte() {
  switch (uart_.pattern) {
    case UartState::Pat55:
      return 0x55;
    case UartState::PatAA:
      return 0xAA;
    case UartState::PatRamp: {
      uint8_t val = uart_.rampValue;
      uart_.rampValue++;
      return val;
    }
    case UartState::PatAscii: {
      static const char kAscii[] = "ESP32 UART TEST\r\n";
      uint8_t val = static_cast<uint8_t>(kAscii[uart_.asciiIndex]);
      uart_.asciiIndex = (uart_.asciiIndex + 1) % (sizeof(kAscii) - 1);
      return val;
    }
  }
  return 0x00;
}

void RuntimeProbe::uartSendBytes(size_t count) {
  if (!uart_.port || count == 0) {
    return;
  }
  for (size_t i = 0; i < count; ++i) {
    uart_.port->write(uartNextByte());
  }
}

void RuntimeProbe::uartStop() {
  if (!uart_.active) {
    return;
  }
  if (uart_.port) {
    uart_.port->end();
  }
  if (uart_.txPin >= 0) {
    pins_.release(uart_.txPin, true);
  }
  if (uart_.rxPin >= 0) {
    pins_.release(uart_.rxPin, true);
  }
  uart_.active = false;
  uart_.port = nullptr;
  uart_.txPin = -1;
  uart_.rxPin = -1;
}

void RuntimeProbe::uartUpdate() {
  if (!uart_.active || !uart_.port) {
    return;
  }

  const uint32_t now = micros();
  if (uart_.txRateBps > 0) {
    const uint32_t elapsed = now - uart_.lastMicros;
    uart_.lastMicros = now;
    const uint64_t added = static_cast<uint64_t>(elapsed) * uart_.txRateBps;
    if (added >= kMaxUartTxBudget || uart_.txBudget > (kMaxUartTxBudget - added)) {
      uart_.txBudget = kMaxUartTxBudget;
    } else {
      uart_.txBudget += added;
    }
    size_t allowed = static_cast<size_t>(uart_.txBudget / kMicrosPerSecond);
    if (allowed > kMaxUartTxBytesPerUpdate) {
      allowed = kMaxUartTxBytesPerUpdate;
    }
    size_t space = uart_.port->availableForWrite();
    size_t toSend = (allowed < space) ? allowed : space;
    if (toSend > 0) {
      uartSendBytes(toSend);
      uart_.txBudget -= static_cast<uint64_t>(toSend) * kMicrosPerSecond;
    }
  } else {
    size_t space = uart_.port->availableForWrite();
    if (space > kMaxUartTxBytesPerUpdate) {
      space = kMaxUartTxBytesPerUpdate;
    }
    if (space > 0) {
      uartSendBytes(space);
    }
  }

  if (uart_.echo) {
    size_t echoed = 0;
    while (echoed < kMaxUartRxBytesPerUpdate && uart_.port->available()) {
      if (Serial.availableForWrite() == 0) {
        break;
      }
      int value = uart_.port->read();
      if (value < 0) {
        break;
      }
      Serial.write(static_cast<uint8_t>(value));
      echoed++;
    }
  } else {
    size_t drained = 0;
    while (drained < kMaxUartRxBytesPerUpdate && uart_.port->available()) {
      int value = uart_.port->read();
      if (value < 0) {
        break;
      }
      uart_.rxCount++;
      drained++;
    }
    uint32_t nowMs = millis();
    if (uart_.rxCount > 0 && (nowMs - uart_.lastRxReportMs) > 1000) {
      Serial.printf("UART RX: %u bytes\n", uart_.rxCount);
      uart_.rxCount = 0;
      uart_.lastRxReportMs = nowMs;
    }
  }
}

void RuntimeProbe::jtagSetDefaultSeq() {
  jtag_.seqLen = 8;
  for (size_t i = 0; i < jtag_.seqLen; ++i) {
    jtag_.tmsSeq[i] = (i % 2) ? 1 : 0;
    jtag_.tdiSeq[i] = (i / 2) % 2 ? 1 : 0;
  }
  jtag_.seqIndex = 0;
}

void RuntimeProbe::jtagApplySeq() {
  if (jtag_.seqLen == 0) {
    return;
  }
  digitalWrite(jtag_.tmsPin, jtag_.tmsSeq[jtag_.seqIndex] ? HIGH : LOW);
  digitalWrite(jtag_.tdiPin, jtag_.tdiSeq[jtag_.seqIndex] ? HIGH : LOW);
}

void RuntimeProbe::jtagStop() {
  if (!jtag_.active) {
    return;
  }
  pins_.release(jtag_.tckPin, true);
  pins_.release(jtag_.tmsPin, true);
  pins_.release(jtag_.tdiPin, true);
  if (jtag_.tdoPin >= 0) {
    pins_.release(jtag_.tdoPin, true);
  }
  jtag_.active = false;
  jtag_.tckPin = -1;
  jtag_.tmsPin = -1;
  jtag_.tdiPin = -1;
  jtag_.tdoPin = -1;
  jtag_.lastTdoReportMs = 0;
}

void RuntimeProbe::jtagUpdate() {
  if (!jtag_.active || jtag_.halfPeriodUs == 0 || jtag_.seqLen == 0) {
    return;
  }
  const uint32_t now = micros();
  if (static_cast<int32_t>(now - jtag_.nextEdgeUs) < 0) {
    return;
  }
  jtag_.nextEdgeUs = now + jtag_.halfPeriodUs;
  if (jtag_.tckHigh) {
    jtag_.tckHigh = false;
    digitalWrite(jtag_.tckPin, LOW);
    jtag_.seqIndex = (jtag_.seqIndex + 1) % jtag_.seqLen;
    jtagApplySeq();
  } else {
    jtag_.tckHigh = true;
    digitalWrite(jtag_.tckPin, HIGH);
    if (jtag_.tdoPin >= 0) {
      uint8_t bit = digitalRead(jtag_.tdoPin) ? 1 : 0;
      jtag_.tdoShift = (jtag_.tdoShift << 1) | bit;
      jtag_.tdoCount++;
      if (jtag_.tdoCount >= 64) {
        const uint32_t nowMs = millis();
        if (static_cast<int32_t>(nowMs - jtag_.lastTdoReportMs) >=
            static_cast<int32_t>(kJtagTdoReportIntervalMs)) {
          Serial.printf("TDO 0x%016llX\n", static_cast<unsigned long long>(jtag_.tdoShift));
          jtag_.lastTdoReportMs = nowMs;
        }
        jtag_.tdoShift = 0;
        jtag_.tdoCount = 0;
      }
    }
  }
}

void RuntimeProbe::i2cStop() {
  if (!i2c_.active) {
    return;
  }
  Wire.end();
  if (i2c_.sdaPin >= 0) {
    pins_.release(i2c_.sdaPin, true);
  }
  if (i2c_.sclPin >= 0) {
    pins_.release(i2c_.sclPin, true);
  }
  i2c_.active = false;
  i2c_.scanActive = false;
  i2c_.sdaPin = -1;
  i2c_.sclPin = -1;
}

void RuntimeProbe::i2cUpdate() {
  if (!i2c_.active || !i2c_.scanActive) {
    return;
  }
  const uint32_t nowMs = millis();
  if (static_cast<int32_t>(nowMs - i2c_.scanNextMs) < 0) {
    return;
  }
  if (i2c_.scanAddr > i2c_.scanEnd) {
    Serial.printf("I2C scan done, found %u device(s)\n", i2c_.scanFound);
    i2c_.scanActive = false;
    return;
  }
  uint8_t addr = i2c_.scanAddr++;
  Wire.beginTransmission(addr);
  uint8_t err = Wire.endTransmission();
  if (err == 0) {
    Serial.printf("I2C found 0x%02X\n", addr);
    i2c_.scanFound++;
  }
  i2c_.scanNextMs = nowMs + 2;
}

void RuntimeProbe::spiStop() {
  if (!spi_.active) {
    return;
  }
  SPI.end();
  if (spi_.sckPin >= 0) {
    pins_.release(spi_.sckPin, true);
  }
  if (spi_.misoPin >= 0) {
    pins_.release(spi_.misoPin, true);
  }
  if (spi_.mosiPin >= 0) {
    pins_.release(spi_.mosiPin, true);
  }
  if (spi_.csPin >= 0) {
    pins_.release(spi_.csPin, true);
  }
  spi_.active = false;
  spi_.sckPin = -1;
  spi_.misoPin = -1;
  spi_.mosiPin = -1;
  spi_.csPin = -1;
}

void RuntimeProbe::stopAllModes() {
  gpioReleasePin();
  uartStop();
  jtagStop();
  i2cStop();
  spiStop();
}

int RuntimeProbe::tokenize(char *line, char *argv[], int maxTokens) const {
  int argc = 0;
  char *p = line;
  while (*p) {
    while (*p && isspace(static_cast<unsigned char>(*p))) {
      ++p;
    }
    if (!*p) {
      break;
    }
    if (argc >= maxTokens) {
      return -1;
    }
    if (*p == '"') {
      ++p;
      argv[argc++] = p;
      while (*p && *p != '"') {
        ++p;
      }
      if (*p == '"') {
        *p = '\0';
        ++p;
      }
    } else {
      argv[argc++] = p;
      while (*p && !isspace(static_cast<unsigned char>(*p))) {
        ++p;
      }
      if (*p) {
        *p = '\0';
        ++p;
      }
    }
  }
  return argc;
}

void RuntimeProbe::printHelp() const {
  Serial.println("Commands:");
  Serial.println("  help");
  Serial.println("  status");
  Serial.println("  pins");
  Serial.println("  defaults");
  Serial.println("  mode off|gpio|uart|jtag|spi|i2c|pwm");
  Serial.println("  gpio set <pin>");
  Serial.println("  gpio toggle <freq_hz>");
  Serial.println("  gpio pulse <width_us> <period_us>");
  Serial.println("  gpio high|low");
  Serial.println("  gpio safe <0|1|input>");
  Serial.println("  gpio stop");
  Serial.println("  uart force <0|1>");
  Serial.println("  uart start <num> <tx_pin> <rx_pin> <baud>");
  Serial.println("  uart start <num> <baud>  (default pins)");
  Serial.println("  uart start <num> default <baud>");
  Serial.println("  uart pattern 55|aa|ramp|ascii");
  Serial.println("  uart txrate <bytes_per_sec>");
  Serial.println("  uart echo <0|1>");
  Serial.println("  uart stop");
  Serial.println("  jtag bitbang <tck> <tms> <tdi> [tdo] <freq_hz>");
  Serial.println("  jtag seq <bits_or_hex>  (pairs: TMS then TDI)");
  Serial.println("  jtag stop");
  Serial.println("  i2c start <sda> <scl> [freq_hz] [timeout_ms]");
  Serial.println("  i2c start default [freq_hz] [timeout_ms]");
  Serial.printf("    i2c limits: freq %u..%u Hz, timeout %u..%u ms\n",
                static_cast<unsigned>(kMinI2cFreqHz),
                static_cast<unsigned>(kMaxI2cFreqHz),
                static_cast<unsigned>(kMinI2cTimeoutMs),
                static_cast<unsigned>(kMaxI2cTimeoutMs));
  Serial.println("  i2c freq <hz>");
  Serial.println("  i2c timeout <ms>");
  Serial.println("  i2c scan [start] [end] | stop");
  Serial.println("  i2c write <addr> <byte...>");
  Serial.println("  i2c read <addr> <len>");
  Serial.println("  i2c writeread <addr> <len> <byte...>");
  Serial.println("  i2c stop");
  Serial.println("  spi start <sck> <miso> <mosi> [cs]");
  Serial.println("  spi start default [cs]");
  Serial.println("  spi freq <hz>");
  Serial.println("  spi mode <0|1|2|3>");
  Serial.println("  spi order msb|lsb");
  Serial.println("  spi cs <pin|-1>");
  Serial.println("  spi dummy <byte>");
  Serial.println("  spi xfer <byte...>");
  Serial.println("  spi read <len>");
  Serial.println("  spi stop");
}

void RuntimeProbe::printStatus() const {
  Serial.printf("Mode: %s\n", modeToString(mode_));
  Serial.printf("GPIO: pin=%d claimed=%s wave=%s safe=%s\n",
                gpio_.pin,
                gpio_.pinClaimed ? "yes" : "no",
                gpio_.wave == GpioState::None
                    ? "none"
                    : (gpio_.wave == GpioState::Toggle
                           ? "toggle"
                           : (gpio_.wave == GpioState::Pulse ? "pulse" : "static")),
                gpio_.safe == GpioState::SafeInput
                    ? "input"
                    : (gpio_.safe == GpioState::SafeHigh ? "high" : "low"));
  if (uart_.active) {
    Serial.printf("UART: num=%u tx=%d rx=%d baud=%u pattern=%s txrate=%u echo=%s\n",
                  uart_.num,
                  uart_.txPin,
                  uart_.rxPin,
                  uart_.baud,
                  uartPatternName(uart_.pattern),
                  uart_.txRateBps,
                  uart_.echo ? "on" : "off");
  } else {
    Serial.println("UART: inactive");
  }
  if (jtag_.active) {
    Serial.printf("JTAG: tck=%d tms=%d tdi=%d tdo=%d freq=%u seqLen=%u\n",
                  jtag_.tckPin,
                  jtag_.tmsPin,
                  jtag_.tdiPin,
                  jtag_.tdoPin,
                  jtag_.freqHz,
                  static_cast<unsigned>(jtag_.seqLen));
  } else {
    Serial.println("JTAG: inactive");
  }
  if (i2c_.active) {
    Serial.printf("I2C: sda=%d scl=%d freq=%u timeout=%u scan=%s\n",
                  i2c_.sdaPin,
                  i2c_.sclPin,
                  i2c_.freqHz,
                  i2c_.timeoutMs,
                  i2c_.scanActive ? "on" : "off");
  } else {
    Serial.println("I2C: inactive");
  }
  if (spi_.active) {
    Serial.printf("SPI: sck=%d miso=%d mosi=%d cs=%d freq=%u mode=%u order=%s dummy=0x%02X\n",
                  spi_.sckPin,
                  spi_.misoPin,
                  spi_.mosiPin,
                  spi_.csPin,
                  spi_.freqHz,
                  spi_.mode,
                  spi_.bitOrder == MSBFIRST ? "msb" : "lsb",
                  spi_.dummyByte);
  } else {
    Serial.println("SPI: inactive");
  }
}

void RuntimeProbe::printPins() const {
  Serial.println("Pin rules:");
#if defined(SOC_GPIO_PIN_COUNT)
  Serial.printf("  Allowed GPIO range for this SoC: 0..%d\n", static_cast<int>(SOC_GPIO_PIN_COUNT) - 1);
#else
  Serial.println("  Negative pins are rejected.");
#endif
  Serial.println("Claimed pins:");
  pins_.printClaims();
}

void RuntimeProbe::printDefaults() const {
  Serial.println("Default pins:");
  int tx = -1;
  int rx = -1;
  int sda = -1;
  int scl = -1;
  int sck = -1;
  int miso = -1;
  int mosi = -1;
  int cs = -1;
  char err[64] = {0};

  if (defaultUartPins(0, &tx, &rx, err, sizeof(err))) {
    Serial.printf("  UART0: tx=%d rx=%d\n", tx, rx);
  } else {
    Serial.printf("  UART0: %s\n", err);
  }
  if (defaultUartPins(1, &tx, &rx, err, sizeof(err))) {
    Serial.printf("  UART1: tx=%d rx=%d\n", tx, rx);
  } else {
    Serial.printf("  UART1: %s\n", err);
  }
  if (defaultI2cPins(&sda, &scl, err, sizeof(err))) {
    Serial.printf("  I2C: sda=%d scl=%d\n", sda, scl);
  } else {
    Serial.printf("  I2C: %s\n", err);
  }
  if (defaultSpiPins(&sck, &miso, &mosi, &cs, err, sizeof(err))) {
    Serial.printf("  SPI: sck=%d miso=%d mosi=%d cs=%d\n", sck, miso, mosi, cs);
  } else {
    Serial.printf("  SPI: %s\n", err);
  }
}

bool RuntimeProbe::parseSeqBits(const char *input, char *err, size_t errLen) {
  if (!input || !*input) {
    snprintf(err, errLen, "missing sequence");
    return false;
  }

  const char *p = input;
  if (p[0] == '0' && (p[1] == 'x' || p[1] == 'X')) {
    p += 2;
  }

  bool bitsOnly = true;
  bool hexOnly = true;
  for (const char *c = p; *c; ++c) {
    if (*c == '_' || isspace(static_cast<unsigned char>(*c))) {
      continue;
    }
    if (*c != '0' && *c != '1') {
      bitsOnly = false;
    }
    if (!isxdigit(static_cast<unsigned char>(*c))) {
      hexOnly = false;
    }
  }

  if (!bitsOnly && !hexOnly) {
    snprintf(err, errLen, "sequence must be bits or hex");
    return false;
  }

  uint8_t bits[JtagState::kMaxSeq * 2];
  size_t bitCount = 0;
  if (bitsOnly) {
    for (const char *c = p; *c; ++c) {
      if (*c == '_' || isspace(static_cast<unsigned char>(*c))) {
        continue;
      }
      if (bitCount >= sizeof(bits)) {
        snprintf(err, errLen, "sequence too long");
        return false;
      }
      bits[bitCount++] = (*c == '1') ? 1 : 0;
    }
  } else {
    for (const char *c = p; *c; ++c) {
      if (*c == '_' || isspace(static_cast<unsigned char>(*c))) {
        continue;
      }
      int value = 0;
      if (*c >= '0' && *c <= '9') {
        value = *c - '0';
      } else if (*c >= 'a' && *c <= 'f') {
        value = *c - 'a' + 10;
      } else if (*c >= 'A' && *c <= 'F') {
        value = *c - 'A' + 10;
      } else {
        snprintf(err, errLen, "invalid hex digit");
        return false;
      }
      for (int bit = 3; bit >= 0; --bit) {
        if (bitCount >= sizeof(bits)) {
          snprintf(err, errLen, "sequence too long");
          return false;
        }
        bits[bitCount++] = (value >> bit) & 0x01;
      }
    }
  }

  if (bitCount < 2 || (bitCount % 2) != 0) {
    snprintf(err, errLen, "sequence needs even number of bits (TMS/TDI pairs)");
    return false;
  }

  size_t steps = bitCount / 2;
  if (steps > JtagState::kMaxSeq) {
    snprintf(err, errLen, "sequence too long (max %u steps)", static_cast<unsigned>(JtagState::kMaxSeq));
    return false;
  }

  jtag_.seqLen = steps;
  for (size_t i = 0; i < steps; ++i) {
    jtag_.tmsSeq[i] = bits[i * 2];
    jtag_.tdiSeq[i] = bits[i * 2 + 1];
  }
  jtag_.seqIndex = 0;
  if (jtag_.active) {
    jtagApplySeq();
  }
  return true;
}

bool RuntimeProbe::handleCommand(int argc, char *argv[], char *err, size_t errLen) {
  if (argc == 0) {
    return true;
  }

  if (strcmp(argv[0], "help") == 0) {
    printHelp();
    return true;
  }
  if (strcmp(argv[0], "status") == 0) {
    printStatus();
    return true;
  }
  if (strcmp(argv[0], "pins") == 0) {
    printPins();
    return true;
  }
  if (strcmp(argv[0], "defaults") == 0) {
    printDefaults();
    return true;
  }

  if (strcmp(argv[0], "mode") == 0) {
    if (argc < 2) {
      snprintf(err, errLen, "mode requires an argument");
      return false;
    }
    if (strcmp(argv[1], "off") == 0) {
      stopAllModes();
      mode_ = Mode::Off;
      return true;
    }
    if (strcmp(argv[1], "gpio") == 0) {
      stopAllModes();
      mode_ = Mode::Gpio;
      return true;
    }
    if (strcmp(argv[1], "uart") == 0) {
      stopAllModes();
      mode_ = Mode::Uart;
      return true;
    }
    if (strcmp(argv[1], "jtag") == 0) {
      stopAllModes();
      mode_ = Mode::Jtag;
      return true;
    }
    if (strcmp(argv[1], "spi") == 0) {
      stopAllModes();
      mode_ = Mode::Spi;
      return true;
    }
    if (strcmp(argv[1], "i2c") == 0) {
      stopAllModes();
      mode_ = Mode::I2c;
      return true;
    }
    if (strcmp(argv[1], "pwm") == 0) {
      stopAllModes();
      mode_ = Mode::Pwm;
      return true;
    }
    snprintf(err, errLen, "unknown mode");
    return false;
  }

  if (strcmp(argv[0], "gpio") == 0) {
    if (mode_ != Mode::Gpio) {
      snprintf(err, errLen, "not in gpio mode");
      return false;
    }
    if (argc < 2) {
      snprintf(err, errLen, "gpio requires subcommand");
      return false;
    }
    if (strcmp(argv[1], "set") == 0) {
      if (argc < 3) {
        snprintf(err, errLen, "gpio set <pin>");
        return false;
      }
      int pin = -1;
      if (!parseInt(argv[2], &pin)) {
        snprintf(err, errLen, "invalid pin");
        return false;
      }
      if (!validatePin(pin, err, errLen)) {
        return false;
      }
      if (gpio_.pinClaimed) {
        gpioReleasePin();
      }
      if (!pins_.claim(pin, "GPIO", err, errLen)) {
        return false;
      }
      gpio_.pin = pin;
      gpio_.pinClaimed = true;
      gpio_.wave = GpioState::None;
      pinMode(pin, INPUT);
      return true;
    }
    if (strcmp(argv[1], "toggle") == 0) {
      if (argc < 3) {
        snprintf(err, errLen, "gpio toggle <freq_hz>");
        return false;
      }
      uint32_t freq = 0;
      if (!parseUint32(argv[2], &freq) || freq == 0) {
        snprintf(err, errLen, "invalid frequency");
        return false;
      }
      if (!gpioEnsureClaimed(err, errLen)) {
        return false;
      }
      uint32_t period = 1000000UL / freq;
      if (period < 2) {
        snprintf(err, errLen, "frequency too high for software toggle");
        return false;
      }
      gpio_.halfPeriodUs = period / 2;
      if (gpio_.halfPeriodUs == 0) {
        gpio_.halfPeriodUs = 1;
      }
      pinMode(gpio_.pin, OUTPUT);
      gpio_.level = false;
      digitalWrite(gpio_.pin, LOW);
      gpio_.nextEdgeUs = micros() + gpio_.halfPeriodUs;
      gpio_.wave = GpioState::Toggle;
      return true;
    }
    if (strcmp(argv[1], "pulse") == 0) {
      if (argc < 4) {
        snprintf(err, errLen, "gpio pulse <width_us> <period_us>");
        return false;
      }
      uint32_t width = 0;
      uint32_t period = 0;
      if (!parseUint32(argv[2], &width) || !parseUint32(argv[3], &period)) {
        snprintf(err, errLen, "invalid pulse parameters");
        return false;
      }
      if (width == 0 || period == 0 || width >= period) {
        snprintf(err, errLen, "width must be < period");
        return false;
      }
      if (!gpioEnsureClaimed(err, errLen)) {
        return false;
      }
      gpio_.pulseWidthUs = width;
      gpio_.pulsePeriodUs = period;
      pinMode(gpio_.pin, OUTPUT);
      gpio_.level = true;
      digitalWrite(gpio_.pin, HIGH);
      gpio_.nextEdgeUs = micros() + gpio_.pulseWidthUs;
      gpio_.wave = GpioState::Pulse;
      return true;
    }
    if (strcmp(argv[1], "high") == 0 || strcmp(argv[1], "low") == 0) {
      if (!gpioEnsureClaimed(err, errLen)) {
        return false;
      }
      pinMode(gpio_.pin, OUTPUT);
      gpio_.level = (strcmp(argv[1], "high") == 0);
      digitalWrite(gpio_.pin, gpio_.level ? HIGH : LOW);
      gpio_.wave = GpioState::Static;
      return true;
    }
    if (strcmp(argv[1], "safe") == 0) {
      if (argc < 3) {
        snprintf(err, errLen, "gpio safe <0|1|input>");
        return false;
      }
      if (strcmp(argv[2], "input") == 0) {
        gpio_.safe = GpioState::SafeInput;
        return true;
      }
      if (strcmp(argv[2], "0") == 0) {
        gpio_.safe = GpioState::SafeLow;
        return true;
      }
      if (strcmp(argv[2], "1") == 0) {
        gpio_.safe = GpioState::SafeHigh;
        return true;
      }
      snprintf(err, errLen, "gpio safe expects 0, 1, or input");
      return false;
    }
    if (strcmp(argv[1], "stop") == 0) {
      gpioReleasePin();
      return true;
    }
    snprintf(err, errLen, "unknown gpio subcommand");
    return false;
  }

  if (strcmp(argv[0], "uart") == 0) {
    if (mode_ != Mode::Uart) {
      snprintf(err, errLen, "not in uart mode");
      return false;
    }
    if (argc < 2) {
      snprintf(err, errLen, "uart requires subcommand");
      return false;
    }
    if (strcmp(argv[1], "force") == 0) {
      if (argc < 3) {
        snprintf(err, errLen, "uart force <0|1>");
        return false;
      }
      int val = 0;
      if (!parseInt(argv[2], &val)) {
        snprintf(err, errLen, "invalid force value");
        return false;
      }
      uart_.forceUart0 = (val != 0);
      return true;
    }
    if (strcmp(argv[1], "start") == 0) {
      if (argc < 4) {
        snprintf(err, errLen, "uart start <num> <tx_pin> <rx_pin> <baud> | default <baud>");
        return false;
      }
      int num = 0;
      int txPin = -1;
      int rxPin = -1;
      uint32_t baud = 0;
      bool useDefaults = false;

      if (!parseInt(argv[2], &num)) {
        snprintf(err, errLen, "invalid uart num");
        return false;
      }
      if (argc == 4) {
        if (!parseUint32(argv[3], &baud)) {
          snprintf(err, errLen, "invalid baud");
          return false;
        }
        useDefaults = true;
      } else if (argc == 5 && strcmp(argv[3], "default") == 0) {
        if (!parseUint32(argv[4], &baud)) {
          snprintf(err, errLen, "invalid baud");
          return false;
        }
        useDefaults = true;
      } else if (argc == 6) {
        if (!parseInt(argv[3], &txPin) || !parseInt(argv[4], &rxPin) ||
            !parseUint32(argv[5], &baud)) {
          snprintf(err, errLen, "invalid uart parameters");
          return false;
        }
      } else {
        snprintf(err, errLen, "uart start <num> <tx_pin> <rx_pin> <baud>");
        return false;
      }
      if (num < 0 || num > 2) {
        snprintf(err, errLen, "uart num must be 0, 1, or 2");
        return false;
      }
      if (num == 0 && !uart_.forceUart0) {
        snprintf(err, errLen, "uart0 requires 'uart force 1'");
        return false;
      }
      if (useDefaults) {
        if (!defaultUartPins(static_cast<uint8_t>(num), &txPin, &rxPin, err, errLen)) {
          return false;
        }
      }
      if (txPin == rxPin) {
        snprintf(err, errLen, "tx and rx pins must differ");
        return false;
      }
      if (baud == 0) {
        snprintf(err, errLen, "baud must be > 0");
        return false;
      }
      if (!validatePin(txPin, err, errLen) || !validatePin(rxPin, err, errLen)) {
        return false;
      }
      HardwareSerial *port = uartByNum(static_cast<uint8_t>(num));
      if (!port) {
        snprintf(err, errLen, "uart num not available");
        return false;
      }
      uartStop();
      if (!pins_.claim(txPin, "UART TX", err, errLen)) {
        return false;
      }
      if (!pins_.claim(rxPin, "UART RX", err, errLen)) {
        pins_.release(txPin, true);
        return false;
      }
      if (num == 0) {
        Serial.println("WARN: UART0 may conflict with Serial monitor.");
      }
      port->begin(baud, SERIAL_8N1, rxPin, txPin);
      uart_.active = true;
      uart_.num = static_cast<uint8_t>(num);
      uart_.txPin = txPin;
      uart_.rxPin = rxPin;
      uart_.baud = baud;
      uart_.port = port;
      uart_.rampValue = 0;
      uart_.asciiIndex = 0;
      uart_.txBudget = 0;
      uart_.lastMicros = micros();
      uart_.rxCount = 0;
      uart_.lastRxReportMs = millis();
      return true;
    }
    if (strcmp(argv[1], "pattern") == 0) {
      if (argc < 3) {
        snprintf(err, errLen, "uart pattern 55|aa|ramp|ascii");
        return false;
      }
      if (strcmp(argv[2], "55") == 0) {
        uart_.pattern = UartState::Pat55;
        return true;
      }
      if (strcmp(argv[2], "aa") == 0) {
        uart_.pattern = UartState::PatAA;
        return true;
      }
      if (strcmp(argv[2], "ramp") == 0) {
        uart_.pattern = UartState::PatRamp;
        uart_.rampValue = 0;
        return true;
      }
      if (strcmp(argv[2], "ascii") == 0) {
        uart_.pattern = UartState::PatAscii;
        uart_.asciiIndex = 0;
        return true;
      }
      snprintf(err, errLen, "unknown uart pattern");
      return false;
    }
    if (strcmp(argv[1], "txrate") == 0) {
      if (argc < 3) {
        snprintf(err, errLen, "uart txrate <bytes_per_sec>");
        return false;
      }
      uint32_t rate = 0;
      if (!parseUint32(argv[2], &rate)) {
        snprintf(err, errLen, "invalid txrate");
        return false;
      }
      uart_.txRateBps = rate;
      uart_.txBudget = 0;
      uart_.lastMicros = micros();
      return true;
    }
    if (strcmp(argv[1], "echo") == 0) {
      if (argc < 3) {
        snprintf(err, errLen, "uart echo <0|1>");
        return false;
      }
      int val = 0;
      if (!parseInt(argv[2], &val)) {
        snprintf(err, errLen, "invalid echo value");
        return false;
      }
      uart_.echo = (val != 0);
      return true;
    }
    if (strcmp(argv[1], "stop") == 0) {
      uartStop();
      return true;
    }
    snprintf(err, errLen, "unknown uart subcommand");
    return false;
  }

  if (strcmp(argv[0], "jtag") == 0) {
    if (mode_ != Mode::Jtag) {
      snprintf(err, errLen, "not in jtag mode");
      return false;
    }
    if (argc < 2) {
      snprintf(err, errLen, "jtag requires subcommand");
      return false;
    }
    if (strcmp(argv[1], "bitbang") == 0) {
      if (argc < 6) {
        snprintf(err, errLen, "jtag bitbang <tck> <tms> <tdi> [tdo] <freq_hz>");
        return false;
      }
      if (argc != 6 && argc != 7) {
        snprintf(err, errLen, "jtag bitbang <tck> <tms> <tdi> [tdo] <freq_hz>");
        return false;
      }
      int tck = -1, tms = -1, tdi = -1, tdo = -1;
      uint32_t freq = 0;
      if (argc == 6) {
        if (!parseInt(argv[2], &tck) || !parseInt(argv[3], &tms) ||
            !parseInt(argv[4], &tdi) || !parseUint32(argv[5], &freq)) {
          snprintf(err, errLen, "invalid jtag parameters");
          return false;
        }
      } else {
        if (!parseInt(argv[2], &tck) || !parseInt(argv[3], &tms) ||
            !parseInt(argv[4], &tdi) || !parseInt(argv[5], &tdo) ||
            !parseUint32(argv[6], &freq)) {
          snprintf(err, errLen, "invalid jtag parameters");
          return false;
        }
      }
      if (!validatePin(tck, err, errLen) || !validatePin(tms, err, errLen) ||
          !validatePin(tdi, err, errLen)) {
        return false;
      }
      if (tdo >= 0 && !validatePin(tdo, err, errLen)) {
        return false;
      }
      int jtagPins[4] = {tck, tms, tdi, tdo};
      if (!pinsAreUnique(jtagPins, 4, "jtag", err, errLen)) {
        return false;
      }
      if (freq == 0) {
        snprintf(err, errLen, "invalid frequency");
        return false;
      }
      uint32_t period = 1000000UL / freq;
      if (period < 2) {
        snprintf(err, errLen, "frequency too high for software bitbang");
        return false;
      }
      jtagStop();
      if (!pins_.claim(tck, "JTAG TCK", err, errLen)) {
        return false;
      }
      if (!pins_.claim(tms, "JTAG TMS", err, errLen)) {
        pins_.release(tck, true);
        return false;
      }
      if (!pins_.claim(tdi, "JTAG TDI", err, errLen)) {
        pins_.release(tck, true);
        pins_.release(tms, true);
        return false;
      }
      if (tdo >= 0) {
        if (!pins_.claim(tdo, "JTAG TDO", err, errLen)) {
          pins_.release(tck, true);
          pins_.release(tms, true);
          pins_.release(tdi, true);
          return false;
        }
      }
      jtag_.tckPin = tck;
      jtag_.tmsPin = tms;
      jtag_.tdiPin = tdi;
      jtag_.tdoPin = tdo;
      jtag_.freqHz = freq;
      jtag_.halfPeriodUs = period / 2;
      if (jtag_.halfPeriodUs == 0) {
        jtag_.halfPeriodUs = 1;
      }
      pinMode(tck, OUTPUT);
      pinMode(tms, OUTPUT);
      pinMode(tdi, OUTPUT);
      if (tdo >= 0) {
        pinMode(tdo, INPUT);
      }
      digitalWrite(tck, LOW);
      jtag_.tckHigh = false;
      jtag_.seqIndex = 0;
      if (jtag_.seqLen == 0) {
        jtagSetDefaultSeq();
      }
      jtagApplySeq();
      jtag_.nextEdgeUs = micros() + jtag_.halfPeriodUs;
      jtag_.tdoShift = 0;
      jtag_.tdoCount = 0;
      jtag_.lastTdoReportMs = millis();
      jtag_.active = true;
      return true;
    }
    if (strcmp(argv[1], "seq") == 0) {
      if (argc < 3) {
        snprintf(err, errLen, "jtag seq <bits_or_hex>");
        return false;
      }
      if (!parseSeqBits(argv[2], err, errLen)) {
        return false;
      }
      return true;
    }
    if (strcmp(argv[1], "stop") == 0) {
      jtagStop();
      return true;
    }
    snprintf(err, errLen, "unknown jtag subcommand");
    return false;
  }

  if (strcmp(argv[0], "i2c") == 0) {
    if (mode_ != Mode::I2c) {
      snprintf(err, errLen, "not in i2c mode");
      return false;
    }
    if (argc < 2) {
      snprintf(err, errLen, "i2c requires subcommand");
      return false;
    }
    if (strcmp(argv[1], "start") == 0) {
      int sda = -1;
      int scl = -1;
      uint32_t freq = i2c_.freqHz;
      uint32_t timeout = i2c_.timeoutMs;
      int argIndex = 2;
      if (argc == 2 || (argc >= 3 && strcmp(argv[2], "default") == 0)) {
        argIndex = (argc >= 3 && strcmp(argv[2], "default") == 0) ? 3 : 2;
        if (!defaultI2cPins(&sda, &scl, err, errLen)) {
          return false;
        }
      } else {
        if (argc < 4) {
          snprintf(err, errLen, "i2c start <sda> <scl> [freq_hz] [timeout_ms]");
          return false;
        }
        if (!parseInt(argv[2], &sda) || !parseInt(argv[3], &scl)) {
          snprintf(err, errLen, "invalid i2c pins");
          return false;
        }
        argIndex = 4;
      }

      if (argc > argIndex && !parseUint32(argv[argIndex], &freq)) {
        snprintf(err, errLen, "invalid i2c frequency");
        return false;
      }
      if (argc > argIndex + 1 && !parseUint32(argv[argIndex + 1], &timeout)) {
        snprintf(err, errLen, "invalid i2c timeout");
        return false;
      }
      if (argc > argIndex + 2) {
        snprintf(err, errLen, "i2c start <sda> <scl> [freq_hz] [timeout_ms]");
        return false;
      }
      if (freq == 0 || timeout == 0) {
        snprintf(err, errLen, "i2c freq/timeout must be > 0");
        return false;
      }
      if (freq < kMinI2cFreqHz || freq > kMaxI2cFreqHz) {
        snprintf(err, errLen, "i2c frequency must be %u..%u",
                 static_cast<unsigned>(kMinI2cFreqHz), static_cast<unsigned>(kMaxI2cFreqHz));
        return false;
      }
      if (timeout < kMinI2cTimeoutMs || timeout > kMaxI2cTimeoutMs) {
        snprintf(err, errLen, "i2c timeout must be %u..%u ms",
                 static_cast<unsigned>(kMinI2cTimeoutMs), static_cast<unsigned>(kMaxI2cTimeoutMs));
        return false;
      }
      if (!validatePin(sda, err, errLen) || !validatePin(scl, err, errLen)) {
        return false;
      }
      if (sda == scl) {
        snprintf(err, errLen, "i2c sda and scl pins must differ");
        return false;
      }
      i2cStop();
      if (!pins_.claim(sda, "I2C SDA", err, errLen)) {
        return false;
      }
      if (!pins_.claim(scl, "I2C SCL", err, errLen)) {
        pins_.release(sda, true);
        return false;
      }
      if (!Wire.begin(sda, scl, freq)) {
        pins_.release(sda, true);
        pins_.release(scl, true);
        snprintf(err, errLen, "i2c start failed");
        return false;
      }
      if (!Wire.setClock(freq)) {
        Wire.end();
        pins_.release(sda, true);
        pins_.release(scl, true);
        snprintf(err, errLen, "i2c clock configuration failed");
        return false;
      }
      Wire.setTimeOut(static_cast<uint16_t>(timeout));
      i2c_.active = true;
      i2c_.sdaPin = sda;
      i2c_.sclPin = scl;
      i2c_.freqHz = freq;
      i2c_.timeoutMs = timeout;
      i2c_.scanActive = false;
      return true;
    }
    if (strcmp(argv[1], "freq") == 0) {
      if (argc < 3) {
        snprintf(err, errLen, "i2c freq <hz>");
        return false;
      }
      if (!i2c_.active) {
        snprintf(err, errLen, "i2c not started");
        return false;
      }
      uint32_t freq = 0;
      if (!parseUint32(argv[2], &freq) || freq == 0) {
        snprintf(err, errLen, "invalid i2c frequency");
        return false;
      }
      if (freq < kMinI2cFreqHz || freq > kMaxI2cFreqHz) {
        snprintf(err, errLen, "i2c frequency must be %u..%u",
                 static_cast<unsigned>(kMinI2cFreqHz), static_cast<unsigned>(kMaxI2cFreqHz));
        return false;
      }
      if (!Wire.setClock(freq)) {
        snprintf(err, errLen, "i2c clock configuration failed");
        return false;
      }
      i2c_.freqHz = freq;
      return true;
    }
    if (strcmp(argv[1], "timeout") == 0) {
      if (argc < 3) {
        snprintf(err, errLen, "i2c timeout <ms>");
        return false;
      }
      if (!i2c_.active) {
        snprintf(err, errLen, "i2c not started");
        return false;
      }
      uint32_t timeout = 0;
      if (!parseUint32(argv[2], &timeout) || timeout == 0) {
        snprintf(err, errLen, "invalid i2c timeout");
        return false;
      }
      if (timeout < kMinI2cTimeoutMs || timeout > kMaxI2cTimeoutMs) {
        snprintf(err, errLen, "i2c timeout must be %u..%u ms",
                 static_cast<unsigned>(kMinI2cTimeoutMs), static_cast<unsigned>(kMaxI2cTimeoutMs));
        return false;
      }
      Wire.setTimeOut(static_cast<uint16_t>(timeout));
      i2c_.timeoutMs = timeout;
      return true;
    }
    if (strcmp(argv[1], "scan") == 0) {
      if (!i2c_.active) {
        snprintf(err, errLen, "i2c not started");
        return false;
      }
      if (argc >= 3 && strcmp(argv[2], "stop") == 0) {
        i2c_.scanActive = false;
        return true;
      }
      uint8_t start = 0x03;
      uint8_t end = 0x77;
      if (argc >= 3) {
        if (!parseI2cAddress(argv[2], &start, err, errLen)) {
          return false;
        }
        end = start;
      }
      if (argc >= 4) {
        if (!parseI2cAddress(argv[3], &end, err, errLen)) {
          return false;
        }
      }
      if (start > end) {
        snprintf(err, errLen, "scan start must be <= end");
        return false;
      }
      i2c_.scanStart = start;
      i2c_.scanEnd = end;
      i2c_.scanAddr = start;
      i2c_.scanFound = 0;
      i2c_.scanActive = true;
      i2c_.scanNextMs = millis();
      Serial.printf("I2C scan 0x%02X-0x%02X\n", start, end);
      return true;
    }
    if (strcmp(argv[1], "write") == 0) {
      if (argc < 4) {
        snprintf(err, errLen, "i2c write <addr> <byte...>");
        return false;
      }
      if (!i2c_.active) {
        snprintf(err, errLen, "i2c not started");
        return false;
      }
      uint8_t addr = 0;
      if (!parseI2cAddress(argv[2], &addr, err, errLen)) {
        return false;
      }
      uint8_t data[kMaxIoBytes];
      size_t count = 0;
      for (int i = 3; i < argc; ++i) {
        if (count >= kMaxIoBytes) {
          snprintf(err, errLen, "too many bytes (max %u)", static_cast<unsigned>(kMaxIoBytes));
          return false;
        }
        if (!parseByte(argv[i], &data[count], err, errLen)) {
          return false;
        }
        count++;
      }
      if (count == 0) {
        snprintf(err, errLen, "no data bytes");
        return false;
      }
      Wire.beginTransmission(addr);
      Wire.write(data, count);
      uint8_t rc = Wire.endTransmission();
      if (rc != 0) {
        const char *msg = (rc == 1) ? "data too long"
                          : (rc == 2) ? "address nack"
                          : (rc == 3) ? "data nack"
                          : (rc == 4) ? "bus error"
                                      : "unknown error";
        snprintf(err, errLen, "i2c write failed (%s)", msg);
        return false;
      }
      return true;
    }
    if (strcmp(argv[1], "read") == 0) {
      if (argc < 4) {
        snprintf(err, errLen, "i2c read <addr> <len>");
        return false;
      }
      if (!i2c_.active) {
        snprintf(err, errLen, "i2c not started");
        return false;
      }
      uint8_t addr = 0;
      if (!parseI2cAddress(argv[2], &addr, err, errLen)) {
        return false;
      }
      uint32_t len = 0;
      if (!parseUint32(argv[3], &len) || len == 0 || len > kMaxIoBytes) {
        snprintf(err, errLen, "read length 1..%u", static_cast<unsigned>(kMaxIoBytes));
        return false;
      }
      uint8_t data[kMaxIoBytes];
      size_t got = Wire.requestFrom(static_cast<int>(addr), static_cast<int>(len));
      if (got == 0) {
        snprintf(err, errLen, "i2c read failed");
        return false;
      }
      for (size_t i = 0; i < got; ++i) {
        if (Wire.available()) {
          data[i] = static_cast<uint8_t>(Wire.read());
        } else {
          data[i] = 0;
        }
      }
      printHexLine("I2C RX:", data, got);
      if (got != len) {
        Serial.printf("WARN: requested %u, got %u\n", static_cast<unsigned>(len), static_cast<unsigned>(got));
      }
      return true;
    }
    if (strcmp(argv[1], "writeread") == 0) {
      if (argc < 5) {
        snprintf(err, errLen, "i2c writeread <addr> <len> <byte...>");
        return false;
      }
      if (!i2c_.active) {
        snprintf(err, errLen, "i2c not started");
        return false;
      }
      uint8_t addr = 0;
      if (!parseI2cAddress(argv[2], &addr, err, errLen)) {
        return false;
      }
      uint32_t len = 0;
      if (!parseUint32(argv[3], &len) || len == 0 || len > kMaxIoBytes) {
        snprintf(err, errLen, "read length 1..%u", static_cast<unsigned>(kMaxIoBytes));
        return false;
      }
      uint8_t data[kMaxIoBytes];
      size_t count = 0;
      for (int i = 4; i < argc; ++i) {
        if (count >= kMaxIoBytes) {
          snprintf(err, errLen, "too many bytes (max %u)", static_cast<unsigned>(kMaxIoBytes));
          return false;
        }
        if (!parseByte(argv[i], &data[count], err, errLen)) {
          return false;
        }
        count++;
      }
      if (count == 0) {
        snprintf(err, errLen, "no data bytes");
        return false;
      }
      Wire.beginTransmission(addr);
      Wire.write(data, count);
      uint8_t rc = Wire.endTransmission(false);
      if (rc != 0) {
        const char *msg = (rc == 1) ? "data too long"
                          : (rc == 2) ? "address nack"
                          : (rc == 3) ? "data nack"
                          : (rc == 4) ? "bus error"
                                      : "unknown error";
        snprintf(err, errLen, "i2c write failed (%s)", msg);
        return false;
      }
      uint8_t rx[kMaxIoBytes];
      size_t got = Wire.requestFrom(static_cast<int>(addr), static_cast<int>(len));
      if (got == 0) {
        snprintf(err, errLen, "i2c read failed");
        return false;
      }
      for (size_t i = 0; i < got; ++i) {
        if (Wire.available()) {
          rx[i] = static_cast<uint8_t>(Wire.read());
        } else {
          rx[i] = 0;
        }
      }
      printHexLine("I2C RX:", rx, got);
      if (got != len) {
        Serial.printf("WARN: requested %u, got %u\n", static_cast<unsigned>(len), static_cast<unsigned>(got));
      }
      return true;
    }
    if (strcmp(argv[1], "stop") == 0) {
      i2cStop();
      return true;
    }
    snprintf(err, errLen, "unknown i2c subcommand");
    return false;
  }

  if (strcmp(argv[0], "spi") == 0) {
    if (mode_ != Mode::Spi) {
      snprintf(err, errLen, "not in spi mode");
      return false;
    }
    if (argc < 2) {
      snprintf(err, errLen, "spi requires subcommand");
      return false;
    }
    if (strcmp(argv[1], "start") == 0) {
      int sck = -1;
      int miso = -1;
      int mosi = -1;
      int cs = -1;
      int argIndex = 2;

      if (argc == 2 || (argc >= 3 && strcmp(argv[2], "default") == 0)) {
        argIndex = (argc >= 3 && strcmp(argv[2], "default") == 0) ? 3 : 2;
        if (!defaultSpiPins(&sck, &miso, &mosi, &cs, err, errLen)) {
          return false;
        }
      } else {
        if (argc < 5) {
          snprintf(err, errLen, "spi start <sck> <miso> <mosi> [cs]");
          return false;
        }
        if (!parseInt(argv[2], &sck) || !parseInt(argv[3], &miso) || !parseInt(argv[4], &mosi)) {
          snprintf(err, errLen, "invalid spi pins");
          return false;
        }
        argIndex = 5;
      }
      if (argc > argIndex) {
        if (!parseInt(argv[argIndex], &cs)) {
          snprintf(err, errLen, "invalid cs pin");
          return false;
        }
      }
      if (!validatePin(sck, err, errLen) || !validatePin(miso, err, errLen) ||
          !validatePin(mosi, err, errLen)) {
        return false;
      }
      if (cs >= 0 && !validatePin(cs, err, errLen)) {
        return false;
      }
      int spiPins[4] = {sck, miso, mosi, cs};
      if (!pinsAreUnique(spiPins, 4, "spi", err, errLen)) {
        return false;
      }
      spiStop();
      if (!pins_.claim(sck, "SPI SCK", err, errLen)) {
        return false;
      }
      if (!pins_.claim(miso, "SPI MISO", err, errLen)) {
        pins_.release(sck, true);
        return false;
      }
      if (!pins_.claim(mosi, "SPI MOSI", err, errLen)) {
        pins_.release(sck, true);
        pins_.release(miso, true);
        return false;
      }
      if (cs >= 0) {
        if (!pins_.claim(cs, "SPI CS", err, errLen)) {
          pins_.release(sck, true);
          pins_.release(miso, true);
          pins_.release(mosi, true);
          return false;
        }
      }
      SPI.begin(sck, miso, mosi, (cs >= 0) ? cs : -1);
      if (cs >= 0) {
        pinMode(cs, OUTPUT);
        digitalWrite(cs, HIGH);
      }
      spi_.active = true;
      spi_.sckPin = sck;
      spi_.misoPin = miso;
      spi_.mosiPin = mosi;
      spi_.csPin = (cs >= 0) ? cs : -1;
      return true;
    }
    if (strcmp(argv[1], "freq") == 0) {
      if (argc < 3) {
        snprintf(err, errLen, "spi freq <hz>");
        return false;
      }
      if (!spi_.active) {
        snprintf(err, errLen, "spi not started");
        return false;
      }
      uint32_t freq = 0;
      if (!parseUint32(argv[2], &freq) || freq == 0) {
        snprintf(err, errLen, "invalid spi frequency");
        return false;
      }
      spi_.freqHz = freq;
      return true;
    }
    if (strcmp(argv[1], "mode") == 0) {
      if (argc < 3) {
        snprintf(err, errLen, "spi mode <0|1|2|3>");
        return false;
      }
      if (!spi_.active) {
        snprintf(err, errLen, "spi not started");
        return false;
      }
      int mode = 0;
      if (!parseInt(argv[2], &mode) || mode < 0 || mode > 3) {
        snprintf(err, errLen, "spi mode must be 0..3");
        return false;
      }
      spi_.mode = static_cast<uint8_t>(mode);
      return true;
    }
    if (strcmp(argv[1], "order") == 0) {
      if (argc < 3) {
        snprintf(err, errLen, "spi order msb|lsb");
        return false;
      }
      if (!spi_.active) {
        snprintf(err, errLen, "spi not started");
        return false;
      }
      if (strcmp(argv[2], "msb") == 0) {
        spi_.bitOrder = MSBFIRST;
        return true;
      }
      if (strcmp(argv[2], "lsb") == 0) {
        spi_.bitOrder = LSBFIRST;
        return true;
      }
      snprintf(err, errLen, "spi order must be msb or lsb");
      return false;
    }
    if (strcmp(argv[1], "cs") == 0) {
      if (argc < 3) {
        snprintf(err, errLen, "spi cs <pin|-1>");
        return false;
      }
      if (!spi_.active) {
        snprintf(err, errLen, "spi not started");
        return false;
      }
      int cs = -1;
      if (!parseInt(argv[2], &cs)) {
        snprintf(err, errLen, "invalid cs pin");
        return false;
      }
      if (cs >= 0 && !validatePin(cs, err, errLen)) {
        return false;
      }
      if (cs >= 0 && (cs == spi_.sckPin || cs == spi_.misoPin || cs == spi_.mosiPin)) {
        snprintf(err, errLen, "spi cs pin must differ from sck/miso/mosi");
        return false;
      }
      if (spi_.csPin >= 0) {
        pins_.release(spi_.csPin, true);
      }
      spi_.csPin = -1;
      if (cs >= 0) {
        if (!pins_.claim(cs, "SPI CS", err, errLen)) {
          return false;
        }
        pinMode(cs, OUTPUT);
        digitalWrite(cs, HIGH);
        spi_.csPin = cs;
      }
      return true;
    }
    if (strcmp(argv[1], "dummy") == 0) {
      if (argc < 3) {
        snprintf(err, errLen, "spi dummy <byte>");
        return false;
      }
      if (!spi_.active) {
        snprintf(err, errLen, "spi not started");
        return false;
      }
      uint8_t val = 0;
      if (!parseByte(argv[2], &val, err, errLen)) {
        return false;
      }
      spi_.dummyByte = val;
      return true;
    }
    if (strcmp(argv[1], "xfer") == 0) {
      if (argc < 3) {
        snprintf(err, errLen, "spi xfer <byte...>");
        return false;
      }
      if (!spi_.active) {
        snprintf(err, errLen, "spi not started");
        return false;
      }
      uint8_t tx[kMaxIoBytes];
      uint8_t rx[kMaxIoBytes];
      size_t count = 0;
      for (int i = 2; i < argc; ++i) {
        if (count >= kMaxIoBytes) {
          snprintf(err, errLen, "too many bytes (max %u)", static_cast<unsigned>(kMaxIoBytes));
          return false;
        }
        if (!parseByte(argv[i], &tx[count], err, errLen)) {
          return false;
        }
        count++;
      }
      if (count == 0) {
        snprintf(err, errLen, "no data bytes");
        return false;
      }
      SPI.beginTransaction(SPISettings(spi_.freqHz, spi_.bitOrder, spi_.mode));
      if (spi_.csPin >= 0) {
        digitalWrite(spi_.csPin, LOW);
      }
      for (size_t i = 0; i < count; ++i) {
        rx[i] = SPI.transfer(tx[i]);
      }
      if (spi_.csPin >= 0) {
        digitalWrite(spi_.csPin, HIGH);
      }
      SPI.endTransaction();
      printHexLine("SPI RX:", rx, count);
      return true;
    }
    if (strcmp(argv[1], "read") == 0) {
      if (argc < 3) {
        snprintf(err, errLen, "spi read <len>");
        return false;
      }
      if (!spi_.active) {
        snprintf(err, errLen, "spi not started");
        return false;
      }
      uint32_t len = 0;
      if (!parseUint32(argv[2], &len) || len == 0 || len > kMaxIoBytes) {
        snprintf(err, errLen, "read length 1..%u", static_cast<unsigned>(kMaxIoBytes));
        return false;
      }
      uint8_t rx[kMaxIoBytes];
      SPI.beginTransaction(SPISettings(spi_.freqHz, spi_.bitOrder, spi_.mode));
      if (spi_.csPin >= 0) {
        digitalWrite(spi_.csPin, LOW);
      }
      for (size_t i = 0; i < len; ++i) {
        rx[i] = SPI.transfer(spi_.dummyByte);
      }
      if (spi_.csPin >= 0) {
        digitalWrite(spi_.csPin, HIGH);
      }
      SPI.endTransaction();
      printHexLine("SPI RX:", rx, len);
      return true;
    }
    if (strcmp(argv[1], "stop") == 0) {
      spiStop();
      return true;
    }
    snprintf(err, errLen, "unknown spi subcommand");
    return false;
  }

  snprintf(err, errLen, "unknown command");
  return false;
}

void RuntimeProbe::handleConsole() {
  size_t processed = 0;
  while (processed < kMaxConsoleBytesPerUpdate && Serial.available()) {
    processed++;
    const char c = static_cast<char>(Serial.read());
    if (c == '\r' || c == '\n') {
      if (lineOverflow_) {
        Serial.println("ERR: command too long");
        lineLen_ = 0;
        lineOverflow_ = false;
        continue;
      }
      if (lineLen_ == 0) {
        continue;
      }
      lineBuf_[lineLen_] = '\0';
      char *argv[24];
      int argc = tokenize(lineBuf_, argv, 24);
      if (argc < 0) {
        Serial.println("ERR: too many arguments");
        lineLen_ = 0;
        continue;
      }
      if (argc == 0) {
        lineLen_ = 0;
        continue;
      }
      char err[96] = {0};
      if (handleCommand(argc, argv, err, sizeof(err))) {
        Serial.println("OK");
      } else {
        Serial.print("ERR: ");
        Serial.println(err[0] ? err : "unknown");
      }
      lineLen_ = 0;
      lineOverflow_ = false;
    } else if (c == '\b' || c == 0x7F) {
      if (lineLen_ > 0) {
        lineLen_--;
      }
    } else if (isprint(static_cast<unsigned char>(c))) {
      if (lineLen_ < sizeof(lineBuf_) - 1) {
        lineBuf_[lineLen_++] = c;
      } else {
        lineOverflow_ = true;
      }
    }
  }
}

void RuntimeProbe::begin() {
  Serial.begin(kSerialBaud);
  jtagSetDefaultSeq();

  Serial.println();
  Serial.println("ESP32 Runtime Interface & GPIO Probe");
  Serial.printf("Chip: %s rev %d\n", ESP.getChipModel(), ESP.getChipRevision());
  Serial.printf("Build: %s %s\n", __DATE__, __TIME__);
  Serial.println("Type 'help' for commands.");
  Serial.println("Examples:");
  Serial.println("  mode gpio");
  Serial.println("  gpio set 5");
  Serial.println("  gpio toggle 100000");
  Serial.println("  gpio pulse 5 20");
  Serial.println("  mode uart");
  Serial.println("  uart start 1 17 18 921600");
  Serial.println("  uart pattern 55");
  Serial.println("  mode jtag");
  Serial.println("  jtag bitbang 39 40 41 42 10000");
  Serial.println("  mode i2c");
  Serial.println("  i2c start 8 9 400000");
  Serial.println("  i2c scan");
  Serial.println("  mode spi");
  Serial.println("  spi start 18 19 23 5");
  Serial.println("  spi xfer 0x9F");
}

void RuntimeProbe::update() {
  handleConsole();
  gpioUpdate();
  uartUpdate();
  jtagUpdate();
  i2cUpdate();
}
