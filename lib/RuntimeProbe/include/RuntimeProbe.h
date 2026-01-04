#pragma once

#include <Arduino.h>

class RuntimeProbe {
 public:
  RuntimeProbe() = default;
  void begin();
  void update();

 private:
  static const uint32_t kSerialBaud = 2000000;

  enum class Mode {
    Off,
    Gpio,
    Uart,
    Jtag,
    Spi,
    I2c,
    Pwm,
  };

  class PinRegistry {
   public:
    static const int kMaxPins = 128;

    struct Entry {
      bool claimed;
      char purpose[24];
    };

    PinRegistry();
    void clear();
    bool isTracked(int pin) const;
    bool isClaimed(int pin) const;
    const char *purpose(int pin) const;
    bool claim(int pin, const char *purpose, char *err, size_t errLen);
    void release(int pin, bool setInput);
    void releaseAll(bool setInput);
    void printClaims() const;

   private:
    Entry entries_[kMaxPins];
  };

  struct GpioState {
    int pin = -1;
    bool pinClaimed = false;
    enum Waveform { None, Toggle, Pulse, Static } wave = None;
    bool level = false;
    uint32_t halfPeriodUs = 0;
    uint32_t pulseWidthUs = 0;
    uint32_t pulsePeriodUs = 0;
    uint32_t nextEdgeUs = 0;
    enum SafeMode { SafeInput, SafeLow, SafeHigh } safe = SafeInput;
  };

  struct UartState {
    bool active = false;
    uint8_t num = 1;
    int txPin = -1;
    int rxPin = -1;
    uint32_t baud = 115200;
    enum Pattern { Pat55, PatAA, PatRamp, PatAscii } pattern = Pat55;
    uint32_t txRateBps = 0;
    bool echo = false;
    bool forceUart0 = false;
    HardwareSerial *port = nullptr;
    uint8_t rampValue = 0;
    size_t asciiIndex = 0;
    uint64_t txBudget = 0;
    uint32_t lastMicros = 0;
    uint32_t rxCount = 0;
    uint32_t lastRxReportMs = 0;
  };

  struct JtagState {
    bool active = false;
    int tckPin = -1;
    int tmsPin = -1;
    int tdiPin = -1;
    int tdoPin = -1;
    uint32_t freqHz = 0;
    uint32_t halfPeriodUs = 0;
    uint32_t nextEdgeUs = 0;
    bool tckHigh = false;
    static const size_t kMaxSeq = 64;
    uint8_t tmsSeq[kMaxSeq];
    uint8_t tdiSeq[kMaxSeq];
    size_t seqLen = 0;
    size_t seqIndex = 0;
    uint64_t tdoShift = 0;
    uint8_t tdoCount = 0;
  };

  struct I2cState {
    bool active = false;
    int sdaPin = -1;
    int sclPin = -1;
    uint32_t freqHz = 100000;
    uint32_t timeoutMs = 50;
    bool scanActive = false;
    uint8_t scanStart = 0x03;
    uint8_t scanEnd = 0x77;
    uint8_t scanAddr = 0x03;
    uint8_t scanFound = 0;
    uint32_t scanNextMs = 0;
  };

  struct SpiState {
    bool active = false;
    int sckPin = -1;
    int misoPin = -1;
    int mosiPin = -1;
    int csPin = -1;
    uint32_t freqHz = 1000000;
    uint8_t mode = 0;
    uint8_t bitOrder = MSBFIRST;
    uint8_t dummyByte = 0x00;
  };

  PinRegistry pins_;
  Mode mode_ = Mode::Off;
  GpioState gpio_;
  UartState uart_;
  JtagState jtag_;
  I2cState i2c_;
  SpiState spi_;
  char lineBuf_[192];
  size_t lineLen_ = 0;

  const char *modeToString(Mode mode) const;
  bool validatePin(int pin, char *err, size_t errLen) const;
  bool parseInt(const char *s, int *out) const;
  bool parseUint32(const char *s, uint32_t *out) const;
  bool parseByte(const char *s, uint8_t *out, char *err, size_t errLen) const;
  bool parseI2cAddress(const char *s, uint8_t *out, char *err, size_t errLen) const;
  bool defaultUartPins(uint8_t num, int *txPin, int *rxPin, char *err, size_t errLen) const;
  bool defaultI2cPins(int *sdaPin, int *sclPin, char *err, size_t errLen) const;
  bool defaultSpiPins(int *sckPin, int *misoPin, int *mosiPin, int *csPin,
                      char *err, size_t errLen) const;

  void gpioReleasePin();
  bool gpioEnsureClaimed(char *err, size_t errLen);
  void gpioUpdate();

  HardwareSerial *uartByNum(uint8_t num);
  const char *uartPatternName(UartState::Pattern pat) const;
  uint8_t uartNextByte();
  void uartSendBytes(size_t count);
  void uartStop();
  void uartUpdate();

  void jtagSetDefaultSeq();
  void jtagApplySeq();
  void jtagStop();
  void jtagUpdate();

  void i2cStop();
  void i2cUpdate();

  void spiStop();

  void stopAllModes();
  int tokenize(char *line, char *argv[], int maxTokens) const;
  void printHelp() const;
  void printStatus() const;
  void printPins() const;
  void printDefaults() const;
  bool parseSeqBits(const char *input, char *err, size_t errLen);
  bool handleCommand(int argc, char *argv[], char *err, size_t errLen);
  void handleConsole();
};
