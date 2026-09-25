#include <Arduino.h>
#include "driver/twai.h"
#include "soc/soc_caps.h"

// The ESP32-S3 has one built-in TWAI (CAN 2.0) controller.  It needs an
// external CAN transceiver; TX/RX below connect to that transceiver's logic
// pins, not directly to CANH/CANL.
#ifndef CAN_TX_PIN
#define CAN_TX_PIN 4
#endif

#ifndef CAN_RX_PIN
#define CAN_RX_PIN 5
#endif

#ifndef SERIAL_BAUD_RATE
#define SERIAL_BAUD_RATE 500000
#endif

static_assert(SOC_TWAI_SUPPORTED, "This firmware requires an ESP32 target with a TWAI controller.");

namespace {
constexpr size_t kCommandBufferSize = 32;
constexpr uint32_t kCanReceiveTimeoutMs = 0;
constexpr uint32_t kCanTransmitTimeoutMs = 20;

bool controllerOpen = false;
bool timestampsEnabled = false;
bool appendLineFeed = false;
uint16_t canSpeedKbps = 500;
twai_timing_config_t timingConfig = TWAI_TIMING_CONFIG_500KBITS();
char commandBuffer[kCommandBufferSize] = {};
size_t commandLength = 0;

const char kHexDigits[] = "0123456789ABCDEF";

void acknowledge() {
  Serial.write('\r');
}

void reject() {
  Serial.write('\a');
}

bool hexValue(char value, uint8_t &result) {
  if (value >= '0' && value <= '9') {
    result = static_cast<uint8_t>(value - '0');
    return true;
  }
  if (value >= 'A' && value <= 'F') {
    result = static_cast<uint8_t>(value - 'A' + 10);
    return true;
  }
  if (value >= 'a' && value <= 'f') {
    result = static_cast<uint8_t>(value - 'a' + 10);
    return true;
  }
  return false;
}

bool parseHex(const char *text, size_t digits, uint32_t &result) {
  result = 0;
  for (size_t index = 0; index < digits; ++index) {
    uint8_t nibble = 0;
    if (!hexValue(text[index], nibble)) {
      return false;
    }
    result = (result << 4U) | nibble;
  }
  return true;
}

bool selectBitrate(char command) {
  switch (command) {
    case '0':
      timingConfig = TWAI_TIMING_CONFIG_10KBITS();
      canSpeedKbps = 10;
      return true;
    case '1':
      timingConfig = TWAI_TIMING_CONFIG_20KBITS();
      canSpeedKbps = 20;
      return true;
    case '2':
      timingConfig = TWAI_TIMING_CONFIG_50KBITS();
      canSpeedKbps = 50;
      return true;
    case '3':
      timingConfig = TWAI_TIMING_CONFIG_100KBITS();
      canSpeedKbps = 100;
      return true;
    case '4':
      timingConfig = TWAI_TIMING_CONFIG_125KBITS();
      canSpeedKbps = 125;
      return true;
    case '5':
      timingConfig = TWAI_TIMING_CONFIG_250KBITS();
      canSpeedKbps = 250;
      return true;
    case '6':
      timingConfig = TWAI_TIMING_CONFIG_500KBITS();
      canSpeedKbps = 500;
      return true;
    case '7':
      timingConfig = TWAI_TIMING_CONFIG_800KBITS();
      canSpeedKbps = 800;
      return true;
    case '8':
      timingConfig = TWAI_TIMING_CONFIG_1MBITS();
      canSpeedKbps = 1000;
      return true;
    default:
      return false;
  }
}

bool openController() {
  if (controllerOpen) {
    return true;
  }

  const twai_general_config_t generalConfig = TWAI_GENERAL_CONFIG_DEFAULT(
      static_cast<gpio_num_t>(CAN_TX_PIN), static_cast<gpio_num_t>(CAN_RX_PIN), TWAI_MODE_NORMAL);
  const twai_filter_config_t filterConfig = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  if (twai_driver_install(&generalConfig, &timingConfig, &filterConfig) != ESP_OK) {
    return false;
  }
  if (twai_start() != ESP_OK) {
    twai_driver_uninstall();
    return false;
  }

  controllerOpen = true;
  return true;
}

bool closeController() {
  if (!controllerOpen) {
    return true;
  }
  if (twai_stop() != ESP_OK) {
    return false;
  }
  if (twai_driver_uninstall() != ESP_OK) {
    return false;
  }

  controllerOpen = false;
  return true;
}

bool sendFrame(const char *command, size_t length, bool remoteFrame, bool extendedFrame) {
  const size_t identifierDigits = extendedFrame ? 8 : 3;
  const size_t dlcIndex = 1 + identifierDigits;
  const size_t dataIndex = dlcIndex + 1;
  const size_t minimumLength = dataIndex;

  if (!controllerOpen || length < minimumLength) {
    return false;
  }

  uint32_t identifier = 0;
  uint32_t parsedDlc = 0;
  if (!parseHex(command + 1, identifierDigits, identifier) ||
      !parseHex(command + dlcIndex, 1, parsedDlc) || parsedDlc > 8) {
    return false;
  }

  // Lawicel encodes an extended identifier as 8 hex digits containing only
  // the 29-bit CAN identifier. SavvyCAN's LAWICEL backend, however, sets bit
  // 31 as a private extended-frame marker before it serializes a T/R command.
  // Accept that known non-standard form (for example T80001100102\r) and
  // remove only the marker before giving the ID to ESP-IDF's TWAI driver.
  if (extendedFrame && (identifier & 0x80000000UL) != 0U) {
    identifier &= 0x1FFFFFFFUL;
  }

  const uint32_t maximumIdentifier = extendedFrame ? 0x1FFFFFFFUL : 0x7FFUL;
  if (identifier > maximumIdentifier) {
    return false;
  }

  const uint8_t dlc = static_cast<uint8_t>(parsedDlc);
  const size_t expectedLength = remoteFrame ? minimumLength : dataIndex + (static_cast<size_t>(dlc) * 2U);
  if (length != expectedLength) {
    return false;
  }

  twai_message_t frame = {};
  frame.identifier = identifier;
  frame.extd = extendedFrame ? 1 : 0;
  frame.rtr = remoteFrame ? 1 : 0;
  frame.data_length_code = dlc;

  if (!remoteFrame) {
    for (uint8_t byteIndex = 0; byteIndex < dlc; ++byteIndex) {
      uint32_t byteValue = 0;
      if (!parseHex(command + dataIndex + (static_cast<size_t>(byteIndex) * 2U), 2, byteValue)) {
        return false;
      }
      frame.data[byteIndex] = static_cast<uint8_t>(byteValue);
    }
  }

  return twai_transmit(&frame, pdMS_TO_TICKS(kCanTransmitTimeoutMs)) == ESP_OK;
}

void printStatus() {
  twai_status_info_t status = {};
  uint8_t flags = 0;
  if (controllerOpen && twai_get_status_info(&status) == ESP_OK) {
    if (status.state == TWAI_STATE_BUS_OFF) {
      flags |= 0x80;
    }
    if (status.state == TWAI_STATE_RECOVERING) {
      flags |= 0x40;
    }
    if (status.tx_error_counter >= 96) {
      flags |= 0x20;
    }
    if (status.rx_error_counter >= 96) {
      flags |= 0x10;
    }
  }
  Serial.print("F");
  Serial.print(kHexDigits[(flags >> 4U) & 0x0FU]);
  Serial.print(kHexDigits[flags & 0x0FU]);
  acknowledge();
}

void printHelp() {
  Serial.printf("ESP32-S3 TWAI SLCAN\r\n"
                "O/C open/close CAN; S0..S8 select bitrate; Z0/Z1 timestamps\r\n"
                "Pins: TX=%d RX=%d; CAN=%u kbit/s; controller=%s\r\n",
                CAN_TX_PIN, CAN_RX_PIN, canSpeedKbps, controllerOpen ? "open" : "closed");
  reject();  // Help is an extension, not a LAWICEL command.
}

void handleCommand(const char *command, size_t length) {
  if (length == 0) {
    reject();
    return;
  }

  switch (command[0]) {
    case 'O':
      if (length == 1 && openController()) {
        acknowledge();
      } else {
        reject();
      }
      break;

    case 'C':
      if (length == 1 && closeController()) {
        acknowledge();
      } else {
        reject();
      }
      break;

    case 'S':
      if (length == 2 && !controllerOpen && selectBitrate(command[1])) {
        acknowledge();
      } else {
        reject();
      }
      break;

    case 's':
      reject();  // Custom BTR timing is intentionally not exposed.
      break;

    case 't':
      sendFrame(command, length, false, false) ? acknowledge() : reject();
      break;

    case 'T':
      sendFrame(command, length, false, true) ? acknowledge() : reject();
      break;

    case 'r':
      sendFrame(command, length, true, false) ? acknowledge() : reject();
      break;

    case 'R':
      sendFrame(command, length, true, true) ? acknowledge() : reject();
      break;

    case 'Z':
      if (length == 2 && (command[1] == '0' || command[1] == '1')) {
        timestampsEnabled = command[1] == '1';
        acknowledge();
      } else {
        reject();
      }
      break;

    case 'F':
      if (length == 1) {
        printStatus();
      } else {
        reject();
      }
      break;

    case 'V':
      if (length == 1) {
        Serial.print("V0100");
        acknowledge();
      } else {
        reject();
      }
      break;

    case 'N':
      if (length == 1) {
        Serial.print("N0001");
        acknowledge();
      } else {
        reject();
      }
      break;

    case 'M':
    case 'm':
      // The controller must be reinstalled to change filtering.  Accept the
      // standard command shape while retaining the documented accept-all mode.
      if (!controllerOpen && length == 9) {
        uint32_t ignored = 0;
        if (parseHex(command + 1, 8, ignored)) {
          acknowledge();
          break;
        }
      }
      reject();
      break;

    case 'l':
      if (length == 1) {
        appendLineFeed = !appendLineFeed;
        acknowledge();
      } else {
        reject();
      }
      break;

    case 'h':
      if (length == 1) {
        printHelp();
      } else {
        reject();
      }
      break;

    default:
      reject();
      break;
  }
}

void receiveSerialCommands() {
  while (Serial.available() > 0) {
    const char value = static_cast<char>(Serial.read());
    if (value == '\r') {
      handleCommand(commandBuffer, commandLength);
      commandLength = 0;
      continue;
    }

    // Accept LF after CR from a terminal, but do not treat it as a command.
    if (value == '\n' && commandLength == 0) {
      continue;
    }

    if (commandLength + 1 >= kCommandBufferSize) {
      commandLength = 0;
      reject();
      continue;
    }
    commandBuffer[commandLength++] = value;
  }
}

void emitFrame(const twai_message_t &frame) {
  // Extended frame (8 data bytes), timestamp, CR, and optional LF need 32 bytes.
  char output[32] = {};
  size_t position = 0;
  output[position++] = frame.rtr ? (frame.extd ? 'R' : 'r') : (frame.extd ? 'T' : 't');

  const uint8_t identifierDigits = frame.extd ? 8 : 3;
  for (int8_t shift = static_cast<int8_t>((identifierDigits - 1U) * 4U); shift >= 0; shift -= 4) {
    output[position++] = kHexDigits[(frame.identifier >> shift) & 0x0FU];
  }
  output[position++] = kHexDigits[frame.data_length_code & 0x0FU];

  if (!frame.rtr) {
    for (uint8_t index = 0; index < frame.data_length_code; ++index) {
      output[position++] = kHexDigits[(frame.data[index] >> 4U) & 0x0FU];
      output[position++] = kHexDigits[frame.data[index] & 0x0FU];
    }
  }

  if (timestampsEnabled) {
    const uint16_t timestamp = static_cast<uint16_t>(millis() % 60000UL);
    output[position++] = kHexDigits[(timestamp >> 12U) & 0x0FU];
    output[position++] = kHexDigits[(timestamp >> 8U) & 0x0FU];
    output[position++] = kHexDigits[(timestamp >> 4U) & 0x0FU];
    output[position++] = kHexDigits[timestamp & 0x0FU];
  }

  output[position++] = '\r';
  if (appendLineFeed) {
    output[position++] = '\n';
  }
  Serial.write(reinterpret_cast<const uint8_t *>(output), position);
}

void receiveCanFrames() {
  if (!controllerOpen) {
    return;
  }

  twai_message_t frame = {};
  while (twai_receive(&frame, pdMS_TO_TICKS(kCanReceiveTimeoutMs)) == ESP_OK) {
    emitFrame(frame);
  }
}
}  // namespace

void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  // Native USB CDC can enumerate after setup().  Do not wait forever for a
  // host: the adapter must still start and forward CAN without one attached.
  const uint32_t waitStart = millis();
  while (!Serial && millis() - waitStart < 1500U) {
    delay(10);
  }
}

void loop() {
  receiveSerialCommands();
  receiveCanFrames();
}