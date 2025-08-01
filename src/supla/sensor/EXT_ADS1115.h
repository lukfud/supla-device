/*
 Copyright (C) AC SOFTWARE SP. Z O.O.

 This program is free software; you can redistribute it and/or
 modify it under the terms of the GNU General Public License
 as published by the Free Software Foundation; either version 2
 of the License, or (at your option) any later version.

 This program is distributed in the hope that it will be useful,
 but WITHOUT ANY WARRANTY; without even the implied warranty of
 MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 GNU General Public License for more details.

 You should have received a copy of the GNU General Public License
 along with this program; if not, write to the Free Software
 Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
*/

#pragma once

/*
Dependency: https://github.com/RobTillaart/ADS1X15
Use library manager to install it
*/

#include <ADS1X15.h>
#include <supla/io.h>
#include <supla/log_wrapper.h>

namespace Supla {
namespace Sensor {

class ExtADS1115 : public Supla::Io {
 public:
  explicit ExtADS1115(uint8_t address = 0x48,
                      TwoWire *wire = &Wire,
                      uint8_t dataRrate = 7)
      : Supla::Io(false), ads_(address, wire) {
    if (!ads_.begin()) {
      SUPLA_LOG_DEBUG("Unable to find ADS1115 at address 0x%x", address);
    } else {
      ads_.setDataRate(dataRrate);
      SUPLA_LOG_DEBUG("ADS1115 is connected at address: 0x%x, Gain: %d, "
                  "DataRate: %d", address, ads_.getGain(), ads_.getDataRate());
    }
  }

  void customPinMode(int channelNumber, uint8_t pin, uint8_t mode) override {}

  void customDigitalWrite(int channelNumber, uint8_t pin,
                                                       uint8_t val) override {}

  int customDigitalRead(int channelNumber, uint8_t pin) override {
    return 0;
  }

  unsigned int customPulseIn(int channelNumber, uint8_t pin, uint8_t value,
                                              uint64_t timeoutMicro) override {
    return 0;
  }

  void customAnalogWrite(int channelNumber, uint8_t pin, int val) override {}

  int customAnalogRead(int channelNumber, uint8_t pin) override {
    if (ads_.isConnected()) {
      if (pin > 3) {
        SUPLA_LOG_DEBUG("[ADS1115] invalid pin %d", pin);
        return -1;
      }
      ads_.setGain(gain_);
      return ads_.readADC(pin);
    } else {
      return -1;
    }
  }

  void setGain(uint8_t value) {
    gain_ = value;
  }

 protected:
  ::ADS1115 ads_;
  uint8_t gain_ = 0;
};

};  // namespace Sensor
};  // namespace Supla
