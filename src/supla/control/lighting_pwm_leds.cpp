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

#include "lighting_pwm_leds.h"

#include <supla/log_wrapper.h>

namespace Supla {
namespace Control {

namespace {
constexpr uint8_t LegacyAnalogWriteResolutionBits = 10;
constexpr uint32_t LegacyAnalogWriteFrequencyHz = 1000;

void ConfigureLegacyAnalogOutput(Supla::Io::IoPin &pin) {
  if (pin.io == nullptr) {
    pin.setAnalogOutputResolutionBits(LegacyAnalogWriteResolutionBits);
    pin.setAnalogOutputFrequency(LegacyAnalogWriteFrequencyHz);
  }
}  // Codex[2026-04-11]

bool IsSameDefaultIoPin(const Supla::Io::IoPin &a, const Supla::Io::IoPin &b) {
  return a.io == nullptr && b.io == nullptr && a.isSet() && b.isSet() &&
         a.getPin() == b.getPin();  // Codex[2026-04-11]
}

bool IsSameCustomIo(const Supla::Io::IoPin &a, const Supla::Io::IoPin &b) {
  return a.io != nullptr && b.io != nullptr &&
         a.io == b.io;  // Codex[2026-04-11]
}
}  // namespace

LightingPwmLeds::LightingPwmLeds(
    LightingPwmLeds *parent, int out1, int out2, int out3, int out4, int out5)
    : LightingPwmBase(parent), parentPwm(parent) {
  outputs[0].pin.setPin(out1);
  outputs[1].pin.setPin(out2);
  outputs[2].pin.setPin(out3);
  outputs[3].pin.setPin(out4);
  outputs[4].pin.setPin(out5);
  for (auto &output : outputs) {
    output.pin.setMode(OUTPUT);
  }
  applyDefaultChannelFunctions();
}

LightingPwmLeds::LightingPwmLeds(LightingPwmLeds *parent,
                                 Supla::Io::IoPin out1,
                                 Supla::Io::IoPin out2,
                                 Supla::Io::IoPin out3,
                                 Supla::Io::IoPin out4,
                                 Supla::Io::IoPin out5)
    : LightingPwmBase(parent), parentPwm(parent) {
  outputs[0].pin = out1;
  outputs[1].pin = out2;
  outputs[2].pin = out3;
  outputs[3].pin = out4;
  outputs[4].pin = out5;
  for (auto &output : outputs) {
    output.pin.setMode(OUTPUT);
  }
  applyDefaultChannelFunctions();
}

int LightingPwmLeds::getConfiguredOutputsCount() const {
  int count = 0;
  for (const auto &output : outputs) {
    if (!output.pin.isSet()) {
      break;
    }
    count++;
  }
  return count;
}

void LightingPwmLeds::applyDefaultChannelFunctions() {
  const int outputsCount = getConfiguredOutputsCount();
  uint32_t funcList = SUPLA_RGBW_BIT_FUNC_DIMMER;
  uint32_t defaultFunction = SUPLA_CHANNELFNC_DIMMER;

  if (outputsCount >= 2) {
    funcList |= SUPLA_RGBW_BIT_FUNC_DIMMER_CCT;
    defaultFunction = SUPLA_CHANNELFNC_DIMMER_CCT;
  }
  if (outputsCount >= 3) {
    funcList |= SUPLA_RGBW_BIT_FUNC_RGB_LIGHTING;
    defaultFunction = SUPLA_CHANNELFNC_RGBLIGHTING;
  }
  if (outputsCount >= 4) {
    funcList |= SUPLA_RGBW_BIT_FUNC_DIMMER_AND_RGB_LIGHTING;
    defaultFunction = SUPLA_CHANNELFNC_DIMMERANDRGBLIGHTING;
  }
  if (outputsCount >= 5) {
    funcList |= SUPLA_RGBW_BIT_FUNC_DIMMER_CCT_AND_RGB;
    defaultFunction = SUPLA_CHANNELFNC_DIMMER_CCT_AND_RGB;
  }

  getChannel()->setFuncList(funcList);
  getChannel()->setDefaultFunction(defaultFunction);
}

void LightingPwmLeds::setOutputIo(int outputIndex, Supla::Io::Base *io) {
  if (outputIndex < 0 || outputIndex >= kMaxOutputs) {
    return;
  }
  outputs[outputIndex].pin.io = io;
}

Supla::Io::Base *LightingPwmLeds::getOutputIo(int outputIndex) const {
  if (outputIndex < 0 || outputIndex >= kMaxOutputs) {
    return nullptr;
  }
  return outputs[outputIndex].pin.io;
}

int LightingPwmLeds::getOutputPin(int outputIndex) const {
  if (outputIndex < 0 || outputIndex >= kMaxOutputs) {
    return -1;
  }
  return outputs[outputIndex].pin.getPin();
}

void LightingPwmLeds::setRGBCCTValueOnDevice(uint32_t output[5],
                                             int usedOutputs) {
  if (!initDone || !enabled) {
    return;
  }

  bool changed = false;
  for (int i = 0; i < usedOutputs; i++) {
    if (outputs[i].lastSourceValue != static_cast<int32_t>(output[i])) {
      tryCounter = 0;
      changed = true;
      break;
    }
  }

  tryCounter++;

  if (!changed && tryCounter > 10) {
    tryCounter = 10;
    return;
  }

  for (int i = 0; i < usedOutputs; i++) {
    outputs[i].lastSourceValue = static_cast<int32_t>(output[i]);
    uint32_t value = output[i];
    uint32_t outputMax = outputs[i].pin.analogWriteMaxValue();
    if (outputMax > 0 && outputMax != maxHwValue) {
      value = static_cast<uint32_t>(
          (static_cast<uint64_t>(value) * outputMax + maxHwValue / 2) /
          maxHwValue);
    }
    if (outputs[i].lastDutyValue == static_cast<int32_t>(value)) {
      continue;
    }
    outputs[i].lastDutyValue = static_cast<int32_t>(value);
    outputs[i].pin.analogWrite(value);
  }
}

void LightingPwmLeds::applyPwmFrequencyToOutputs() {
  const uint16_t frequency = getPwmFrequency();
  Supla::Io::IoPin configuredOutputs[kMaxOutputs] = {};  // Codex[2026-04-11]
  int configuredCount = 0;  // Codex[2026-04-11]

  for (auto &output : outputs) {
    if (!output.pin.isSet()) {  // Codex[2026-04-11]
      continue;
    }

    bool alreadyConfigured = false;
    for (int i = 0; i < configuredCount; i++) {
      if (IsSameCustomIo(output.pin, configuredOutputs[i]) ||
          IsSameDefaultIoPin(output.pin, configuredOutputs[i])) {
        alreadyConfigured = true;
        break;
      }
    }

    if (!alreadyConfigured) {
      configuredOutputs[configuredCount++] = output.pin;
      // Codex[2026-04-11]
      output.pin.setAnalogOutputFrequency(frequency);
      // Codex[2026-04-11]
    }
  }
}

void LightingPwmLeds::onLoadConfig(SuplaDeviceClass *sdc) {
  LightingPwmBase::onLoadConfig(sdc);
  applyPwmFrequencyToOutputs();
}

void LightingPwmLeds::onInit() {
  if (initDone) {
    return;
  }

  uint32_t outputMaxValue = 0;
  for (const auto &output : outputs) {
    uint32_t value = output.pin.analogWriteMaxValue();
    if (value > outputMaxValue) {
      outputMaxValue = value;
    }
  }
  if (outputMaxValue > 0) {
    setMaxHwValue(static_cast<int>(outputMaxValue));
  }

  if (hasParent()) {
    SUPLA_LOG_DEBUG("Light[%d]: initialize parent PWM", getChannelNumber());
    parentPwm->onInit();  // Codex[2026-04-11]
  }

  applyPwmFrequencyToOutputs();

  for (auto &output : outputs) {
    if (!output.pin.isSet()) {  // Codex[2026-04-11]
      continue;
    }
    ConfigureLegacyAnalogOutput(output.pin);  // Codex[2026-04-11]
    output.pin.configureAnalogOutput();
    output.pin.pinMode();
  }

  LightingPwmBase::onInit();
}

}  // namespace Control
}  // namespace Supla
