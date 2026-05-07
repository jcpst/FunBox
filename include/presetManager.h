#pragma once
#ifndef PRESET_MANAGER_H
#define PRESET_MANAGER_H

#include "daisy_petal.h"
#include "funbox.h"
#include <cstdint>
#include <new>

/*
PresetManager class
author: extracted from Ceres pedal code
date  : 5/6/2026

Reusable MIDI preset manager for Funbox pedals. Handles 128-slot preset
save/recall via MIDI Program Change, program-mode entry via footswitch hold,
knob-override detection (physical knob movement exits preset mode), and
confirm-blink LED state for the save workflow.

Usage:
  1. Declare a global `PresetManager presets;`
  2. In main(), after hw.Init(): `presets.Init(hw);`
  3. In AudioCallback, after ProcessAnalogControls/ProcessDigitalControls:
       presets.CheckKnobOverride();
  4. In UpdateButtons(): `presets.UpdateFootswitch(hw);`
  5. In updateSwitch*(): `presets.OnSwitchChanged();`
  6. In MIDI ProgramChange handler:
       presets.HandleProgramChange(prog, bypass, effectMode, led);
  7. In main loop:
       presets.FlushIfNeeded();
       float ledBrightness = presets.GetLedBrightness(normalValue);
*/

namespace funbox {

static const uint8_t PRESET_MAGIC          = 0xA5;
static const float   KNOB_MOVE_THRESHOLD   = 0.05f;
static const uint32_t PROGRAM_HOLD_MS      = 3000;
static const uint32_t PROGRAM_BLINK_MS     = 500;
static const uint32_t CONFIRM_BLINK_MS     = 100;
static const int      CONFIRM_BLINK_COUNT  = 3;

struct Preset {
    uint8_t magic;       // PRESET_MAGIC if this slot has been written
    bool    bypass;
    int     effectMode;
    float   knobs[6];    // Raw 0–1 knob values (pre-parameter-mapping)
};

struct PresetBank {
    Preset presets[128];

    bool operator!=(const PresetBank& a) const {
        for (int i = 0; i < 128; i++) {
            if (a.presets[i].magic      != presets[i].magic)      return true;
            if (a.presets[i].bypass     != presets[i].bypass)     return true;
            if (a.presets[i].effectMode != presets[i].effectMode) return true;
            for (int k = 0; k < 6; k++) {
                if (a.presets[i].knobs[k] != presets[i].knobs[k]) return true;
            }
        }
        return false;
    }
};


class PresetManager
{
  public:
    PresetManager() {}
    ~PresetManager() {}

    // ---- State readable by the pedal ----

    // True when a recalled preset is active (knob values come from preset)
    bool  usePreset        = false;
    // True when preset's effectMode overrides the physical switch
    bool  presetModeActive = false;
    // The effectMode stored in the active preset
    int   presetMode       = 0;
    // Shadow knob values (raw 0–1) from the last recalled preset
    float presetKnobs[6]   = {};

    // True while in program mode (waiting for PC message to save)
    bool  programMode       = false;


    /// Initialize preset storage. Call once in main() after hw.Init().
    void Init(daisy::DaisyPetal& hw)
    {
        hw_ = &hw;
        storage_ = new(storageBuf_) daisy::PersistentStorage<PresetBank>(hw.seed.qspi);
        PresetBank defaultBank = {};
        storage_->Init(defaultBank);
    }

    /// Call from AudioCallback after ProcessAnalogControls/ProcessDigitalControls.
    /// Checks if any physical knob has moved enough to exit preset override.
    void CheckKnobOverride()
    {
        if (!usePreset) return;
        static const int knobMap[6] = {
            Funbox::KNOB_1, Funbox::KNOB_2, Funbox::KNOB_3,
            Funbox::KNOB_4, Funbox::KNOB_5, Funbox::KNOB_6
        };
        for (int k = 0; k < 6; k++) {
            float diff = hw_->knob[knobMap[k]].Value() - presetKnobs[k];
            if (diff < 0.0f) diff = -diff;
            if (diff > KNOB_MOVE_THRESHOLD) {
                usePreset        = false;
                presetModeActive = false;
                return;
            }
        }
    }

    /// Call when a physical switch position changes to clear preset mode override.
    void OnSwitchChanged()
    {
        if (presetModeActive)
            presetModeActive = false;
    }

    /// Call from UpdateButtons(). Handles 3-second hold to enter/exit program mode,
    /// and short-press bypass toggle. Returns true when bypass was toggled by short press.
    bool UpdateFootswitch(daisy::DaisyPetal& hw, bool& bypass, daisy::Led& led)
    {
        bool toggled = false;

        // Detect 3-second hold on footswitch 1 to enter/exit program mode
        if (hw.switches[Funbox::FOOTSWITCH_1].Pressed()) {
            if (hw.switches[Funbox::FOOTSWITCH_1].TimeHeldMs() >= PROGRAM_HOLD_MS
                && !longHoldTriggered_) {
                longHoldTriggered_ = true;
                programMode = !programMode;
                if (programMode) {
                    blinkTimer_     = daisy::System::GetNow();
                    ledBlinkState_  = true;
                }
            }
        }

        // On release: short press = bypass toggle, long hold = already handled
        if (hw.switches[Funbox::FOOTSWITCH_1].FallingEdge()) {
            if (longHoldTriggered_) {
                longHoldTriggered_ = false;
            } else {
                bypass = !bypass;
                led.Set(bypass ? 0.0f : 1.0f);
                toggled = true;
            }
        }

        led.Update();
        return toggled;
    }

    /// Handle a MIDI Program Change message.
    /// In program mode: saves current state to the slot.
    /// Outside program mode: recalls the preset into bypass/effectMode.
    /// Returns true if the message was handled (slot was valid or save occurred).
    bool HandleProgramChange(uint8_t prog, bool& bypass, int& effectMode, daisy::Led& led)
    {
        if (programMode) {
            // ---- SAVE current settings to this program slot ----
            PresetBank& bank = storage_->GetSettings();
            Preset& slot     = bank.presets[prog];

            slot.magic      = PRESET_MAGIC;
            slot.bypass     = bypass;
            slot.effectMode = effectMode;
            slot.knobs[0]   = hw_->knob[Funbox::KNOB_1].Value();
            slot.knobs[1]   = hw_->knob[Funbox::KNOB_2].Value();
            slot.knobs[2]   = hw_->knob[Funbox::KNOB_3].Value();
            slot.knobs[3]   = hw_->knob[Funbox::KNOB_4].Value();
            slot.knobs[4]   = hw_->knob[Funbox::KNOB_5].Value();
            slot.knobs[5]   = hw_->knob[Funbox::KNOB_6].Value();

            triggerSave_ = true;
            programMode  = false;

            // Start confirm blink sequence
            confirmBlinking_   = true;
            confirmBlinkCount_ = 0;
            confirmBlinkTimer_ = daisy::System::GetNow();
            ledBlinkState_     = true;
            return true;
        }
        else
        {
            // ---- RECALL preset from this program slot ----
            PresetBank& bank = storage_->GetSettings();
            Preset& slot     = bank.presets[prog];

            if (slot.magic != PRESET_MAGIC)
                return false;  // Uninitialized slot — ignore

            bypass           = slot.bypass;
            presetMode       = slot.effectMode;
            effectMode       = slot.effectMode;
            presetModeActive = true;

            for (int k = 0; k < 6; k++)
                presetKnobs[k] = slot.knobs[k];

            usePreset = true;

            led.Set(bypass ? 0.0f : 1.0f);
            return true;
        }
    }

    /// Write to QSPI flash if a save is pending. Call from main loop.
    void FlushIfNeeded()
    {
        if (triggerSave_) {
            storage_->Save();
            triggerSave_ = false;
        }
    }

    /// Returns the appropriate LED1 brightness, handling program-mode blink
    /// and confirm-blink automatically. Pass the normal (non-program-mode)
    /// brightness value; it will be returned when not in a special LED state.
    float GetLedBrightness(float normalBrightness)
    {
        uint32_t now = daisy::System::GetNow();

        if (programMode && !confirmBlinking_) {
            // Program mode blink (500ms toggle)
            if (now - blinkTimer_ >= PROGRAM_BLINK_MS) {
                blinkTimer_    = now;
                ledBlinkState_ = !ledBlinkState_;
            }
            return ledBlinkState_ ? 1.0f : 0.0f;
        }

        if (confirmBlinking_) {
            // Confirm blink after save (3 blinks at 100ms on/off)
            if (now - confirmBlinkTimer_ >= CONFIRM_BLINK_MS) {
                confirmBlinkTimer_ = now;
                ledBlinkState_     = !ledBlinkState_;
                if (!ledBlinkState_) {
                    confirmBlinkCount_++;
                    if (confirmBlinkCount_ >= CONFIRM_BLINK_COUNT) {
                        confirmBlinking_ = false;
                    }
                }
            }
            return ledBlinkState_ ? 1.0f : 0.0f;
        }

        return normalBrightness;
    }

    /// True while a confirm blink sequence is active (suppress normal LED logic)
    bool IsBlinking() const { return confirmBlinking_ || programMode; }

    /// Get raw knob value: returns preset value if active, otherwise live knob
    float GetKnob(int idx) const
    {
        static const int knobMap[6] = {
            Funbox::KNOB_1, Funbox::KNOB_2, Funbox::KNOB_3,
            Funbox::KNOB_4, Funbox::KNOB_5, Funbox::KNOB_6
        };
        return usePreset ? presetKnobs[idx] : hw_->knob[knobMap[idx]].Value();
    }

    /// Returns presetMode if preset override is active, otherwise the physical mode
    int GetEffectMode(int physicalMode) const
    {
        return presetModeActive ? presetMode : physicalMode;
    }


  private:
    daisy::DaisyPetal*                       hw_  = nullptr;
    daisy::PersistentStorage<PresetBank>*    storage_ = nullptr;
    alignas(daisy::PersistentStorage<PresetBank>)
        uint8_t storageBuf_[sizeof(daisy::PersistentStorage<PresetBank>)];

    bool     longHoldTriggered_ = false;
    bool     triggerSave_       = false;

    // LED blink state
    bool     confirmBlinking_   = false;
    int      confirmBlinkCount_ = 0;
    uint32_t confirmBlinkTimer_ = 0;
    uint32_t blinkTimer_        = 0;
    bool     ledBlinkState_     = false;
};

} // namespace funbox

#endif




