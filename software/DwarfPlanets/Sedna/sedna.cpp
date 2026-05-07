#include "daisy_petal.h"
#include "daisysp.h"
#include "funbox.h"
#include <cmath>
#include <cstdlib>

//
// Sedna - Rainbow Machine-inspired pitch shifting effect
// Varispeed delay pitch engine with dual voices, tone control, and Magic regeneration.
// Emulates FV-1 clock-manipulation style pitch warping using dual crossfaded delay taps.
//
// For the GuitarML Funbox_v3/Daisy Seed platform.
//

using namespace daisy;
using namespace daisysp;
using namespace funbox;

// ==================== Varispeed Pitch Shifter ====================
// Dual crossfaded delay taps with variable read speed.
// Supports fractional semitone transposition via powf ratio calculation.

#define PITCH_BUF_SIZE 16384

// SDRAM-placed buffers for the two pitch shifter voices
float DSY_SDRAM_BSS pitch_buf_primary[PITCH_BUF_SIZE];
float DSY_SDRAM_BSS pitch_buf_secondary[PITCH_BUF_SIZE];

struct VarPitchShifter
{
    float    *buf;
    uint32_t  write_pos;
    uint32_t  del_size;
    float     phs[2];       // dual phasors, 180 deg apart
    float     prev_phs[2];
    float     mod_freq;
    float     sr;
    bool      shift_up;
    float     fun;           // random modulation amount ("imperfection")
    float     fun_amt[2];
    float     slew[2];
    float     slew_coeff[2];

    void Init(float *buffer, float sample_rate)
    {
        buf       = buffer;
        sr        = sample_rate;
        write_pos = 0;
        del_size  = PITCH_BUF_SIZE / 2;
        phs[0]    = 0.0f;
        phs[1]    = 0.5f;
        prev_phs[0] = 0.0f;
        prev_phs[1] = 0.5f;
        mod_freq  = 0.0f;
        shift_up  = true;
        fun       = 0.0f;
        for (int i = 0; i < 2; i++)
        {
            fun_amt[i]    = 0.0f;
            slew[i]       = 0.0f;
            slew_coeff[i] = 0.0005f;
        }
        for (uint32_t i = 0; i < PITCH_BUF_SIZE; i++)
            buf[i] = 0.0f;
    }

    void SetTranspose(float semitones)
    {
        float ratio = powf(2.0f, fabsf(semitones) / 12.0f);
        shift_up    = (semitones >= 0.0f);
        mod_freq    = fabsf(ratio - 1.0f) * sr / (float)del_size;
        if (mod_freq < 0.0f)
            mod_freq = 0.0f;
    }

    void SetDelSize(uint32_t size)
    {
        if (size < 64)
            size = 64;
        if (size > PITCH_BUF_SIZE)
            size = PITCH_BUF_SIZE;
        del_size = size;
    }

    void SetFun(float f) { fun = f; }

    float Process(float in)
    {
        // Write input to circular buffer
        buf[write_pos] = in;

        // Advance phasors and sum two crossfaded delay taps
        float inc = mod_freq / sr;
        float val = 0.0f;

        for (int t = 0; t < 2; t++)
        {
            float p = phs[t];

            // Random modulation on phasor wrap (tape flutter character)
            if (prev_phs[t] > p + 0.5f)
            {
                fun_amt[t]    = fun * ((float)(rand() % 255) / 255.0f) * (del_size * 0.25f);
                slew_coeff[t] = 0.0002f + ((float)(rand() % 255) / 255.0f) * 0.001f;
            }
            slew[t] += slew_coeff[t] * (fun_amt[t] - slew[t]);
            prev_phs[t] = p;

            // Advance phasor
            p += inc;
            if (p >= 1.0f)
                p -= 1.0f;
            phs[t] = p;

            // Compute delay tap position
            float phase = shift_up ? (1.0f - p) : p;
            float delay = phase * (float)(del_size - 1) + slew[t];

            // Clamp delay
            if (delay < 0.0f)
                delay = 0.0f;
            if (delay > (float)(PITCH_BUF_SIZE - 2))
                delay = (float)(PITCH_BUF_SIZE - 2);

            // Crossfade gain: sin window on phasor phase
            float gain = sinf(p * PI_F);

            // Read with linear interpolation
            float read_pos = (float)write_pos - delay;
            if (read_pos < 0.0f)
                read_pos += (float)PITCH_BUF_SIZE;

            uint32_t idx  = (uint32_t)read_pos;
            float    frac = read_pos - (float)idx;
            uint32_t i0   = idx % PITCH_BUF_SIZE;
            uint32_t i1   = (idx + 1) % PITCH_BUF_SIZE;

            val += (buf[i0] * (1.0f - frac) + buf[i1] * frac) * gain;
        }

        // Advance write pointer
        write_pos = (write_pos + 1) % PITCH_BUF_SIZE;

        return val;
    }
};


// ==================== Hardware & Global State ====================

DaisyPetal hw;
Parameter pitch_param, primary_param, tracking_param, tone_param, secondary_param, magic_param;

bool bypass;
bool magic_active;
bool magic_momentary;

bool pswitch1[2], pswitch2[2], pswitch3[2], pdip[4];
int  switch1[2], switch2[2], switch3[2], dip[4];

Led led1, led2;

// DSP objects
VarPitchShifter primary_shifter;
VarPitchShifter secondary_shifter;
Tone tone_filter_l;
Tone tone_filter_r;

// State
float feedback_sample = 0.0f;
float pitch_range     = 1.0f;       // +/- semitones (default +/-1)
float tracking_min    = 1200.0f;    // min del_size in samples
float tracking_max    = 9600.0f;    // max del_size in samples
int   secondary_mode  = 1;          // 0=below, 1=auto, 2=above


// ==================== Switch Handlers ====================

void updateSwitch1() // Pitch range: left=narrow, center=normal, right=wide
{
    if (pswitch1[0] == true) {          // left - narrow +/-0.5 semitones
        pitch_range = 0.5f;
    } else if (pswitch1[1] == true) {   // right - wide +/-5 semitones
        pitch_range = 5.0f;
    } else {                            // center - normal +/-1 semitone
        pitch_range = 1.0f;
    }
}

void updateSwitch2() // Tracking range: left=long, center=medium, right=short
{
    if (pswitch2[0] == true) {          // left - long (ambient)
        tracking_min = 2400.0f;
        tracking_max = 16000.0f;
    } else if (pswitch2[1] == true) {   // right - short (chorus-like)
        tracking_min = 480.0f;
        tracking_max = 2400.0f;
    } else {                            // center - medium (default)
        tracking_min = 1200.0f;
        tracking_max = 9600.0f;
    }
}

void updateSwitch3() // Secondary mode: left=below, center=auto, right=above
{
    if (pswitch3[0] == true) {          // left - always octave below
        secondary_mode = 0;
    } else if (pswitch3[1] == true) {   // right - always octave above
        secondary_mode = 2;
    } else {                            // center - auto (follows pitch direction)
        secondary_mode = 1;
    }
}


// ==================== Button Handling ====================

void UpdateButtons()
{
    // Footswitch 1: Bypass (standard latching toggle)
    if (hw.switches[Funbox::FOOTSWITCH_1].FallingEdge())
    {
        bypass = !bypass;
        led1.Set(bypass ? 0.0f : 1.0f);
    }

    // Footswitch 2: Magic (flexiswitch - momentary hold OR toggle press)
    // Hold >=300ms: momentary (active while held, off when released)
    // Short press <300ms: latching toggle
    if (hw.switches[Funbox::FOOTSWITCH_2].Pressed())
    {
        if (hw.switches[Funbox::FOOTSWITCH_2].TimeHeldMs() >= 300 && !magic_momentary)
        {
            magic_active    = true;
            magic_momentary = true;
        }
    }
    if (hw.switches[Funbox::FOOTSWITCH_2].FallingEdge())
    {
        if (magic_momentary)
        {
            // Was held: momentary release
            magic_active    = false;
            magic_momentary = false;
        }
        else
        {
            // Short press: toggle
            magic_active = !magic_active;
        }
    }
    led2.Set(magic_active ? 1.0f : 0.0f);

    led1.Update();
    led2.Update();
}


// ==================== Switch Handling ====================

void UpdateSwitches()
{
    // 3-way Switch 1
    bool changed1 = false;
    for (int i = 0; i < 2; i++) {
        if (hw.switches[switch1[i]].Pressed() != pswitch1[i]) {
            pswitch1[i] = hw.switches[switch1[i]].Pressed();
            changed1 = true;
        }
    }
    if (changed1)
        updateSwitch1();

    // 3-way Switch 2
    bool changed2 = false;
    for (int i = 0; i < 2; i++) {
        if (hw.switches[switch2[i]].Pressed() != pswitch2[i]) {
            pswitch2[i] = hw.switches[switch2[i]].Pressed();
            changed2 = true;
        }
    }
    if (changed2)
        updateSwitch2();

    // 3-way Switch 3
    bool changed3 = false;
    for (int i = 0; i < 2; i++) {
        if (hw.switches[switch3[i]].Pressed() != pswitch3[i]) {
            pswitch3[i] = hw.switches[switch3[i]].Pressed();
            changed3 = true;
        }
    }
    if (changed3)
        updateSwitch3();

    // Dip switches
    for (int i = 0; i < 4; i++) {
        if (hw.switches[dip[i]].Pressed() != pdip[i]) {
            pdip[i] = hw.switches[dip[i]].Pressed();
        }
    }
}


// ==================== Audio Callback ====================

static void AudioCallback(AudioHandle::InputBuffer  in,
                          AudioHandle::OutputBuffer out,
                          size_t                    size)
{
    hw.ProcessAnalogControls();
    hw.ProcessDigitalControls();

    UpdateButtons();
    UpdateSwitches();

    // Read knobs
    float vPitch     = pitch_param.Process();      // -1 to +1 (normalized, maps to semitones via pitch_range)
    float vPrimary   = primary_param.Process();    // 0 to 1
    float vTracking  = tracking_param.Process();   // 0 to 1 (CW = tight/short, CCW = ambient/long)
    float vTone      = tone_param.Process();       // 200 to 16000 Hz
    float vSecondary = secondary_param.Process();  // 0 to 1
    float vMagic     = magic_param.Process();      // 0 to 1

    // Pitch: apply dead zone at center (~5% of normalized range = ~5% of knob travel)
    float pitch_semitones;
    if (fabsf(vPitch) < 0.05f)
    {
        pitch_semitones = 0.0f; // Dead zone: unison / chorus mode
    }
    else
    {
        // Remove dead zone from range and rescale
        float sign = (vPitch > 0.0f) ? 1.0f : -1.0f;
        pitch_semitones = sign * (fabsf(vPitch) - 0.05f) / 0.95f * pitch_range;
    }

    // Tracking: CW (high) = short delay (tight tracking), CCW (low) = long delay (ambient)
    uint32_t del_size = (uint32_t)(tracking_max - vTracking * (tracking_max - tracking_min));

    // Configure primary pitch shifter
    primary_shifter.SetDelSize(del_size);
    primary_shifter.SetTranspose(pitch_semitones);

    // Secondary voice: determine octave shift direction
    float secondary_semitones;
    if (fabsf(pitch_semitones) < 0.001f)
    {
        // At unison: secondary is a slight detune for chorus effect
        secondary_semitones = 0.08f;
    }
    else if (secondary_mode == 0)
    {
        // Always octave below
        secondary_semitones = pitch_semitones - 12.0f;
    }
    else if (secondary_mode == 2)
    {
        // Always octave above
        secondary_semitones = pitch_semitones + 12.0f;
    }
    else
    {
        // Auto: above when pitch > 0, below when pitch < 0
        if (pitch_semitones > 0.0f)
            secondary_semitones = pitch_semitones + 12.0f;
        else
            secondary_semitones = pitch_semitones - 12.0f;
    }

    secondary_shifter.SetDelSize(del_size);
    secondary_shifter.SetTranspose(secondary_semitones);

    // Fun: slight imperfection at all times + increases with magic
    float fun_amount = 0.005f + (magic_active ? vMagic * 0.02f : 0.0f);
    primary_shifter.SetFun(fun_amount);
    secondary_shifter.SetFun(fun_amount);

    // Tone filter
    tone_filter_l.SetFreq(vTone);
    tone_filter_r.SetFreq(vTone);

    // Feedback amount (only when Magic is active)
    float feedback_amount = magic_active ? vMagic * 0.95f : 0.0f;

    for (size_t i = 0; i < size; i++)
    {
        if (bypass)
        {
            out[0][i] = in[0][i];
            out[1][i] = in[1][i];
        }
        else
        {
            float dry = in[0][i];

            // Mix dry input with saturated feedback (Magic regeneration)
            float shifter_input = dry + tanhf(feedback_sample * feedback_amount);

            // Primary pitch-shifted voice
            float primary_wet = primary_shifter.Process(shifter_input) * vPrimary;

            // Secondary pitch-shifted voice (octave)
            float secondary_wet = secondary_shifter.Process(shifter_input) * vSecondary;

            if (pdip[0] == false)
            {
                // Mono mode: combined wet through single filter path
                float wet_mix  = primary_wet + secondary_wet;
                float filtered = tone_filter_l.Process(wet_mix);

                // Store for feedback
                feedback_sample = filtered;

                // Output: dry + filtered wet, soft-clipped
                float out_sample = tanhf(dry + filtered);
                out[0][i] = out_sample;
                out[1][i] = out_sample;
            }
            else
            {
                // MISO mode: primary biased left, secondary biased right for stereo spread
                float filt_l = tone_filter_l.Process(primary_wet * 0.8f + secondary_wet * 0.2f);
                float filt_r = tone_filter_r.Process(primary_wet * 0.2f + secondary_wet * 0.8f);

                // Combined feedback from both channels
                feedback_sample = (filt_l + filt_r) * 0.5f;

                out[0][i] = tanhf(dry + filt_l);
                out[1][i] = tanhf(dry + filt_r);
            }
        }
    }
}


// ==================== Main ====================

int main(void)
{
    float samplerate;

    hw.Init();
    samplerate = hw.AudioSampleRate();

    hw.SetAudioBlockSize(2); // Block size 2 to eliminate digital whine on Funbox PCB

    // Map physical switches
    switch1[0] = Funbox::SWITCH_1_LEFT;
    switch1[1] = Funbox::SWITCH_1_RIGHT;
    switch2[0] = Funbox::SWITCH_2_LEFT;
    switch2[1] = Funbox::SWITCH_2_RIGHT;
    switch3[0] = Funbox::SWITCH_3_LEFT;
    switch3[1] = Funbox::SWITCH_3_RIGHT;
    dip[0] = Funbox::SWITCH_DIP_1;
    dip[1] = Funbox::SWITCH_DIP_2;
    dip[2] = Funbox::SWITCH_DIP_3;
    dip[3] = Funbox::SWITCH_DIP_4;

    // Initialize switch state
    for (int i = 0; i < 2; i++) {
        pswitch1[i] = false;
        pswitch2[i] = false;
        pswitch3[i] = false;
    }
    for (int i = 0; i < 4; i++) {
        pdip[i] = false;
    }

    // Knob parameter ranges
    // Knob 1: Pitch — normalized -1 to +1, scaled by pitch_range to get semitones
    pitch_param.Init(hw.knob[Funbox::KNOB_1], -1.0f, 1.0f, Parameter::LINEAR);
    // Knob 2: Primary — wet mix level of primary harmony voice
    primary_param.Init(hw.knob[Funbox::KNOB_2], 0.0f, 1.0f, Parameter::LINEAR);
    // Knob 3: Tracking — CW=tight/short delay, CCW=ambient/long delay
    tracking_param.Init(hw.knob[Funbox::KNOB_3], 0.0f, 1.0f, Parameter::LINEAR);
    // Knob 4: Tone — lowpass cutoff frequency (CW=bright, CCW=dark)
    tone_param.Init(hw.knob[Funbox::KNOB_4], 200.0f, 16000.0f, Parameter::EXPONENTIAL);
    // Knob 5: Secondary — wet mix level of octave voice
    secondary_param.Init(hw.knob[Funbox::KNOB_5], 0.0f, 1.0f, Parameter::LINEAR);
    // Knob 6: Magic — feedback/regeneration amount (when Magic is active)
    magic_param.Init(hw.knob[Funbox::KNOB_6], 0.0f, 1.0f, Parameter::LINEAR);

    // Initialize pitch shifters with SDRAM buffers
    primary_shifter.Init(pitch_buf_primary, samplerate);
    secondary_shifter.Init(pitch_buf_secondary, samplerate);

    // Initialize tone filters
    tone_filter_l.Init(samplerate);
    tone_filter_l.SetFreq(8000.0f);
    tone_filter_r.Init(samplerate);
    tone_filter_r.SetFreq(8000.0f);

    // Initialize state
    feedback_sample = 0.0f;
    bypass          = true;
    magic_active    = false;
    magic_momentary = false;

    // Init the LEDs and set bypass
    led1.Init(hw.seed.GetPin(Funbox::LED_1), false);
    led1.Update();

    led2.Init(hw.seed.GetPin(Funbox::LED_2), false);
    led2.Update();

    hw.StartAdc();
    hw.StartAudio(AudioCallback);
    while (1)
    {
        System::Delay(10);
    }
}

