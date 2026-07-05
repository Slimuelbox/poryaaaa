#include "m4a_channel.h"
#include "m4a_tables.h"
#include <string.h>

/* Number of samples over which the wave channel (type 3) fades to zero on
 * note-off, preventing a DC-offset pop. ~5.8 ms at 44100 Hz. */
#define DECLICK_SAMPLES 256

/*
 * PCM Channel Implementation
 * Matches the SoundMainRAM mixer in m4a_1.s
 */

/*
 * Advance the pulse synth's duty-cycle LFO one frame and recompute the phase
 * threshold the oscillator is compared against.  Matches the pulse path of
 * C_setup_synth in the improved mixer, which runs once per mixing frame:
 * count (the GBA reuses the sample countdown field) accumulates the LFO step,
 * the offset value is folded into a triangle wave with a bitwise NOT when
 * negative, and the result scaled by the mod amount plus the base duty gives
 * the threshold.  All arithmetic wraps at 32 bits like the ARM code.
 */
static void m4a_pcm_synth_pulse_update(M4APCMChannel *ch)
{
    const uint8_t *cfg = (const uint8_t *)ch->wav->data;
    uint32_t lfo = (uint32_t)ch->count + ((uint32_t)cfg[3] << 24);
    ch->count = (int32_t)lfo;
    uint32_t folded = lfo + ((uint32_t)cfg[5] << 24);
    if ((int32_t)folded < 0)
        folded = ~folded;
    ch->synthPulseDuty = ((uint32_t)cfg[2] << 24) + (folded >> 8) * cfg[4];
}

void m4a_pcm_channel_start(M4APCMChannel *ch, WaveData *wav, uint8_t type)
{
    ch->wav = wav;
    ch->type = type;
    ch->envelopeVolume = 0;

    if (wav->size == 0) {
        /* Synth voice: no sample data, phase accumulator drives the oscillator.
         * Triangle (type 2) starts at quarter-cycle (0x40000000) to avoid pop. */
        uint8_t synthType = ((uint8_t *)wav->data)[1];
        ch->currentPointer = NULL;
        ch->count = 0;  /* IIR state for saw wave */
        ch->isLoop = false;
        ch->loopLen = 0;
        ch->loopStart = NULL;
        ch->fw = (synthType == 2) ? 0x40000000 : 0;
    } else {
        ch->currentPointer = wav->data;
        ch->count = wav->size;
        ch->fw = 0;

    /* Check for loop - GBA checks wav->status bits 14-15 (0xC000) */
    ch->isLoop = (wav->status & 0xC000) != 0;
    if (ch->isLoop) {
        ch->loopStart = wav->data + wav->loopStart;
        ch->loopLen = wav->size - wav->loopStart;
        if (ch->loopLen <= 0) {
            ch->isLoop = false;
            ch->loopLen = 0;
        }
    }

    ch->status = CHN_ENV_ATTACK;
    if (ch->isLoop)
        ch->status |= CHN_LOOP;

    uint8_t envVol = ch->attack;
    if (envVol >= 0xFF) {
        envVol = 0xFF;
        ch->status = CHN_ENV_DECAY | (ch->status & CHN_LOOP);
    }
    ch->envelopeVolume = envVol;
}

void m4a_pcm_channel_stop(M4APCMChannel *ch)
{
    ch->status = 0;
}

/*
 * PCM envelope tick - called at ~60Hz
 * Matches the envelope processing in SoundMainRAM (m4a_1.s)
 */
void m4a_pcm_channel_tick(M4APCMChannel *ch, uint8_t masterVolume)
{
    if (!(ch->status & CHN_ON))
        return;

    uint8_t envVol = ch->envelopeVolume;

    if (ch->status & CHN_START) {
        /* Channel just started - handled in render path start */
        if (ch->status & CHN_STOP) {
            /* Immediate stop */
            ch->status = 0;
            return;
        }
        ch->status = CHN_ENV_ATTACK;
        if (ch->isLoop)
            ch->status |= CHN_LOOP;
        envVol = 0;
        ch->fw = 0;
        /* Fall through to attack */
    }

    if (ch->status & CHN_IEC) {
        /* Pseudo-echo countdown.  Signed check matches the GBA's subs/bhi:
         * a starting length of 0 underflows and must stop the channel
         * immediately rather than wrapping to 255. */
        ch->pseudoEchoLength--;
        if ((int8_t)ch->pseudoEchoLength <= 0) {
            ch->status = 0;
            return;
        }
    } else if (ch->status & CHN_STOP) {
        /* Release phase */
        envVol = (envVol * ch->release) >> 8;
        if (envVol <= ch->pseudoEchoVolume) {
            if (ch->pseudoEchoVolume == 0) {
                ch->status = 0;
                return;
            }
            envVol = ch->pseudoEchoVolume;
            ch->status |= CHN_IEC;
        }
    } else {
        uint8_t envState = ch->status & CHN_ENV_MASK;
        if (envState == CHN_ENV_DECAY) {
            envVol = (envVol * ch->decay) >> 8;
            if (envVol <= ch->sustain) {
                envVol = ch->sustain;
                if (envVol == 0) {
                    /* Sustain is 0, go to pseudo-echo */
                    if (ch->pseudoEchoVolume == 0) {
                        ch->status = 0;
                        return;
                    }
                    envVol = ch->pseudoEchoVolume;
                    ch->status = (ch->status & ~CHN_ENV_MASK) | CHN_IEC;
                } else {
                    ch->status--;  /* decay -> sustain */
                }
            }
        } else if (envState == CHN_ENV_ATTACK) {
            /* Use 32-bit arithmetic to match GBA behavior (ldrb zero-extends,
             * so the GBA does this addition in 32-bit registers, not 8-bit). */
            uint32_t sum = (uint32_t)envVol + ch->attack;
            if (sum >= 0xFF) {
                envVol = 0xFF;
                ch->status--;  /* attack -> decay */
            } else {
                envVol = (uint8_t)sum;
            }
        }
        /* sustain: envVol stays as-is */
    }

    ch->envelopeVolume = envVol;

    /* Calculate final per-channel volumes
     * Matches: masterVolume+1 * envVol >> 4, then * rightVolume >> 8 */
    uint32_t vol = ((uint32_t)(masterVolume + 1) * envVol) >> 4;
    ch->envelopeVolumeRight = ((uint32_t)ch->rightVolume * vol) >> 8;
    ch->envelopeVolumeLeft = ((uint32_t)ch->leftVolume * vol) >> 8;

    /* Golden Sun pulse synth: the duty-cycle LFO advances once per frame,
     * mirroring C_setup_synth running once per SoundMainRAM call. */
    if (ch->synthType == M4A_SYNTH_PULSE && ch->wav)
        m4a_pcm_synth_pulse_update(ch);
}

/*
 * PCM channel render - generates one output sample
 * Matches the interpolating mixer in SoundMainRAM (m4a_1.s)
 *
 * The GBA mixer uses a 23-bit fractional sample position (fw field).
 * For non-fixed-frequency voices, it does linear interpolation between
 * adjacent samples. For fixed-frequency voices (type & 0x08), it just
 * reads one sample per output sample (no interpolation).
 */
void m4a_pcm_channel_render(M4APCMChannel *ch, int32_t *mixL, int32_t *mixR)
{
    if (!(ch->status & CHN_ON) || (ch->status & CHN_START))
        return;

    /* Synth voice: wav->size == 0 means software oscillator, not sample data.
     * Params: data[1]=type, data[2]=baseDuty, data[3]=widthChange1,
     *         data[4]=modAmount, data[5]=widthChange2.
     * Phase accumulator uses the full 32-bit fw; advances by frequency<<3 per
     * output sample (matches C_setup_synth's r4 = freq*VAR_DEF_PITCH_FAC,
     * per-sample advance r4<<3). */
    if (ch->wav->size == 0) {
        const uint8_t *p = (const uint8_t *)ch->wav->data;
        uint8_t synthType = p[1];
        uint32_t phase    = ch->fw;
        uint32_t step     = ch->frequency << 3;
        int32_t  sample;

        switch (synthType) {
        case 0: {  /* Pulse wave (C_synth_pulse_loop) */
            uint32_t r2   = (uint32_t)p[3] << 24;           /* widthChange1 << 24 */
            uint32_t r6   = r2 + ((uint32_t)p[5] << 24);   /* + widthChange2 << 24 */
            if ((int32_t)r6 < 0) r6 = ~r6;                 /* mvnmi */
            uint32_t duty = (r6 >> 8) * (uint32_t)p[4] + ((uint32_t)p[2] << 24);
            sample = (phase < duty) ? 64 : -64;
            phase += step;
            break;
        }
        case 1: {  /* Saw wave (C_synth_saw_loop) — leaky integrator approximation.
                    * ch->count holds the IIR state (r2 in assembly). Volume is
                    * halved in the GBA (r11 lsr#1), reproduced here via >>9. */
            phase += step;
            int32_t r9    = (int32_t)(phase >> 24) - 0x70 - (int32_t)(phase >> 26);
            int32_t state = (int32_t)ch->count;
            state = r9 + (state >> 1);
            ch->count = state;
            sample = state;
            *mixR += (sample * ch->envelopeVolumeRight) >> 9;
            *mixL += (sample * ch->envelopeVolumeLeft)  >> 9;
            ch->fw = phase;
            return;
        }
        case 2: {  /* Triangle wave (C_synth_triangle_loop) */
            phase += step;
            if ((phase & 0x80000000) == 0)
                sample = (int32_t)(phase >> 23) - 128;      /* ascending: -128..127 */
            else
                sample = 384 - (int32_t)(phase >> 23);      /* descending: 128..-127 */
            break;
        }
        default:
            sample = 0;
            phase += step;
            break;
        }

        ch->fw = phase;
        *mixR += (value * ch->envelopeVolumeRight) >> 8;
        *mixL += (value * ch->envelopeVolumeLeft) >> 8;
        return;
    }

    int8_t *ptr = ch->currentPointer;
    uint32_t fw = ch->fw;
    int32_t count = ch->count;
    int32_t sample;

    bool isReverse = (ch->type == VOICE_CRY_REVERSE);

    if (ch->type & VOICE_TYPE_FIX) {
        sample = ptr[0];
    } else if (isReverse) {
        int8_t s0 = ptr[0];
        int8_t s1 = (ptr > ch->wav->data) ? ptr[-1] : s0;
        int32_t diff = s1 - s0;
        sample = s0 + (int32_t)(((int64_t)diff * (int32_t)fw) >> 23);
    } else {
        int8_t s0 = ptr[0];
        int8_t s1 = ptr[1];
        int32_t diff = s1 - s0;
        sample = s0 + (int32_t)(((int64_t)diff * (int32_t)fw) >> 23);
    }

    int32_t ampR = sample * ch->envelopeVolumeRight;
    int32_t ampL = sample * ch->envelopeVolumeLeft;
    *mixR += ampR >> 8;
    *mixL += ampL >> 8;

    /* Advance position */
    fw += ch->frequency;
    uint32_t advance = fw >> 23;
    if (advance) {
        fw &= 0x7FFFFF;  /* keep fractional part */
        count -= advance;
        if (count <= 0) {
            if (!isReverse && ch->isLoop && ch->loopLen > 0) {
                /* Wrap around loop */
                while (count <= 0)
                    count += ch->loopLen;
                ptr = ch->loopStart + (ch->loopLen - count);
            } else {
                ch->status = 0;
            }
        } else if (isReverse) {
            ptr -= advance;
        } else {
            ptr += advance;
        }
    }

    ch->currentPointer = ptr;
    ch->fw = fw;
    ch->count = count;
}

/*
 * CGB Channel Implementation
 * Matches CgbSound() in m4a.c
 */

void m4a_cgb_channel_start(M4ACGBChannel *ch)
{
    ch->status = CHN_ENV_ATTACK;
    ch->modify = 0x03; /* pitch + vol */
    ch->phase = 0;

    /* Invalidate the cached phase increment / wave DC sum so the first
     * rendered sample of this note recomputes them from the current
     * frequency and wave table. */
    ch->phaseIncFreq = 0xFFFFFFFFu;
    ch->waveSumPointer = NULL;
    ch->envelopeCounter = ch->attack;
    if (ch->attack == 0) {
        /* Skip attack if instantaneous */
        ch->envelopeVolume = ch->envelopeGoal;
        ch->status = CHN_ENV_DECAY;
        ch->envelopeCounter = ch->decay;
        if (ch->decay == 0) {
            /* Skip decay too */
            if (ch->sustain == 0) {
                ch->status = CHN_ENV_RELEASE;
            } else {
                ch->envelopeVolume = ch->sustainGoal;
                ch->status = CHN_ENV_SUSTAIN;
            }
        }
    } else {
        ch->envelopeVolume = 0;
    }

    /* Cancel any in-progress declick so the new note starts cleanly. */
    ch->declickSample = 0;
    ch->declickSamplesRemaining = 0;

    /* Initialize LFSR for noise channel.
     * Bit 3 of frequency is the period/mode bit (NR43 bit 3):
     * 0 = 15-bit LFSR, 1 = 7-bit short-period LFSR. */
    if (ch->type == 4) {
        if (ch->frequency & 0x08)
            ch->lfsr = 0x7F;    /* 7-bit */
        else
            ch->lfsr = 0x7FFF;  /* 15-bit */
    }
}

void m4a_cgb_channel_stop(M4ACGBChannel *ch)
{
    if (ch->type == 3)
        ch->declickSamplesRemaining = DECLICK_SAMPLES;
    ch->status = 0;
}

/*
 * CGB pan calculation - matches CgbPan() in m4a.c.
 * Determines whether the channel is hard-panned to one side.
 * Sets ch->pan to 0x0F (hard right) or 0xF0 (hard left) and returns 1,
 * or leaves ch->pan unchanged and returns 0 (center/both sides).
 */
static int cgb_pan(M4ACGBChannel *ch)
{
    uint32_t rightVolume = (uint8_t)ch->rightVolume;
    uint32_t leftVolume  = (uint8_t)ch->leftVolume;

    if (rightVolume >= leftVolume) {
        if (rightVolume / 2 >= leftVolume) {
            ch->pan = 0x0F;
            return 1;
        }
    } else {
        if (leftVolume / 2 >= rightVolume) {
            ch->pan = 0xF0;
            return 1;
        }
    }
    return 0;
}

/*
 * CGB mod vol calculation - matches CgbModVol in m4a.c.
 * Converts the software left/right volumes (from velocity + CC7 + pan) into
 * the 4-bit hardware envelope goal and the NR51 pan routing bits.
 */
void m4a_cgb_mod_vol(M4ACGBChannel *ch)
{
    if (!cgb_pan(ch)) {
        /* Center (or near-center) pan: output to both sides */
        ch->pan = 0xFF;
        ch->envelopeGoal = (uint32_t)(ch->leftVolume + ch->rightVolume) / 16;
    } else {
        /* Hard-panned: pan already set by cgb_pan(); clamp goal to hardware max */
        ch->envelopeGoal = (uint32_t)(ch->leftVolume + ch->rightVolume) / 16;
        if (ch->envelopeGoal > 15)
            ch->envelopeGoal = 15;
    }
    ch->sustainGoal = (ch->envelopeGoal * ch->sustain + 15) >> 4;
    ch->pan &= ch->panMask;
}

/*
 * CGB envelope tick - matches CgbSound() envelope logic in m4a.c
 * Called at ~60Hz, with double-step every 15 frames (when c15==0)
 */
void m4a_cgb_channel_tick(M4ACGBChannel *ch, uint8_t c15)
{
    if (!(ch->status & CHN_ON))
        return;

    /* Declared before the goto targets below: the start and release-start
     * paths jump into the envelope block, and must see initialized values so
     * they still honor the every-15th-frame double step (CgbSound initializes
     * prevC15 before any of the equivalent branches). */
    int doubleStep = (c15 == 0) ? 1 : 0;
    int steps = 0;

    if (ch->status & CHN_START) {
        if (ch->status & CHN_STOP) {
            if (ch->type == 3)
                ch->declickSamplesRemaining = DECLICK_SAMPLES;
            ch->status = 0;
            return;
        }
        ch->status = CHN_ENV_ATTACK;
        ch->modify = 0x03;
        m4a_cgb_mod_vol(ch);
        ch->envelopeCounter = ch->attack;
        if (ch->attack != 0) {
            ch->envelopeVolume = 0;
        } else {
            /* skip attack */
            ch->envelopeVolume = ch->envelopeGoal;
            ch->status = CHN_ENV_DECAY;
            ch->envelopeCounter = ch->decay;
            if (ch->decay == 0) {
                if (ch->sustain == 0) {
                    goto pseudo_echo;
                }
                ch->status = CHN_ENV_SUSTAIN;
                ch->envelopeVolume = ch->sustainGoal;
            }
        }
        goto step_complete;
    }

    if (ch->status & CHN_IEC) {
        ch->pseudoEchoLength--;
        if ((int8_t)ch->pseudoEchoLength <= 0) {
            if (ch->type == 3)
                ch->declickSamplesRemaining = DECLICK_SAMPLES;
            ch->status = 0;
            return;
        }
        goto envelope_complete;
    }

    if ((ch->status & CHN_STOP) && (ch->status & CHN_ENV_MASK)) {
        ch->status &= ~CHN_ENV_MASK;
        ch->envelopeCounter = ch->release;
        if (ch->release != 0) {
            ch->modify |= 0x01;
            goto step_complete;
        } else {
            goto pseudo_echo;
        }
    }

    {
step_repeat:
        if (ch->envelopeCounter == 0) {
            m4a_cgb_mod_vol(ch);
            uint8_t envState = ch->status & CHN_ENV_MASK;

            if (envState == CHN_ENV_RELEASE) {
                ch->envelopeVolume--;
                if ((int8_t)ch->envelopeVolume <= 0) {
                pseudo_echo:
                    ch->envelopeVolume = ((ch->envelopeGoal * ch->pseudoEchoVolume) + 0xFF) >> 8;
                    if (ch->envelopeVolume) {
                        ch->status |= CHN_IEC;
                        ch->modify |= 0x01;
                        goto envelope_complete;
                    } else {
                        if (ch->type == 3)
                            ch->declickSamplesRemaining = DECLICK_SAMPLES;
                        ch->status = 0;
                        return;
                    }
                }
                ch->envelopeCounter = ch->release;
            } else if (envState == CHN_ENV_SUSTAIN) {
                ch->envelopeVolume = ch->sustainGoal;
                ch->envelopeCounter = 7;
            } else if (envState == CHN_ENV_DECAY) {
                ch->envelopeVolume--;
                if ((int8_t)ch->envelopeVolume <= (int8_t)ch->sustainGoal) {
                    if (ch->sustain == 0) {
                        ch->status &= ~CHN_ENV_MASK;
                        goto pseudo_echo;
                    }
                    ch->status--;  /* decay -> sustain */
                    ch->modify |= 0x01;
                    ch->envelopeVolume = ch->sustainGoal;
                    ch->envelopeCounter = 7;
                    goto step_complete;
                }
                ch->envelopeCounter = ch->decay;
            } else {
                /* Attack */
                ch->envelopeVolume++;
                if (ch->envelopeVolume >= ch->envelopeGoal) {
                    ch->status--;  /* attack -> decay */
                    ch->envelopeCounter = ch->decay;
                    if (ch->decay != 0) {
                        ch->modify |= 0x01;
                        ch->envelopeVolume = ch->envelopeGoal;
                    } else {
                        if (ch->sustain == 0) {
                            ch->status &= ~CHN_ENV_MASK;
                            goto pseudo_echo;
                        }
                        ch->status--;
                        ch->envelopeVolume = ch->sustainGoal;
                        ch->envelopeCounter = 7;
                    }
                    goto step_complete;
                }
                ch->envelopeCounter = ch->attack;
            }
        }

step_complete:
        ch->envelopeCounter--;
        /* Double step every 15 frames (when c15==0) to match hardware 1/64s rate */
        if (doubleStep && steps == 0) {
            steps = 1;
            goto step_repeat;
        }
    }

envelope_complete:
    ch->modify = 0;
}

/*
 * CGB channel render - generates one output sample by software synthesis
 */
void m4a_cgb_channel_render(M4ACGBChannel *ch, int32_t *mixL, int32_t *mixR,
                            float sampleRate)
{
    if (!(ch->status & CHN_ON)) {
        /* Wave channel declick: linearly fade the last sample to zero over
         * DECLICK_SAMPLES frames to prevent a pop caused by the DC offset. */
        if (ch->type == 3 && ch->declickSamplesRemaining > 0) {
            ch->declickSamplesRemaining--;
            int32_t faded = (ch->declickSample * (int32_t)ch->declickSamplesRemaining) / DECLICK_SAMPLES;
            if (ch->pan & 0x0F) *mixR += faded;
            if (ch->pan & 0xF0) *mixL += faded;
        }
        return;
    }
    if (ch->status & CHN_START)
        return;

    int32_t sample = 0;
    uint8_t cgbType = ch->type;

    if (cgbType == 1 || cgbType == 2) {
        /* Square wave synthesis */
        static const uint8_t dutyPatterns[4] = { 0x01, 0x81, 0xE1, 0x7E };
        uint8_t pattern = dutyPatterns[ch->dutyCycle & 3];
        int bit = (ch->phase >> 29) & 7;
        sample = (pattern & (1 << bit)) ? 64 : -64;

        /* Advance phase.
         * CGB frequency register value is used to compute the actual frequency.
         * The CGB freq register value = 2048 - (131072 / freq_hz).
         * So freq_hz = 131072 / (2048 - reg_value).
         * We convert to a phase increment for our 32-bit accumulator.  The
         * increment is constant between (tick-rate) frequency changes, so it is
         * cached and only recomputed when ch->frequency changes. */
        if (ch->frequency != ch->phaseIncFreq) {
            int32_t freqReg = ch->frequency;
            if (freqReg >= 2048) freqReg = 2047;
            float freqHz = 131072.0f / (float)(2048 - freqReg);
            ch->phaseInc = (uint32_t)(freqHz / sampleRate * 4294967296.0f);
            ch->phaseIncFreq = ch->frequency;
        }
        ch->phase += ch->phaseInc;
    } else if (cgbType == 3) {
        /* Programmable wave channel */
        if (ch->wavePointer) {
            /* 32 4-bit samples packed into 16 bytes (4 uint32_t) */
            uint8_t *waveData = (uint8_t *)ch->wavePointer;
            int pos = (ch->phase >> 27) & 0x1F;  /* 5 bits = 0-31 */
            uint8_t nibble;
            if (pos & 1)
                nibble = waveData[pos >> 1] & 0x0F;
            else
                nibble = (waveData[pos >> 1] >> 4) & 0x0F;
            /* Sum of all 32 raw nibbles for DC offset removal.  The wave mean
             * varies per waveform, so using a fixed midpoint of 8 leaves a DC
             * offset that causes clicks at note start/end.  The sum depends
             * only on the wave table contents, so it is cached and recomputed
             * only when wavePointer changes. */
            if (ch->wavePointer != ch->waveSumPointer) {
                int32_t waveSum = 0;
                for (int i = 0; i < 16; i++) {
                    waveSum += (waveData[i] >> 4) & 0x0F;
                    waveSum += waveData[i] & 0x0F;
                }
                ch->waveSum = waveSum;
                ch->waveSumPointer = ch->wavePointer;
            }
            int32_t waveSum = ch->waveSum;

            /* Volume control: apply shift to raw 4-bit nibble to match
             * GBA hardware quantization (NR32 register).
             * On real hardware, the right-shift is lossy on the small
             * 4-bit value, creating quantized "plateaus" in the output.
             * Apply the same volume logic to the mean for accurate DC removal. */
            int32_t shifted = (int32_t)nibble;
            /* envelopeVolume can exceed 15 (CgbModVol's center-pan goal is
             * unclamped, up to 31); hardware truncates on the register write,
             * so only the low 4 bits are audible. */
            int nr32 = gCgb3Vol[ch->envelopeVolume & 0x0F];
            int32_t meanShifted;
            if (nr32 == 0) {
                shifted = 0;
                meanShifted = 0;
            } else if (nr32 & 0x80) {
                /* GBA 75% mode: (nibble * 3) >> 2; mean = (waveSum * 3) >> 7 */
                shifted = (shifted + (shifted << 1)) >> 2;
                meanShifted = (waveSum * 3) >> 7;
            } else {
                int shift = ((nr32 >> 5) & 3) - 1;
                shifted >>= shift;
                meanShifted = waveSum >> (5 + shift);
            }
            /* Center around the waveform's actual mean to remove DC offset */
            sample = (shifted - meanShifted) * 8;

            /* Advance phase (cached; recomputed only on frequency change). */
            if (ch->frequency != ch->phaseIncFreq) {
                int32_t freqReg = ch->frequency;
                if (freqReg >= 2048) freqReg = 2047;
                float freqHz = 2097152.0f / (float)(2048 - freqReg);
                /* Wave channel plays 32 samples per period */
                freqHz /= 32.0f;
                ch->phaseInc = (uint32_t)(freqHz / sampleRate * 4294967296.0f);
                ch->phaseIncFreq = ch->frequency;
            }
            ch->phase += ch->phaseInc;
        }
    } else if (cgbType == 4) {
        /* Noise channel using LFSR */
        sample = (ch->lfsr & 1) ? 64 : -64;

        /* Advance LFSR at noise frequency rate */
        uint8_t noiseParams = ch->frequency & 0xFF;

        /* Phase increment is constant between frequency changes; cache it. */
        if (ch->frequency != ch->phaseIncFreq) {
            uint8_t divRatio = noiseParams & 0x07;
            uint8_t shiftFreq = (noiseParams >> 4) & 0x0F;
            /* bool shortMode = (noiseParams >> 3) & 1; */

            float baseFreq = 524288.0f;
            float divisor = (divRatio == 0) ? 0.5f : (float)divRatio;
            float noiseFreq = baseFreq / divisor / (float)(1 << (shiftFreq + 1));

            ch->phaseInc = (uint32_t)(noiseFreq / sampleRate * 4294967296.0f);
            ch->phaseIncFreq = ch->frequency;
        }
        uint32_t oldPhase = ch->phase;
        ch->phase += ch->phaseInc;

        /* Clock LFSR on phase wrap.
         * Bit 3 of frequency = period mode: 0 = 15-bit LFSR, 1 = 7-bit LFSR. */
        if (ch->phase < oldPhase) {
            uint16_t bit = ((ch->lfsr >> 1) ^ ch->lfsr) & 1;
            if (noiseParams & 0x08)
                ch->lfsr = (ch->lfsr >> 1) | (bit << 6);   /* 7-bit */
            else
                ch->lfsr = (ch->lfsr >> 1) | (bit << 14);  /* 15-bit */
        }
    }

    /* Apply envelope volume (for non-wave channels).
     * envelopeVolume is the 4-bit GBA hardware volume (0-15), matching what
     * CgbSound writes to NR12/NR22/NR42. */
    if (cgbType != 3) {
        /* Mask to 4 bits: envelopeVolume can reach 31 (unclamped center-pan
         * goal), but the NRx2 write is (envelopeVolume << 4) truncated to a
         * byte, so hardware plays envelopeVolume & 0xF. */
        sample = (sample * (ch->envelopeVolume & 0x0F)) >> 4;
    }

    /* Scale CGB to match the GBA hardware mixing ratio.
     * SOUNDCNT_H is initialised with SOUND_ALL_MIX_FULL (volume bits = 2), so
     * mGBA applies psgShift = 4 - 2 = 2 (CGB >> 2) while PCM is << 2.
     * That is a 16:1 ratio; >> 2 here keeps us in the same integer domain as
     * the PCM mixer which already incorporates the << 2 implicitly through its
     * larger sample values (~±127 vs CGB's ~±60). */
    sample >>= 1;

    /* Track last sample for wave channel declick on note-off. */
    if (cgbType == 3)
        ch->declickSample = sample;

    /* Route to left/right using NR51-style pan bits in ch->pan.
     * Panning is binary on the GBA (NR51 enable bits), not a level multiplier. */
    if (ch->pan & 0x0F)
        *mixR += sample;
    if (ch->pan & 0xF0)
        *mixL += sample;
}
