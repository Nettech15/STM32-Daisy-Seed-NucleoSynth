/*
This code is a modification of my old STM32CubeIDE project called "NucleoSynth".
I (Christopher @Nettech15) have modified it to run on the Daisy Seed environment using the VSCode IDE.
The USB-MIDI and Audio In/Out routines are handled by the LibDaisy Library.
The Synth Engine is entirely handled thru HAL, however.
Synth parameters are now controlled by a Miditech i2-61 midi keyboard.
Feel free to copy, modify, and improve this code to match your equipment and sound requirements.

Forked from STM32 Daisy Seed PolySynth v1.3.6. STM32CubeIDE Project.
*/

#include "daisy_seed.h"
#include "daisysp.h"

#include "main.h"

#define WRITE_READ_SRAM_ADDR 0x20000000 // Set memory to SRAM

TIM_MasterConfigTypeDef sMasterConfig;
TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;
TIM_HandleTypeDef htim5;
TIM_HandleTypeDef htim6;
TIM_HandleTypeDef htim7;
TIM_HandleTypeDef htim8;
TIM_HandleTypeDef htim12;
TIM_HandleTypeDef htim13;
TIM_HandleTypeDef htim14;
TIM_HandleTypeDef htim15;
TIM_HandleTypeDef htim16;
TIM_HandleTypeDef htim17;

using namespace daisy;
using namespace daisysp;

// globals
DaisySeed hardware;
MidiUsbHandler midi;
float sysSampleRate;
float sysCallbackRate;

// Synth Engine
int16_t delay_in = 0, delay_out = 500;
uint16_t counter[12], pwval, pwmval, modval;
uint8_t param, msgnum, midimsg, key, velocity, ctrl, data, midichan;
uint8_t wavesel, velsel, pwm, pwm2, mod, vcf, tun, det, sus, notepos, bend = 64, patch;
uint8_t paramvalue[32];

uint16_t seqclock = 0, seqtime = 0, seqmode = 0, seqnote, seqvelocity, seqmsg;
uint32_t seqmem = 0x00000000;

uint8_t voice = 0, n, i, j, y, r, c, d, e, f, g, h, k, m;
uint16_t lfo1rate, lfo1 = 63, lfo1cnt = 1, lfo1pol = 1;
uint16_t lfo2rate, lfo2 = 63, lfo2cnt = 1, lfo2pol = 1;
uint16_t lfo3rate, lfo3 = 63, lfo3cnt = 1, lfo3pol = 1;
uint16_t lfo4rate, lfo4 = 63, lfo4cnt = 1, lfo4pol = 1;
int8_t scale;

const uint32_t pitchtbl[] = {16384,
	15464,14596,13777,13004,12274,11585,10935,10321,9742,9195,8679,
	8192,7732,7298,6889,6502,6137,5793,5468,5161,4871,4598,4340,
	4096,3866,3649,3444,3251,3069,2896,2734,2580,2435,2299,2170,
	2048,1933,1825,1722,1625,1534,1448,1367,1290,1218,1149,1085,
	1024,967,912,861,813,767,724,683,645,609,575,542,
	512,483,456,431,406,384,362,342,323,304,287,271,
	256,242,228,215,203,192,181,171,161,152,144,136,
	128,121,114,108 ,102,96,91,85,81,76,72,68,64};
uint8_t chanstat[6] = {255,255,255,255,255,255};
uint8_t vcfadsr[6] = {0,0,0,0,0,0}, vcaadsr[6] = {0,0,0,0,0,0};

volatile uint32_t loadcount;
volatile uint32_t storecount;

uint32_t envtrig = 48;
uint16_t initmodval;

float channel[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
float vcfcutoff[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
float vcacutoff[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
float vcfkeyfollow[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
float envkeyfollow[6] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
float summer = 0.0f, pbend = 0.0f;
float modlvl = 0.0f, pwmlvl = 0.0f, pwm2lvl = 0.0f, vcflvl = 0.0f, vcfval = 0.0f;
float vcfkflvl, envkflvl, oscmix, vcfreleaselvl = 0.000001f, vcareleaselvl = 0.000001f, resonance;
float vcfattackrate[6], vcfdecayrate[6], vcfsustainlvl[6], vcfreleaserate[6];
float vcfvellvl[6], vcavellvl[6], vcfenvlvl;
float vcfattack, vcfdecay, vcfsustain, vcfrelease;
float vcaattackrate[6], vcadecayrate[6], vcasustainlvl[6], vcareleaserate[6];
float vcaattack, vcadecay, vcasustain, vcarelease;
float delaybuffer[1000];

void TIM1_Config(uint16_t);
void TIM2_Config(uint16_t);
void TIM3_Config(uint16_t);
void TIM4_Config(uint16_t);
void TIM5_Config(uint16_t);
void TIM6_Config(uint16_t);
void TIM7_Config(uint16_t);
void TIM8_Config(uint16_t);
void TIM16_Config(uint16_t);
void TIM17_Config(uint16_t);
void TIM12_Config(uint16_t);
void TIM13_Config(uint16_t);
void TIM14_Config(uint16_t);
void Error_Handler(void);
void play_note(uint8_t, uint8_t);
void stop_note(uint8_t);
void mixer(void);
float KarlsenLPF(float, float, float, uint8_t);
void envelope(void);
void MX_GPIO_Init(void);
void MakeSound(void);
void LocalMidiHandler(uint8_t, uint8_t);

void SequencerPlay(uint16_t);
void SequencerRecord(uint8_t, uint8_t);

void PrepOscs(void);
void UpdateLFOs(void);
void ProcessPitch(void);
void ProcessADSRs(void);
void AudioOutput(void);

void writeSram(uint32_t, uint8_t);
uint8_t readSram(uint32_t);
void writeSramWord(uint32_t, uint16_t);
uint16_t readSramWord(uint32_t);

void InitPatch(void);
void ParamUpdate(uint8_t, uint8_t);

// audio callback

void AudioCallback(AudioHandle::InterleavingInputBuffer  in,
                   AudioHandle::InterleavingOutputBuffer out,
                   size_t                                size)
{	
	/* Call	the mixer routine at a 48 khz rate */
	mixer();

	/* Call the envelope routine at a 1000 hz rate */
	envtrig--;
	if(envtrig == 0)
	{
		envtrig = 48; /* 48 khz / 48 = 1000 hz */
		envelope();
	}

	for (size_t n = 0; n < size; n += 2)
	{				
		out[n] = in[n] + summer;
		out[n + 1] = in[n + 1] + delaybuffer[delay_out];
	}
}

// midi handler
void HandleMidiMessage(MidiEvent m)
{
    switch(m.type)
    {
        case NoteOn:
        {
            NoteOnEvent p = m.AsNoteOn();
			play_note(p.note, p.velocity);
			hardware.SetLed(true);
	        break;
        }
        case NoteOff:
        {
            NoteOnEvent p = m.AsNoteOn();
			stop_note(p.note);
			hardware.SetLed(false);			
	        break;
        } 
		case PitchBend:
        {
            PitchBendEvent p = m.AsPitchBend();
			bend = 64 + (p.value >> 7);		
	        break;
        } 
        case ControlChange:
        {
            ControlChangeEvent p = m.AsControlChange();
            switch(p.control_number)
            {
				case 1: // Modulation Wheel
					LocalMidiHandler(1, p.value); 
                    break;
				case 7: // Data Slider Default (Volume)
					LocalMidiHandler(param, p.value);
					break;
                default: break;
            }
			break;
        }
		case ProgramChange:
        {
            ProgramChangeEvent p = m.AsProgramChange();
			if(p.program == 31)
			{
				LocalMidiHandler(31, 0);
				break;
			}
			if(p.program == 32)
			{
				LocalMidiHandler(32, 0);
				break;
			}
			if(p.program == 33)
			{
				LocalMidiHandler(33, 0);
				break;
			}
			if(p.program == 34)
			{
				LocalMidiHandler(34, 0);
				tun = 64;
				break;
			}
			param = p.program;
			break;
		}  
    }
}

int main(void)
{
	// Init the Daisy Seed hardware
	hardware.Init(true); // true = boost to 480MHz

	hardware.SetAudioBlockSize(1);
	sysSampleRate = hardware.AudioSampleRate();
	sysCallbackRate = hardware.AudioCallbackRate();

	// Initialize USB Midi 
    MidiUsbHandler::Config midi_cfg;
    midi_cfg.transport_config.periph = MidiUsbTransport::Config::INTERNAL;
	midi.Init(midi_cfg);

	// let everything settle
	System::Delay(100);

    /* Initialize all configured peripherals */
    MX_GPIO_Init();

    /* Power up the timers and set IRQ for TIM1 */
    __HAL_RCC_TIM2_CLK_ENABLE();
    __HAL_RCC_TIM3_CLK_ENABLE();
    __HAL_RCC_TIM4_CLK_ENABLE();
    __HAL_RCC_TIM5_CLK_ENABLE();
    __HAL_RCC_TIM6_CLK_ENABLE();
    __HAL_RCC_TIM7_CLK_ENABLE();
    __HAL_RCC_TIM8_CLK_ENABLE();
    __HAL_RCC_TIM12_CLK_ENABLE();
    __HAL_RCC_TIM13_CLK_ENABLE();
    __HAL_RCC_TIM14_CLK_ENABLE();
    __HAL_RCC_TIM15_CLK_ENABLE();
    __HAL_RCC_TIM16_CLK_ENABLE();
    __HAL_RCC_TIM17_CLK_ENABLE();
    __HAL_RCC_TIM1_CLK_ENABLE();
    /* TIM1 interrupt Init */
    HAL_NVIC_SetPriority(TIM1_UP_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(TIM1_UP_IRQn);

  /* Load the initial global system settings */
	param = 0;				/* Choose no param to edit (default) */
	sus = 0;				/* Set the sustain pedal to off */

	/* Set initial values for continuous controllers */	
	paramvalue[1] = 0;			/* Set the modulation wheel to off */
	ParamUpdate(1, paramvalue[1]);
	paramvalue[2] = 64;			/* Set the master tuning to center */
	ParamUpdate(2, paramvalue[2]);

	initmodval = 1800; 			/* Set the systems initial tuning value */

	/* Get the default patch loaded into the synth engine */
	InitPatch();
	
	/* Prepare the oscillators for use */
	PrepOscs();

	// Start calling the audio callback
	hardware.StartAudio(AudioCallback);

	// Loop forever
	for(;;)
	{
        // handle MIDI Events
        midi.Listen();
        while(midi.HasEvents())
        {
            HandleMidiMessage(midi.PopEvent());
        }	
	}
}

void envelope(void)
{
	/* Process the pitch of the notes */
	ProcessPitch();
	
	/* Process the VCF/VCA ADSRs */
	ProcessADSRs();

	/* Process sequencer requests */
	SequencerPlay(seqmode);
}

void mixer(void)
{
	/* Process the waveforms for all voices */
	MakeSound();
	
	/* Process the final sound and play thru the DAC */
	AudioOutput();
	
	/* Update the LFOs */
	UpdateLFOs();
}

void AudioOutput(void)
{	
	/* Clear the sum of the oscillators */
	summer = 0.0f;
	
	/* Apply the VCA/VCF with key follow to all voices */
	for(i=0;i<6;i++)
	{
		channel[i] = channel[i] * (vcacutoff[i] * vcavellvl[i]);
		summer = summer + KarlsenLPF(channel[i], (vcfval * vcfenvlvl) * ((vcfcutoff[i] * vcfkeyfollow[i]) * vcfvellvl[i]), resonance, i);
	}
	
	/* Get the average of the sum */
	summer = summer / 6.0f;
	
	/* Process the delayed signal */
	delaybuffer[delay_in] = summer;
	delay_in++;
	if(delay_in == 1000)delay_in = 0;
	delay_out++;
	if(delay_out == 1000)delay_out = 0;
}

void PrepOscs(void)
{
	/* Cycle oscillator timers so there is no initial output spike */
	TIM14_Config(2048);
	TIM2_Config(2048);
	TIM3_Config(2048);
	TIM4_Config(2048);
	TIM5_Config(2048);
	TIM8_Config(2048);
	TIM6_Config(2048);
	TIM7_Config(2048);
	TIM16_Config(2048);
	TIM17_Config(2048);
	TIM12_Config(2048);
	TIM13_Config(2048);
	HAL_TIM_Base_Stop(&htim14);
	HAL_TIM_Base_Stop(&htim2);
	HAL_TIM_Base_Stop(&htim3);
	HAL_TIM_Base_Stop(&htim4);
	HAL_TIM_Base_Stop(&htim5);
	HAL_TIM_Base_Stop(&htim8);
	HAL_TIM_Base_Stop(&htim6);
	HAL_TIM_Base_Stop(&htim7);
	HAL_TIM_Base_Stop(&htim16);
	HAL_TIM_Base_Stop(&htim17);
	HAL_TIM_Base_Stop(&htim12);
	HAL_TIM_Base_Stop(&htim13);
}

void SequencerPlay(uint16_t modenum)
{
	if(modenum == 0)
	{
		seqclock = 0;
		seqmem = 0x00000000;
		return;
	}
	
	if(modenum == 4)
	{
		/* Read the time-stamp from SRAM and compare to seqclock*/
		seqtime = readSramWord(seqmem);
		
		while(seqtime == seqclock)
		{
			/* Move pointer to note data */
			seqmem++;
			seqmem++;
			
			/* Read the note from SRAM */
			seqnote = readSram(seqmem);
			seqmsg = seqnote & 0x80;
			seqnote = seqnote & 0x7F;
			seqmem++;
		
			/* Read the velocity from SRAM */
			seqvelocity = readSram(seqmem);
			seqmem++;
			
			if(seqvelocity == 0xFF)
			{
				/* End of sequence reached, reset the position to the beginning and restart */
				seqclock = 0;
				seqmem = 0x00000000;
				return;
			}
			
			if(seqmsg)
			{
				/* Play the note */
				play_note(seqnote, seqvelocity);
			}
			else
			{
				/* Stop the note */
				stop_note(seqnote);
			}
			
			/* Check to see if there are other events at the current sequencer clock */
			seqtime = readSramWord(seqmem);
		}
	}
	
	/* All events processed so increment the sequencer clock */
	seqclock++;
}

void SequencerRecord(uint8_t recnote, uint8_t recvelocity)
{
	/* Write the timestamp to SRAM */
		writeSramWord(seqmem, seqclock);
		seqmem++;
		seqmem++;
		
		/* Write the note to SRAM */
		writeSram(seqmem, recnote);
		seqmem++;
		
		/* Write the velocity to SRAM */
		writeSram(seqmem, recvelocity);
		seqmem++;
	
		if(seqmem > 0x1FFF8)
		{
			/* Place a timestamp and stop marker at the current position and select idle mode */
			writeSramWord(seqmem, seqclock);
			seqmem++;
			seqmem++;
			writeSramWord(seqmem, 0xFFFF);
			seqmem++;
			seqmem++;
			writeSramWord(seqmem, 0xFFFF);
			seqmode = 0;
			seqclock = 0;
			seqmem = 0x00000000;
		}
}

void play_note(uint8_t note, uint8_t velocity)
{
	if(seqmode == 1)
	{
		SequencerRecord((note | 0x80), velocity);
	}

	// If the note is playing already, just re-trigger it
	for(j=0;j<6;j++)
	{
		if(chanstat[j] == note)
		{
			// Load VCF/VCA ADS parameters.
			if(velsel == 1 | velsel == 3)
			{
				vcfvellvl[j] = ((float)(velocity) / 127.0f);
			}
			else
			{
				vcfvellvl[j] = 1.0f;
			}
			vcfsustainlvl[j] = vcfsustain;
			if(velsel >= 2)
			{
				vcavellvl[j] = ((float)(velocity) / 127.0f);
			}
			else
			{
				vcavellvl[j] = 1.0f;
			}
			vcasustainlvl[j] = vcasustain;
			
			// Set the VCF/VCA ADSR to attack state and cutoff level to zero.
			vcfcutoff[j] = 0.0f;
			vcfadsr[j] = 1;
			vcacutoff[j] = 0.0f;
			vcaadsr[j] = 1;
			return;
		}
	}	
	
	// Cycle thru available voices and assign the note to it, or steal from released voice. 
	for(j=0;j<6;j++)
	{
		if(vcfadsr[voice] == 4 | vcaadsr[voice] == 4 | chanstat[voice] == 255) // Voice is free or in released state?
		{
			// Load VCF/VCA ADS parameters.
			if(velsel == 1 | velsel == 3)
			{
				vcfvellvl[voice] = ((float)(velocity) / 127.0f);
			}
			else
			{
				vcfvellvl[voice] = 1.0f;
			}
			vcfsustainlvl[voice] = vcfsustain;
			if(velsel >= 2)
			{
				vcavellvl[voice] = ((float)(velocity) / 127.0f);
			}
			else
			{
				vcavellvl[voice] = 1.0f;
			}
			vcasustainlvl[voice] = vcasustain;
			
			// Set the VCF/VCA ADSR to attack state and cutoff level to zero.
			vcfcutoff[voice] = 0.0f;
			vcfadsr[voice] = 1;
			vcacutoff[voice] = 0.0f;
			vcaadsr[voice] = 1;
			
			chanstat[voice] = note; // Set the channel status to the note number playing
			
			// update the pitch of the voices oscillators.
			switch(voice)
			{
				case(0):
					TIM14_Config(pitchtbl[note + scale]);
					TIM6_Config(pitchtbl[note]);
					break;
				case(1):
					TIM2_Config(pitchtbl[note + scale]);
					TIM7_Config(pitchtbl[note]);
					break;
				case(2):
					TIM3_Config(pitchtbl[note + scale]);
					TIM16_Config(pitchtbl[note]);
					break;
				case(3):
					TIM4_Config(pitchtbl[note + scale]);
					TIM17_Config(pitchtbl[note]);
					break;
				case(4):
					TIM5_Config(pitchtbl[note + scale]);
					TIM12_Config(pitchtbl[note]);
					break;
				case(5):
					TIM8_Config(pitchtbl[note + scale]);
					TIM13_Config(pitchtbl[note]);
					break;
			}
			
			// Move to the next voice and return.
			voice++;
			if(voice > 5)
			{
				voice = 0;
			}
			return;
		}
		
		// Move to the next voice and continue searching.
		voice++;
		if(voice > 5)
		{
			voice = 0;
		}
	}
}

void stop_note(uint8_t note)
{
	if(seqmode == 1)
	{
		SequencerRecord(note, 0);
	}

	/* Find the voice that is playing the note and release it */
	for(n=0;n<6;n++)
	{
		if(note == chanstat[n]) // Is this the voice that is playing the note?
		{
			vcfadsr[n] = 4; // Set the voice VCF ADSR to release state
			vcaadsr[n] = 4; // Set the voice VCA ADSR to release state
			return;
		}	
	}
}

void MakeSound(void)
{
	/* Get the timer values (sawtooth oscillators) into an array */
	counter[0] = __HAL_TIM_GetCounter(&htim14);
	counter[1] = __HAL_TIM_GetCounter(&htim2);
	counter[2] = __HAL_TIM_GetCounter(&htim3);
	counter[3] = __HAL_TIM_GetCounter(&htim4);
	counter[4] = __HAL_TIM_GetCounter(&htim5);
	counter[5] = __HAL_TIM_GetCounter(&htim8);
	counter[6] = __HAL_TIM_GetCounter(&htim6);
	counter[7] = __HAL_TIM_GetCounter(&htim7);
	counter[8] = __HAL_TIM_GetCounter(&htim16);
	counter[9] = __HAL_TIM_GetCounter(&htim17);
	counter[10] = __HAL_TIM_GetCounter(&htim12);
	counter[11] = __HAL_TIM_GetCounter(&htim13);
	
	if(wavesel) /* Fall thru if wavsel = 0, all oscillators to sawtooth */
	{
		if(wavesel > 0)
		{
			/* Calculate the pwm value for the upper oscillators square wave */
			pwmval = pwval << 3;
		
			/* Apply the LFO to the pwm value before generating the square wave */
			pwmval = pwmval + ((pwm << 3) * pwmlvl);
		
			/* Generate square waves from sawtooth waves */
			for(i=0;i<6;i++)
			{
				if(counter[i] > pwmval)
				{
					counter[i] = initmodval;
				}
				else
				{
					counter[i] = 0;
				}
			}
		}
		
		if(wavesel > 1)
		{
			/* Calculate the pwm value for the lower oscillators square wave */
			pwmval = pwval << 3;
		
			/* Apply the LFO to the pwm value before generating the square wave */
			pwmval = pwmval + ((pwm2 << 3) * pwm2lvl);
		
			/* Generate square waves from sawtooth waves */
			for(i=6;i<12;i++)
			{
				if(counter[i] > pwmval)
				{
					counter[i] = initmodval;
				}
				else
				{
					counter[i] = 0;
				}
			}
		}	
	}

	/* Convert waveforms to floating point and apply oscillator mix */
	for(i=0;i<6;i++)
	{
		channel[i] = oscmix * (-1.0f + (((float)counter[i] / 1023.0f)));
		channel[i] = channel[i] + ((1.0f - oscmix) * ((-1.0f + (((float)counter[i + 6] / 1023.0f)))));
	}
}

void UpdateLFOs(void)
{
	/* Update the MOD LFO */
	lfo1cnt--;
	if(lfo1cnt == 0)
	{
		lfo1 = lfo1 + lfo1pol;
		if(lfo1 == 127 | lfo1 == 0)
		{
			lfo1pol = -lfo1pol;
		}
		lfo1cnt = lfo1rate;
	}
	
	modlvl = 1.0f - (((float)(lfo1) / 64.0f));
	
	/* Update the PWM LFO */
	lfo2cnt--;
	if(lfo2cnt == 0)
	{
		lfo2 = lfo2 + lfo2pol;
		if(lfo2 == 127 | lfo2 == 0)
		{
			lfo2pol = -lfo2pol;
		}
		lfo2cnt = lfo2rate;
	}
	
	pwmlvl = 1.0f - (((float)(lfo2)) / 64.0f);
	
	/* Update the VCF LFO */
	lfo3cnt--;
	if(lfo3cnt == 0)
	{
		lfo3 = lfo3 + lfo3pol;
		if(lfo3 == 127 | lfo3 == 0)
		{
			lfo3pol = -lfo3pol;
		}
		lfo3cnt = lfo3rate;
	}
	
	vcflvl = 1.0f - (((float)(lfo3) / 127.0f));
	
	/* Update the PWM2 LFO */
	lfo4cnt--;
	if(lfo4cnt == 0)
	{
		lfo4 = lfo4 + lfo4pol;
		if(lfo4 == 127 | lfo4 == 0)
		{
			lfo4pol = -lfo4pol;
		}
		lfo4cnt = lfo4rate;
	}

	pwm2lvl = 1.0f - (((float)(lfo4) / 64.0f));
}

void ProcessPitch(void)
{
	/* Process the pitch of the notes, start with system tuning value */
	modval = initmodval;
	
	/* Apply the LFO to modulate all the oscillators */
	modval = modval + (mod * modlvl);
	
	/* Calculate the pitch bend value */
	pbend = 1.0f - (((float)bend / 64.0f));
	
	/* Add in the pitch bend and tuning */
	modval = modval + (int16_t)(pbend * 224.0f) + (63 - tun);
	
	/* Calculate the vcf LFO mod value */
	vcfval = (1.0f - ((float)(vcf) / 127.0f)) + (((float)(vcf) / 127.0f) * vcflvl);
	
	/* Update all of the oscillators */
	TIM14->ARR = modval;
	TIM2->ARR = modval;
	TIM3->ARR = modval;
	TIM4->ARR = modval;
	TIM5->ARR = modval;
	TIM8->ARR = modval;
	/* Add in the de-tune for lower oscillators */
	TIM6->ARR = modval + det;
	TIM7->ARR = modval + det;
	TIM16->ARR = modval + det;
	TIM17->ARR = modval + det;
	TIM12->ARR = modval + det;
	TIM13->ARR = modval + det;
}

void ProcessADSRs(void)
{
	/* Process the VCF/VCA ADSR and apply to all voices */
	for(y=0;y<6;y++)
	{
		if(chanstat[y] != 255) // Is a note currently playing?
		{
			/* Calculate the key follow values based on the key follow levels */
			vcfkeyfollow[y] = (1 - vcfkflvl) + (((float)(chanstat[y]) / 76.0f) * vcfkflvl);
			envkeyfollow[y] = (1 - envkflvl) + (((float)(chanstat[y]) / 76.0f) * envkflvl);
			
			/* Update the state of the VCF ADSR */
			switch(vcfadsr[y])
			{
				case (0): // Idle state
					break;
				case (1): // Attack state
					// Calculate the VCF attack rate
					vcfattackrate[y] =  1.0f / vcfattack;
					// Ramp up the cutoff by incrementing by VCF attack rate contoured with key follow
					vcfcutoff[y] = vcfcutoff[y] + (vcfattackrate[y] * envkeyfollow[y]);
					if(vcfcutoff[y] >= 1.0f) // Attack level reached yet?
					{
						vcfcutoff[y] = 1.0f;
						vcfadsr[y] = 2; // switch to decay state
					}
					break;
				case (2): // Decay state
					// Calculate the VCF decay rate
					vcfdecayrate[y] =  (1.0f - vcfsustainlvl[y]) / vcfdecay;
					// Ramp down the cutoff by incrementing by VCF decay rate contoured with key follow
					vcfcutoff[y] = vcfcutoff[y] - (vcfdecayrate[y] * envkeyfollow[y]);
					if(vcfcutoff[y] <= vcfsustainlvl[y]) // Decay level reached yet?
					{
						vcfcutoff[y] = vcfsustainlvl[y] + vcfreleaselvl; // ensure sustainlvl > releaselvl
						vcfadsr[y] = 3; // switch to sustain state
					}
					break;
				case (3): // Sustain state
					// Just hold the VCF cutoff value steady until note is released
					break;
				case (4): // Release state
					// Calculate the VCF release rate
					vcfreleaserate[y] =  vcfcutoff[y] / vcfrelease;
					// Ramp down the cutoff by incrementing by VCF release rate
					vcfcutoff[y] = vcfcutoff[y] - (vcfreleaserate[y] * envkeyfollow[y]);
					if(vcfcutoff[y] <= vcfreleaselvl) // Release level reached yet?
					{
						vcfcutoff[y] = 0.0f;
						vcfadsr[y] = 0; // switch to idle state
						chanstat[y] = 255; // Set voice to no note playing
					}
					break;
				}
			
				/* Update the state of the VCA ADSR */
				switch(vcaadsr[y])
				{
					case (0): // Idle state
						break;
					case (1): // Attack state
						// Calculate the VCA attack rate
						vcaattackrate[y] =  1.0f / vcaattack;
						// Ramp up the amplitude by incrementing by VCA attack rate contoured with key follow
						vcacutoff[y] = vcacutoff[y] + (vcaattackrate[y] * envkeyfollow[y]);
						if(vcacutoff[y] >= 1.0f) // Attack level reached yet?
						{
							vcacutoff[y] = 1.0f;
							vcaadsr[y] = 2; // switch to decay state
						}
						break;
					case (2): // Decay state
						// Calculate the VCA decay rate
						vcadecayrate[y] =  (1.0f - vcasustainlvl[y]) / vcadecay;
						// Ramp down the cutoff by incrementing by VCA decay rate contoured with key follow
						vcacutoff[y] = vcacutoff[y] - (vcadecayrate[y] * envkeyfollow[y]);
						if(vcacutoff[y] <= vcasustainlvl[y]) // Decay level reached yet?
						{
							vcacutoff[y] = vcasustainlvl[y] + vcareleaselvl; // ensure sustainlvl > releaselvl
							vcaadsr[y] = 3; // switch to sustain state
						}
						break;
					case (3): // Sustain state
						// Just hold the VCA cutoff value steady until note is released
						break;
					case (4): // Release state
						// Calculate the VCA release rate
						vcareleaserate[y] =  vcacutoff[y] / vcarelease;
						// Ramp down the cutoff by incrementing by VCA release rate
						vcacutoff[y] = vcacutoff[y] - (vcareleaserate[y] * envkeyfollow[y]);
						if(vcacutoff[y] <= vcareleaselvl) // Release level reached yet?
						{
							vcacutoff[y] = 0.0f;
							vcaadsr[y] = 0; // switch to idle state
							chanstat[y] = 255; // Set voice to no note playing
						}
						break;
					}
		}
	}
}

// Karlsen 24dB Filter
// b_f = frequency 0..1
// b_q = resonance 0..5

float KarlsenLPF(float signal, float freq, float res, uint8_t m)
{
	static float b_inSH[6], b_in[6], b_f[6], b_q[6], b_fp[6], pole1[6], pole2[6], pole3[6], pole4[6];
	b_inSH[m] = signal;
	b_in[m] = signal;
	if(freq > 1.0f)freq = 1.0f;
	if(freq < 0.0f)freq = 0.0f;
	b_f[m] = freq;
	b_q[m] = res;
	uint8_t b_oversample = 0;
		
	while (b_oversample < 2)
	{	
		//2x oversampling
		float prevfp;
		prevfp = b_fp[m];
		if (prevfp > 1.0f) {prevfp = 1.0f;}	// Q-limiter

		b_fp[m] = (b_fp[m] * 0.418f) + ((b_q[m] * pole4[m]) * 0.582f);	// dynamic feedback
		float intfp;
		intfp = (b_fp[m] * 0.36f) + (prevfp * 0.64f);	// feedback phase
		b_in[m] =	b_inSH[m] - intfp;	// inverted feedback	

		pole1[m] = (b_in[m] * b_f[m]) + (pole1[m] * (1.0f - b_f[m]));	// pole 1
		if (pole1[m] > 1.0f) {pole1[m] = 1.0f;} else if (pole1[m] < -1.0f) {pole1[m] = -1.0f;} // pole 1 clipping
		pole2[m] = (pole1[m] * b_f[m]) + (pole2[m] * (1 - b_f[m]));	// pole 2
		pole3[m] = (pole2[m] * b_f[m]) + (pole3[m] * (1 - b_f[m]));	// pole 3
		pole4[m] = (pole3[m] * b_f[m]) + (pole4[m] * (1 - b_f[m]));	// pole 4

		b_oversample++;
	}
	return pole4[m];
}

void LocalMidiHandler(uint8_t m_param, uint8_t m_data)
{
	switch(m_param)
	{
		case (0):  // Default parameter doesn't affect the sound
			break;
		case (1): // Modulation Wheel
			mod = m_data;
			break;
		case (2): // Tuning
			tun = m_data;
			break;
		case (3): // Wave Select
			wavesel = m_data >> 5;
			break;
		case (4): // OSC Mix
			oscmix = (((float)(m_data)) * 0.007874f);
			break;
		case (5): // De-Tune 
			det = m_data >> 3;
			break;
		case (6): // Scale
			scale = (m_data - 64) >> 2;
			break;
		case (7): // Resonance
			resonance = (((float)(m_data)) * 0.007874f * 4.0f);
			break;
		case (8): // Pulse Width Value
			pwval = m_data;
			break;
		case (9): // VCF Attack
			vcfattack = (((float)(m_data)) * 10.0f);
			break;
		case (10): // VCF Decay
			vcfdecay = (((float)(m_data)) * 10.0f);
			break;
		case (11): // VCF Sustain
			vcfsustain = (((float)(m_data)) * 0.007874f);
			break;
		case (12): // VCF Release
			vcfrelease = (((float)(m_data)) * 10.0f);
			break;
		case (13): // VCA Attack
			vcaattack = (((float)(m_data)) * 10.0f);
			break;
		case (14): // VCA Decay
			vcadecay = (((float)(m_data)) * 10.0f);
			break;
		case (15): // VCA Sustain
			vcasustain = (((float)(m_data)) * 0.007874f);
			break;
		case (16): // VCA Release
			vcarelease = (((float)(m_data)) * 10.0f);
			break;
		case (17): // VCF Follow Level
			vcfkflvl = (((float)(m_data)) * 0.007874f);
			break;
		case (18): // ENV Follow Level
			envkflvl = (((float)(m_data)) * 0.007874f);
			break;
		case (19): // Velocity Select
			velsel = m_data >> 5;
			break;
		case (20): // VCF Envelope Level
			vcfenvlvl = (((float)(m_data)) * 0.007874f);
			break;
		case (21): // Mod LFO rate
			lfo1rate = (128 - m_data) << 2; 
			break;
		case (22): // Pwm LFO rate
			lfo2rate = (128 - m_data) << 2;
			break;
		case (23): // Pwm2 LFO rate
			lfo4rate = (128 - m_data) << 2;
			break;
		case (24): // Vcf LFO rate
			lfo3rate = (128 - m_data) << 2;
			break;
		case (25): // VCF LFO Mod level
			vcf = m_data;
			break;
		case (26): // PWM Level
			pwm = m_data;
			break;
		case (27): // PWM2 Level
			pwm2 = m_data;
			break;
		case (31): // Sequencer Mode Record
			seqmode = 1;
			seqclock = 0;
			break;
		case (32): // Sequencer Mode Record End
			if(seqmode == 1)
			{
				seqmode = 2;
				/* Place a time-stamp and stop marker at the current position, then stop */
				writeSramWord(seqmem, seqclock);
				seqmem++;
				seqmem++;
				writeSramWord(seqmem, 0xFFFF);
				seqmem++;
				seqmem++;
				writeSramWord(seqmem, 0xFFFF);
				seqmode = 3;
				seqclock = 0;
				seqmem = 0x00000000;
			}
			break;
		case (33): // Sequencer Mode Stop
			seqmode = 3;
			seqclock = 0;
			seqmem = 0x00000000;
			break;
		case (34): // Sequencer Mode Play
			seqmode = 4;
			seqclock = 0;
			seqmem = 0x00000000;
			break;
	}
	paramvalue[m_param] = m_data;
}

void ParamUpdate(uint8_t u_param, uint8_t u_data)
{
	switch(u_param)
	{
		case (0):  // Default parameter doesn't affect sound
			break;
		case (1): // Modulation Wheel
			mod = u_data;
			break;
		case (2): // Tuning
			tun = u_data;
			break;
		case (3): // Wave Select
			wavesel = u_data >> 5;
			break;
		case (4): // OSC Mix
			oscmix = (((float)(u_data)) * 0.007874f);
			break;
		case (5): // De-Tune 
			det = u_data >> 3;
			break;
		case (6): // Scale
			scale = (u_data - 64) >> 2;
			break;
		case (7): // Resonance
			resonance = (((float)(u_data)) * 0.007874f * 4.0f);
			break;
		case (8): // Pulse Width Value
			pwval = u_data;
			break;
		case (9): // VCF Attack
			vcfattack = (((float)(u_data)) * 10.0f);
			break;
		case (10): // VCF Decay
			vcfdecay = (((float)(u_data)) * 10.0f);
			break;
		case (11): // VCF Sustain
			vcfsustain = (((float)(u_data)) * 0.007874f);
			break;
		case (12): // VCF Release
			vcfrelease = (((float)(u_data)) * 10.0f);
			break;
		case (13): // VCA Attack
			vcaattack = (((float)(u_data)) * 10.0f);
			break;
		case (14): // VCA Decay
			vcadecay = (((float)(u_data)) * 10.0f);
			break;
		case (15): // VCA Sustain
			vcasustain = (((float)(u_data)) * 0.007874f);
			break;
		case (16): // VCA Release
			vcarelease = (((float)(u_data)) * 10.0f);
			break;
		case (17): // VCF Follow Level
			vcfkflvl = (((float)(u_data)) * 0.007874f);
			break;
		case (18): // ENV Follow Level
			envkflvl = (((float)(u_data)) * 0.007874f);
			break;
		case (19): // Velocity Select
			velsel = u_data >> 5;
			break;
		case (20): // VCF Envelope Level
			vcfenvlvl = (((float)(u_data)) * 0.007874f);
			break;
		case (21): // Mod LFO rate
			lfo1rate = (128 - u_data) << 2; 
			break;
		case (22): // Pwm LFO rate
			lfo2rate = (128 - u_data) << 2;
			break;
		case (23): // Pwm2 LFO rate
			lfo4rate = (128 - u_data) << 2;
			break;
		case (24): // Vcf LFO rate
			lfo3rate = (128 - u_data) << 2;
			break;
		case (25): // VCF LFO Mod level
			vcf = u_data;
			break;
		case (26): // PWM Level
			pwm = u_data;
			break;
		case (27): // PWM2 Level
			pwm2 = u_data;
			break;	
	}
}

void writeSram(uint32_t l_addr, uint8_t l_data)
{
  /* Pointer write on specific location of backup SRAM */
  *(__IO uint8_t *) (WRITE_READ_SRAM_ADDR + l_addr) = l_data;
}

uint8_t readSram(uint32_t l_addr)
{
	uint8_t i_retval;
	
  /* Pointer write from specific location of backup SRAM */
  i_retval =  *(__IO uint8_t *) (WRITE_READ_SRAM_ADDR + l_addr);
	
  return i_retval;
}

void writeSramWord(uint32_t l_addr_w, uint16_t l_data_w)
{
  /* Pointer write on specific location of backup SRAM */
  *(__IO uint16_t *) (WRITE_READ_SRAM_ADDR + l_addr_w) = l_data_w;
}

uint16_t readSramWord(uint32_t l_addr_w)
{
	uint16_t i_retval_w;
	
  /* Pointer write from specific location of backup SRAM */
  i_retval_w =  *(__IO uint16_t *) (WRITE_READ_SRAM_ADDR + l_addr_w);
	
  return i_retval_w;
}

void InitPatch(void)
{	
	/* Restore the default patch */
	paramvalue[0] = 64;			/* Set pitch bend to center */
	paramvalue[1] = 0;			/* Set the modulation wheel to off */
	paramvalue[2] = 64;			/* Set the master tuning to center */
	paramvalue[3] = 0;			/* Set the upper and lower waveforms to sawtooth */
	paramvalue[4] = 64;			/* Set oscillator mix to center */
	paramvalue[5] = 32;			/* Set the de-tune to 32 */
	paramvalue[6] = 64;			/* Set scale to no offset for upper oscillators */
	paramvalue[7] = 31;			/* Set resonance to 31 */
	paramvalue[8] = 127; 		/* Set the default PW value to 50% */
	paramvalue[9] = 0; 			/* Set VCF attack time */
	paramvalue[10] = 30;		/* Set VCF decay time */
	paramvalue[11] = 28;		/* Set VCF sustain level */
	paramvalue[12] = 8;			/* Set VCF release time */
	paramvalue[13] = 0;			/* Set VCA attack time */
	paramvalue[14] = 20;		/* Set VCA decay time */
	paramvalue[15] = 127;		/* Set VCA sustain level */
	paramvalue[16] = 8;			/* Set VCA release time */
	paramvalue[17] = 64;		/* Set VCF keyboard follow level to half */
	paramvalue[18] = 64;		/* Set ENV keyboard follow level to half */
	paramvalue[19] = 48;		/* Set Velocity to affect VCF only */
	paramvalue[20] = 127;		/* Set VCF envelope level to full */
	paramvalue[21] = 116;		/* Set MOD LFO rate */
	paramvalue[22] = 64;		/* Set PWM LFO rate */
	paramvalue[23] = 32;		/* Set PWM2 LFO rate */
	paramvalue[24] = 120;		/* Set VCF LFO rate */
	paramvalue[25] = 0;			/* Set VCF level to zero */
	paramvalue[26] = 0;			/* Set PWM level to zero */
	paramvalue[27] = 0;			/* Set PWM2 level to zero */

	for(loadcount=3;loadcount<28;loadcount++)
	{
		ParamUpdate(loadcount, paramvalue[loadcount]);
	}
}

void TIM2_Config(uint16_t period)
{
  htim2.Instance = TIM2;

  htim2.Init.Period            = 1027;
  htim2.Init.Prescaler         = period;
  htim2.Init.ClockDivision     = 0;
  htim2.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim2.Init.RepetitionCounter = 0;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  HAL_TIM_Base_Init(&htim2);

  HAL_TIM_Base_Start(&htim2);
}

void TIM3_Config(uint16_t period)
{
  htim3.Instance = TIM3;

  htim3.Init.Period            = 1027;
  htim3.Init.Prescaler         = period;
  htim3.Init.ClockDivision     = 0;
  htim3.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim3.Init.RepetitionCounter = 0;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  HAL_TIM_Base_Init(&htim3);

  HAL_TIM_Base_Start(&htim3);
}

void TIM4_Config(uint16_t period)
{
  htim4.Instance = TIM4;

  htim4.Init.Period            = 1027;
  htim4.Init.Prescaler         = period;
  htim4.Init.ClockDivision     = 0;
  htim4.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim4.Init.RepetitionCounter = 0;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  HAL_TIM_Base_Init(&htim4);

  HAL_TIM_Base_Start(&htim4);
}

void TIM5_Config(uint16_t period)
{
  htim5.Instance = TIM5;

  htim5.Init.Period            = 1027;
  htim5.Init.Prescaler         = period;
  htim5.Init.ClockDivision     = 0;
  htim5.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim5.Init.RepetitionCounter = 0;
  htim5.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  HAL_TIM_Base_Init(&htim5);

  HAL_TIM_Base_Start(&htim5);
}

void TIM8_Config(uint16_t period)
{
  htim8.Instance = TIM8;

  htim8.Init.Period            = 1027;
  htim8.Init.Prescaler         = period;
  htim8.Init.ClockDivision     = 0;
  htim8.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim8.Init.RepetitionCounter = 0;
  htim8.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  HAL_TIM_Base_Init(&htim8);

  HAL_TIM_Base_Start(&htim8);
}

void TIM6_Config(uint16_t period)
{
  htim6.Instance = TIM6;

  htim6.Init.Period            = 1027;
  htim6.Init.Prescaler         = period;
  htim6.Init.ClockDivision     = 0;
  htim6.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim6.Init.RepetitionCounter = 0;
  htim6.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  HAL_TIM_Base_Init(&htim6);

  HAL_TIM_Base_Start(&htim6);
}

void TIM7_Config(uint16_t period)
{
  htim7.Instance = TIM7;

  htim7.Init.Period            = 1027;
  htim7.Init.Prescaler         = period;
  htim7.Init.ClockDivision     = 0;
  htim7.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim7.Init.RepetitionCounter = 0;
  htim7.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  HAL_TIM_Base_Init(&htim7);

  HAL_TIM_Base_Start(&htim7);
}

void TIM16_Config(uint16_t period)
{
  htim16.Instance = TIM16;

  htim16.Init.Period            = 1027;
  htim16.Init.Prescaler         = period;
  htim16.Init.ClockDivision     = 0;
  htim16.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim16.Init.RepetitionCounter = 0;
  htim16.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  HAL_TIM_Base_Init(&htim16);

  HAL_TIM_Base_Start(&htim16);
}

void TIM17_Config(uint16_t period)
{
  htim17.Instance = TIM17;

  htim17.Init.Period            = 1027;
  htim17.Init.Prescaler         = period;
  htim17.Init.ClockDivision     = 0;
  htim17.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim17.Init.RepetitionCounter = 0;
  htim17.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  HAL_TIM_Base_Init(&htim17);

  HAL_TIM_Base_Start(&htim17);
}

void TIM12_Config(uint16_t period)
{
  htim12.Instance = TIM12;

  htim12.Init.Period            = 1027;
  htim12.Init.Prescaler         = period;
  htim12.Init.ClockDivision     = 0;
  htim12.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim12.Init.RepetitionCounter = 0;
  htim12.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  HAL_TIM_Base_Init(&htim12);

  HAL_TIM_Base_Start(&htim12);
}

void TIM13_Config(uint16_t period)
{
  htim13.Instance = TIM13;

  htim13.Init.Period            = 1027;
  htim13.Init.Prescaler         = period;
  htim13.Init.ClockDivision     = 0;
  htim13.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim13.Init.RepetitionCounter = 0;
  htim13.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  HAL_TIM_Base_Init(&htim13);

  HAL_TIM_Base_Start(&htim13);
}

void TIM14_Config(uint16_t period)
{
  htim14.Instance = TIM14;

  htim14.Init.Period            = 1027;
  htim14.Init.Prescaler         = period;
  htim14.Init.ClockDivision     = 0;
  htim14.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim14.Init.RepetitionCounter = 0;
  htim14.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  HAL_TIM_Base_Init(&htim14);

  HAL_TIM_Base_Start(&htim14);
}

void TIM1_Config(uint16_t period)
{
  htim1.Instance = TIM1;

  htim1.Init.Period            = 1027;
  htim1.Init.Prescaler         = period;
  htim1.Init.ClockDivision     = 0;
  htim1.Init.CounterMode       = TIM_COUNTERMODE_UP;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_ENABLE;
  HAL_TIM_Base_Init(&htim1);

  sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;

  HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig);

  HAL_TIM_Base_Start_IT(&htim1);
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
void MX_GPIO_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOH_CLK_ENABLE();
  	__HAL_RCC_GPIOA_CLK_ENABLE();
  	__HAL_RCC_GPIOB_CLK_ENABLE();
  	__HAL_RCC_GPIOC_CLK_ENABLE();
  	__HAL_RCC_GPIOD_CLK_ENABLE();

    /*Configure GPIO pin : Boot_Button_Pin */
    GPIO_InitStruct.Pin = Boot_Button_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(Boot_Button_GPIO_Port, &GPIO_InitStruct);
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
