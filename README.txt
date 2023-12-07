Description of the project:

This code is a port of my old STM32CubeIDE project "NucleoSynth" that was written back in 2006.
I (Christopher @Nettech15) have modified it to run on the Daisy Seed environment using the VSCode IDE.
The USB-MIDI and Audio In/Out routines are handled by the LibDaisy Library.
The Synth Engine is entirely handled thru HAL, however.
Synth parameters are now controlled by a Miditech i2-61 midi keyboard.
Feel free to copy, modify, and improve this code to match your equipment and sound requirements.

Forked from STM32 Daisy Seed PolySynth v1.3.6. STM32CubeIDE Project.

Specifications:

- 6 voice polyphonic.
- Dual oscillator per voice.
- VCA per voice. 
- VCF (4-pole Resonant low-pass) per voice.
- Separate ADSR's for VCA and VCF
- MIDI Editable parameters. 
- Waveform select for both oscillators; Saw / Square(PWM).
- Oscillator Mix. 
- Oscillator #2 De-tune. 
- Scale for Oscillator #2. 
- LFO for Pitch Modulation Wheel.
- LFO for Pulse Width Modulation #1.
- LFO for Pulse Width Modulation #2.
- LFO for VCF Modulation.
- Keyboard follow for VCF and ENV.
- Keyboard velocity routable to VCA and/or VCF. 
- VCF envelope level. 
- Stereo simulation effect.
- ScratchPad Sequencer.
