#include <Arduino.h>
#include <USB-MIDI.h>
#include <arduinoFFT.h>
#include "pitches.h"
#include "noteList.h"
#include <AccelStepper.h>
#include "DuePWM.h"

#define ARM_MATH_CM3
#include <arm_math.h>

#define STEPPER1_DIR_PIN 3
#define STEPPER1_STEP_PIN 2


USBMIDI_CREATE_DEFAULT_INSTANCE();
AccelStepper stepper(AccelStepper::DRIVER, STEPPER1_STEP_PIN, STEPPER1_DIR_PIN);
//arduinoFFT FFT = arduinoFFT();

#define PWM_FREQ1  8000
#define PWM_FREQ2  8000

DuePWM pwm( PWM_FREQ1, PWM_FREQ2 );

static const unsigned sMaxNumNotes = 16;
MidiNoteList<sMaxNumNotes> midiNotes;

const uint16_t samples = 2048; //This value MUST ALWAYS be a power of 2
const float32_t samplingFrequency = 21000;

arm_cfft_radix4_instance_f32 C;
arm_rfft_instance_f32 S;

float32_t fftbufReal[samples];
float32_t fftbufComplex[samples*2];

volatile bool adc_ready = false;
uint16_t adc_buf[samples];

volatile int pwm_duty = 0;

const uint16_t pwmCount = 256;
const uint16_t pitchCount = sizeof(midiNotePitches) / sizeof(uint16_t);

float32_t pwm_to_freq_values[pwmCount];
uint16_t freq_to_pwm_values[pitchCount] = {0};


void generateLookupFreqToPWMTable() {
  for (int i = 0; i < pitchCount; i++) {
    uint8_t iteration = 1;
    uint16_t j = pwmCount / (0x01 << iteration);
    float32_t freq = (float32_t) midiNotePitches[i];

    while (true) {
      if (pwm_to_freq_values[j] == freq) {
        freq_to_pwm_values[i] = j;
        break;
      } else if (pwm_to_freq_values[j] < freq) {
        if ((pwm_to_freq_values[j] + 1) > freq) {
          freq_to_pwm_values[i] = j;
          // TODO: Interpolate
          break;
        } else {
          j +=  pwmCount / (0x01 << iteration);
          if (j > pwmCount) {
            freq_to_pwm_values[i] = pwmCount - 1;
            break;
          }
        }
      } else if (pwm_to_freq_values[j] > freq) {
        if ((pwm_to_freq_values[j] - 1) < freq) {
          freq_to_pwm_values[i] = j;
          // TODO: Interpolate
          break;
        } else {
          if ((pwmCount / (0x01 << iteration)) < j) {
            j -=  pwmCount / (0x01 << iteration);
          } else {
            freq_to_pwm_values[i] = 0;
            break;
          }
        }
      }

      iteration++;
    }
  }
}

void setupTimer(void) {

  pmc_enable_periph_clk (TC_INTERFACE_ID + 0*3+0) ;  // clock the TC0 channel 0

  TcChannel * t = &(TC0->TC_CHANNEL)[0] ;    // pointer to TC0 registers for its channel 0
  t->TC_CCR = TC_CCR_CLKDIS ;  // disable internal clocking while setup regs
  t->TC_IDR = 0xFFFFFFFF ;     // disable interrupts
  t->TC_SR ;                   // read int status reg to clear pending
  t->TC_CMR = TC_CMR_TCCLKS_TIMER_CLOCK1 |   // use TCLK1 (prescale by 2, = 42MHz)
              TC_CMR_WAVE |                  // waveform mode
              TC_CMR_WAVSEL_UP_RC |          // count-up PWM using RC as threshold
              TC_CMR_EEVT_XC0 |     // Set external events from XC0 (this setup TIOB as output)
              TC_CMR_ACPA_CLEAR | TC_CMR_ACPC_CLEAR |
              TC_CMR_BCPB_CLEAR | TC_CMR_BCPC_CLEAR ;
  
  t->TC_RC =  42000000UL/samplingFrequency ;     // counter resets on RC, so sets period in terms of 42MHz clock
  t->TC_RA =  1 ;    
  t->TC_CMR = (t->TC_CMR & 0xFFF0FFFF) | TC_CMR_ACPA_CLEAR | TC_CMR_ACPC_SET ;  // set clear and set from RA and RC compares
  
  t->TC_CCR = TC_CCR_CLKEN | TC_CCR_SWTRG ;  // re-enable local clocking and switch to hardware trigger source.

}

void ADC_Handler(){     

  if (ADC->ADC_ISR & ADC_ISR_ENDRX) {                                 // DMA End of RX Buffer Interrupt
    ADC->ADC_MR = (ADC->ADC_MR & 0xFFFFFFF0);                         // No Trigger
    ADC->ADC_PTCR = ADC_PTCR_RXTDIS;                                  // Disable RX DMA Transfer
    ADC->ADC_RPR = (uint32_t)adc_buf;                                 // Load DMA buffer pointer
    ADC->ADC_RCR = samples;                                           // Load DMA transfer size
    adc_ready = true;
  } 
}

void setupADC(void) {

  pmc_enable_periph_clk(ID_ADC);
  adc_init(ADC, SystemCoreClock, ADC_FREQ_MAX, ADC_STARTUP_FAST);

  ADC->ADC_PTCR = ADC_PTCR_RXTDIS;                                    // Disable RX DMA Transfer
  ADC->ADC_IDR = 0xFFFFFFFF;                                          // Disable interrupts
  ADC->ADC_IER = ADC_IER_ENDRX;                                       // Enable End of RX Buffer Interrupt
  ADC->ADC_CHDR = 0xFFFF;                                             // Disable all channels
  ADC->ADC_CHER = ADC_CHER_CH7;                                       // Enable just A0 (AD7)
  ADC->ADC_CGR = 0x15555555;                                          // No Gain
  ADC->ADC_COR = 0x00000000;                                          // No Offsets

  ADC->ADC_RPR = (uint32_t)adc_buf;                                   // Load DMA buffer pointer
  ADC->ADC_RCR = samples;                                             // Load DMA transfer size

  //ADC->ADC_MR |=0x80;                                               // Free running (only for Debug)
  ADC->ADC_MR = (ADC->ADC_MR & 0xFFFFFFF0);                           // No Trigger

  NVIC_SetPriority(ADC_IRQn,6); 
  NVIC_EnableIRQ(ADC_IRQn);
}

void startADC(void){

  adc_ready = false;
  ADC->ADC_PTCR = ADC_PTCR_RXTEN;                                     // Enable RX DMA Transfer
  ADC->ADC_MR = (ADC->ADC_MR & 0xFFFFFFF0)| ADC_MR_TRGSEL_ADC_TRIG1 | ADC_MR_TRGEN;  // Trig source TIO from TC0
}

void handleNoteUpdate(void)
{
if (midiNotes.empty())
    {
        //stepper.setSpeed(0);
    }
    else
    {
        byte currentNote = 0;
        midiNotes.getLast(currentNote);
        //stepper.setSpeed((float)midiNotePitches[currentNote]*4.0);
        //goalFreq = midiNotePitches[currentNote];
    }
}

void handleNoteOn(byte inChannel, byte inNote, byte inVelocity)
{
  Serial.print("[MIDI] NoteOn  ");
  Serial.print(inNote);
  Serial.print("\tvelocity: ");
  Serial.println(inVelocity);
  midiNotes.remove(inNote);
  midiNotes.add(MidiNote(inNote, inVelocity));
  handleNoteUpdate();
}

void handleNoteOff(byte inChannel, byte inNote, byte inVelocity)
{
  Serial.print("[MIDI] NoteOff ");
  Serial.print(inNote);
  Serial.print("\tvelocity: ");
  Serial.println(inVelocity);
  midiNotes.remove(inNote);
  handleNoteUpdate();
}

void handleStop()
{
  Serial.print("[MIDI] Stop ");
  byte currentNote = 0;
  while(midiNotes.getLast(currentNote))
  {
    midiNotes.remove(currentNote);
  }
  handleNoteUpdate();
}

float32_t interpolatePeak() {
	float32_t maxY = 0;
	uint16_t IndexOfMaxY = 0;
	//If sampling_frequency = 2 * max_frequency in signal,
	//value would be stored at position samples/2
	for (uint16_t i = 1; i < ((samples >> 1) + 1); i++) {
		if ((fftbufReal[i-1] < fftbufReal[i]) && (fftbufReal[i] > fftbufReal[i+1])) {
			if (fftbufReal[i] > maxY) {
				maxY = fftbufReal[i];
				IndexOfMaxY = i;
			}
		}
	}
	float32_t delta = 0.5 * ((fftbufReal[IndexOfMaxY-1] - fftbufReal[IndexOfMaxY+1]) / (fftbufReal[IndexOfMaxY-1] - (2.0 * fftbufReal[IndexOfMaxY]) + fftbufReal[IndexOfMaxY+1]));
	float32_t interpolatedX = ((IndexOfMaxY + delta)  * samplingFrequency) / (samples-1);
	if(IndexOfMaxY==(samples >> 1)) //To improve calculation on edge values
		interpolatedX = ((IndexOfMaxY + delta)  * samplingFrequency) / (samples);
	// returned value: interpolated frequency peak apex
	return(interpolatedX);
}

void runParametrization(void) {

  float32_t fftResultAmplitude;
  float32_t fftResultFreq;
  uint32_t fftResultBin;
  
  Serial.println("[DC Motor] Starting Sweep.");

  for (uint16_t pwm_duty = 0; pwm_duty < pwmCount; pwm_duty++)
    {
      pwm.pinDuty( 6, pwm_duty );
      startADC();
      while(!adc_ready);

      // Copy and rescale data from ADC buffer to FFT buffer
      for (uint16_t i = 0; i < samples; i++)
        {
          fftbufReal[i] = adc_buf[i];
        }
      arm_scale_f32(fftbufReal,0.001,fftbufReal,samples);

      // Perform FFT
      arm_rfft_f32(&S,fftbufReal,fftbufComplex);
      // Compute Magnitude. Spectum is Symmetric, so only half length necessary
      arm_cmplx_mag_squared_f32(fftbufComplex,fftbufReal,samples/2);
      // Zero DC Component
      fftbufReal[0] = 0; 
      // Find Peak and calculate Frequency
      // TODO: This is doppelt gemoppelt
      arm_max_f32(fftbufReal,samples/4,&fftResultAmplitude,&fftResultBin);
      //fftResultFreq = ((float32_t)samplingFrequency / (float32_t)samples) * fftResultBin;
      fftResultFreq = interpolatePeak();

      //FFT.Windowing(vReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);  /* Weigh data */
      //FFT.Compute(vReal, vImag, samples, FFT_FORWARD); /* Compute FFT */
      //FFT.ComplexToMagnitude(vReal, vImag, samples); /* Compute magnitudes */
      //float32_t x = FFT.MajorPeak(vReal, samples, samplingFrequency);

        // FFT Detection Threshold
      if (fftResultAmplitude > 1000.0) {

        pwm_to_freq_values[pwm_duty] = fftResultFreq;

        Serial.write(27);
        Serial.print("[2J"); // clear screen
        Serial.write(27);
        Serial.print("[H"); // cursor to home
        Serial.print("[DC Motor] Measuring at PWM ");
        Serial.print(pwm_duty);
        Serial.print("\t Frequency: ");
        Serial.print(fftResultFreq, 1);
        Serial.print("\t Amplitude: ");
        Serial.println(fftResultAmplitude, 1);

      }
      else {

        // No Valid Signal Detected
        pwm_to_freq_values[pwm_duty] = 0;

        Serial.write(27);
        Serial.print("[2J"); // clear screen
        Serial.write(27);
        Serial.print("[H"); // cursor to home
        Serial.print("[DC Motor] Measuring at PWM ");
        Serial.print(pwm_duty);
        Serial.println("\t No Audio Activity Detected ");
        
      }
    }
}

void setup() {

  Serial.begin(115200);
  while (!Serial);
  Serial.println("Mementos init...");

  setupTimer();
  setupADC();

  MIDI.begin();
  MIDI.setHandleNoteOn(handleNoteOn);
  MIDI.setHandleNoteOff(handleNoteOff);
  MIDI.setHandleStop(handleStop);

  stepper.setMaxSpeed(4000);

  handleNoteUpdate();

  pwm.setFreq1( PWM_FREQ1 );
  pwm.setFreq2( PWM_FREQ2 );
  pwm.pinFreq1( 6 );
  pwm.pinDuty( 6, pwm_duty );

  arm_rfft_init_f32(&S,&C,samples,false,true);

  Serial.println("Mementos ready.");

  runParametrization();
  
  

  
}

void loop() {

  


  MIDI.read();
  //stepper.runSpeed();


}
