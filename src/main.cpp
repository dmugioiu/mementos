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
const double samplingFrequency = 21000;

arm_cfft_radix4_instance_f32 C;
arm_rfft_instance_f32 S;

float32_t fftbufReal[samples];
float32_t fftbufComplex[samples*2];

volatile int bufn,obufn;
uint16_t buf[4][samples];   // 4 buffers of x readings

volatile int pwm_duty = 0;
volatile int goalFreq = 0;

const uint16_t pwmCount = 256;
const uint16_t pitchCount = sizeof(midiNotePitches) / sizeof(uint16_t);

float32_t pwm_to_freq_values[pwmCount];
uint16_t freq_to_pwm_values[pitchCount] = {0};


bool nCalibrated = true;

//arm_linear_interp_instance_f32 S = {256, 0, 1, &pwm_to_freq_values[0]};


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

void ADC_Handler(){     // move DMA pointers to next buffer
  int f=ADC->ADC_ISR;
  if (f&(1<<27)){ // DMA End of RX Buffer Interrupt
   bufn=(bufn+1)&3;
   ADC->ADC_RNPR=(uint32_t)buf[bufn];
   ADC->ADC_RNCR=samples;
  } 
}

void setupADC(void) {

  pmc_enable_periph_clk(ID_ADC);
  adc_init(ADC, SystemCoreClock, ADC_FREQ_MAX, ADC_STARTUP_FAST);

  //ADC->ADC_MR |=0x80; // Free running (only for Debug)
  ADC->ADC_MR = (ADC->ADC_MR & 0xFFFFFFF0)| (1 << 1) | ADC_MR_TRGEN;   // Trig source TIO from TC0

  NVIC_EnableIRQ(ADC_IRQn);
  NVIC_SetPriority(ADC_IRQn,6); 

  ADC->ADC_IDR = 0xFFFFFFFF ;   // Disable interrupts
  ADC->ADC_IER = (1<<27) ;         // Enable End of RX Buffer Interrupt
  ADC->ADC_CHDR = 0xFFFF ;      // Disable all channels
  ADC->ADC_CHER = 0x80 ;        // Enable just A0 (AD7)
  ADC->ADC_CGR = 0x15555555 ;   // No Gain
  ADC->ADC_COR = 0x00000000 ;   // No Offsets

  
  
 
  ADC->ADC_RPR=(uint32_t)buf[0];   // Current DMA buffer pointer
  ADC->ADC_RCR=samples; // Current DMA transfer size
  ADC->ADC_RNPR=(uint32_t)buf[1]; // Next DMA DMA buffer pointer
  ADC->ADC_RNCR=samples; // Next DMA transfer size
  bufn=obufn=1;
  ADC->ADC_PTCR=1; // Enable RX DMA Transfer
  ADC->ADC_CR=2; // Start ADC

 
  
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
        goalFreq = midiNotePitches[currentNote];
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

void setup() {

  Serial.begin(115200);
  while (!Serial);

  setupADC();
  setupTimer();

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


  Serial.println("Mementos ready.");

  setupADC();
  setupTimer();

  arm_rfft_init_f32(&S,&C,samples,false,true);

  
}

void loop() {

  float32_t fftResultAmplitude;
  uint32_t fftResultBin;
  float32_t fftResultFreq;

  // Buffer is Ready
  if (obufn!=bufn && nCalibrated) {

    // Copy and rescale data from ADC buffer to FFT buffer
    for (uint16_t i = 0; i < samples; i++)
      {
        fftbufReal[i] = buf[obufn][i];
      }
    arm_scale_f32(fftbufReal,0.001,fftbufReal,samples);

    // Perform FFT
    arm_rfft_f32(&S,fftbufReal,fftbufComplex);
    // Compute Magnitude. Spectum is Symmetric, so only half length necessary
    arm_cmplx_mag_squared_f32(fftbufComplex,fftbufReal,samples/2);
    // Zero DC Component
    fftbufReal[0] = 0; 
    // Find Peak and calculate Frequency
    arm_max_f32(fftbufReal,samples/4,&fftResultAmplitude,&fftResultBin);
    fftResultFreq = ((float32_t)samplingFrequency / (float32_t)samples) * fftResultBin;

    // Rotate Buffer
    obufn=(obufn+1)&3;

    //FFT.Windowing(vReal, samples, FFT_WIN_TYP_HAMMING, FFT_FORWARD);  /* Weigh data */
    //FFT.Compute(vReal, vImag, samples, FFT_FORWARD); /* Compute FFT */
    //FFT.ComplexToMagnitude(vReal, vImag, samples); /* Compute magnitudes */
    //double x = FFT.MajorPeak(vReal, samples, samplingFrequency);
    
    // FFT Detection Threshold
    if (fftResultAmplitude > 1000.0) {

      pwm_to_freq_values[pwm_duty] = fftResultFreq;
      pwm_duty++;
      pwm.pinDuty( 6, pwm_duty );

      // if (fftResultFreq > goalFreq + 2) {
      //   pwm_duty--;
      //   if (pwm_duty < 0) pwm_duty = 0;
      // }
      // if (fftResultFreq < goalFreq - 2) {
      //   pwm_duty++;
      //   if (pwm_duty > 255) pwm_duty = 255;
      // }


      Serial.write(27);
      Serial.print("[2J"); // clear screen
      Serial.write(27);
      Serial.print("[H"); // cursor to home
      Serial.print("[Motor] Control Loop Target Frequency ");
      Serial.print(goalFreq);
      Serial.print("\t Actual Frequency: ");
      Serial.print(fftResultFreq, 1);
      Serial.print("\t Amplitude: ");
      Serial.print(fftResultAmplitude, 1);
      Serial.print("\t PWM: ");
      Serial.println(pwm_duty);

    }
    else {

      // No Valid Signal Detected
      pwm_to_freq_values[pwm_duty] = 0;
      pwm_duty++;
      pwm.pinDuty( 6, pwm_duty );


      Serial.write(27);
      Serial.print("[2J"); // clear screen
      Serial.write(27);
      Serial.print("[H"); // cursor to home
      Serial.println("[Motor] No Audio Activity Detected ");
      Serial.print("\t PWM: ");
      Serial.println(pwm_duty);
    }
    if (pwm_duty > 254) {
      nCalibrated = false;
      pwm_duty = 0;
      pwm.pinDuty( 6, pwm_duty );
      generateLookupFreqToPWMTable();
      for (uint16_t i = 0; i < 256; i++)
      {
        Serial.println(pwm_to_freq_values[i]);
      }

      for (uint16_t i = 0; i < pitchCount; i++)
      {
        Serial.println(freq_to_pwm_values[i]);
      }

    }
  
  }

  MIDI.read();
  //stepper.runSpeed();


}
