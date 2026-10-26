// RTTY Receiver Code - Alternate code


const float SAMPLING_FREQ = 8928.0;
const int NUM_SAMPLES = 49; 
const int TEST_PIN = 8;     

const float MARK_FREQ = 2295.0;
const float SPACE_FREQ = 2125.0;

float coeff_mark = 0.0;
float coeff_space = 0.0;

float squelch_limit = 10000.0;

volatile int adc_samples[NUM_SAMPLES];
volatile int sample_count = 0;
volatile bool buffer_full = false;

int working_samples[NUM_SAMPLES];

int receiver_step = 0;
int chunk_count = 0;
int bits_read = 0;
int ascii_value = 0;
int bit_weight = 1; 

void setup() {
  Serial.begin(115200);
  pinMode(TEST_PIN, OUTPUT);

  float omega_mark = (2.0 * PI * MARK_FREQ) / SAMPLING_FREQ;
  coeff_mark = 2.0 * cos(omega_mark);

  float omega_space = (2.0 * PI * SPACE_FREQ) / SAMPLING_FREQ;
  coeff_space = 2.0 * cos(omega_space);

  setupTimerAndADC();

  Serial.println("RTTY Receiver Starting...");
}

void loop() {
  if (buffer_full == true) {
    buffer_full = false;

     for (int i = 0; i < NUM_SAMPLES; i++) {
       working_samples[i] = adc_samples[i];
    }

    long sum = 0;
     for (int i = 0; i < NUM_SAMPLES; i++) {
      sum = sum + working_samples[i];
    }
     float dc_average = (float)sum / NUM_SAMPLES;

    float mark_power = runGoertzel(coeff_mark, dc_average);
    float space_power = runGoertzel(coeff_space, dc_average);

    //Serial.print(mark_power);
    //Serial.print(" ");
    //Serial.println(space_power);

    int detected_bit = 1; 

    if ((mark_power + space_power) > squelch_limit) {
      if (mark_power > space_power) {
        detected_bit = 1;
      } else {
        detected_bit = 0;
      }
    }

    if (detected_bit == 1) {
      digitalWrite(TEST_PIN, HIGH);
    } else {
      digitalWrite(TEST_PIN, LOW);
    }

    // - - UART Receiver Logic 

    if (receiver_step == 0) {
      if (detected_bit == 0) {
        receiver_step = 1;
        chunk_count = 0;
      }
    }
    // Step 1: Wait 1 chunk to reach the middle of the Start Bit and make sure it's still 0
    else if (receiver_step == 1) {
      chunk_count = chunk_count + 1;
      if (chunk_count == 1) {
        if (detected_bit == 0) {
          // Start bit is valid, get ready to read 8 data bits
          receiver_step = 2;
          chunk_count = 0;
          bits_read = 0;
          ascii_value = 0;
          bit_weight = 1;
        } else {
          // False alarm from noise, go back to waiting
          receiver_step = 0;
        }
      }
    }
    // Step 2: Wait 4 chunks (1 full bit time = 22 ms) between each of the 8 data bits
    else if (receiver_step == 2) {
      chunk_count = chunk_count + 1;
      if (chunk_count == 4) {
        chunk_count = 0;

        if (detected_bit == 1) {
          ascii_value = ascii_value + bit_weight;
        }

        bit_weight = bit_weight * 2;
        bits_read = bits_read + 1;

        // Once all 8 bits are read, move to the Stop Bit
        if (bits_read == 8) {
          receiver_step = 3;
        }
      }
    }
    else if (receiver_step == 3) {
      chunk_count = chunk_count + 1;
      if (chunk_count == 4) {
        if (detected_bit == 1) {
          char letter = (char)ascii_value;
          Serial.print(letter);
        }
        receiver_step = 0;
      }
      }
      }
      }

float runGoertzel(float coeff, float average_offset) {
  float q0 = 0.0;
  float q1 = 0.0;
  float q2 = 0.0;

  for (int i = 0; i < NUM_SAMPLES; i++) {
    float sample = (float)working_samples[i] - average_offset;
    q0 = (coeff * q1) - q2 + sample;
    q2 = q1;
    q1 = q0;
  }

  float magnitude_squared = (q1 * q1) + (q2 * q2) - (q1 * q2 * coeff);
  return magnitude_squared;
}

ISR(TIMER1_COMPA_vect) {
  int reading = ADC;

  ADCSRA |= (1 << ADSC);

  adc_samples[sample_count] = reading;
  sample_count = sample_count + 1;

  if (sample_count >= NUM_SAMPLES) {
    sample_count = 0;
    buffer_full = true;
  }
}

// Function to configure the ATmega328 ADC and Timer 1 registers
void setupTimerAndADC() {
  cli(); 

  ADMUX = (1 << REFS0);

  ADCSRA = (1 << ADEN) | (1 << ADSC) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);

  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;

  OCR1A = 1791;
  TCCR1B |= (1 << WGM12) | (1 << CS10);


  TIMSK1 |= (1 << OCIE1A);

  sei(); // Turn interrupts back on
}