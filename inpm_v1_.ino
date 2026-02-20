#include <driver/i2s.h>
#include <arduinoFFT.h>

// ================= I2S CONFIG =================
#define I2S_WS 15
#define I2S_SD 33
#define I2S_SCK 14
#define I2S_PORT I2S_NUM_0

#define I2S_SAMPLE_RATE 16000
#define SAMPLES_PER_CHUNK 256
#define I2S_SAMPLE_BITS 32

#ifndef LED_BUILTIN
#define LED_BUILTIN 2
#endif

// =================================================================
// ====== ADAPTIVE TUNING (THE "TWO-STAGE" FILTER) ======
// =================================================================

#define SOFTWARE_GAIN_FACTOR 0.8    
#define TRIGGER_AMP_THRESHOLD 1000  

#define MIN_CONSECUTIVE_CHUNKS 8    
#define MAX_CONSECUTIVE_CHUNKS 35  
#define MAX_EVENT_DURATION 600      

#define RATIO_STANDARD 3.0
#define ZCR_STANDARD 75
#define RATIO_STRICT 13.0
#define ZCR_STRICT 100
#define MAX_LOW_ENERGY_THRESHOLD 60000

// =================================================================

ArduinoFFT<double> FFT = ArduinoFFT<double>();
double vReal[SAMPLES_PER_CHUNK];
double vImag[SAMPLES_PER_CHUNK];

// Buffer to hold the data for FFT analysis
int16_t peak_chunk_buffer[SAMPLES_PER_CHUNK];

unsigned long ledOnTime = 0;
const int ledOnDuration = 5000;
TaskHandle_t AudioTaskHandle;

// =================================================================
// ====== CORE 0 TASK ======
// =================================================================
void AudioProcessingTask(void * parameter) {
  
  enum State { IDLE, TRIGGERED };
  State currentState = IDLE;
  unsigned long eventStartTime = 0;
  int consecutiveLoudChunks = 0;
  int16_t peak_amplitude_of_event = 0;

  // We double the raw read buffer to hold both Left and Right samples
  int32_t samples32[SAMPLES_PER_CHUNK * 2]; 
  
  // Separate arrays for de-interleaved stereo data
  int16_t samplesLeft[SAMPLES_PER_CHUNK];
  int16_t samplesRight[SAMPLES_PER_CHUNK];
  
  size_t bytes_read;

  for(;;) {
    i2s_read(I2S_PORT, samples32, sizeof(samples32), &bytes_read, portMAX_DELAY);
    
    if (bytes_read > 0) {
      int16_t current_peak_L = 0;
      int16_t current_peak_R = 0;
      int sampleIdx = 0;

      // De-interleave the 32-bit data into two 16-bit arrays
      for (int i = 0; i < (SAMPLES_PER_CHUNK * 2); i += 2) {
        
        // --- Process Left Channel ---
        int32_t sL = samples32[i] >> 16; 
        sL *= SOFTWARE_GAIN_FACTOR;
        if (sL > 32767) sL = 32767;
        if (sL < -32768) sL = -32768;
        samplesLeft[sampleIdx] = (int16_t)sL;
        if (abs(samplesLeft[sampleIdx]) > current_peak_L) current_peak_L = abs(samplesLeft[sampleIdx]);

        // --- Process Right Channel ---
        int32_t sR = samples32[i+1] >> 16; 
        sR *= SOFTWARE_GAIN_FACTOR;
        if (sR > 32767) sR = 32767;
        if (sR < -32768) sR = -32768;
        samplesRight[sampleIdx] = (int16_t)sR;
        if (abs(samplesRight[sampleIdx]) > current_peak_R) current_peak_R = abs(samplesRight[sampleIdx]);

        sampleIdx++;
      }

      // Determine the overall loudest peak between both microphones
      int16_t current_peak = max(current_peak_L, current_peak_R);

      // Determine which channel had the loudest audio to save for FFT analysis
      int16_t* loudest_channel_buffer = (current_peak_L > current_peak_R) ? samplesLeft : samplesRight;

      switch (currentState) {
        case IDLE:
          if (current_peak > TRIGGER_AMP_THRESHOLD) {
            currentState = TRIGGERED;
            eventStartTime = millis();
            consecutiveLoudChunks = 1;
            peak_amplitude_of_event = current_peak;
            
            // Save the audio from the loudest microphone for FFT
            memcpy(peak_chunk_buffer, loudest_channel_buffer, sizeof(samplesLeft));
            Serial.printf("Triggered (Amp: %d)... ", current_peak);
          }
          break;

        case TRIGGERED:
          if (current_peak > (TRIGGER_AMP_THRESHOLD / 2)) {
            consecutiveLoudChunks++;
            if (current_peak > peak_amplitude_of_event) {
              peak_amplitude_of_event = current_peak;
              memcpy(peak_chunk_buffer, loudest_channel_buffer, sizeof(samplesLeft));
            }
          }

          if (millis() - eventStartTime > MAX_EVENT_DURATION) {
            if (consecutiveLoudChunks >= MIN_CONSECUTIVE_CHUNKS && consecutiveLoudChunks <= MAX_CONSECUTIVE_CHUNKS) {
              
              // FFT Processing (Running on the loudest channel's data)
              for (int i = 0; i < SAMPLES_PER_CHUNK; i++) {
                vReal[i] = peak_chunk_buffer[i];
                vImag[i] = 0;
              }
              FFT.windowing(vReal, SAMPLES_PER_CHUNK, FFT_WIN_TYP_HAMMING, FFT_FORWARD);
              FFT.compute(vReal, vImag, SAMPLES_PER_CHUNK, FFT_FORWARD);
              FFT.complexToMagnitude(vReal, vImag, SAMPLES_PER_CHUNK);

              double lowEnergy = 0;
              double highEnergy = 0;

              for (int i = 2; i < SAMPLES_PER_CHUNK / 2; i++) {
                double freq = i * 62.5;
                if (freq < 1000) lowEnergy += vReal[i];
                if (freq > 2500) highEnergy += vReal[i];
              }
              if (lowEnergy == 0) lowEnergy = 1; 

              double ratio = highEnergy / lowEnergy;
              
              int peak_zcr = 0;
              int16_t p = 0;
              for(int k=0; k<SAMPLES_PER_CHUNK; k++){
                 if ((peak_chunk_buffer[k] > 0 && p <= 0) || (peak_chunk_buffer[k] < 0 && p >= 0)) peak_zcr++;
                 p = peak_chunk_buffer[k];
              }

              Serial.printf("Done. Dur:%d | Ratio:%.2f | ZCR:%d ", consecutiveLoudChunks, ratio, peak_zcr);

              bool isGunshot = false;
              bool passBass = (lowEnergy < MAX_LOW_ENERGY_THRESHOLD);

              if (consecutiveLoudChunks <= 25) {
                 if (ratio > RATIO_STANDARD && peak_zcr > ZCR_STANDARD && passBass) {
                    isGunshot = true;
                    Serial.print("[Standard Pass]");
                 }
              } else {
                 if ((ratio > RATIO_STRICT || peak_zcr > ZCR_STRICT) && passBass) {
                    isGunshot = true;
                    Serial.print("[Strict Pass]");
                 } else {
                    Serial.print("[Strict Fail: Likely Thunder]");
                 }
              }

              if (isGunshot) {
                digitalWrite(LED_BUILTIN, HIGH);
                ledOnTime = millis();
                Serial.println(" -> >>> STEREO GUNSHOT CONFIRMED <<<");
              } else {
                 Serial.println(" -> REJECTED");
              }

            } else {
              Serial.printf("Done. REJECTED: Duration (%d) out of bounds.\n", consecutiveLoudChunks);
            }
            
            currentState = IDLE;
          }
          break;
      }

      if (ledOnTime > 0 && millis() - ledOnTime > ledOnDuration) {
        digitalWrite(LED_BUILTIN, LOW);
        ledOnTime = 0;
      }
    }
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, LOW);
  i2sInit();

  Serial.println("System Initializing...");

  xTaskCreatePinnedToCore(
    AudioProcessingTask,   
    "AudioTask",           
    10000,                 
    NULL,                  
    1,                     
    &AudioTaskHandle,      
    0                      
  );
  
  Serial.println("Stereo Adaptive Gunshot Detector Launched on Core 0");
}

void loop() {
  delay(1000); 
}

void i2sInit() {
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = I2S_SAMPLE_RATE,
    .bits_per_sample = I2S_BITS_PER_SAMPLE_32BIT,
    // FIX: Read both the high and low states of the WS clock
    .channel_format = I2S_CHANNEL_FMT_RIGHT_LEFT,
    .communication_format = I2S_COMM_FORMAT_STAND_I2S,
    .intr_alloc_flags = 0,
    .dma_buf_count = 8,
    // Keep this the same; we doubled the read buffer size instead
    .dma_buf_len = SAMPLES_PER_CHUNK, 
    .use_apll = false
  };
  i2s_driver_install(I2S_PORT, &i2s_config, 0, NULL);
  const i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = -1,
    .data_in_num = I2S_SD
  };
  i2s_set_pin(I2S_PORT, &pin_config);
}
