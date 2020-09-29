#include <sensor_config.h>
#include <ArduinoJson.h>
#if ENABLE_AUDIO
#include <PDM.h>
static void onPDMdata();
volatile int samplesRead;
short sampleBuffer[2048];

int setup_audio(JsonDocument& config_message, int column_start)
{
  int column_index = column_start;
    PDM.onReceive(onPDMdata);
    if (!PDM.begin(1, 16000))
    {
        Serial.println("Failed to start PDM!");
        while (1);
    }

    config_message["sample_rate"] = AUDIO_SAMPLE_RATE;
    config_message["column_location"]["Microphone"] = column_index++;
    return column_index;
}

uint8_t* getSampleBuffer()
{
    return (uint8_t*)sampleBuffer;
}

static void onPDMdata() {
  // query the number of bytes available
  int bytesAvailable = PDM.available();

  // read into the sample buffer
  PDM.read(sampleBuffer, bytesAvailable);

  // 16-bit, 2 bytes per sample
  samplesRead = bytesAvailable/2;
}

#endif //ENABLE_AUDIO
