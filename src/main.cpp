#include <Arduino.h>
#include "Audio.h"
#include <SD.h>
#include <SPI.h>
#include <driver/i2s.h>
/*
// Configuración del I2S
#define I2S_WS 16
#define I2S_SCK 17
#define I2S_SD 15

// Configuración de la tarjeta SD
#define SD_CS 39

// Configuración del archivo de audio
#define FILE_NAME "/audio.WAV"
#define SAMPLE_RATE 16000
#define BITS_PER_SAMPLE I2S_BITS_PER_SAMPLE_16BIT
#define CHANNEL_FORMAT I2S_CHANNEL_FMT_ONLY_LEFT
#define BUFFER_SIZE 1024

void setupI2S() {
  // Configurar el pinout del I2S
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = BITS_PER_SAMPLE,
    .channel_format = CHANNEL_FORMAT,
    .communication_format = I2S_COMM_FORMAT_I2S_MSB,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = BUFFER_SIZE,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_SD
  };

  // Inicializar el I2S
  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);
  i2s_zero_dma_buffer(I2S_NUM_0);
}

void setupSD() {
  // Inicializar la tarjeta SD
  SPI.begin(36, 37, 35); //  void begin(int8_t sck=-1, int8_t miso=-1, int8_t mosi=-1, int8_t ss=-1);
  if (!SD.begin(SD_CS)) {
    Serial.println("Error al montar la tarjeta SD");
    while (true);
  }
}

void recordAudioToFile(const char* filename, int duration) {
  // Abrir el archivo para escribir
  File file = SD.open(filename, FILE_WRITE);
  if (!file) {
    Serial.println("Error al abrir el archivo para escribir");
    return;
  }

  // Capturar y guardar audio
  int64_t endTime = millis() + duration;
  size_t bytesRead;
  uint8_t buffer[BUFFER_SIZE];

  while (millis() < endTime) {
    i2s_read(I2S_NUM_0, buffer, BUFFER_SIZE, &bytesRead, portMAX_DELAY);
    file.write(buffer, bytesRead);
  }

  // Cerrar el archivo
  file.close();
}
*/
/*
String readAudioFileAndConvertToBase64(const char* filename) {
  File file = SD.open(filename, FILE_READ);
  if (!file) {
    Serial.println("Error al abrir el archivo para leer");
    return "";
  }

  size_t size = file.size();
  uint8_t* buffer = (uint8_t*)malloc(size);
  file.read(buffer, size);
  file.close();

  String base64Audio = base64::encode(buffer, size);
  free(buffer);
  return base64Audio;
}
*/
/*
void setup() {
  // Iniciar la comunicación serie
  Serial.begin(115200);
  
  // Configurar I2S
  setupI2S();

  // Configurar SD
  setupSD();

  // Grabar audio por 10 segundos y guardarlo en un archivo
  recordAudioToFile(FILE_NAME, 10000);
  Serial.println("Grabación completada y guardada en la tarjeta SD");

  // Convertir el archivo de audio a Base64
  
  // String base64Audio = readAudioFileAndConvertToBase64(FILE_NAME);
  // Serial.println("Audio en Base64: ");
  // Serial.println(base64Audio);
}

void loop() {
  // Nada que hacer aquí
}
*/
#include <Arduino.h>
#include "Audio.h"
#include <SD.h>
#include <SPI.h>
#include <driver/i2s.h>

// Configuración del I2S
#define I2S_WS 16
#define I2S_SCK 17
#define I2S_SD 15

// Configuración de la tarjeta SD
#define SD_CS 39

// Configuración del archivo de audio
#define FILE_NAME "/audio.WAV"
#define SAMPLE_RATE 16000
#define BITS_PER_SAMPLE I2S_BITS_PER_SAMPLE_16BIT
#define CHANNEL_FORMAT I2S_CHANNEL_FMT_ONLY_LEFT
#define BUFFER_SIZE 1024

void setupI2S() {
  // Configurar el pinout del I2S
  i2s_config_t i2s_config = {
    .mode = (i2s_mode_t)(I2S_MODE_MASTER | I2S_MODE_RX),
    .sample_rate = SAMPLE_RATE,
    .bits_per_sample = BITS_PER_SAMPLE,
    .channel_format = CHANNEL_FORMAT,
    .communication_format = I2S_COMM_FORMAT_I2S_MSB,
    .intr_alloc_flags = ESP_INTR_FLAG_LEVEL1,
    .dma_buf_count = 4,
    .dma_buf_len = BUFFER_SIZE,
    .use_apll = false,
    .tx_desc_auto_clear = false,
    .fixed_mclk = 0
  };

  i2s_pin_config_t pin_config = {
    .bck_io_num = I2S_SCK,
    .ws_io_num = I2S_WS,
    .data_out_num = I2S_PIN_NO_CHANGE,
    .data_in_num = I2S_SD
  };

  // Inicializar el I2S
  i2s_driver_install(I2S_NUM_0, &i2s_config, 0, NULL);
  i2s_set_pin(I2S_NUM_0, &pin_config);
  i2s_zero_dma_buffer(I2S_NUM_0);
}

void setupSD() {
  // Inicializar la tarjeta SD
  SPI.begin(36, 37, 35); //  void begin(int8_t sck=-1, int8_t miso=-1, int8_t mosi=-1, int8_t ss=-1);
  if (!SD.begin(SD_CS)) {
    Serial.println("Error al montar la tarjeta SD");
    while (true);
  }
}

void writeWavHeader(File file, int sampleRate, int bitsPerSample, int channels, int dataSize) {
  byte header[44];
  
  // Chunk ID "RIFF"
  header[0] = 'R'; header[1] = 'I'; header[2] = 'F'; header[3] = 'F';
  
  // Chunk size
  int chunkSize = dataSize + 36;
  header[4] = (byte)(chunkSize & 0xFF);
  header[5] = (byte)((chunkSize >> 8) & 0xFF);
  header[6] = (byte)((chunkSize >> 16) & 0xFF);
  header[7] = (byte)((chunkSize >> 24) & 0xFF);
  
  // Format "WAVE"
  header[8] = 'W'; header[9] = 'A'; header[10] = 'V'; header[11] = 'E';
  
  // Subchunk1 ID "fmt "
  header[12] = 'f'; header[13] = 'm'; header[14] = 't'; header[15] = ' ';
  
  // Subchunk1 size (16 for PCM)
  header[16] = 16; header[17] = 0; header[18] = 0; header[19] = 0;
  
  // Audio format (1 for PCM)
  header[20] = 1; header[21] = 0;
  
  // Number of channels
  header[22] = channels; header[23] = 0;
  
  // Sample rate
  header[24] = (byte)(sampleRate & 0xFF);
  header[25] = (byte)((sampleRate >> 8) & 0xFF);
  header[26] = (byte)((sampleRate >> 16) & 0xFF);
  header[27] = (byte)((sampleRate >> 24) & 0xFF);
  
  // Byte rate
  int byteRate = sampleRate * channels * bitsPerSample / 8;
  header[28] = (byte)(byteRate & 0xFF);
  header[29] = (byte)((byteRate >> 8) & 0xFF);
  header[30] = (byte)((byteRate >> 16) & 0xFF);
  header[31] = (byte)((byteRate >> 24) & 0xFF);
  
  // Block align
  int blockAlign = channels * bitsPerSample / 8;
  header[32] = (byte)(blockAlign & 0xFF);
  header[33] = (byte)((blockAlign >> 8) & 0xFF);
  
  // Bits per sample
  header[34] = bitsPerSample; header[35] = 0;
  
  // Subchunk2 ID "data"
  header[36] = 'd'; header[37] = 'a'; header[38] = 't'; header[39] = 'a';
  
  // Subchunk2 size
  header[40] = (byte)(dataSize & 0xFF);
  header[41] = (byte)((dataSize >> 8) & 0xFF);
  header[42] = (byte)((dataSize >> 16) & 0xFF);
  header[43] = (byte)((dataSize >> 24) & 0xFF);
  
  // Write the header to the file
  file.write(header, 44);
}

void recordAudioToFile(const char* filename, int duration) {
  // Abrir el archivo para escribir
  File file = SD.open(filename, FILE_WRITE);
  if (!file) {
    Serial.println("Error al abrir el archivo para escribir");
    return;
  }

  // Escribir el encabezado WAV (con un tamaño de datos de 0 inicialmente)
  writeWavHeader(file, SAMPLE_RATE, 16, 1, 0);

  // Capturar y guardar audio
  int64_t endTime = millis() + duration;
  size_t bytesRead;
  uint8_t buffer[BUFFER_SIZE];
  int totalBytesWritten = 0;

  while (millis() < endTime) {
    i2s_read(I2S_NUM_0, buffer, BUFFER_SIZE, &bytesRead, portMAX_DELAY);
    file.write(buffer, bytesRead);
    totalBytesWritten += bytesRead;
  }

  // Actualizar el encabezado WAV con el tamaño correcto de datos
  file.seek(0);
  writeWavHeader(file, SAMPLE_RATE, 16, 1, totalBytesWritten);

  // Cerrar el archivo
  file.close();
}

void setup() {
  // Iniciar la comunicación serie
  Serial.begin(115200);
  
  // Configurar I2S
  setupI2S();

  // Configurar SD
  setupSD();

  // Grabar audio por 10 segundos y guardarlo en un archivo
  recordAudioToFile(FILE_NAME, 10000);
  Serial.println("Grabación completada y guardada en la tarjeta SD");
}

void loop() {
  // Nada que hacer aquí
}
