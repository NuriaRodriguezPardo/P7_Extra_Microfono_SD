# Practica 7.Extra: Guardar arxivo audio de microfono a SD
## Objetivos: 
- Crear un archivo de audio a partir de un microfono (.WAV)
- Guardar dicho archivo de audio a la targeta SD.
## Materiales: 
- ESP32-S3
- Microfono I2S
- SD
- Adaptador SD
- Lector SD 
- Librerias: 
```
    lib_deps = 
	bblanchon/ArduinoJson@^7.0.4
	esphome/ESP32-audioI2S@^2.0.7
```
## Procedimiento: 
**Codigo:**
```cpp
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
    }

```
**Descripción:**<br>
Grabar audio y lo almacena en una tarjeta SD. La configuración inicial define los pines y parámetros necesarios para el protocolo I2S, que es un estándar para la transmisión de datos de audio digitales entre dispositivos, y la tarjeta SD, que servirá como almacenamiento para el audio grabado.<br>
La función ``setupI2S`` configura el protocolo I2S, especificando parámetros como la frecuencia de muestreo, el formato de bits, y otros detalles relacionados con la configuración de los pines y el formato de comunicación. Esta función instala el controlador I2S y configura los pines necesarios para la transmisión de datos de audio.<br>
La función ``setupSD`` inicializa la tarjeta SD utilizando el bus SPI. Se especifican los pines para la comunicación SPI y se intenta montar la tarjeta SD. Si la inicialización falla, el programa imprime un mensaje de error y entra en un bucle infinito para evitar continuar sin una tarjeta SD funcional.<br>
La función ``recordAudioToFile`` se encarga de grabar el audio y guardarlo en un archivo especificado en la tarjeta SD. Abre el archivo en modo de escritura y luego entra en un bucle que dura el tiempo especificado (en este caso, 10 segundos). Durante este tiempo, el audio se lee continuamente del bus I2S y se escribe en el archivo en la tarjeta SD. Al finalizar la grabación, el archivo se cierra para asegurar que todos los datos se escriban correctamente.<br>
En la función ``setup``, se inicializa la comunicación serie a una velocidad de 115200 baudios para la depuración. Luego se llaman a setupI2S y setupSD para configurar los módulos respectivos. Finalmente, se graba el audio durante 10 segundos llamando a recordAudioToFile. La función loop está vacía, ya que todas las operaciones importantes se realizan en el setup.
### Salida puerto serie: 
```
    Grabación completada y guardada en la tarjeta SD
```
### Diagrama de flujos: 
```mermaid
    flowchart TD
        A[Inicio] --> B[Configuración Inicial]
        B --> C[Configurar I2S]
        C --> D[Configurar SD]
        D --> E[Grabar Audio]
        E --> F[Fin]

        subgraph Configuración Inicial
            B1[Definir pines y parámetros]
        end

        subgraph Configurar I2S
            C1[Definir configuración I2S]
            C2[Instalar controlador I2S]
            C3[Configurar pines I2S]
            C4[Limpiar buffer DMA]
            C1 --> C2
            C2 --> C3
            C3 --> C4
        end

        subgraph Configurar SD
            D1[Inicializar SPI]
            D2[Montar tarjeta SD]
            D1 --> D2
        end

        subgraph Grabar Audio
            E1[Abrir archivo para escritura]
            E2{¿Archivo abierto?}
            E3[Error al abrir archivo]
            E4[Capturar y guardar audio]
            E5[Cerrar archivo]
            E1 --> E2
            E2 -- No --> E3
            E2 -- Sí --> E4
            E4 --> E5
        end

        B1 --> C
        C4 --> D
        D2 --> E
        E5 --> F
```
## Conclusión: 
Graba audio mediante el protocolo I2S y lo almacena en una tarjeta SD.