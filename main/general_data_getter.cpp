extern "C" {
    #include "freertos/FreeRTOS.h"
    #include "freertos/task.h"
    #include "esp_sleep.h"
    #include "esp_log.h"
    #include "esp_adc/adc_oneshot.h"
    #include "esp_now.h"
    #include "esp_wifi.h"
    #include "esp_netif.h"
    #include "esp_event.h"
    #include "driver/gpio.h"
    #include "esp_system.h"
    #include "hal/adc_types.h"
    #include "esp_system.h"  // Add this at the top of your file with other includes

    // SENSORS headers
    #include "sensors/bmp180/bmp180.h"
    #include "sensors/dht11/dht11.h"
    #include "sensors/yl69/yl69.h"
    #include "sensors/acoustic/acoustic.h"

    // NVS headers
    #include "nvs_flash.h"
    #include "nvs.h"
    #include "encrypt.h"
    #include "driver/i2c_master.h"
    #include <inttypes.h>  // Add this to the top of your file
    #include "esp_timer.h"
    #include "cJSON.h"
    #include "encrypt.h"

}

#include <string>
#include <vector>
#include <sstream>
#include <fstream>
#include <cstring>
#include <unordered_map>
#include <queue>
#include <mutex>
#include <cstdlib>       // For strtof
#include <random>

// Global variables
// Queue to hold data chunks to be sent
std::queue<std::vector<uint8_t>> dataQueue;
std::mutex queueMutex;
bool sendingInProgress = false;
bool auhProcessed = false;
uint8_t recipientMacAddr[ESP_NOW_ETH_ALEN];

// Declare 'voltage dividir GPIO' | declare the 'ADC_PIN' | declare 'ADC_POWER_DOWN_DELAY''
#define VOLTAGE_DIVIDER_GPIO GPIO_NUM_33 // pin
#define ADC_PIN ADC_CHANNEL_0 // pin
#define ADC_POWER_DOWN_DELAY 10
#define WIFI_CHANNEL 1   // Default Wi-Fi channel for ESP-NOW
#define LOG_TAG_ESP_NOW "ESP-NOW-DATA_GETTER"

// BMP180
#define I2C_NUM_0 0
#define BMP180_ADDRESS 0x77

// DHT22
#define DHT_TYPE_DHT22 22
#define DHT_GPIO_PIN GPIO_NUM_4

// records into the NVS
#define MAX_RECORDS 150  // Define the maximum number of records to store
#define RECORD_KEY_PREFIX "record_"  // Prefix for NVS keys
#define NVS_INDEX_KEY "nvs_index"  // Key for storing the current write index

const char* LOG_TAG = "DATA-GETTER";

// Define message types as enums for better clarity
enum MessageType {
    MSG_AUH = 'A',
    MSG_YES = 'Y',
    MSG_START = 'S',
    MSG_DONE = 'D',
    MSG_ENC = 'E'
};

// Maximum payload size for ESP-NOW
const size_t MAX_PAYLOAD_SIZE = 250;

adc_oneshot_unit_handle_t adc_handle = NULL;

bool awaitingResponse = false;
bool auhReceived = false;
bool espNowMessageReceived = false;

void onESPNowSend(const uint8_t *mac_addr, esp_now_send_status_t status);
void onResponseReceived(const esp_now_recv_info_t *recv_info, const uint8_t *data, int len);

void initNVS() {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        // NVS partition was truncated or has a new version, erase it and initialize again
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
}

void initADC() {
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
        // Initialize other members if necessary
        .clk_src = ADC_RTC_CLK_SRC_DEFAULT, // Initialize missing fields
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle));

    adc_oneshot_chan_cfg_t chan_config = {
        .atten = ADC_ATTEN_DB_12, // Replace deprecated ADC_ATTEN_DB_11
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_0, &chan_config));
}

void initWiFi() {
    ESP_LOGI(LOG_TAG_ESP_NOW, "Initializing Wi-Fi...");

    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_err_t ret = esp_wifi_init(&cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(LOG_TAG_ESP_NOW, "Failed to initialize Wi-Fi: %s", esp_err_to_name(ret));
        return;
    }

    ret = esp_wifi_set_mode(WIFI_MODE_STA);
    if (ret != ESP_OK) {
        ESP_LOGE(LOG_TAG_ESP_NOW, "Failed to set Wi-Fi mode: %s", esp_err_to_name(ret));
        return;
    }

    ret = esp_wifi_start();
    if (ret != ESP_OK) {
        ESP_LOGE(LOG_TAG_ESP_NOW, "Failed to start Wi-Fi: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(LOG_TAG_ESP_NOW, "Wi-Fi initialized in station mode.");
}

void initESPNow() {
    esp_err_t ret = esp_now_init();
    if (ret != ESP_OK) {
        ESP_LOGE(LOG_TAG_ESP_NOW, "ESP-NOW initialization failed: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(LOG_TAG_ESP_NOW, "ESP-NOW initialized successfully.");

    ret = esp_now_register_send_cb(onESPNowSend);
    if (ret != ESP_OK) {
        ESP_LOGE(LOG_TAG_ESP_NOW, "Failed to register send callback: %s", esp_err_to_name(ret));
        return;
    }

    ret = esp_now_register_recv_cb(onResponseReceived);  // or onDataReceive
    if (ret != ESP_OK) {
        ESP_LOGE(LOG_TAG_ESP_NOW, "Failed to register receive callback: %s", esp_err_to_name(ret));
        return;
    }

    ESP_LOGI(LOG_TAG_ESP_NOW, "ESP-NOW setup complete.");
}



// TXT FILE <start>
// Structure to hold sensor data
struct SensorInfo {
    std::string id;
    std::string name;
};

struct SensorData {
    std::string sensorName; // Name of the sensor
    float value;            // The data value from the sensor
};
// Timer handles
esp_timer_handle_t auhWaitTimer = NULL;
bool auhWaitTimerExpired = false;
void auhWaitTimerCallback(void* arg) {
    auhWaitTimerExpired = true;
    ESP_LOGW(LOG_TAG, "AUH? wait timer expired.");
}

void startAUHWaitTimer() {
    auhWaitTimerExpired = false;

    // Create and start the timer
    esp_timer_create_args_t timer_args = {
        .callback = &auhWaitTimerCallback,
        .arg = NULL,
        .dispatch_method = ESP_TIMER_TASK,
        .name = "auhWaitTimer",
        .skip_unhandled_events = false  // Initialize to false explicitly
    };

    ESP_ERROR_CHECK(esp_timer_create(&timer_args, &auhWaitTimer));
    ESP_ERROR_CHECK(esp_timer_start_once(auhWaitTimer, 5000000));  // Wait for 5 seconds (in microseconds)
}

// Para verificar se há dados no NVS, pois se não há, não faz sentido mandar para o Drone.

const char *tag = "DEEP_SLEEP"; 

// Function to initialize Wi-Fi in station mode (required for ESP-NOW)


void onESPNowSend(const uint8_t *mac_addr, esp_now_send_status_t status) {
    if (status == ESP_NOW_SEND_SUCCESS) {
        ESP_LOGI("ESP-NOW", "Send success to %02X:%02X:%02X:%02X:%02X:%02X",
                 mac_addr[0], mac_addr[1], mac_addr[2],
                 mac_addr[3], mac_addr[4], mac_addr[5]);
    } else {
        ESP_LOGE("ESP-NOW", "Send fail to %02X:%02X:%02X:%02X:%02X:%02X",
                 mac_addr[0], mac_addr[1], mac_addr[2],
                 mac_addr[3], mac_addr[4], mac_addr[5]);
    }
}

// Main initialization function for ESP-NOW

// 10 - 12 KB of data into the NVS
// Flowchart
// Deep Sleep function, Check battery level function, Cryptograph data
// Handle different types of sensors (each one with a unique driver) -> use a type of menu


std::vector<SensorInfo> getSensorsList() {
    std::vector<SensorInfo> sensorList = {
        {"sensor_1", "BMP180"},
        {"sensor_2", "DHT11"},
        {"sensor_3", "SoilMoisture"},
        //{"sensor_4", "KY037"}
    };
    return sensorList;
}

// ? DEBUG FUNCTION
// just to LOG the information about the sensors
void displaySensors(const std::vector<SensorInfo>& sensors) {
    for (const auto& sensor : sensors) {
        ESP_LOGI("Sensor Info", "ID: %s, Name: %s",
                 sensor.id.c_str(), sensor.name.c_str());
    }
    // Output from log: I (xxx) Sensor Info: ID: sensor_1, Name: BMP180
}
// TXT FILE <end>

class ESP32Base {
public:
    virtual ~ESP32Base() {}
    void deepSleep(const char* logTag, uint64_t time_in_us);
    float checkBatteryLevel(adc_oneshot_unit_handle_t adc1_handle);
    float map(float x, float in_min, float in_max, float out_min, float out_max);
    virtual void readSensorData() = 0;
};


// Deep Sleep and Battery Managment
// DEEP SLEEP FUNCTION
void ESP32Base::deepSleep(const char* logTag, uint64_t time_in_us) {
    ESP_LOGI(logTag, "Entering deep sleep mode...");
    ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup(time_in_us));
    esp_deep_sleep_start();
}


// ! IT NEEDS TO BE FIXED
float ESP32Base::checkBatteryLevel(adc_oneshot_unit_handle_t adc1_handle) {
    int adcValue;
    esp_err_t ret = adc_oneshot_read(adc1_handle, ADC_CHANNEL_0, &adcValue);
    if (ret != ESP_OK) {
        ESP_LOGE("ADC", "Failed to read ADC value: %s", esp_err_to_name(ret));
        return -1;
    }
    gpio_set_level(VOLTAGE_DIVIDER_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    float voltage = adcValue * (3.3 / 4095.0) * 2;
    gpio_set_level(VOLTAGE_DIVIDER_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(ADC_POWER_DOWN_DELAY));
    float batteryLevel = map(voltage, 3.0, 4.2, 0, 100);
    if (batteryLevel > 100) batteryLevel = 100;
    else if (batteryLevel < 0) batteryLevel = 0;
    batteryLevel=80;
    ESP_LOGI("ESP32Base", "Battery Level: %.2f%%", batteryLevel);
    return batteryLevel;
}

float ESP32Base::map(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


class ESP32DerivedClass : public ESP32Base {
public:
    void readSensorData() override;
    void formatData(const std::string& sensorName, float sensorValue);
    void saveDataToNVS(const std::string& sensorName, const std::string& sensorData);
    void readAllDataFromNVS();  
private:
    // Static member variables
    static std::vector<std::string> bmp180Data;
    static std::vector<std::string> dht11Data;
    static std::vector<std::string> soilMoistureData;
    static std::vector<std::string> ky037Data;

    // Sensor read functions
    void readBMP180();
    void readDHT11();
    void readSoilMoisture();
    void readKY037();
};

std::vector<std::string> ESP32DerivedClass::bmp180Data;
std::vector<std::string> ESP32DerivedClass::dht11Data;
std::vector<std::string> ESP32DerivedClass::soilMoistureData;
std::vector<std::string> ESP32DerivedClass::ky037Data;

void ESP32DerivedClass::readSensorData() {
    //std::vector<SensorInfo> sensorList = getSensorsFromFile("sensors.txt"); 
    std::vector<SensorInfo> sensorList = getSensorsList();
    for (const auto& sensor : sensorList) {
        ESP_LOGI("Sensor Info", "ID: %s, Name: %s", sensor.id.c_str(), sensor.name.c_str());

        if (sensor.name == "BMP180") {
            readBMP180();
        } else if (sensor.name == "DHT11") {
            readDHT11();
        } else if (sensor.name == "SoilMoisture") {
            readSoilMoisture();
        } else if (sensor.name == "KY037") {
            readKY037();
        } else {
            ESP_LOGE("ESP32DerivedClass", "Unknown sensor: %s", sensor.name.c_str());
        }

    }
}

//SDA(GPIO 21 ou 22) | SCL(GPIO 22 ou 23) | VCC e GND
void ESP32DerivedClass::readBMP180() {
    float temperature;
    uint32_t pressure;

    // Reinitialize every time after waking from deep sleep
    if (bmp180_init(GPIO_NUM_21, GPIO_NUM_22) == ESP_OK) {
        // Read temperature
        if (bmp180_read_temperature(&temperature) == ESP_OK) {
            formatData("BMP180_T", temperature);
            ESP_LOGI("BMP180", "Temperature in Celsius");

        } else {
            ESP_LOGE("BMP180", "Failed to read temperature");
        }

        // Read pressure
        if (bmp180_read_pressure(&pressure) == ESP_OK) {
            formatData("BMP180_P", pressure);
            ESP_LOGI("BMP180", "Barometric pressure in Pascal, normal atm is 1000 to 1020 hPa(hectopascal)");

        } else {
            ESP_LOGE("BMP180", "Failed to read pressure");
        }
    } else {
        ESP_LOGE("BMP180", "Failed to initialize BMP180");
    }   
}

// DHT_GPIO_PIN (choose a GPIO) | VCC e GND | 4.7k from DHT22 to VCC
// DHT_GPIO_PIN (choose a GPIO) | VCC e GND | 4.7k from DHT22 to VCC
void ESP32DerivedClass::readDHT11() {
    // Initialize DHT11 with the specified GPIO pin
    DHT11_init(DHT_GPIO_PIN);

    // Delay to allow the sensor to stabilize
    vTaskDelay(pdMS_TO_TICKS(2000));

    // Read data from the DHT11 sensor
    struct dht11_reading result = DHT11_read();

    // Check the status of the reading
    if (result.status == DHT11_OK) {
        // Successful read
        float temp = result.temperature;
        float hum = result.humidity;

        // Format and log the data
        formatData("DHT11_T", temp);
        formatData("DHT11_H", hum);
        ESP_LOGI("DHT11", "Temperature: %.1f°C, Humidity: %.1f%%", temp, hum);
    } else if (result.status == DHT11_TIMEOUT_ERROR) {
        // Timeout error
        ESP_LOGE("DHT11", "Timeout error reading from DHT11 sensor.");
    } else if (result.status == DHT11_CRC_ERROR) {
        // Checksum error
        ESP_LOGE("DHT11", "Checksum error reading from DHT11 sensor.");
    } else {
        // Unknown error
        ESP_LOGE("DHT11", "Unknown error reading from DHT11 sensor.");
    }
}

// ADC1_CHANNEL_0(GPIO 36) | VCC e GND
void ESP32DerivedClass::readSoilMoisture() {
    yl69_setup(ADC_CHANNEL_0);

    uint32_t adcValue = yl69_read(adc_handle);

    uint32_t moisture = yl69_normalization(adcValue);

    formatData("SoilMoisture", static_cast<float>(moisture));
}

/* PARA O MICROFONE QUE VAI CAPTURAR O SOM*/
// ADC1_CHANNEL_1(GPIO 37) | VCC e GND
// driver's functions to read the digital output. Only one GPIO
void ESP32DerivedClass::readKY037() {
    // Initialize the acoustic sensor (if not already initialized)
    init_acoustic(false);

    // Get the sound level ("High" or "Low")
    char* soundLevelStr = get_acoustic();

    // Convert to numerical value
    float soundLevel = (strcmp(soundLevelStr, "High") == 0) ? 100.0f : 0.0f;

    // Format and send the data
    formatData("KY037", soundLevel);
}

void ESP32DerivedClass::formatData(const std::string& sensorName, float sensorValue) {
    std::string formattedData = sensorName + ": " + std::to_string(sensorValue);
    // Append data to corresponding sensor vector
    if (sensorName == "BMP180") {
        bmp180Data.push_back(formattedData);
    } else if (sensorName == "DHT11") {
        dht11Data.push_back(formattedData);
    } else if (sensorName == "SoilMoisture") {
        soilMoistureData.push_back(formattedData);
    } else if (sensorName == "KY037") {
        ky037Data.push_back(formattedData);
    }

    // Save the sensor data to NVS
    saveDataToNVS(sensorName, formattedData);
}

void ESP32DerivedClass::saveDataToNVS(const std::string& sensorName, const std::string& newSensorData) {
    // Define dataToSave, which will be the data we actually save
    std::string dataToSave = newSensorData;

    // Parse newSensorData to extract the sensor value
    size_t colonPos = newSensorData.find(":");
    if (colonPos != std::string::npos) {
        std::string valueStr = newSensorData.substr(colonPos + 1);
        // Trim whitespace
        valueStr.erase(0, valueStr.find_first_not_of(" \t"));
        valueStr.erase(valueStr.find_last_not_of(" \t") + 1);

        // Use strtof instead of std::stof to avoid exceptions
        char* endPtr = nullptr;
        float sensorValue = strtof(valueStr.c_str(), &endPtr);

        if (endPtr == valueStr.c_str() || *endPtr != '\0') {
            // Conversion failed
            ESP_LOGE("saveDataToNVS", "Error parsing sensor value: %s", valueStr.c_str());
            // Proceed with original newSensorData
        } else {
            // Decide whether to modify the sensor value (e.g., 10% chance)
            bool modifyValue = false;

            // Generate a random number between 0 and 99 using C++ <random>
            static std::random_device rd;                   // Seed
            static std::mt19937 gen(rd());                  // Mersenne Twister generator
            static std::uniform_int_distribution<> distr(0, 99); // Range 0 to 99

            uint32_t randomNumber = distr(gen);  // Generate random number
            if (randomNumber < 10) {              // 10% chance
                modifyValue = true;
            }

            if (modifyValue) {
                // Modify the sensor value to be significantly different
                float modifiedValue = sensorValue * 10.0f;  // For example, multiply by 10
                // Reconstruct dataToSave with the modified value
                dataToSave = sensorName + ": " + std::to_string(modifiedValue);

                ESP_LOGI("saveDataToNVS", "Anomaly introduced. Modified %s value from %.2f to %.2f",
                         sensorName.c_str(), sensorValue, modifiedValue);
            }
        }
    } else {
        ESP_LOGW("saveDataToNVS", "Invalid data format: %s", newSensorData.c_str());
        // Proceed with original newSensorData
    }

    // Proceed to save dataToSave to NVS
    nvs_handle_t nvsHandle;
    esp_err_t err = nvs_open("sensor_storage", NVS_READWRITE, &nvsHandle);
    if (err != ESP_OK) {
        ESP_LOGE("NVS", "Failed to open NVS handle: %s", esp_err_to_name(err));
        return;
    }

    int32_t currentIndex = 0;
    err = nvs_get_i32(nvsHandle, NVS_INDEX_KEY, &currentIndex);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        currentIndex = 0;  // No previous data, start at 0
    }

    char key[32];
    snprintf(key, sizeof(key), "%s%" PRIi32, RECORD_KEY_PREFIX, currentIndex);  // Generate unique key

    err = nvs_set_str(nvsHandle, key, dataToSave.c_str());  // Save dataToSave instead of newSensorData
    if (err != ESP_OK) {
        ESP_LOGE("NVS", "Failed to write to NVS: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI("NVS", "Sensor data saved at key: %s", key);
    }

    // Increment index and save it
    currentIndex = (currentIndex + 1) % MAX_RECORDS;
    err = nvs_set_i32(nvsHandle, NVS_INDEX_KEY, currentIndex);
    if (err != ESP_OK) {
        ESP_LOGE("NVS", "Failed to update NVS index: %s", esp_err_to_name(err));
    }

    err = nvs_commit(nvsHandle);  // Commit to ensure data is stored
    if (err != ESP_OK) {
        ESP_LOGE("NVS", "Error committing NVS: %s", esp_err_to_name(err));
    }

    nvs_close(nvsHandle);
}


// ! TEMPORARY
void ESP32DerivedClass::readAllDataFromNVS() {
    nvs_handle_t nvsHandle;
    esp_err_t err = nvs_open("sensor_storage", NVS_READONLY, &nvsHandle);
    if (err != ESP_OK) {
        ESP_LOGE("NVS", "Failed to open NVS handle for reading: %s", esp_err_to_name(err));
        return;
    }

    // Loop through stored records
    // Threshold defined by me as the max records
    for (int32_t i = 0; i < MAX_RECORDS; i++) {
        char key[32];
        snprintf(key, sizeof(key), "%s%" PRIi32, RECORD_KEY_PREFIX, i);  // Generate key

        // Get the string associated with the key
        size_t required_size = 0;
        err = nvs_get_str(nvsHandle, key, NULL, &required_size);
        if (err == ESP_OK) {
            std::vector<char> value_buf(required_size);
            err = nvs_get_str(nvsHandle, key, value_buf.data(), &required_size);
            if (err == ESP_OK) {
                ESP_LOGI("NVS_READ", "Key: %s, Value: %s", key, value_buf.data());
            } else {
                ESP_LOGE("NVS_READ", "Failed to read value for key %s", key);
            }
        } else if (err == ESP_ERR_NVS_NOT_FOUND) {
            ESP_LOGI("NVS_READ", "Key %s not found, probably empty.", key);
        } else {
            ESP_LOGE("NVS_READ", "Error getting string for key %s: %s", key, esp_err_to_name(err));
        }
    }

    nvs_close(nvsHandle);
}

// NVS and DATA functions

// no arguments
// ! IMPORTANT FUNCTION 
std::vector<SensorData> getAllDataFromNVS() {
    std::vector<SensorData> sensorDataList;
    nvs_iterator_t it = NULL;
    esp_err_t err = nvs_entry_find(NVS_DEFAULT_PART_NAME, "sensor_storage", NVS_TYPE_STR, &it);
    if (err != ESP_OK) {
        ESP_LOGE("NVS", "Error finding NVS entries");
        return sensorDataList;
    }

    nvs_handle_t my_handle;
    err = nvs_open("sensor_storage", NVS_READONLY, &my_handle);  // Open NVS once
    if (err != ESP_OK) {
        ESP_LOGE("NVS", "Error opening NVS handle");
        return sensorDataList;
    }

    while (it != NULL) {
        nvs_entry_info_t info;
        nvs_entry_info(it, &info);

        // Read the value (a string like "DHT11_T: 28.000000")
        size_t required_size = 0;
        err = nvs_get_str(my_handle, info.key, NULL, &required_size);
        if (err == ESP_OK) {
            std::vector<char> value_buf(required_size);
            err = nvs_get_str(my_handle, info.key, value_buf.data(), &required_size);
            if (err == ESP_OK) {
                std::string sensorEntry = value_buf.data();
                ESP_LOGI("NVS_READ", "Retrieved sensor entry: %s", sensorEntry.c_str());
                
                // Split into sensorName and sensorValue
                size_t colonPos = sensorEntry.find(": ");
                if (colonPos == std::string::npos) {
                    ESP_LOGE("NVS", "Invalid format in NVS for key %s: %s", info.key, sensorEntry.c_str());
                    continue;  // Skip invalid entries
                }

                std::string sensorName = sensorEntry.substr(0, colonPos);  // Extract sensor name
                std::string sensorValueStr = sensorEntry.substr(colonPos + 2);  // Extract sensor value

                // Convert the sensorValueStr to a float
                // ! It will only accept number values
                char* end;
                float sensorValue = strtof(sensorValueStr.c_str(), &end);
                if (end == sensorValueStr.c_str() || *end != '\0') {
                    ESP_LOGE("NVS", "Invalid float value in NVS for key %s: %s", info.key, sensorValueStr.c_str());
                    continue;  // Skip invalid values
                }

                // Add to sensor data list
                SensorData sensorData;
                sensorData.sensorName = sensorName;
                sensorData.value = sensorValue;
                sensorDataList.push_back(sensorData);
            } else {
                ESP_LOGE("NVS", "Error reading value for key %s", info.key);
            }
        } else {
            ESP_LOGE("NVS", "Error getting string size for key %s", info.key);
        }

        nvs_entry_next(&it);  // Move to the next entry
    }

    nvs_close(my_handle);  // Close NVS once after the loop
    return sensorDataList;
}

/*
Example of the data in the queue to transmit:

1. "START"
2. [{"sensorName":"Temp
3. erature","value":23.5
4. },{"sensorName":"Hum
5. idity","value":45.2}
6. ,{"sensorName":"Pre
7. ssure","value":1013.
8. 25}]
9. "DONE"

*/

#define MAX_PAYLOAD_SIZE 250

esp_err_t sendNVSDataToDrone(const uint8_t* macAddress) {
    std::vector<SensorData> sensorDataList = getAllDataFromNVS();
    if (sensorDataList.empty()) {
        ESP_LOGI(LOG_TAG, "No data in NVS to send.");
        return ESP_ERR_NOT_FOUND;
    }

    // Create a cJSON array to hold all sensor data
    cJSON *jsonArray = cJSON_CreateArray();
    if (jsonArray == NULL) {
        ESP_LOGE(LOG_TAG, "Failed to create cJSON array.");
        return ESP_ERR_NO_MEM;
    }

    // Add each sensor data as a cJSON object
    for (const auto& sensorData : sensorDataList) {
        cJSON *jsonObject = cJSON_CreateObject();
        if (jsonObject == NULL) {
            ESP_LOGE(LOG_TAG, "Failed to create cJSON object.");
            cJSON_Delete(jsonArray);
            return ESP_ERR_NO_MEM;
        }
        cJSON_AddStringToObject(jsonObject, "sensorName", sensorData.sensorName.c_str());
        cJSON_AddNumberToObject(jsonObject, "value", sensorData.value);
        cJSON_AddItemToArray(jsonArray, jsonObject);
    }

    // Convert the JSON array to a string
    char *serializedJson = cJSON_PrintUnformatted(jsonArray);
    cJSON_Delete(jsonArray);  // Free the JSON array memory
    if (serializedJson == NULL) {
        ESP_LOGE(LOG_TAG, "Failed to serialize JSON data.");
        return ESP_ERR_NO_MEM;
    }

    ESP_LOGI("Serialized Data", "Data to send: %s", serializedJson);

    size_t dataLength = strlen(serializedJson);
    size_t offset = 0;

    {
        std::lock_guard<std::mutex> lock(queueMutex);

        // **Enqueue unencrypted "START" message**
        dataQueue.push(std::vector<uint8_t>{MSG_START});

        // Split data into chunks, encrypt, and enqueue them
        size_t max_payload_size = MAX_PAYLOAD_SIZE;  // ESP-NOW payload limit
        size_t max_chunk_size = max_payload_size - 1 - 16;  // Reserve space for MSG_ENC and padding

        while (offset < dataLength) {
            size_t remaining = dataLength - offset;
            size_t chunkSize = (remaining >= max_chunk_size) ? max_chunk_size : remaining;

            // Adjust chunkSize to be a multiple of 16 for AES
            if (chunkSize % 16 != 0) {
                chunkSize += 16 - (chunkSize % 16);
                if (offset + chunkSize > dataLength) {
                    chunkSize = dataLength - offset;
                }
            }

            std::vector<uint8_t> dataChunk(serializedJson + offset, serializedJson + offset + chunkSize);

            size_t encrypted_len;
            uint8_t* encrypted_data = my_aes_encrypt(dataChunk.data(), dataChunk.size(), &encrypted_len);
            if (encrypted_data == NULL) {
                ESP_LOGE(LOG_TAG, "Encryption failed for data chunk.");
                free(serializedJson);
                return ESP_FAIL;
            }

            // Convert the encrypted data into a vector and then free the raw pointer
            std::vector<uint8_t> encryptedChunk(encrypted_data, encrypted_data + encrypted_len);
            free(encrypted_data);  // Free the memory allocated by aes_encrypt

            // Prepend the message type
            encryptedChunk.insert(encryptedChunk.begin(), MSG_ENC);
            dataQueue.push(encryptedChunk);

            ESP_LOGI(LOG_TAG, "Encrypted data chunk enqueued. Size: %d bytes", encryptedChunk.size());

            offset += chunkSize;
        }

        // **Enqueue unencrypted "DONE" message**
        dataQueue.push(std::vector<uint8_t>{MSG_DONE});
    }

    free(serializedJson);  // Free the serialized string memory
    ESP_LOGI(LOG_TAG, "Data chunks queued for transmission.");

    // Start sending if not already in progress
    {
        std::lock_guard<std::mutex> lock(queueMutex);
        if (!sendingInProgress && !dataQueue.empty()) {
            std::vector<uint8_t> firstChunk = dataQueue.front();
            dataQueue.pop();
            esp_err_t result = esp_now_send(macAddress, firstChunk.data(), firstChunk.size());
            if (result == ESP_OK) {
                ESP_LOGI(LOG_TAG, "First chunk sent.");
                sendingInProgress = true;
            } else {
                ESP_LOGE(LOG_TAG, "Error sending first chunk: %s", esp_err_to_name(result));
                sendingInProgress = false;  // Reset flag on error
                return result;
            }
        }
    }

    return ESP_OK;
}




// Function to erase all data stored in NVS and reset the vectors(sensors)
esp_err_t eraseAllDataFromNVS() {
    // Initialize the NVS handle
    nvs_handle_t nvsHandle;
    esp_err_t err = nvs_open("sensor_storage", NVS_READWRITE, &nvsHandle);  // Use correct namespace
    if (err != ESP_OK) {
        ESP_LOGE("NVS", "Failed to open NVS handle: %s", esp_err_to_name(err));
        return err;
    }

    // Erase all data in the NVS namespace
    err = nvs_erase_all(nvsHandle);
    if (err != ESP_OK) {
        ESP_LOGE("NVS", "Failed to erase NVS data: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI("NVS", "All NVS data erased successfully.");
    }

    // Commit the changes to ensure the data is erased
    err = nvs_commit(nvsHandle);
    if (err != ESP_OK) {
        ESP_LOGE("NVS", "Failed to commit NVS erase: %s", esp_err_to_name(err));
    }

    // Close the NVS handle
    nvs_close(nvsHandle);

    ESP_LOGI("NVS", "NVS erase and commit completed.");

    return err;
}
bool hasValidDataInNVS() {
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("sensor_storage", NVS_READONLY, &my_handle);
    if (err != ESP_OK) {
        ESP_LOGE("NVS", "Error opening NVS");
        return false;
    }

    std::vector<SensorData> sensorDataList = getAllDataFromNVS();
    if (sensorDataList.empty()) {
        ESP_LOGI("NVS", "No valid sensor data found.");
        nvs_close(my_handle);
        return false;
    }

    nvs_close(my_handle);
    return true;

}

// Global variable to track connection status on the sending ESP32
//
// AVALIAR !!!
bool espNowConnected = false;

void onResponseReceived(const esp_now_recv_info_t *recv_info, const uint8_t *data, int data_len) {
    const uint8_t *mac_addr = recv_info->src_addr;

    if (data_len < 1) {
        ESP_LOGW(LOG_TAG, "Received empty data.");
        return;
    }

    uint8_t messageType = data[0];
    std::vector<uint8_t> messageData(data + 1, data + data_len);

    ESP_LOGI(LOG_TAG, "Received message type: %c from: %02X:%02X:%02X:%02X:%02X:%02X",
             messageType,
             mac_addr[0], mac_addr[1], mac_addr[2],
             mac_addr[3], mac_addr[4], mac_addr[5]);

    switch (messageType) {
        case MSG_AUH:
            if (!auhProcessed) {
                ESP_LOGI(LOG_TAG, "Received 'AUH?' from the drone. Processing...");

                // Store the recipient's MAC address
                memcpy(recipientMacAddr, mac_addr, ESP_NOW_ETH_ALEN);

                // Add the drone as a peer in the Sender ESP
                esp_now_peer_info_t peerInfo = {};
                memcpy(peerInfo.peer_addr, recipientMacAddr, ESP_NOW_ETH_ALEN);
                peerInfo.channel = 0;  // Use current channel
                peerInfo.encrypt = false;

                if (!esp_now_is_peer_exist(recipientMacAddr)) {
                    esp_err_t result = esp_now_add_peer(&peerInfo);
                    if (result != ESP_OK) {
                        ESP_LOGE(LOG_TAG, "Failed to add peer: %s", esp_err_to_name(result));
                        return;  // Exit if adding peer fails
                    } else {
                        ESP_LOGI(LOG_TAG, "Peer added successfully.");
                    }
                }

                // Prepare the 'YES' response
                std::vector<uint8_t> response = {'Y', 'E', 'S'};

                // Send the 'YES' response back via ESP-NOW
                esp_err_t result = esp_now_send(recipientMacAddr, response.data(), response.size());
                if (result == ESP_OK) {
                    ESP_LOGI(LOG_TAG, "Sent 'YES' response to the drone.");

                    // Set the flag to indicate that 'AUH?' has been processed
                    auhProcessed = true;

                    // Add a small delay to ensure the drone is ready
                    vTaskDelay(pdMS_TO_TICKS(100));  // Wait for 100 milliseconds

                    // Now proceed to send the data to the drone
                    if (hasValidDataInNVS()) {
                        // Send data to the drone
                        esp_err_t err = sendNVSDataToDrone(recipientMacAddr);
                        if (err == ESP_OK) {
                            // Erase NVS data after successful transmission
                            eraseAllDataFromNVS();
                            ESP_LOGI(LOG_TAG, "Data sent successfully to the drone.");
                        } else {
                            ESP_LOGE(LOG_TAG, "Failed to send data to the drone.");
                        }
                    } else {
                        ESP_LOGW(LOG_TAG, "No valid data in NVS to send.");
                    }
                } else {
                    ESP_LOGE(LOG_TAG, "Failed to send 'YES' response: %s", esp_err_to_name(result));
                }
            } else {
                ESP_LOGI(LOG_TAG, "Received 'AUH?' again from the drone. Ignoring as it's already processed.");
            }
            break;

        case MSG_YES:
            // Handle 'YES' response if necessary
            ESP_LOGI(LOG_TAG, "Received 'YES' from drone.");
            break;

        case MSG_START:
            ESP_LOGI(LOG_TAG, "Received 'START' from drone.");
            break;

        case MSG_DONE:
            ESP_LOGI(LOG_TAG, "Received 'DONE' from drone.");
            // Optionally, mark the transmission as complete
            sendingInProgress = false;
            break;

        case MSG_ENC:
        {
            size_t decrypted_len;  // Declare decrypted_len
            uint8_t* decrypted_data = my_aes_decrypt(messageData.data(), messageData.size(), &decrypted_len);
            if (decrypted_data != NULL) {
                std::vector<uint8_t> decryptedData(decrypted_data, decrypted_data + decrypted_len);
                free(decrypted_data);  // Free the allocated memory
                // Process decryptedData
            } else {
                ESP_LOGE(LOG_TAG, "Failed to decrypt received data.");
            }
        }
        break;

        default:
            ESP_LOGW(LOG_TAG, "Received unknown message type: %c", messageType);
            break;
    }
}



void on_data_sent(const uint8_t *mac_addr, esp_now_send_status_t status) {
    std::lock_guard<std::mutex> lock(queueMutex);
    if (status == ESP_NOW_SEND_SUCCESS) {
        ESP_LOGI(LOG_TAG, "Data sent successfully.");
        // If there is more data in the queue, send the next chunk
        if (!dataQueue.empty()) {
            std::vector<uint8_t> nextChunk = dataQueue.front();
            dataQueue.pop();

            // Determine message type for logging
            char msgType = nextChunk[0];
            if (msgType == MSG_DONE) {
                ESP_LOGI(LOG_TAG, "Sending 'DONE' message to receiver.");
            } else if (msgType == MSG_START) {
                ESP_LOGI(LOG_TAG, "Sending 'START' message to receiver.");
            } else if (msgType == MSG_ENC) {
                ESP_LOGI(LOG_TAG, "Sending encrypted data chunk.");
            } else {
                ESP_LOGI(LOG_TAG, "Sending unknown type message.");
            }

            // Send the next chunk
            esp_err_t result = esp_now_send(recipientMacAddr, nextChunk.data(), nextChunk.size());
            if (result != ESP_OK) {
                ESP_LOGE(LOG_TAG, "Error sending next data chunk: %s", esp_err_to_name(result));
                sendingInProgress = false;
            } else {
                // Continue sending
                if (msgType == MSG_DONE) {
                    ESP_LOGI(LOG_TAG, "'DONE' message sent. Transmission complete.");
                }
            }
        } else {
            // No more data to send
            sendingInProgress = false;
            ESP_LOGI(LOG_TAG, "All data chunks have been sent.");
        }
    } else {
        ESP_LOGE(LOG_TAG, "Data send failed.");
        sendingInProgress = false;
    }
}



// FIM DO AVALIAR !!!


extern "C" void app_main() {
    // Initialize NVS
    initNVS();
    initADC();
    ESP32DerivedClass esp32;

    // Initialize ESP-NOW
    initWiFi();
    initESPNow();

    // Get sensors list
    std::vector<SensorInfo> sensors = getSensorsList();
    if (!sensors.empty()) {
        displaySensors(sensors);
    } else {
        ESP_LOGE("app_main", "No sensors found in the file.");
    }


    // Configuration for deep sleep
    uint64_t deepSleepDuration_us = 10 * 1000000;  // 10 seconds in microseconds

    // Register the receive callback
    esp_now_register_recv_cb(onResponseReceived);
    esp_now_register_send_cb(on_data_sent);
    // Start AUH? wait timer
    startAUHWaitTimer();

    ESP_LOGI("Main", "Waiting for 'AUH?' message...");

    // Wait until AUH? is received or timer expires
    while (!auhReceived && !auhWaitTimerExpired) { 
        vTaskDelay(pdMS_TO_TICKS(100));  // Check every 100 ms
    }

    if (!auhReceived) {
        ESP_LOGI("Main", "'AUH?' not received. Proceeding to check battery and read sensors.");

        // Check battery level
        float batteryLevel = esp32.checkBatteryLevel(adc_handle);
        if (batteryLevel > 20.0) {
            ESP_LOGI("Main", "Battery level sufficient. Reading sensor data.");
            esp32.readSensorData();
            ESP_LOGI("Main", "Time to display");
            vTaskDelay(pdMS_TO_TICKS(3000));  // Check every 100 ms
            esp32.readAllDataFromNVS();
        } else {
            ESP_LOGW("Main", "Battery level low. Skipping sensor reading.");
        }
    } else {
        ESP_LOGI("Main", "Data sent to drone after receiving 'AUH?'.");
    }

    // Enter deep sleep
    ESP_LOGI("Main", "Entering deep sleep for %llu microseconds.", deepSleepDuration_us);
    esp32.deepSleep("SLEEP", deepSleepDuration_us);
}
