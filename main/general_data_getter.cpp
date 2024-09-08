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

    // SENSORS headers
    #include "sensors/bmp180/bmp180.h"
    #include "sensors/dht22/dht22.h"
    #include "sensors/yl69/yl69.h"

    // NVS headers
    #include "nvs_flash.h"
    #include "nvs.h"
    #include "encrypt.h"

}
#include <string>
#include <vector>
#include <sstream>
#include <fstream>
#include <cstring>

// Declare 'voltage dividir GPIO' | declare the 'ADC_PIN' | declare 'ADC_POWER_DOWN_DELAY''
#define VOLTAGE_DIVIDER_GPIO GPIO_NUM_33 // pin
#define ADC_PIN ADC_CHANNEL_0 // pin
#define ADC_POWER_DOWN_DELAY 10

// 10 - 12 KB of data into the NVS
// Flowchart
// Deep Sleep function, Check battery level function, Cryptograph data
// Handle different types of sensors (each one with a unique driver) -> use a type of menu

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

// Function to split a string by a delimiter (e.g., comma)
std::vector<std::string> splitString(const std::string &str, char delimiter) {
    std::vector<std::string> tokens;
    std::stringstream ss(str);
    std::string item;
    while (std::getline(ss, item, delimiter)) {
        tokens.push_back(item);
    }
    return tokens;
    //Output: ["sensor_1", "BMP180"]
}

// Function to read sensors from the text file
std::vector<SensorInfo> getSensorsFromFile(const char* filename) {
    std::vector<SensorInfo> sensorList;
    std::ifstream file(filename);

    if (!file.is_open()) {
        ESP_LOGE("getSensorsFromFile", "Failed to open file: %s", filename);
        return sensorList;
    }

    std::string line;
    while (std::getline(file, line)) {
        std::vector<std::string> sensorData = splitString(line, ',');
        if (sensorData.size() == 2) { // Ensure the line has all 2 components (name and type)
            SensorInfo sensor;
            sensor.id = sensorData[0];
            sensor.name = sensorData[1];
            sensorList.push_back(sensor);
        } else {
            ESP_LOGE("getSensorsFromFile", "Incorrect format in line: %s", line.c_str());
        }
    }

    file.close();
    return sensorList;

    /* Output: [
        { "sensor_1", "BMP180" },
        { "sensor_2", "DHT22" },
        ...
    ]*/
}


// just to LOG the information about the sensors
void displaySensors(const std::vector<SensorInfo>& sensors) {
    for (const auto& sensor : sensors) {
        ESP_LOGI("Sensor Info", "ID: %s, Name: %s",
                 sensor.id.c_str(), sensor.name.c_str());
    }
    // Output from log: I (xxx) Sensor Info: ID: sensor_1, Name: BMP180
}
// TXT FILE <end>

// Deep Sleep and Battery Managment
// DEEP SLEEP FUNCTION
void ESP32Base::deepSleep(const char* logTag, uint64_t time_in_us) {
    ESP_LOGI(logTag, "Entering deep sleep mode...");
    ESP_ERROR_CHECK(esp_sleep_enable_timer_wakeup(time_in_us));
    esp_deep_sleep_start();
}

float ESP32Base::checkBatteryLevel(adc_oneshot_unit_handle_t adc1_handle) {
    gpio_set_level(VOLTAGE_DIVIDER_GPIO, 1);
    vTaskDelay(pdMS_TO_TICKS(10));
    int adcValue;
    adc_oneshot_read(adc1_handle, ADC_PIN, &adcValue);
    float voltage = adcValue * (3.3 / 4095.0) * 2;
    gpio_set_level(VOLTAGE_DIVIDER_GPIO, 0);
    vTaskDelay(pdMS_TO_TICKS(ADC_POWER_DOWN_DELAY));
    float batteryLevel = map(voltage, 3.0, 4.2, 0, 100);
    if (batteryLevel > 100) batteryLevel = 100;
    else if (batteryLevel < 0) batteryLevel = 0;
    ESP_LOGI("ESP32Base", "Battery Level: %.2f%%", batteryLevel);
    batteryLevel=80;
    return batteryLevel;
}

float ESP32Base::map(float x, float in_min, float in_max, float out_min, float out_max) {
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


// it appends into the NVS(key and value)
// storeDataInNVS("temperature_sensor", 26.2);
// NVS Storage -> Key: "temperature_sensor" Value: "25.6,26.2"
void storeDataInNVS(const char* sensorName, float data) {
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("sensor_data", NVS_READWRITE, &my_handle);
    if (err != ESP_OK) {
        ESP_LOGE("NVS", "Error opening NVS");
        return;
    }

    // First, retrieve existing data for this sensor
    size_t required_size = 0;
    std::string newData;

    err = nvs_get_str(my_handle, sensorName, NULL, &required_size);
    if (err == ESP_OK && required_size > 0) {
        std::vector<char> existingData(required_size);
        err = nvs_get_str(my_handle, sensorName, existingData.data(), &required_size);
        if (err == ESP_OK) {
            newData = std::string(existingData.data()) + "," + std::to_string(data);
        }
    } else if (err == ESP_ERR_NVS_NOT_FOUND) {
        // No existing data, start fresh
        newData = std::to_string(data);
    } else {
        ESP_LOGE("NVS", "Error reading from NVS");
        nvs_close(my_handle);
        return;
    }

    // Store the updated data (append the new reading)
    err = nvs_set_str(my_handle, sensorName, newData.c_str());
    if (err != ESP_OK) {
        ESP_LOGE("NVS", "Error writing to NVS");
    }

    // Commit the changes
    err = nvs_commit(my_handle);
    if (err != ESP_OK) {
        ESP_LOGE("NVS", "Error committing to NVS");
    }

    nvs_close(my_handle);
}


void formatData(const char* sensorName, float data) {
    ESP_LOGI("SensorData", "Sensor: %s, Data: %.2f", sensorName, data);
    storeDataInNVS(sensorName, data);  // Store sensor data in NVS
}

class ESP32Base {
public:
    virtual ~ESP32Base() {}
    void deepSleep(const char* logTag, uint64_t time_in_us);
    float checkBatteryLevel(adc_oneshot_unit_handle_t adc1_handle);
    float map(float x, float in_min, float in_max, float out_min, float out_max);
    virtual void readSensorData() = 0;
};


class ESP32DerivedClass : public ESP32Base {
public:

    std::vector<std::string> bmp180Data;
    std::vector<std::string> dht22Data;
    std::vector<std::string> soilMoistureData;
    std::vector<std::string> ky037Data;

    void readSensorData() override {
        std::vector<std::string> sensorList = getSensorsFromFile("sensors.txt");
        for (const auto& sensor : sensorList) {
            ESP_LOGI("Sensor Info", "ID: %s, Name: %s", sensor.id.c_str(), sensor.name.c_str());

            if (sensor.name == "BMP180") {
                readBMP180();
            } else if (sensor.name == "DHT22") {
                readDHT22();
            } else if (sensor.name == "SoilMoisture") {
                readSoilMoisture();
            } else if (sensor.name == "KY037") {
                readKY037();
            } else {
                ESP_LOGE("ESP32DerivedClass", "Unknown sensor: %s", sensor.name.c_str());
            }
    
        }
    }

private:

    //SDA(GPIO 21 ou 22) | SCL(GPIO 22 ou 23) | VCC e GND
    void readBMP180() {
        bmp180_t sensor;
        if (bmp180_init(&sensor, I2C_NUM_0, BMP180_I2C_ADDRESS) == ESP_OK) {
            float pressure;
            bmp180_read_pressure(&sensor, &pressure);
            formatData("BMP180", pressure);
        } else {
            ESP_LOGE("BMP180", "Failed to initialize BMP180");
        }
    }
    // DHT_GPIO_PIN (choose a GPIO) | VCC e GND | 4.7k from DHT22 to VCC
    void readDHT22() {
        int16_t temperature = 0, humidity = 0;
        if (dht_read_data(DHT_TYPE_DHT22, DHT_GPIO_PIN, &humidity, &temperature) == ESP_OK) {
            float temp = temperature / 10.0;
            formatData("DHT22", temp);
        } else {
            ESP_LOGE("DHT22", "Failed to read data from DHT22");
        }
    }
    // ADC1_CHANNEL_0(GPIO 36) | VCC e GND
    void readSoilMoisture() {
        int adcValue = 0;
        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_11);
        adcValue = adc1_get_raw(ADC1_CHANNEL_0);

        float moisture = ((float)adcValue / 4095.0) * 100.0;
        formatData("SoilMoisture", moisture);
    }

    // ADC1_CHANNEL_1(GPIO 37) | VCC e GND
    void readKY037() {
        int adcValue = 0;
        adc1_config_width(ADC_WIDTH_BIT_12);
        adc1_config_channel_atten(ADC1_CHANNEL_1, ADC_ATTEN_DB_12);
        adcValue = adc1_get_raw(ADC1_CHANNEL_1);

        float soundLevel = ((float)adcValue / 4095.0) * 100.0;
        formatData("KY037", soundLevel);
    }

    void formatData(const std::string& sensorName, float sensorValue) {
    std::string formattedData = sensorName + ": " + std::to_string(sensorValue);
        // Append data to corresponding sensor vector
        if (sensorName == "BMP180") {
            bmp180Data.push_back(formattedData);
        } else if (sensorName == "DHT22") {
            dht22Data.push_back(formattedData);
        } else if (sensorName == "SoilMoisture") {
            soilMoistureData.push_back(formattedData);
        } else if (sensorName == "KY037") {
            ky037Data.push_back(formattedData);
        }

        // Save the sensor data to NVS
        saveDataToNVS(sensorName, formattedData);
    }

    void saveDataToNVS(const std::string& sensorName, const std::string& sensorData) {
        nvs_handle_t nvsHandle;
        esp_err_t err = nvs_open("sensor_storage", NVS_READWRITE, &nvsHandle);
        if (err != ESP_OK) {
            ESP_LOGE("NVS", "Failed to open NVS handle: %s", esp_err_to_name(err));
            return;
        }

        // Save sensor data
        err = nvs_set_str(nvsHandle, sensorName.c_str(), sensorData.c_str());
        if (err != ESP_OK) {
            ESP_LOGE("NVS", "Failed to write to NVS: %s", esp_err_to_name(err));
        } else {
            ESP_LOGI("NVS", "Sensor data saved: %s", sensorData.c_str());
        }

        // Commit the changes to NVS
        err = nvs_commit(nvsHandle);
        if (err != ESP_OK) {
            ESP_LOGE("NVS", "Failed to commit NVS changes: %s", esp_err_to_name(err));
        }

        // Close NVS handle
        nvs_close(nvsHandle);
    }


};

void initADC(adc_oneshot_unit_handle_t* adc1_handle) {
    adc_oneshot_unit_init_cfg_t init_config1 = {
        .unit_id = ADC_UNIT_1,
        .clk_src = ADC_RTC_CLK_SRC_DEFAULT,
        .ulp_mode = ADC_ULP_MODE_DISABLE
    };
    esp_err_t ret = adc_oneshot_new_unit(&init_config1, adc1_handle);
    if (ret != ESP_OK) {
        ESP_LOGE("initADC", "Failed to initialize ADC unit: %s", esp_err_to_name(ret));
        return;
    }
    adc_oneshot_chan_cfg_t config = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };
    ret = adc_oneshot_config_channel(*adc1_handle, ADC_CHANNEL_0, &config);
    if (ret != ESP_OK) {
        ESP_LOGE("initADC", "Failed to configure ADC channel: %s", esp_err_to_name(ret));
    }
}


// NVS and DATA functions

// no arguments
std::vector<SensorData> getAllDataFromNVS() {
    std::vector<SensorData> sensorDataList;
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("sensor_data", NVS_READONLY, &my_handle);
    if (err != ESP_OK) {
        ESP_LOGE("NVS", "Error opening NVS");
        return sensorDataList;
    }

    nvs_iterator_t it = nvs_entry_find(NVS_DEFAULT_PART_NAME, "sensor_data", NVS_TYPE_ANY, NULL);
    while (it != nullptr) {
        nvs_entry_info_t info;
        nvs_entry_info(it, &info);

        // Assuming the sensor data is stored as float values
        float sensorValue;
        err = nvs_get_float(my_handle, info.key, &sensorValue);
        if (err == ESP_OK) {
            SensorData sensorData;
            sensorData.sensorName = info.key; // Use the key as the sensor name
            sensorData.value = sensorValue;
            sensorDataList.push_back(sensorData);
        } else {
            ESP_LOGE("NVS", "Error reading sensor data for %s", info.key);
        }

        // Move to the next entry
        nvs_entry_next(&it);
    }

    nvs_close(my_handle);
    return sensorDataList;
    //std::vector<SensorData>, where each SensorData contains sensorName and value(float)
}

// argument is the output of the previous function
std::string serializeSensorData(const std::vector<SensorData>& sensorDataList) {
    std::string serializedData = "{";

    for (const auto& data : sensorDataList) {
        serializedData += "\"" + data.sensorName + "\":" + std::to_string(data.value) + ",";
    }

    // Remove the last comma and close the JSON object
    if (!sensorDataList.empty()) {
        serializedData.pop_back();
    }
    serializedData += "}";

    return serializedData;
    // output something like this {"BMP180": 1013.25, "DHT22": 24.5}
}

esp_err_t sendNVSDataToDrone() {
    // Get all data from NVS
    std::vector<SensorData> sensorDataList = getAllDataFromNVS();

    // Check if there is any data to send
    if (sensorDataList.empty()) {
        ESP_LOGI("ESP-NOW", "No data in NVS to send.");
        return ESP_ERR_NOT_FOUND;
    }

    // Serialize the sensor data
    std::string serializedData = serializeSensorData(sensorDataList);

    // Send the serialized data using ESP-NOW --> Remember the encryption using the AES
    esp_err_t err = esp_now_send(peerInfo.peer_addr, (uint8_t*)serializedData.c_str(), serializedData.length());
    if (err != ESP_OK) {
        ESP_LOGE("ESP-NOW", "Error sending data: %s", esp_err_to_name(err));
        return err;
    }

    // Erase NVS data after successful transmission
    err = eraseAllDataFromNVS();
    if (err != ESP_OK) {
        ESP_LOGE("NVS", "Error erasing data from NVS: %s", esp_err_to_name(err));
    }

    return ESP_OK;
}

// Function to erase all data stored in NVS and reset the vectors(sensors)
esp_err_t eraseAllDataFromNVS() {
    // Initialize the NVS handle
    nvs_handle_t nvsHandle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &nvsHandle);
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

    // Reset all sensor data vectors
    bmp180Data.clear();
    dht22Data.clear();
    soilMoistureData.clear();
    ky037Data.clear();

    ESP_LOGI("NVS", "All sensor vectors cleared successfully.");

    return err;
}



// ESP-NOW Connection functions

// esp_err_t tryConnectToDrone() {
//     // Initialize ESP-NOW
//     if (esp_now_init() != ESP_OK) {
//         ESP_LOGE("ESP-NOW", "ESP-NOW initialization failed");
//         return ESP_ERR_INVALID_STATE;
//     }

//     // Register the send and receive callbacks
//     esp_now_register_send_cb(onDataSent);
//     esp_now_register_recv_cb(onDataReceived);

//     // Add the peer (drone ESP32)
//     esp_now_peer_info_t peerInfo = {};
//     memcpy(peerInfo.peer_addr, DRONE_PEER_ADDR, ESP_NOW_ETH_ALEN);
//     peerInfo.channel = 0; // Use the current channel
//     peerInfo.encrypt = false; // No encryption

//     esp_err_t addPeerErr = esp_now_add_peer(&peerInfo);
//     if (addPeerErr != ESP_OK) {
//         ESP_LOGE("ESP-NOW", "Failed to add peer: %s", esp_err_to_name(addPeerErr));
//         return addPeerErr;
//     }

//     // Check if the connection is established
//     // Normally, you would use the callback functions to handle connection status updates
//     // For simplicity, we'll assume the connection attempt is successful here
//     ESP_LOGI("ESP-NOW", "Attempting to connect to drone...");

//     // Check if the connection was successful (dummy condition for illustration)
//     // Replace with actual connection check if necessary
//     bool connected = true; // Replace with actual connection status check
//     if (connected) {
//         ESP_LOGI("ESP-NOW", "Connected to drone successfully");
//         return ESP_OK;
//     } else {
//         ESP_LOGE("ESP-NOW", "Failed to connect to drone");
//         return ESP_ERR_TIMEOUT;
//     }
// }

// // Callback function for handling data sent events
// void onDataSent(const uint8_t *mac_addr, esp_now_send_status_t status) {
//     ESP_LOGI("ESP-NOW", "Data sent to %02x:%02x:%02x:%02x:%02x:%02x, status: %d",
//              mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5], status);
// }

// // Callback function for handling incoming data
// void onDataReceived(const uint8_t *mac_addr, const uint8_t *data, int len) {
//     ESP_LOGI("ESP-NOW", "Data received from %02x:%02x:%02x:%02x:%02x:%02x, length: %d",
//              mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5], len);
//     // Handle the received data here
// }

// END OF FUNCTIONS RELATED TO ESP-NOW CONNECTION
// Global variable to track connection status on the sending ESP32
//
// AVALIAR !!!
bool espNowConnected = false;

// Callback function to handle data received via ESP-NOW
void onDataReceive(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
    // Convert the data to a string
    std::string receivedMessage((char*)data, data_len);

    // Check if the message is "AUH?"
    if (receivedMessage == "AUH?") {
        ESP_LOGI(LOG_TAG, "Received 'AUH?' from: %02x:%02x:%02x:%02x:%02x:%02x",
            mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);

        // Respond with "YES"
        const char* responseMsg = "YES";
        esp_now_send(mac_addr, (uint8_t*)responseMsg, strlen(responseMsg));

        ESP_LOGI(LOG_TAG, "Responded with 'YES' to: %02x:%02x:%02x:%02x:%02x:%02x",
            mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);
    }
}

// On the master ESP32, handle the response in the callback and update the `espNowConnected` flag
void onResponseReceived(const uint8_t *mac_addr, const uint8_t *data, int data_len) {
    // Convert the data to a string
    std::string response((char*)data, data_len);

    // Check if the response is "YES"
    if (response == "YES") {
        ESP_LOGI(LOG_TAG, "Received 'YES' from: %02x:%02x:%02x:%02x:%02x:%02x",
            mac_addr[0], mac_addr[1], mac_addr[2], mac_addr[3], mac_addr[4], mac_addr[5]);

        // Mark the connection as successful
        espNowConnected = true;
    }
}
// FIM DO AVALIAR !!!

// Para verificar se há dados no NVS, pois se não há, não faz sentido mandar para o Drone.
bool hasValidDataInNVS() {
    nvs_handle_t my_handle;
    esp_err_t err = nvs_open("sensor_data", NVS_READONLY, &my_handle);
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
const char *tag = "DEEP_SLEEP";

extern "C" void app_main() {

    const char* filename = "sensors.txt";
    std::vector<SensorInfo> sensors = getSensorsFromFile(filename);

    if (!sensors.empty()) {
            displaySensors(sensors); // Display sensor info for debugging/logging
        } else {
            ESP_LOGE("app_main", "No sensors found in the file.");
    }

    adc_oneshot_unit_handle_t adc1_handle;
    initADC(&adc1_handle);

    ESP32DerivedClass esp32;

    // Configuration for deep sleep
    uint64_t deepSleepDuration_us = 10 * 1000000;  // 10 seconds in microseconds
    bool espNowConnected = false;

    while (true) {
        // Try to connect to another ESP32 via ESP-NOW

        espNowConnected = tryConnectToDrone(); // DATA RECEIVED (just check one time)

        if (espNowConnected) {
            // If connected, send all NVS data to the connected ESP32
            // It will be a loop until all data is sent and received
            esp_err_t err = sendNVSDataToDrone();
            if (err == ESP_OK) {
                // Erase NVS data after successful transmission
                eraseAllDataFromNVS();

                // Disconnect and enter deep sleep
                ESP_LOGI("ESP-NOW", "All data sent successfully and NVS clear, entering deep sleep.");

                espNowDisconnect(); // DO NOT NEED <- ESP-Now only occurs when data is sent
                esp32.deepSleep(tag, deepSleepDuration_us);
            } else {
                ESP_LOGE("ESP-NOW", "Failed to send data. Retrying...");
                // Optionally, add a delay or retry mechanism here
            }
        } else {
            // If not connected, check battery level
            float batteryLevel = esp32.checkBatteryLevel(adc1_handle);

            if (batteryLevel > 20.0) {
                // Battery level is sufficient to read sensor data
                ESP_LOGI("Main", "Battery level: %.2f%%, reading sensor data.", batteryLevel);
                esp32.readSensorData();  // This reads and stores data in NVS
            } else {
                // Battery level is below threshold, do not read sensor data
                ESP_LOGW("Main", "Battery level below 20%% (%.2f%%). Skipping sensor reading.", batteryLevel);
            }

            // Enter deep sleep
            ESP_LOGI("Main", "Entering deep sleep for %llu microseconds.", deepSleepDuration_us);
            esp32.deepSleep("SLEEP", deepSleepDuration_us);
        }

        // After waking up from deep sleep, the loop restarts
    }
}
