/*
 * BluetoothHelper.cpp
 * Copyright (C) 2019-2021 Linar Yusupov
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
// #define ESP32
#if defined(ESP32)

#include "Platform_ESP32.h"
#include "SoCHelper.h"
#include "EEPROMHelper.h"
#include "BluetoothHelper.h"
#include "NMEAHelper.h"
#include "GDL90Helper.h"

#include "SkyView.h"

#if !defined(CONFIG_BT_ENABLED) || !defined(CONFIG_BLUEDROID_ENABLED)
#error Bluetooth is not enabled!
#endif

// #include <BluetoothSerial.h>

#include <BLEDevice.h>

#include "WiFiHelper.h"   // HOSTNAME
#include <BLEUtils.h>
#include <BLEScan.h>
#include <BLEAdvertisedDevice.h>
#include <FS.h>
#include <SPIFFS.h>
#include <set>

std::vector<String> scanResults;

#include <set>

static bool bleInitializedForScan = false;

std::vector<String> scanForBLEDevices(uint32_t scanTimeSeconds) {
  scanResults.clear();

  // Prevent scanning while a connection is active
  if (ESP32_BT_ctl.status == BT_STATUS_CON) {
    Serial.println("[BLE] Scan aborted: BLE already connected.");
    return scanResults;
  }

  // Initialize BLE only once for scanning context
  if (!bleInitializedForScan) {
    BLEDevice::init("");  // Empty name
    bleInitializedForScan = true;
  }

  BLEScan* scanner = BLEDevice::getScan();
  scanner->setActiveScan(true);
  scanner->setInterval(1349);
  scanner->setWindow(449);
  scanner->clearResults();  // Free memory from last scan

  Serial.println("[BLE] Starting scan...");
  BLEScanResults results = scanner->start(scanTimeSeconds, false);
  Serial.printf("[BLE] Scan completed: %d device(s) found.\n", results.getCount());

  std::set<String> uniqueNames;

  for (int i = 0; i < results.getCount(); i++) {
    BLEAdvertisedDevice device = results.getDevice(i);
    if (device.haveName()) {
      String name = device.getName().c_str();
      name.trim();
      if (name.length() > 0) {
        uniqueNames.insert(name);
      }
    }
  }

  for (const auto& name : uniqueNames) {
    scanResults.push_back(name);
    Serial.println("[BLE] Found: " + name);
  }

  scanner->clearResults();  // Free memory again

  return scanResults;
}



std::vector<String> allowedBLENames;

void loadAllowedBLENames() {
  allowedBLENames.clear();
  if (!SPIFFS.begin(true)) {
    Serial.println("Failed to mount SPIFFS");
    return;
  }

  File file = SPIFFS.open("/BLEConnections.txt");
  if (!file) {
    Serial.println("BLEConnections.txt not found");
    return;
  }

  while (file.available()) {
    String name = file.readStringUntil('\n');
    name.trim();
    if (name.length() > 0) {
      allowedBLENames.push_back(name);
    }
  }

  file.close();
}


// BluetoothSerial SerialBT;
String BT_name = HOSTNAME;

Bluetooth_ctl_t ESP32_BT_ctl = {
  .mutex   = portMUX_INITIALIZER_UNLOCKED,
  .command = BT_CMD_NONE,
  .status  = BT_STATUS_NC
};

/* LE */
static BLERemoteCharacteristic* pRemoteCharacteristic;
static BLEAdvertisedDevice* AppDevice;
static BLEClient* pClient;

static BLEUUID  serviceUUID(SERVICE_UUID);
static BLEUUID  charUUID(CHARACTERISTIC_UUID);

cbuf *BLE_FIFO_RX, *BLE_FIFO_TX;

static unsigned long BT_TimeMarker = 0;
static unsigned long BLE_Notify_TimeMarker = 0;

static void AppNotifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {
    if (length > 0) {
      BLE_FIFO_RX->write((char *) pData, (BLE_FIFO_RX->room() > length ?
                                          length : BLE_FIFO_RX->room()));
    }
}

class AppClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {
  }

  void onDisconnect(BLEClient* pclient) {
    ESP32_BT_ctl.status = BT_STATUS_NC;

    Serial.println(F("BLE: disconnected from Server."));
  }
};

class AppAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {

  void onResult(BLEAdvertisedDevice advertisedDevice) override {
    if (!advertisedDevice.haveName()) return;

    String devName = advertisedDevice.getName().c_str();

    bool known = std::any_of(
      allowedBLENames.begin(), allowedBLENames.end(),
      [&](const String& name) { return name == devName; }
    );

    if (known && advertisedDevice.haveServiceUUID() &&
        advertisedDevice.isAdvertisingService(serviceUUID)) {

      BLEDevice::getScan()->stop();

      if (AppDevice) {
        AppDevice->~BLEAdvertisedDevice();
      }

      AppDevice = new BLEAdvertisedDevice(advertisedDevice);
      ESP32_BT_ctl.command = BT_CMD_CONNECT;
    }
  }

};


static bool ESP32_BLEConnectToServer() {
  if (!pClient->connect(AppDevice)) {
    Serial.println(F("BLE: Failed to connect to device."));
    return false;
  }

  BLERemoteService* pRemoteService = pClient->getService(serviceUUID);
  if (pRemoteService == nullptr) {
    Serial.print(F("BLE: Failed to find our service UUID: "));
    Serial.println(serviceUUID.toString().c_str());
    pClient->disconnect();
    return false;
  }

  pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
  if (pRemoteCharacteristic == nullptr) {
    Serial.print(F("BLE: Failed to find our characteristic UUID: "));
    Serial.println(charUUID.toString().c_str());
    pClient->disconnect();
    return false;
  }

    if(pRemoteCharacteristic->canNotify())
      pRemoteCharacteristic->registerForNotify(AppNotifyCallback);

    ESP32_BT_ctl.status = BT_STATUS_CON;
    return true;
}

static void ESP32_Bluetooth_setup(){
  switch (settings->connection) {

  case CON_BLUETOOTH_LE:
    {
      loadAllowedBLENames();

      BLE_FIFO_RX = new cbuf(BLE_FIFO_RX_SIZE);
      BLE_FIFO_TX = new cbuf(BLE_FIFO_TX_SIZE);

      esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);

      BLEDevice::init("");

      pClient = BLEDevice::createClient();
      pClient->setClientCallbacks(new AppClientCallback());

      BLEScan* pBLEScan = BLEDevice::getScan();
      pBLEScan->setAdvertisedDeviceCallbacks(new AppAdvertisedDeviceCallbacks());
      pBLEScan->setInterval(1349);
      pBLEScan->setWindow(449);
      pBLEScan->setActiveScan(true);
      pBLEScan->start(3, false);

      BLE_Notify_TimeMarker = millis();
    }
    break;
  default:
    break;
  }
}

static void ESP32_Bluetooth_loop()
{

  bool hasData = false;

  switch(settings->connection)
  {
  case CON_BLUETOOTH_LE:
    {
      if (ESP32_BT_ctl.command == BT_CMD_CONNECT) {
        if (ESP32_BLEConnectToServer()) {
          Serial.println(F("BLE: connected to Server."));
        }
        ESP32_BT_ctl.command = BT_CMD_NONE;
      }

      switch (settings->protocol)
      {
      case PROTOCOL_GDL90:
        hasData = GDL90_isConnected();
        break;
      case PROTOCOL_NMEA:
      default:
        hasData = NMEA_isConnected();
        break;
      }

      if (hasData) {
        BT_TimeMarker = millis();
      } else if (millis() - BT_TimeMarker > BT_NODATA_TIMEOUT) {

        Serial.println(F("BLE: attempt to (re)connect..."));

        if (pClient) {
          if (pClient->isConnected()) {
            pClient->disconnect();
          }
        }

        BLEDevice::getScan()->start(3, false);

#if 0
        /* approx. 170 bytes memory leak still remains */
        Serial.print("Free Heap: ");
        Serial.println(ESP.getFreeHeap());
#endif

        BT_TimeMarker = millis();
      }

      // notify changed value
      // bluetooth stack will go into congestion, if too many packets are sent
      if (ESP32_BT_ctl.status == BT_STATUS_CON &&
          (millis() - BLE_Notify_TimeMarker > 10)) { /* < 18000 baud */

          uint8_t chunk[BLE_MAX_WRITE_CHUNK_SIZE];
          size_t size = (BLE_FIFO_TX->available() < BLE_MAX_WRITE_CHUNK_SIZE ?
                         BLE_FIFO_TX->available() : BLE_MAX_WRITE_CHUNK_SIZE);

          if (size > 0) {
            BLE_FIFO_TX->read((char *) chunk, size);

            pRemoteCharacteristic->writeValue(chunk, size);

            BLE_Notify_TimeMarker = millis();
          }
      }
    }
    break;
  default:
    break;
  }
}

static void ESP32_Bluetooth_fini()
{
  switch(settings->connection)
  {
  case CON_BLUETOOTH_LE:
    {
      BLEDevice::deinit();
    }
    break;
  default:
    break;
  }
}

static int ESP32_Bluetooth_available()
{
  int rval = 0;

  switch(settings->connection)
  {
  case CON_BLUETOOTH_LE:
    {
      rval = BLE_FIFO_RX->available();
    }
    break;
  default:
    break;
  }

  return rval;
}

static int ESP32_Bluetooth_read()
{
  int rval = -1;

  switch(settings->connection)
  {
  case CON_BLUETOOTH_LE:
    {
      rval = BLE_FIFO_RX->read();
      break;
    }
    break;
  default:
    break;
  }

  return rval;
}

static size_t ESP32_Bluetooth_write(const uint8_t *buffer, size_t size)
{
  int rval = 0;

  switch(settings->connection)
  {
  case CON_BLUETOOTH_LE:
    {
      rval = BLE_FIFO_TX->write((char *) buffer,
                          (BLE_FIFO_TX->room() > size ? size : BLE_FIFO_TX->room()));
    }
    break;
  default:
    break;
  }

  return rval;
}

Bluetooth_ops_t ESP32_Bluetooth_ops = {
  "ESP32 Bluetooth",
  ESP32_Bluetooth_setup,
  ESP32_Bluetooth_loop,
  ESP32_Bluetooth_fini,
  ESP32_Bluetooth_available,
  ESP32_Bluetooth_read,
  ESP32_Bluetooth_write
};

#endif /* ESP32 */
