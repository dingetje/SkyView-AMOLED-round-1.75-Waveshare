/*
 * WebHelper.cpp
 * Copyright (C) 2016-2022 Linar Yusupov
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

#include <Arduino.h>
#include <TimeLib.h>
// #include "SPIFFS.h"
#include "SoCHelper.h"
#include "WebHelper.h"
#include "TrafficHelper.h"
#include "NMEAHelper.h"
#include "BatteryHelper.h"
#include "GDL90Helper.h"
#include "BuddyHelper.h"
#include "BluetoothHelper.h"
#include <ArduinoJson.h>  
#include "esp_private/esp_clk.h"
#include "esp_private/pm_impl.h"
#include <DebugLog.h>

#define NOLOGO

// static uint32_t prev_rx_pkt_cnt = 0;
extern buddy_info_t buddies[21];

#if !defined(NOLOGO)
static const char Logo[] PROGMEM = {
#include "Logo.h"
    } ;
#endif

#include "jquery_min_js.h"

int get_cpu_frequency_mhz() 
{
  rtc_cpu_freq_config_t conf;
  rtc_clk_cpu_freq_get_config(&conf);
  return conf.freq_mhz;
}

byte getVal(char c)
{
  if(c >= '0' && c <= '9')
  {
    return (byte)(c - '0');
  }
  return (byte)(toupper(c)-'A'+10);
}

#if DEBUG
void Hex2Bin(String str, byte *buffer)
{
  char hexdata[2 * PKT_SIZE + 1];
  
  str.toCharArray(hexdata, sizeof(hexdata));
  for(int j = 0; j < PKT_SIZE * 2 ; j+=2)
  {
    buffer[j>>1] = getVal(hexdata[j+1]) + (getVal(hexdata[j]) << 4);
  }
}
#endif

static const char about_html[] PROGMEM = "<html>\
  <head>\
    <meta name='viewport' content='width=device-width, initial-scale=1'>\
    <title>About</title>\
  </head>\
<body>\
<h1 align=center>About</h1>\
<p>This firmware is a part of SoftRF project</p>\
<p>URL: http://github.com/lyusupov/SoftRF</p>\
<p>Author: <b>Linar Yusupov</b></p>\
<p>This version: https://github.com/moshe-braner/SoftRF</p>\
<p>E-mail: mo\
she.braner\
@gmail.com</p>\
<h2 align=center>Credits</h2>\
<p align=center>(in historical order)</p>\
<table width=100%%>\
<tr><th align=left>Ivan Grokhotkov</th><td align=left>Arduino core for ESP8266</td></tr>\
<tr><th align=left>Paul Stoffregen</th><td align=left>Arduino Time library</td></tr>\
<tr><th align=left>Mikal Hart</th><td align=left>TinyGPS++ and PString libraries</td></tr>\
<tr><th align=left>Hristo Gochkov</th><td align=left>Arduino core for ESP32</td></tr>\
<tr><th align=left>JS Foundation</th><td align=left>jQuery library</td></tr>\
<tr><th align=left>Mike McCauley</th><td align=left>BCM2835 C library</td></tr>\
<tr><th align=left>Jean-Marc Zingg</th><td align=left>GxEPD2 library</td></tr>\
<tr><th align=left>Adafruit Industries</th><td align=left>SSD1306 and GFX libraries</td></tr>\
<tr><th align=left>Ryan David</th><td align=left>GDL90 decoder</td></tr>\
<tr><th align=left>Arundale Ramanathan</th><td align=left>uCDB Arduino library</td></tr>\
<tr><th align=left>FlarmNet<br>GliderNet</th><td align=left>aircrafts data</td></tr>\
<tr><th align=left>Shenzhen Xin Yuan<br>(LilyGO) ET company</th><td align=left>TTGO T5S V1.9 board</td></tr>\
<tr><th align=left>Tuan Nha</th><td align=left>ESP32 I2S WAV player</td></tr>\
<tr><th align=left>Brian Park</th><td align=left>AceButton library</td></tr>\
<tr><th align=left>flashrom.org project</th><td align=left>Flashrom library</td></tr>\
<tr><th align=left>Evandro Copercini</th><td align=left>ESP32 BT SPP library</td></tr>\
</table>\
<hr>\
Copyright (C) 2019-2022 &nbsp;&nbsp;&nbsp; Linar Yusupov\
</body>\
</html>";

#include <vector>
#include <FS.h>
#include <SPIFFS.h>

void writeBatteryLog(float startVoltage, float endVoltage, unsigned long durationSeconds, float estimated_mAh) 
{
  PRINTLN("writeBatteryLog: logging battery usage...");
  File file = SPIFFS.open("/battery_log.txt", FILE_APPEND);
  if (!file) 
  {
    PRINTLN("Failed to open battery_log.txt for appending");
    return;
  }

  file.printf("%.2f,%.2f,%lu,%.1f\n",
              startVoltage, endVoltage, durationSeconds, estimated_mAh);
  file.close();
}

String getLastBatteryLogEntry() 
{
  File file = SPIFFS.open("/battery_log.txt", FILE_READ);
  if (!file) return "No battery log found.";

  String lastLine = "";
  while (file.available()) 
  {
    String line = file.readStringUntil('\n');
    if (line.length() > 5) lastLine = line;
  }

  file.close();
  return lastLine;
}

String formatBatteryLog(String logLine) 
{
  int idx = 0;
  String parts[4];
  while (logLine.length() && idx < 4) 
  {
    int comma = logLine.indexOf(',');
    if (comma == -1) 
    {
      parts[idx++] = logLine;
      break;
    }
    parts[idx++] = logLine.substring(0, comma);
    logLine = logLine.substring(comma + 1);
  }

  if (idx < 4) return F("Incomplete log entry");

  String result;
  result.reserve(100);  // Optional: reserve memory to reduce heap fragmentation

  result += F("Start: ");
  result += parts[0];
  result += F("V, End: ");
  result += parts[1];
  result += F("V, Duration: ");
  result += parts[2];
  result += F(" sec, Used: ");
  result += parts[3];
  result += F(" mAh");

  return result;
}


std::vector<String> getAllowedBLENameList() 
{
  std::vector<String> allowedNames;

  // SPIFFS should already be mounted at this point
  if (SPIFFS_is_mounted == false)
  {
    PRINTLN("[BLE] SPIFFS not mounted");
    return allowedNames;
  }
  File file = SPIFFS.open("/BLEConnections.txt", "r");
  if (!file || file.isDirectory()) 
  {
    PRINTLN("[BLE] No allowed BLE file found.");
    return allowedNames;
  }

  while (file.available()) 
  {
    String line = file.readStringUntil('\n');
    line.trim();  // Remove whitespace, including \r
    if (line.length() > 0) 
    {
      allowedNames.push_back(line);
    }
  }

  file.close();
  return allowedNames;
}

void handleBLEScan() 
{
  std::vector<String> foundDevices = scanForBLEDevices(3);
  DynamicJsonDocument doc(1024);
  JsonArray array = doc.to<JsonArray>();
  for (const auto& name : foundDevices) 
  {
    array.add(name);
  }
  String json;
  serializeJson(doc, json);
  server.send(200, "application/json", json);
}

void handleAddBLEDevice() 
{
  if (!server.hasArg("name")) 
  {
    server.send(400, "text/plain", "Missing name");
    return;
  }

  String newName = server.arg("name");
  File file = SPIFFS.open("/BLEConnections.txt", FILE_APPEND);
  file.println(newName);
  file.close();

  loadAllowedBLENames(); // refresh in memory
  server.send(200, "text/plain", "Added");
}

void handleDeleteBLEDevice() 
{
  if (!server.hasArg("name")) 
  {
    server.send(400, "text/plain", "Missing name");
    return;
  }

  String toDelete = server.arg("name");
  std::vector<String> updatedList;
  auto allowedBLENames = getAllowedBLENameList();
  for (const auto& name : allowedBLENames) 
  {
    if (name != toDelete) updatedList.push_back(name);
  }

  File file = SPIFFS.open("/BLEConnections.txt", FILE_WRITE);
  for (const auto& name : updatedList) 
  {
    file.println(name);
  }
  file.close();

  loadAllowedBLENames();
  server.send(200, "text/plain", "Deleted");
}

const char* charging_status_string(int status_code) 
{
  switch (status_code) {
    case 0: return "Not Charging";
    case 1: return "Pre-Charge";
    case 2: return "Fast Charging";
    case 3: return "Charge Done";
    default: return "Unknown";
  }
}

const char* bleManagerHTML = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <title>BLE Device Manager</title>
  <meta charset="UTF-8">
  <meta name="viewport" content="width=device-width, initial-scale=1">

  <style>
    body { font-family: sans-serif; padding: 1em; }
    h2 { margin-top: 1.5em; }
    button { margin: 0.25em; padding: 0.5em 1em; }
    .device { display: flex; align-items: center; margin: 0.25em 0; }
    .device span { flex-grow: 1; }
  </style>
</head>
<body>
  <h1>BLE Device Manager</h1>

  <button onclick="scanDevices()">🔍 Scan BLE Devices</button>

  <h2>Scanned Devices</h2>
  <div id="scan-results">Click "Scan" to discover nearby devices...</div>

  <h2>Allowed Devices</h2>
  <div id="allowed-list">Loading...</div>

  <script>
    async function scanDevices() {
      const scanDiv = document.getElementById("scan-results");
      scanDiv.innerHTML = "Scanning...";

      try {
        const res = await fetch("/ble/scan");
        const names = await res.json();

        if (!names.length) {
          scanDiv.innerHTML = "No devices found.";
          return;
        }

        scanDiv.innerHTML = "";
        names.forEach(name => {
          const el = document.createElement("div");
          el.className = "device";
          el.innerHTML = `<span>${name}</span> 
                          <button onclick="addDevice('${name}')">➕ Add</button>`;
          scanDiv.appendChild(el);
        });
      } catch (e) {
        scanDiv.innerHTML = "Error scanning: " + e;
      }
    }

    async function loadAllowedDevices() {
      const listDiv = document.getElementById("allowed-list");
      listDiv.innerHTML = "Loading...";

      try {
        const res = await fetch("/ble/allowed");
        const names = await res.json();

        if (!names.length) {
          listDiv.innerHTML = "No devices in allowed list.";
          return;
        }

        listDiv.innerHTML = "";
        names.forEach(name => {
          const el = document.createElement("div");
          el.className = "device";
          el.innerHTML = `<span>${name}</span> 
                          <button onclick="removeDevice('${name}')">🗑️ Remove</button>`;
          listDiv.appendChild(el);
        });
      } catch (e) {
        listDiv.innerHTML = "Error loading list: " + e;
      }
    }

    async function addDevice(name) {
      await fetch("/ble/add?name=" + encodeURIComponent(name), { method: "POST" });
      await loadAllowedDevices();
    }

    async function removeDevice(name) {
      await fetch("/ble/delete?name=" + encodeURIComponent(name), { method: "POST" });
      await loadAllowedDevices();
    }

    // Load allowed list on page load
    loadAllowedDevices();
  </script>
</body>
</html>
)rawliteral";

void handleSettings() {

  size_t size = 5000;
  char *offset;
  size_t len = 0;
  char *Settings_temp = (char *) malloc(size);

  if (Settings_temp == NULL) {
    return;
  }

  offset = Settings_temp;

  /* Common part 1 */
  snprintf_P ( offset, size,
    PSTR("<html>\
<head>\
<meta name='viewport' content='width=device-width, initial-scale=1'>\
<title>Settings</title>\
</head>\
<body>\
<h1 align=center>Settings</h1>\
<form action='/input' method='GET'>\
<table width=100%%>"));

  len = strlen(offset);
  offset += len;
  size -= len;

  /* SoC specific part 1 */
  if (SoC->id == SOC_ESP32) {
    snprintf_P ( offset, size,
      PSTR("\
<tr>\
<th align=left>Display adapter</th>\
<td align=right>\
<select name='adapter'>"));
  len = strlen(offset);
  offset += len;
  size -= len;

#if not defined(WAVESHARE_AMOLED_1_75)
    snprintf_P ( offset, size,
      PSTR("\
<option %s value='%d'>Display TTGO T5S</option>\
<option %s value='%d'>e-Paper TTGO T5 4.7</option>\
<option %s value='%d'>e-Paper Waveshare ESP32</option>\
<option %s value='%d'>OLED</option>"),
  (settings->adapter == ADAPTER_TTGO_T5S        ? "selected" : ""), ADAPTER_TTGO_T5S,
  (settings->adapter == ADAPTER_TTGO_T5_4_7     ? "selected" : ""), ADAPTER_TTGO_T5_4_7,
  (settings->adapter == ADAPTER_WAVESHARE_ESP32 ? "selected" : ""), ADAPTER_WAVESHARE_ESP32,
  (settings->adapter == ADAPTER_OLED            ? "selected" : ""), ADAPTER_OLED
      );
#else
    snprintf_P ( offset, size,
      PSTR("\
<option %s value='%d'>Waveshare Display AMOLED 1.75</option>"),
  (settings->adapter == ADAPTER_WAVESHARE_AMOLED_1_75 ? "selected" : ""), ADAPTER_WAVESHARE_AMOLED_1_75
      );
#endif

    len = strlen(offset);
    offset += len;
    size -= len;

    snprintf_P ( offset, size,
      PSTR("\
</select>\
</td>\
</tr>"));
  }
  else if (SoC->id == SOC_ESP8266) {
    snprintf_P ( offset, size,
      PSTR("\
<tr>\
<th align=left>Display adapter</th>\
<td align=right>\
<select name='adapter'>\
<option %s value='%d'>NodeMCU</option>\
<option %s value='%d'>e-Paper Waveshare ESP8266</option>\
</select>\
</td>\
</tr>"),
    (settings->adapter == ADAPTER_NODEMCU           ? "selected" : ""), ADAPTER_NODEMCU,
    (settings->adapter == ADAPTER_WAVESHARE_ESP8266 ? "selected" : ""), ADAPTER_WAVESHARE_ESP8266
    );
  }

  len = strlen(offset);
  offset += len;
  size -= len;

  /* Common part 2 */
  snprintf_P ( offset, size,
    PSTR("\
<tr>\
<th align=left>Connection type</th>\
<td align=right>\
<select name='connection'>\
<option %s value='%d'>Serial</option>\
<option %s value='%d'>WiFi UDP</option>\
<!-- option %s value='%d'>Bluetooth SPP</option -->\
<option %s value='%d'>Bluetooth LE</option>\
<!-- option %s value='%d'>NMEA File</option -->\
</select>\
</td>\
</tr>\
<tr>\
<th align=left>Protocol</th>\
<td align=right>\
<select name='protocol'>\
<option %s value='%d'>NMEA</option>\
<option %s value='%d'>GDL90</option>\
</select>\
</td>\
</tr>\
<tr>\
<th align=left>Baud rate</th>\
<td align=right>\
<select name='baudrate'>\
<option %s value='%d'>4800</option>\
<option %s value='%d'>9600</option>\
<option %s value='%d'>19200</option>\
<option %s value='%d'>38400</option>\
<option %s value='%d'>57600</option>"),
  (settings->connection == CON_SERIAL        ? "selected" : ""), CON_SERIAL,
  (settings->connection == CON_WIFI_UDP      ? "selected" : ""), CON_WIFI_UDP,
  (settings->connection == CON_BLUETOOTH_SPP ? "selected" : ""), CON_BLUETOOTH_SPP,
  (settings->connection == CON_BLUETOOTH_LE  ? "selected" : ""), CON_BLUETOOTH_LE,
  (settings->connection == CON_FILE          ? "selected" : ""), CON_FILE,
  (settings->protocol   == PROTOCOL_NMEA     ? "selected" : ""), PROTOCOL_NMEA,
  (settings->protocol   == PROTOCOL_GDL90    ? "selected" : ""), PROTOCOL_GDL90,
  (settings->baudrate   == B4800             ? "selected" : ""), B4800,
  (settings->baudrate   == B9600             ? "selected" : ""), B9600,
  (settings->baudrate   == B19200            ? "selected" : ""), B19200,
  (settings->baudrate   == B38400            ? "selected" : ""), B38400,
  (settings->baudrate   == B57600            ? "selected" : ""), B57600
  );

  len = strlen(offset);
  offset += len;
  size -= len;

  /* SoC specific part 2 */
  if (SoC->id == SOC_ESP32) {
    snprintf_P ( offset, size,
      PSTR("\
<option %s value='%d'>115200</option>\
<option %s value='%d'>2000000</option>"),
    (settings->baudrate   == B115200        ? "selected" : ""), B115200,
    (settings->baudrate   == B2000000       ? "selected" : ""), B2000000
    );
    len = strlen(offset);
    offset += len;
    size -= len;
  }

    /* Common part 3 */
  snprintf_P ( offset, size,
    PSTR("\
</select>\
</td>\
</tr>\
<tr>\
<th align=left>Source Id</th>\
<td align=right>\
<INPUT type='text' name='server' maxlength='21' size='21' value='%s'>\
</td>\
</tr>\
<tr>\
<th align=left>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;\
&nbsp;&nbsp;Key</th>\
<td align=right>\
<INPUT type='text' name='key' maxlength='17' size='17' value='%s'>\
</td>\
</tr>\
<tr><th align=left>&nbsp;</th><td align=right>&nbsp;</td></tr>\
<tr>\
<th align=left>Bridge Output</th>\
<td align=right>\
<select name='bridge'>\
<option %s value='%d'>None</option>\
<option %s value='%d'>Serial</option>\
<option %s value='%d'>WiFi UDP</option>\
<option %s value='%d'>Bluetooth SPP</option>\
<option %s value='%d'>Bluetooth LE</option>\
</select>\
</td>\
</tr>\
<tr><th align=left>&nbsp;</th><td align=right>&nbsp;</td></tr>\
<tr>\
<th align=left>Units</th>\
<td align=right>\
<select name='units'>\
<option %s value='%d'>metric</option>\
<option %s value='%d'>imperial</option>\
<option %s value='%d'>mixed</option>\
</select>\
</td>\
</tr>\
<tr>\
<th align=left>View mode</th>\
<td align=right>\
<select name='vmode'>\
<option %s value='%d'>radar</option>\
<option %s value='%d'>text</option>\
</select>\
</td>\
</tr>\
<tr>\
<th align=left>Radar orientation</th>\
<td align=right>\
<select name='orientation'>\
<option %s value='%d'>Track Up</option>\
<option %s value='%d'>North Up</option>\
</select>\
</td>\
</tr>\
<tr>\
<th align=left>Zoom level</th>\
<td align=right>\
<select name='zoom'>\
<option %s value='%d'>lowest</option>\
<option %s value='%d'>low</option>\
<option %s value='%d'>medium</option>\
<option %s value='%d'>high</option>\
</select>\
</td>\
</tr>\
<tr>\
<th align=left>Aircrafts data</th>\
<td align=right>\
<select name='adb'>\
<option %s value='%d'>off</option>\
<!-- <option %s value='%d'>auto</option> -->\
<!-- <option %s value='%d'>FlarmNet</option> -->\
<option %s value='%d'>GliderNet (OGN)</option>\
<!-- <option %s value='%d'>ICAO</option> -->\
</select>\
</td>\
</tr>\
<tr>\
<th align=left>ID preference</th>\
<td align=right>\
<select name='idpref'>\
<option %s value='%d'>registration</option>\
<option %s value='%d'>tail/CN</option>\
<option %s value='%d'>make & model</option>\
</select>\
</td>\
</tr>"),
  settings->server, settings->key,
  (settings->bridge == BRIDGE_NONE    ? "selected" : ""), BRIDGE_NONE,
  (settings->bridge == BRIDGE_SERIAL  ? "selected" : ""), BRIDGE_SERIAL,
  (settings->bridge == BRIDGE_UDP     ? "selected" : ""), BRIDGE_UDP,
  (settings->bridge == BRIDGE_BT_SPP  ? "selected" : ""), BRIDGE_BT_SPP,
  (settings->bridge == BRIDGE_BT_LE   ? "selected" : ""), BRIDGE_BT_LE,
  (settings->units == UNITS_METRIC    ? "selected" : ""), UNITS_METRIC,
  (settings->units == UNITS_IMPERIAL  ? "selected" : ""), UNITS_IMPERIAL,
  (settings->units == UNITS_MIXED     ? "selected" : ""), UNITS_MIXED,
  (settings->vmode == VIEW_MODE_RADAR ? "selected" : ""), VIEW_MODE_RADAR,
  (settings->vmode == VIEW_MODE_TEXT  ? "selected" : ""), VIEW_MODE_TEXT,
  (settings->orientation == DIRECTION_TRACK_UP ? "selected" : ""), DIRECTION_TRACK_UP,
  (settings->orientation == DIRECTION_NORTH_UP ? "selected" : ""), DIRECTION_NORTH_UP,
  (settings->zoom == ZOOM_LOWEST ? "selected" : ""), ZOOM_LOWEST,
  (settings->zoom == ZOOM_LOW    ? "selected" : ""), ZOOM_LOW,
  (settings->zoom == ZOOM_MEDIUM ? "selected" : ""), ZOOM_MEDIUM,
  (settings->zoom == ZOOM_HIGH   ? "selected" : ""), ZOOM_HIGH,
  (settings->adb == DB_NONE      ? "selected" : ""), DB_NONE,
  (settings->adb == DB_AUTO      ? "selected" : ""), DB_AUTO,
  (settings->adb == DB_FLN       ? "selected" : ""), DB_FLN,
  (settings->adb == DB_OGN       ? "selected" : ""), DB_OGN,
  (settings->adb == DB_ICAO      ? "selected" : ""), DB_ICAO,
  (settings->idpref == ID_REG    ? "selected" : ""), ID_REG,
  (settings->idpref == ID_TAIL   ? "selected" : ""), ID_TAIL,
  (settings->idpref == ID_MAM    ? "selected" : ""), ID_MAM
  );

  len = strlen(offset);
  offset += len;
  size -= len;

  /* SoC specific part 3 */
  if (SoC->id == SOC_ESP32) {
    snprintf_P ( offset, size,
      PSTR("\
<tr>\
<th align=left>Voice</th>\
<td align=right>\
<select name='voice'>\
<option %s value='%d'>Off</option>\
<option %s value='%d'>Traffic Advisories and Collision alerts</option>\
<option %s value='%d'>Collisions Alerts only</option>\
</select>\
</td>\
</tr>"),
    (settings->voice == VOICE_OFF ? "selected" : ""), VOICE_OFF,
    (settings->voice == VOICE_ON ? "selected" : ""), VOICE_ON,
    (settings->voice == VOICE_DANGER ? "selected" : ""), VOICE_DANGER
    );
    len = strlen(offset);
    offset += len;
    size -= len;
  }

  /* Common part 4 */
  snprintf_P ( offset, size,
    PSTR("\
<tr>\
<th align=left>Compass page</th>\
<td align=right>\
<select name='compass'>\
<option %s value='%d'>on</option>\
<option %s value='%d'>off</option>\
</select>\
</td>\
</tr>\
<tr>\
<th align=left>Traffic advisories filter</th>\
<td align=right>\
<select name='filter'>\
<option %s value='%d'>Unfiltered</option>\
<option %s value='%d'>Altitude (&#177; 500 m)</option>\
<option %s value='%d'>Alarm Only</option>\
</select>\
</td>\
</tr>\
<tr>\
<th align=left>Power save</th>\
<td align=right>\
<select name='power_save'>\
<option %s value='%d'>Disabled</option>\
<option %s value='%d'>WiFi OFF (5 min.)</option>\
</select>\
</td>\
</tr>\
<tr>\
<th align=left>Team Member Id</th>\
<td align=right>\
<INPUT type='text' name='team' maxlength='6' size='6' value='%06X'>\
</td>\
</tr>\
</table>\
<p align=center><INPUT type='submit' value='Save and restart'></p>\
</form>\
</body>\
</html>"),
    (settings->compass     == COMPASS_ON   ? "selected" : ""), COMPASS_ON,
    (settings->compass     == COMPASS_OFF  ? "selected" : ""), COMPASS_OFF,
    (settings->filter     == TRAFFIC_FILTER_OFF  ? "selected" : ""), TRAFFIC_FILTER_OFF,
    (settings->filter     == TRAFFIC_FILTER_500M ? "selected" : ""), TRAFFIC_FILTER_500M,
    (settings->filter     == TRAFFIC_FILTER_ALARM ? "selected" : ""), TRAFFIC_FILTER_ALARM,
    (settings->power_save == POWER_SAVE_NONE     ? "selected" : ""), POWER_SAVE_NONE,
    (settings->power_save == POWER_SAVE_WIFI     ? "selected" : ""), POWER_SAVE_WIFI,
     settings->team
  );

  SoC->swSer_enableRx(false);
  server.sendHeader(String(F("Cache-Control")), String(F("no-cache, no-store, must-revalidate")));
  server.sendHeader(String(F("Pragma")), String(F("no-cache")));
  server.sendHeader(String(F("Expires")), String(F("-1")));
  server.send ( 200, "text/html", Settings_temp );
  SoC->swSer_enableRx(true);
  free(Settings_temp);
}

void handleRoot() {
  int sec = millis() / 1000;
  int min = sec / 60;
  int hr = min / 60;

  float vdd = Battery_voltage() ;
  bool low_voltage = (Battery_voltage() <= Battery_threshold());

//  time_t timestamp = now();
  char str_Vcc[8];

  size_t size = 3000;
  char *offset;
  size_t len = 0;

  char *Root_temp = (char *) malloc(size);
  if (Root_temp == NULL) {
    return;
  }
  offset = Root_temp;

  dtostrf(vdd, 4, 2, str_Vcc);

  const char* charge_status_str = charging_status_string(charging_status());
#if defined(SYK6970)
  String chargingCurrent = String(read_SY6970_charge_current());
#elif defined(XPOWERS_CHIP_AXP2101)
  String chargingCurrent = String(read_AXP2101_charge_current());
#else
  String chargingCurrent = String("N/A");
#endif
  int cpu_freq = get_cpu_frequency_mhz();
  String lastBatteryLog = formatBatteryLog(getLastBatteryLogEntry());

  snprintf_P ( offset, size,
    PSTR("<html>\
  <head>\
    <meta charset=\"UTF-8\">\
    <meta name='viewport' content='width=device-width, initial-scale=1'>\
    <title>SkyView status</title>\
  </head>\
<body>\
 <table width=100%%>\
  <tr><!-- <td align=left><h1>&nbsp;&nbsp;&nbsp;&nbsp;&nbsp;</h1></td> -->\
  <td align=center><h1>SkyView status</h1></td>\
  <!-- <td align=right><img src='/logo.png'></td> --></tr>\
 </table>\
 <table width=100%%>\
  <tr><th align=left>Device Id</th><td align=right>%06X</td></tr>\
  <tr><th align=left>Software Version</th><td align=right>%s&nbsp;&nbsp;%s</td></tr>\
  <tr><th align=left>Uptime</th><td align=right>%02d:%02d:%02d</td></tr>\
  <tr><th align=left>CPU Freq</th><td align=right>%s MHz</font></td></tr>\
  <tr><th align=left>Free memory</th><td align=right>%u</td></tr>\
  <tr><th align=left>Battery voltage</th><td align=right><font color=%s>%sV</font></td></tr>\
  <tr><th align=left>Charging Status</th><td align=right>%s</font></td></tr>\
  <tr><th align=left>Charge Current</th><td align=right>%s mA</font></td></tr>\
  <tr><th align=left>Last Battery Log</th><td align=right>%s</td></tr>\
  <tr><th align=left>&nbsp;</th><td align=right>&nbsp;</td></tr>\
  <tr><th align=left>Display</th><td align=right>%s</td></tr>\
  <tr><th align=left>Connection type</th><td align=right>%s</td></tr>"),
    SoC->getChipId() & 0xFFFFFF, SKYVIEW_FIRMWARE_VERSION,
    (SoC == NULL ? "NONE" : SoC->name),
    hr, min % 60, sec % 60, String(cpu_freq).c_str(), ESP.getFreeHeap(),
    low_voltage ? "red" : "green", str_Vcc, charge_status_str,
    chargingCurrent.c_str(), lastBatteryLog.c_str(), "AMOLED 1.75",
    settings->connection == CON_SERIAL        ? "Serial" :
    settings->connection == CON_BLUETOOTH_SPP ? "Bluetooth SPP" :
    settings->connection == CON_BLUETOOTH_LE  ? "Bluetooth LE" :
    settings->connection == CON_WIFI_UDP      ? "WiFi" : "NONE"
  );

  len = strlen(offset);
  offset += len;
  size -= len;

  switch (settings->connection)
  {
  case CON_WIFI_UDP:
    snprintf_P ( offset, size,
      PSTR("\
  <tr><th align=left>Link partner</th><td align=right>%s</td></tr>\
  <tr><th align=left>Link status</th><td align=right>%s established</td></tr>\
  <tr><th align=left>Assigned IP address</th><td align=right>%s</td></tr>"),
      settings->server && strlen(settings->server) > 0 ? settings->server : "NOT SET",
      WiFi.status() == WL_CONNECTED ? "" : "not",
      WiFi.localIP().toString().c_str()
    );
    len = strlen(offset);
    offset += len;
    size -= len;
  case CON_SERIAL:
  case CON_BLUETOOTH_SPP:
  case CON_BLUETOOTH_LE:
    switch (settings->protocol)
    {
    case PROTOCOL_GDL90:
      snprintf_P ( offset, size,
        PSTR("\
  <tr><th align=left>Connection status</th><td align=right>%s connected</td></tr>\
  <tr><th align=left>Data type</th><td align=right>%s %s</td></tr>\
  "),
        GDL90_isConnected()  ? "" : "not",
        GDL90_isConnected()  && !GDL90_hasHeartBeat() ? "UNK" : "",
        GDL90_hasHeartBeat() ? "GDL90"  : ""
      );
      break;
    case PROTOCOL_NMEA:
    default:
      snprintf_P ( offset, size,
        PSTR("\
  <tr><th align=left>Connection status</th><td align=right>%s connected</td></tr>\
  <tr><th align=left>Data type</th><td align=right>%s %s %s</td></tr>\
  "),
        NMEA_isConnected() ? "" : "not",
        NMEA_isConnected() && !(NMEA_hasGNSS() || NMEA_hasFLARM()) ? "UNK" : "",
        NMEA_hasGNSS()     ? "GNSS"  : "",
        NMEA_hasFLARM()    ? "FLARM" : ""
      );
      break;
    }

    len = strlen(offset);
    offset += len;
    size -= len;
    break;
  case CON_NONE:
  default:
    break;
  }

  // show number of OGN records
  if (settings->adb == DB_OGN)
  {
    snprintf_P(offset, size,
    PSTR("\
      <tr><th align=left>OGN Records</th><td align=right>%d</td></tr>"), 
      OGN_Records());

    len = strlen(offset);
    offset += len;
    size -= len;
  }

  snprintf_P(offset, size,
  PSTR("\
    <tr><th align=left>Bridge output</th><td align=right>%s</td></tr>\
    <tr><th align=left>Battery Log Actions</th>\
        <td align=right>\
          <form action=\"/download_log\" method=\"get\" style=\"display:inline\">\
            <button type=\"submit\">Download</button>\
          </form>\
          <form action=\"/clear_log\" method=\"post\" style=\"display:inline\" onsubmit=\"return confirm('Clear battery log?');\">\
            <button type=\"submit\">Clear</button>\
          </form>\
        </td></tr>\
  </table>\
  <hr>\
  <table width=100%%>\
    <tr>\
      <td align=left><input type=button onClick=\"location.href='/settings'\" value='Settings'></td>\
      <td align=center><input type=button onClick=\"location.href='/buddylist'\" value='Buddy List'></td>\
      <td align=center><input type=button onClick=\"location.href='/ble/manage'\" value='BLE Manage'></td>\
      <td align=center><input type=button onClick=\"location.href='/about'\" value='About'></td>\
      <td align=right><input type=button onClick=\"location.href='/firmware'\" value='Firmware update'></td>\
    </tr>\
  </table>\
  </body>\
  </html>"),
    settings->bridge == BRIDGE_BT_LE  ? "Bluetooth LE" :
    settings->bridge == BRIDGE_UDP    ? "WiFi UDP" :
    settings->bridge == BRIDGE_SERIAL ? "Serial"     : "NONE"
  );


  SoC->swSer_enableRx(false);
  server.sendHeader(String(F("Cache-Control")), String(F("no-cache, no-store, must-revalidate")));
  server.sendHeader(String(F("Pragma")), String(F("no-cache")));
  server.sendHeader(String(F("Expires")), String(F("-1")));
  server.send ( 200, "text/html", Root_temp );
  SoC->swSer_enableRx(true);
  free(Root_temp);
}



void handleBuddyList() 
{
  server.send(200, "text/html", R"rawliteral(
<html>
  <head>
    <title>Buddy List Management</title>
  </head>
<body>
  <h1>Upload or Download Buddy List</h1>
  <input type="file" id="fileInput">
  <button onclick="uploadFile()">Upload</button>
  <button onclick="downloadFile()">Download</button>
  <script>
    function uploadFile() {
      let file = document.getElementById('fileInput').files[0];
      if (!file) return;
      let formData = new FormData();
      formData.append('file', file);
      fetch('/upload-buddy', { method: 'POST', body: formData })
      .then(response => alert('File uploaded successfully!'))
      .catch(error => alert('Upload failed.'));
    }
    function downloadFile() {
      window.location.href = '/download-buddy';
    }
  </script>
</body>
</html>
  )rawliteral");
}

void handleInput() 
{

  char *Input_temp = (char *) malloc(1900);
  if (Input_temp == NULL) 
  {
    return;
  }

  for ( uint8_t i = 0; i < server.args(); i++ ) 
  {
    if (server.argName(i).equals("adapter")) 
    {
      settings->adapter = server.arg(i).toInt();
    }
    else if (server.argName(i).equals("connection")) 
    {
      settings->connection = server.arg(i).toInt();
    }
    else if (server.argName(i).equals("bridge")) 
    {
      settings->bridge = server.arg(i).toInt();
    }
    else if (server.argName(i).equals("protocol")) 
    {
      settings->protocol = server.arg(i).toInt();
    }
    else if (server.argName(i).equals("baudrate"))
    {
      settings->baudrate = server.arg(i).toInt();
    }
    else if (server.argName(i).equals("server")) 
    {
      server.arg(i).toCharArray(settings->server, sizeof(settings->server));
    }
    else if (server.argName(i).equals("key")) 
    {
      server.arg(i).toCharArray(settings->key, sizeof(settings->key));
    }
    else if (server.argName(i).equals("units")) 
    {
      settings->units = server.arg(i).toInt();
    }
    else if (server.argName(i).equals("vmode"))
    {
      settings->vmode = server.arg(i).toInt();
    }
    else if (server.argName(i).equals("orientation"))
    {
      settings->orientation = server.arg(i).toInt();
    }
    else if (server.argName(i).equals("zoom"))
    {
      settings->zoom = server.arg(i).toInt();
    }
    else if (server.argName(i).equals("adb"))
    {
      settings->adb = server.arg(i).toInt();
    }
    else if (server.argName(i).equals("idpref"))
    {
      settings->idpref = server.arg(i).toInt();
    }
    else if (server.argName(i).equals("voice"))
    {
      settings->voice = server.arg(i).toInt();
    }
    else if (server.argName(i).equals("compass"))
    {
      settings->compass = server.arg(i).toInt();
    }
    else if (server.argName(i).equals("filter"))
    {
      settings->filter = server.arg(i).toInt();
    }
    else if (server.argName(i).equals("power_save"))
    {
      settings->power_save = server.arg(i).toInt();
    }
    else if (server.argName(i).equals("team"))
    {
      char buf[7];
      server.arg(i).toCharArray(buf, sizeof(buf));
      settings->team = strtoul(buf, NULL, 16);
    }
  }

  // disallow wireless->wireless bridge
  if (settings->connection != CON_SERIAL && settings->bridge != BRIDGE_SERIAL)
  {
    settings->bridge = BRIDGE_NONE;
  }

  snprintf_P ( Input_temp, 2000,
PSTR("<html>\
<head>\
<meta http-equiv='refresh' content='15; url=/'>\
<meta name='viewport' content='width=device-width, initial-scale=1'>\
<title>SkyView Settings</title>\
</head>\
<body>\
<h1 align=center>New settings:</h1>\
<table width=100%%>\
<tr><th align=left>Adapter</th><td align=right>%d</td></tr>\
<tr><th align=left>Connection</th><td align=right>%d</td></tr>\
<tr><th align=left>Protocol</th><td align=right>%d</td></tr>\
<tr><th align=left>Baud rate</th><td align=right>%d</td></tr>\
<tr><th align=left>Server</th><td align=right>%s</td></tr>\
<tr><th align=left>Key</th><td align=right>%s</td></tr>\
<tr><th align=left>Bridge</th><td align=right>%d</td></tr>\
<tr><th align=left>Units</th><td align=right>%d</td></tr>\
<tr><th align=left>View mode</th><td align=right>%d</td></tr>\
<tr><th align=left>Radar orientation</th><td align=right>%d</td></tr>\
<tr><th align=left>Zoom level</th><td align=right>%d</td></tr>\
<tr><th align=left>Aircrafts data</th><td align=right>%d</td></tr>\
<tr><th align=left>ID preference</th><td align=right>%d</td></tr>\
<tr><th align=left>Voice</th><td align=right>%d</td></tr>\
<tr><th align=left>Compass page</th><td align=right>%d</td></tr>\
<tr><th align=left>Filter</th><td align=right>%d</td></tr>\
<tr><th align=left>Power Save</th><td align=right>%d</td></tr>\
<tr><th align=left>Team</th><td align=right>%06X</td></tr>\
</table>\
<hr>\
  <p align=center><h1 align=center>Restart is in progress... Please, wait!</h1></p>\
</body>\
</html>"),
  settings->adapter, settings->connection, settings->protocol,
  settings->baudrate, settings->server, settings->key, settings->bridge,
  settings->units, settings->vmode, settings->orientation, settings->zoom,
  settings->adb, settings->idpref, settings->voice, settings->compass,
  settings->filter, settings->power_save, settings->team
  );

  SoC->swSer_enableRx(false);
  server.send ( 200, "text/html", Input_temp );
//  SoC->swSer_enableRx(true);
  delay(1000);
  free(Input_temp);
  EEPROM_store();
  delay(1000);
  ESP.restart();
}

void handleNotFound()
{
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += ( server.method() == HTTP_GET ) ? "GET" : "POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";

  for ( uint8_t i = 0; i < server.args(); i++ )
  {
    message += " " + server.argName ( i ) + ": " + server.arg ( i ) + "\n";
  }
  server.send ( 404, "text/plain", message );
}

void Web_setup()
{
//  if (!SPIFFS.begin(true))
//  {
//    PRINTLN("SPIFFS Mount Failed!");
    // if (!SPIFFS.format()) {
    //   PRINTLN("SPIFFS Format Failed");
    //   return;
    // }
    // if (!SPIFFS.begin()) {
    //   PRINTLN("SPIFFS Mount Failed again");
    //   return;
    // }
//  }

  if (SPIFFS_is_mounted == false) 
  {
    PRINTLN("[WEB] SPIFFS expected to be mounted already, but it's not. Aborting web server setup.");
    return;
  }
  server.on ( "/", handleRoot );
  server.on ( "/settings", handleSettings );
  server.on("/buddylist", handleBuddyList);

  server.on("/upload-buddy", HTTP_POST, []()
  {
    server.send(200, "text/plain", "Buddy list uploaded");
  }, []()
  {
    HTTPUpload& upload = server.upload();
    static File fsUploadFile;
  
    if (upload.status == UPLOAD_FILE_START)
    {
      PRINTLN("Starting upload: buddylist.txt");
      fsUploadFile = SPIFFS.open("/buddylist.txt", FILE_WRITE);
    }
    else if (upload.status == UPLOAD_FILE_WRITE)
    {
      if (fsUploadFile)
        fsUploadFile.write(upload.buf, upload.currentSize);
    }
    else if (upload.status == UPLOAD_FILE_END)
    {
      if (fsUploadFile)
      {
        fsUploadFile.close();
      }
      PRINTLN("Upload complete.");
      BuddyManager::readBuddyList(); // Read the buddy list after upload
    }
  });
  
  server.on("/download-buddy", HTTP_GET, []()
  {
    File file = SPIFFS.open("/buddylist.txt", "r");
    if (!file || file.isDirectory()) {
      server.send(404, "text/plain", "File not found");
      return;
    }
    server.streamFile(file, "text/plain");
    file.close();
  });
  server.on("/ble/manage", HTTP_GET, []() {
  server.send(200, "text/html; charset=UTF-8", bleManagerHTML);
  });
  server.on("/ble/scan", HTTP_GET, handleBLEScan);
  server.on("/ble/add", HTTP_POST, handleAddBLEDevice);
  server.on("/ble/delete", HTTP_POST, handleDeleteBLEDevice);
  server.on("/ble/allowed", HTTP_GET, []() {
  auto allowed = getAllowedBLENameList();
  DynamicJsonDocument doc(1024);
  JsonArray arr = doc.to<JsonArray>();

  for (const auto& name : allowed)
  {
    arr.add(name);
  }

  String json;
  serializeJson(doc, json);
  server.send(200, "application/json", json);
  });

  server.on("/download_log", HTTP_GET, []()
  {
    File logFile = SPIFFS.open("/battery_log.txt", "r");
    if (!logFile)
    {
      server.send(404, "text/plain", "Log file not found");
      return;
    }
    server.sendHeader("Content-Type", "text/plain");
    server.sendHeader("Content-Disposition", "attachment; filename=\"battery_log.txt\"");
    server.streamFile(logFile, "text/plain");
    logFile.close();
  });

  server.on("/clear_log", HTTP_POST, []()
  {
    if (SPIFFS.exists("/battery_log.txt"))
    {
      SPIFFS.remove("/battery_log.txt");
    }
    server.send(200, "text/html", "<html><body><h2>Battery log cleared.</h2><a href='/'>Back</a></body></html>");
  });

  server.on ( "/about", []()
  {
    SoC->swSer_enableRx(false);
    server.sendHeader(String(F("Cache-Control")), String(F("no-cache, no-store, must-revalidate")));
    server.sendHeader(String(F("Pragma")), String(F("no-cache")));
    server.sendHeader(String(F("Expires")), String(F("-1")));
    server.send_P ( 200, PSTR("text/html"), about_html);
    SoC->swSer_enableRx(true);
  });

  server.on ( "/input", handleInput );
  server.on ( "/inline", []()
  {
    server.send ( 200, "text/plain", "this works as well" );
  });

  server.on("/firmware", HTTP_GET, []()
  {
    SoC->swSer_enableRx(false);
    server.sendHeader(String(F("Connection")), String(F("close")));
    server.sendHeader(String(F("Access-Control-Allow-Origin")), String(F("*")));
    server.send_P(200,
      PSTR("text/html"),
      PSTR("\
<html>\
  <head>\
    <meta name='viewport' content='width=device-width, initial-scale=1'>\
    <title>Firmware update</title>\
  </head>\
<body>\
<body>\
 <h1 align=center>Firmware update</h1>\
 <hr>\
 <table width=100%%>\
  <tr>\
    <td align=left>\
<script src='/jquery.min.js'></script>\
<form method='POST' action='#' enctype='multipart/form-data' id='upload_form'>\
    <input type='file' name='update'>\
    <input type='submit' value='Update'>\
</form>\
<div id='prg'>progress: 0%</div>\
<script>\
$('form').submit(function(e){\
    e.preventDefault();\
      var form = $('#upload_form')[0];\
      var data = new FormData(form);\
       $.ajax({\
            url: '/update',\
            type: 'POST',\
            data: data,\
            contentType: false,\
            processData:false,\
            xhr: function() {\
                var xhr = new window.XMLHttpRequest();\
                xhr.upload.addEventListener('progress', function(evt) {\
                    if (evt.lengthComputable) {\
                        var per = evt.loaded / evt.total;\
                        $('#prg').html('progress: ' + Math.round(per*100) + '%');\
                    }\
               }, false);\
               return xhr;\
            },\
            success:function(d, s) {\
                console.log('success!')\
           },\
            error: function (a, b, c) {\
            }\
          });\
});\
</script>\
    </td>\
  </tr>\
 </table>\
</body>\
</html>")
    );
    SoC->swSer_enableRx(true);
  });
  server.onNotFound ( handleNotFound );

  // this method handles the firmware update
  server.on("/update", HTTP_POST, []()
  {
    SoC->swSer_enableRx(false);
    server.sendHeader(String(F("Connection")), String(F("close")));
    server.sendHeader(String(F("Access-Control-Allow-Origin")), "*");
    server.send(200, String(F("text/plain")), (Update.hasError())?"FAIL":"OK");
//    SoC->swSer_enableRx(true);
    delay(1000);
    ESP.restart();
  },[]()
  {
    HTTPUpload& upload = server.upload();
    if(upload.status == UPLOAD_FILE_START)
    {
      Serial.setDebugOutput(true);
      SoC->WiFiUDP_stopAll();
      SoC->WDT_fini();
      Serial.printf("Update: %s\r\n", upload.filename.c_str());
      //start with max available size
      uint32_t maxSketchSpace = SoC->maxSketchSpace();
      if(!Update.begin(maxSketchSpace))
      {
        Update.printError(Serial);
      }
    }
    else if(upload.status == UPLOAD_FILE_WRITE)
    {
      if(Update.write(upload.buf, upload.currentSize) != upload.currentSize)
      {
        Update.printError(Serial);
      }
    }
    else if(upload.status == UPLOAD_FILE_END)
    {
       //true to set the size to the current progress
      if(Update.end(true))
      {
        Serial.printf("Update Success: %u\r\nRebooting...\r\n", upload.totalSize);
      }
      else
      {
        Update.printError(Serial);
      }
      Serial.setDebugOutput(false);
    }
    yield();
  });

#if !defined(NOLOGO)
  server.on ( "/logo.png", []() {
    server.send_P ( 200, "image/png", Logo, sizeof(Logo) );
  } );
#endif

  // host the jquery.min.js javascript
  // statically defined in lib\jQuery\jquery_min_js.h
  server.on ( "/jquery.min.js", []()
  {
    PGM_P content = jquery_min_js_gz;
    size_t bytes_left = jquery_min_js_gz_len;
    size_t chunk_size;

    server.setContentLength(bytes_left);
    server.sendHeader(String(F("Content-Encoding")),String(F("gzip")));
    server.send(200, String(F("application/javascript")), "");
    do
    {
      chunk_size = bytes_left > JS_MAX_CHUNK_SIZE ? JS_MAX_CHUNK_SIZE : bytes_left;
      server.sendContent_P(content, chunk_size);
      content += chunk_size;
      bytes_left -= chunk_size;
    } while (bytes_left > 0) ;
  });

  server.begin();
  Serial.println (F("HTTP server has started at port: 80"));
  delay(1000);
}

void Web_loop()
{
  server.handleClient();
}

void Web_fini()
{
  server.stop();
}
