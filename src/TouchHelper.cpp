#include <Arduino.h>
// #include "TouchDrvCST92xx.h"
#include <driver/touch/TouchDrvCSTXXX.hpp>
#include "pin_config.h"
#include "TouchHelper.h"
#include "TFTHelper.h"
#include "NMEAHelper.h"
#include "View_Radar_TFT.h"
#include "View_Text_TFT.h"
#include "Platform_ESP32.h"
#include "SkyView.h"
#include "EEPROMHelper.h"
#include "WebHelper.h"
#include "BluetoothHelper.h"
#include <TimeLib.h>
#include "TrafficHelper.h"
#include <DebugLog.h>

// Create an instance of the CST9217 class

TouchDrvCST92xx touchSensor;

uint8_t touchAddress = 0x5A;
extern int TFT_view_mode;

int16_t endX = -1, endY = -1;
static int16_t startX = -1, startY = -1;
static uint32_t startTime = 0;
int16_t currentX[5], currentY[5];

// Task Handle
TaskHandle_t touchTaskHandle = NULL;

bool IIC_Interrupt_Flag = false;
unsigned long lastTapTime = 0;
unsigned long debounceDelay = 100; // in milliseconds

bool isLabels = true;
bool show_compass = true;;
extern bool isLocked;

void Touch_2D_Unrotate(float &tX, float &tY) 
{
    float angleRad = D2R * ThisAircraft.Track;
    float trSin = sin(angleRad);
    float trCos = cos(angleRad);

    float tTemp = tX * trCos + tY * trSin;
    tY = -tX * trSin + tY * trCos;
    tX = tTemp;
}


void findTouchedTarget(int rawTouchX, int rawTouchY) 
{
  // ignore if not in radar view
  if (TFT_view_mode != VIEW_MODE_RADAR)
  {
    return;
  }
  int zoom = getCurrentZoom();
  int range = 3000;  // default
  // Convert touch to center-relative coordinates (pixels)
  float touchX = rawTouchX - (LCD_WIDTH / 2);
  float touchY = -(rawTouchY - (LCD_HEIGHT / 2));
  Serial.printf("Touch coordinates: (%f, %f)\n", touchX, touchY);

  // Rotate to NORTH_UP if needed
  if (settings->orientation == DIRECTION_TRACK_UP) 
  {
    Touch_2D_Unrotate(touchX, touchY);
    //Serial.printf("Unrotated touch coordinates: (%f, %f)\n", touchX, touchY);
  }

  // Get current range in meters
  if (settings->units == UNITS_METRIC || settings->units == UNITS_MIXED) 
  {
    switch (zoom) 
    {
      case ZOOM_LOWEST:  range = 9000; break;
      case ZOOM_LOW:     range = 6000; break;
      case ZOOM_HIGH:    range = 900;  break;
      case ZOOM_MEDIUM:
      default:           range = 3000; break;
    }
  }
  else 
  {
    switch (zoom) 
    {
      case ZOOM_LOWEST:  range = 9260; break;
      case ZOOM_LOW:     range = 4630; break;
      case ZOOM_HIGH:    range = 926;  break;
      case ZOOM_MEDIUM:
      default:           range = 1852; break;
    }
  }

  // Convert pixel distances to meters
  float radius = LCD_WIDTH / 2.0f - 2.0f;
  float scale = (float)range / radius;  // meters per pixel
  float touchXMeters = touchX * scale;
  float touchYMeters = touchY * scale;
  int touchHitRadius = range / 10; // 10% of the range as hit radius

  //Serial.printf("Touch in meters: (%f, %f)\n", touchXMeters, touchYMeters);

  for (int i = 0; i < MAX_TRACKING_OBJECTS; i++) {

    if (Container[i].ID == 0) continue;
    if ((now() - Container[i].timestamp) > TFT_EXPIRATION_TIME) continue;

    Serial.printf("Checking target ID: %06X\n", Container[i].ID);
    Serial.printf("Target coordinates: (%f, %f)\n", Container[i].RelativeEast, Container[i].RelativeNorth);

    float dx = Container[i].RelativeEast - touchXMeters;
    float dy = Container[i].RelativeNorth - touchYMeters;

    if ((dx * dx + dy * dy) < (touchHitRadius * touchHitRadius)) 
    {
      Serial.printf("Touched target ID: %06X at (%f, %f)\n", Container[i].ID, Container[i].RelativeEast, Container[i].RelativeNorth);
      // TODO: handle selection
      isLocked = true;
      setFocusOn(true, Container[i].ID);
      delay(100); // Debounce delay
      TFT_Mode(true); // Switch to text view
      break;
    }
  }
}

void touchWakeUp() 
{
    digitalWrite(SENSOR_RST, HIGH);
    delay(100);
    touchSensor.wakeup();
}

void Touch_setup() 
{
  PRINTLN("Initializing touch sensor...");
  attachInterrupt(TP_INT, []()
  { IIC_Interrupt_Flag = true; }, FALLING);
  
  touchSensor.setPins(SENSOR_RST, SENSOR_IRQ);
  if (touchSensor.begin(Wire, touchAddress, IIC_SDA, IIC_SCL) == false)
  {
    PRINTLN("[ERROR] CST9217 initialization failed!");
  }
  else
  {
    PRINT("Model :");
    PRINTLN(touchSensor.getModelName());
    touchSensor.setMaxCoordinates(LCD_WIDTH, LCD_HEIGHT); // Set touch max xy
  }
  // start the touch task on core 1
  xTaskCreatePinnedToCore(touchTask, "Touch Task", 4096, NULL, 1, &touchTaskHandle, 1);
}

void tapHandler(int x, int y) 
{
  //It looks like the touch sensor is upside down so we need to flip the coordinates
  //x = LCDWIDTH - x;  
  //y = LCDHIEGHT - y;
  PRINTLN("Tap detected at coordinates: " + String(x) + ", " + String(y));
  bool hasFix = settings->protocol == PROTOCOL_NMEA  ? isValidGNSSFix() : false;
  if (LCD_WIDTH - x > 290 && LCD_WIDTH - x < 400 && LCD_HEIGHT - y > 360 && LCD_HEIGHT - y <  466
    && (TFT_view_mode == VIEW_MODE_TEXT || (TFT_view_mode == VIEW_MODE_RADAR && !hasFix))) 
  {
    PRINTLN("Going to SettingsPage ");
    settings_page_num = 1;
    settings_page();
  }
  else if (TFT_view_mode == VIEW_MODE_SETTINGS && settings_page_num == 1 && LCD_WIDTH - x > 340 && LCD_WIDTH - x < 410 && LCD_HEIGHT - y > 370 && LCD_HEIGHT - y < 430)
  {
    LOG_INFO("Going to sleep from settings page...");
    shutdown("SLEEP");
  }
  else if (LCD_WIDTH - x > 130 && LCD_WIDTH - x < 340 && LCD_HEIGHT - y > 420 && LCD_HEIGHT - y < 466
    && TFT_view_mode == VIEW_MODE_SETTINGS)
  {
    PRINTLN("Going Back to previous page ");
    if (settings_page_num == 2)
    {
      settings_page_num = 1;
      settings_page();
    }
    else
    {
      TFT_Mode(true);
    }
  }
  else if (LCD_WIDTH - x > 40 && LCD_WIDTH - x < 400 && LCD_HEIGHT - y > 95 && LCD_HEIGHT - y < 148
    && TFT_view_mode == VIEW_MODE_SETTINGS && settings_page_num == 1)
  {
    PRINTLN("Cycling Alt Filter ");
    switch (settings->filter)
    {
      case TRAFFIC_FILTER_OFF:    settings->filter = TRAFFIC_FILTER_500M;   break;
      case TRAFFIC_FILTER_500M:   settings->filter = TRAFFIC_FILTER_1000M;  break;
      case TRAFFIC_FILTER_1000M:  settings->filter = TRAFFIC_FILTER_3500FT; break;
      case TRAFFIC_FILTER_3500FT: settings->filter = TRAFFIC_FILTER_5000FT; break;
      case TRAFFIC_FILTER_5000FT: settings->filter = TRAFFIC_FILTER_OFF;    break;
      default:                    settings->filter = TRAFFIC_FILTER_OFF;    break;
    }
    settings_page();
  }
  else if (LCD_WIDTH - x > 320 && LCD_WIDTH - x < 400 && LCD_HEIGHT - y > 150 && LCD_HEIGHT - y < 203 && TFT_view_mode == VIEW_MODE_SETTINGS && settings_page_num == 1)
  {
    PRINTLN("Changing Compass View ");
    show_compass = !show_compass;
    settings_page();
  }
  else if (LCD_WIDTH - x > 320 && LCD_WIDTH - x < 400 && LCD_HEIGHT - y > 205 && LCD_HEIGHT - y < 258 && TFT_view_mode == VIEW_MODE_SETTINGS && settings_page_num == 1)
  {
    PRINTLN("Changing Radar Orientation North Up / Track Up ");
    if (settings->orientation == DIRECTION_NORTH_UP)
      settings->orientation = DIRECTION_TRACK_UP;
    else
      settings->orientation = DIRECTION_NORTH_UP;
    settings_page();
  }
  else if (LCD_WIDTH - x > 320 && LCD_WIDTH - x < 400 && LCD_HEIGHT - y > 260 && LCD_HEIGHT - y < 313 && TFT_view_mode == VIEW_MODE_SETTINGS && settings_page_num == 1)
  {
    PRINTLN("Toggle Labels ");
    isLabels = !isLabels;
    settings->show_labels = isLabels;
    settings_page();
  }
  else if (LCD_WIDTH - x > 320 && LCD_WIDTH - x < 400 && LCD_HEIGHT - y > 315 && LCD_HEIGHT - y < 368 && TFT_view_mode == VIEW_MODE_SETTINGS && settings_page_num == 1)
  {
    PRINTLN("Toggle Icon Style ");
    if (settings->icon_style == ICON_STYLE_ARROWHEAD)
      settings->icon_style = ICON_STYLE_CLASSIC;
    else
      settings->icon_style = ICON_STYLE_ARROWHEAD;
    settings_page();
  } 
  else if (LCD_WIDTH - x > 0 && LCD_WIDTH - x < 400 && LCD_HEIGHT - y > 70 && LCD_HEIGHT - y < 150 && TFT_view_mode == VIEW_MODE_TEXT) 
  {
    //Lock focus on current target
    if (!isLocked) 
    {
     isLocked = true;
     setFocusOn(true);
     PRINTLN("Locking focus on current target ");
    }
    else 
    {
      isLocked = false;
      setFocusOn(false);
      PRINTLN("Unlocking focus from current target ");
    }
    TFTTimeMarker = 0; // Force update of the display
  }
  else if (TFT_view_mode == VIEW_MODE_SETTINGS && settings_page_num == 2)
  {
    // BLE Manager page touch handlers

    // Disconnect button (x: 58-148, y: 72-120)
    if (LCD_WIDTH - x >= 58 && LCD_WIDTH - x <= 148 && LCD_HEIGHT - y >= 72 && LCD_HEIGHT - y <= 120)
    {
      if (ESP32_BT_ctl.status == BT_STATUS_CON)
      {
        PRINTLN("BLE Disconnect button pressed");
        ESP32_BT_ctl.command = BT_CMD_DISCONNECT;
        ble_manager_page();
      }
    }
    
    // Scan button (physical x: 164-334, y: 72-120)
    else if (LCD_WIDTH - x >= 164 && LCD_WIDTH - x <= 334 && LCD_HEIGHT - y >= 72 && LCD_HEIGHT - y <= 120)
    {
      PRINTLN("BLE Scan button pressed");
      start_ble_scan();
      ble_manager_page();
    }
    // Add button for each row (check this before row selection)
    else if (LCD_WIDTH - x >= 292 && LCD_WIDTH - x <= 334 && LCD_HEIGHT - y >= 158 && LCD_HEIGHT - y <= 334)
    {
      int device_index = (LCD_HEIGHT - y - 158) / 44;
      if (device_index >= 0 && device_index < ble_scan_results.size() && device_index < 4)
      {
        PRINTLN("Adding BLE device: " + ble_scan_results[device_index]);
        addAllowedBLEDevice(ble_scan_results[device_index]);
        ble_selected_device = -1;
        ble_device_added = true;
        ble_add_confirmation_time = millis();
        ble_manager_page(); // Refresh display
      }
    }
    // Device selection (max 4 shown)
    else if (LCD_WIDTH - x >= 42 && LCD_WIDTH - x <= 336 && LCD_HEIGHT - y >= 158 && LCD_HEIGHT - y <= 334)
    {
      int device_index = (LCD_HEIGHT - y - 158) / 44;
      if (device_index >= 0 && device_index < ble_scan_results.size() && device_index < 4)
      {
        PRINTLN("BLE Device selected: " + String(device_index));
        ble_selected_device = device_index;
        ble_manager_page(); // Update display
      }
    }
  }
  else if (TFT_view_mode == VIEW_MODE_RADAR) 
  {
    findTouchedTarget(LCD_WIDTH - x, LCD_HEIGHT - y);
  }
  else
  {
    PRINTLN("No Tap match found...");
  }
}

void touchTask(void *parameter) 
{   
  PRINTLN("Touch task started...");
  while(true) 
  {     
    if (IIC_Interrupt_Flag) 
    {
      // Serial.println("Touch Interrupt triggered!");
      IIC_Interrupt_Flag = false; // Reset interrupt flag
      uint8_t points = touchSensor.getPoint(currentX, currentY, 1); // Read single touch point
  
      if (points > 0) 
      {
        // Record the starting touch position and time
        if (startX == -1 && startY == -1) 
        {
          startX = currentX[0];
          startY = currentY[0];
          startTime = millis();
        }
  
        // Continuously update the end position
        endX = currentX[0];
        endY = currentY[0];
      }
      else 
      {
        // If no more points are detected, process swipe
        if (startX != -1 && startY != -1) 
        {
          uint32_t duration = millis() - startTime;  
          int16_t deltaX = endX - startX;
          int16_t deltaY = endY - startY;
  
          // Swipe detection logic
          if (duration < 500) // Limit gesture duration
          { 
            if (abs(deltaX) > abs(deltaY)) // Horizontal swipe
            { 
              if (deltaX > 30) 
              {
                LOG_DEBUG("Swipe Left");
                if (TFT_view_mode == VIEW_MODE_SETTINGS)
                {
                  // Navigate to next settings page (only if BLE is the connection type)
                  if (settings_page_num < 2 && settings->connection == CON_BLUETOOTH_LE) {
                    settings_page_num++;
                    settings_page();
                  }
                }
                else
                {
                  TFT_Mode(true);
                }
              }
              else if (deltaX < -30) 
              {
                LOG_DEBUG("Swipe Right");
                if (TFT_view_mode == VIEW_MODE_SETTINGS)
                {
                  // Navigate to previous settings page
                  if (settings_page_num > 1) {
                    settings_page_num--;
                    settings_page();
                  }
                }
                else
                {
                  TFT_Mode(false);
                }
              }
            }
            else if (abs(deltaX) < abs(deltaY))    // Vertical swipe
            {
                if (deltaY > 50) 
                {
                  LOG_DEBUG("Swipe Up - Radar Zoom Out");
                  if (TFT_view_mode != VIEW_MODE_SETTINGS)
                  {
                    TFT_Up();
                  }
                }
                else if (deltaY < -50) 
                {
                  LOG_DEBUG("Swipe Down - Radar Zoom In");
                  if (TFT_view_mode != VIEW_MODE_SETTINGS)
                  {
                    TFT_Down();
                  }
                }
              }
              else if (abs(deltaX) < 50 && abs(deltaY) < 50) 
              {
                unsigned long currentTime = millis();
                if (currentTime - lastTapTime >= debounceDelay) 
                {
                  lastTapTime = currentTime;
                  // Serial.println("Tap");
                  delay(100); // Debounce delay
                  tapHandler(endX, endY); // Call tap handler with coordinates
                }
              }
            }
            else if (duration > 500 && duration < 2000 && abs(deltaX) < 50 && abs(deltaY) < 50) 
            {
              LOG_DEBUG("Touch Long Press");
            }
            // Reset variables for next swipe detection
            startX = startY = -1;
            startTime = 0;
            endX = endY = -1;
        }
      }      
    }
    delay(50); // Polling delay
  }
}
