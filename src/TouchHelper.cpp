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

void Touch_2D_Unrotate(float &tX, float &tY) {
    float angleRad = D2R * ThisAircraft.Track;
    float trSin = sin(angleRad);
    float trCos = cos(angleRad);

    float tTemp = tX * trCos + tY * trSin;
    tY = -tX * trSin + tY * trCos;
    tX = tTemp;
}


void findTouchedTarget(int rawTouchX, int rawTouchY) {
  if (TFT_view_mode != VIEW_MODE_RADAR) return;
  int zoom = getCurrentZoom();
  int range = 3000;  // default
  // Convert touch to center-relative coordinates (pixels)
  float touchX = rawTouchX - (LCD_WIDTH / 2);
  float touchY = -(rawTouchY - (LCD_HEIGHT / 2));
  Serial.printf("Touch coordinates: (%f, %f)\n", touchX, touchY);

  // Rotate to NORTH_UP if needed
  if (settings->orientation == DIRECTION_TRACK_UP) {
    Touch_2D_Unrotate(touchX, touchY);
    //Serial.printf("Unrotated touch coordinates: (%f, %f)\n", touchX, touchY);
  }

  // Get current range in meters
  if (settings->units == UNITS_METRIC || settings->units == UNITS_MIXED) {
    switch (zoom) {
      case ZOOM_LOWEST:  range = 9000; break;
      case ZOOM_LOW:     range = 6000; break;
      case ZOOM_HIGH:    range = 900;  break;
      case ZOOM_MEDIUM:
      default:           range = 3000; break;
    }
  } else {
    switch (zoom) {
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

    if ((dx * dx + dy * dy) < (touchHitRadius * touchHitRadius)) {
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
    && (TFT_view_mode == VIEW_MODE_TEXT || (TFT_view_mode == VIEW_MODE_RADAR && !hasFix))) {
    PRINTLN("Going to SettingsPage ");
    settings_page();
  }
  else if (TFT_view_mode == VIEW_MODE_SETTINGS && LCD_WIDTH - x > 340 && LCD_WIDTH - x < 410 && LCD_HEIGHT - y > 340 && LCD_HEIGHT - y < 415)
  {
    //Sleep device and wake up on button press wake up button is PIN 0
    LOG_INFO("Going to sleep from settings page...");
    // vTaskDelete(touchTaskHandle);
    // touchTaskHandle = NULL;
    shutdown("SLEEP");
    // ESP32_TFT_fini("SLEEP");
    // delay(1000);
    // ESP32_fini();
    // delay(1000);
  } 
  else if (LCD_WIDTH - x > 160 && LCD_WIDTH - x < 330 && LCD_HEIGHT - y > 410 && LCD_HEIGHT - y < 466
    && TFT_view_mode == VIEW_MODE_SETTINGS) {
    //Back button
    PRINTLN("Going Back to previous page ");
    TFT_Mode(true);
  } 
  else if (LCD_WIDTH - x > 320 && LCD_WIDTH - x < 400 && LCD_HEIGHT - y > 110 && LCD_HEIGHT - y < 170
    && TFT_view_mode == VIEW_MODE_SETTINGS) {
    //Traffic Filter +- 500m
    PRINTLN("Changing Traffic Filter +- 500m ");
    if (settings->filter  == TRAFFIC_FILTER_500M) {
      settings->filter  = TRAFFIC_FILTER_OFF;
      settings_page();
    }
    else {
      settings->filter  = TRAFFIC_FILTER_500M;
      settings_page();
    }
  } 
  else if (LCD_WIDTH - x > 320 && LCD_WIDTH - x < 400 && LCD_HEIGHT - y > 227 && LCD_HEIGHT - y < 290 && TFT_view_mode == VIEW_MODE_SETTINGS) {
    //Radar Orientation North Up / Track Up
    PRINTLN("Changing Radar Orientation North Up / Track Up ");
    if (settings->orientation  == DIRECTION_NORTH_UP) {
      settings->orientation  = DIRECTION_TRACK_UP;
      settings_page();
    }
    else {
      settings->orientation  = DIRECTION_NORTH_UP;
      settings_page();
    }
  } 
  else if (LCD_WIDTH - x > 320 && LCD_WIDTH - x < 400 && LCD_HEIGHT - y > 290 && LCD_HEIGHT - y < 350 && TFT_view_mode == VIEW_MODE_SETTINGS) {
    //Enable Labels (Initials)
    PRINTLN("Toggle Labels ");
    if (!isLabels) {
      isLabels = true;
      settings->show_labels = true;
      settings_page();
    }
    else {
      isLabels = false;
      settings->show_labels = false;
      settings_page();
    }
  } 
  else if (LCD_WIDTH - x > 320 && LCD_WIDTH - x < 400 && LCD_HEIGHT - y > 170 && LCD_HEIGHT - y < 230 && TFT_view_mode == VIEW_MODE_SETTINGS) {
    //Show Compass Page
    PRINTLN("Changing Compass View ");
    if (!show_compass) {
      show_compass = true;
      settings_page();
    }
    else {
      show_compass = false;
      settings_page();
    }
  } 
  else if (LCD_WIDTH - x > 0 && LCD_WIDTH - x < 400 && LCD_HEIGHT - y > 70 && LCD_HEIGHT - y < 150 && TFT_view_mode == VIEW_MODE_TEXT) {
    //Lock focus on current target
    if (!isLocked) {
      isLocked = true;
     setFocusOn(true);
     PRINTLN("Locking focus on current target ");
    }
    else {
      isLocked = false;
      setFocusOn(false);
      PRINTLN("Unlocking focus from current target ");
    }
    TFTTimeMarker = 0; // Force update of the display
  } else if (TFT_view_mode == VIEW_MODE_RADAR) {
    findTouchedTarget(LCD_WIDTH - x, LCD_HEIGHT - y);

  } else {
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
                TFT_Mode(true);
              }
              else if (deltaX < -30) 
              {
                LOG_DEBUG("Swipe Right");
                TFT_Mode(false);
              }
            }
            else if (abs(deltaX) < abs(deltaY))    // Vertical swipe
            {
                if (deltaY > 50) 
                {
                  LOG_DEBUG("Swipe Up - Radar Zoom Out");
                  TFT_Up();
                }
                else if (deltaY < -50) 
                {
                  LOG_DEBUG("Swipe Down - Radar Zoom In");
                  TFT_Down();
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
