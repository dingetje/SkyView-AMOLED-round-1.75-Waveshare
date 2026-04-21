#if defined(USE_TFT)
#include "TFT_eSPI.h"
// #include "Free_Fonts.h"
#include "SoCHelper.h"
#include "EEPROMHelper.h"
#include "TrafficHelper.h"
#include "TFTHelper.h"
#include "TouchHelper.h"
#include "WiFiHelper.h"
#include "BatteryHelper.h"
#include "BluetoothHelper.h"
#include "WebHelper.h"
// #include <Adafruit_GFX.h>    // Core graphics library
// #include "Arduino_GFX_Library.h"
#include "Platform_ESP32.h"
#include "Arduino_DriveBus_Library.h"
#include "TouchDrvCST92xx.h"
#include <../pins_config.h>
#include <driver/display/CO5300.h>
#include "power.h"

#include <DebugLog.h>

// #include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
// #include <Adafruit_ST77xx.h> // Hardware-specific library for ST7789

// #include <Fonts/GFXFF/FreeSansBold12pt7b.h>
// #include <Fonts/GFXFF/FreeMonoBold24pt7b.h>


#include "SkyView.h"
uint16_t display_column_offset = 6;
uint16_t display_row_offset = 0;
int TFT_view_mode = 0;
unsigned long TFTTimeMarker = 0;
bool EPD_display_frontpage = false;

int prev_TFT_view_mode = 0;
extern bool wifi_sta;
extern bool TFTrefresh;

// Settings page state tracking
uint8_t settings_page_num = 1; // 1 = general settings, 2 = BLE manager
std::vector<String> ble_scan_results;
bool ble_scanning = false;
unsigned long ble_scan_start_time = 0;
int ble_selected_device = -1;
bool ble_device_added = false;
uint32_t ble_add_confirmation_time = 0;
bool ble_page_needs_refresh = false;


#if defined(AMOLED)

#define MAX_BRIGHTNESS 255

// SPIClass SPI_2(HSPI);

// Arduino_DataBus *bus = new Arduino_ESP32QSPI(
//     LCD_CS /* CS */, LCD_SCLK /* SCK */, LCD_SDIO0 /* SDIO0 */, LCD_SDIO1 /* SDIO1 */,
//     LCD_SDIO2 /* SDIO2 */, LCD_SDIO3 /* SDIO3 */);

#if defined DO0143FAT01
// Arduino_GFX *gfx = new Arduino_SH8601(bus, LCD_RST /* RST */,
//                                       0 /* rotation */, false /* IPS */, LCD_WIDTH, LCD_HEIGHT);
TFT_eSPI tft = TFT_eSPI();
TFT_eSprite sprite = TFT_eSprite(&tft);
TFT_eSprite sprite2 = TFT_eSprite(&tft);

#elif defined H0175Y003AM || defined WAVESHARE_AMOLED_1_75
xSemaphoreHandle spiMutex;
TFT_eSPI tft = TFT_eSPI();
TFT_eSprite sprite = TFT_eSprite(&tft);
TFT_eSprite sprite2 = TFT_eSprite(&tft);
TFT_eSprite bearingSprite = TFT_eSprite(&tft);
TFT_eSprite batterySprite = TFT_eSprite(&tft);
TFT_eSprite compasSprite = TFT_eSprite(&tft);
TFT_eSprite compas2Sprite = TFT_eSprite(&tft);


#else
#error "Unknown macro definition. Please select the correct macro definition."
#endif

  
#endif

unsigned long drawTime = 0;

float batteryToPercentage(float voltage) {
  if (voltage >= 4.2) return 100.0;
  if (voltage <= 3.0) return 0.0;

  // Extended voltage-percentage mapping
  static const float voltageLevels[] = {4.2, 4.00, 3.9, 3.8, 3.7, 3.6, 3.5, 3.4, 3.3, 3.2, 3.0};
  static const float percentages[] = {100, 95, 90, 75, 55, 40, 25, 10, 5, 2, 0};

  for (int i = 0; i < 11; i++) { // Loop through voltage levels
      if (voltage > voltageLevels[i + 1]) {
          // Linear interpolation between two points
          return percentages[i] + ((voltage - voltageLevels[i]) /
              (voltageLevels[i + 1] - voltageLevels[i])) * (percentages[i + 1] - percentages[i]);
      }
  }
  return 0.0; // Fallback
}
void draw_battery() 
{
    //Battery indicator
    uint16_t battery_x = 295;
    uint16_t battery_y = 35;
    float battery = 0;
    uint8_t batteryPercentage = 0;
    uint16_t batt_color = TFT_CYAN;
      // draw battery symbol
   
    
    battery = Battery_voltage();
    // PRINT(F(" Battery= "));  PRINTLN(battery);
    batteryPercentage = (int)batteryToPercentage(battery);
    // PRINT(F(" Batterypercentage= "));  PRINTLN(batteryPercentage);

    // battery is a 1S Lipo, so ~3.7V nominal with a ~4.1V max.
    if (battery < 3.65 &&  battery >= 3.5) 
    {
      batt_color = TFT_YELLOW;
    }
    else if (battery < 3.5) 
    {
      batt_color = TFT_RED;
    }
    else
    {
      batt_color = TFT_CYAN;
    }
    // Chargin Done?
    if (charging_status() == 3) 
    { 
      batt_color = TFT_GREEN;
    }
    if (charging_status() == 1 || charging_status() == 2) 
    {
      //draw charging icon
      sprite.fillTriangle(battery_x - 20, battery_y + 10, battery_x - 8, battery_y - 3, battery_x - 12, battery_y + 10, batt_color);
      sprite.fillTriangle(battery_x -4, battery_y + 10, battery_x - 16, battery_y + 23, battery_x - 12, battery_y + 10, batt_color);
    }
    sprite.drawRoundRect(battery_x, battery_y, 32, 20, 3, batt_color);
    sprite.fillRect(battery_x + 32, battery_y + 7, 2, 7, batt_color);
    int fillWidth = (int)(30 * ((float)batteryPercentage / 100));
    // PRINT(F("Fill width = "));
    // PRINTLN(fillWidth);
    sprite.fillRect(battery_x + 2, battery_y + 3, fillWidth, 14, batt_color);

    sprite.setCursor(battery_x, battery_y + 24, 4);
    sprite.setTextColor(TFT_WHITE, TFT_BLACK);
    sprite.printf("%d%%", batteryPercentage); // Use %% to print the % character
}

// if BLE source, also draw remote BLE battery state
void draw_extBattery() 
{
  uint8_t extBatteryPerc = getBLEbattery();
  if (extBatteryPerc == 0) 
  {
    return; // No external battery data available
  }
  //Battery indicator
  uint16_t battery_x = 80;
  uint16_t battery_y = 325;
  // uint8_t extBatteryPerc = 75;
  uint16_t batt_color = TFT_CYAN;

  if (extBatteryPerc < 50 &&  extBatteryPerc >= 25) 
  {
    batt_color = TFT_YELLOW;
  }
  else if (extBatteryPerc < 25) 
  {
    batt_color = TFT_RED;
  }
  else
  {
    batt_color = TFT_CYAN;
  }

  //draw bluetooth icon
  sprite.drawWideLine(battery_x - 15, battery_y - 3, battery_x - 15, battery_y + 28, 2, TFT_BLUEBUTTON);
  sprite.drawWideLine(battery_x - 15, battery_y - 3, battery_x - 7, battery_y + 5, 2, TFT_BLUEBUTTON);
  sprite.drawWideLine(battery_x - 25, battery_y + 18, battery_x - 7, battery_y + 5, 2, TFT_BLUEBUTTON);
  sprite.drawWideLine(battery_x - 25, battery_y + 5, battery_x - 7, battery_y + 18, 2, TFT_BLUEBUTTON);
  sprite.drawWideLine(battery_x - 15, battery_y + 28, battery_x - 7, battery_y + 18, 2, TFT_BLUEBUTTON);

  // draw battery with color
  sprite.drawRoundRect(battery_x + 2, battery_y, 14, 28, 2, batt_color);
  sprite.fillRect(battery_x + 5, battery_y - 4, 7, 3, batt_color);
  float fillHeight = 24 * extBatteryPerc / 100; // Calculate fill height based on percentage

  sprite.fillRect(battery_x + 4, battery_y + (24 - fillHeight), 10, fillHeight, batt_color);

  sprite.setCursor(battery_x + 24, battery_y + 7, 4);
  sprite.setTextColor(TFT_WHITE, TFT_BLACK);
  sprite.printf("%d%%", extBatteryPerc); // Use %% to print the % character
}


void draw_splash_screen()
{
  PRINTLN("Drawing SkyView Splash screen...");
  sprite.fillSprite(TFT_BLACK);
  sprite.setTextDatum(MC_DATUM);
  sprite.setTextColor(TFT_WHITE, TFT_BLACK);
  sprite.setFreeFont(&Orbitron_Light_32);
  sprite.setCursor(144, 160);
  // sprite.setTextSize(2);
  sprite.printf("SkyView");
  PRINT("SkyView width: ");
  PRINTLN(sprite.textWidth("SkyView"));
  PRINT("SkyView height: ");
  PRINTLN(sprite.fontHeight(4));
  sprite.setFreeFont(&FreeSansBold12pt7b);
  PRINT("powered by... width: ");
  PRINTLN(sprite.textWidth("powered by SoftRF"));
  PRINT("powered by ... height: ");
  PRINTLN(sprite.fontHeight(4));
  sprite.fillRect(114,200,66,66,TFT_RED);
  sprite.fillRect(200,200,66,66,TFT_GREEN);
  sprite.fillRect(286,200,66,66,TFT_BLUE); 
  sprite.setTextSize(1);

  sprite.drawString("powered by SoftRF",233,293,4);
  sprite.drawString(SKYVIEW_FIRMWARE_VERSION,180,400,2);
  lcd_PushColors(display_column_offset, display_row_offset, LCD_WIDTH, LCD_HEIGHT, (uint16_t*)sprite.getPointer());
  for (int i = 0; i <= MAX_BRIGHTNESS; i++)
  {
    lcd_brightness(i);
    delay(4);
  }

  // required for Voice output, mount it during splash time
  MountSDCard();

  delay(2000);
  for (int i = MAX_BRIGHTNESS; i > 0; i--)
  {
    lcd_brightness(i);
    delay(4);
  }
  PRINTLN("Exit splash screen...");
}

void TFT_setup(void) 
{
  pinMode(LCD_EN, OUTPUT);
  digitalWrite(LCD_EN, HIGH);
  delay(30);
  pinMode(SENSOR_RST, OUTPUT);
  digitalWrite(SENSOR_RST, LOW);
  delay(30);
  digitalWrite(SENSOR_RST, HIGH);
  delay(50);
  setupWireIfNeeded(SENSOR_SDA, SENSOR_SCL);
  CO5300_init();
  sprite.setColorDepth(16);
  PRINT("TFT_setup. PSRAM_ENABLE: ");
  PRINTLN(sprite.getAttribute(PSRAM_ENABLE));
  sprite.setAttribute(PSRAM_ENABLE, 1);
  // Initialise SPI Mutex
  spiMutex = xSemaphoreCreateMutex();
  if (spiMutex == NULL) 
  {
      PRINTLN("Failed to create SPI mutex!");
  }
  // Clear GRAM borders in rotation 0 to avoid green artifacts at display edges
  lcd_setRotation(0);
  lcd_fill(0, 0, 480, 7, 0x0000);     // top 7 rows
  lcd_fill(0, 466, 480, 480, 0x0000);  // bottom 14 rows
  lcd_fill(0, 0, 7, 480, 0x0000);     // left 7 columns
  lcd_fill(473, 0, 480, 480, 0x0000);  // right 7 columns
  if (settings->rotation != 0 && settings->rotation != 2)
    settings->rotation = 0;
  lcd_setRotation(settings->rotation);
  display_column_offset = 7;
  display_row_offset = 0;
  lcd_brightness(0); // 0-255

  Serial.printf("Free heap: %d bytes\n", esp_get_free_heap_size());
  Serial.printf("Largest block: %d bytes\n", heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
  sprite.createSprite(LCD_WIDTH, LCD_HEIGHT);    // full screen landscape sprite in psram
  batterySprite.createSprite(100, 32);
  if (sprite.createSprite(LCD_WIDTH, LCD_HEIGHT) == NULL) 
  {
    PRINTLN("Failed to create sprite. Not enough memory.");
    delay(5000);
  }
  else 
  {
    PRINT("TFT_setup. Created Sprite| Free Heap: ");
    PRINTLN(esp_get_free_heap_size());
  }
  TFT_view_mode = settings->vmode;
  draw_splash_screen();
  TFT_radar_setup();
}

void TFT_loop(void) 
{
  switch (TFT_view_mode)
  {
  case VIEW_MODE_RADAR:
    TFT_radar_loop();
    break;
  case VIEW_MODE_TEXT:
    TFT_text_loop();
    break;
  case VIEW_MODE_COMPASS:
    TFT_compass_loop();
    break;
  default:
    break;
  }
  yield();  // Ensure the watchdog gets reset
  delay(20);
}

void TFT_Mode(boolean next)
{
  if (hw_info.display != DISPLAY_TFT)
  {
    return;
  }

  if (TFT_view_mode == VIEW_MODE_RADAR) 
  {
    if (next) 
    {
      if (xSemaphoreTake(spiMutex, portMAX_DELAY)) 
      {
        TFT_view_mode = VIEW_MODE_TEXT;
        if (!bearingSprite.created()) 
        {
          bearingSprite.createSprite(78, 54);
          bearingSprite.setColorDepth(16);
          bearingSprite.setSwapBytes(true);
        }
        TFTTimeMarker = millis() + 1001;
        xSemaphoreGive(spiMutex);
        delay(10);
      }
      else
      {
        LOG_ERROR("Failed to acquire SPI semaphore!");
      }
    }
    else 
    {
      if (xSemaphoreTake(spiMutex, portMAX_DELAY)) 
      {
        if (show_compass) 
        {
          TFT_view_mode = VIEW_MODE_COMPASS;
          if (!compasSprite.created()) 
          {
            compasSprite.createSprite(LCD_WIDTH, LCD_HEIGHT);
            compasSprite.setColorDepth(16);
            compasSprite.setSwapBytes(true);
          }
          if (!compas2Sprite.created()) 
          {
            compas2Sprite.createSprite(LCD_WIDTH, LCD_HEIGHT);
            compas2Sprite.setColorDepth(16);
            compas2Sprite.setSwapBytes(true);
          }
        }
        else 
        {
          TFT_view_mode = VIEW_MODE_TEXT;
          if (!bearingSprite.created()) 
          {
            bearingSprite.createSprite(78, 54);
            bearingSprite.setColorDepth(16);
            bearingSprite.setSwapBytes(true);
          } 
        }
        lcd_brightness(MAX_BRIGHTNESS);
        TFTrefresh = true;
        TFTTimeMarker = millis() + 1001;
        xSemaphoreGive(spiMutex);
        delay(10);
      }
      else
      {
        PRINTLN("Failed to acquire SPI semaphore!");
      }
    }
  }
  else if (TFT_view_mode == VIEW_MODE_TEXT) 
  {
    if (next && show_compass) 
    {
      if (xSemaphoreTake(spiMutex, portMAX_DELAY)) 
      {
        TFT_view_mode = VIEW_MODE_COMPASS;
        if (!compasSprite.created()) 
        {
          compasSprite.createSprite(LCD_WIDTH, LCD_HEIGHT);
          compasSprite.setColorDepth(16);
          compasSprite.setSwapBytes(true);
        }
        if (!compas2Sprite.created()) 
        {
          compas2Sprite.createSprite(LCD_WIDTH, LCD_HEIGHT);
          compas2Sprite.setColorDepth(16);
          compas2Sprite.setSwapBytes(true);
        }
        if (!compas2Sprite.created()) 
        {
          compas2Sprite.createSprite(LCD_WIDTH, LCD_HEIGHT);
          compas2Sprite.setColorDepth(16);
          compas2Sprite.setSwapBytes(true);
        }
        lcd_brightness(MAX_BRIGHTNESS);
        TFTTimeMarker = millis() + 1001;
        TFTrefresh = true;
        xSemaphoreGive(spiMutex);
        delay(10);
        // EPD_display_frontpage = false;
      }
      else 
      {
        PRINTLN("Failed to acquire SPI semaphore!");
      }
    }
    else 
    {
      if (xSemaphoreTake(spiMutex, portMAX_DELAY)) 
      {
        TFT_view_mode = VIEW_MODE_RADAR;
        bearingSprite.deleteSprite();
        TFTTimeMarker = millis() + 1001;
        xSemaphoreGive(spiMutex);
        delay(10);
        // EPD_display_frontpage = false;
      }
      else 
      {
        PRINTLN("Failed to acquire SPI semaphore!");
      }
    }
  }
  else if (TFT_view_mode == VIEW_MODE_COMPASS) 
  {
    if (next) 
    {
      if (xSemaphoreTake(spiMutex, portMAX_DELAY)) 
      {
        TFT_view_mode = VIEW_MODE_RADAR;
        compasSprite.deleteSprite();
        compas2Sprite.deleteSprite();
        // sprite.createSprite(LCD_WIDTH, LCD_HEIGHT);
        // sprite.setColorDepth(16);
        // sprite.setSwapBytes(true);
        lcd_brightness(MAX_BRIGHTNESS);
        TFTTimeMarker = millis() + 1001;
        // EPD_display_frontpage = false;
        xSemaphoreGive(spiMutex);
        delay(10);
      }
      else 
      {
        PRINTLN("Failed to acquire SPI semaphore!");
      }
    }
    else
    {
      if (xSemaphoreTake(spiMutex, portMAX_DELAY)) 
      {
        TFT_view_mode = VIEW_MODE_TEXT; 
        compasSprite.deleteSprite();
        compas2Sprite.deleteSprite();
        // sprite.createSprite(LCD_WIDTH, LCD_HEIGHT);
        // sprite.setColorDepth(16);
        // sprite.setSwapBytes(true);
        // bearingSprite.createSprite(78, 54);
        // bearingSprite.setColorDepth(16);
        // bearingSprite.setSwapBytes(true);
        lcd_brightness(MAX_BRIGHTNESS);
        TFTTimeMarker = millis() + 1001;
        // EPD_display_frontpage = false;
        xSemaphoreGive(spiMutex);
        delay(10);
      }
      else
      {
        PRINTLN("Failed to acquire SPI semaphore!");
      }
    }
  }
  else if (TFT_view_mode == VIEW_MODE_SETTINGS) 
  {
    if (next) 
    {
      TFT_view_mode = prev_TFT_view_mode;
      EPD_display_frontpage = false;
    }
  }
}

void TFT_Up()
{
  if (hw_info.display == DISPLAY_TFT) 
  {
    switch (TFT_view_mode)
    {
      case VIEW_MODE_RADAR:
        TFT_radar_unzoom();
        break;
      case VIEW_MODE_TEXT:
        TFT_text_prev();
        break;
      default:
        break;
    }
  }
}

void TFT_Down()
{
  if (hw_info.display == DISPLAY_TFT) 
  {
    switch (TFT_view_mode)
    {
    case VIEW_MODE_RADAR:
      TFT_radar_zoom();
      break;
    case VIEW_MODE_TEXT:
      TFT_text_next();
      break;
    default:
      break;
    }
  }
}

void settings_button(uint16_t x, uint16_t y, bool on) 
{
  if (on) 
  {
    sprite.fillSmoothRoundRect(x, y - 25, 50, 31, 13, TFT_BLUEBUTTON, TFT_BLACK);
    sprite.fillSmoothCircle(x + 33, y - 10, 13, TFT_WHITE, TFT_BLUEBUTTON);
  }
  else
  {
    sprite.fillSmoothRoundRect(x, y - 25, 50, 31, 13, TFT_DARKGREY, TFT_BLACK);
    sprite.fillSmoothCircle(x + 17, y - 10, 13, TFT_LIGHTGREY, TFT_BLUEBUTTON);
  }
}

// BLE Scanning task - runs asynchronously to avoid blocking the UI
void ble_scan_task(void *parameter)
{
  // Perform the BLE scan
  ble_scan_results = scanForBLEDevices(10);  // 10 second scan
  
  // Mark scanning as complete
  ble_scanning = false;
  
  // Refresh the display with results
  ble_manager_page();
  
  // Delete this task after completion
  vTaskDelete(NULL);
}

// Start BLE scanning
void start_ble_scan()
{
  if (!ble_scanning)
  {
    ble_scanning = true;
    ble_scan_results.clear();
    ble_selected_device = -1;
    
    // Create a task to handle BLE scanning asynchronously
    xTaskCreate(ble_scan_task, "BLE_Scan", 4096, NULL, 1, NULL);
  }
}

void settings_page_1()
{
  if (xSemaphoreTake(spiMutex, portMAX_DELAY)) 
  {
    delay(50);
    if (TFT_view_mode != VIEW_MODE_SETTINGS) 
    {
      prev_TFT_view_mode = TFT_view_mode; 
      TFT_view_mode = VIEW_MODE_SETTINGS;
    }
    uint16_t button_x = 340;
    uint16_t text_y = 0;
  
    sprite.fillSprite(TFT_BLACK);
    sprite.setTextColor(TFT_WHITE, TFT_BLACK);
    sprite.setTextDatum(MC_DATUM);
    sprite.setFreeFont(&Orbitron_Light_24);
    sprite.setCursor(160, 40);
    sprite.printf("Settings (1/2)");

    text_y = 120;
    sprite.setCursor(button_x - 300, text_y);
    sprite.printf("Alt Filter: %s",
      settings->filter == TRAFFIC_FILTER_OFF    ? "OFF" :
      settings->filter == TRAFFIC_FILTER_500M   ? "500m" :
      settings->filter == TRAFFIC_FILTER_1000M  ? "1000m" :
      settings->filter == TRAFFIC_FILTER_3500FT ? "3500ft" :
      settings->filter == TRAFFIC_FILTER_5000FT ? "5000ft" : "OFF");

    text_y = 175;
    sprite.setCursor(button_x - 300, text_y);
    sprite.printf("Compass Page");
    settings_button(button_x, text_y, show_compass);

    text_y = 230;
    sprite.setCursor(button_x - 300, text_y);
    sprite.printf("Radar North Up");
    settings_button(button_x, text_y, settings->orientation == DIRECTION_NORTH_UP);

    text_y = 285;
    sprite.setCursor(button_x - 300, text_y);
    sprite.printf("Show Labels");
    settings_button(button_x, text_y, isLabels);

    text_y = 340;
    sprite.setCursor(button_x - 300, text_y);
    sprite.printf("Arrowhead Icons");
    settings_button(button_x, text_y, settings->icon_style == ICON_STYLE_ARROWHEAD);

    sprite.setCursor(button_x - 120, 400);
    sprite.printf("Sleep");

    sprite.setCursor(button_x - 130, 440);
    sprite.printf("BACK");
    sprite.fillTriangle(180, 430, 197, 417, 197, 443, TFT_BLUEBUTTON);
    sprite.fillTriangle(160, 430, 180, 417, 180, 443, TFT_BLUEBUTTON);

    // Add pagination indicator - right arrow for next page (only if BLE is the connection type)
    if (settings->connection == CON_BLUETOOTH_LE)
    {
      sprite.setCursor(320, 440);
      sprite.printf("→");
    }

    sprite.setSwapBytes(true);
    sprite.pushImage(button_x, 380, 48, 47, power_button_small);
    
    lcd_PushColors(display_column_offset, display_row_offset, LCD_WIDTH, LCD_HEIGHT, (uint16_t*)sprite.getPointer());
    lcd_brightness(MAX_BRIGHTNESS);
    xSemaphoreGive(spiMutex);
  }
  else
  {
    LOG_ERROR("Failed to acquire SPI semaphore!");
  }
}

void settings_page()
{
  // Only show BLE Manager page if BLE is the active connection
  if (settings_page_num == 2 && settings->connection != CON_BLUETOOTH_LE)
  {
    settings_page_num = 1;
  }
  
  if (settings_page_num == 1)
  {
    settings_page_1();
  }
  else if (settings_page_num == 2 && settings->connection == CON_BLUETOOTH_LE)
  {
    ble_manager_page();
  }
}

void ble_manager_page()
{
  if (xSemaphoreTake(spiMutex, portMAX_DELAY)) 
  {
    delay(50);
    if (TFT_view_mode != VIEW_MODE_SETTINGS) 
    {
      prev_TFT_view_mode = TFT_view_mode; 
      TFT_view_mode = VIEW_MODE_SETTINGS;
    }

    sprite.fillSprite(TFT_BLACK);

    uint16_t button_x = 340;
    const int centerX = LCD_WIDTH / 2;
    const int titleY = 32;
    const int scanBtnX = 164;
    const int scanBtnY = 72;
    const int scanBtnW = 170;
    const int scanBtnH = 48;
    const int scanTextY = scanBtnY + (scanBtnH / 2) + 1;
    const int disconnectBtnX = 58;
    const int disconnectBtnY = 72;
    const int disconnectBtnW = 90;
    const int disconnectBtnH = 48;
    const int listX = 42;
    const int listY = 158;
    const int rowH = 44;
    const int nameW = 240;
    const int addBtnX = 292;
    const int addBtnW = 42;
    const int maxResultsShown = 4;

    sprite.setTextColor(TFT_WHITE, TFT_BLACK);
    sprite.setTextDatum(MC_DATUM);
    sprite.setFreeFont(&Orbitron_Light_24);
    sprite.drawString("BLE Manager", centerX, titleY, 1);

    sprite.setFreeFont(&FreeSansBold12pt7b);
    if (ble_scanning)
    {
      sprite.fillSmoothRoundRect(scanBtnX, scanBtnY, scanBtnW, scanBtnH, 10, TFT_DARKGREY, TFT_BLACK);
      sprite.drawRoundRect(scanBtnX, scanBtnY, scanBtnW, scanBtnH, 10, TFT_GREY);
      sprite.setTextColor(TFT_WHITE, TFT_BLACK);
      sprite.setTextDatum(MC_DATUM);
      sprite.drawString("Scanning...", scanBtnX + (scanBtnW / 2), scanTextY, 1);
    }
    else
    {
      sprite.fillSmoothRoundRect(scanBtnX, scanBtnY, scanBtnW, scanBtnH, 10, TFT_BLUEBUTTON, TFT_BLACK);
      sprite.drawRoundRect(scanBtnX, scanBtnY, scanBtnW, scanBtnH, 10, TFT_WHITE);
      sprite.setTextColor(TFT_WHITE, TFT_BLUEBUTTON);
      sprite.setTextDatum(MC_DATUM);
      sprite.drawString("Scan", scanBtnX + (scanBtnW / 2), scanTextY, 1);
    }

    if (ESP32_BT_ctl.status == BT_STATUS_CON)
    {
      sprite.fillSmoothRoundRect(disconnectBtnX, disconnectBtnY, disconnectBtnW, disconnectBtnH, 10, TFT_MAROON, TFT_BLACK);
      sprite.drawRoundRect(disconnectBtnX, disconnectBtnY, disconnectBtnW, disconnectBtnH, 10, TFT_RED);
      sprite.setTextColor(TFT_WHITE, TFT_MAROON);
      sprite.setTextDatum(MC_DATUM);
      sprite.drawString("DISC", disconnectBtnX + (disconnectBtnW / 2), disconnectBtnY + (disconnectBtnH / 2) + 1, 1);
    }

    if (ble_scan_results.empty())
    {
      sprite.setTextColor(TFT_LIGHTGREY, TFT_BLACK);
      sprite.setTextDatum(MC_DATUM);
      sprite.drawString("No devices found", centerX, 190, 1);
    }
    else
    {
      // Get list of whitelisted devices
      std::vector<String> whitelistedDevices = getAllowedBLENameList();
      
      for (int i = 0; i < ble_scan_results.size() && i < maxResultsShown; i++)
      {
        const int rowY = listY + i * rowH;
        const bool isSelected = (i == ble_selected_device);
        const String connectedName = getConnectedBLEDeviceName();
        
        // Check if this device is already whitelisted
        bool isWhitelisted = false;
        for (const auto& whitelistedName : whitelistedDevices)
        {
          if (whitelistedName == ble_scan_results[i])
          {
            isWhitelisted = true;
            break;
          }
        }
        const bool isConnectedDevice = (connectedName.length() > 0 && connectedName == ble_scan_results[i]);

        sprite.fillSmoothRoundRect(listX, rowY, 294, 36, 6, isSelected ? TFT_NAVY : TFT_BLACK, TFT_BLACK);
        sprite.drawRoundRect(listX, rowY, 294, 36, 6, isSelected ? TFT_CYAN : TFT_DARKGREY);

        int nameX = listX + (isConnectedDevice ? 18 : 10);
        int nameRightLimit = isConnectedDevice ? 242 : (addBtnX - 8);
        int maxNameWidth = nameRightLimit - nameX;

        String name = ble_scan_results[i];
        while (name.length() > 1 && sprite.textWidth(name, 1) > maxNameWidth)
        {
          name = name.substring(0, name.length() - 2) + "~";
        }

        sprite.setTextColor(isSelected ? TFT_CYAN : TFT_WHITE, TFT_BLACK);
        sprite.setTextDatum(TL_DATUM);
        sprite.drawString(name, nameX, rowY + 10, 1);

        if (isConnectedDevice)
        {
          sprite.fillCircle(listX + 8, rowY + 18, 4, TFT_GREEN);
          sprite.fillSmoothRoundRect(246, rowY + 6, 40, 24, 5, TFT_DARKGREEN, TFT_BLACK);
          sprite.drawRoundRect(246, rowY + 6, 40, 24, 5, TFT_GREEN);
          sprite.setTextColor(TFT_GREEN, TFT_DARKGREEN);
          sprite.setTextDatum(MC_DATUM);
          sprite.drawString("ON", 266, rowY + 18, 1);
        }

        // Show checkmark if whitelisted, + button if not
        if (isWhitelisted)
        {
          sprite.fillSmoothRoundRect(addBtnX, rowY + 4, addBtnW, 28, 5, TFT_DARKGREEN, TFT_BLACK);
          sprite.drawRoundRect(addBtnX, rowY + 4, addBtnW, 28, 5, TFT_GREEN);
          sprite.setTextColor(TFT_GREEN, TFT_DARKGREEN);
          sprite.setTextDatum(MC_DATUM);
          sprite.drawString("✓", addBtnX + 21, rowY + 18, 1);
        }
        else
        {
          sprite.fillSmoothRoundRect(addBtnX, rowY + 4, addBtnW, 28, 5, TFT_DARKGREEN, TFT_BLACK);
          sprite.drawRoundRect(addBtnX, rowY + 4, addBtnW, 28, 5, TFT_CYAN);
          sprite.setTextColor(TFT_WHITE, TFT_DARKGREEN);
          sprite.setTextDatum(MC_DATUM);
          sprite.drawString("+", addBtnX + 21, rowY + 18, 1);
        }
      }
    }

    // Back button - styled to match settings_page_1 exactly
    sprite.setFreeFont(&Orbitron_Light_24);
    sprite.setTextColor(TFT_WHITE, TFT_BLACK);
    sprite.setTextDatum(TL_DATUM);
    sprite.setCursor(button_x - 130, 440);
    sprite.printf("BACK");
    sprite.fillTriangle(180, 430, 197, 417, 197, 443, TFT_BLUEBUTTON);
    sprite.fillTriangle(160, 430, 180, 417, 180, 443, TFT_BLUEBUTTON);

    // Left arrow pagination indicator
    sprite.setFreeFont(&Orbitron_Light_24);
    sprite.setTextDatum(TL_DATUM);
    sprite.setTextColor(TFT_CYAN, TFT_BLACK);
    sprite.drawString("<", 32, 440, 4);
    
    // Device added confirmation message (show for 2 seconds)
    if (ble_device_added && (millis() - ble_add_confirmation_time) < 2000)
    {
      sprite.setFreeFont(&Orbitron_Light_24);
      sprite.setTextColor(TFT_GREEN, TFT_BLACK);
      sprite.setTextDatum(MC_DATUM);
      sprite.drawString("v Device Added", centerX, 120, 1);
    }
    else if (ble_device_added && (millis() - ble_add_confirmation_time) >= 2000)
    {
      ble_device_added = false;
    }
    
    lcd_PushColors(display_column_offset, display_row_offset, LCD_WIDTH, LCD_HEIGHT, (uint16_t*)sprite.getPointer());
    lcd_brightness(MAX_BRIGHTNESS);
    xSemaphoreGive(spiMutex);
  }
  else
  {
    LOG_ERROR("Failed to acquire SPI semaphore!");
  }
}
#endif /* USE_TFT */

