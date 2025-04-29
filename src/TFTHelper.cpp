#if defined(USE_TFT)
#include "TFT_eSPI.h"
// #include "Free_Fonts.h"
#include "SoCHelper.h"
#include "EEPROMHelper.h"
#include "TrafficHelper.h"
#include "TFTHelper.h"
#include "WiFiHelper.h"
#include "BatteryHelper.h"
// #include <Adafruit_GFX.h>    // Core graphics library
// #include "Arduino_GFX_Library.h"
#include "Arduino_DriveBus_Library.h"
#include "TouchDrvCST92xx.h"
#include <../pins_config.h>
#include <driver/display/CO5300.h>
#include "power.h"

// #include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
// #include <Adafruit_ST77xx.h> // Hardware-specific library for ST7789

// #include <Fonts/GFXFF/FreeSansBold12pt7b.h>
// #include <Fonts/GFXFF/FreeMonoBold24pt7b.h>


#include "SkyView.h"
int TFT_view_mode = 0;
unsigned long TFTTimeMarker = 0;
bool EPD_display_frontpage = false;

int prev_TFT_view_mode = 0;
extern bool wifi_sta;



#if defined(AMOLED)



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

std::shared_ptr<Arduino_IIC_DriveBus> IIC_Bus =
  std::make_shared<Arduino_HWIIC>(IIC_SDA, IIC_SCL, &Wire);
#elif defined H0175Y003AM
xSemaphoreHandle spiMutex;
TFT_eSPI tft = TFT_eSPI();
TFT_eSprite sprite = TFT_eSprite(&tft);
TFT_eSprite sprite2 = TFT_eSprite(&tft);
TFT_eSprite bearingSprite = TFT_eSprite(&tft);
TFT_eSprite batterySprite = TFT_eSprite(&tft);
TFT_eSprite compasSprite = TFT_eSprite(&tft);
TFT_eSprite compas2Sprite = TFT_eSprite(&tft);

std::shared_ptr<Arduino_IIC_DriveBus> IIC_Bus =
  std::make_shared<Arduino_HWIIC>(IIC_SDA, IIC_SCL, &Wire);


#else
#error "Unknown macro definition. Please select the correct macro definition."
#endif

  
#endif
// TFT_eSPI tft = TFT_eSPI();
// Adafruit_ST7789 tft = Adafruit_ST7789(SOC_GPIO_PIN_SS_TFT, SOC_GPIO_PIN_DC_TFT, SOC_GPIO_PIN_MOSI_TFT, SOC_GPIO_PIN_SCK_TFT, -1);
buddy_info_t buddies[] = {
  { 0x201076, "XCT_Vlad" },
  { 0x86D7FD, "T-Echo" },
  { 0xE18990, "ESP-Stick" },
  { 0x46CBDC, "Sense CapT1000" },
  { 0x6254B0, "SenseCap2"},
  { 0x2006CD, "Tim Pentreath" },
  { 0x201172, "Steve Wagner"},
  { 0x201066, "Katrina Wagner"},
  { 0x20069D, "Steve Wagner"},
  { 0x2006A8, "Katrina Wagner"},
  { 0x111F40, "Chris H" },
  { 0x1139D4, "Tom K" },
  { 0xFFFFFFFF, NULL } // Sentinel value
};

unsigned long drawTime = 0;

float batteryToPercentage(float voltage) {
  if (voltage >= 4.2) return 100.0;
  if (voltage <= 3.0) return 0.0;

  // Extended voltage-percentage mapping
  float voltageLevels[] = {4.14, 4.00, 3.9, 3.8, 3.7, 3.6, 3.5, 3.4, 3.3, 3.2, 3.0};
  float percentages[] = {100, 95, 90, 75, 55, 40, 25, 10, 5, 2, 0};

  for (int i = 0; i < 11; i++) { // Loop through voltage levels
      if (voltage > voltageLevels[i + 1]) {
          // Linear interpolation between two points
          return percentages[i] + ((voltage - voltageLevels[i]) /
              (voltageLevels[i + 1] - voltageLevels[i])) * (percentages[i + 1] - percentages[i]);
      }
  }
  return 0.0; // Fallback
}
void draw_battery() {
    //Battery indicator
    uint16_t battery_x = 295;
    uint16_t battery_y = 35;
    float battery = 0;
    uint8_t batteryPercentage = 0;
    uint16_t batt_color = TFT_CYAN;
      // draw battery symbol
   
    
    battery = Battery_voltage();
    Serial.print(F(" Battery= "));  Serial.println(battery);
    batteryPercentage = (int)batteryToPercentage(battery);
    // float percentage = (((battery - 3.2) + 0.001) / (4.2 - 3.2)) * 100.0;
    // batteryPercentage = percentage > 100.0 ? 100 : round(percentage);
    Serial.print(F(" Batterypercentage= "));  Serial.println(batteryPercentage);

    if (battery < 3.65 &&  battery >= 3.5) {
      batt_color = TFT_YELLOW;
    } else if (battery < 3.5) {
      batt_color = TFT_RED;
    } else  {
      batt_color = TFT_CYAN;
    }
    sprite.drawRoundRect(battery_x, battery_y, 32, 20, 3, batt_color);
    sprite.fillRect(battery_x + 32, battery_y + 7, 2, 7, batt_color);
    int fillWidth = (int)(30 * ((float)batteryPercentage / 100));
    Serial.print(F("Fill width = "));
    Serial.println(fillWidth);
    sprite.fillRect(battery_x + 2, battery_y + 3, fillWidth, 14, batt_color);
    // sprite.fillRect(battery_x + 2, battery_y + 3, (int)(30 * (batteryPercentage / 100)), 14, batt_color);
    sprite.setCursor(battery_x, battery_y + 24, 4);
    sprite.setTextColor(TFT_WHITE, TFT_BLACK);
    sprite.printf("%d%%", batteryPercentage); // Use %% to print the % character
}

void draw_first()
{
  sprite.fillSprite(TFT_BLACK);
  sprite.setTextDatum(MC_DATUM);
  sprite.setTextColor(TFT_WHITE, TFT_BLACK);
  sprite.setFreeFont(&Orbitron_Light_32);
  sprite.setCursor(144, 160);
  // sprite.setTextSize(2);
  sprite.printf("SkyView");
  Serial.print("SkyView width: ");
  Serial.println(sprite.textWidth("SkyView"));
  Serial.print("SkyView height: ");
  Serial.println(sprite.fontHeight(4));
  sprite.setFreeFont(&FreeSansBold12pt7b);
  Serial.print("powered by... width: ");
  Serial.println(sprite.textWidth("powered by SoftRF"));
  Serial.print("powered by ... height: ");
  Serial.println(sprite.fontHeight(4));
  sprite.fillRect(114,200,66,66,TFT_RED);
  sprite.fillRect(200,200,66,66,TFT_GREEN);
  sprite.fillRect(286,200,66,66,TFT_BLUE); 
  sprite.setTextSize(1);

  sprite.drawString("powered by SoftRF",233,293,4);
  lcd_PushColors(6, 0, 466, 466, (uint16_t*)sprite.getPointer());
  for (int i = 0; i <= 255; i++)
  {
    lcd_brightness(i);
    delay(3);
  }
  delay(2000);

}

void TFT_setup(void) {
  pinMode(LCD_EN, OUTPUT);
  digitalWrite(LCD_EN, HIGH);
  delay(30);
  pinMode(SENSOR_RST, OUTPUT);
  digitalWrite(SENSOR_RST, LOW);
  delay(30);
  digitalWrite(SENSOR_RST, HIGH);
  delay(50);
  Wire.begin(SENSOR_SDA, SENSOR_SCL);
  CO5300_init();
  sprite.setColorDepth(16);
  Serial.print("TFT_setup. PSRAM_ENABLE: ");
  Serial.println(sprite.getAttribute(PSRAM_ENABLE));
  sprite.setAttribute(PSRAM_ENABLE, 1);
  // Initialise SPI Mutex
  spiMutex = xSemaphoreCreateMutex();
  if (spiMutex == NULL) {
      Serial.println("Failed to create SPI mutex!");
  }
  lcd_setRotation(0); //adjust #define display_column_offset for different rotations
  lcd_brightness(0); // 0-255    

  Serial.printf("Free heap: %d bytes\n", esp_get_free_heap_size());
  Serial.printf("Largest block: %d bytes\n", heap_caps_get_largest_free_block(MALLOC_CAP_8BIT));
  sprite.createSprite(466, 466);    // full screen landscape sprite in psram
  batterySprite.createSprite(100, 32);
  if (sprite.createSprite(466, 466) == NULL) {
    Serial.println("Failed to create sprite. Not enough memory.");
    delay(5000);
  }
  else {
    Serial.print("TFT_setup. Created Sprite| Free Heap: ");
    Serial.println(esp_get_free_heap_size());
  }

  TFT_view_mode = settings->vmode;
  draw_first();
  TFT_radar_setup();
}

void TFT_loop(void) {
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
  if (hw_info.display == DISPLAY_TFT) {

    if (TFT_view_mode == VIEW_MODE_RADAR) {
      if (next) {
        TFT_view_mode = VIEW_MODE_TEXT;
        bearingSprite.createSprite(78, 54);
        bearingSprite.setColorDepth(16);
        bearingSprite.setSwapBytes(true);
        TFTTimeMarker = millis() + 1001;
      // EPD_display_frontpage = false;
      }
      else {
        TFT_view_mode = VIEW_MODE_COMPASS;
        // sprite.deleteSprite();
        compasSprite.createSprite(466, 466);
        compasSprite.setColorDepth(16);
        compasSprite.setSwapBytes(true);
        compas2Sprite.createSprite(466, 466);
        compas2Sprite.setColorDepth(16);
        compas2Sprite.setSwapBytes(true);
        lcd_brightness(255);
        TFTTimeMarker = millis() + 1001;
        // EPD_display_frontpage = false;
      }

}   else if (TFT_view_mode == VIEW_MODE_TEXT) {
        if (next) {
          TFT_view_mode = VIEW_MODE_COMPASS;
          // sprite.deleteSprite();
          bearingSprite.deleteSprite();
          compasSprite.createSprite(466, 466);
          compasSprite.setColorDepth(16);
          compasSprite.setSwapBytes(true);
          compas2Sprite.createSprite(466, 466);
          compas2Sprite.setColorDepth(16);
          compas2Sprite.setSwapBytes(true);
          lcd_brightness(255);
          TFTTimeMarker = millis() + 1001;
          // EPD_display_frontpage = false;
        }
        else {  
          TFT_view_mode = VIEW_MODE_RADAR;
          bearingSprite.deleteSprite();
          TFTTimeMarker = millis() + 1001;
          // EPD_display_frontpage = false;
      }
    }
    else if (TFT_view_mode == VIEW_MODE_COMPASS) {
      if (next) {
        if (xSemaphoreTake(spiMutex, portMAX_DELAY)) {
          TFT_view_mode = VIEW_MODE_RADAR;
          compasSprite.deleteSprite();
          compas2Sprite.deleteSprite();
          sprite.createSprite(466, 466);
          sprite.setColorDepth(16);
          sprite.setSwapBytes(true);
          lcd_brightness(255);
          TFTTimeMarker = millis() + 1001;
          // EPD_display_frontpage = false;
          xSemaphoreGive(spiMutex);
          delay(10);
        } else {
          Serial.println("Failed to acquire SPI semaphore!");
        }
      }
      else {
        if (xSemaphoreTake(spiMutex, portMAX_DELAY)) {
          TFT_view_mode = VIEW_MODE_TEXT; 
          compasSprite.deleteSprite();
          compas2Sprite.deleteSprite();
          sprite.createSprite(466, 466);
          sprite.setColorDepth(16);
          sprite.setSwapBytes(true);
          bearingSprite.createSprite(78, 54);
          bearingSprite.setColorDepth(16);
          bearingSprite.setSwapBytes(true);
          lcd_brightness(255);
          TFTTimeMarker = millis() + 1001;
          // EPD_display_frontpage = false;
          xSemaphoreGive(spiMutex);
          delay(10);
        } else {
          Serial.println("Failed to acquire SPI semaphore!");
        }
    }
  }
    else if (TFT_view_mode == VIEW_MODE_SETTINGS) {
      if (next) {
        TFT_view_mode = prev_TFT_view_mode;
        EPD_display_frontpage = false;
      }

    }
  }
}

void TFT_Up()
{
  if (hw_info.display == DISPLAY_TFT) {
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
  if (hw_info.display == DISPLAY_TFT) {
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

void settings_button(uint16_t x, uint16_t y, bool on){
  switch (on) {
    case true:
      sprite.fillSmoothRoundRect(x, y - 25, 50, 31, 13, TFT_BLUEBUTTON, TFT_BLACK);
      sprite.fillSmoothCircle(x + 33, y - 10, 13, TFT_WHITE, TFT_BLUEBUTTON);
      break;
    case false:
      sprite.fillSmoothRoundRect(x, y - 25, 50, 31, 13, TFT_DARKGREY, TFT_BLACK);
      sprite.fillSmoothCircle(x + 17, y - 10, 13, TFT_LIGHTGREY, TFT_BLUEBUTTON);
      break;
    default:
      break;
  }
}

void settings_page() {
  if (xSemaphoreTake(spiMutex, portMAX_DELAY)) {
    delay(50);
    if (TFT_view_mode != VIEW_MODE_SETTINGS) {
      prev_TFT_view_mode = TFT_view_mode; 
      TFT_view_mode = VIEW_MODE_SETTINGS;}
    uint16_t button_x = 340;
    uint16_t text_y = 0;
  
    sprite.fillSprite(TFT_BLACK);
    sprite.setTextColor(TFT_WHITE, TFT_BLACK);
    sprite.setTextDatum(MC_DATUM);
    sprite.setFreeFont(&Orbitron_Light_24);
    sprite.setCursor(160, 40);
    sprite.printf("Settings");

    text_y = 140; //bottom of the text
    sprite.setCursor(button_x - 300, text_y);
    sprite.printf("Traffic filter 500m");
    if ( settings->filter  == TRAFFIC_FILTER_500M) {
      settings_button(button_x, text_y, true);
    } else {
      settings_button(button_x, text_y, false); 
    }

    text_y = 200;
    sprite.setCursor(button_x - 300, text_y);
    sprite.printf("Compass Page");
    if (show_compass) {
      settings_button(button_x, text_y, true);
    } else {
      settings_button(button_x, text_y, false); 
    }
    
    text_y = 260;
    sprite.setCursor(button_x - 300, text_y);
    sprite.printf("Radar North Up");
    if (settings->orientation == DIRECTION_NORTH_UP) {
      settings_button(button_x, text_y, true);
    } else {
      settings_button(button_x, text_y, false); 
    }

    text_y = 320;
    sprite.setCursor(button_x - 300, text_y);
    sprite.printf("Enable Wifi STA");

    if (wifi_sta) {
      settings_button(button_x, text_y, true);

    } else {
      settings_button(button_x, text_y, false);

    }


    text_y = 400;
    sprite.setCursor(button_x - 120, 380);
    sprite.printf("Sleep");

    sprite.setCursor(button_x - 130, 440);
    sprite.printf("BACK");
    sprite.fillTriangle(180, 430, 197, 417, 197, 443, TFT_BLUEBUTTON);
    sprite.fillTriangle(160, 430, 180, 417, 180, 443, TFT_BLUEBUTTON);

    sprite.setSwapBytes(true);
    sprite.pushImage(button_x, 350, 48, 47, power_button_small);
    
    lcd_PushColors(display_column_offset, 0, 466, 466, (uint16_t*)sprite.getPointer());
    lcd_brightness(255);
    xSemaphoreGive(spiMutex);
} else {
    Serial.println("Failed to acquire SPI semaphore!");
}

}
#endif /* USE_TFT */