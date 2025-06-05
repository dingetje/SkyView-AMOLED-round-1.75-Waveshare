# Flashing the T-Display AMOLED Board for the First Time

Follow this process to flash the board for the first time:

1. **Download the required files**:  
    - Navigate to the "binaries" folder in this repository.  
    - Download the files and place them in a folder on your PC, e.g., `C:\temp\SkyView`.

2. **Download the ESP32 Flashing Tool**:  
    - Visit the [Espressif website](https://docs.espressif.com/projects/esp-test-tools/en/latest/esp32/production_stage/tools/flash_download_tool.html) to download the ESP32 Flashing Tool.

3. **Set up the Flashing Tool**:  
    - When starting the tool, specify the board as `ESP32-S3`.  
    - Refer to the screenshot below and configure the files and addresses exactly as shown.


![alt text](https://github.com/slash-bit/SkyView-AMOLED-round-1.75-TFT_eSPI/tree/main/Documentation/images/AMOLED_Board_Flashing_setting.png "Board Selection")


4. **Select the COM Port**:  
    - In the `COM:` dropdown list, select the COM port of the AMOLED board (after connecting it to your PC).

5. **Erase the Board**:  
    - Click the **Erase** button and wait until it says "Finished" (this may take a few seconds).  
    - Optionally, press the reset button on the board afterward.

6. **Flash the Firmware**:  
    - Click the **Start** button to begin flashing the firmware.  
    - Wait until it says "Finished."

7. **Power Cycle the Board**:  
    - Press the reset button to power cycle the board.

The SkyView software is now installed.  

For subsequent firmware updates, you can either repeat this method or use the much easier WebFlash process.


![alt text](https://github.com/slash-bit/SkyView-AMOLED-round-1.75-TFT_eSPI/tree/main/Documentation/images/Flashing_Tool_Settings.png "Files and addresses")