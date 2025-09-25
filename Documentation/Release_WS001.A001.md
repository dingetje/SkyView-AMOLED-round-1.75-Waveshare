# First release
This is the first release for the Waveshare AMOLED 1.75 Round board.
It is based on the VB006 firmware for the LilyGo board: [https://github.com/slash-bit/SkyView-AMOLED-round-1.75-TFT_eSPI/](https://github.com/slash-bit/SkyView-AMOLED-round-1.75-TFT_eSPI/) but has a few modifications and enhancements.

## PWR button
- Use PWR button to power on/off the device:
  - **very long** press (~7 seconds) to power on.
  - **long press** (~3 sec) to power off.
## BOOT button
- Use BOOT button as MODE button to switch to next screen.

## OGN database
- Offers OGN device identification via ogn.cdb file on the SD card.
- In addition offers a Buddy List (can be modified via the Web Server).
- Shows OGN records on the Web Server status page (if enabled and ogn.db found on SD card).

## Audio
- Offers audio traffic information and danger alerts if:
  - Speaker is connected to the SPK output of the board (the board comes with a suitable tiny speaker).
  - SD card with Audio files is inserted.
  - Voice is enabled in **Settings** page of the Web Server.

## Battery
- The Waveshare board uses an AXP2101 PMU for the battery management.
- Battery usage is higher than the LilyGo board (~180mA), a 3.7V/1100mAh Lithium battery lasts at least 6 hours.
