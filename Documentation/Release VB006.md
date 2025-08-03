### Release VB006

- Revamped battery/power management using the built-in SY6970 chip
- Bluetooth now uses the NimBLE stack, reducing power consumption
- Improved UI: tap on aircraft to see details

---

## What's New in VB006

1. **Battery Charging and Power Management**

    - Battery charging is now slower to improve battery life. A charge indicator icon shows when the battery is charging or fully charged.
    - Charging current near 100% is deliberately lower to improve energy absorption (be patient and charge until the icon disappears and the battery icon turns green).
    - Additional battery indicator shows external BLE device battery status (such as SoftRF T1000E).
    - Battery log: a log is stored for each battery discharge cycle, so you can see how long the battery lasted from full to low. The battery log can be retrieved from the web interface.

    <div align="left">

    ![Battery_image](images/SkyView-battery-image.jpg)

    </div>

---

2. **BLE Stack Improvements**

    - With NimBLE, power consumption is improved.
    - When scanning for BLE devices, the scan results only show FLARM/FANET-capable devices.

---

3. **Tap on Target Aircraft**

    - Tap on an aircraft in the Radar screen to bring up its Info Card.
    - The Info Card becomes locked to this aircraft (hint: little green lock icon). This ensures the aircraft card is always displayed as long as the aircraft is in view.
    - To move to another aircraft, tap the green lock icon.
    - Or swipe left to return to the Radar screen.

    <div align="left">

    ![TapOnTarget](images/SkyView-TapOntarget.gif)

    </div>

---

4. **Lock on Aircraft**

    - Click anywhere around the aircraft ID to lock the page onto this aircraft. A green lock will appear.
    - This means the Info will be shown only for this aircraft and will not change (as long as data is being received from this aircraft).
    - Tapping the lock icon will return to unlocked mode.

    <div align="left">

    ![Info_lock](images/SkyView-lock_s.jpg)

    </div>

---

***Easy Firmware Update***

- Watch the video above for instructions on how to **update SkyView firmware** via the built-in **Web Interface**.  
  https://youtu.be/NbYAM7KYvmo

Link to the new Firmware:  
<span style="font-size: smaller;">

[SkyView_firmware_VB006.bin](https://github.com/slash-bit/SkyView-AMOLED-round-1.75-TFT_eSPI/blob/main/binaries/SkyView_firmware_VB006.bin)
</span>

---
