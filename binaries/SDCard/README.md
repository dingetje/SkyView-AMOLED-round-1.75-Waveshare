# SD.zip
Extract this archive in the root of an empty FAT32 SD card.
The folder structure should look like this:


<img width="123" height="187" alt="image" src="https://github.com/user-attachments/assets/9f8940db-4c65-44a3-92e4-282a33c7c4e2" />

  
## Notes
- Aircrafts folder is used to store the ogn.db.
- Voice folder stores the voice data (WAV format, 16-bit mono, 22050 sample rate).
  - voice1 is a slow male voice used for traffic advisories.
  - voice2 data is currently not used.
  - voice3 is a faster female voice for alert and danger messages.
  - There's also a startup.wav and power_on.wav file in the root of the Audio folder, used during power on. These are played regardless of the **Voice** setting.
- logs is not yet used, reserved for future log file(s).
- NMEA contains a test file, currently not used.
 
