# Preparing SD card

The waveshare board supports an SD card that can be used for both the OGN database and Voice data.

1. In the **binaries/SDCard** folder you can download the [SD.zip](../binaries/SDCard/SD.zip) archive (make sure to **Download raw file**).
2. unzip this archive in the root of a FAT32 formatted SD card [^1].

> [!WARNING]
> Make sure not to format the SD card with **exFAT** format, this is not supported on the Waveshare board!

4. This should result in a directory structure in the root of the SD card, similar to this:

   
   <img width="213" height="214" alt="image" src="https://github.com/user-attachments/assets/9e0fff79-8af3-44e0-83b2-1a5f9ed97965" />



## Using the OGN db
- Make sure in the SkyView settings via [http://192.168.1.1](http://192.168.1.1) to set **Aircraft data** to **Glidernet (OGN)**  

  
  <img width="571" height="32" alt="image" src="https://github.com/user-attachments/assets/02e131de-e4d7-4cbf-939c-7a04b35fcf0f" />



## Enabling voice output
1. Make sure to have a speaker attached to the **SPK** output of the Waveshare board (🔉).
2. Make sure to have the SD card mounted with the **Voice** files.
3. in the SkyView settings via [http://192.168.1.1](http://192.168.1.1) enable one of the voice options at **Voice**


   <img width="560" height="295" alt="image" src="https://github.com/user-attachments/assets/e9be2fd3-3ac3-462f-885d-660831178ff4" />




## Updating the OGN db
To update the OGN db follow these steps:
1. Download most recent .CDB file that contains OGN Aircrafts Data from [this location](http://soaringweather.no-ip.info/ADB/data/ogn.cdb)
2. Eject the SD card from your Waveshareboard and mount in on your PC using a suitable SD card reader
3. Copy the downloaded ogn.cdb in the **Arcrafts** folder, overwriting the existing file
4. Safe eject the SD card
5. Place the SD card back in the Waveshare board
6. Verify on the status page at [http://192.168.1.1](http://192.168.1.1) that the OGN records show a non-zero value

   
   <img width="566" height="31" alt="image" src="https://github.com/user-attachments/assets/ff697961-3754-48fb-a865-4109986f107b" />


   [^1]: the maximum size for the SD card under Windows is 32GB for FAT32 format, exFAT is not supported!
