# LilyGo-EPD-4-7-OWM-Weather-Display

WORK IN PROGRESS

My take on #LilyGo-EPD-4-7-OWM-Weather-Display - adapting for own purposes:
- switched to freertos to faciltate concurrent sensor readings, calculations and future tasks between light sleep periods.
- introduced i2c bme280/sht40 sensors for localized readouts (libraries: https://github.com/UncleRus/esp-idf-lib)
- simple interrupt logic for multiple screens handling
- slightly improved redability (still much to be desired)

Planned:
- ESP-NOW transmission handling
- db/web server for historic data access/sharing
- command repeating via ESP-NOW



[How to get started quickly](https://youtu.be/bTdW0QTEK70)

# FAQ
1. After flash is completed, burning cannot continue. Please put the device into download mode.
   1. Connect the board via the USB cable
   2. Press and hold the BOOT(**IO0**) button , While still pressing the BOOT(**IO0**) button, press **REST**
   3. Release the **REST**
   4. Release the BOOT(**IO0**) button
   5. Upload sketch

