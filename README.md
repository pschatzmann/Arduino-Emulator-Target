# Arduino-Emulator-Target

Arduino library and example sketch for the [Arduino-Emulator](https://github.com/pschatzmann/Arduino-Emulator).

The sketch just runs the commands sent vial UDP from the emulator on a ESP32 microcontroller.

Please note that this library is a POC and has not been extensively tested.


## Usage

1. Install/copy this folder into your Arduino `libraries` directory.
2. Open the example sketch:
	 - `File` -> `Examples` -> `Arduino-Emulator-Target` -> `EmulatorTarget`
3. Set `wifi_ssid` and `wifi_pwd` in the sketch.
4. Compile/upload to your board.

## Notes

- GPIO/SPI/Serial command handling is implemented.
- Some commands are intentionally marked as not supported and logged.
