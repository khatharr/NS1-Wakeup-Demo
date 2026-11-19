# NS1 Wakeup Demo

This project uses an ESP32 to emulate a Pro Controller for the original Nintendo Switch, referred to as Switch 1 in this project, including the communication flow used to wake the console from sleep mode.

## Requirements

- A board with an ESP32 chip revision 3
- Windows PC
- PlatformIO Core
- SDL3

`NS Input Passthrough`, which is used for testing, loads `SDL3.dll` from the same directory as the executable.
If the file is not present there, it loads the DLL from the Steam installation directory.

## Supported Environment and Limitations

The current implementation has been tested with an ESP32 chip revision 3 and ESP-IDF 5.5.0.
The firmware has been confirmed to build with ESP-IDF 5.5.0 on PlatformIO.

Because some Bluetooth controller settings cannot be configured through the public ESP-IDF APIs, the firmware directly accesses ROM and shared memory.
As a result, it may not work correctly on other ESP32 revisions.

## Setup

### Flash the Firmware to the ESP32

Run the following command from the project root directory.
Replace `COMxxx` with the COM port assigned to the ESP32.

```powershell
pio run --target upload --upload-port COMxxx
```

### Pair the ESP32 with the Switch as a Pro Controller

The controller must be paired with the Switch before it can wake the console from sleep mode.

Launch `NS Input Passthrough`, select the ESP32's COM port, and click `Open Port`.
To pair the controller again, click `Pair`.

```powershell
& '.\input_passthrough\bin\NS Input Passthrough.exe'
```

### Test Input Passthrough

Select `NS1` or `NS2` to match the target Switch console.
While the Switch is in sleep mode, press the HOME or PS button on a controller connected to the PC to start the wake-up and reconnection process.
When the ESP32 is not connected to the Switch, pressing any button other than HOME starts the normal reconnection process.
You can also manually start each connection operation using `Reconnect`, `Wake Up`, and `Disconnect` in the GUI.

To use the Switch 2 wake-up feature, select `NS2`, click `BLE Scan`, and capture the wake-up packet in advance by waking the Switch 2 with a Switch 2 controller that is already paired with it.
After the wake-up packet has been saved, the ESP32 can wake the Switch 2 from sleep mode.

## Directory Structure

| Path | Description |
|---|---|
| `src/main.c` | Initializes Bluetooth and creates tasks |
| `src/bt/` | Manages pairing, connection state, and HID connections |
| `src/hid/` | Processes Pro Controller HID reports |
| `src/bridge/` | Receives UART frames and processes commands |
| `src/wake/` | Manages wake-up communication and connection cycles |
| `src/baseband/` | Directly accesses the Bluetooth controller's ROM and shared memory |
| `input_passthrough/` | Test tool that sends input from a Windows PC game controller to the ESP32 |
| `docs/` | [Switch 1 wake-up protocol documentation](docs/wakeup-protocol.en.md) |

## License

The source code and documentation included in this project are provided under the [CC BY-NC 4.0](https://creativecommons.org/licenses/by-nc/4.0/) license.

You may use this project in your own work for noncommercial purposes, provided that you give appropriate credit and indicate any changes.
