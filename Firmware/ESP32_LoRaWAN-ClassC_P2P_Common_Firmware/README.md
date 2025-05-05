# Changelog

## [1.0.0] - 2025-04-28

### Added
- Add two pages and implement a toggle button to switch between them.

[UI Features]

- Added HOME/SETTING pages with toggle button navigation
- Implemented dynamic panel system supporting 6 sensor node types (max 147 panels)
- Sort panels by creation/update time or battery level with priority weighting. 
- Real-time panel counter in header

[Interaction]

- Long-press gesture to delete panels
- Slider control for screen brightness adjustment
- Temperature unit toggle (℃/℉) with instant UI update

[Data Integration]

- Panel management via UART from the LA66 module (using the data format documented in the "P2P" and "LoRaWAN_CLASS_C" chapters of the [LTS5 LoRa HMI Touch Screen Wiki](https://wiki.dragino.com/xwiki/bin/view/Main/User%20Manual%20for%20LoRaWAN%20End%20Nodes/LTS5%20LoRa%20Touch%20Screen/)).
  - Automatically generate panels for unregistered sensor types.  
  - Update existing panels if the DEUI, panel type, and device name all match. 

[System Configuration]

- Boot animation enable/disable option
- Settings persistence (brightness, temperature unit, boot animation) via Save button
- Device info section in SETTING:
  - Model: LTS5
  - Version: 1.0.0
  - Boot Time (dd:hh:mm:ss): 01:12:30:45

