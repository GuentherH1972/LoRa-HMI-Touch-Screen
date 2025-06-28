# Changelog

## [1.1.0] - 2025-06-21

### Added

- Added commands for activating LoRaWAN network, firmware type retrieval, and configuration information retrieval for ESP32.
- Add support for Fport information to the panel data package sent to ESP32.
- Added feedback on the execution of upward commands on the button panel.

### Changed

- Change the default mode of LoRaWAN for LA66 from CLASS A to CLASS C.

## [1.0.0] - 2025-04-28

### Added
- Implement data processing and forwarding between the ESP32 and the LoRaWAN Gateway by modifying the LA66's existing LoRaWAN v1.3 firmware — this functionality requires the device to operate in Class C mode for proper operation.
