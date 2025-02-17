Product Name: `Leany`

PCB Version: `rev0.3`

Release Date: `2025-01-21`

Release Type: `Major Update`

Firmware version Compatibility: `v0.1.2`

## New Features
- Added support for I2C multi-master configuration.
- Enabled BLE 5.0 support.

## Bug Fixes
- Resolved SPI timing issue (#7890).
- Fixed crash when exceeding 100k ADC samples.

## Testing
- Oscilloscope
- Digital analyser

## Known Issues
- PWM frequency mismatch on channel 3 under high load.
