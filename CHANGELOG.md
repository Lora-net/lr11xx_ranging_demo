# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [v1.2.0] - 2025-03-15

### Added

- Encapsulate the software timer based on the system tick handler.
- Add "make help" command.
- Calculate the pathloss exponent value according to RTToF result. Print it in the JSON log and show it on the LCD display. 

### Changed

- Use the software timer instead of the MCU timer which was not accurate expecially for small timing.
- Use the software timer instead of the "app_ranging_timer()" which was used in ranging process.
- Calculate "RANGING_ALL_SYMBOL" dynamically. So, now, the user can set the preamble length as they want. No longer limited 12 symbols. Simultaneously modify the README file.
- Just handle the needed interrupts, in avoid of breaking the code logic.
- Change the logic when the subordinate arose a "LR11XX_SYSTEM_IRQ_RTTOF_REQ_DISCARDED" interrupt.

### Fixed

- Since there is no "LR11XX_SYSTEM_IRQ_RTTOF_TIMEOUT" interrupt at subordinate side, add the "LR11XX_SYSTEM_IRQ_RTTOF_REQ_VALID" interrupt and a software timer to handle timeout logic.
- Recalculate the ranging time once. Two additional time considerations, one is PA ramp time and the other is TCXO startup time.
- Fix the issue the display would change the texts until the next ranging came after pushing the "Blue" button to go into relative mode.
- Fix an issue the subsequent ranging will be affected if one channel fails in the hopping array.
- Need to add two more symbols in the calculation of ranging symbols while using SF5 or SF6. 

## [v1.1.2] - 2024-12-26

### Changed

-  Modify the README file. 

## [v1.1.1] - 2024-11-05

### Changed

- Remove the "$" symbol from README file.

## [v1.1.0] - 2024-09-20

### Added

- Add the button feature.
- Add the display for showing some information.

### Fixed

- The device would not return the main loop when it arose "Global Ranging Timeout".
- Change the leds status when ranging finished.
- Fix a bug the ranging's accuracy was terrible while using BW125 or BW250.
- Fix a bug in the function "get_ranging_hopping_channels()". It is possible to overflow.

### Changed
- Allowing the negative range values can be displayed, and are not clipped to 0m.


## [v1.0.0] - 2024-07-29

### Added

- Initial version
