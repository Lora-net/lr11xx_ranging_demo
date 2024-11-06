# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).


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
