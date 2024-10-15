# LR11xx SDK

This LR11xx SDK just contains the ranging example for LR11x0 transceivers.

## Requirements

### Supported boards

This SDK is developed on the ST Microeletronic [NUCLEO-L476RG development board](https://www.st.com/en/evaluation-tools/nucleo-l476rg.html).

### Supported shields

The list of compatible Semtech LR1110 shields is:

| Shield       | PCB      | Frequency matching | Characteristics                        |
| ------------ | -------- | ------------------ | -------------------------------------- |
| LR1110MB1DIS | E516V02B | 868/915 MHz        | GNSS with LNA for Passive GNSS Antenna |
| LR1110MB1DJS | E592V01B | 868/915 MHz        | GNSS without LNA                       |
| LR1110MB1GIS | E516V02B | 490 MHz            | GNSS with LNA for Passive GNSS Antenna |
| LR1110MB1GJS | E592V01B | 490 MHz            | GNSS without LNA                       |

The list of compatible Semtech LR1120 shields is:

| Shield       | PCB      | Frequency matching | Characteristics                        |
| ------------ | -------- | ------------------ | -------------------------------------- |
| LR1120MB1DIS | E655V01A | 868/915 MHz        | GNSS with LNA for Passive GNSS Antenna |
| LR1120MB1DJS | E656V01A | 868/915 MHz        | GNSS without LNA                       |
| LR1120MB1GIS | E655V01A | 490 MHz            | GNSS with LNA for Passive GNSS Antenna |
| LR1120MB1GJS | E656V01A | 490 MHz            | GNSS without LNA                       |

### Firmware

This SDK requires the transceiver to run the following version:

* LR1110: firmware version ([0x0401](https://github.com/Lora-net/radio_firmware_images/tree/master/lr1110/transceiver)).
* LR1120: firmware version ([0x0201](https://github.com/Lora-net/radio_firmware_images/tree/master/lr1120/transceiver)).

To update the transceiver with the desired firmware version, please use [the updater tool application](https://github.com/Lora-net/SWTL001).

### Workaround

#### High ACP (Adjacent Channel Power)

This issue has been fixed in firmware LR1110 0x0308 and LR1120 0x0102 - it was not present in LR1121 0x0101. The associated workaround is disabled in this project starting from v2.1.0.

If one wants to re-enable the workaround, this [line](common/apps_common.mk#L32) has to be commented.
