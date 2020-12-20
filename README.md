# Out-of-tree PX4 modules for broadcast of position over LoRaWAN

This repository holds out-of-tree modules for the PX4 autopilot firmware. One module, `lora`, listens for uORB messages with packets to transmit over LoRaWAN, and transmits those packets. A second module, `gps_lora_beacon`, prepares such packets out of GPS location information. The code is rushed in some places, but it works.

For implementation of LoRaWAN, the code relies on [LacunaSpace/basicmac](https://github.com/LacunaSpace/basicmac) library, which is derived from the LMiC library originally released by IBM Zurich.

The code is developed and tested with [this PX4 tree](https://github.com/ThunderFly-aerospace/PX4Firmware/tree/povik/fik-6-lora).

## Bugs & Known Limitations

 * Currently there is blanket wait after each command, as the BUSY signal is not connected to the autopilot processor. This is most likely what makes the LoRaWAN stack miss its RX windows, and prevents it from catching downlink packets.

