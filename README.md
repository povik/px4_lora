# Out-of-tree PX4 modules for broadcast of position over LoRaWAN

This repository holds out-of-tree modules for the PX4 autopilot firmware. One module, `lora`, listens for uORB messages with packets to transmit over LoRaWAN, and transmits those packets. A second module, `gps_lora_beacon`, prepares such packets out of GPS location information. The code is rushed in some places, but it works.

For implementation of LoRaWAN, the code relies on [LacunaSpace/basicmac](https://github.com/LacunaSpace/basicmac) library, which is derived from the LMiC library originally released by IBM Zurich.
