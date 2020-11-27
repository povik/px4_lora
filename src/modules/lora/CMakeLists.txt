px4_add_module(
	MODULE modules__lora
	MAIN lora
	COMPILE_FLAGS
		-Wno-shadow -Wno-unused-variable -Wno-fatal-errors
		-Wno-error=strict-prototypes
	INCLUDES
		.

	SRCS
		lora.cpp
		hal.cpp
		basicmac/lmic/debug.c
		basicmac/lmic/lce.c
		basicmac/lmic/lmic.c
		basicmac/lmic/oslmic.c
		basicmac/lmic/radio-sx126x.c
		basicmac/lmic/radio-sx127x.c
		basicmac/lmic/radio.c
		basicmac/aes/aes-common.c
		basicmac/aes/aes-ideetron.c
		basicmac/aes/aes-original.c
	)
