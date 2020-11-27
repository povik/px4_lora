#include <stdio.h>
#include <px4_platform_common/getopt.h>
#include <px4_platform_common/i2c_spi_buses.h>
#include <px4_platform_common/tasks.h>
#include <px4_platform_common/posix.h>

extern "C"
int
lsbus_main(int argc, char *argv[])
{
	BusCLIArguments cli{false, true};
	cli.default_spi_frequency = 10*1000*1000;

	const char *verb = cli.parseDefaultArguments(argc, argv);

	if (!verb) {
		return -1;
	}

	BusInstanceIterator iterator("lsbus", cli, 0);

	while (iterator.next()) {
		printf("bus=0x%x devid=0x%x\n", iterator.bus(), iterator.devid());
	}

	return 0;
}
