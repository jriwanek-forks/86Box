//
// Created by root on 4/16/21.
//

#include "RetroWave_86Box.h"

RetroWaveContext retrowave_global_context;
int retrowave_global_context_inited = 0;

static void retrowave_iocb_empty(void *userp, uint32_t data_rate, const void *tx_buf, void *rx_buf, uint32_t len) {

}

void retrowave_init_86box(const char *path) {
	if (retrowave_global_context_inited)
		return;

	int rc = -1;

#if defined (__linux__) || defined (__unix__) || (defined (__APPLE__) && defined (__MACH__))
		char buf[128];
		snprintf(buf, sizeof(buf)-1, "/dev/%s", path);

		rc = retrowave_init_posix_serialport(&retrowave_global_context, buf);
#endif

#ifdef _WIN32
		rc = retrowave_init_win32_serialport(&retrowave_global_context, path);
#endif

	if (rc < 0) {
		printf("RetroWave: Failed to init board! Please change configuration!\n");
		retrowave_init(&retrowave_global_context);
		retrowave_global_context.callback_io = retrowave_iocb_empty;
	}

	printf("Opened tty: /dev/%s\n", path);

	retrowave_io_init(&retrowave_global_context);

	retrowave_global_context_inited = 1;
}
