#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <stdbool.h>

#include <bluetooth/bluetooth.h>
#include <glib.h>

#include "src/plugin.h"
#include "src/adapter.h"
#include "src/device.h"
#include "src/log.h"

long int wmemu_pincb(struct btd_adapter *adapter, struct btd_device *device,
                     char *pinbuf, int *user_io_capability)
{
	//force Wii pin (bdaddr bytes backwards)
	//very basic, no filtering, does this for all devices
	memcpy(pinbuf, device, 6);
	return 6;
}

static int wmemu_probe(struct btd_adapter *adapter)
{
	btd_adapter_register_pin_cb(adapter, wmemu_pincb);

	return 0;
}

static void wmemu_remove(struct btd_adapter *adapter)
{
	btd_adapter_unregister_pin_cb(adapter, wmemu_pincb);
}

static struct btd_adapter_driver wmemu_driver = {
	.name	= "wmemu",
	.probe	= wmemu_probe,
	.remove	= wmemu_remove,
};

static int wmemu_init(void)
{
	return btd_register_adapter_driver(&wmemu_driver);
}

static void wmemu_exit(void)
{
	btd_unregister_adapter_driver(&wmemu_driver);
}

BLUETOOTH_PLUGIN_DEFINE(wmemu, VERSION,
		BLUETOOTH_PLUGIN_PRIORITY_LOW, wmemu_init, wmemu_exit)
