# Imports
import pydbus

# Global definitions
bluez_service = 'org.bluez'

# BLE code
bus = pydbus.SystemBus()
mngr = bus.get(bluez_service, '/')

# For accessing the host bluetooth adapter, it must be requested using its system name:
# Global definitions 
adapter_path = '/org/bluez/hci0' # ANPASSEN!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

# BLE code
adapter = bus.get(bluez_service, adapter_path)

#  The following code snipped connects to a BLE-device with a given MAC-address.

# Global definitions
dev_id = '<MAC-address here>' # ANPASSEN!!!!!!!!! (vorher mit scan adresse suchen)

# BLE code
device_path = f"{adapter_path}/dev_{dev_id.replace(':','_')}"
device = bus.get(bluez_service, device_path)
device.Connect()


# The following code example enumerates the path to
# a characteristic with a given uuid and prints its value tostandard out

# Global definitions
uuid = '<UUID here>' # ANPASSEN!!!!!!!!!!!!!!!!!!!!!!!!!!

# BLE code
def get_characteristic_path(dev_path, uuid):
    mng_objs = mngr.GetManagedObjects()
    for path in mng_objs:
        chr_uuid = mng_objs[path].get('org.bluez.GattCharacteristic1', {}).get('UUID')
        if path.startswith(dev_path) and chr_uuid == uuid:
            return path

char_path = get_characteristic_path(device._path, uuid)
value = bus.get(bluez_service, char_path)
print(value.ReadValue({}))

# To get informed when a selected characterisic changes, we have to
# register a callback method to the characteristic and enable the
# notifications for the given characteristic

# BLE code
def value_change_handler(iface, prop_changed, prop_removed):
    if 'Value' in prop_changed:
        # prints the value to the standard output and is
        # registered to the value characteristic from above
        print(f"Value: {prop_changed['Value']}")

# Afterwards the notification mechanism for the value characteristic
# is enabled by calling StartNotify
value.onPropertiesChanged = value_change_handler
value.StartNotify()

# The program would terminate after starting the notification
# subsystem, as our python program ends at this point.
# To enter the event processing loop (MainLoop) and wait
# for notifications, we have to create and start the event processing.

# Imports
from gi.repository import GLib

# Mainloop code
mainloop = GLib.MainLoop()
try:
    mainloop.run()
except KeyboardInterrupt:
    # in case of an keyboard interrupt (Ctrl-C), the mainloop
    # is stopped, the notification system for the value characteristic disabled
    # and we disconnect from our BLE-device.
    mainloop.quit()
    value.StopNotify()
    device.Disconnect()

    