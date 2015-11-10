import bluetooth
devices = bluetooth.discover_devices()
for dev in devices:
	print '%s: %s' % (dev, bluetooth.lookup_name(dev))
	services = bluetooth.find_service(address=dev)
	for i in services:
		print i
	print
