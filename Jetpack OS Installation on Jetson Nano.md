Observations :

	The SBC provided is a custom Jetson Nano board. So the default method for installing os on it wont work, it has 2 camera ports and a USB power port which is not in the officially manufactured boards.It also has an onboard memory installed (16GiB).

	We need the SDK Manager for installing os in it. 
	
	Steps to install :
		1. Install Ubuntu 18.04 as it is recommended. https://releases.ubuntu.com/18.04/ 
		   There is some docker method also available, but I haven't tried it yet.
		2. Install SDK Manager in it. https://developer.nvidia.com/sdk-manager
		3. Now before installing we need some things : 
			1. Female to female jumper wire
			2. Two USB cables, one for data transfer and another to power up the device.
			3. Optionals : 
				1. Ethernet Cable or Wifi Adapter
				2. Monitor
				3. Keyboard & Mouse
		4. And External Storage if we want to expand storage since onboard one is just 16GiB.