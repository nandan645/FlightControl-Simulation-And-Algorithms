Observations :

	The SBC provided is a custom Jetson Nano board. So the default method for installing os on it wont work, it has 2 camera ports and a USB power port which is not in the officially manufactured boards.It also has an onboard memory installed (16GiB).

	We need the SDK Manager for installing os in it. 
	
Steps to install :

		1. Install Ubuntu 18.04 as it is recommended. https://releases.ubuntu.com/18.04/ 
		   There is some docker method also available, but I haven't tried it yet.
		2. Install SDK Manager in it. https://developer.nvidia.com/sdk-manager
			1. Now before installing we need some things : 
				1. Female to female jumper wire
				2. Two USB cables, one for data transfer and another to power up the device.
				3. Optionals : 
					1. Ethernet Cable or Wifi Adapter
					2. Monitor
					3. Keyboard & Mouse
			2. And External Storage if we want to expand storage since onboard one is just 16GiB.
		3. Now we need to set-up Jetson Nano Custom board for installation.
			1. Connect FC REC and GND with jumper wires.
			2. Connect the board with host device with Ubuntu 18.04 installed in it.
			3. Plug in the power cable to the board. 
		4. The setup is complete, now open up the SDK Manager on host device, it'll detect the board as Jetson Nano, leave it as default. 
		5. Select the latest options available, and click on next.
		6. For now don't select jetpack files (the extra ones), just install the main on. Ill attatch pictures later. 
		7. When installation is complete you can unplug it from the host.
		8. Remove the jumper wires before booting up the device.