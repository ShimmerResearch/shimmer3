# S3_Sleep
This is a general purpose firmware which puts the Shimmer3 device in its lowest power consuming state ("sleep"). The firmware configures the pins on the Shimmer device depending on what unit (SR number) the device is.

This firmware provides as a useful starting point for someone wishing to create their own firmware where the initial states of all the pins have been set accordingly through the ```Board_init()``` function in ```main()```.