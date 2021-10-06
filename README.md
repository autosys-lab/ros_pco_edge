# ros_pco_edge

This package is a ros2 driver for PCO cameras using the PCO Camera Linux API taken from [here](https://www.pco.de/fileadmin/fileadmin/user_upload/pco-software/pco.linux_usb_pl_1_01_19.tar.gz)
The package from PCO contains two folders pco_common and pco_usb_pl, the pco_common has been taken almost as is and made
into a package with just changes to the include paths (reference the project name and folder).
pco_usb_pl contains a communication and grabber class and some test programs. The test programs have been mostly removed
but the classes remain and a driver using image_transport has been written over it.