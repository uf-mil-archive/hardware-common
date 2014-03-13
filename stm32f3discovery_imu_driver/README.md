This is an example of an ARM firmware project that uses the bootloader.


To use it, plug both USB ports on your STM32F3DISCOVERY board into your computer and run:

    rosmake
    st-flash write ~/catkin_ws/build/uf-mil/hardware-common/stm32f3discovery_imu_driver/bootloader_out/bootloader.bin 0x8000000
    rosrun arm_bootloader get_device_id # write down the device id output
    rosrun stm32f3discovery_imu_driver stm32f3discovery_imu_driver _port:=/dev/serial/by-id/usb-uf-mil_subbus_d687e66c-if00 _dest:=0xd687e66c # replace device_id (2x)
