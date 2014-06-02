This repository contains ROS packages that can be used for
interfacing with custom hardware, including a bootloader
for ARM devices (arm_bootloader), a custom packetization
protocol (uf_subbus_protocol), and an example of using both
(stm32f3discovery_imu_driver).

This repository depends on the GNU ARM Embedded Toolchain,
installable with:

    sudo rm -f /etc/apt/sources.list.d/terry_guo-gcc-arm-embedded-*.list && sudo add-apt-repository -y ppa:terry.guo/gcc-arm-embedded && sudo sed -i -e 's/quantal/precise/g' /etc/apt/sources.list.d/terry_guo-gcc-arm-embedded-*.list && sudo apt-get update && sudo apt-get install gcc-arm-none-eabi

This package also depends on autoconf, automake, and
libtool:

    sudo apt-get install autoconf automake libtool
