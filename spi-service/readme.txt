Build:
    make package/spi-service/clean
    make package/index
    make package/spi-service/compile V=s

Install:
    adb root
    mount -o rw,remount / 
    adb push spi-service_1.0-1_arm_cortex-a7_neon-vfpv4.ipk /tmp
    opkg remove spi-service
    opkg install /tmp/spi-service_1.0-1_arm_cortex-a7_neon-vfpv4.ipk