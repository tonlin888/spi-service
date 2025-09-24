Build:
    make package/spi-service/clean
    make package/spi-service/{clean,prepare} V=s
    make package/index
    make package/spi-service/compile V=s
    要修改 feeds 的 Makefile

Install:
    adb root
    mount -o rw,remount /
    adb push spi-service_1.0-1_arm_cortex-a7_neon-vfpv4.ipk /tmp
    opkg remove spi-service
    opkg install /tmp/spi-service_1.0-1_arm_cortex-a7_neon-vfpv4.ipk

procd:
    /etc/init.d/spi-service disable
    /etc/init.d/spi-service stop
    logread | grep spi-service