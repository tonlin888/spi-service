Build:
    make package/spi-service/clean; make package/spi-service/compile V=s
    make package/spi-service/{clean,prepare} V=s
    make package/index

Install:
    adb root; adb shell "mount -o rw,remount /"
    adb push spi-service_1.0-1_arm_cortex-a7_neon-vfpv4.ipk /tmp
    adb shell "opkg remove spi-service"; adb shell "opkg install /tmp/spi-service_1.0-1_arm_cortex-a7_neon-vfpv4.ipk"

procd:
    /etc/init.d/spi-service disable
    /etc/init.d/spi-service stop
    logread | grep spi-service