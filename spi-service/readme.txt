Build:
    make package/spi-service/clean; make package/spi-service/compile V=s
    make package/spi-service/{clean,prepare} V=s
    make package/index
    ./scripts/feeds update -a
    ./scripts/feeds update packages

Install:
    adb root; adb shell "mount -o rw,remount /"
    adb push spi-service_1.0-1_arm_cortex-a7_neon-vfpv4.ipk /tmp
    adb shell "opkg remove spi-service"; adb shell "opkg install /tmp/spi-service_1.0-1_arm_cortex-a7_neon-vfpv4.ipk"
    adb shell "opkg remove cmd-test"; adb shell "opkg install /tmp/cmd-test_1.0-1_arm_cortex-a7_neon-vfpv4.ipk"

procd:
    adb shell "/etc/init.d/spi-service disable"; adb shell "/etc/init.d/spi-service stop"; adb shell 'pid=$(ps | grep spi-service | grep -v grep | cut -c1-5); kill -9 $pid'
    logread | grep spi-service

log:
    spi-service > /tmp/spi-service.log 2>&1
    cmd-test 2>&1 | tee /tmp/cmd-test.log