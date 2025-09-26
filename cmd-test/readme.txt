Build:
    make package/cmd-test/clean; make package/cmd-test/compile V=s

Install:
    (./owrt/bin/packages/arm_cortex-a7_neon-vfpv4/cmd_test/cmd-test_1.0-1_arm_cortex-a7_neon-vfpv4.ipk)
    adb root
    adb shell "mount -o rw,remount /"
    adb push cmd-test_1.0-1_arm_cortex-a7_neon-vfpv4.ipk /tmp
    adb shell "opkg remove cmd-test"; adb shell "opkg install /tmp/cmd-test_1.0-1_arm_cortex-a7_neon-vfpv4.ipk"