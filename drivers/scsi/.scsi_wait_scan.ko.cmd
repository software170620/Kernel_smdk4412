cmd_drivers/scsi/scsi_wait_scan.ko := /home/yyoung.kim/Toolchain/prebuilt/linux-x86/toolchain/arm-eabi-4.4.3/bin/arm-eabi-ld -EL -r  -T /home/yyoung.kim/KOR_ANDR_OSS_OPEN_M0SKT-FINAL/Kernel/scripts/module-common.lds --build-id  -o drivers/scsi/scsi_wait_scan.ko drivers/scsi/scsi_wait_scan.o drivers/scsi/scsi_wait_scan.mod.o