cmd_drivers/spi/built-in.o :=  /home/yyoung.kim/Toolchain/prebuilt/linux-x86/toolchain/arm-eabi-4.4.3/bin/arm-eabi-ld -EL    -r -o drivers/spi/built-in.o drivers/spi/spi.o drivers/spi/spi_bitbang.o drivers/spi/spi_gpio.o drivers/spi/spi_s3c64xx.o drivers/spi/spidev.o 