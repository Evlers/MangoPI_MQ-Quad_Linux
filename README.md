## MangoPI MQ-Quad board linux base on Allwinner H616
Linux-5.19.4
<br>

## Peripheral support
| **Peripheral** |               **Describe**               |
|----------------|------------------------------------------|
|   RTL8723DS    | WiFi/BT                                  |
|   USB Host     | 1 typec host with2 usb host in misc port |
|   USB OTG      | 1 typec otg                              |
|   ethernet     | ethernet for mac1 with ephy in misc port |
|     UART       | uart1 with uart2                         |
|  Mini HDMI     | hdmi with audio                          |
|      I2C       | i2c1 with i2c2                           |
|  Nor Flash     | spi0 nor flash                           |
|      SPI       | spi1dev drivers                          |
|      GPU       | gpu(panforst)                            |

## Build
```
# using mangopi_mq_quad_defconfig
make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- mangopi_mq_quad_defconfig

# build kernel image
make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- -jx Image
# output file: arch/arm64/boot/Image

# build device tree
make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- -jx dtbs
# output file: arch/arm64/boot/dts/allwinner/sun50i-h616-mangopi-mcore.dtb

# build modules and install modules
make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- -jx modules
make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- INSTALL_MOD_PATH=<any-path-you-like> modules modules_install
# After a successful build, you can find the module in the provided INSTALL_MOD_PATH directory

# install headers files
make ARCH=arm64 INSTALL_HDR_PATH=<any-path-you-like> headers_install
# Change INSTALL_HDR_PATH to the path you want to install to, such as /usr or <some_prefix>/usr to facilitate late cross-compilation
```

## Reference
https://github.com/open-cores/mangguo-h616-armbian
