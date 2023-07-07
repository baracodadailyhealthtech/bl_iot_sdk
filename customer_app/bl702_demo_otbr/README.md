
Bouffalo Lab Openthread Border Router runs all openthread stack and Border Router function on single chip - BL706, and uses Ethernet or Wi-Fi for backbone connection.

- Ethenet OTBR
- Wi-Fi OTBR, needs a BL602 module which runs a Wi-Fi transceiver. Please refer to `sdk_app_throughput` demo for more detail.

# Build

Type following command to build:

```shell
./genromap
```

- Build control on Ethernet OTBR or Wi-Fi OTBR
  - `CONFIG_USE_WIFI_BR=0` in `genromap` is for Ethernet OTBR build
  - `CONFIG_USE_WIFI_BR=1` in `genromap` is for Wi-Fi OTBR build

- Build to auto form a Thread Network.
  - `CONFIG_THREAD_AUTO_START=1` in `genromap` enables to form a Thread network after it is assigned IPv6 address. 
    The network information as below macro defined in `include/main.h`. 
    ```c
    #define THREAD_CHANNEL      11
    #define THREAD_PANID        0xB702
    #define THREAD_EXTPANID     {0x11, 0x11, 0x11, 0x11, 0x22, 0x22, 0x22, 0x22}
    #define THREAD_NETWORK_KEY  {0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xaa, 0xbb, 0xcc, 0xdd, 0xee, 0xff}
    ```
  - `CONFIG_THREAD_AUTO_START=0` in `genromap` doesn't form a Thread network. 
    Openthread command line is available on UART with `otc ` prefix to create and start a Thread network, such as `otc state`. 

# Ethernet OTBR

  - Connect OTBR to IPv6 border router, and power up OTBR.
  - After IPv6 address is assigned, Thread network will automatically start if `CONFIG_THREAD_AUTO_START=1`.

# Wi-Fi OTBR
  
  - Power up OTBR.
  - Open OTBR UART or USB CDC command line with serial port tool. And type following command to connect a AP.
    ```shell
    wifi_connect <wifi_ssid> <wifi_password>
    ```
  - After connected and IPv6 address assigned. The IP information will be printed out.
  - Then Thread network will automatically start if `CONFIG_THREAD_AUTO_START=1`. 
  - And Wi-Fi AP credention and Thread network information will saved in flash. Power cycle will resume connection to AP and Thread network.
