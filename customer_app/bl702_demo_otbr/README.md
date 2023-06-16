For Ethernet border router
1, Reference document.
    openthread[https://openthread.google.cn/platforms]
    Thread Border Router - Bidirectional IPv6 Connectivity and DNS-Based Service Discovery[https://openthread.google.cn/codelabs/openthread-border-router#0]
    Build a Thread network with nRF52840 boards and OpenThread[https://openthread.google.cn/codelabs/openthread-hardware#0]

2、Build border router and thead node.
    Build border router.
        Run ./genromap, and generate bin file in build_out/bl702_demo_otbr.bin

    Build thread node.
        Enter ../bl702_demo_thread, change the default Makefile as fallows, enable SRP:
            -OT_FLAGS += -DOPENTHREAD_CONFIG_TMF_NETDATA_SERVICE_ENABLE=0
            -OT_FLAGS += -DOPENTHREAD_CONFIG_SRP_SERVER_ENABLE=0
            +OT_FLAGS += -DOPENTHREAD_CONFIG_TMF_NETDATA_SERVICE_ENABLE=1
            +OT_FLAGS += -DOPENTHREAD_CONFIG_SRP_SERVER_ENABLE=1

        Run ./genotcli, and generate bin file in build_out/bl702_demo_event.bin

3, This border ruter demo is runing when powerup. When IPv6 of ehternet is got, the thread using the default parameter start network. and do service advertising_proxy.

    openthread default network parameter is set using command as fallows, this have done in code:    
        otc dataset set active 0e080000000000010000000300000b35060004001fffe002085b141bbd9a20f36e0708fdfcc861d016eeba0510575a1113893c98599ef90b7d8f72b352030f4f70656e5468726561642d313238350102128504106b859f819fbcafeb717370d802b3c8600c0402a0fff8

        otc ifconfig up

        otc thread start


    The thread node using the fallow command join border router. and create a service.
        Join the OTBR network
            Border router
                $ otc dataset active -x
                0e080000000000010000000300000b35060004001fffe002085b141bbd9a20f36e0708fdfcc861d016eeba0510575a1113893c98599ef90b7d8f72b352030f4f70656e5468726561642d313238350102128504106b859f819fbcafeb717370d802b3c8600c0402a0fff8
                Done
            Thread
                > otc dataset set active 0e080000000000010000000300000b35060004001fffe002085b141bbd9a20f36e0708fdfcc861d016eeba0510575a1113893c98599ef90b7d8f72b352030f4f70656e5468726561642d313238350102128504106b859f819fbcafeb717370d802b3c8600c0402a0fff8
                Done

                > otc ifconfig up
                Done
                > otc thread start
                Done

                > otc state
                child
                Done
                
                > otc netdata show
                Prefixes:
                fd76:a5d1:fcb0:1707::/64 paos med 4000
                Routes:
                fd49:7770:7fc5:0::/64 s med 4000
                Services:
                44970 5d c000 s 4000
                44970 01 9a04b000000e10 s 4000
                Done
                > ipaddr
                fd76:a5d1:fcb0:1707:d3dc:26d3:f70b:b927
                fda8:5ce9:df1e:6620:0:ff:fe00:4001
                fda8:5ce9:df1e:6620:ed74:123:cc5d:74ba
                fe80:0:0:0:d4a9:39a0:abce:b02e
                Done
            
            > otc ping fd76:a5d1:fcb0:1707:f3c7:d88c:efd1:24a9
                Done


        We can register a service with the srp client command.

        Go to the SRP client node screen session and auto-start the SRP client:


        > otc srp client autostart enable
        Done
        Set the hostname that will be advertised on the Wi-Fi/Ethernet link:


        > otc srp client host name ot-host
        Done
        For a device on the Wi-Fi/Ethernet link to reach a Thread end device, the OMR address of the end device needs to be advertised:


        > otc srp client host address fd76:a5d1:fcb0:1707:d3dc:26d3:f70b:b927
        Done
        At the end, register a fake _ipps._tcp service:


        > otc srp client service add ot-service _ipps._tcp 12345
        Done
        Wait a few seconds and we should be able to see the service registered:


        > otc srp client service
        instance:"ot-service", name:"_ipps._tcp", state:Registered, port:12345, priority:0, weight:0
        Done


    in windows powershell. you can use "ping -6 ot-host.local", to check the thread is connected.
    
For wifi Border router
ble 配网使用说明：
    在使用前需要在给602烧录 602/bl602_demo_sdiowifi.bin 的 spi wifi 固件。 702 烧录本工程代码编译出来的bin文件。

    1、702+602 模组，通过串口cli发送如果命令， 启动ble配网。
    
        a、hostnet_init\r\n 初始化wifi.

        b、blsync_ble_start\r\n 启动ble配网。

    2、使用bouffalolan_ble app 发现设备并添加设备入网。
       APP下载地址：https://dev.bouffalolab.com/media/upload/download/Bouffalo_BLE_v1.0_20211118.apk


702烧录相关文件：
    Factory Params: 702/bl_factory_params_IoTKitA_32M.dts
    Partition Table: 702/partition_cfg_2M.toml
    Firmware Bin: 

602烧录相关文件：
    Factory Params: 602/bl_factory_params_IoTKitA_32M.dts
    Partition Table: 602/partition_cfg_2M.toml
    Boot2 Bin: 602/boot2_iap_release.bin
    Firmware Bin:  602/bl602_demo_sdiowifi.bin


备注:
    后缀为3R_V2Hub的用于树实的板子。后缀为BL602-IoT-DVK-3S用于博流的板子

Border router 使用说明配置Matter 设备入网说明：
    thread 节点：
        清空配置参数：
            device factoryreset
            otc factoryreset

    
    python controller:
        启动 python controller
            source out/python_env/bin/activate
            sudo out/python_env/bin/chip-device-ctrl

        添加设备，目前版本 border router dataset 信息是固定的。
            ble-scan
            connect -ble 3872 20202021 1234
            zcl NetworkCommissioning AddThreadNetwork 1234 0 0 operationalDataset=hex:0e080000000000010000000300000b35060004001fffe002085b141bbd9a20f36e0708fdfcc861d016eeba0510575a1113893c98599ef90b7d8f72b352030f4f70656e5468726561642d313238350102128504106b859f819fbcafeb717370d802b3c8600c0402a0fff8 breadcrumb=0 timeoutMs=3000
            zcl NetworkCommissioning EnableNetwork 1234 0 0 networkID=hex:5b141bbd9a20f36e breadcrumb=0 timeoutMs=3000
            resolve 1234
            zcl OnOff Toggle 1234 1 0



