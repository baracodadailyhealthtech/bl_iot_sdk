# Openthread examples

## Command line example

Command line example is for openthread official examples: `ot-cli-ftd` and `ot-cli-mtd`, which doesn't have extra configurations during startup. 

### Build

Type following command to build:

```shell
./genotcli
```

`genotcli` script has follwoing build options to build FTD/MTD command line device.

| option        | comments                                                     |
| ------------- | ------------------------------------------------------------ |
| CONFIG_FTD    | `0`, build MTD image, which enables joiner<br />`1`, build FTD image, which enables both of commissioner and joiner. |
| CONFIG_PREFIX | set prefix for command. |

### Execution - 1

1. Build a FTD image and a MTD image,  (Two FTD images are also OK) and download to the eval boards.

2. Start a network, types command as following:

   ```shell
   > dataset init new
   Done
   > dataset commit active
   Done
   > ifconfig up
   Done
   > thread start
   Done
   ```

   After a while , type `state` to check the device whether to be leader.

3. Start the commission role, types command as following: 

   ```shell
   > commissioner start
   Done
   ```

   And then, types command as following to let other devices to attach:

   ```shell
   > commissioner joiner add * J01NME
   ```

4. Start another device and attach to the network as a joiner, type commands as following:

   ```shell
   > ifconfig up
   Done
   joiner start J01NME
   > Done
   ```

   After get `Join success!` confirmation message, and type following command to start the device.

   ```shell
   > thread start
   Done
   ```

5.  Check the status

   After a while, type `state` command to get device status.

6. Ping each other

   Type `ipaddr`  to get IP address, and then `ping <ip address>` in command line of another device. And then sniffer tool should captured ICMPv6 packets.

Please refer the [official page](https://openthread.google.cn/guides/build/commissioning) for more detail information.

### Execution - 2

While，directly configuring dataset to attach the device to the network also works.

1. Build a FTD image and a MTD image,  (Two FTD images are also OK) and download to the eval boards.

2. Start a network, types command as following:

   ```shell
   > dataset init new
   Done
   > dataset commit active
   Done
   > ifconfig up
   Done
   > thread start
   Done
   ```

   After a while , type `state` to check the device whether to be leader.

3. Get current active dataset of the network, and type command as folllowing:
   ```shell
   > dataset active -x
   0e080000000000010000000300000b35060004001fffe00208ee2c03898c34f15c0708fd38d25691e24621051015f62338e7c77a9b19dad9f3165f9405030f4f70656e5468726561642d633763390102c7c90410a304ef9fae8a047130e4461361f9489c0c0402a0fff8
   Done
   ```
4. Start another device and attach to the network with dataset, type commands as following:

   ```shell
   > dataset set active 0e080000000000010000000300000b35060004001fffe00208ee2c03898c34f15c0708fd38d25691e24621051015f62338e7c77a9b19dad9f3165f9405030f4f70656e5468726561642d633763390102c7c90410a304ef9fae8a047130e4461361f9489c0c0402a0fff8
   dataset set active 0e080000000000010000000300000b35060004001fffe00208ee2c03898c34f15c0708fd38d25691e24621051015f62338e7c77a9b19dad9f3165f9405030f4f70656e5468726561642d633763390102c7c90410a304ef9fae8a047130e4461361f9489c0c0402a0fff8
   Done
   > ifconfig up
   Done
   > thread start
   Done
   ```

5.  Check the status

   After a while, type `state` command to get device status.

6. Ping each other

   Type `ipaddr`  to get IP address, and then `ping <ip address>` in command line of another device. And then sniffer tool should captured ICMPv6 packets.

## Sleepy End Device example

Type following command to build:

```shell
./genotsed
```
`genotcli` script has follwoing build options to build FTD/MTD command line device.

| option        | comments                                                     |
| ------------- | ------------------------------------------------------------ |
| CONFIG_CSL_RX    | `0`, SED device, CSL receiver disabled<br />`1`, SSED device, CSL recevier enabled. |