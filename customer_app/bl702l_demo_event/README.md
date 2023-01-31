# Build Script Description
```html
genblem1s1:        1 BLE connection is supported, BL702L can be master or slave.
genblem1s1p:       Based on genblem1s1, support BLE PDS(power down sleep) feature.
genblem8s1:        8 BLE connections are supported, BL702L can be master or slave.
genromap:          Used to generate sdk.
genzrstartup：     build Zigbee Router image; after boot, device will scan and join a network automatically if haven't join a network, otherwise, restore network and resume operation.
genromapzsedstartupwithpds31：build Zigbee Sleepy End Device image; after boot, device will scan and join a network automatically if haven't join a network, otherwise, restore network and resume operation. Device enters into pds31 when it can sleep.
```
