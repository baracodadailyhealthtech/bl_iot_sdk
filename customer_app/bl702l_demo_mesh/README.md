The "bl702l_demo_mesh" project has the following main features:

If the macro definition `CONFIG_MESH_APP` is set to 1, it will execute the demonstration features of the Mesh App. If `CONFIG_MESH_APP` is set to 0, it will execute the testing commands of the Mesh Components.

CONFIG_MESH_APP =1:

Upon powering up the device:

In the unprovisioned state: When the device is in an unprovisioned state, it broadcasts its unprovisioned status and waits for a mobile application to discover and provision the device. This signifies that the device is searching for a network to join and sends signals to notify the mobile application.

In the provisioned state: Once the device has successfully joined the network, it activates the mesh functionality and broadcasts its availability for connection. At this point, it awaits connections from the application for control and listens for messages from the air via scanning. Provisioned devices can be controlled through a mobile application.

Included models:

The "bl702l_demo_mesh" project includes the following types of models:

Generic On/Off Server: A server model for controlling the on/off state of devices.

Generic On/Off Client: A client model for controlling the on/off state of other devices.

Generic Level Server: A server model for controlling device brightness levels.

Light Lightness Server: A server model for controlling the brightness of lighting.

Light Control Server: A server model for controlling the state of lighting.

Time Server: A server model for synchronizing device time information.

Scene Server: A server model for managing different scenes.

Scheduler Server: A server model for scheduling and executing operations.

Support for quick model configuration: This project inherently supports quick model configuration. Specifically, when the Generic On/Off model is bound to an app key, the rest of the models will also be bound to the same app key. This means that once the Generic On/Off model is successfully bound to an app key, other models will share the same key for communication and control within the network.

In summary, the "bl702l_demo_mesh" project is designed to support device provisioning, configuration, and control. It encompasses various models to meet diverse application requirements and offers a convenient model configuration method, simplifying and enhancing communication between devices.

CONFIG_MESH_APP =0:

Please refer to the "Docs » Example » BLE » Mesh" section for more information.

