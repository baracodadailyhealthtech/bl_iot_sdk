demo_mesh 工程主要功能：
	1、设备上电后
		在未入网时，进入未入网状态，发未入网广播，等待手机 app 发现并配置设备入网
		在已入网时，启动 mesh 功能，同时发送可连接广播，等待app连接控制，及通过scan监听空中的消息包。已配网的设备可以使用手机 app 进行控制。 
	
	2、demo_mesh中包含的model如下
		generic onoff server
		generic onoff client
		generic level server
		light lightness server
		light control server
		time server
		scene server
		scheduler server
		
    3、该工程默认支持model的快速配置，当generic onoff model被绑定appkey之后，其余的model都会被绑定相同的appkey