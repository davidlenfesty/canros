# canros
UAVCAN to ROS interface.

canros only supports Python 2 at this stage.

## Installation
Clone into the `src` folder in your catkin workspace and run `catkin_make install`.

    cd ~/catkin_ws/src
    git clone https://github.com/MonashUAS/canros.git
    cd ..
    catkin_make install


## Running the canros server
Run the canros server though ROS.

    rosrun canros server.py <can_interface> <uavcan_id>
    
    uavcan_id:      UAVCAN node id for canros. Must be between 1 and 127 inclusive.
    can_interface:  Address of CAN interface.


## API
The easiest way to use canros is to use the Python API which can be imported using `import canros` from within the catkin workspace.
For details on the API see the inline documentation in [__init__.py](src/canros/__init__.py).


### Examples
Send a message into the UAVCAN network once every seccond.

	#!/usr/bin/python

	from __future__ import print_function
	import canros
	import rospy

	rospy.init_node("canros_message_test")

	# Retrieve canros object for the message 'uavcan.protocol.enumeration.Indication'
	indication = canros.Message("uavcan.protocol.enumeration.Indication")

	# Create a ROS Publisher using the canros object
	pub = indication.Publisher(queue_size=10)

	# Use canros object to obtain the ROS type
	msg = indication.Type()

	# Obtain the ROS type for the message 'uavcan.protocol.param.NumericValue'
	# which is part of 'uavcan.protocol.enumeration.Indication'
	msg.value = canros.Message("uavcan.protocol.param.NumericValue").Type()

	# 'uavcan.protocol.param.NumericValue' is a union type so the 'canros_union_tag' field needs to be set
	msg.value.canros_union_tag = msg.value.CANROS_UNION_TAG_REAL_VALUE
	msg.value.real_value = 98.7654321

	msg.parameter_name = canros.to_uint8("canros test")

	# Spin while publishing the message every second.
	while True:
		rospy.sleep(1)
		if rospy.is_shutdown():
			raise Exception("ROS shutdown")
		pub.publish(msg)

Provide a service for consumption by the UAVCAN network.

	#!/usr/bin/python

	from __future__ import print_function
	import canros
	import rospy

	rospy.init_node("canros_service_test")

	# Retrieve canros object for the service 'uavcan.protocol.param.GetSet'
	get_set = canros.Service("uavcan.protocol.param.GetSet")

	def handler(req):
		print(req)

		# Use canros object to obtain the ROS response type
		resp = get_set.Response_Type()

		# The 'max_value' and 'min_value' feilds are union types so the 'canros_union_tag'
		# feild needs to be set
		resp.max_value.canros_union_tag = resp.max_value.CANROS_UNION_TAG_EMPTY
		resp.min_value.canros_union_tag = resp.min_value.CANROS_UNION_TAG_REAL_VALUE

		resp.min_value.real_value = 1.23456789
			resp.value = req.value
		resp.default_value = req.value
		resp.name = canros.to_uint8("service test: " + req.name)

		return resp

	# Use canros object to create a new ROS service
	get_set.Service(handler)

	# Spin
	while True:
		rospy.sleep(1)
		if rospy.is_shutdown():
			raise Exception("ROS shutdown")

## Conversion between ROS and UAVCAN
Though the specifications for ROS and UAVCAN types are similar, there are a few differences.

### Bit lengths
UAVCAN allows for an arbitrary bit length for signed and unsigned integers up to 64 bits, as well as a 16 bit float.
Such types are unavailable in ROS therefore, when creating the ROS message definitions, canros rounds up to the next bit length avialable.

### Union types
UAVCAN allows a message to be declared a union type which is unavailable in ROS.
For compatability, canros introduces a new `canros_union_tag` feild into ROS definitions.
The `canros_union_tag` feild is an unsigned integer which should be set to the index of the feild to be considered.
Constants named `CANROS_UNION_<field>` (where `<field>` is the name of the field in uppercase) are also added to the ROS definition.
This does not save bandwidth in the ROS network as all fields are still sent.

### Type names
UAVCAN types are placed in a folder hierarchy and then addressed relativly or by using the full name which is period (`.`) delimited (e.g. `uavcan.protocol.param.NumericValue`).
Type hierarchy nor perilod delimiting in type names is possible in ROS therefore canros names all ROS type definitions as the UAVCAN full name with double underscores (`__`) replacing the periods (e.g. `uavcan__protocol__param__NumericValue`).

### Topics
All communications with canros are done under the `/canros` topic.
Messages are sent and received under the `/msg` sub topic.
Service requests that go into the UAVCAN network are sent through the `/srv/req` sub topic.
Service responses that go into the UAVCAN network are sent through the `/srv/resp` sub topic.
Finally the UAVCAN name is used for heirachy.

For example the message `uavcan.protocol.enumeration.Indication` is sent and received on the topc `/canros/msg/uavcan/protocol/enumeration/Indication`.

### UAVCAN node IDs
ROS does not provide a method for attaching meta data such as source and destination addresses to a message. Therefore, for messages that require the source or destination UAVCAN node id, the `canros_uavcan_id` feild is added to the message definition. 
The `canros_uavcan_id` is added to all request definitions in services as well as a hard coded selection of messages.
Currently `uavcan.protocol.NodeStatus` is the only such hard coded message.

If a message is going from the UAVCAN network to the ROS network, then the `canros_uavcan_id` field will be populated with the UAVCAN node id of the node that sent the message.
If a message is going from the ROS network to the UAVCAN network, then the `canros_uavcan_id` field should be set to the UAVCAN node that should receive the message.