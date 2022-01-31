#!/usr/bin/python

from queue import Queue
import logging
import re

import canros
import pyuavcan_v0 as uavcan

import rclpy
from rclpy.node import Node
from rclpy.exceptions import ParameterNotDeclaredException
import rcl_interfaces
from rcl_interfaces.msg import ParameterValue, ParameterType

try:
	import constants
	import uavcan_msgs
except ModuleNotFoundError:
	from canros import constants, uavcan_msgs

class Base(object):
	def __init__(self, uavcan_type):
		self.__uavcan_type = uavcan_type
		super(Base, self).__init__(self.UAVCAN_Type.full_name)

	@property
	def UAVCAN_Type(self):
		return self.__uavcan_type

	# Should be overriden
	def ROS_Subscribe(self, uavcan_node: uavcan.node.Node, ros_node: Node):
		raise NotImplementedError()
	def UAVCAN_Subscribe(self, uavcan_node: uavcan.node.Node, ros_node: Node):
		raise NotImplementedError()

class Message(Base, uavcan_msgs.Message):
	def __init__(self, uavcan_type):
		super(Message, self).__init__(uavcan_type)
		self.__ros_publisher = None

	@property
	def ROS_Publisher(self):
		if self.__ros_publisher is None:
			# TODO should make this impossible to reach for any consumer.
			# Not sure if this *should* get exported outside of the server, idk if anyone
			# is using canros as a library
			raise Exception("Publisher not created! This is a library issue, please report bug.")
		return self.__ros_publisher

	def ROS_Subscribe(self, uavcan_node: uavcan.node.Node, ros_node: Node):
		def handler(event):
			if event._connection_header["callerid"] == ros_node.get_name():
				# The message came from canros so ignore
				return
			uavcan_msg = uavcan_msgs.copy_ros_uavcan(self.UAVCAN_Type(), event)
			uavcan_node.broadcast(uavcan_msg, priority=uavcan.TRANSFER_PRIORITY_LOWEST)
		self.Subscriber(ros_node, handler, 10)

	def UAVCAN_Subscribe(self, uavcan_node: uavcan.node.Node, ros_node: Node):
		self.__ros_publisher = self.Publisher(ros_node, 10)
		def handler(event):
			ros_msg = uavcan_msgs.copy_uavcan_ros(self.Type(), event.message)
			if self.HasIdFeild:
				setattr(ros_msg, constants.uavcan_id_field_name, event.transfer.source_node_id)
			self.ROS_Publisher.publish(ros_msg)
		uavcan_node.add_handler(self.UAVCAN_Type, handler)

class Service(Base, uavcan_msgs.Service):
	def __init__(self, uavcan_type):
		super(Service, self).__init__(uavcan_type)
		self.__ros_service_proxy = None

	# This is the canros server so the service naming scheme is the reverse of the canros API.
	@property
	def Request_Name(self):
		return super(Service, self).Response_Name
	@property
	def Response_Name(self):
		return super(Service, self).Request_Name
	@property
	def Request_Type(self):
		return super(Service, self).Response_Type
	@property
	def Response_Type(self):
		return super(Service, self).Request_Type
	@property
	def Request_Topic(self):
		return super(Service, self).Response_Topic
	@property
	def Response_Topic(self):
		return super(Service, self).Request_Topic

	@property
	def ROS_Client(self):
		if self.__ros_client is None:
			# TODO this should also be made impossible
			raise Exception("No client created! Library error please file bug.")
		return self.__ros_client

	def ROS_Subscribe(self, uavcan_node: uavcan.node.Node, ros_node: Node):
		def handler(event):
			uavcan_req = uavcan_msgs.copy_ros_uavcan(self.UAVCAN_Type.Request(), event, request=True)

			q = Queue(maxsize=1)
			def callback(event):
				q.put(event.response if event else None)

			uavcan_id = getattr(event, constants.uavcan_id_field_name)
			if uavcan_id == 0:
				return
			uavcan_node.request(uavcan_req, uavcan_id, callback, timeout=1)   # Default UAVCAN service timeout is 1 second

			uavcan_resp = q.get()
			if uavcan_resp is None:
				return
			return uavcan_msgs.copy_uavcan_ros(self.Response_Type.call(), uavcan_resp, request=False)
		self.Client(ros_node)

	def UAVCAN_Subscribe(self, uavcan_node: uavcan.node.Node, ros_node: None):
		self.__ros_client = ros_node.create_client(self.Type, self.Request_Topic)
		def handler(event):
			ros_req = uavcan_msgs.copy_uavcan_ros(self.Request_Type(), event.request, request=True)
			setattr(ros_req, constants.uavcan_id_field_name, event.transfer.source_node_id)
			try:
				ros_resp = self.ROS_Client.call(ros_req)
			except rospy.ServiceException:
				return
			return uavcan_msgs.copy_ros_uavcan(self.UAVCAN_Type.Response(), ros_resp, request=False)
		uavcan_node.add_handler(self.UAVCAN_Type, handler)

'''
Returns the hardware id for the system.
Roughly follows MachineIDReader from:
https://github.com/UAVCAN/libuavcan/blob/master/libuavcan_drivers/linux/include/uavcan_linux/system_utils.hpp.
'''
def hardware_id():
	search_locations = [
		"/etc/machine-id",
		"/var/lib/dbus/machine-id",
		"/sys/class/dmi/id/product_uuid"
	]

	for loc in search_locations:
		try:
			with open(loc, 'r') as f:
				first_line = f.readline().lower()
				hex_string = re.sub(r'[^0-9a-f]', '', first_line)
				byte_array = [int(hex, 16) for hex in hex_string]
				if len(byte_array) >= 16:
					return byte_array[0:16]
		except IOError:
			pass

	raise Exception("Unable to obtain a hardware ID for this system")


# XXX ideally more of the application data would fit in here.
# Semantically this *should* handle pulling ROS params and starting UAVCAN
# node, but not really worth the effort IMO.
class UavcanNode(Node):
	"""ROS UAVCAN Handler node"""

	def __init__(self):
		super().__init__('canros_server')
		# TODO sane defaults or no defaults or whatever
		self.declare_parameter('can_interface', "vcan0")
		self.declare_parameter('uavcan_id', 127)
		self.declare_parameter('blacklist', [])

def main(args=None):
	# Init ROS node
	rclpy.init(args=args)
	ros_node = UavcanNode()

	# Get can_interface parameter
	try:
		can_interface = ros_node.get_parameter('can_interface').get_parameter_value()
		if can_interface.type != ParameterType.PARAMETER_STRING:
			print("'can_interface' must be a string")
			return
		can_interface = can_interface.string_value
	except ParameterNotDeclaredException:
		print("'can_interface' ROS parameter must be set")
		return

	# Get uavcan_node_id parameter
	try:
		uavcan_node_id = ros_node.get_parameter('uavcan_id').get_parameter_value()
		if uavcan_node_id.type != ParameterType.PARAMETER_INTEGER:
			raise ValueError
		uavcan_node_id = uavcan_node_id.integer_value
		if uavcan_node_id < 0 or uavcan_node_id > 127:
			raise ValueError()
	except ParameterNotDeclaredException:
		print("'uavcan_id' ROS parameter must be set")
		return
	except ValueError:
		print("'uavcan_id' must be an integer from 0-127")
		return

	try:
		blacklist = ros_node.get_parameter('blacklist').get_parameter_value()
		print(blacklist)
		if blacklist.type == ParameterType.PARAMETER_STRING:
			blacklist = list(blacklist.string_value)
		elif blacklist.type == ParameterType.PARAMETER_STRING_ARRAY:
			blacklist = list(blacklist.string_array_value)
		else:
			# TODO proper error handling here, I'm misunderstanding something about how
			# parameters are being passed. This logic is probably fine, I'm just having
			# issues using it
			# raise ValueError
			blacklist = []
	except ParameterNotDeclaredException:
		blacklist = []
	except ValueError:
		print("'blacklist' must be a list or strings")
		return

	# Init UAVCAN logging
	uavcan.driver.slcan.logger.addHandler(logging.StreamHandler())
	uavcan.driver.slcan.logger.setLevel('DEBUG')

	# Set UAVCAN node information
	uavcan_node_info = uavcan.protocol.GetNodeInfo.Response()
	uavcan_node_info.name = constants.uavcan_name
	uavcan_node_info.software_version.major = 0
	uavcan_node_info.software_version.minor = 1
	uavcan_node_info.hardware_version.unique_id = hardware_id()

	# Start UAVCAN node
	uavcan_node = uavcan.make_node(can_interface, node_id=uavcan_node_id, node_info=uavcan_node_info)

	# Load types
	for uavcan_name, typ in uavcan.TYPENAMES.items():
		if typ.default_dtid is None:
			continue
		if uavcan_name in blacklist:
			continue

		# Construct message type
		if typ.kind == typ.KIND_MESSAGE:
			msg_type = Message(typ)
		else:
			msg_type = Service(typ)

		# Subscribe to/from both nodes
		msg_type.UAVCAN_Subscribe(uavcan_node, ros_node)
		msg_type.ROS_Subscribe(uavcan_node, ros_node)

	# GetInfo
	def GetInfoHandler(_):
		rosmsg = canros.srv.GetNodeInfoResponse()
		rosmsg.node_info = uavcan_msgs.copy_uavcan_ros(rosmsg.node_info, uavcan_node.node_info, request=False)
		setattr(rosmsg.node_info.status, constants.uavcan_id_field_name, uavcan_node.node_id)
		return rosmsg
	ros_node.create_service(canros.srv.GetNodeInfo, constants.get_info_topic, GetInfoHandler)

	# Spin
	uavcan_errors = 0
	#while not ros_node.is_shutdown():
	while True:
		try:
			uavcan_node.spin(0)
			# TODO come up with better concurrency/execution model
			rclpy.spin_once(ros_node)
			if uavcan_errors > 0:
				uavcan_errors = 0
		except uavcan.transport.TransferError:
			uavcan_errors += 1
			if uavcan_errors >= 1000:
				print("Too many UAVCAN transport errors")
				break

	uavcan_node.close()
	print("canros server exited successfully")

if __name__ == "__main__":
	main()
