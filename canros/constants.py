ros_node_name = "canros"
uavcan_name = "org.monashuas.canros"

msgs_with_id = ["uavcan.protocol.NodeStatus"]

uavcan_id_field_name = "canros_uavcan_id"
union_tag_field_name = "canros_union_tag"
union_const_prefix = "CANROS_UNION_TAG_"

ros_topic_prefix = "/canros"
get_info_topic = ros_topic_prefix + "/GetInfo"
