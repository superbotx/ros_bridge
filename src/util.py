import subprocess
import yaml
import json
import roslib.message
import rospy
import re
import base64
from pprint import pprint

def get_ros_nodes():
    ros_node_cmd_output = subprocess.check_output(['rosnode', 'list']).split('\n')
    ros_nodes = []
    ros_nodes_hash = []
    for ros_node_raw in ros_node_cmd_output:
        ros_node_hash = ros_node_raw.strip()[1:]
        ros_node = ros_node_hash.split('_')[0]
        if ros_node != '':
            ros_nodes.append(ros_node)
        if ros_node_hash != '':
            ros_nodes_hash.append(ros_node_hash)
    return ros_nodes, ros_nodes_hash

def get_ros_topics():
    ros_topic_cmd_output = subprocess.check_output(['rostopic', 'list']).split('\n')
    ros_topics = []
    for ros_topic_raw in ros_topic_cmd_output:
        ros_topic = ros_topic_raw.strip()[1:]
        if ros_topic != '':
            package_name, module_name = get_ros_topic_type(ros_topic)
            msg_package = __import__(package_name, globals(), locals(), [module_name], -1)
            msg_module = getattr(msg_package, module_name)
            ros_topic_item = {
                'name': ros_topic,
                'package_name': package_name,
                'module_name': module_name,
                'module': msg_module
            }
            ros_topics.append(ros_topic_item)
    return ros_topics

def get_ros_topic_type(topic_name):
    ros_topic_type_raw = subprocess.check_output(['rostopic', 'type', '/'+topic_name])
    fields = ros_topic_type_raw.strip().split('/')
    package_name = fields[0] + '.msg'
    module_name = fields[1]
    return package_name, module_name

def msg2json(msg):
   ''' Convert a ROS message to JSON format'''
   y = yaml.load(str(msg))
   return json.dumps(y,indent=4)

python_to_ros_type_map = {
    'bool'    : ['bool'],
    'int'     : ['int8', 'byte', 'uint8', 'char',
                 'int16', 'uint16', 'int32', 'uint32',
                 'int64', 'uint64', 'float32', 'float64'],
    'float'   : ['float32', 'float64'],
    'str'     : ['string'],
    'unicode' : ['string'],
    'long'    : ['uint64']
}

python_primitive_types = [bool, int, long, float]
python_string_types = [str, unicode]
python_list_types = [list, tuple]

ros_time_types = ['time', 'duration']
ros_primitive_types = ['bool', 'byte', 'char', 'int8', 'uint8', 'int16',
                       'uint16', 'int32', 'uint32', 'int64', 'uint64',
                       'float32', 'float64', 'string']
ros_header_types = ['Header', 'std_msgs/Header', 'roslib/Header']
ros_binary_types_regexp = re.compile(r'(uint8|char)\[[^\]]*\]')

list_brackets = re.compile(r'\[[^\]]*\]')

def convert_dictionary_to_ros_message(message_type, dictionary):
    """
    Takes in the message type and a Python dictionary and returns a ROS message.
    Example:
        message_type = "std_msgs/String"
        dict_message = { "data": "Hello, Robot" }
        ros_message = convert_dictionary_to_ros_message(message_type, dict_message)
    """
    message_class = roslib.message.get_message_class(message_type)
    message = message_class()
    message_fields = dict(_get_message_fields(message))

    for field_name, field_value in dictionary.items():
        if field_name in message_fields:
            field_type = message_fields[field_name]
            field_value = _convert_to_ros_type(field_type, field_value)
            setattr(message, field_name, field_value)
        else:
            error_message = 'ROS message type "{0}" has no field named "{1}"'\
                .format(message_type, field_name)
            raise ValueError(error_message)

    return message

def _convert_to_ros_type(field_type, field_value):
    if is_ros_binary_type(field_type, field_value):
        field_value = _convert_to_ros_binary(field_type, field_value)
    elif field_type in ros_time_types:
        field_value = _convert_to_ros_time(field_type, field_value)
    elif field_type in ros_primitive_types:
        field_value = _convert_to_ros_primitive(field_type, field_value)
    elif _is_field_type_an_array(field_type):
        field_value = _convert_to_ros_array(field_type, field_value)
    else:
        field_value = convert_dictionary_to_ros_message(field_type, field_value)

    return field_value

def _convert_to_ros_binary(field_type, field_value):
    binary_value_as_string = field_value
    if type(field_value) in python_string_types:
        binary_value_as_string = base64.standard_b64decode(field_value)
    else:
        binary_value_as_string = str(bytearray(field_value))

    return binary_value_as_string

def _convert_to_ros_time(field_type, field_value):
    time = None

    if field_type == 'time' and field_value == 'now':
        time = rospy.get_rostime()
    else:
        if field_type == 'time':
            time = rospy.rostime.Time()
        elif field_type == 'duration':
            time = rospy.rostime.Duration()
        if 'secs' in field_value:
            setattr(time, 'secs', field_value['secs'])
        if 'nsecs' in field_value:
            setattr(time, 'nsecs', field_value['nsecs'])

    return time

def _convert_to_ros_primitive(field_type, field_value):
    return field_value

def _convert_to_ros_array(field_type, list_value):
    list_type = list_brackets.sub('', field_type)
    return [_convert_to_ros_type(list_type, value) for value in list_value]

def convert_ros_message_to_dictionary(message):
    """
    Takes in a ROS message and returns a Python dictionary.
    Example:
        ros_message = std_msgs.msg.String(data="Hello, Robot")
        dict_message = convert_ros_message_to_dictionary(ros_message)
    """
    dictionary = {}
    message_fields = _get_message_fields(message)
    for field_name, field_type in message_fields:
        field_value = getattr(message, field_name)
        dictionary[field_name] = _convert_from_ros_type(field_type, field_value)

    return dictionary

def _convert_from_ros_type(field_type, field_value):
    if is_ros_binary_type(field_type, field_value):
        field_value = _convert_from_ros_binary(field_type, field_value)
    elif field_type in ros_time_types:
        field_value = _convert_from_ros_time(field_type, field_value)
    elif field_type in ros_primitive_types:
        field_value = field_value
    elif _is_field_type_an_array(field_type):
        field_value = _convert_from_ros_array(field_type, field_value)
    else:
        field_value = convert_ros_message_to_dictionary(field_value)

    return field_value


def is_ros_binary_type(field_type, field_value):
    """ Checks if the field is a binary array one, fixed size or not
    is_ros_binary_type("uint8", 42)
    >>> False
    is_ros_binary_type("uint8[]", [42, 18])
    >>> True
    is_ros_binary_type("uint8[3]", [42, 18, 21]
    >>> True
    is_ros_binary_type("char", 42)
    >>> False
    is_ros_binary_type("char[]", [42, 18])
    >>> True
    is_ros_binary_type("char[3]", [42, 18, 21]
    >>> True
    """
    return re.search(ros_binary_types_regexp, field_type) is not None

def _convert_from_ros_binary(field_type, field_value):
    field_value = base64.standard_b64encode(field_value)
    return field_value

def _convert_from_ros_time(field_type, field_value):
    field_value = {
        'secs'  : field_value.secs,
        'nsecs' : field_value.nsecs
    }
    return field_value

def _convert_from_ros_primitive(field_type, field_value):
    return field_value

def _convert_from_ros_array(field_type, field_value):
    list_type = list_brackets.sub('', field_type)
    return [_convert_from_ros_type(list_type, value) for value in field_value]

def _get_message_fields(message):
    return zip(message.__slots__, message._slot_types)

def _is_field_type_an_array(field_type):
    return list_brackets.search(field_type) is not None

def convert_json_to_ros_message(message_type, json_message):
    """
    Takes in the message type and a JSON-formatted string and returns a ROS
    message.
    Example:
        message_type = "std_msgs/String"
        json_message = '{"data": "Hello, Robot"}'
        ros_message = convert_json_to_ros_message(message_type, json_message)
    """
    dictionary = json.loads(json_message)
    return convert_dictionary_to_ros_message(message_type, dictionary)

def convert_ros_message_to_json(message):
    """
    Takes in a ROS message and returns a JSON-formatted string.
    Example:
        ros_message = std_msgs.msg.String(data="Hello, Robot")
        json_message = convert_ros_message_to_json(ros_message)
    """
    dictionary = convert_ros_message_to_dictionary(message)
    json_message = json.dumps(dictionary)
    return json_message
