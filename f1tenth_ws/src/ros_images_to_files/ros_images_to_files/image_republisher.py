"""
Notes:
    * image transport python binding is not available in ROS2 humble
    * Use C++ instead
    * use image transport for theora support
Features:
 * subscribes to images/pointclouds with configurable QoS
 * supports compressed images
 * support automatic subscription to camera info
 * supports optional hardware encoding

Todo:
    * allow specifying raw image name and using the specified transport to add the correct topic name
"""

import os
import sys
import getopt
import subprocess

import numpy as np
import cv2
import rclpy

from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy import qos
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from cv_bridge import CvBridge

from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from sensor_msgs.msg import Image, CompressedImage

try:
    from theora_image_transport.msg import Packet
    THEORA_PACKET_AVAILABLE = True
except (ImportError, ModuleNotFoundError) as e:
    print("Theora image transport not available.")
    THEORA_PACKET_AVAILABLE = False

try:
    from image_transport_py import ImageTransport
    IMAGE_TRANSPORT_AVAILABLE = True
except (ImportError, ModuleNotFoundError) as e:
    print(f"image_tranport python bindings are only available in ROS2 Jazzy and above")
    IMAGE_TRANSPORT_AVAILABLE = True


class ImageRepublisherNode(Node):
    def __init__(self):
        """Constructor for ImageRepublisherNode"""
        super(ImageRepublisherNode, self).__init__('image_republisher')

        # declare parameters with defaults.
        self.declare_parameter(name='input_image_topic', value="in/image", descriptor=ParameterDescriptor(
                                   description='',
                                   type=ParameterType.PARAMETER_STRING))
        self.declare_parameter(name='output_image_topic', value="out/image", descriptor=ParameterDescriptor(
                                   description='',
                                   type=ParameterType.PARAMETER_BOOL))
        self.declare_parameter(name='input_image_transport', value="auto", descriptor=ParameterDescriptor(
                                   description='auto, raw, compressed',
                                   type=ParameterType.PARAMETER_STRING))
        self.declare_parameter(name='output_image_transport', value="raw", descriptor=ParameterDescriptor(
                description='auto, raw, compressed',
                type=ParameterType.PARAMETER_STRING))
        self.declare_parameter(name='republish_frequency', value=-1.0, descriptor=ParameterDescriptor(
                                   description='Used to throttle publishing frequency '
                                               'to rates less than the input message frequency. -1.0 for no throttling.',
                                   type=ParameterType.PARAMETER_DOUBLE
        ))
        self.declare_parameter(name='time_source', value='msg', descriptor=ParameterDescriptor(
            description='Whether to use the message timestamp or current clock time',
            type=ParameterType.PARAMETER_STRING
        ))
        self.declare_parameter(name='queue_size', value=10, descriptor=ParameterDescriptor(
                                   description='',
                                   type=ParameterType.PARAMETER_INTEGER))
        self.declare_parameter(name='input_qos', value="SENSOR_DATA", descriptor=ParameterDescriptor(
                                   description='',
                                   type=ParameterType.PARAMETER_STRING))
        self.declare_parameter(name='output_qos', value="SENSOR_DATA", descriptor=ParameterDescriptor(
                description='',
                type=ParameterType.PARAMETER_STRING))
        self.declare_parameter(name='show_image', value=True, descriptor=ParameterDescriptor(
            description='',
            type=ParameterType.PARAMETER_BOOL))

        # setup variables from parameters
        self.use_sim_time = self.get_parameter('use_sim_time').get_parameter_value().bool_value
        self.input_image_topic = self.get_parameter('input_image_topic').get_parameter_value().string_value
        self.output_image_topic = self.get_parameter('output_image_topic').get_parameter_value().string_value
        self.input_image_transport = self.get_parameter('input_image_transport').get_parameter_value().string_value
        self.output_image_transport = self.get_parameter('output_image_transport').get_parameter_value().string_value
        self.republish_frequency = self.get_parameter('republish_frequency').get_parameter_value().double_value
        self.time_source = self.get_parameter('time_source').get_parameter_value().string_value
        self.queue_size = self.get_parameter('queue_size').get_parameter_value().integer_value
        self.input_qos = self.get_parameter('input_qos').get_parameter_value().string_value
        self.output_qos = self.get_parameter('output_qos').get_parameter_value().string_value
        self.show_image = self.get_parameter('show_image').get_parameter_value().bool_value

        self.republish_dt = 1 / self.republish_frequency

        # check if the image transports are valid
        if self.input_image_transport not in ['auto', 'raw', 'compressed']:
            self.get_logger().error("Invalid input image transport: " + self.input_image_transport)
            sys.exit(1)
        if self.output_image_transport not in ['auto', 'raw', 'compressed']:
            self.get_logger().error("Invalid output image transport: " + self.output_image_transport)
            sys.exit(1)

        # Initialize variables
        self.frame_number = 0
        self.bridge = CvBridge()
        self.previous_time = None

        # setup QoS
        input_qos_profile = output_qos_profile = QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=self.queue_size
        )
        if self.input_qos.lower() == "sensor_data":
            input_qos_profile = QoSProfile(
                    reliability=QoSReliabilityPolicy.BEST_EFFORT,
                    history=QoSHistoryPolicy.KEEP_LAST,
                    depth=self.queue_size
            )

        if self.output_qos.lower() == "sensor_data":
            output_qos_profile = QoSProfile(
                    reliability=QoSReliabilityPolicy.BEST_EFFORT,
                    history=QoSHistoryPolicy.KEEP_LAST,
                    depth=self.queue_size
            )

        self.input_message_format = self.input_image_transport
        if self.input_image_transport == 'auto':
            if 'compressed' in self.input_image_topic:
                self.input_message_format = 'compressed'
            # elif 'theora' in self.input_image_topic:
            #     self.input_message_format = 'theora'
            else:
                self.input_message_format = 'raw'

        self.image_transport_to_msg_type = {
            'compressed': CompressedImage,
            'raw': Image
        }
        # if THEORA_PACKET_AVAILABLE:
        #     self.image_transport_to_msg_type['theora'] = Packet

        self.input_message_type = self.image_transport_to_msg_type.get(self.input_message_format, Image)

        self.output_message_format = self.output_image_transport
        if self.output_image_transport == 'auto':
            if 'compressed' in self.input_image_topic:
                self.output_message_format = 'compressed'
            # elif 'theora' in self.input_image_topic:
            #     self.output_message_format = 'theora'
            else:
                self.output_message_format = 'raw'
        self.output_message_type = self.image_transport_to_msg_type.get(self.output_message_format, Image)

        # setup image transport subscriber
        # self.sub_image_transport = ImageTransport(self, self.input_image_topic, self.output_image_topic,
        #                                       self.input_image_transport, self.output_image_transport, self.queue_size,
        #                                       self.input_qos, self.output_qos)

        # self.sub_image_transport = ImageTransport(
        #         'imagetransport_sub', image_transport='compressed'
        # )
        # image_transport.subscribe('camera/image', 10, self.image_callback)

        # setup subscribers
        self.image_subscriber = self.create_subscription(self.input_message_type, self.input_image_topic,
                                                         self.image_callback,
                                                         qos_profile=input_qos_profile)

        # setup publisher
        self.image_publisher = self.create_publisher(self.output_message_type,
                                                     self.output_image_topic,
                                                     qos_profile=input_qos_profile)

        self.get_logger().info(f"Started image_republisher node. Subscribing to image topic: {self.input_image_topic} with transport {self.input_image_transport}")

    def image_callback(self, msg):
        if self.time_source == 'msg':
            current_time = rclpy.time.Time.from_msg(msg.header.stamp)
        else:
            current_time = self.get_clock().now()

        if self.previous_time is None:
            if self.time_source == 'msg':
                self.previous_time = rclpy.time.Time.from_msg(msg.header.stamp)
            else:
                self.previous_time = self.get_clock().now()

        if self.republish_frequency > 0.0:
            if abs((current_time - self.previous_time).nanoseconds / 1e9) >= self.republish_dt:
                # self.get_logger().info(
                #     f"time difference in seconds: {abs((current_time - self.previous_time).nanoseconds / 1e9)}, {self.republish_dt}")
                self.previous_time = current_time
            else:
                return
        try:
            msg_fmt = 'passthrough'
            codec = 'jpg'
            if self.input_message_format == 'raw':
                msg_encoding = msg.encoding
                msg_fmt = "bgr8"
                is_color = True
                is_depth = False

            elif self.input_message_format == 'compressed':
                # format: rgb8; jpeg compressed bgr8
                msg_info = msg.format
                msg_encoding_split = msg_info.split(';')
                uncompressed_msg_fmt = msg_encoding_split[0]
                compressed_img_info =  msg_encoding_split[1].split()
                codec = compressed_img_info[0]
                msg_encoding = compressed_img_info[-1]

            # set the desired output encoding
            # (http://wiki.ros.org/cv_bridge/Tutorials/UsingCvBridgeToConvertBetweenROSImagesAndOpenCVImages#cv_bridge.2FTutorials.2FUsingCvBridgeCppDiamondback.Converting_ROS_image_messages_to_OpenCV_images)
            if (msg_encoding.find("mono8") != -1) or (msg_encoding.find("8UC1") != -1):
                msg_fmt = "8UC1"
                is_color = False
            elif (msg_encoding.find("bgra") != -1) or (msg_encoding.find("rgba") != -1):
                msg_fmt = "8UC4"
            elif (msg_encoding.find("bgr8") != -1) or (msg_encoding.find("rgb8") != -1):
                msg_fmt = "bgr8"  # or 8UC3
            elif msg_encoding.find("16UC1") != -1:
                msg_fmt = "16UC1"
                is_color = False
                is_depth = True
            else:
                self.get_logger().error("Unsupported encoding:", msg_encoding)
                self.exit(1)

            # convert ROS2 image message to OpenCV
            if self.input_message_format in ("compressed"):
                if msg_fmt != 'bgr8':
                    # due to a hardcoded bgr8 source format in compressed_imgmsg_to_cv2, must use passthrough for non bgr8 messages
                    msg_fmt = 'passthrough'
                cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, msg_fmt)
                # compressed_msg = CompressedImage()
                # compressed_msg.data = msg.data  # Conversion to JPEG required here
            else:
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding=msg_fmt)

            # publish the image
            if self.output_message_format in ("compressed", "packet"):
                cv_msg = self.bridge.cv2_to_compressed_imgmsg(cv_image)
            else:
                cv_msg = self.bridge.cv2_to_imgmsg(cv_image)

            cv_msg.header = msg.header
            self.image_publisher.publish(cv_msg)

            if self.show_image:
                cv2.imshow("camera", cv_image)
                cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error('Error processing image: %s' % str(e))


def main(args=None):
    rclpy.init(args=args)
    image_republisher_node = ImageRepublisherNode()
    try:
        rclpy.spin(image_republisher_node)
        image_republisher_node.destroy_node()
        rclpy.shutdown()
    except Exception as e:
        image_republisher_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
