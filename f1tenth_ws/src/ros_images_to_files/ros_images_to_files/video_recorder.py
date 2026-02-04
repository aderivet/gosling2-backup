'''
Todo:
    * switch to image transport for more modularity and better compressed image support use try-catch to handle image transport not being installed (https://github.com/ros-perception/image_transport_tutorials?tab=readme-ov-file#py_simple_image_pub | https://github.com/ros-perception/image_transport_tutorials?tab=readme-ov-file#py_simple_image_pub)
    * Extend this package as a robust image tool with QoS and image transport (including republishing) support for publishing and subscribing (also add pointcloud support)
'''
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
except Exception:
    pass


def normalize_depth_image(depth_image, max_val=255, dtype=None):
    """

    :param depth_image:
    :param max_val: Either 1 or 255
    :param dtype: Either cv2.CV_32F or cv2.CV_8UC1
    :return:
    """
    if dtype is None:
        dtype = cv2.CV_8UC1
        if max_val == 1:
            dtype = cv2.CV_32F

    # note, if the depth is a single file and Opencv<4.7.0, then it has been normalized
    # We need to restore. CV_32F for 0-1 or CV_8UC1 for 0-255
    depth_image_normalized = cv2.normalize(depth_image, depth_image, 0, max_val, cv2.NORM_MINMAX,
                             dtype=dtype)  # 1 or 255 depends on datatype, 1 for float, e.g 32F and 255 for int eg 8U

    return depth_image_normalized


class VideoRecorderNode(Node):
    """docstring for ClassName"""
    
    def __init__(self):
        """Constructor for ViderRecorderNode"""
        super(VideoRecorderNode, self).__init__('image_to_video_recorder')

        # declare parameters with defaults.
        self.declare_parameter(name='image_topic', value="image", descriptor=ParameterDescriptor(
                                       description='',
                                       type=ParameterType.PARAMETER_STRING))
        self.declare_parameter(name='image_topic_is_compressed', value=False, descriptor=ParameterDescriptor(
                                       description='',
                                       type=ParameterType.PARAMETER_BOOL))
        self.declare_parameter(name='output_file_name', value="output.avi", descriptor=ParameterDescriptor(
                                       description='Suggested extensions: .avi, .mp4, .mpeg',
                                       type=ParameterType.PARAMETER_STRING))
        self.declare_parameter(name='queue_size', value=100, descriptor=ParameterDescriptor(
                                       description='',
                                       type=ParameterType.PARAMETER_INTEGER))
        self.declare_parameter(name='fps', value=30.0, descriptor=ParameterDescriptor(
                                       description='',
                                       type=ParameterType.PARAMETER_DOUBLE))
        self.declare_parameter(name='qos', value="SENSOR_DATA", descriptor=ParameterDescriptor(
                                       description='',
                                       type=ParameterType.PARAMETER_STRING))
        self.declare_parameter(name='normalize_depth', value=True, descriptor=ParameterDescriptor(
                description='',
                type=ParameterType.PARAMETER_BOOL))
        self.declare_parameter(name='normalized_max', value=255, descriptor=ParameterDescriptor(
                description='',
                type=ParameterType.PARAMETER_DOUBLE))
        self.declare_parameter(name='show_image', value=False, descriptor=ParameterDescriptor(
                                       description='',
                                       type=ParameterType.PARAMETER_BOOL))

        # setup variables from parameters
        self.use_sim_time = self.get_parameter('use_sim_time').get_parameter_value().bool_value
        self.image_topic = self.get_parameter('image_topic').get_parameter_value().string_value
        self.image_topic_is_compressed = self.get_parameter(
                'image_topic_is_compressed').get_parameter_value().bool_value
        self.output_file_name = self.get_parameter('output_file_name').get_parameter_value().string_value
        self.queue_size = self.get_parameter('queue_size').get_parameter_value().integer_value
        self.fps = self.get_parameter('fps').get_parameter_value().double_value
        self.qos = self.get_parameter('qos').get_parameter_value().string_value
        self.normalize_depth = self.get_parameter('normalize_depth').get_parameter_value().bool_value
        self.normalized_max = self.get_parameter('normalized_max').get_parameter_value().double_value
        self.show_image = self.get_parameter('show_image').get_parameter_value().bool_value

        # Initialize variables
        self.frame_number = 0
        self.bridge = CvBridge()

        # setup QoS
        qos_profile = QoSProfile(
                reliability=QoSReliabilityPolicy.RELIABLE,
                history=QoSHistoryPolicy.KEEP_LAST,
                depth=self.queue_size
        )
        if self.qos.lower() == "sensor_data":
            qos_profile = QoSProfile(
                    reliability=QoSReliabilityPolicy.BEST_EFFORT,
                    history=QoSHistoryPolicy.KEEP_LAST,
                    depth=self.queue_size
            )

        self.message_format = "raw"
        self.message_type = Image
        if self.image_topic_is_compressed or "compressed" in self.image_topic:
            self.message_format = "compressed"
            self.message_type = CompressedImage

        # Setup subscriber
        self.image_sub = self.create_subscription(
                self.message_type,
                self.image_topic,
                self.image_callback,
                qos_profile=qos_profile)

        self.video_writer = None
        self.get_logger().info(f"Started image_to_video_recorder node.")

    def image_callback(self, msg):
        self.frame_number += 1
        try:
            msg_encoding = msg.encoding
            msg_fmt = "bgr8"
            is_color = True
            is_depth = False

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
            if self.message_format in ("compressed", "packet"):
                cv_image = self.bridge.compressed_imgmsg_to_cv2(msg, msg_fmt)
            else:
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding=msg_fmt)

            # normalize depth to fall between 0 (black) and 1/255 (white) to write with video writer
            if self.normalize_depth and is_depth:
                # note, if the depth is a single file and Opencv<4.7.0, then it has been normalized
                # We need to restore
                cv_image = normalize_depth_image(cv_image, max_val=self.normalized_max, dtype=None)

            # save the images to a video
            if not is_depth or self.normalize_depth:
                if self.video_writer is None:
                    try:
                        size = cv_image.shape
                        height = size[0]
                        width = size[1]
                        # file_name = os.path.basename(os.path.abspath(self.output_file_name)).split('.')[0]
                        # file_extension = os.path.basename(os.path.abspath(self.output_file_name)).split('.')[1]
                        # video_filename = os.path.join(
                        #         os.path.join(f"{os.sep}".join(os.path.split(log_file2)[:-1])),
                        #         f"{file_name}.{file_extension}")
                        video_format = self.output_file_name[self.output_file_name.find(".") + 1:]
                        video_filename = self.output_file_name[:self.output_file_name.find(".") + 1] + video_format
                        # mp4: "mp4v", avi: "mpeg" [mjpg in some versions]
                        if video_format == 'avi':
                            fourcc = cv2.VideoWriter_fourcc(*'XVID')
                        elif video_format == 'mp4':
                            fourcc = cv2.VideoWriter_fourcc(*'mp4v')
                        else:
                            fourcc = cv2.VideoWriter_fourcc(*'mpeg')

                        self.get_logger().info(f"Writing to {video_filename}")

                        self.video_writer = cv2.VideoWriter(video_filename, fourcc, self.fps, (width, height), is_color)
                    except Exception as e:
                        self.get_logger().error('Error initializing video writer: %s' % str(e))

                self.video_writer.write(cv_image)

            else:
                '''For OpenCV versions before 4.7.0, VideoWriter does not support 16 bit values, 
                therefore we store depth images as a directory of frames.'''
                # # new_directory_name = self.output_file_name[:self.output_file_name.find(".") + 1]
                # # new_directory_name = os.path.basename(os.path.abspath(self.output_file_name)).split('.')[0]
                # if not os.path.exists(self.output_file_name):
                #     os.makedirs(os.path.dirname(self.output_file_name), exist_ok=True)

                # lossless 16 bit image in PNG format
                self.get_logger().info(f"Writing to {self.output_file_name}_{self.frame_number}.png")
                cv2.imwrite(filename=f"{self.output_file_name}_{self.frame_number}.png", img=cv_image)
                np.save(f"{self.output_file_name}_{self.frame_number}.npy", cv_image)

            if self.show_image:
                cv2.imshow("camera", cv_image)
                cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error('Error processing image: %s' % str(e))

    def destroy_node(self):
        if self.video_writer is not None:
            self.video_writer.release()
            self.get_logger().info('Closing the video writer.')
        super(VideoRecorderNode, self).destroy_node()


def main(args=None):
    rclpy.init(args=args)
    video_recorder_node = VideoRecorderNode()
    try:
        rclpy.spin(video_recorder_node)
        video_recorder_node.destroy_node()
        rclpy.shutdown()

    except Exception as e:
        video_recorder_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()