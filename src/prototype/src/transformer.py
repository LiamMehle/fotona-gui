#!/bin/python3
import rospy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header, Time
import numpy as np


def hsv_to_rgb(h:np.ndarray, s:np.ndarray, v:np.ndarray) -> np.ndarray:
    zero = np.zeros_like(h)
    c = v * s
    x = c * ( 1 - np.abs( (h / (2*np.pi/3) ) % 2 - 1) )
    m = v - c
    color_section = (h/(2*np.pi/3)) // 1
    rgb = np.zeros(shape=(color_section.shape[0], 3))
    rgb[color_section==0] = np.array([c, x, zero]).T[color_section==0]
    rgb[color_section==1] = np.array([x, c, zero]).T[color_section==1]
    rgb[color_section==2] = np.array([zero, c, x]).T[color_section==2]
    rgb[color_section==3] = np.array([zero, x, c]).T[color_section==3]
    rgb[color_section==4] = np.array([x, zero, c]).T[color_section==4]
    rgb[color_section==5] = np.array([c, zero, x]).T[color_section==5]
    return rgb


def color(height: np.ndarray) -> np.ndarray:
    max_height = height.max()
    min_height = height.min()
    normalized_height = (height - min_height) / (max_height - min_height)
    
    hue = normalized_height * 2 * np.pi
    
    saturation = np.full(hue.shape, .8)
    value      = np.full(hue.shape, 1)
    return hsv_to_rgb(hue, saturation, value)


def process_cloud(input_cloud: PointCloud2, sensor_publisher: rospy.Publisher) -> None:
    # every point is 19 bytes
    raw_bytes = np.frombuffer(input_cloud.data, dtype=np.uint8).reshape(-1, 19)
    # array of `points`
    coords    = np.frombuffer(raw_bytes[:,:4*3].tobytes(), dtype=np.float32).reshape(-1, 3)
    # xyzrgb
    new_data  = np.zeros(shape=(coords.shape[0], 6), dtype=np.float32)
    new_data[:,:3] = coords
    new_data[:,3:] = color(coords[:,2])
    output_point_cloud = input_cloud
    output_point_cloud.fields = [
        PointField('x', 0,  7, 1),
        PointField('y', 4,  7, 1),
        PointField('z', 8,  7, 1),
        PointField('r', 12, 7, 1),
        PointField('g', 16, 7, 1),
        PointField('b', 20, 7, 1),
    ]
    output_point_cloud.data = new_data.tobytes()
    output_point_cloud.point_step = len(output_point_cloud.data) // output_point_cloud.width // output_point_cloud.height
    output_point_cloud.row_step   = output_point_cloud.point_step * output_point_cloud.width
    sensor_publisher.publish(output_point_cloud)

def main() -> None:
    rospy.init_node('transformer', anonymous=False)
    sensor_publisher  = rospy.Publisher('/pico_flexx/points_with_color', PointCloud2, queue_size=1024)
    sensor_subscriber = rospy.Subscriber('/pico_flexx/points',           PointCloud2, callback=process_cloud, callback_args=sensor_publisher, queue_size=1024)
    # sub = rospy.Subscriber('/pico_flexx/points', PointCloud2, test_callback, queue_size=1024)
    rate = rospy.Rate(33) # 10hz
    seq = 0
    while not rospy.is_shutdown():
        rate.sleep()

if __name__ == '__main__':
    main()