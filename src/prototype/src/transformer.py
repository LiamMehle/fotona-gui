#!/bin/python3
import rospy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header, Time
import numpy as np
import torch

@torch.jit.script
def hsv_to_rgb(h:torch.Tensor, s:torch.Tensor, v:torch.Tensor) -> torch.Tensor:
    zero = torch.zeros_like(h)
    c = v * s
    x = c * ( 1 - torch.abs( (h / (2*torch.pi/3) ) % 2 - 1) )
    m = v - c
    color_section = (h/(2*torch.pi/3)) // 1
    rgb = torch.zeros(size=(color_section.shape[0], 3), dtype=torch.float32)
    rgb[color_section==0] = torch.cat([c, x, zero]).reshape(3, -1).T[color_section==0]
    rgb[color_section==1] = torch.cat([x, c, zero]).reshape(3, -1).T[color_section==1]
    rgb[color_section==2] = torch.cat([zero, c, x]).reshape(3, -1).T[color_section==2]
    rgb[color_section==3] = torch.cat([zero, x, c]).reshape(3, -1).T[color_section==3]
    rgb[color_section==4] = torch.cat([x, zero, c]).reshape(3, -1).T[color_section==4]
    rgb[color_section==5] = torch.cat([c, zero, x]).reshape(3, -1).T[color_section==5]
    return rgb

@torch.jit.script
def color(height: torch.Tensor) -> torch.Tensor:
    max_height = height.max()
    min_height = height.min()
    normalized_height = (height - min_height) / (max_height - min_height)
    
    hue = normalized_height * 2 * torch.pi
    
    saturation = torch.full(hue.shape, .8)
    value      = torch.full(hue.shape, 1)
    return hsv_to_rgb(hue, saturation, value)


def process_cloud(input_cloud: PointCloud2, sensor_publisher: rospy.Publisher) -> None:
    # every point is 19 bytes
    raw_bytes = np.frombuffer(input_cloud.data, dtype=np.uint8).reshape(-1, 19)
    # array of `points`
    coords    = torch.tensor(np.frombuffer(raw_bytes[:,:4*3].tobytes(), dtype=np.float32)).reshape(-1, 3)
    # xyzrgb
    new_data  = torch.zeros((coords.shape[0], 6), dtype=torch.float32)
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
    output_point_cloud.data = new_data.numpy().tobytes()
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