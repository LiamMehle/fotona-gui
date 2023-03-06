#!/usr/bin/python3
import rospy
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header, Time
import numpy as np

def generate_pointcloud(seq: int, time: Time) -> PointCloud2:
    x_dim       = 171
    y_dim       = 224
    point_step  = (4*(32) + 16 + 8 )//8
    
    header          = Header()
    header.seq      = seq
    header.stamp    = time
    header.frame_id = 'pico_flexx_optical_frame'

    point_fields = [
        PointField('x',          0, 7, 1),
        PointField('y',          4, 7, 1),
        PointField('z',          8, 7, 1),
        PointField('noise',     12, 7, 1),
        PointField('intensity', 16, 7, 1),
        PointField('gray',      18, 7, 1),
    ]

    data = np.zeros((y_dim, x_dim, 3), dtype=np.float32)
    data[np.arange(y_dim),:,0] = np.linspace(-1, 1, x_dim, dtype=np.float32).reshape((1, -1))
    data[:,np.arange(x_dim),1] = np.linspace(-1, 1, y_dim, dtype=np.float32).reshape((-1, 1))
    data[:,:,2] = data[:,:,0] * data[:,:,1]

    raw_data = np.zeros((y_dim, x_dim, point_step), dtype=np.byte)
    raw_data[:,:,np.arange(3*4)] = np.frombuffer(data.tobytes(), dtype=np.byte).reshape((y_dim, x_dim, -1))

    cloud              = PointCloud2()
    # fake the data as observed
    cloud.header       = header
    cloud.height       = y_dim
    cloud.width        = x_dim
    cloud.fields       = point_fields
    cloud.is_bigendian = False
    cloud.point_step   = point_step
    cloud.row_step     = 4256
    cloud.data         = raw_data.tobytes()
    
    return cloud


def talker():
    rospy.init_node('talker', anonymous=False)
    pub = rospy.Publisher('/pico_flexx/points', PointCloud2, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    seq = 0
    while not rospy.is_shutdown():
        point_cloud = generate_pointcloud(seq, rospy.get_rostime())
        rospy.loginfo('sending pointcloud')
        pub.publish(point_cloud)
        seq +=1
        rate.sleep()

if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass