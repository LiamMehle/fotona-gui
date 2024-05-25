#cython: language_level=3
import rospy
from cython import parallel
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header, Time
import numpy as np
cimport numpy as cnp
import time as t
import cython
# import torch

@cython.cdivision(True)
def generate_pointcloud(seq: int, time: Time) -> PointCloud2:
    # x_dim       = 171
    # y_dim       = 224
    cdef int x_dim      = 800
    cdef int y_dim      = x_dim
    cdef int point_step = (4*(32) + 16 + 8 )//8
    
    header          = Header()
    header.seq      = seq
    header.stamp    = time
    header.frame_id = 'pico_flexx_optical_frame'

    point_fields = [
        PointField('x',          0, 7, 1),  # float32
        PointField('y',          4, 7, 1),
        PointField('z',          8, 7, 1),
        PointField('noise',     12, 7, 1),
        PointField('intensity', 16, 4, 1),  # uint16
        PointField('gray',      18, 2, 1),  # uint8
    ]
    now = t.perf_counter() / 5
    center = np.array([np.sin(now), np.cos(now)]) / 2
    cdef float x_min = center[1]-1, x_max = center[1]+1
    cdef float y_min = center[0]-1, y_max = center[0]+1

    cdef cnp.ndarray[float, ndim=3] data = np.zeros((y_dim, x_dim, 3), dtype=np.float32)
    cdef float xp, yp
    cdef int x, y
    for y in parallel.prange(y_dim, nogil=True, schedule='static', num_threads=4):
        for x in range(x_dim):
            xp = x
            yp = y
            xp = (xp/x_dim)*(x_max-x_min)+x_min
            yp = (yp/y_dim)*(y_max-y_min)+y_min
            data[y, x, 0] = xp
            data[y, x, 1] = yp
            data[y, x, 2] = 0 #xp * yp

#   cdef cnp.ndarray[float, ndim=2] x = np.linspace(y_min, y_max, x_dim, dtype=np.float32).reshape(1, -1)
#   cdef cnp.ndarray[float, ndim=2] y = np.linspace(x_min, x_max, y_dim, dtype=np.float32).reshape(-1, 1)
#   cdef cnp.ndarray[float, ndim=2] z = x * y

    # data duplication and reformatting?
    z_max = data[:,:,2].max()
    z_min = data[:,:,2].min()
#   data[np.arange(y_dim),:,0] = x
#   data[:,np.arange(x_dim),1] = y
#   data[:,:,2] = z

    # data packing into expected shape
    raw_data = np.zeros((y_dim, x_dim, point_step), dtype=np.byte)
    intensity = (data[:,:,2] - z_min) / (z_max - z_min) * (2**16-1)
    xyz_range       = np.arange(0,  12)
    intensity_range = np.arange(16, 18)
    raw_data[:,:,xyz_range]       = np.frombuffer(data.tobytes(),              dtype=np.byte).reshape((y_dim, x_dim, -1))
    raw_data[:,:,intensity_range] = np.frombuffer(intensity.astype(np.int16).tobytes(), dtype=np.byte).reshape((y_dim, x_dim, -1))
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

cdef test_callback(message: PointCloud2):
    print('got a callback')

def talker():
    rospy.init_node('talker', anonymous=False)
    pub = rospy.Publisher('/pico_flexx/points', PointCloud2, queue_size=1024)
    # sub = rospy.Subscriber('/pico_flexx/points', PointCloud2, test_callback, queue_size=1024)
    rate = rospy.Rate(30)
    seq = 0
    while not rospy.is_shutdown():
        if pub.get_num_connections() != 0:
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
