import struct

from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header


def build_pointcloud2(header_stamp, frame_id, points):
    header = Header()
    header.stamp = header_stamp
    header.frame_id = frame_id

    fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
        PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
    ]

    data = b"".join(struct.pack("ffff", *point) for point in points)

    return PointCloud2(
        header=header,
        height=1,
        width=len(points),
        fields=fields,
        is_bigendian=False,
        point_step=16,
        row_step=16 * len(points),
        data=data,
        is_dense=True,
    )

