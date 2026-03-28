import struct

import numpy as np


def build_pointcloud2(header_stamp, frame_id, points):
    from sensor_msgs.msg import PointCloud2, PointField
    from std_msgs.msg import Header

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


def unpack_pointcloud2(cloud):
    field_map = {field.name: field.offset for field in cloud.fields}
    required = ["x", "y", "z"]

    for name in required:
        if name not in field_map:
            raise ValueError(f"missing field in cloud: {name}")

    intensity_offset = field_map.get("intensity")
    points = []

    for idx in range(cloud.width * cloud.height):
        base = idx * cloud.point_step
        x = struct.unpack_from("f", cloud.data, base + field_map["x"])[0]
        y = struct.unpack_from("f", cloud.data, base + field_map["y"])[0]
        z = struct.unpack_from("f", cloud.data, base + field_map["z"])[0]
        intensity = 0.0
        if intensity_offset is not None:
            intensity = struct.unpack_from("f", cloud.data, base + intensity_offset)[0]
        points.append((x, y, z, intensity))

    return points


def make_bev_config(
    x_min=-40.0,
    x_max=40.0,
    y_min=-40.0,
    y_max=40.0,
    z_min=-3.0,
    z_max=2.0,
    resolution=0.16,
    num_height_bins=8,
):
    width = int(round((x_max - x_min) / resolution))
    height = int(round((y_max - y_min) / resolution))

    return {
        "x_min": float(x_min),
        "x_max": float(x_max),
        "y_min": float(y_min),
        "y_max": float(y_max),
        "z_min": float(z_min),
        "z_max": float(z_max),
        "resolution": float(resolution),
        "num_height_bins": int(num_height_bins),
        "width": width,
        "height": height,
    }


def point_to_bev_index(x, y, z, config):
    if x < config["x_min"] or x >= config["x_max"]:
        return None
    if y < config["y_min"] or y >= config["y_max"]:
        return None
    if z < config["z_min"] or z > config["z_max"]:
        return None

    col = int((x - config["x_min"]) / config["resolution"])
    row = int((y - config["y_min"]) / config["resolution"])

    if col < 0 or col >= config["width"]:
        return None
    if row < 0 or row >= config["height"]:
        return None

    z_span = config["z_max"] - config["z_min"]
    z_pos = (z - config["z_min"]) / z_span
    bin_idx = int(z_pos * config["num_height_bins"])
    bin_idx = min(max(bin_idx, 0), config["num_height_bins"] - 1)

    return row, col, bin_idx


def project_points_to_bev(points, config):
    bev = np.zeros(
        (config["height"], config["width"], config["num_height_bins"]),
        dtype=np.float32,
    )

    for x, y, z, _ in points:
        idx = point_to_bev_index(x, y, z, config)
        if idx is None:
            continue
        row, col, bin_idx = idx
        bev[row, col, bin_idx] += 1.0

    return bev


def build_preview_image(bev, mode="max_bin"):
    if bev.ndim != 3:
        raise ValueError("bev must be a 3D array")

    occ = bev > 0.0
    img = np.zeros(bev.shape[:2], dtype=np.uint8)

    if mode != "max_bin":
        raise ValueError(f"unsupported preview mode: {mode}")

    rows, cols = np.where(occ.any(axis=2))
    for row, col in zip(rows, cols):
        bin_idx = int(np.where(occ[row, col])[0].max())
        level = (bin_idx + 1) / bev.shape[2]
        img[row, col] = int(255 * level)

    return img


def build_bev_tensor_msg(bev):
    from std_msgs.msg import Float32MultiArray, MultiArrayDimension

    msg = Float32MultiArray()
    msg.layout.dim = [
        MultiArrayDimension(label="rows", size=bev.shape[0], stride=int(bev.size)),
        MultiArrayDimension(
            label="cols", size=bev.shape[1], stride=int(bev.shape[1] * bev.shape[2])
        ),
        MultiArrayDimension(label="bins", size=bev.shape[2], stride=int(bev.shape[2])),
    ]
    msg.data = bev.reshape(-1).astype(np.float32).tolist()
    return msg


def build_mono_image_msg(header_stamp, frame_id, img):
    from sensor_msgs.msg import Image

    msg = Image()
    msg.header.stamp = header_stamp
    msg.header.frame_id = frame_id
    msg.height = int(img.shape[0])
    msg.width = int(img.shape[1])
    msg.encoding = "mono8"
    msg.is_bigendian = False
    msg.step = int(img.shape[1])
    msg.data = img.tobytes()
    return msg


def run_euclidean_clustering(
    points,
    distance_threshold=1.2,
    min_cluster_size=5,
    max_cluster_size=200,
    cluster_z_min=-1.0,
    cluster_z_max=2.0,
):
    cloud = [
        point
        for point in points
        if cluster_z_min <= point[2] <= cluster_z_max
    ]
    clusters = []
    visited = [False] * len(cloud)
    dist_sq = distance_threshold * distance_threshold

    for start_idx in range(len(cloud)):
        if visited[start_idx]:
            continue

        queue = [start_idx]
        visited[start_idx] = True
        cluster = []

        while queue:
            idx = queue.pop()
            cluster.append(cloud[idx])
            px, py, pz, _ = cloud[idx]

            for next_idx in range(len(cloud)):
                if visited[next_idx]:
                    continue

                qx, qy, qz, _ = cloud[next_idx]
                dx = px - qx
                dy = py - qy
                dz = pz - qz

                if dx * dx + dy * dy + dz * dz <= dist_sq:
                    visited[next_idx] = True
                    queue.append(next_idx)

        if min_cluster_size <= len(cluster) <= max_cluster_size:
            clusters.append(cluster)

    return clusters


def summarize_cluster(cluster):
    xyz = np.array([[x, y, z] for x, y, z, _ in cluster], dtype=np.float32)
    mins = xyz.min(axis=0)
    maxs = xyz.max(axis=0)
    center = (mins + maxs) / 2.0
    size = np.maximum(maxs - mins, np.array([0.2, 0.2, 0.2], dtype=np.float32))

    return {
        "center_x": float(center[0]),
        "center_y": float(center[1]),
        "center_z": float(center[2]),
        "size_x": float(size[0]),
        "size_y": float(size[1]),
        "size_z": float(size[2]),
        "count": int(len(cluster)),
    }


def build_cluster_markers(header_stamp, frame_id, summaries):
    from visualization_msgs.msg import Marker, MarkerArray

    markers = MarkerArray()

    clear_marker = Marker()
    clear_marker.header.stamp = header_stamp
    clear_marker.header.frame_id = frame_id
    clear_marker.action = Marker.DELETEALL
    markers.markers.append(clear_marker)

    for idx, summary in enumerate(summaries):
        marker = Marker()
        marker.header.stamp = header_stamp
        marker.header.frame_id = frame_id
        marker.ns = "clusters"
        marker.id = idx
        marker.type = Marker.CUBE
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0
        marker.pose.position.x = summary["center_x"]
        marker.pose.position.y = summary["center_y"]
        marker.pose.position.z = summary["center_z"]
        marker.scale.x = summary["size_x"]
        marker.scale.y = summary["size_y"]
        marker.scale.z = summary["size_z"]
        marker.color.r = 1.0
        marker.color.g = 0.55
        marker.color.b = 0.15
        marker.color.a = 0.75
        markers.markers.append(marker)

    return markers
