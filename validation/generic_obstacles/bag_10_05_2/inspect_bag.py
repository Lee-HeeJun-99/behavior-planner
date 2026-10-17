#!/usr/bin/env python3
"""Inventory and recorded-output audit; requires ROS Python types, no ROS node."""
import json
import math
import sqlite3
from pathlib import Path
from rclpy.serialization import deserialize_message
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import PointCloud2

root = Path(__file__).resolve().parent
db = sqlite3.connect('file:/home/wgr/behavior_ws/10_05_2/10_05_2_0.db3?mode=ro', uri=True)
topics = [dict(zip(('topic', 'type', 'count', 'first_ns', 'last_ns'), row))
          for row in db.execute('SELECT t.name,t.type,count(m.id),min(m.timestamp),'
                                'max(m.timestamp) FROM topics t LEFT JOIN messages m '
                                'ON t.id=m.topic_id GROUP BY t.id')]
(root / 'topics.json').write_text(json.dumps(topics, indent=2))
recorded = []
for timestamp, data in db.execute("SELECT m.timestamp,m.data FROM messages m JOIN topics t "
                                 "ON m.topic_id=t.id WHERE t.name='/LiDAR/object_cen' "
                                 "ORDER BY m.timestamp,m.id"):
    values = list(deserialize_message(data, Float64MultiArray).data)
    recorded.append(dict(timestamp_ns=timestamp, count=len(values)//2,
                         trailing_sentinel=bool(values and values[-1] == -1000),
                         finite=all(math.isfinite(v) for v in values)))
(root / 'recorded_centers.json').write_text(json.dumps(recorded, indent=2))
raw = db.execute("SELECT m.data FROM messages m JOIN topics t ON m.topic_id=t.id "
                 "WHERE t.name='/velodyne_points' ORDER BY m.timestamp,m.id LIMIT 1").fetchone()
message = deserialize_message(raw[0], PointCloud2)
(root / 'cloud_schema.json').write_text(json.dumps(dict(
    frame_id=message.header.frame_id, height=message.height, width=message.width,
    is_dense=message.is_dense, is_bigendian=message.is_bigendian,
    point_step=message.point_step, row_step=message.row_step,
    fields=[dict(name=f.name, offset=f.offset, datatype=f.datatype, count=f.count)
            for f in message.fields]), indent=2))
db.close()
