#!/usr/bin/env python2

import rospy
import rosbag
from geometry_msgs.msg import TransformStamped, PointStamped
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import NavSatFix, NavSatStatus

from pos2bag.pos_parser import PosParser
from pos2bag.llh2xyz import LLH2XYZ

import datetime
import pytz
import sys


rospy.init_node('pos2bag_node')

src_epsg = rospy.get_param('~src_epsg', 4326)
# Japan Plane Rectangular CS X (JGD2011)
tgt_epsg = rospy.get_param('~tgt_epsg', 6678)
frame_id = rospy.get_param('~frame_id', 'gps_origin')
child_frame_id = rospy.get_param('~child_frame_id', 'gps_antenna')
fix_topic_name = rospy.get_param('~fix_topic_name', 'fix')
point_topic_name = rospy.get_param('~point_topic_name', 'gnss_point')

if len(sys.argv) < 3:
    sys.stderr.write('Not enough arguments\n')
    sys.exit(1)

filename = sys.argv[1]
out_bag = rosbag.Bag(sys.argv[2], 'w')

data = PosParser.parse(filename)

converter = LLH2XYZ(src_epsg=src_epsg, tgt_epsg=tgt_epsg)
epoch = datetime.datetime.fromtimestamp(0, tz=pytz.UTC)
for d in data:
    t = rospy.Time((d.datetime - epoch).total_seconds())

    # TF
    tf_stamped = TransformStamped()
    tf_stamped.header.frame_id = frame_id
    tf_stamped.header.stamp = t
    tf_stamped.child_frame_id = child_frame_id
    tf_stamped.transform = converter.make_tf(d)
    tf_msg = TFMessage()
    tf_msg.transforms.append(tf_stamped)
    out_bag.write('/tf', tf_msg, t)

    # PointStamped
    ps = PointStamped()
    ps.header.frame_id = frame_id
    ps.header.stamp = t
    ps.point.x = tf_stamped.transform.translation.x
    ps.point.y = tf_stamped.transform.translation.y
    ps.point.z = tf_stamped.transform.translation.z
    out_bag.write(point_topic_name, ps, t)

    # Fix
    fix = NavSatFix()
    fix.header.stamp = t
    fix.header.frame_id = frame_id
    if d.Q in (1, 2):
        # Fix, Float
        fix.status.status = NavSatStatus.STATUS_GBAS_FIX
    elif d.Q in (3, 4):
        # SBAS, DGPS
        fix.status.status = NavSatStatus.STATUS_SBAS_FIX
    elif d.Q == 5:
        # Single
        fix.status.status = NavSatStatus.STATUS_FIX
    else:
        fix.status.status = NavSatStatus.STATUS_NO_FIX
    fix.latitude = d.lat
    fix.longitude = d.lon
    fix.altitude = d.height
    fix.position_covariance = [
        d.sde**2,  d.sdne**2, d.sdeu**2,
        d.sdne**2, d.sdn**2,  d.sdun**2,
        d.sdeu**2, d.sdun**2, d.sdu**2
    ]
    fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_KNOWN
    out_bag.write(fix_topic_name, fix, t)

out_bag.close()
