#!/usr/bin/env python2

import rospy
import rosbag
from geometry_msgs.msg import Transform, TransformStamped, PointStamped
from tf2_msgs.msg import TFMessage
from sensor_msgs.msg import NavSatFix, NavSatStatus

from pos2bag.pos_parser import PosParser
from pos2bag.llh2xyz import LLH2XYZ

import argparse
import datetime
import pytz
import sys


def parse_args():
    description = 'Converts a .pos file to .bag'
    parser = argparse.ArgumentParser(description=description,
                                     add_help=False)
    parser.add_argument('-h', '--help', action='help',
                        help='show this help message and exit')
    parser.add_argument('--source-epsg',
                        dest='src_epsg',
                        default=4326,
                        nargs=1,
                        metavar='EPSG',
                        help='EPSG used for the lat-lon-height')
    parser.add_argument('-e', '--target-epsg',
                        dest='tgt_epsg',
                        default=32654,
                        nargs=1,
                        metavar='EPSG',
                        help='EPSG used for the lat-lon-height')
    parser.add_argument('-g', '--gnss-frame',
                        dest='frame_id',
                        default='gnss_origin',
                        action='store',
                        nargs=1,
                        metavar='FRAME',
                        help='frame_id of the GNSS origin')
    parser.add_argument('-r', '--receiver-frame',
                        dest='child_frame_id',
                        default='antenna',
                        action='store',
                        nargs=1,
                        metavar='FRAME',
                        help='frame_id of the receiver')
    parser.add_argument('--no-tf',
                        dest='no_tf',
                        action='store_true',
                        help='do not write out TF transforms')
    parser.add_argument('--no-points',
                        dest='no_points',
                        action='store_true',
                        help='do not write out GNSS points')
    parser.add_argument('--no-fix',
                        dest='no_fix',
                        action='store_true',
                        help='do not write out NavSatFix messages')
    parser.add_argument('-p', '--point-topic',
                        dest='point_topic',
                        default='/gnss_point',
                        action='store',
                        nargs=1,
                        metavar='TOPIC_NAME',
                        help='topic name for the point')
    parser.add_argument('-f', '--fix-topic',
                        dest='fix_topic',
                        default='/fix',
                        action='store',
                        nargs=1,
                        metavar='TOPIC_NAME',
                        help='topic name for the fix')
    parser.add_argument('input_pos',
                        help='pos file used for input')
    parser.add_argument('output_bag',
                        help='bag file to output')
    args = parser.parse_args()
    return args


def make_transform(t, x, y, z, args):
    tform = TransformStamped()
    tform.header.frame_id = args.frame_id
    tform.header.stamp = t
    tform.child_frame_id = args.child_frame_id
    tform.transform.translation.x = x
    tform.transform.translation.y = y
    tform.transform.translation.z = z
    tform.transform.rotation.x = 0.0
    tform.transform.rotation.y = 0.0
    tform.transform.rotation.z = 0.0
    tform.transform.rotation.w = 1.0

    msg = TFMessage()
    msg.transforms.append(tform)
    return msg


def make_fix(t, data, args):
    fix = NavSatFix()
    fix.header.stamp = t
    fix.header.frame_id = args.frame_id
    if data.Q in (1, 2, 4):
        # Fix, Float, GPS
        fix.status.status = NavSatStatus.STATUS_GBAS_FIX
    elif data.Q == 3:
        # SBAS
        fix.status.status = NavSatStatus.STATUS_SBAS_FIX
    elif data.Q == 5:
        # Single
        fix.status.status = NavSatStatus.STATUS_FIX
    else:
        fix.status.status = NavSatStatus.STATUS_NO_FIX
    fix.latitude = data.lat
    fix.longitude = data.lon
    fix.altitude = data.height
    fix.position_covariance = [
        data.sde ** 2, data.sdne ** 2, data.sdeu ** 2,
        data.sdne ** 2, data.sdn ** 2, data.sdun ** 2,
        data.sdeu ** 2, data.sdun ** 2, data.sdu ** 2
    ]
    fix.position_covariance_type = NavSatFix.COVARIANCE_TYPE_KNOWN

    return fix


def main():
    args = parse_args()

    if 'input_pos' not in args or 'output_bag' not in args:
        sys.stderr.write(args.usage())
        sys.exit(1)

    all_data = PosParser.parse(args.input_pos)
    out_bag = rosbag.Bag(args.output_bag, 'w')

    epoch = datetime.datetime.fromtimestamp(0, tz=pytz.UTC)
    for data in all_data:
        t = rospy.Time((data.datetime - epoch).total_seconds())
        converter = LLH2XYZ(src_epsg=args.src_epsg, tgt_epsg=args.tgt_epsg)
        x, y, z = converter.convert(data.lat, data.lon, data.height)

        # TF
        if not args.no_tf:
            msg = make_transform(t, x, y, z, args)
            out_bag.write('/tf', msg, t)

        # PointStamped
        if not args.no_points:
            ps = PointStamped()
            ps.header.frame_id = args.frame_id
            ps.header.stamp = t
            ps.point.x = x
            ps.point.y = y
            ps.point.z = z
            out_bag.write(args.point_topic, ps, t)

        # Fix
        if not args.no_fix:
            msg = make_fix(t, data, args)
            out_bag.write(args.fix_topic, msg, t)

    out_bag.close()


if __name__ == '__main__':
    main()
