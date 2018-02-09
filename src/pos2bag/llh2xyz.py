#!/usr/bin/env python2

from osgeo import osr
from geometry_msgs.msg import Transform


class LLH2XYZ:
    def __init__(self, src_epsg, tgt_epsg):
        self.src_sr = osr.SpatialReference()
        self.src_sr.ImportFromEPSG(src_epsg)
        self.tgt_sr = osr.SpatialReference()
        self.tgt_sr.ImportFromEPSG(tgt_epsg)
        self.converter = osr.CoordinateTransformation(self.src_sr,
                                                      self.tgt_sr)

    def convert(self, lat, lon, height):
        x, y, z = self.converter.TransformPoint(lon, lat, height)
        return x, y, z

    def make_tf(self, data):
        tf = Transform()
        x, y, z = self.convert(data.lat, data.lon, data.height)
        tf.translation.x = x
        tf.translation.y = y
        tf.translation.z = z
        tf.rotation.x = 0.0
        tf.rotation.y = 0.0
        tf.rotation.z = 0.0
        tf.rotation.w = 1.0
        return tf
