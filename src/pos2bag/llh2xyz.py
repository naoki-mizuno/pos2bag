#!/usr/bin/env python2

from osgeo import osr


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
