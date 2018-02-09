#!/usr/bin/env python2

import six
from .pos_data import PosData


class PosParser:
    def __init__(self):
        pass

    @staticmethod
    def parse(filename, after=None, before=None):
        data = []
        if isinstance(filename, six.string_types):
            fh = open(filename)
        else:
            fh = filename

        for line in fh.readlines():
            # Comment lines start with a %
            if line.startswith('%'):
                continue
            d = PosData.from_str(line)
            if after is not None and d.datetime < after:
                continue
            if before is not None and d.datetime > before:
                continue
            data.append(d)
        return data
