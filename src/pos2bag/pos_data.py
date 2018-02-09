#!/usr/bin/env python2

import datetime
import pytz


class PosData:
    # Data quality
    Q_FIX = 1
    Q_FLOAT = 2
    Q_SBAS = 3
    Q_DGPS = 4
    Q_SINGLE = 5
    Q_PPP = 6

    def __init__(self):
        self.datetime = None
        self.lat = None
        self.lon = None
        self.height = None
        self.Q = None
        self.ns = None
        self.sdn = None
        self.sde = None
        self.sdu = None
        self.sdne = None
        self.sdeu = None
        self.sdun = None
        self.age = None
        self.ratio = None

    @staticmethod
    def from_str(line):
        vals = line.strip().split()
        if len(vals) != 15:
            raise ValueError('Invalid line in .pos file: ' + line)

        d = PosData()
        date_str, time_str, d.lat, d.lon, d.height,\
            d.Q, d.ns, d.sdn, d.sde, d.sdu,\
            d.sdne, d.sdeu, d.sdun, d.age, d.ratio = vals
        dt = datetime.datetime.strptime(date_str + ' ' + time_str,
                                        '%Y/%m/%d %H:%M:%S.%f')
        # Timezone is in UTC for pos files
        dt = dt.replace(tzinfo=pytz.UTC)

        # Convert to the correct data types
        d.datetime = dt
        d.lat = float(d.lat)
        d.lon = float(d.lon)
        d.height = float(d.height)
        d.Q = int(d.Q)
        d.ns = int(d.ns)
        d.sdn = float(d.sdn)
        d.sde = float(d.sde)
        d.sdu = float(d.sdu)
        d.sdne = float(d.sdne)
        d.sdeu = float(d.sdeu)
        d.sdun = float(d.sdun)
        d.age = float(d.age)
        d.ratio = float(d.ratio)
        return d
