#!/usr/bin/env python2
import fileinput

from CoppeliaSimConnector import CoppeliaSimConnector
from IIwaKinematics import IIwaKinematics
import PyKDL
import numpy as np


connector = CoppeliaSimConnector()
kin = IIwaKinematics(connector)

def transform_to_kdl(t):
    return PyKDL.Frame(PyKDL.Rotation.Quaternion(t.transform.rotation.x, t.transform.rotation.y,
                                                 t.transform.rotation.z, t.transform.rotation.w),
                       PyKDL.Vector(t.transform.translation.x,
                                    t.transform.translation.y,
                                    t.transform.translation.z))

rot = PyKDL.Rotation()
rot.DoRotX(-np.pi)

for line in fileinput.input():
    try:
        connector.step()
    except:
        pass
    line = line.strip()
    if line != "":
        try:
            kin.ptp(PyKDL.Frame(rot, PyKDL.Vector(float(line), 0.45, 0.225)), 1.0)
        except:
            pass

