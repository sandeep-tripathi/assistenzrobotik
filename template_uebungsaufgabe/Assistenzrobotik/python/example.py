#!/usr/bin/env python2

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


while True:
    rot = PyKDL.Rotation()
    rot.DoRotX(-np.pi)

    kin.ptp(PyKDL.Frame(rot, PyKDL.Vector(0.5, -0.5, 0.5)))

    kin.lin(PyKDL.Frame(rot, PyKDL.Vector(0.0, -0.5, 0.5)), 1)

    rot.DoRotZ(+np.pi)

    kin.ptp(PyKDL.Frame(rot, PyKDL.Vector(0.5, 0, 0.5)))
