#!/usr/bin/env python3
"""Builds protos/Go1.proto for Webots from the Go1 URDF.

    xacro .../go1_description/xacro/robot.xacro > go1.urdf
    python3 tools/make_proto.py go1.urdf > protos/Go1.proto

Takes link frames, masses, inertias, collision shapes and joint limits from the URDF.
Rotor links are fixed in the URDF, so their mass is added to the parent link.
"""
import math
import sys
import xml.etree.ElementTree as ET

LEGS = ['FR', 'FL', 'RR', 'RL']
# The robot spawns in this pose: zero angles are outside the calf limits.
SPAWN = {'hip': 0.0, 'thigh': 0.8, 'calf': -1.5}
MESHES = '../../unitree_ws/src/unitree_ros2/go1_description/meshes/'
# Not measured on the robot: the lidar is assumed to sit on top of the trunk, in the middle.
LIDAR_XYZ = (0.0, 0.0, 0.10)


def floats(text):
    return [float(v) for v in text.split()]


def axis_angle(rpy):
    """URDF roll-pitch-yaw to a Webots axis and angle."""
    r, p, y = rpy
    cr, sr, cp, sp, cy, sy = (math.cos(r / 2), math.sin(r / 2), math.cos(p / 2),
                              math.sin(p / 2), math.cos(y / 2), math.sin(y / 2))
    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    yy = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    n = math.sqrt(x * x + yy * yy + z * z)
    if n < 1e-9:
        return '0 0 1 0'
    return f'{x / n:.6f} {yy / n:.6f} {z / n:.6f} {2 * math.atan2(n, w):.6f}'


def vec(values):
    return ' '.join(f'{v:.6g}' for v in values)


def origin(element):
    o = element.find('origin') if element is not None else None
    if o is None:
        return [0.0, 0.0, 0.0], [0.0, 0.0, 0.0]
    return floats(o.get('xyz', '0 0 0')), floats(o.get('rpy', '0 0 0'))


class Urdf:
    def __init__(self, path):
        root = ET.parse(path).getroot()
        self.links = {link.get('name'): link for link in root.findall('link')}
        self.joints = {j.get('name'): j for j in root.findall('joint') if j.get('type')}

    def rotor_mass(self, parent):
        total = 0.0
        for joint in self.joints.values():
            child = joint.find('child').get('link')
            if joint.find('parent').get('link') == parent and child.endswith('_rotor'):
                total += float(self.links[child].find('inertial/mass').get('value'))
        return total

    def physics(self, name, pad):
        inertial = self.links[name].find('inertial')
        mass = float(inertial.find('mass').get('value')) + self.rotor_mass(name)
        com, _ = origin(inertial)
        i = inertial.find('inertia')
        diag = vec(float(i.get(k)) for k in ('ixx', 'iyy', 'izz'))
        off = vec(float(i.get(k)) for k in ('ixy', 'ixz', 'iyz'))
        return (f'{pad}physics Physics {{\n{pad}  density -1\n{pad}  mass {mass:.6g}\n'
                f'{pad}  centerOfMass [ {vec(com)} ]\n'
                f'{pad}  inertiaMatrix [ {diag} {off} ]\n{pad}}}\n')

    def bounding(self, name, pad):
        collision = self.links[name].find('collision')
        xyz, rpy = origin(collision)
        geometry = collision.find('geometry')[0]
        if geometry.tag == 'box':
            shape = f'Box {{ size {geometry.get("size")} }}'
        elif geometry.tag == 'cylinder':
            # URDF cylinders point along Z, Webots R2025a cylinders too.
            shape = f'Cylinder {{ radius {geometry.get("radius")} height {geometry.get("length")} }}'
        else:
            shape = f'Sphere {{ radius {geometry.get("radius")} }}'
        return (f'{pad}boundingObject Pose {{\n{pad}  translation {vec(xyz)}\n'
                f'{pad}  rotation {axis_angle(rpy)}\n{pad}  children [ {shape} ]\n{pad}}}\n')

    def visual(self, name, pad):
        visual = self.links[name].find('visual')
        mesh = visual.find('geometry/mesh')
        xyz, rpy = origin(visual)
        url = MESHES + mesh.get('filename').rsplit('/', 1)[1]
        return (f'{pad}Pose {{\n{pad}  translation {vec(xyz)}\n{pad}  rotation {axis_angle(rpy)}\n'
                f'{pad}  children [ CadShape {{ url "{url}" }} ]\n{pad}}}\n')

    def leg(self, leg, pad):
        def joint(part, child_body, depth):
            p = pad + '  ' * depth
            j = self.joints[f'{leg}_{part}_joint']
            xyz, _ = origin(j)
            axis = j.find('axis').get('xyz')
            limit = j.find('limit')
            link = f'{leg}_{part}'
            position = SPAWN[part]
            return (
                f'{p}HingeJoint {{\n'
                f'{p}  jointParameters HingeJointParameters {{\n'
                f'{p}    position {position}\n{p}    axis {axis}\n{p}    anchor {vec(xyz)}\n{p}  }}\n'
                f'{p}  device [\n'
                f'{p}    RotationalMotor {{\n{p}      name "{leg}_{part}_joint"\n'
                f'{p}      maxVelocity {limit.get("velocity")}\n'
                f'{p}      minPosition {limit.get("lower")}\n{p}      maxPosition {limit.get("upper")}\n'
                f'{p}      maxTorque {limit.get("effort")}\n{p}    }}\n'
                f'{p}    PositionSensor {{ name "{leg}_{part}_joint_sensor" }}\n'
                f'{p}  ]\n'
                f'{p}  endPoint Solid {{\n'
                f'{p}    translation {vec(xyz)}\n{p}    rotation {axis} {position}\n'
                f'{p}    children [\n'
                f'{self.visual(link, p + "      ")}'
                f'{child_body}'
                f'{p}    ]\n'
                f'{p}    name "{link}"\n'
                f'{self.bounding(link, p + "    ")}'
                f'{self.physics(link, p + "    ")}'
                f'{p}  }}\n'
                f'{p}}}\n')

        p = pad + '  ' * 6
        foot_xyz, _ = origin(self.joints[f'{leg}_foot_fixed'])
        foot = (f'{p}Solid {{\n{p}  translation {vec(foot_xyz)}\n{p}  name "{leg}_foot"\n'
                f'{p}  contactMaterial "foot"\n'
                f'{self.bounding(f"{leg}_foot", p + "  ")}'
                f'{self.physics(f"{leg}_foot", p + "  ")}'
                f'{p}}}\n')
        return joint('hip', joint('thigh', joint('calf', foot, 6), 3), 0)


def main():
    urdf = Urdf(sys.argv[1])
    pad = '      '
    legs = ''.join(urdf.leg(leg, pad) for leg in LEGS)
    print(f'''#VRML_SIM R2025a utf8
# Unitree Go1 for Webots. Generated by tools/make_proto.py from the go1_description URDF: do not edit.
PROTO Go1 [
  field SFVec3f    translation    0 0 0.34
  field SFRotation rotation       0 0 1 0
  field SFString   name           "Go1"
  field SFString   controller     "go1_sim"
  field MFString   controllerArgs []
]
{{
  Robot {{
    translation IS translation
    rotation IS rotation
    name IS name
    controller IS controller
    controllerArgs IS controllerArgs
    selfCollision FALSE
    children [
      GPS {{ name "gps" }}
      InertialUnit {{ name "imu" }}
      Gyro {{ name "gyro" }}
      Accelerometer {{ name "accelerometer" }}
      Lidar {{
        translation {vec(LIDAR_XYZ)}
        name "lidar"
        horizontalResolution 921
        fieldOfView 6.2831853
        numberOfLayers 1
        minRange 0.1
        maxRange 40
        near 0.02
      }}
      Pose {{
        translation {LIDAR_XYZ[0]} {LIDAR_XYZ[1]} {LIDAR_XYZ[2] - 0.035}
        children [
          Shape {{
            appearance PBRAppearance {{ baseColor 0.05 0.05 0.06 roughness 0.4 }}
            geometry Cylinder {{ radius 0.035 height 0.04 }}
          }}
        ]
      }}
{urdf.visual("trunk", pad)}{legs}    ]
{urdf.bounding("trunk", "    ")}{urdf.physics("trunk", "    ")}  }}
}}''')


if __name__ == '__main__':
    main()
