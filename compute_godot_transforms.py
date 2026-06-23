#!/usr/bin/env python3
"""
Parse SDF file and compute Godot Transform3D values for all links and joints.

Coordinate Systems:
- SDF: right-handed, Z-up. Axes: +X=right, +Y=forward(into screen), +Z=up
- Godot: left-handed, Y-up. Axes: +X=right, +Y=up, +Z=forward(out of screen)

T (SDF -> Godot): v_godot = T * v_sdf
T = [[1,  0,  0],
     [0,  0,  1],
     [0, -1,  0]]

So: (x, y, z)_sdf -> (x, z, -y)_godot
"""

import numpy as np
import xml.etree.ElementTree as ET

# Coordinate transform matrix: SDF -> Godot
T = np.array([
    [1,  0,  0],
    [0,  0,  1],
    [0, -1,  0]
], dtype=np.float64)

# T transpose
T_T = T.T


def rx(theta):
    """Rotation matrix about X axis (right-hand rule)."""
    c = np.cos(theta)
    s = np.sin(theta)
    return np.array([
        [1, 0,  0],
        [0, c, -s],
        [0, s,  c]
    ], dtype=np.float64)


def ry(theta):
    """Rotation matrix about Y axis (right-hand rule)."""
    c = np.cos(theta)
    s = np.sin(theta)
    return np.array([
        [ c, 0, s],
        [ 0, 1, 0],
        [-s, 0, c]
    ], dtype=np.float64)


def rz(theta):
    """Rotation matrix about Z axis (right-hand rule)."""
    c = np.cos(theta)
    s = np.sin(theta)
    return np.array([
        [c, -s, 0],
        [s,  c, 0],
        [0,  0, 1]
    ], dtype=np.float64)


def sdf_pose_to_rotation_matrix(roll, pitch, yaw):
    """
    SDF uses intrinsic rotations: first roll about X, then pitch about Y, then yaw about Z.
    For column vectors, this is equivalent to:
    R = Rz(yaw) * Ry(pitch) * Rx(roll)
    """
    return rz(yaw) @ ry(pitch) @ rx(roll)


def compute_link_godot_transform(x, y, z, roll, pitch, yaw):
    """
    Compute Godot Transform3D for a link from its SDF pose.

    Position: pos_godot = (x, z, -y)
    Rotation: R_godot = T * R_sdf * T^T

    Godot's Transform3D constructor takes basis vectors as columns:
    Transform3D(basis_col1, basis_col2, basis_col3, position)
    where basis_col1 = R_godot[:,0], basis_col2 = R_godot[:,1], basis_col3 = R_godot[:,2]
    """
    # Position transform: (x, y, z)_sdf -> (x, z, -y)_godot
    pos_godot = np.array([x, z, -y], dtype=np.float64)

    # Rotation in SDF frame
    R_sdf = sdf_pose_to_rotation_matrix(roll, pitch, yaw)

    # Rotation in Godot frame
    R_godot = T @ R_sdf @ T_T

    return pos_godot, R_godot


def format_transform3d(pos, R):
    """Format as Godot Transform3D(m00,m01,m02,m10,m11,m12,m20,m21,m22,x,y,z)."""
    # Godot Transform3D: basis vectors as columns, then position
    # m00=R[0,0], m01=R[0,1], m02=R[0,2]
    # m10=R[1,0], m11=R[1,1], m12=R[1,2]
    # m20=R[2,0], m21=R[2,1], m22=R[2,2]
    # x=pos[0], y=pos[1], z=pos[2]
    return (f"Transform3D({R[0,0]:.10f}, {R[0,1]:.10f}, {R[0,2]:.10f}, "
            f"{R[1,0]:.10f}, {R[1,1]:.10f}, {R[1,2]:.10f}, "
            f"{R[2,0]:.10f}, {R[2,1]:.10f}, {R[2,2]:.10f}, "
            f"{pos[0]:.10f}, {pos[1]:.10f}, {pos[2]:.10f})")


def axis_angle_to_rotation_matrix(axis, angle):
    """Convert axis-angle to rotation matrix (Rodrigues' formula)."""
    k = axis / np.linalg.norm(axis)
    K = np.array([
        [0,    -k[2],  k[1]],
        [k[2],  0,    -k[0]],
        [-k[1], k[0],  0   ]
    ], dtype=np.float64)
    R = np.eye(3) + np.sin(angle) * K + (1 - np.cos(angle)) * (K @ K)
    return R


def compute_joint_basis(axis_godot):
    """
    Compute joint rotation basis so that the joint's Z axis aligns with axis_godot.
    Default HingeJoint3D axis is Z (0, 0, 1).
    """
    axis = np.array(axis_godot, dtype=np.float64)
    norm = np.linalg.norm(axis)
    if norm < 1e-10:
        return np.eye(3)
    target_axis = axis / norm

    default_axis = np.array([0, 0, 1], dtype=np.float64)

    dot = np.dot(default_axis, target_axis)

    if dot > 0.999999:
        # Already aligned
        return np.eye(3)
    elif dot < -0.999999:
        # Anti-parallel: rotate 180 degrees around X
        return rx(np.pi)
    else:
        rotation_axis = np.cross(default_axis, target_axis)
        angle = np.arccos(np.clip(dot, -1.0, 1.0))
        return axis_angle_to_rotation_matrix(rotation_axis, angle)


def parse_sdf(sdf_path):
    """Parse SDF file and extract links and joints."""
    tree = ET.parse(sdf_path)
    root = tree.getroot()

    model = root.find('model')

    links = {}
    for link_elem in model.findall('link'):
        name = link_elem.get('name')
        pose_elem = link_elem.find('pose')
        pose_text = pose_elem.text.strip()
        pose_vals = [float(v) for v in pose_text.split()]
        links[name] = {
            'x': pose_vals[0],
            'y': pose_vals[1],
            'z': pose_vals[2],
            'roll': pose_vals[3],
            'pitch': pose_vals[4],
            'yaw': pose_vals[5]
        }

    joints = {}
    for joint_elem in model.findall('joint'):
        name = joint_elem.get('name')
        pose_elem = joint_elem.find('pose')
        pose_text = pose_elem.text.strip()
        pose_vals = [float(v) for v in pose_text.split()]

        parent = joint_elem.find('parent').text.strip()
        child = joint_elem.find('child').text.strip()

        axis_elem = joint_elem.find('axis')
        xyz_elem = axis_elem.find('xyz')
        axis_text = xyz_elem.text.strip()
        axis_vals = [float(v) for v in axis_text.split()]

        joints[name] = {
            'x': pose_vals[0],
            'y': pose_vals[1],
            'z': pose_vals[2],
            'roll': pose_vals[3],
            'pitch': pose_vals[4],
            'yaw': pose_vals[5],
            'parent': parent,
            'child': child,
            'axis_x': axis_vals[0],
            'axis_y': axis_vals[1],
            'axis_z': axis_vals[2]
        }

    return links, joints


def main():
    sdf_path = '/home/penguin/Desktop/origami-robot/sdf/origami_robot/origami_robot.sdf'
    links, joints = parse_sdf(sdf_path)

    print("=" * 80)
    print("LINK TRANSFORMS (SDF -> Godot)")
    print("=" * 80)
    print()

    link_transforms = {}

    for name in sorted(links.keys()):
        link = links[name]
        pos_godot, R_godot = compute_link_godot_transform(
            link['x'], link['y'], link['z'],
            link['roll'], link['pitch'], link['yaw']
        )
        link_transforms[name] = (pos_godot, R_godot)

        print(f"--- {name} ---")
        print(f"  SDF pose:    x={link['x']:.10f}, y={link['y']:.10f}, z={link['z']:.10f}")
        print(f"               roll={link['roll']:.10f}, pitch={link['pitch']:.10f}, yaw={link['yaw']:.10f}")
        print()
        print(f"  Godot position: ({pos_godot[0]:.10f}, {pos_godot[1]:.10f}, {pos_godot[2]:.10f})")
        print()
        print(f"  Godot basis (columns):")
        print(f"    col0 (X): ({R_godot[0,0]:.10f}, {R_godot[1,0]:.10f}, {R_godot[2,0]:.10f})")
        print(f"    col1 (Y): ({R_godot[0,1]:.10f}, {R_godot[1,1]:.10f}, {R_godot[2,1]:.10f})")
        print(f"    col2 (Z): ({R_godot[0,2]:.10f}, {R_godot[1,2]:.10f}, {R_godot[2,2]:.10f})")
        print()
        print(f"  Godot Transform3D:")
        print(f"    {format_transform3d(pos_godot, R_godot)}")
        print()

    print("=" * 80)
    print("JOINT TRANSFORMS (placed at world/root level)")
    print("=" * 80)
    print()

    for name in sorted(joints.keys()):
        joint = joints[name]
        parent_name = joint['parent']

        # Joint pose is relative to parent link frame in SDF
        j_local_pos_sdf = np.array([joint['x'], joint['y'], joint['z']], dtype=np.float64)

        # Get parent link transform
        parent_pos_godot, parent_R_godot = link_transforms[parent_name]

        # joint_world_pos_godot = parent_link_pos_godot + R_parent_godot * T * joint_local_pos_sdf
        # = parent_link_pos_godot + R_parent_godot * (jx, jz, -jy)
        transformed_local = T @ j_local_pos_sdf  # (jx, jz, -jy)
        joint_world_pos = parent_pos_godot + parent_R_godot @ transformed_local

        # Joint axis in model frame
        axis_sdf = np.array([joint['axis_x'], joint['axis_y'], joint['axis_z']], dtype=np.float64)
        axis_godot = T @ axis_sdf  # (ax, az, -ay)

        # Joint basis: rotate so Z aligns with axis_godot
        joint_basis = compute_joint_basis(axis_godot)

        print(f"--- {name} ---")
        print(f"  Parent: {parent_name}, Child: {joint['child']}")
        print(f"  SDF joint local pos: ({joint['x']:.10f}, {joint['y']:.10f}, {joint['z']:.10f})")
        print(f"  SDF axis: ({joint['axis_x']:.10f}, {joint['axis_y']:.10f}, {joint['axis_z']:.10f})")
        print()
        print(f"  Godot axis: ({axis_godot[0]:.10f}, {axis_godot[1]:.10f}, {axis_godot[2]:.10f})")
        print(f"  Godot position: ({joint_world_pos[0]:.10f}, {joint_world_pos[1]:.10f}, {joint_world_pos[2]:.10f})")
        print()
        print(f"  Godot basis (columns):")
        print(f"    col0 (X): ({joint_basis[0,0]:.10f}, {joint_basis[1,0]:.10f}, {joint_basis[2,0]:.10f})")
        print(f"    col1 (Y): ({joint_basis[0,1]:.10f}, {joint_basis[1,1]:.10f}, {joint_basis[2,1]:.10f})")
        print(f"    col2 (Z): ({joint_basis[0,2]:.10f}, {joint_basis[1,2]:.10f}, {joint_basis[2,2]:.10f})")
        print()
        print(f"  Godot Transform3D:")
        print(f"    {format_transform3d(joint_world_pos, joint_basis)}")
        print()


if __name__ == '__main__':
    main()
