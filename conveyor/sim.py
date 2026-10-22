import os
import tempfile

import pybullet as p
import pybullet_data

DT = 1 / 240
MAX_TORQUE = 4.0


def roller_urdf(n, radius, spacing, width, mass):
    iyy = 0.5 * mass * radius ** 2
    ixx = mass * (3 * radius ** 2 + width ** 2) / 12
    parts = ['<robot name="conveyor"><link name="base"><inertial><mass value="0"/><inertia ixx="0" iyy="0" izz="0" ixy="0" ixz="0" iyz="0"/></inertial></link>']
    for i in range(n):
        parts.append(f"""
<link name="roller{i}">
  <inertial><mass value="{mass}"/><inertia ixx="{ixx}" iyy="{iyy}" izz="{ixx}" ixy="0" ixz="0" iyz="0"/></inertial>
  <visual><origin rpy="1.5708 0 0"/><geometry><cylinder radius="{radius}" length="{width}"/></geometry></visual>
  <collision><origin rpy="1.5708 0 0"/><geometry><cylinder radius="{radius}" length="{width}"/></geometry></collision>
</link>
<joint name="motor{i}" type="continuous">
  <parent link="base"/><child link="roller{i}"/>
  <origin xyz="{i * spacing} 0 {radius}"/>
  <axis xyz="0 1 0"/>
</joint>""")
    parts.append("</robot>")
    return "".join(parts)


class Conveyor:
    def __init__(self, gui=False, rollers=10, radius=0.05, spacing=0.12, width=0.4,
                 roller_mass=2.0, damping=0.02, boxes=3, box_mass=1.0, box_size=0.2):
        self.radius = radius
        self.damping = damping
        self.length = (rollers - 1) * spacing
        self.box_half = box_size / 2
        self.client = p.connect(p.GUI if gui else p.DIRECT)
        p.setAdditionalSearchPath(pybullet_data.getDataPath())
        p.setGravity(0, 0, -9.81)
        p.setTimeStep(DT)
        p.loadURDF("plane.urdf", [0, 0, -0.5])
        path = os.path.join(tempfile.gettempdir(), "conveyor.urdf")
        with open(path, "w") as f:
            f.write(roller_urdf(rollers, radius, spacing, width, roller_mass))
        self.body = p.loadURDF(path, useFixedBase=True)
        self.joints = list(range(rollers))
        p.setJointMotorControlArray(self.body, self.joints, p.VELOCITY_CONTROL, forces=[0] * rollers)
        for j in self.joints:
            p.changeDynamics(self.body, j, lateralFriction=1.0, linearDamping=0, angularDamping=0)
        col = p.createCollisionShape(p.GEOM_BOX, halfExtents=[self.box_half] * 3)
        vis = p.createVisualShape(p.GEOM_BOX, halfExtents=[self.box_half] * 3, rgbaColor=[0.85, 0.55, 0.2, 1])
        self.boxes = []
        for i in range(boxes):
            x = self.length * (i + 0.5) / boxes
            b = p.createMultiBody(box_mass, col, vis, [x, 0, 2 * radius + self.box_half + 0.01])
            p.changeDynamics(b, -1, lateralFriction=1.0)
            self.boxes.append(b)
        if gui:
            p.resetDebugVisualizerCamera(1.8, 50, -30, [self.length / 2, 0, 0.1])
        self.time = 0.0

    def speed(self):
        states = p.getJointStates(self.body, self.joints)
        return sum(s[1] for s in states) / len(states) * self.radius

    def step(self, torque):
        states = p.getJointStates(self.body, self.joints)
        share = torque / len(self.joints)
        forces = [share - self.damping * s[1] for s in states]
        p.setJointMotorControlArray(self.body, self.joints, p.TORQUE_CONTROL, forces=forces)
        p.stepSimulation()
        self.time += DT
        self.recycle()

    def recycle(self):
        for b in self.boxes:
            (x, y, z), _ = p.getBasePositionAndOrientation(b)
            if x > self.length + self.box_half or z < 0:
                p.resetBasePositionAndOrientation(b, [0, 0, 2 * self.radius + self.box_half + 0.01], [0, 0, 0, 1])
                p.resetBaseVelocity(b, [0, 0, 0], [0, 0, 0])

    def boxes_on_belt(self):
        count = 0
        for b in self.boxes:
            if p.getContactPoints(bodyA=self.body, bodyB=b):
                count += 1
        return count

    def close(self):
        p.disconnect(self.client)
