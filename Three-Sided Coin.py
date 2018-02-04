import time
import datetime
import math
import random

import pybullet
import pandas as pd


# Diameter of coin
# Width is fixed as 1 cm
DIAMETER = 2 * 1.41

# Mass in gram
MASS = 10

# Number of objects per test
OBJECTS = 100

# Number of tests
TESTS = 10
# Time steps for physics simulation in seconds
# Smaller value = more accurate simulation, but more computationally expensive
STEPSIZE = 1/120.0

# Substeps per Timestep
# Higher value = more accurate simulation, but more computationally expensive
SUBSTEPS = 2

# Used to scale up centimeters to decimeters, while keeping accurate physics
# Bullet physics doesn't work well with objects at cm scale
SCALE = 10

# How long force is applied in seconds
FORCEAPPLICATIONTIME = 0.05

# Force with which to throw/flip coins
FORCE = 1

# Restitution i.e. "bouncyness", keep below 1
# higher = "bouncier"
RESTITUTION = 0.7

# Distance between coins in cm
# keep above diameter
DISTANCE = 10

# Slow down simulation each step to see what is happening
SLOWDOWN = 0.0
DAMPING = 0.05
CUTOFF = 2

SHOW_DEBUG_GUI = False


data = {
    "Diameter": DIAMETER * 10, # Convert cm to mm for spreadsheet
    "Thickness": 1 * 10, # Convert cm to mm for spreadsheet
    "Heads": 0,
    "Tails": 0,
    "Side": 0,
    "Total counted": 0,
    "Total thrown": 0
}

sysRand = random.SystemRandom()

pybullet.connect(pybullet.GUI)

# Set up simulation
pybullet.setRealTimeSimulation(0)
pybullet.setTimeStep(STEPSIZE)
pybullet.resetDebugVisualizerCamera(10, 0, -50, [5, 2, 0])
pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_GUI, SHOW_DEBUG_GUI)
pybullet.setPhysicsEngineParameter(numSubSteps=SUBSTEPS)

for test in range(TESTS):
    # Turn off rendering for building world
    pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_RENDERING, 0)

    # Remove all objects
    pybullet.resetSimulation()
    pybullet.setGravity(0, 0, -9.80665 * SCALE)

    # Create ground plane
    pybullet.createCollisionShape(pybullet.GEOM_PLANE)
    pybullet.createMultiBody(0, 0)
    pybullet.changeDynamics(0, -1, restitution=RESTITUTION)

    # Create collision shape for coin, which will be used for all bodies
    colCylinder = pybullet.createCollisionShape(pybullet.GEOM_CYLINDER, radius=DIAMETER / 2 / 100 * SCALE, height=0.01 * SCALE)

    # Create bodies with random forces applied
    for i in range(OBJECTS):
        x = pybullet.createMultiBody(baseMass=MASS / 1000 * SCALE, baseCollisionShapeIndex=colCylinder,
                              basePosition=[i % 10 / 100 * DISTANCE * SCALE, i / 10 / 100 * DISTANCE * SCALE, 2], 
                              baseOrientation=pybullet.getQuaternionFromEuler([sysRand.random() * math.pi * 4, sysRand.random() * math.pi * 4, sysRand.random() * math.pi * 4]))

        pybullet.changeDynamics(x, -1, linearDamping=DAMPING, angularDamping=DAMPING, restitution=RESTITUTION)

        pybullet.applyExternalForce(x, -1, [(sysRand.random() * 2 * FORCE - FORCE) * SCALE / STEPSIZE * FORCEAPPLICATIONTIME,
                                     (sysRand.random() * 2 * FORCE - FORCE) * SCALE / STEPSIZE * FORCEAPPLICATIONTIME,
                                     (sysRand.random() * FORCE + FORCE) * SCALE / STEPSIZE * FORCEAPPLICATIONTIME],
                             [(sysRand.random() * DIAMETER - DIAMETER / 2) / 100 * SCALE,
                              (sysRand.random() * DIAMETER - DIAMETER / 2) / 100 * SCALE,
                              (sysRand.random() * DIAMETER - DIAMETER / 2) / 100 * SCALE],
                             pybullet.LINK_FRAME)
        

    # Turn on rendering again
    pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_RENDERING, 1)

    for i in range(int(1/STEPSIZE * CUTOFF)):
        pybullet.stepSimulation()
        time.sleep(SLOWDOWN)

        # Optional camera
        """
        camData = pybullet.getDebugVisualizerCamera()
        viewMat = camData[2]
        projMat = camData[3]
        pybullet.getCameraImage(256, 256, viewMatrix=viewMat,
                                   projectionMatrix=projMat,
                                   renderer=pybullet.ER_BULLET_HARDWARE_OPENGL)
        keys = pybullet.getKeyboardEvents()
        """

    # First object is ground plane -> ignore
    for i in range(1, OBJECTS + 1):
        pos, angle = pybullet.getBasePositionAndOrientation(i)
        angle = pybullet.getEulerFromQuaternion(angle)

        data["Total thrown"] += 1
        # Check how coin landed:
        angle = (angle[0] + 2 * math.pi) % (2 * math.pi)
        print(angle)
        # Side
        if math.pi / 2 - 0.3 <= angle <= math.pi / 2 + 0.3 or math.pi / 2 * 3 - 0.3 <= angle <= math.pi / 2 * 3 + 0.3:
            data["Side"] += 1
            data["Total counted"] += 1
        # Heads
        if angle <= 1.3 or 2 * math.pi - 1.3 <= angle:
            data["Heads"] += 1
            data["Total counted"] += 1
        # Tails
        if math.pi - 1.3 <= angle <= math.pi + 1.3:
            data["Tails"] += 1
            data["Total counted"] += 1
        

    pybullet.resetSimulation()

currenttime = datetime.datetime.now().replace(microsecond=0)
currenttime = currenttime.strftime("%Y-%m-%d %H-%M-%S" + ".csv")

df = pd.DataFrame(data, index=[0])
df.to_csv("TSC " + currenttime)

print(df)
