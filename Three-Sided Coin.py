import time
import datetime
import math
import random
import itertools

import numpy as np

import pybullet
import pandas as pd

# Number of tests after which to stop
# or enter very large number and end simulation via pressing 'u'
# Will test every combination of test values TESTS times
TESTS = 1000

testValues = {
    "Diameter": [17.3, 20, 7/3 * 10, math.e * 10, 28.2], # IN MM
    "Mass": [5, 10, 15, 20], # IN GRAMS
    "Restitution": [0.55, 0.6, 0.65, 0.70],
    "Force": [1],
    "Force application Time": [0.05],
    "Lateral friction": [0.5],
    "Spinning friction": [0.0005],
    "Rolling friction": [0.002],
    "Distance between coins": [10],
    "Linear damping": [0.05],
    "Angular damping": [0.05]
}

# Width is fixed as 1 cm
# Diameter of coin IN MM
DIAMETER = 17.3

# Mass in gram
MASS = 5

# Force with which to throw/flip coins
FORCE = 1

# How long force is applied in seconds
FORCE_APPLICATION_TIME = 0.05

# Restitution i.e. "bouncyness", keep below 1
# higher = "bouncier"
# ratio of final to initial relative velocity between two objects after collision
RESTITUTION = 0.6

# Friction
LATERAL_FRICTION = 0.5
SPINNING_FRICTION = 0.0005
ROLLING_FRICTION = 0.002

# Distance between coins in cm, coins will touch with lower values,
# might lead to skewed results
# keep above diameter
DISTANCE_BETWEEN_COINS = 10

# Linear and angular damping
LINEAR_DAMPING = 0.05
ANGULAR_DAMPING = 0.05


# -- Variables below are valid for all tests -- #
# Number of objects per test
OBJECTS = 100

# Time steps for physics simulation in seconds
# Smaller value = more accurate simulation, but more computationally expensive
STEPSIZE = 1/120.0

# Substeps per Timestep
# Higher value = more accurate simulation, but more computationally expensive
SUBSTEPS = 2

# Used to scale up centimeters to decimeters, while keeping accurate physics
# Bullet physics doesn't work well with objects at cm scale
SCALE = 10

# Time after which to stop one run of simulation
# must be high enough for all coins to settle, 
# but low enough to not run (much) longer than needed
CUTOFF = 2

# Slow down simulation each step to see what is happening
# mainly for code refinement/debugging purposes
SLOWDOWN = 0.0

SHOW_DEBUG_GUI = False

testValues["Thickness"] = [10.0]
prodValues = np.array([x for x in itertools.product(*testValues.values())]).transpose()

df = pd.DataFrame(columns=["Heads","Tails","Side","Total"], index=pd.MultiIndex.from_arrays(prodValues, names=testValues.keys()))
df["Heads"] = 0
df["Tails"] = 0
df["Side"] = 0
df["Total"] = 0
df = df.reorder_levels([
    "Diameter",
    "Thickness",
    "Mass",
    "Restitution",
    "Force",
    "Force application Time",
    "Lateral friction",
    "Spinning friction",
    "Rolling friction",
    "Distance between coins",
    "Linear damping",
    "Angular damping"])
df.sort_index(level=[
    "Diameter",
    "Thickness",
    "Mass",
    "Restitution",
    "Force",
    "Force application Time",
    "Lateral friction",
    "Spinning friction",
    "Rolling friction",
    "Distance between coins",
    "Linear damping",
    "Angular damping"], inplace=True)


# Random distribution of SO(3)
# see http://planning.cs.uiuc.edu/node198.html
def generateRandomQuaternion():
    u1 = sysRand.random()
    u2 = sysRand.random()
    u3 = sysRand.random()
    w = math.sqrt(1 - u1) * math.sin(2 * math.pi * u2)
    x = math.sqrt(1 - u1) * math.cos(2 * math.pi * u2)
    y = math.sqrt(u1) * math.sin(2 * math.pi * u3)
    z = math.sqrt(u1) * math.cos(2 * math.pi * u3)
    return (x, y, z, w)

sysRand = random.SystemRandom()

pybullet.connect(pybullet.GUI)

# Set up simulation
pybullet.setRealTimeSimulation(0)
pybullet.setTimeStep(STEPSIZE)
pybullet.resetDebugVisualizerCamera(10, 0, -50, [5, 2, 0])
pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_GUI, SHOW_DEBUG_GUI)
pybullet.setPhysicsEngineParameter(numSubSteps=SUBSTEPS)

print("------------------------------------")
print("Simulation started, press 'u' to end")
print("------------------------------------")

for test in range(TESTS):
    print("Test #" + str(test + 1))
    stopSim = False
    for ind in df.index:
        dfPart = df.loc[[ind]]
        values = dict(zip(dfPart.index.names, dfPart.index.values[0]))
        DIAMETER = values["Diameter"]
        MASS = values["Mass"]
        RESTITUTION = values["Restitution"]
        FORCE = values["Force"]
        FORCE_APPLICATION_TIME = values["Force application Time"]
        LATERAL_FRICTION = values["Lateral friction"]
        SPINNING_FRICTION = values["Spinning friction"]
        ROLLING_FRICTION = values["Rolling friction"]
        DISTANCE_BETWEEN_COINS = values["Distance between coins"]
        LINEAR_DAMPING = values["Linear damping"]
        ANGULAR_DAMPING = values["Angular damping"]

        # Turn off rendering for building world
        pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_RENDERING, 0)

        # Remove all objects
        pybullet.resetSimulation()
        pybullet.setGravity(0, 0, -9.80665 * SCALE)

        # Create ground plane
        pybullet.createCollisionShape(pybullet.GEOM_PLANE)
        pybullet.createMultiBody(0, 0)
        pybullet.changeDynamics(0, -1, restitution=RESTITUTION, lateralFriction=LATERAL_FRICTION, spinningFriction=SPINNING_FRICTION, rollingFriction=ROLLING_FRICTION)

        # Create collision shape for coin, which will be used for all bodies
        colCylinder = pybullet.createCollisionShape(pybullet.GEOM_CYLINDER, radius=DIAMETER / 2 / 1000 * SCALE, height=0.01 * SCALE)

        # Create bodies with random forces applied
        for i in range(OBJECTS):
            x = pybullet.createMultiBody(baseMass=MASS / 1000 * SCALE, baseCollisionShapeIndex=colCylinder,
                                    basePosition=[i % 10 / 100 * DISTANCE_BETWEEN_COINS * SCALE, i / 10 / 100 * DISTANCE_BETWEEN_COINS * SCALE, 2], 
                                    baseOrientation=generateRandomQuaternion())

            pybullet.changeDynamics(x, -1, linearDamping=LINEAR_DAMPING, angularDamping=ANGULAR_DAMPING, restitution=RESTITUTION,
                                    lateralFriction=LATERAL_FRICTION, spinningFriction=SPINNING_FRICTION, rollingFriction=ROLLING_FRICTION)

            pybullet.applyExternalForce(x, -1, [(sysRand.random() * 2 * FORCE - FORCE) * SCALE / STEPSIZE * FORCE_APPLICATION_TIME,
                                                (sysRand.random() * 2 * FORCE - FORCE) * SCALE / STEPSIZE * FORCE_APPLICATION_TIME,
                                                (sysRand.random() * 2 * FORCE - FORCE) * SCALE / STEPSIZE * FORCE_APPLICATION_TIME],
                                    [(sysRand.random() * DIAMETER - DIAMETER / 2) / 100 * SCALE,
                                    (sysRand.random() * DIAMETER - DIAMETER / 2) / 100 * SCALE,
                                    (sysRand.random() * DIAMETER - DIAMETER / 2) / 100 * SCALE],
                                    pybullet.LINK_FRAME)
            

        # Turn on rendering again
        pybullet.configureDebugVisualizer(pybullet.COV_ENABLE_RENDERING, 1)

        for i in range(int(1/STEPSIZE * CUTOFF)):
            pybullet.stepSimulation()
            time.sleep(SLOWDOWN)
            keys = pybullet.getKeyboardEvents()
            for k in keys:
                if(k == 117):
                    if not stopSim:
                        stopSim = True
                        print("Will exit after this test finished")

        # First object is ground plane -> ignore
        for i in range(1, OBJECTS + 1):
            pos, angle = pybullet.getBasePositionAndOrientation(i)
            angle = pybullet.getEulerFromQuaternion(angle)
            df.loc[ind]["Total"] += 1
            # Check how coin landed:
            angle = (angle[0] + 2 * math.pi) % (2 * math.pi)

            # Side
            if math.pi / 2 - (math.pi / 2 - 1.3) <= angle <= math.pi / 2 + (math.pi / 2 - 1.3) or math.pi / 2 * 3 - (math.pi / 2 - 1.3) <= angle <= math.pi / 2 * 3 + (math.pi / 2 - 1.3):
                df.loc[ind]["Side"] += 1

            # Heads
            if angle <= 1.3 or 2 * math.pi - 1.3 <= angle:
                df.loc[ind]["Heads"] += 1

            # Tails
            if math.pi - 1.3 <= angle <= math.pi + 1.3:
                df.loc[ind]["Tails"] += 1

    if(stopSim):
        break


currenttime = datetime.datetime.now().replace(microsecond=0)
currenttime = currenttime.strftime("%Y-%m-%d %H-%M-%S" + ".csv")

print(df)
print("Writing data to file...")
df.to_csv("TSC " + currenttime)
