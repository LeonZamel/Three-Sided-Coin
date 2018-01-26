import pybullet as p
import random
import time
import math

# Diameter of coin
# Width is fixed as 1
DIAMETER = 2 * 1.41

# Number of objects per test
OBJECTS = 100

# Number of tests
TESTS = 10

# Time steps for physics simulation in seconds
# Smaller value = more accurate simulation, but more computationally expensive
STEPSIZE = 1e-4

# Used to scale up centimeters to meters, while keeping accurate physics
# Bullet physics doesn't work well with objects at cm scale
SCALE = 100

SLOWDOWN = 0
FORCE = 50
DAMPING = 0.1
CUTOFF = 2

SHOW_DEBUG_GUI = False

total = 0
sysRand = random.SystemRandom()

p.connect(p.GUI)

# Set up simulation
p.setRealTimeSimulation(0)
p.setTimeStep(STEPSIZE)
p.resetDebugVisualizerCamera(80, 0, -50, [50, 25, 0])
p.configureDebugVisualizer(p.COV_ENABLE_GUI, SHOW_DEBUG_GUI)

for test in range(TESTS):
    # Turn off rendering for building world
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)

    # Remove all objects
    p.resetSimulation()
    p.setGravity(0, 0, -9.80665 * SCALE)

    # Create ground plane
    p.createCollisionShape(p.GEOM_PLANE)
    p.createMultiBody(0, 0)

    # Create collision shape for coin, which will be used for all bodies
    colCylinder = p.createCollisionShape(p.GEOM_CYLINDER, radius=DIAMETER / 2)

    # Create bodies with random froces applied
    for i in range(OBJECTS):
        x = p.createMultiBody(baseMass=1, baseCollisionShapeIndex=colCylinder,
                              basePosition=[i % 10 * 10, i / 10 * 10, 5])
        p.applyExternalForce(x, -1, [sysRand.randint(-FORCE/STEPSIZE, FORCE/STEPSIZE) * SCALE,
                                     sysRand.randint(-FORCE/STEPSIZE, FORCE/STEPSIZE) * SCALE,
                                     FORCE/STEPSIZE * SCALE],
                             [(sysRand.random() * 4 - 2) * SCALE,
                              (sysRand.random() * 4 - 2) * SCALE,
                              (sysRand.random() * 4 - 2) * SCALE],
                             p.LINK_FRAME)
        p.changeDynamics(x, -1, linearDamping=DAMPING, angularDamping=DAMPING)

    # Turn on rendering again
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)

    for i in range(int(1/STEPSIZE * CUTOFF)):
        p.stepSimulation()
        time.sleep(SLOWDOWN)
        
        """
        camData = p.getDebugVisualizerCamera()
        viewMat = camData[2]
        projMat = camData[3]
        p.getCameraImage(256, 256, viewMatrix=viewMat,
                                   projectionMatrix=projMat,
                                   renderer=p.ER_BULLET_HARDWARE_OPENGL)
        keys = p.getKeyboardEvents()
        """
        

    count = 0
    # First object is ground plane -> ignore
    for i in range(1, OBJECTS + 1):
        pos, angle = p.getBasePositionAndOrientation(i)
        angle = p.getEulerFromQuaternion(angle)
        # Check if cylinder is on its side
        if math.pi / 2 - 0.1 <= (angle[0] + math.pi) % math.pi <= math.pi / 2 + 0.1:
            count += 1

    total += count
    p.resetSimulation()
    print("Test " + str(test + 1) + ": "+ str(count))

print("Total: " + str(total))