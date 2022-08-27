import pybullet as p
import time
import pybullet_data
import math
from math import pi
import numpy as np
from scipy.spatial.transform import Rotation
import random
from poseconfigs import *
joint_indice = [x[0] for x in specs]

bittleheight=0.1154

def get_gravity_vec(q):
    rot_ = Rotation.from_quat(q)
    mat_ = rot_.as_matrix()
    vec_ = np.dot(np.transpose(mat_), np.array([[0.], [0.], [-1.]]))
    return vec_
def reset_sim(friction = 0.7):
    p.resetSimulation()
    p.resetDebugVisualizerCamera(cameraDistance=0.05, cameraYaw=0, cameraPitch=-40,
                                 cameraTargetPosition=[-0.02, -0.09, 0.2])
    p.setGravity(0, 0, -9.81)
    p.setTimeStep(1. / 1000)
    # p.setPhysicsEngineParameter(fixedTimeStep=1. / 120.)


    planeId = p.loadURDF("plane.urdf",useFixedBase=True)
    p.changeDynamics(bodyUniqueId=planeId,linkIndex=-1,lateralFriction=friction,
                     rollingFriction=0.0001,spinningFriction=0.0001,linearDamping=0,angularDamping=0,
                     mass=0,localInertiaDiagonal=[0, 0, 0],restitution=1.0)
    return planeId


    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 0)
    random.seed(10)


    import pandas as pd
    height_file = r"../benchmark/hard/elevation0002.txt"
    heightfieldData = pd.read_csv(height_file,sep=' ',header=None).values[::-1,:].flatten('C')
    terrainShape = p.createCollisionShape(shapeType=p.GEOM_HEIGHTFIELD, meshScale=[.025, .025, .25],
                                          heightfieldTextureScaling=128,
                                          heightfieldData=heightfieldData, numHeightfieldRows=225,
                                          numHeightfieldColumns=225)
    terrain = p.createMultiBody(0, terrainShape)
    p.resetBasePositionAndOrientation(terrain, [0, 0, -.25], [0, 0, 0, 1])

    # terrain = p.createMultiBody(0, terrainShape)
    # p.resetBasePositionAndOrientation(terrain, [0, 0, -heightPerturbationRange], [0, 0, 0, 1])
    p.changeDynamics(bodyUniqueId=terrain,linkIndex=-1,lateralFriction=friction,
                     rollingFriction=0.0001,spinningFriction=0.0001,linearDamping=0,angularDamping=0,
                     mass=0,localInertiaDiagonal=[0, 0, 0],restitution=1.0)

    return terrain

def set_pos12(uid,arr): # positive = forward
    for i in range(12):
        set_pos(uid,i,arr[i])
        
def set_pos(uid,jid,pos):
    # p.setJointMotorControl2(uid,joint_indice[jid],force=33.5, maxVelocity = 20,
    #                             controlMode= p.POSITION_CONTROL,targetPosition = pos)
    motor_state = p.getJointState(uid,joint_indice[jid])
    p.setJointMotorControl2(uid, joint_indice[jid], force= 55 * (pos-motor_state[0]) + 0.8 * (0-motor_state[1]),
                            controlMode=p.TORQUE_CONTROL)
def get_state(uid):
    states = p.getJointStates(uid,joint_indice)
    pos = [s[0] for s in states]
    velo = [s[1] for s in states]
    react = [s[2] for s in states]
    torq = [s[3] for s in states]
    omega = p.getBaseVelocity(uid)[1]
    q = p.getBasePositionAndOrientation(uid)[1]
    grav = np.reshape(get_gravity_vec(q),(-1,))
    return pos,velo,list(omega),grav.tolist(),react,torq

physicsClient = p.connect(p.GUI)  # or p.DIRECT for non-graphical version
p.setAdditionalSearchPath(pybullet_data.getDataPath())  # optionally
bittle_path = r'/home/zc/terrain/fallrec/unitree_ros/robots/a1_description/urdf/a1.urdf'

while True:
    planeId = reset_sim()
    p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
    p.configureDebugVisualizer(p.COV_ENABLE_GUI,0)
    p.setGravity(0, 0, -9.81)
    # limits: [-0.8,0.8],[-1,4],[-2.5,-1]
    StartPos,rst_pos,q =  ([0,0,0.057], [0, 1.0, -1.8]*4, [pi,0,0])
    StartOrientation = p.getQuaternionFromEuler(q)
    # StartOrientation = p.getQuaternionFromEuler(rand_q)
    bittleid = p.loadURDF(bittle_path, StartPos, StartOrientation,useFixedBase=False,
                          flags=p.URDF_USE_SELF_COLLISION)

    print('index,lower,upper,force,velo')
    for j in range(p.getNumJoints(bittleid)):
        infos = p.getJointInfo(bittleid, j)
        if infos[2] == 0 or True:
            print(infos[0:2],',',infos[8],',',infos[9],',',infos[10],',',infos[11])
            print(j,':',p.getDynamicsInfo(bittleid,j)[0],',')
            # p.changeDynamics(bittleid,j,mass=p.getDynamicsInfo(bittleid,j)[0]*2)
            # print(p.getDynamicsInfo(bittleid, j)[0])


    for i in range(12):
        p.resetJointState(bittleid,joint_indice[i],rst_pos[i])
    for i in range(12):
        p.setJointMotorControl2(bittleid, joint_indice[i], p.VELOCITY_CONTROL, force=0)

    width, height, rgbImg,_,_ = p.getCameraImage(
        width=224,
        height=224,
    )
    # p.configureDebugVisualizer(p.COV_ENABLE_RENDERING, 1)
    while True:
        p.stepSimulation()
        # print(p.getBasePositionAndOrientation(bittleid)[0],
        #       p.getEulerFromQuaternion(p.getBasePositionAndOrientation(bittleid)[1]))
        set_pos12(bittleid,rst_pos)
        time.sleep(1./1000.)

p.disconnect()
