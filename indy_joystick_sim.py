import pybullet as p
import time
import math
import numpy as np
from datetime import datetime
from multiprocessing import Queue, Process
from queue import Empty as QueueEmpty

def indy_joystick_sim(name, queue):
    print('Son process %s' % name)
    clid = p.connect(p.SHARED_MEMORY)
    if (clid < 0):
        p.connect(p.GUI)
    #p.connect(p.SHARED_MEMORY_GUI)

    planeId = p.loadURDF("/home/frank/code/pybullet_smc/bullet3/data/plane.urdf", [0, 0, 0])
    randx = np.random.rand(1)*20-10
    randy = np.random.rand(1)*20-10
    randd = (np.random.rand(1)*180-90)/1000
    rando = np.random.rand(1)*15-7.5
    objId = p.loadURDF('/home/frank/code/pybullet_smc/bullet3/data/banana/object_1.urdf', basePosition=[-0.5, 0.0, 0],baseOrientation=p.getQuaternionFromEuler([(33.9+randx )/180*math.pi,(25.5+randy)/180*math.pi,0]))
    indyId= p.loadURDF("indy7.urdf", [0, 0, 0])
    p.resetBasePositionAndOrientation(indyId, [0, 0, 0.03], [0, 0, 0, 1])
    #cid = p.createConstraint(indyId, -1, -1, -1, p.JOINT_FIXED, [0, 0, 0], [0, 0, 0], [0, 0, 0],[0,0,0])
    indyEndEffectorIndex = 7
    numJoints = p.getNumJoints(indyId)
    print("numJoints",numJoints)

    #numJoints =8
    #lower limits for null space
    ll = [-.967, -2, -2.96, 0.19, -2.96, -2.09, -3.05]
    #upper limits for null space
    ul = [.967, 2, 2.96, 2.29, 2.96, 2.09, 3.05]
    #joint ranges for null space
    jr = [6.10865, 6.10865, 6.10865, 6.10865, 6.10865, 6.10865, 6.10865]
    #restposes for null space
    rp = [0,0,0,0,0,0,0]
    #joint damping coefficents
    jd = [0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5]
    for i in range(7):
        p.resetJointState(indyId, i, rp[i])

    p.setGravity(0, 0, -10)
    t = 0.
    prevPose = [0, 0, 0]
    prevPose1 = [0, 0, 0]
    hasPrevPose = 0
    useNullSpace = 1

    useOrientation = 1
    #If we set useSimulation=0, it sets the arm pose to be the IK result directly without using dynamic control.
    #This can be used to test the IK result accuracy.
    useSimulation = 0
    useRealTimeSimulation = 1
    ikSolver = 0
    p.setRealTimeSimulation(useRealTimeSimulation)
    #trailDuration is duration (in seconds) after debug lines will be removed automatically
    #use 0 for no-removal
    trailDuration = 1

    i=0

    fov = 60
    aspect = 320/180
    near = 0.02
    far = 5

    angle = 0.0;
    while 1:
        i+=1

        #p.getCameraImage(320,
        #                 200,
        #                 flags=p.ER_SEGMENTATION_MASK_OBJECT_AND_LINKINDEX,
        #                 renderer=p.ER_BULLET_HARDWARE_OPENGL)
        if (useRealTimeSimulation):
            dt = datetime.now()
            t = (dt.second / 60.) * 2. * math.pi
        else:
            t = t + 0.01
        if (useSimulation and useRealTimeSimulation == 0):
            p.stepSimulation()
        movey = 0.5*math.sin(angle)
        angle = angle+0.1
        #obj_pos_orn = p.getBasePositionAndOrientation(objId)
        #p.resetBasePositionAndOrientation(objId,(obj_pos_orn[0][0], movey,obj_pos_orn[0][2]),obj_pos_orn[1])
        try:
            value = queue.get(True, 1)
            #print(value)
        except QueueEmpty:
            pass
        for i in range(1):

            #pos = [-0.5, 0.2 * math.cos(t), 0.3 + 0.2 * math.sin(t)]
            if value:
                pos = [-0.5,0, 0.5]
                x = 0.5 * value['joystick0']['axis0']
                y = 0.5 * value['joystick0']['axis1']
                z = 0.5 - 0.5 * value['joystick0']['axis2']
                pos = [x, y, z]
                thela1 = 180 * value['joystick0']['axis3']
                thela2 = 180 * value['joystick0']['axis4']
                thela3 = 180 * value['joystick0']['axis5']

                orn = p.getQuaternionFromEuler([(33.9+thela1 )/180*math.pi,(25.5+thela2)/180*math.pi,(25.5+thela3)/180*math.pi])
                #print(orn)
                finger_target = 0.04 - 0.03 * value['joystick0']['button5']
                print(finger_target)
            else:
                pos = [-0.5,0, 0.5]
                orn = p.getQuaternionFromEuler([0, -math.pi, 0])
                finger_target = 0.04
            #end effector points down, not up (in case useOrientation==1)
            

            if (useNullSpace == 1):
                if (useOrientation == 1):
                    
                    jointPoses = p.calculateInverseKinematics(indyId, indyEndEffectorIndex, pos, orn,
                                                            ll,
                                                            ul,
                                                            jr,
                                                            rp)
                # print("jointPoses : ",jointPoses)
                else:
                    jointPoses = p.calculateInverseKinematics(indyId,
                                                            indyEndEffectorIndex,
                                                            pos,
                                                            lowerLimits=ll,
                                                            upperLimits=ul,
                                                            jointRanges=jr,
                                                            restPoses=rp)
            else:
                if (useOrientation == 1):
                    #print(orn)
                    jointPoses = p.calculateInverseKinematics(indyId,
                                                            indyEndEffectorIndex,
                                                            pos,
                                                            orn,
                                                            jointDamping=jd,
                                                            solver=ikSolver,
                                                            maxNumIterations=100,
                                                            residualThreshold=.01)
                else:
                    jointPoses = p.calculateInverseKinematics(indyId,
                                                            5,
                                                            pos,
                                                            solver=ikSolver)

            if (useSimulation):
                i = 0
                for i in range(7):
                    p.setJointMotorControl2(bodyIndex=indyId,
                                            jointIndex=i+1,
                                            controlMode=p.POSITION_CONTROL,
                                            targetPosition=jointPoses[i],
                                            targetVelocity=0,
                                            force=500,
                                            positionGain=0.15,
                                            velocityGain=1.5)
            else:
            #reset the joint state (ignoring all dynamics, not recommended to use during simulation)
                for i in range(7):
                    p.resetJointState(indyId, i+1, jointPoses[i])


        for i in [9,10]:
            p.setJointMotorControl2(indyId, i, p.POSITION_CONTROL,finger_target ,force= 10)
        ls = p.getLinkState(indyId, indyEndEffectorIndex)
        
        end_pos = ls[4]
        end_orn = p.getEulerFromQuaternion(ls[5])
        R = np.zeros((3,3),dtype = float)
        R[0,0] = 1
        R[1,1] = 1
        R[2,2] = 1

        tr = np.zeros((3),dtype = float)
        tr[0]= 0
        tr[1]= 0
        tr[2]= -0.067
        T = np.zeros((4,4),dtype=float)
        T[0:3,0:3] = R
        T[0,3] = tr[0]
        T[1,3] = tr[1]
        T[2,3] = tr[2]
        T[3,3] = 1
        
        pos_temp=np.zeros(4,dtype=float)
        pos_temp[0] = end_pos[0]
        pos_temp[1] = end_pos[1]
        pos_temp[2] = end_pos[2]
        pos_temp[3] = 1
        
        end_pos = np.matmul(T,pos_temp)
        tcp_pos = end_pos.copy()
        #R = np.zeros((3,3),dtype = float)

        #R = np.array([[0.0015,-1,-0.0161],[1,0.0021,-0.0418],[0.0418,-0.0160,1]],dtype=float)
        #print(R)
        tr = np.zeros((3),dtype = float)
        tr[0]= -0.0579
        tr[1]= 0.0346
        tr[2]= -0.1058
        T = np.zeros((4,4),dtype=float)
        T[0:3,0:3] = R
        T[0,3] = tr[0]
        T[1,3] = tr[1]
        T[2,3] = tr[2]
        T[3,3] = 1

        pos_temp=np.zeros(4,dtype=float)
        pos_temp[0] = end_pos[0]
        pos_temp[1] = end_pos[1]
        pos_temp[2] = end_pos[2]
        pos_temp[3] = 1
        
        cam_pos = np.matmul(T,pos_temp)
        view_matrix = p.computeViewMatrix(cam_pos[0:3], [cam_pos[0], cam_pos[1], 0], [0,1,0])
        projection_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)
        images = p.getCameraImage(320,
                                180,
                                view_matrix,
                                projection_matrix,
                                shadow=True,
                                renderer=p.ER_BULLET_HARDWARE_OPENGL)
        #print(T)
        if (hasPrevPose):
            p.addUserDebugLine(prevPose, pos, [0, 0, 0.3], 1, 20)
            p.addUserDebugLine(prevPose1, ls[4], [1, 0, 0], 1, trailDuration)
            #p.addUserDebugLine(ls[4], tcp_pos[0:3], [0, 1, 0], 1, trailDuration)
            #p.addUserDebugLine(tcp_pos[0:3], cam_pos[0:3], [0, 0, 1], 1, trailDuration)
            p.addUserDebugLine(cam_pos[0:3], (cam_pos[0],cam_pos[1],-5), [1, 0, 0], 1, trailDuration)
        prevPose = pos
        prevPose1 = ls[4]
        #hasPrevPose = 1
    p.disconnect()
