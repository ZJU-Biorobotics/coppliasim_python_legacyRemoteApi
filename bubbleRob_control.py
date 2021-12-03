import sim as vrep
import sys
import numpy as np
import time
import matplotlib.pyplot as mlp
vrep.simxFinish(-1)
clientID=vrep.simxStart('127.0.0.1',19999,True,True,5000,5)

if clientID!=-1:
    print("Connected to remote API server")
else:
    print("Not connected to remote API server")
    sys.exit("Could not connect")

err_code,l_motor_handle = vrep.simxGetObjectHandle(clientID,"bubbleRob_leftMotor",vrep.simx_opmode_blocking)

err_code,r_motor_handle = vrep.simxGetObjectHandle(clientID,"bubbleRob_rightMotor",vrep.simx_opmode_blocking)

err_code = vrep.simxSetJointTargetVelocity(clientID,l_motor_handle,1.0,vrep.simx_opmode_streaming)
err_code = vrep.simxSetJointTargetVelocity(clientID,r_motor_handle,1.0,vrep.simx_opmode_streaming)
err_code,ps_handle = vrep.simxGetObjectHandle(clientID,"bubbleRob_sensingNose", vrep.simx_opmode_blocking)
err_code,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector=vrep.simxReadProximitySensor(clientID,
ps_handle,vrep.simx_opmode_streaming)
print(err_code,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector)

err_code,camera = vrep.simxGetObjectHandle(clientID,"Vision_sensor",
vrep.simx_opmode_blocking)
err_code,resolution,image = vrep.simxGetVisionSensorImage(clientID,
    camera,0,vrep.simx_opmode_streaming)
#print(err_code,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector)
t = time.time()
sensor_val=0
while(time.time()-t<100):
    time.sleep(0.2)
    sensor_val = np.linalg.norm(detectedPoint)
    if sensor_val < 0.2 and sensor_val > 0.01:
        l_steer = -1/sensor_val
    else: 
        l_steer = 1.0
    err_code = vrep.simxSetJointTargetVelocity(clientID,l_motor_handle,l_steer,vrep.simx_opmode_streaming)
    err_code = vrep.simxSetJointTargetVelocity(clientID,r_motor_handle,1.0,vrep.simx_opmode_streaming)
    
    err_code,detectionState,detectedPoint,detectedObjectHandle,
    detectedSurfaceNormalVector=vrep.simxReadProximitySensor(clientID,
    ps_handle,vrep.simx_opmode_buffer)
    print(err_code,detectionState,detectedPoint,detectedObjectHandle,detectedSurfaceNormalVector)
    err_code,resolution,image = vrep.simxGetVisionSensorImage(clientID,
    camera,0,vrep.simx_opmode_buffer)
    
    print (sensor_val,detectedPoint)
    img = np.array(image, dtype = np.uint8)
    #print(img,resolution)
    img.resize([resolution[0],resolution[1],3])
    
    mlp.imshow(img,origin="lower")
    time.sleep(1)
    
vrep.simxStopSimulation(clientID,vrep.simx_opmode_oneshot)
print("Done")

