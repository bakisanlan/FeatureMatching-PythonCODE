import numpy as np
import matplotlib.pyplot as plt
from StateEstimatorMPF import StateEstimatorMPF
from AerialImageModel import AerialImageModel
from DataBaseScanner import DatabaseScanner
from UAVCamera import UAVCamera
from INSBot import INSbot
from Timer import Timer
from plotter import plot_positions,PlotCamera
from utils import *

#Sim settings
dt      = 0.1
simTime = 0
Tf      = 40
num_level = 1
ReferenceFrame = 'NED'
MAP            = 'itu'  #unity_fl1
detector       = 'SP'
snapFrame = 1
IMUtype = 2

#UAV Initilize
V = 20
pos0 = np.array([-600/2, -600/2, -2500])
eul0 = np.array([np.radians(45),0,0])
# V = 250
# pos0 = np.array([0, 0, -2500])
# eul0 = np.array([np.radians(0),0,0])
quat0 = eul2quat(eul0)
V0 = np.array([V,0,0])
acc_bias = np.zeros(3)
gyro_bias = np.zeros(3)
init_state = np.concatenate((pos0,V0,quat0,acc_bias,gyro_bias),axis=0)
u = [V, np.deg2rad(0)]
hAircraft = INSbot(init_state, dt=0.1, ReferenceFrame=ReferenceFrame, IMUtype=IMUtype)

# Aerial Image DataBase and Camera
hAIM = AerialImageModel(MAP,num_level=num_level, detector= detector)
hUAVCamera = UAVCamera(dt = dt, snapFrame = snapFrame)

# Database Scanner
hDB = DatabaseScanner(AIM= hAIM,num_level = num_level, snapFrame= snapFrame)

# MPF State Esimator
N = 100
mu_part  = np.array([0,0,0])
std_part = np.array([0, 0, 0 ])
mu_kalman  = np.zeros(12)
cov_kalman = np.zeros((12,12))
R = 9
StateEstimatorMPF = StateEstimatorMPF(N,mu_part,std_part,mu_kalman,cov_kalman,dt,9)
StateEstimatorMPF.DataBaseScanner =  hDB
StateEstimatorMPF.Accelerometer   = hAircraft.IMU.Accelerometer
StateEstimatorMPF.Gyroscope       = hAircraft.IMU.Gyroscope

# Create tracker list
traceState           = []
traceINSnoFuseState  = []
traceEstimatedState  = []
traceParticlesState  = []

# Figure create object holder for side by side view of UAV and particles 
CamPlotter = PlotCamera(snapFrame= snapFrame)

# Show the figure
plt.show(block = False)
while simTime < Tf:
   with Timer('Simulation Elapsed Time'):
        
        #UAV Move
        hAircraft.move(u)
        if (hAircraft.TrueState[0] > 8900) | (hAircraft.TrueState[0] < -8900) | (hAircraft.TrueState[1] > 8900) | (hAircraft.TrueState[1] < -8900):
            print('UAV went outside of map border')
            break
        inputParticle = hAircraft.IMUreading
        #UAV snap image
        # UAVFeaturesFrame, UAVKp, UAVDesc = hUAVCamera.snapUAVImage(StateEstimatorMPF.DataBaseScanner)
        with Timer('UAV Cam'):
            UAVFeaturesFrame, UAVKp, UAVDesc = hUAVCamera.snapUAVImageDataBase(StateEstimatorMPF.DataBaseScanner,hAircraft.TrueState[0:2],quat2eul(hAircraft.TrueState[6:10])[0])
        
        # MPF Estimation
        with Timer('MPF'):
            param = StateEstimatorMPF.getEstimate(inputParticle,hAircraft.NomState,UAVKp, UAVDesc)
        estState = hAircraft.correctINS(param["State"])
        particlesPos = StateEstimatorMPF.particles + hAircraft.NomState[0:3].reshape(-1, 1) # shape 3,N
        FramemostLikelihoodPart = StateEstimatorMPF.FramemostLikelihoodPart.copy()

        # Storing values
        traceState.append(hAircraft.TrueState[0:3].copy())
        traceINSnoFuseState.append(hAircraft.NomState[0:3]. copy())
        traceEstimatedState.append(estState[0:3].copy())
        traceParticlesState.append(particlesPos.T.copy())

        # Time update
        simTime += dt
        
        # #Update feature plot
        CamPlotter.snapNow(UAVFeaturesFrame,FramemostLikelihoodPart)
                        
plot_positions(traceState,traceINSnoFuseState,traceEstimatedState,traceParticlesState,plot_2d= True)

