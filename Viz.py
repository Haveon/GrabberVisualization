from LineSampler import LabViewFile, LaptopFile
from matplotlib import pyplot as plt
from scipy.interpolate import UnivariateSpline
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from rigidTrans import rotMatFromQuat, extVec, homogenousTransformationMatrix

class trialData:
    def __init__(self, laptopFname, labViewFname):
        topFile = LaptopFile(laptopFname)
        labFile = LabViewFile(labViewFname)
        
        self.labData = labFile.data
        self.topData = topFile.data

        self.batchNumber     = topFile.batchNumber
        self.objectNumber    = topFile.objectNumber
        self.trialNumber     = topFile.trialNumber
        self.clockDifference = topFile.clockDifference
        self.servoStartPos   = topFile.servoStartPos
        self.endGoal         = topFile.endGoal
        self.endServoPos     = topFile.endServoPos

        self.tool1   = labFile.tool1
        self.tool2   = labFile.tool2

        self.trackerToBowl = np.loadtxt('248Calibration.txt', delimiter=',')
        self.palmToTool1   = np.loadtxt('449Calibration.txt', delimiter=',')
        self.palmToTool2   = np.loadtxt('339Calibration.txt', delimiter=',')

        self.vec1 = [_.vec1 for _ in self.labData]
        self.qua1 = [_.qua1 for _ in self.labData]
        self.vec2 = [_.vec2 for _ in self.labData]
        self.qua2 = [_.qua2 for _ in self.labData]
        
        self.servoRead = [_.servoRead for _ in self.topData]

        self._collateData()
        self.dist = [(sum(map(pow, i, [2, 2, 2]))**0.5)*1e-3 for i in self.center]

    def _collateData(self):
        start = self.topData[0].startTime
        end   = self.topData[-1].startTime
        diff  = self.clockDifference

        self.ts = [(_.startTime - start).total_seconds() for _ in self.topData]        
        labTimeStamps = [(line.timeStamp - diff - start).total_seconds() for line in self.labData]
        finalData = []
        for i in range(len(self.vec1)):
            pos1Check = self.vec1[i][0] != 0.0 and self.vec1[i][1] != 0.0 and self.vec1[i][2] != 0.0
            qua1Check = self.qua1[i][0] != 0.0 and self.qua1[i][1] != 0.0 and self.qua1[i][2] != 0.0 and self.qua1[i][3]!=0.0
            pos2Check = self.vec2[i][0] != 0.0 and self.vec2[i][1] != 0.0 and self.vec2[i][2] != 0.0
            qua2Check = self.qua2[i][0] != 0.0 and self.qua2[i][1] != 0.0 and self.qua2[i][2] != 0.0 and self.qua2[i][3]!=0.0
            
            pack = []
            if pos1Check and qua1Check: #true if valid data sample
                R = rotMatFromQuat(self.qua1[i])
                H = homogenousTransformationMatrix(R, self.vec1[i])
                h = self.trackerToBowl.dot(H.dot(self.palmToTool1))
                pack.append(h)
            
            if pos2Check and qua2Check: #true if valid data sample
                R = rotMatFromQuat(self.qua2[i])
                H = homogenousTransformationMatrix(R, self.vec2[i])
                h = self.trackerToBowl.dot(H.dot(self.palmToTool2))
                pack.append(h)

            if pack: #false if pack empty
                h = np.mean(pack,axis=0)
                tmp = list(h.flatten())[:-4]
                bundle = [labTimeStamps[i]]+tmp
                finalData.append(bundle)

        keys = ['r0','r1','r2', 'x', 'r3', 'r4', 'r5', 'y', 'r6', 'r7', 'r8', 'z']
        data = zip(*finalData)
        validLabTimes = data[0]
        s = {}
        for k,d in zip(keys,data[1:]):
            s[k] = UnivariateSpline(validLabTimes, d)

        self.center       = np.array(zip( s['x'](self.ts),  s['y'](self.ts),  s['z'](self.ts)))
        self.orientationX = np.array(zip( s['r0'](self.ts), s['r3'](self.ts), s['r6'](self.ts)))
        self.orientationY = np.array(zip( s['r1'](self.ts), s['r4'](self.ts), s['r7'](self.ts)))
        self.orientationZ = np.array(zip( s['r2'](self.ts), s['r5'](self.ts), s['r8'](self.ts)))
        return None

    ### This should not be part of the class
    def plotData(self, fname):
        for i in range(len(self.ts)):
            fig = plt.figure(figsize=(16,9), dpi=100)
            ax = fig.add_subplot(121)
            index = np.arange(4)
            bar_width = 0.35

            rects = ax.bar(index, self.servoRead[i], bar_width)
            plt.xticks(index+0.5*bar_width, ('M1', 'M2', 'M3', 'M4'))
            plt.ylim([14000,18000])
            plt.suptitle(str(self.ts[i]))

            ax = fig.add_subplot(122, projection='3d')
            vec1 = self.center[i]
            x1,y1,z1 = self.orientationX[i],self.orientationY[i],self.orientationZ[i]
            x1 = 30*x1 +vec1
            y1 = 30*y1 +vec1
            z1 = 30*z1 +vec1

            ax.plot([x1[0],vec1[0]],[x1[1],vec1[1]],[x1[2],vec1[2]], 'r')
            ax.plot([y1[0],vec1[0]],[y1[1],vec1[1]],[y1[2],vec1[2]], 'b')
            ax.plot([z1[0],vec1[0]],[z1[1],vec1[1]],[z1[2],vec1[2]], 'g')

            ax.set_xlim(-150,150)
            ax.set_ylim(-150,150)
            ax.set_zlim(0,200)

            ax.plot([0,100],[0,0],[0,0],'k')
            ax.plot([0,0],[0,100],[0,0],'k')
            ax.plot([0,0],[0,0],[0,100],'k')
            
            plt.savefig(fname+str(i)+'.png')
            plt.close()

if __name__ == '__main__':
    tFile = 'Data/15333183/15333183-0-1013'
    dFile = 'Data/15333183/15333183-0-1013-Polaris.txt'
    d = trialData(tFile, dFile)
    #d.plotData('')