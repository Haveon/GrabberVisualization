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

        self._synchronize()
        self._referenceTransformation()

        self.dist = [(sum(map(pow, i, [2, 2, 2]))**0.5)*1e-3 for i in self.center1]

    def _synchronize(self):
        start = self.topData[0].startTime
        end   = self.topData[-1].startTime
        diff  = self.clockDifference

        self.ts = [(_.startTime - start).total_seconds() for _ in self.topData]        
        labTimeStamps = [(line.timeStamp - diff - start).total_seconds() for line in self.labData]

        s = self._intepolateLabData(labTimeStamps) # Returns a dictionary of splines

        self.vec1 = zip( s['x1'](self.ts),  s['y1'](self.ts),  s['z1'](self.ts))
        self.qua1 = zip( s['q10'](self.ts), s['q11'](self.ts), s['q12'](self.ts), s['q13'](self.ts))
        self.vec2 = zip( s['x2'](self.ts),  s['y2'](self.ts),  s['z2'](self.ts))
        self.qua2 = zip( s['q20'](self.ts), s['q21'](self.ts), s['q22'](self.ts), s['q23'](self.ts))
        return

    def _intepolateLabData(self, labTS):
        indeces1 = []
        indeces2 = []
        for i in range(len(self.vec1)):
            if self.vec1[i][0] != 0.0 and self.vec1[i][1] != 0.0 and self.vec1[i][2] != 0.0:
                indeces1.append(i)
            if self.vec2[i][0] != 0.0 and self.vec2[i][1] != 0.0 and self.vec2[i][2] != 0.0:
                indeces2.append(i)

        s = {}

        ts1, x1, y1, z1, q10, q11, q12, q13 = [], [], [], [], [], [], [], []
        for i in indeces1:
            ts1.append(labTS[i])
            x1.append(self.vec1[i][0])
            y1.append(self.vec1[i][1])
            z1.append(self.vec1[i][2])
            q10.append(self.qua1[i][0])
            q11.append(self.qua1[i][1])
            q12.append(self.qua1[i][2])
            q13.append(self.qua1[i][3])

        s['x1'], s['y1'], s['z1'] = UnivariateSpline(ts1, x1), UnivariateSpline(ts1, y1), UnivariateSpline(ts1, z1)
        s['q10'], s['q11'], s['q12'], s['q13'] = UnivariateSpline(ts1, q10), UnivariateSpline(ts1, q11), UnivariateSpline(ts1, q12), UnivariateSpline(ts1, q13)

        ts2, x2, y2, z2, q20, q21, q22, q23 = [], [], [], [], [], [], [], []
        for i in indeces2:
            ts2.append(labTS[i])
            x2.append(self.vec2[i][0])
            y2.append(self.vec2[i][1])
            z2.append(self.vec2[i][2])
            q20.append(self.qua2[i][0])
            q21.append(self.qua2[i][1])
            q22.append(self.qua2[i][2])
            q23.append(self.qua2[i][3])
        
        s['x2'], s['y2'], s['z2'] = UnivariateSpline(ts2, x2), UnivariateSpline(ts2, y2), UnivariateSpline(ts2, z2)
        s['q20'], s['q21'], s['q22'], s['q23'] = UnivariateSpline(ts2, q20), UnivariateSpline(ts2, q21), UnivariateSpline(ts2, q22), UnivariateSpline(ts2, q23)

        return s

    def _referenceTransformation(self):
        self.center1 = []
        self.orientation1 = []
        self.center2 = []
        self.orientation2 = []
        for v1,v2,q1,q2 in zip(self.vec1, self.vec2, self.qua1, self.qua2):
            R1 = rotMatFromQuat(q1)
            R2 = rotMatFromQuat(q2)

            H1 = homogenousTransformationMatrix(R1, v1)
            H2 = homogenousTransformationMatrix(R2, v2)

            h1 = self.trackerToBowl.dot(H1.dot(self.palmToTool1))
            h2 = self.trackerToBowl.dot(H2.dot(self.palmToTool2))

            self.center1.append(h1[:-1,3])
            self.center2.append(h2[:-1,3])

            self.orientation1.append(h1[:-1,:-1])
            self.orientation2.append(h2[:-1,:-1])
        return
    
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
            vec1 = self.center1[i]
            x1,y1,z1 = self.orientation1[i][:,0],self.orientation1[i][:,1],self.orientation1[i][:,2]
            x1 = 30*x1 +vec1
            y1 = 30*y1 +vec1
            z1 = 30*z1 +vec1

            ax.plot([x1[0],vec1[0]],[x1[1],vec1[1]],[x1[2],vec1[2]], 'r')
            ax.plot([y1[0],vec1[0]],[y1[1],vec1[1]],[y1[2],vec1[2]], 'b')
            ax.plot([z1[0],vec1[0]],[z1[1],vec1[1]],[z1[2],vec1[2]], 'g')

            vec2 = self.center1[i]
            x2,y2,z2 = self.orientation2[i][:,0],self.orientation2[i][:,1],self.orientation2[i][:,2]
            x2 = 50*x2 +vec2
            y2 = 50*y2 +vec2
            z2 = 50*z2 +vec2

            ax.plot([x2[0],vec2[0]],[x2[1],vec2[1]],[x2[2],vec2[2]])
            ax.plot([y2[0],vec2[0]],[y2[1],vec2[1]],[y2[2],vec2[2]])
            ax.plot([z2[0],vec2[0]],[z2[1],vec2[1]],[z2[2],vec2[2]])

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
    print d.orientation1[2]
    print d.orientation2[2]
    print sum(map(pow, d.center1[2] - d.center2[2], [2,2,2]))**0.5
    #d.plotData('')