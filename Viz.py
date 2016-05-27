from LineSampler import LabViewFile, LaptopFile
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
from rigidTrans import rotMatFromQuat, extVec, homogenousTransformationMatrix

def quaMul(q1,q2):
    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2
    w = w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2
    x = w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2
    y = w1 * y2 + y1 * w2 + z1 * x2 - x1 * z2
    z = w1 * z2 + z1 * w2 + x1 * y2 - y1 * x2
    return [w, x, y, z]

def qConj(q):
    w, x, y, z = q
    return [w, -x, -y, -z]

def quaVecMul(q1, v1):
    q2 = [0.0] + v1
    return quaMul(quaMul(q1, q2), qConj(q1))[1:]

def frameTransform(quat, trans):
    x = [10., 0, 0]
    y = [0, 10., 0]
    z = [0, 0, 10.]

    x_prime = np.sum([quaVecMul(quat, x), trans], axis=0)
    y_prime = np.sum([quaVecMul(quat, y), trans], axis=0)
    z_prime = np.sum([quaVecMul(quat, z), trans], axis=0)

    return x_prime, y_prime, z_prime

#-------------------------------------------------------------

class trialData:
    def __init__(self, laptopFname, labViewFname):
        topFile = LaptopFile(laptopFname)
        labFile = LabViewFile(labViewFname)

        self.batchNumber     = topFile.batchNumber
        self.objectNumber    = topFile.objectNumber
        self.trialNumber     = topFile.trialNumber
        self.clockDifference = topFile.clockDifference
        self.servoStartPos   = topFile.servoStartPos
        self.topData         = topFile.data
        self.endGoal         = topFile.endGoal
        self.endServoPos     = topFile.endServoPos

        self.tool1   = labFile.tool1
        self.tool2   = labFile.tool2
        self.labData = labFile.data

        self.trackerToBowl = np.loadtxt('248Calibration.txt', delimiter=',')
        self.palmToTool1 = np.loadtxt('449Calibration.txt', delimiter=',')
        self.palmToTool2 = np.loadtxt('339Calibration.txt', delimiter=',')

        self._synchronize()
        self._referenceTransformation()

    def _synchronize(self):
        """
        This changes self.labData in place!!
        """
        start = self.topData[0].startTime
        end   = self.topData[-1].startTime
        diff  = self.clockDifference

        topTimeStamps = [_.startTime for _ in self.topData]        
        labTimeStamps = []
        for line in self.labData:
            line.timeStamp -= diff
            labTimeStamps.append(line.timeStamp)

        indexes = []
        for laptopTime in topTimeStamps:
            tmp = [abs(labViewtime-laptopTime) for labViewtime in labTimeStamps]
            indexes.append(np.argmin(tmp))
            
        self.labData = [self.labData[i] for i in indexes]
        return

    def _getAxisMeasurements(self):

        x1 = [_[0] for _ in self.center1]
        y1 = [_[1] for _ in self.center1]
        z1 = [_[2] for _ in self.center1]
        
        x2 = [_[0] for _ in self.center2]
        y2 = [_[1] for _ in self.center2]
        z2 = [_[2] for _ in self.center2]

        xs,ys,zs = [_ for _ in x1+x2 if _ !=0.0], [_ for _ in y1+y2 if _ !=0.0], [_ for _ in z1+z2 if _ !=0.0]
        x_min, x_max = min(xs), max(xs)
        y_min, y_max = min(ys), max(ys)
        z_min, z_max = min(zs), max(zs)

        return ((x_min,x_max),(y_min,y_max),(z_min,z_max))


    def _referenceTransformation(self):
        self.center1 = []
        self.orientation1 = []
        self.center2 = []
        self.orientation2 = []
        for datum in self.labData:
            R1 = rotMatFromQuat(datum.qua1)
            R2 = rotMatFromQuat(datum.qua2)

            H1 = homogenousTransformationMatrix(R1, datum.vec1)
            H2 = homogenousTransformationMatrix(R2, datum.vec2)

            h1 = self.trackerToBowl.dot(H1.dot(self.palmToTool1))
            h2 = self.trackerToBowl.dot(H2.dot(self.palmToTool2))

            self.center1.append(h1[:-1,3])
            self.center2.append(h2[:-1,3])

            self.orientation1.append(h1[:-1,:-1])
            self.orientation2.append(h2[:-1,:-1])
        return

    def plotData(self, fname):
        for i in range(len(self.labData)):
            fig = plt.figure(figsize=(16,9), dpi=100)
            ax = fig.add_subplot(121)
            servoData = self.topData[i].servoRead
            index = np.arange(4)
            bar_width = 0.35

            rects = ax.bar(index, servoData, bar_width)
            plt.xticks(index+0.5*bar_width, ('M1', 'M2', 'M3', 'M4'))
            plt.ylim([14000,18000])
            plt.title(str(self.topData[i].startTime))

            vec1 = self.center1[i]
            x1,y1,z1 = self.orientation1[i][:,0],self.orientation1[i][:,1],self.orientation1[i][:,2]
            x1 = 30*x1 +vec1
            y1 = 30*y1 +vec1
            z1 = 30*z1 +vec1

            ax = fig.add_subplot(122, projection='3d')
            ax.plot([x1[0],vec1[0]],[x1[1],vec1[1]],[x1[2],vec1[2]], 'r')
            ax.plot([y1[0],vec1[0]],[y1[1],vec1[1]],[y1[2],vec1[2]], 'b')
            ax.plot([z1[0],vec1[0]],[z1[1],vec1[1]],[z1[2],vec1[2]], 'g')

            vec2 = self.center1[i]
            x2,y2,z2 = self.orientation2[i][:,0],self.orientation2[i][:,1],self.orientation2[i][:,2]
            x2 = 0*x2 +vec2
            y2 = 0*y2 +vec2
            z2 = 0*z2 +vec2

            ax.plot([x2[0],vec2[0]],[x2[1],vec2[1]],[x2[2],vec2[2]])
            ax.plot([y2[0],vec2[0]],[y2[1],vec2[1]],[y2[2],vec2[2]])
            ax.plot([z2[0],vec2[0]],[z2[1],vec2[1]],[z2[2],vec2[2]])

            plt.title(str(self.labData[i].timeStamp))
            #xBound, yBound, zBound = self._getAxisMeasurements()
            ax.set_xlim(-200,200)
            ax.set_ylim(-200,200)
            ax.set_zlim(0,300)

            ax.plot([0,100],[0,0],[0,0],'k')
            ax.plot([0,0],[0,100],[0,0],'k')
            ax.plot([0,0],[0,0],[0,100],'k')
            
            plt.savefig(fname+str(i)+'.png')
            plt.close()

if __name__ == '__main__':
    tFile = 'Data/33519252-0-1008'
    dFile = 'Data/33519252-0-1008-2016-05-25-at-20-18-33 .txt'
    d = trialData(tFile, dFile)
    d.plotData('')