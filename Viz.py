from LineSampler import LabViewFile, LaptopFile
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np

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

topFile = LaptopFile('59435648-0-1000.txt')
labFile = LabViewFile('59435648-0-1000-2016-05-15-at-23-36-37 .txt')

start = topFile.data[0].startTime
end   = topFile.data[-1].startTime
diff  = topFile.clockDifference

labData = [i for i in labFile.data if end >= i.timeStamp-diff >= start]

x1,y1,z1 = [_.vec1[0] for _ in labData], [_.vec1[1] for _ in labData], [_.vec1[2] for _ in labData]
x2,y2,z2 = [_.vec2[0] for _ in labData], [_.vec2[1] for _ in labData], [_.vec2[2] for _ in labData]
x_min, x_max = min(x1+x2), max(x1+x2)
y_min, y_max = min(y1+y2), max(y1+y2)
z_min, z_max = min(z1+z2), max(z1+z2)

for i in range(len(topFile.data)):
    fig = plt.figure(figsize=(16,9), dpi=100)
    ax = fig.add_subplot(121)
    servoData = topFile.data[i].servoRead
    index = np.arange(4)
    bar_width = 0.35

    rects = ax.bar(index, servoData, bar_width)
    plt.xticks(index+0.5*bar_width, ('M1', 'M2', 'M3', 'M4'))
    plt.ylim([14000,18000])
    plt.title(str(topFile.data[i].startTime))

    vec1 = labData[i].vec1
    qua1 = labData[i].qua1
    x1,y1,z1 =  frameTransform(qua1, vec1)

    ax = fig.add_subplot(122, projection='3d')
    ax.plot([x1[0],vec1[0]],[x1[1],vec1[1]],[x1[2],vec1[2]])
    ax.plot([y1[0],vec1[0]],[y1[1],vec1[1]],[y1[2],vec1[2]])
    ax.plot([z1[0],vec1[0]],[z1[1],vec1[1]],[z1[2],vec1[2]])

    vec2 = labData[i].vec2
    qua2 = labData[i].qua2
    x2,y2,z2 =  frameTransform(qua2, vec2)

    ax.plot([x2[0],vec1[0]],[x2[1],vec1[1]],[x2[2],vec1[2]])
    ax.plot([y2[0],vec1[0]],[y2[1],vec1[1]],[y2[2],vec1[2]])
    ax.plot([z2[0],vec1[0]],[z2[1],vec1[1]],[z2[2],vec1[2]])

    plt.title(str(labData[i].timeStamp-diff))
    ax.set_xlim(x_min,x_max)
    ax.set_ylim(y_min,y_max)
    ax.set_zlim(z_min,z_max)
    plt.savefig(str(i)+'.png')