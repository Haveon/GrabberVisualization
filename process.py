from Viz import trialData
import numpy as np
from os import listdir, mkdir

def loadAllData():
    data = {'15333183':(['0','1','2','3'],[('1002','1005','1011','1012'),('1000','1009','1010','1012'),('1001','1002','1009','1013','1014'),()]), 
            '27140403': (['0','1'],[('1002','1005','1010','1011','1017'),('1004','1005','1007','1010','1011')]), 
            '71175377':(['0'],[('1001','1003','1004','1005','1006','1007','1009','1010','1018','1022','1030','1033')]), 
            '95917866': (['0'],[('1003','1007')])}

    result = []
    for folder in data:
        for obj in data[folder][0]:
            try:
                
                i=1000
                while True:
                    if not(str(i) in data[folder][1][int(obj)]):
                        d = folder+'-'+obj+'-'+str(i)
                        fpath = 'Data/'+folder+'/'+d+'f/'
                        tFile = 'Data/{}/{}-{}-{}'.format(folder,folder,obj, str(i))
                        dFile = 'Data/{}/{}-{}-{}-Polaris.txt'.format(folder,folder,obj, str(i))
                        try:
                            tmp = trialData(tFile, dFile)
                        except ValueError,e:
                            print tFile
                            print dFile
                            assert False
                        result.append(tmp)
                    i+=1
            
            except IOError, e:
                continue

    return result

def fetchCenterOrientation(data):
    center = []
    orientation = []
    for datum in data:
        center.append(datum.center1[-1])
        orientation.append(datum.orientation1[-1])

    return center,orientation

def fetchTimeDistanceServo(data):
    results = []
    for datum in data:
        time  = datum.ts
        dist  = datum.dist
        servo = datum.servoRead
        
        r = np.array([(t, servo[i][0], servo[i][1], servo[i][2], servo[i][3], dist[i]) for i,t in enumerate(time)])
        results.append(r)

    return results

if __name__ == '__main__':
    import pickle
    data = loadAllData()
    res = fetchTimeDistanceServo(data)
    with open('radSer.pkl','wb') as f:
        pickle.dump(res, f)

    center,orientation = fetchCenterOrientation(data)
    with open('results.txt','wb') as f:
        for c,o in zip(center,orientation):
            x,y,z = c
            m00, m01, m02 = o[:,0]
            m10, m11, m12 = o[:,1]
            m20, m21, m22 = o[:,2]
            line = ','.join(map(str, [x,y,z,m00, m01, m02,m10, m11, m12,m20, m21, m22]))
            f.write(line+'\n')
    with open('results.pkl','wb') as f:
        pickle.dump({'center': center, 'orientation': orientation},f)