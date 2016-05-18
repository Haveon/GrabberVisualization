from datetime import datetime

class LaptopLine:
    def __init__(self, line):
        dateSetting = '%Y-%m-%d-%H-%M-%S-%f'
        split = line.split(',')

        self.startTime       = datetime.strptime(split[0], dateSetting)
        self.afterJoystickTS = datetime.strptime(split[1], dateSetting)
        self.joystick        = map(float, split[2:4])
        self.afterServoSetTS = datetime.strptime(split[4], dateSetting)
        self.servoSet        = map(int, split[6:10])
        self.afterServoReadTS= datetime.strptime(split[10], dateSetting)
        self.servoRead       = map(int, split[12:])

    def __repr__(self):
        return 'Data from time {}'.format(self.startTime)

class LaptopFile:
    def __init__(self, fname):
        with open(fname) as f:
            lines = f.readlines()
        
        name = lines[0][:-1]
        info = name[11:].split('-')
        self.batchNumber, self.objectNumber, self.trialNumber = map(int, info)

        clkDiff = lines[1][:-1]
        self.clockDifference = int(clkDiff[61:])

        servoStart = lines[2][:-1]
        self.servoStartPos = servoStart[18:]

        self.data = []
        for line in lines[3:-2]:
            datum = LaptopLine(line[:-1])
            self.data.append(datum)

        endGoalLine = lines[-2][:-1]
        self.endGoal= map(int, endGoalLine[24:-1].split(','))

        endServoPosLine = lines[-1][:-1]
        self.endServoPos= map(int, endServoPosLine[23:-1].split(','))

    def __repr__(self):
        batch  = 'Batch Number: {}, '.format(self.batchNumber)
        objNum = 'Object Number: {}, '.format(self.objectNumber)
        triNum = 'Trial Number: {}, '.format(self.trialNumber)
        numData= 'Number of Data Lines: {}'.format(len(self.data))
        return ''.join([batch,objNum,triNum,numData])

class LabViewLine(object):
    def __init__(self, line):
        line = line[line.find(':')+2:-1]
        dateSetting = '%Y-%m-%d-at-%H-%M-%S.%f'
        self.timeStamp = datetime.strptime(line[:25]+'0000', dateSetting)

        if 'Both Rigid bodies out of volume?' in line:
            self.x1, self.y1, self.z1 = 0.0, 0.0, 0.0
            self.q1_0, self.q1_1, self.q1_2, self.q1_2 = 0.0, 0.0, 0.0, 0.0
            
            self.x2, self.y2, self.z2 = 0.0, 0.0, 0.0
            self.q2_0, self.q2_1, self.q2_2, self.q2_2 = 0.0, 0.0, 0.0, 0.0

        else:
            data = line[line.find(':')+1:].split('***')

            tool1= data[0].split(',')
            self.x1, self.y1, self.z1 = map(float, tool1[:3])
            self.q1_0, self.q1_1, self.q1_2, self.q1_2 = map(float, tool1[3:-1])

            tool2 = data[1].split(',')
            self.x2, self.y2, self.z2 = map(float, tool2[:3])
            self.q2_0, self.q2_1, self.q2_2, self.q2_2 = map(float, tool2[3:])

    def __repr__(self):
        return 'Data from time {}'.format(self.timeStamp)

class LabViewFile(object):
    def __init__(self, fname):
        with open(fname) as f:
            lines = f.readlines()

        self.tool1 = lines[0][lines[0].rfind('\\')+1:-6]
        self.tool2 = lines[1][lines[1].rfind('\\')+1:-6]

        info = lines[2][:-1]
        self.batchNumber, self.objectNumber, self.trialNumber = map(int, info[info.rfind(':')+1:-1].split('-'))

        self.data = []
        for line in lines[6:]:
            datum = LabViewLine(line)
            self.data.append(datum)
        
    def __repr__(self):
        batch  = 'Batch Number: {}, '.format(self.batchNumber)
        objNum = 'Object Number: {}, '.format(self.objectNumber)
        triNum = 'Trial Number: {}, '.format(self.trialNumber)
        numData= 'Number of Data Lines: {}'.format(len(self.data))
        return ''.join([batch,objNum,triNum,numData])

if __name__ == '__main__':
    topFile = LaptopFile('59435648-0-1000.txt')
    labFile = LabViewFile('59435648-0-1000-2016-05-15-at-23-36-37 .txt')
    print topFile.data[2]
