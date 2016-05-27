from datetime import datetime, timedelta

class LaptopLine:
    def __init__(self, line):
        dateSetting = '%Y-%m-%d-%H-%M-%S-%f'
        split = line.split(',')

        self.startTime       = datetime.strptime(split[0], dateSetting)
        self.afterJoystickTS = datetime.strptime(split[1], dateSetting)
        self.joystick        = map(float, split[2:4])
        self.afterServoSetTS = datetime.strptime(split[4], dateSetting)
        self.servoSet        = map(int, split[5:9])
        self.afterServoReadTS= datetime.strptime(split[9], dateSetting)
        self.servoRead       = map(int, split[10:])

    def __repr__(self):
        return 'Data from time {}'.format(self.startTime)

class LaptopFile:
    def __init__(self, fname):
        with open(fname) as f:
            lines = f.readlines()
        
        name = lines[0][:-1]
        info = name[11:].split('-')
        self.batchNumber, self.objectNumber, self.trialNumber = map(int, info)

        clkDiff = lines[2][:-1]
        self.clockDifference = timedelta(microseconds=int(clkDiff[61:]))

        servoStart = lines[3][:-1]
        self.servoStartPos = servoStart[18:]

        self.data = []
        for line in lines[4:-2]:
            datum = LaptopLine(line[:-1])
            self.data.append(datum)

        endGoalLine = lines[-2][:-1]
        endGoalLine = endGoalLine[endGoalLine.rfind(':')+3:-1].split(',')
        self.endGoal= map(int, endGoalLine[1:])

        endServoPosLine = lines[-1][:-1]
        endServoPosLine = endServoPosLine[endServoPosLine.rfind(':')+3:-1].split(',')
        self.endServoPos= map(int, endServoPosLine[1:])

    def __repr__(self):
        batch  = 'Batch Number: {}, '.format(self.batchNumber)
        objNum = 'Object Number: {}, '.format(self.objectNumber)
        triNum = 'Trial Number: {}, '.format(self.trialNumber)
        numData= 'Number of Data Lines: {}'.format(len(self.data))
        return ''.join([batch,objNum,triNum,numData])

class LabViewLine(object):
    def __init__(self, line):
        line = line[line.find(':')+2:-1]
        dateSetting = '%Y-%m-%d-%H-%M-%S.%f'
        self.timeStamp = datetime.strptime(line[:25], dateSetting)

        if 'Both Rigid bodies out of volume?' in line:
            self.vec1 = [0.0, 0.0, 0.0]
            self.qua1 = [0.0, 0.0, 0.0, 0.0]
            
            self.vec2 = [0.0, 0.0, 0.0]
            self.qua2 = [0.0, 0.0, 0.0, 0.0]

        else:
            data = line[line.rfind(':')+1:].split('***')

            tool1= data[0].split(',')
            self.vec1 = map(float, tool1[:3])
            self.qua1 = map(float, tool1[3:-1])

            tool2 = data[1].split(',')
            self.vec2 = map(float, tool2[:3])
            self.qua2 = map(float, tool2[3:])

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
    topFile = LaptopFile('Data/33519252-0-1011')
    labFile = LabViewFile('Data/33519252-0-1011-2016-05-25-at-20-20-24 .txt')
