import numpy as np

def extVec(vec):
    """
    Takes in an array as [x, y, z]
    """
    vec = [_ for _ in vec] + [1]
    vec = np.array(vec)
    vec.shape = (4,1)
    return vec

def unExtVec(extVec):
    return extVec[:-1]

def rotMatFromQuat(quat):
    """
    Takes in array as [qr, qx, qy, qz]
    https://en.wikipedia.org/wiki/Quaternions_and_spatial_rotation
    """
    qr, qx, qy, qz = quat
    first = [1-2*qy*qy-2*qz*qz, 2*(qx*qy-qz*qr),   2*(qx*qz+qy*qr)]
    second= [2*(qx*qy+qz*qr),   1-2*qx*qx-2*qz*qz, 2*(qy*qz-qx*qr)]
    third = [2*(qx*qz-qy*qr),   2*(qy*qz+qx*qr),   1-2*qx*qx-2*qy*qy]
    R = np.array([first,second,third])
    return R

def homogenousTransformationMatrix(rotMat, origin):
    H = np.zeros((4,4))
    H[0:3,0:3] = rotMat
    H[:,3] = extVec(origin).flatten()
    return H

def inverseHomegenousTransformation(H):
    R = H[0:3,0:3]
    origin = H[:-1,3]
    origin.shape = (3,1)
    
    R = R.T
    origin = -R.dot(origin)

    return homogenousTransformationMatrix(R, list(origin.flatten()))


if __name__ == '__main__':
    R = rotMatFromQuat([0.191109, -0.173541, -0.965400, -0.036942])
    H = homogenousTransformationMatrix(R, [12, 3, 9])
    h = inverseHomegenousTransformation(H)