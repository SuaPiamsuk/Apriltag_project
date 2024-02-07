import numpy
import numpy.matlib as npm

# Q is a Nx4 numpy matrix and contains the quaternions to average in the rows.
# The quaternions are arranged as (w,x,y,z), with w being the scalar
# The result will be the average quaternion of the input. Note that the signs
# of the output quaternion can be reversed, since q and -q describe the same orientation
def averageQuaternions(Q):
    # Number of quaternions to average
    M = Q.shape[0]
    A = npm.zeros(shape=(4,4))

    for i in range(0,M):
        q = Q[i,:]
        # multiply q with its transposed version q' and add A
        A = numpy.outer(q,q) + A

    # scale
    A = (1.0/M)*A
    # compute eigenvalues and -vectors
    eigenValues, eigenVectors = numpy.linalg.eig(A)
    # Sort by largest eigenvalue
    eigenVectors = eigenVectors[:,eigenValues.argsort()[::-1]]
    # return the real part of the largest eigenvector (has only real part)
    return numpy.real(eigenVectors[:,0].A1)


###################################################################################
#################################### how to use ###################################
###################################################################################

# สร้างเมทริกซ์ Q จาก quaternions
Q = numpy.array([
   [0.5, 0.5, 0.5, 0.5],  # Quaternion 1
   [0.8, 0.2, 0.1, 0.5],  # Quaternion 2
#    [0.2, 0.3, 0.9, 0.1]   # Quaternion 3
])

# หาค่า quaternion เฉลี่ย
avg_quaternion = averageQuaternions(Q)

print("Average Quaternion:", avg_quaternion)