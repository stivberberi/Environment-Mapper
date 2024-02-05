import math
import open3d
import numpy
import serial

'''
Stiv Berberi
Python Open3D Code and data collection
April 15, 2021
'''

'''
Opening the serial port and gathering the data into a string
'''
def getData(comPort, baudRate):
    s = serial.Serial(comPort, baudRate)

    print("Opening: " + s.name)

    dataString = ""
    c = ""
    while (True):                        # continuously loop until the uC sends a "D" to denote it is done
        x = s.read()        # read one byte
        c = x.decode()      # convert byte type to str
        if (c == "D"):
            break
        dataString += c                 # add the character 'c' into the data string, will be parsed later

    print("Closing: " + s.name)                     # close the port, data collection is done once it exits the while loop
    s.close()

    data = dataString.split("\r\n")             # \r\n characters were added when the uC sent them across UART, removing them now
    data.remove('')                             # ending character is removed
    return data

'''
Create a point cloud in an xyz file
'''
def makePointCloud(data):
    fileName = "points.xyz"
    file = open(fileName, 'w')          # create new xyz file

    angle = 0           # start at 0 for the angle
    x = 0               # start at 0 for the x coordinate, increment by 10 later

    for i in range(len(data)):

        y = float(data[i]) * math.cos(angle)
        z = float(data[i]) * math.sin(angle)            # get y and z coordinates from angles (know that sensors measure every 11.25 degrees)

        angle += float(11.25 * math.pi/180)

        if((i % 32 == 0) and (i != 0)):             # each rotation will have 32 points captured
            x += 100                    # increment x coordinate by 10cm (100mm)

        file.write(str(x) + " ")
        file.write(str(y) + " ")
        file.write(str(z) + "\n")               # write the coordinates into the file, each point on a row with each coordinate separated by a space
    
    file.close()

    return fileName

'''
Function to use Open3D to visualize the data
'''
def visualization(fileName, numPts, iterations):
    pointCloud = open3d.io.read_point_cloud(fileName, format = "xyz")       # open3d makes a point cloud from xyz file

    point = [0]*numPts                      
    for i in range(len(point)):
        point[i] = i
    
    lines = []
    p = 0
    for j in range(iterations):
        for k in range(len(point)):
            lines.append([point[k] + p, point[(k+1)%len(point)] + p])
        p += numPts
    
    p = 0
    for l in range(9):
        for m in range(len(point)):
            lines.append(point[k] + p, point[(k+1) % len(point)] + p*numPts])
        p += numPts

    lineSet = open3d.geometry.LineSet(points = open3d.utility.Vector3dVector(numpy.asarray(pointCloud.points)), lines = open3d.utility.Vector2iVector(lines))
    open3d.visualization.draw_geometries([lineSet])

    return

def main():
    baud = 115200
    com = "COM4"
    data = getData(com, baud)

    file = makePointCloud(data)                 
    
    numPts = 32             # number of points per rotation
    numIterations = 1       # number of iterations the x coordinate will move 
    visualization(file, numPts, numIterations)

    return

main()
