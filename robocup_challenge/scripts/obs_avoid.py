#!/usr/bin/env python
from path_planning import *

rospy.init_node('fish')

A = planner(31.2, 54.4, 39, 57)

x = []
y = []
for item in reversed(A[0]):
    y.append(item)
for item in reversed(A[1]):
    x.append(item)

# instanciar el modulo de deection
B = RGBD()
pose = B.detect()
print("pose: ",pose)
print(" ")


#
#poses = B.detect()[0][0]
#n = len(poses)
#print(" ")
#print(B.detect()[0], type(B.detect()[0]))



if __name__ == '__main__':
    rospy.init_node('fish')

    #real_pose = utils_hb.make_pose_from_camera(poses)
    #map_pose = utils_hb.get_pose_relative_coordinate('map', real_pose)
    #utils_hb.rviz_marker('map', map_pose.pose.position.x, map_pose.pose.position.y, map_pose.pose.position.z)
    ##print(map_pose)
    #print("")
    #print("")
    #print("x pose: ", map_pose.pose.position.x*10,"y pose: ", map_pose.pose.position.y*10)

    print("")
    print("")

    X = []
    Y = []

    for i in range(len(B.detect()[0])):
        poses = B.detect()[0][i]

        real_pose = utils_hb.make_pose_from_camera(poses)
        map_pose = utils_hb.get_pose_relative_coordinate('map', real_pose)
        utils_hb.rviz_marker('map', map_pose.pose.position.x, map_pose.pose.position.y, map_pose.pose.position.z)
        X.append(map_pose.pose.position.x)
        Y.append(map_pose.pose.position.y)

    print(X,Y)

    '''
    poses = B.detect()[0][0]
    print(poses)
    X = []
    Y = []
    
    #print(value)
    real_pose = utils_hb.make_pose_from_camera(poses)
    map_pose = utils_hb.get_pose_relative_coordinate('map', real_pose)
    utils_hb.rviz_marker('map', map_pose.pose.position.x, map_pose.pose.position.y, map_pose.pose.position.z)
    X.append(map_pose.pose.position.x)
    Y.append(map_pose.pose.position.y)
    #print(X,Y)

    print("")
    print("")

    poses = B.detect()[0][1]
    print(poses)
    X = []
    Y = []
    
    #print(value)
    real_pose = utils_hb.make_pose_from_camera(poses)
    map_pose = utils_hb.get_pose_relative_coordinate('map', real_pose)
    utils_hb.rviz_marker('map', map_pose.pose.position.x, map_pose.pose.position.y, map_pose.pose.position.z)
    X.append(map_pose.pose.position.x)
    Y.append(map_pose.pose.position.y)
    #print(X,Y)

    tuple_pose = X, Y
    print("")
    print("")
    print(tuple_pose)

    poses = B.detect()[0][2]
    print(poses)
    X = []
    Y = []
    
    #print(value)
    real_pose = utils_hb.make_pose_from_camera(poses)
    map_pose = utils_hb.get_pose_relative_coordinate('map', real_pose)
    utils_hb.rviz_marker('map', map_pose.pose.position.x, map_pose.pose.position.y, map_pose.pose.position.z)
    X.append(map_pose.pose.position.x)
    Y.append(map_pose.pose.position.y)
    #print(X,Y)

    tuple_pose = X, Y
    print("")
    print("")
    print(tuple_pose)

    #for i in range(n):
    #    #main()
    #    move_head_tilt(-1)
    #    m = Move()
    #    m.set_pose(x[i], y[i], 130)
    #    #m.get_pose()
    #    m.go()
'''