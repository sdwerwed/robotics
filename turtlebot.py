#!/usr/bin/env python
import matplotlib.pyplot as plt
import numpy as np
import cv2
from turtlebot import Turtlebot, detector, Rate, get_time, sleep
from datetime import datetime



dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_ARUCO_ORIGINAL)

par = cv2.aruco.DetectorParameters_create()
par.adaptiveThreshConstant = 7.0
par.adaptiveThreshWinSizeMax = 23
par.adaptiveThreshWinSizeMin = 3
par.adaptiveThreshWinSizeStep = 10
par.cornerRefinementMaxIterations = 30
par.cornerRefinementMinAccuracy = 0.1
par.cornerRefinementWinSize = 5
# par.doCornerRefinement = False
par.errorCorrectionRate = 0.6
par.markerBorderBits = 1
par.maxErroneousBitsInBorderRate = 0.35
par.maxMarkerPerimeterRate = 4.0
par.minCornerDistanceRate = 0.05
par.minDistanceToBorder = 3
par.minMarkerDistanceRate = 0.05
par.minMarkerPerimeterRate = 0.03
par.minOtsuStdDev = 5.0
par.perspectiveRemoveIgnoredMarginPerCell = 0.13
par.perspectiveRemovePixelPerCell = 4
par.polygonalApproxAccuracyRate = 0.03

def detect_markers(image):
    detections, ids, falsepos = cv2.aruco.detectMarkers(image, dictionary)

    if ids is None:
        return []

    dets = []
    for d, i in zip(detections, ids):
        dets.append((d[0], i[0]))

    return dets

def find_pozition_beacon(pc, markers, odometry):
    R = R_from_ang_single(odometry[2])
    # print("Markers.T: ", markers[0][0].T)
    # print("positions.T: ", [[odometry[0]], [odometry[1]]])
    # print("angle: ", odometry[2])
    # print("R: ", R)
    # print("markersnew: ", markers)

    x0, y0 = markers[0, 0][0][0]-4, markers[0, 0][0][1]-4
    x1, y1 = markers[0, 0][1][0]-4, markers[0, 0][1][1]-4
    x2, y2 = markers[0, 0][2][0]-4, markers[0, 0][2][1]-4
    x3, y3 = markers[0, 0][3][0]-4, markers[0, 0][3][1]-4

    if int(y0) in range(50,480) and int(x0) in range(0,600) and pc[int(y0), int(x0), 2] > 0 and pc[int(y0), int(x0), 2] < 3:

        print("x0=", x0)
        print("y0=", y0)
        crd= (np.dot(R, [[pc[int(y0), int(x0), 0]], [pc[int(y0), int(x0), 2]]]) + [[odometry[0]], [odometry[1]]]).T
        return crd[0,1], crd[0,0]


    if int(y1) in range(50,480) and int(x1) in range(0,600) and pc[int(y1), int(x1), 2] > 0 and pc[int(y1), int(x1), 2] < 3:
        print("x1=", x0)
        print("y2=", y0)
        crd = (np.dot(R, [[pc[int(y1), int(x1), 0]], [pc[int(y1), int(x1), 2]]]) + [[odometry[0]], [odometry[1]]]).T
        return crd[0, 1], crd[0, 0]

    if int(y2) in range(50,480) and int(x2) in range(0,600) and pc[int(y2), int(x2), 2] > 0 and pc[int(y2), int(x2), 2] < 3:
        print("x2=", x0)
        print("y2=", y0)
        crd = (np.dot(R, [[pc[int(y2), int(x2), 0]], [pc[int(y2), int(x2), 2]]]) + [[odometry[0]], [odometry[1]]]).T
        return crd[0, 1], crd[0, 0]

    if int(y3) in range(50,480) and int(x3) in range(0,600) and pc[int(y3), int(x3), 2] > 0 and pc[int(y3), int(x3), 2] < 3:
        print("x3=", x0)
        print("y3=", y0)
        crd = (np.dot(R, [[pc[int(y3), int(x3), 0]], [pc[int(y3), int(x3), 2]]]) + [[odometry[0]], [odometry[1]]]).T
        return crd[0, 1], crd[0, 0]

    else:
        return -1

def R_from_ang_single(ang):
    R = np.array([[np.cos(ang), -np.sin(ang)], [np.sin(ang), np.cos(ang)]])
    return R

def click(vent, x, y, flags, param):
    global active
    active = not active
    print active

def pcl_to_2d(pc):
    x = []
    y = []
    z = []
    y_range=(0.02, 0.03)
    z_range=(0, 3)
    for u in range(0, 480):
        for v in range(0, 640):
            # Masking
            if np.isfinite(pc[u, v, 0]) and np.isfinite(pc[u, v, 1]) and np.isfinite(pc[u, v, 2]) and pc[u, v, 1] > y_range[0] and pc[u, v, 1] < y_range[1] and pc[u, v, 2] > z_range[0] and pc[u, v, 2] < z_range[1]:
                x.append(pc[u, v, 0])
                y.append(pc[u, v, 1])
                z.append(pc[u, v, 2])


    # fig = plt.figure()
    # ax = fig.add_subplot(111)
    # ax.scatter(x, z, marker='.')
    # ax.set_xlabel('X Label')
    # ax.set_ylabel('Y Label')
    # ax.axis('equal')
    # ax.set_xlim((-3, 3))
    # ax.set_ylim((0, 5))
    # plt.show()
    return np.array([x, z])[:, :1450]
    # x = np.array(x)
    # z = np.array(z)
    # cv2.plot.Plot2d_create(x, z)

def get_line(start, end):
    # Setup initial conditions
    x1, y1 = start
    x2, y2 = end
    dx = x2 - x1
    dy = y2 - y1

    # Determine how steep the line is
    is_steep = abs(dy) > abs(dx)

    # Rotate line
    if is_steep:
        x1, y1 = y1, x1
        x2, y2 = y2, x2

    # Swap start and end points if necessary and store swap state
    swapped = False
    if x1 > x2:
        x1, x2 = x2, x1
        y1, y2 = y2, y1
        swapped = True

    # Recalculate differentials
    dx = x2 - x1
    dy = y2 - y1

    # Calculate error
    error = int(dx / 2.0)
    ystep = 1 if y1 < y2 else -1

    # Iterate over bounding box generating points between start and end
    y = y1
    points = []
    for x in range(x1, x2 + 1):
        coord = (y, x) if is_steep else (x, y)
        points.append(coord)
        error -= abs(dy)
        if error < 0:
            y += ystep
            error += dx

    # Reverse the list if the coordinates were swapped
    if swapped:
        points.reverse()

    return points

def occ(twd_map, odomet, grid, scale, minx_occ, miny_occ ):


    robot_poz_x = odomet[0]*scale + minx_occ
    robot_poz_y = odomet[1]*scale + miny_occ

    robot_angle = odomet[2] + np.deg2rad(90)
    twd_map = twd_map[:, ~np.isnan(twd_map).any(axis=0)]

    for i in range (0, twd_map.shape[1]):

        a = int(round(twd_map[0, i]+minx_occ))
        b = int(round(twd_map[1, i]+miny_occ))
        if grid[a, b] != 1:
            grid[a, b] = 1
    print("rovot_poz_x,y", (robot_poz_x, robot_poz_y))
    for i in np.arange( robot_angle - np.deg2rad(30), robot_angle + np.deg2rad(30), 0.01):
        x = robot_poz_x + np.cos(i) * 3*scale
        y = robot_poz_y + np.sin(i) * 3*scale

        B = get_line([int(round(robot_poz_x)), int(round(robot_poz_y))], [int(round(x)), int(round(y))])
        for i in range(0, len(B)):
            if grid[B[i][0], B[i][1]] != 0:
                if grid[B[i][0], B[i][1]] == 1:
                    break
                else:
                     grid[B[i][0], B[i][1]] = 0
    plt.figure(3)
    plt.title('Grid')
    plt.imshow(grid.T, interpolation='none')
    plt.gca().invert_yaxis()
    plt.draw()
    plt.pause(0.00000001)


def collision(grid, x1,y1):
    r=3
    x1 = int(round(x1))
    y1 = int(round(y1))
    print("X1,Y1 oc", x1,y1)
    print("Grid in oc ",grid[x1-r:x1+r,y1-r:y1+r])
    # grid[int(x1 - r):int(x1 + r), int(y1 - r):int(y1 + r)] = -1
    # plt.figure()
    # plt.imshow(grid.T, interpolation='none')
    # plt.gca().invert_yaxis()
    # plt.show()
    grid[x1,y1]=-1
    if 1 in grid[x1-r:x1+r,y1-r:y1+r]:
        return True
    else:

        return False


def main():



    Found_first_beacon = False
    Found_second_beacon = False
    Found_third_beacon = False
    Start_caprure_2D = False
    see_map_with_rotation = False
    full_rotarion_1 = False
    full_rotarion_2 = False
    movment1 = False
    data = np.empty((2,1500,100))
    data[:] = np.nan
    odom = np.empty((1, 3, 100))
    odom[:] = np.nan
    counter=0;
    sample_T_2D=5;
    C=np.array([[],[]])
    MOVE = 1
    ROTATE = 2
    running = True
    active = True

    linear_vel = 0.2
    angular_vel = 0.4
    turtle = Turtlebot(rgb=True, depth=False, pc=True)
    direction = -1

    scale = float(12)
    grid_width = 150
    grid = -1 * np.ones((grid_width, grid_width))
    minx_occ = 70
    miny_occ = 70

    plt.ion()
    plt.show()


    turtle.reset_odometry()
    # get odometry
    odometry = turtle.get_odometry()
    if np.array_equal(odometry, [0.0, 0.0, 0.0]):
        print ("Odom reset correctly")
    else:
        print ("Odom is not zero")
        return -1
    rate = Rate(10)
    t = get_time()


    # plt.figure(3)
    # plt.title('Grid')
    # plt.imshow(grid.T, interpolation='none')
    # plt.gca().invert_yaxis()
    # plt.show()

    while not turtle.is_shutting_down():


        # get RGB
        img_rgb = turtle.get_rgb_image()
        # get odometry
        odometry = turtle.get_odometry()
        odometry=[odometry[1],odometry[0],odometry[2]]
        odometry = np.dot(odometry, [[-1, 0, 0],
                            [0, 1, 0],
                            [0, 0, 1]])
        # print("Odometry", odometry)
        # get point cloud
        pc = turtle.get_point_cloud()


        if pc is None:
            continue



        if img_rgb is not None and pc is not None:

            # plt.figure(3)
            # plt.title('Grid')
            # plt.imshow(grid.T, interpolation='none')
            # plt.gca().invert_yaxis()
            # plt.show()

            # Print 2D and update
            if  get_time() - t > sample_T_2D:



                A = pcl_to_2d(pc)
                # get odometry
                odometry = turtle.get_odometry()
                odometry = [odometry[1], odometry[0], odometry[2]]
                odometry = np.dot(odometry, [[-1, 0, 0],
                                             [0, 1, 0],
                                             [0, 0, 1]])

                R = R_from_ang_single(odometry[2])
                T = [[odometry[0]], [odometry[1]]]
                map = np.dot(R, A) + T
                occ(map*scale,odometry, grid, scale, minx_occ, miny_occ)
                t = get_time()

            #     # C = np.hstack((C, map))
            #     # print("shape map= ", C.shape)
            #     # plt.scatter(C[0, :], C[1, :], marker='.')
            #     # plt.scatter(odometry[0], odometry[1])
            #     # plt.axis('equal')
            #     # plt.ylim(-5, 5)
            #     # plt.xlim(-5, 5)
            #     # plt.pause(0.001)
            #     # show image
            #     cv2.imshow("grid", grid)
            #     cv2.waitKey(1)

            #     Start_caprure_2D = True





            # Find the 2nd beacon
            if Found_first_beacon == True and Found_second_beacon == False:
                turtle.cmd_velocity(angular=0.3)

            if Found_first_beacon == True and Found_second_beacon == True and see_map_with_rotation == False:
                turtle.cmd_velocity(angular=0.7)
                if -1.5 <= odometry[2] <= 0:
                    see_map_with_rotation = True
                    # np.save("/home/ros/Desktop/aroprojectjim/grid.npy", grid)





            # Make full rotation
            if 0 < odometry[0] < 2.5 and 2.5 < odometry[1] < 3.4 and (full_rotarion_1 == False):
                print ("inside if for rotation")
                t=get_time()
                print("Full rotation 1 before")
                while  (get_time()-t <16) :
                    turtle.cmd_velocity(angular=0.5)

                    # get RGB
                    img_rgb = turtle.get_rgb_image()

                    # get point cloud
                    pc = turtle.get_point_cloud()

                    markers = np.array(detect_markers(img_rgb))
                    # draw markers in the image

                    detector.draw_markers(img_rgb, markers)
                    # show image
                    cv2.imshow("markers", img_rgb)
                    cv2.waitKey(1)
                    # Odometry
                    # print(turtle.get_odometry())
                    if len(markers) != 0:
                        id = markers[0, 1]
                        if id not in id_array and int(id) in range(5, 17):

                            # Print depth
                            # plt.figure(2)
                            # plt.title('depth')
                            # plt.imshow(depth)
                            # plt.show(clear)

                            # find coordinates
                            # get odometry
                            odometry = turtle.get_odometry()
                            odometry = [odometry[1], odometry[0], odometry[2]]
                            odometry = np.dot(odometry, [[-1, 0, 0],
                                                         [0, 1, 0],
                                                         [0, 0, 1]])
                            coordinates_beacon = find_pozition_beacon(pc, markers, odometry)
                            if coordinates_beacon != -1:

                                id_array.append(id)
                                cord_array.append([coordinates_beacon[0], coordinates_beacon[1]])

                                if len(id_array) == 1:
                                    print ("found first beacon ")
                                    Found_first_beacon = True

                                    # play sound
                                    turtle.play_sound(sound_id=3)

                                if len(id_array) == 2:
                                    Found_second_beacon = True

                                    # play sound
                                    turtle.play_sound(sound_id=3)

                                if len(id_array) == 3:
                                    Found_second_beacon = True

                                    # play sound
                                    turtle.play_sound(sound_id=3)

                                if len(id_array) == 4:
                                    Found_second_beacon = True

                                    # play sound
                                    turtle.play_sound(sound_id=4)

                                    return 1

                print("Full rotation 1 after")
                full_rotarion_1 = True

            # Make full rotation
            if -3 < odometry[0] < 0 and 2.5 < odometry[1] < 3.4 and (full_rotarion_2 == False):
                print ("inside if for rotation 2")
                t=get_time()
                while  (get_time()-t <16) :
                    turtle.cmd_velocity(angular=0.5)

                    # get RGB
                    img_rgb = turtle.get_rgb_image()

                    # get point cloud
                    pc = turtle.get_point_cloud()

                    markers = np.array(detect_markers(img_rgb))
                    # draw markers in the image

                    detector.draw_markers(img_rgb, markers)
                    # show image
                    cv2.imshow("markers", img_rgb)
                    cv2.waitKey(1)
                    # Odometry
                    # print(turtle.get_odometry())
                    if len(markers) != 0:
                        id = markers[0, 1]
                        if id not in id_array and int(id) in range(5, 17):

                            # Print depth
                            # plt.figure(2)
                            # plt.title('depth')
                            # plt.imshow(depth)
                            # plt.show(clear)

                            # find coordinates
                            # get odometry
                            odometry = turtle.get_odometry()
                            odometry = [odometry[1], odometry[0], odometry[2]]
                            odometry = np.dot(odometry, [[-1, 0, 0],
                                                         [0, 1, 0],
                                                         [0, 0, 1]])
                            coordinates_beacon = find_pozition_beacon(pc, markers, odometry)
                            if coordinates_beacon != -1:

                                id_array.append(id)
                                cord_array.append([coordinates_beacon[0], coordinates_beacon[1]])

                                if len(id_array) == 1:
                                    print ("found first beacon ")
                                    Found_first_beacon = True

                                    # play sound
                                    turtle.play_sound(sound_id=3)

                                if len(id_array) == 2:
                                    Found_second_beacon = True

                                    # play sound
                                    turtle.play_sound(sound_id=3)

                                if len(id_array) == 3:
                                    Found_second_beacon = True

                                    # play sound
                                    turtle.play_sound(sound_id=3)

                                if len(id_array) == 4:
                                    Found_second_beacon = True

                                    # play sound
                                    turtle.play_sound(sound_id=4)

                                    return 1
                full_rotarion_2 = True


            # Find the 3rd and 4rd beacon

            if  Found_first_beacon == True and Found_second_beacon == True and see_map_with_rotation == True:


                # mask out floor points
                mask = pc[:, :, 1] < 0.2

                # mask point too far
                mask = np.logical_and(mask, pc[:, :, 2] < 3.0)

                if np.count_nonzero(mask) <= 0:
                    continue

                # check obstacle
                mask = np.logical_and(mask, pc[:, :, 1] > 0)
                mask = np.logical_and(mask, pc[:, :, 1] < 0.1)
                data = np.sort(pc[:, :, 2][mask])



                state = MOVE
                if data.size > 50:

                    dist = np.percentile(data, 10)
                    # get odometry
                    odometry = turtle.get_odometry()
                    odometry = [odometry[1], odometry[0], odometry[2]]
                    odometry = np.dot(odometry, [[-1, 0, 0],
                                                 [0, 1, 0],
                                                 [0, 0, 1]])
                    # print ("Odom before collision function :", odometry)
                    # col = collision(grid, odometry[0]*scale + minx_occ, odometry[1]*scale + miny_occ)
                    # print("Colission free ", col )
                    # print("dist ", dist)
                    # print("Odom after collision function: ", odometry)
                    if dist < 0.8 :
                        state = ROTATE



                # command velocity
                if active and state == MOVE:
                    turtle.cmd_velocity(linear=linear_vel)
                    direction = 1

                elif active and state == ROTATE:
                    if direction is None:
                        direction = np.sign(np.random.rand() - 0.5)

                    turtle.cmd_velocity(angular=direction * angular_vel)


        # End of the random walking
            # get RGB
            img_rgb = turtle.get_rgb_image()

            # get point cloud
            pc = turtle.get_point_cloud()

            # get odometry
            b = turtle.get_odometry()
            a = [b[1], b[0], b[2]]
            odometry = np.dot(a, [[-1, 0, 0],
                                         [0, 1, 0],
                                         [0, 0, 1]])

            markers = np.array(detect_markers(img_rgb))
            # draw markers in the image

            detector.draw_markers(img_rgb, markers)
            # show image
            cv2.imshow("markers", img_rgb)
            cv2.waitKey(1)
            # Odometry
            # print(turtle.get_odometry())
            if len(markers)!=0:
                id = markers[0, 1]
                if id not in id_array and int(id) in range(5, 17):



                    # Print depth
                    # plt.figure(2)
                    # plt.title('depth')
                    # plt.imshow(depth)
                    # plt.show(clear)


                    # find coordinates
                    # # get odometry
                    # odometry = turtle.get_odometry()
                    # odometry = [odometry[1], odometry[0], odometry[2]]
                    # odometry = np.dot(odometry, [[-1, 0, 0],
                    #                              [0, 1, 0],
                    #                              [0, 0, 1]])
                    coordinates_beacon = find_pozition_beacon(pc,markers, odometry)
                    if coordinates_beacon!=-1:

                        id_array.append(id)
                        cord_array.append([coordinates_beacon[0], coordinates_beacon[1]])

                        if len(id_array) == 1:
                            print ("found first beacon ")
                            Found_first_beacon = True

                            # play sound
                            turtle.play_sound(sound_id=3)

                        if len(id_array) == 2:
                            Found_second_beacon = True

                            # play sound
                            turtle.play_sound(sound_id=3)

                        if len(id_array) == 3:
                            Found_second_beacon = True

                            # play sound
                            turtle.play_sound(sound_id=3)

                        if len(id_array) == 4:
                            Found_second_beacon = True

                            # play sound
                            turtle.play_sound(sound_id=4)

                            return 1



        for i in  range (len(id_array)):
            print("Beacon ", id_array[i], cord_array[i][0], -cord_array[i][1])






if __name__ == "__main__":
    id_array = []
    cord_array = []
    main()
    for i in  range (len(id_array)):
        print("Beacon ", id_array[i], cord_array[i][0], -cord_array[i][1])


