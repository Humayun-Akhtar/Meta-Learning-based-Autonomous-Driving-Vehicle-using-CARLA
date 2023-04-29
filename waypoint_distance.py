import glob
import os
import sys
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla
import random 
import time
import csv

actorList = []
t_iteration = 30
# def record_waypoint(agent, map, wp, time):
    

try:
    client = carla.Client('localhost', 2000)
    client.set_timeout(15.0)
    world = client.get_world()
    blueprint_library = world.get_blueprint_library()

    bp = blueprint_library.filter("wrangler_rubicon")[0]
    print(bp)
    agent_spawn_point = carla.Transform(carla.Location(x=-23.6,y=137.5,z=1),carla.Rotation(yaw=0))
    agent = world.spawn_actor( bp,agent_spawn_point)
    agent.set_autopilot(True)
    actorList.append(agent)

    ## Camera sensor 
    # cam_bp = blueprint_library.find("sensor.camera.rgb")
    # cam_bp.set_attribute("image_size_x", f"{IM_WIDTH}")
    # cam_bp.set_attribute("image_size_x", f"{IM_HEIGHT}")
    # cam_bp.set_attribute("fov", "110")
    # cam_spawn_point = carla.Transform(carla.Location(x=2.5, z=0.7))
    # sevehiclensor = world.spawn_actor(cam_bp,cam_spawn_point, attach_to=agent)
    # actorList(sensor)

    ## GPS Sensor
    
    collision_sensor_bp = blueprint_library.find('sensor.other.collision')
    map = world.get_map()
    sp = map.get_spawn_points()
    t_end = time.time() + t_iteration
    waypoints_array = []
    i = 0
    while True:
        waypoint = map.get_waypoint(agent.get_location())
        # record_waypoint(agent, map, waypoint, 0.5)
        w = []
        wp = waypoint
        X,Y, Yaw = wp.transform.location.x, wp.transform.location.y, wp.transform.rotation.yaw
        velocity = agent.get_velocity()
        vx, vy  = velocity.x, velocity.y
        if i==0:
            w.append((vx**2+vy**2)**0.5)
            w.append(Yaw)
            w.append(X)
            w.append(Y)
            print(w)
            waypoints_array.append(w)
            i+=1
        current_loccation = agent.get_location()
        prev_point = carla.Location(x=waypoints_array[i-1][2], y=waypoints_array[i-1][3])
        distance = current_loccation.distance(prev_point)
        # print(distance)
        if distance >=1:
            distance = 0
            w.append((vx**2+vy**2)**0.5)
            w.append(Yaw)
            w.append(X)
            w.append(Y)
            print(w)
            waypoints_array.append(w)
            i+=1


    

finally:
    states = []
    l = len(waypoints_array)
    for i in range(l-1):
        s = []
        s.append(waypoints_array[i][0])
        s.append(waypoints_array[i][1])
        s.append(waypoints_array[i+1][1])
        d = ((waypoints_array[i+1][3] - waypoints_array[i][3])**2 + (waypoints_array[i+1][2] - waypoints_array[i][2])**2)**0.5
        s.append(d)
        states.append(s)

    with open('waypoint.csv', 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerows(states)
    # file.close()
    for actor in actorList:
        actor.destroy()
    print('All Cleaned Up')

