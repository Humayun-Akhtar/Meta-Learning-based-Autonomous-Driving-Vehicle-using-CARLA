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
    while time.time()<t_end:
        waypoint = map.get_waypoint(agent.get_location())
        # record_waypoint(agent, map, waypoint, 0.5)
        w = []
        wp = waypoint
        X,Y, Yaw = wp.transform.location.x, wp.transform.location.y, wp.transform.rotation.yaw
        w.append(X)
        w.append(Y)
        w.append(Yaw)
        print(w)
        waypoints_array.append(w)
        time.sleep(0.5)     

    with open('waypoint.csv', 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerows(waypoints_array)

finally:
    # file.close()
    for actor in actorList:
        actor.destroy()
    print('All Cleaned Up')