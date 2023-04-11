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
actorList =[]

try:
    # connect to the CARLA server
    client = carla.Client("localhost", 2000)
    client.set_timeout(2.0)

    # create a simulation world
    world = client.get_world()

    # create a blueprint for the car
    car_blueprint = world.get_blueprint_library().filter("wrangler_rubicon")[0]
    # print(type(carla.LaneInvasionEvent))

    start_waypoint = carla.Transform(carla.Location(x=-30.6,y=137.5,z=1),carla.Rotation(yaw=0))
    vehicle = world.spawn_actor(car_blueprint, start_waypoint)
    # laneInvader = world.get_blueprint_library().find('sensor.other.lane_invasion')
    
    actorList.append(vehicle)

    # spawn the car at the first waypoint
    # start_waypoint = world.get_map().get_waypoint(carla.Location(x=-23.597570419311523, y=137.0661163330078))
    # start_transform = start_waypoint.transform
    # vehicle = world.spawn_actor(car_blueprint, start_transform)

    # get the second waypoint
    end_waypoint = world.get_map().get_waypoint(carla.Location(x=-10.597314834594727, y=137.0661163330078, z=1))#,carla.Rotation(yaw=0))
    print(end_waypoint)
    # get the initial state
    start_location = start_waypoint.location
    start_rotation = start_waypoint.rotation
    state = (start_location, start_rotation)
    # define the action space
    actions = ['accelerate', 'brake', 'steer_left', 'steer_right']
    # define the reward function
    def get_reward(state, action, next_state, goal):
        # compute the distance between the car and the goal
        distance = carla.Location(state[0]).distance(carla.Location(goal))

        # compute the distance to the goal in the next state
        next_distance = carla.Location(next_state[0]).distance(carla.Location(goal))

        # compute the reward as the change in distance
        reward = distance - next_distance

        # add a penalty for taking actions that cause the car to crash
        if reward < 0:
            reward -= 10.0

        return reward
    # initialize the Q table
    Q = {}

    # set the hyperparameters
    alpha = 0.1  # learning rate
    gamma = 0.9  # discount factor
    epsilon = 0.1  # exploration rate

    # initialize the state and action
    state = (start_location, start_rotation)
    action = None
    import random

    # train the agent
    for episode in range(2000):
        # set the initial state and action
        state = (start_location, start_rotation)
        action = None
        done = False

        while not done:
            # choose the next action
            if random.uniform(0, 1) < epsilon:
                # explore
                action = random.choice(actions)
            else:
                # exploit
                if state in Q and sum(Q[state].values()) != 0:
                    action = max(Q[state], key=Q[state].get)
                else:
                    action = random.choice(actions)

            # execute the action
            if action == 'accelerate':
                vehicle.apply_control(carla.VehicleControl(throttle=1.0, brake=0.0, steer=0.0))
            elif action == 'brake':
                vehicle.apply_control(carla.VehicleControl(throttle=0.0, brake=1.0, steer=0.0))
            elif action == 'steer_left':
                vehicle.apply_control(carla.VehicleControl(throttle=1.0, brake=0.0, steer=-0.5))
            elif action == 'steer_right':
                vehicle.apply_control(carla.VehicleControl(throttle=1.0, brake=0.0, steer=0.5))

            # get the next state and reward
            next_location = vehicle.get_location()
            next_rotation = vehicle.get_transform().rotation
            next_state = (next_location, next_rotation)
            reward = get_reward(state, action, next_state, end_waypoint.transform.location)

            print(f"Episode {episode} finished with reward = {reward}\n ")

            # choose the next action
            if random.uniform(0, 1) < epsilon:
                # explore
                next_action = random.choice(actions)
            else:
                # exploit
                if next_state in Q and sum(Q[next_state].values()) != 0:
                    next_action = max(Q[next_state], key=Q[next_state].get)
                else:
                    next_action = random.choice(actions)

            # update the Q value
            if state not in Q:
                Q[state] = {}
            if action not in Q[state]:
                Q[state][action] = 0.0
            if next_state not in Q:
                Q[next_state] = {}
            if next_action not in Q[next_state]:
                Q[next_state][next_action] = 0.0
            Q[state][action] += alpha * (reward + gamma * Q[next_state][next_action] - Q[state][action])

            # update the state and action
            state = next_state
            action = next_action

            # check if the goal has been reached
            if next_location.distance(end_waypoint.transform.location) < 2.0:
                done = True
                print(f"Episode {episode} finished after {episode} steps.\n ")
                break
finally:
    # file.close()
    for actor in actorList:
        actor.destroy()
    print('All Cleaned Up')