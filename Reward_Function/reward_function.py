import math


class Reward:
    def __init__(self, verbose=False):
        self.first_racingpoint_index = None
        self.verbose = verbose

    def reward_function(self, params):

        # Import package (needed for heading)
        import math

        ################## HELPER FUNCTIONS ###################

        def dist_2_points(x1, x2, y1, y2):
            return abs(abs(x1-x2)**2 + abs(y1-y2)**2)**0.5

        def closest_2_racing_points_index(racing_coords, car_coords):

            # Calculate all distances to racing points
            distances = []
            for i in range(len(racing_coords)):
                distance = dist_2_points(x1=racing_coords[i][0], x2=car_coords[0],
                                         y1=racing_coords[i][1], y2=car_coords[1])
                distances.append(distance)

            # Get index of the closest racing point
            closest_index = distances.index(min(distances))

            # Get index of the second closest racing point
            distances_no_closest = distances.copy()
            distances_no_closest[closest_index] = 999
            second_closest_index = distances_no_closest.index(
                min(distances_no_closest))

            return [closest_index, second_closest_index]

        def dist_to_racing_line(closest_coords, second_closest_coords, car_coords):
            
            # Calculate the distances between 2 closest racing points
            a = abs(dist_2_points(x1=closest_coords[0],
                                  x2=second_closest_coords[0],
                                  y1=closest_coords[1],
                                  y2=second_closest_coords[1]))

            # Distances between car and closest and second closest racing point
            b = abs(dist_2_points(x1=car_coords[0],
                                  x2=closest_coords[0],
                                  y1=car_coords[1],
                                  y2=closest_coords[1]))
            c = abs(dist_2_points(x1=car_coords[0],
                                  x2=second_closest_coords[0],
                                  y1=car_coords[1],
                                  y2=second_closest_coords[1]))

            # Calculate distance between car and racing line (goes through 2 closest racing points)
            # try-except in case a=0 (rare bug in DeepRacer)
            try:
                distance = abs(-(a**4) + 2*(a**2)*(b**2) + 2*(a**2)*(c**2) -
                               (b**4) + 2*(b**2)*(c**2) - (c**4))**0.5 / (2*a)
            except:
                distance = b

            return distance

        # Calculate which one of the closest racing points is the next one and which one the previous one
        def next_prev_racing_point(closest_coords, second_closest_coords, car_coords, heading):

            # Virtually set the car more into the heading direction
            heading_vector = [math.cos(math.radians(
                heading)), math.sin(math.radians(heading))]
            new_car_coords = [car_coords[0]+heading_vector[0],
                              car_coords[1]+heading_vector[1]]

            # Calculate distance from new car coords to 2 closest racing points
            distance_closest_coords_new = dist_2_points(x1=new_car_coords[0],
                                                        x2=closest_coords[0],
                                                        y1=new_car_coords[1],
                                                        y2=closest_coords[1])
            distance_second_closest_coords_new = dist_2_points(x1=new_car_coords[0],
                                                               x2=second_closest_coords[0],
                                                               y1=new_car_coords[1],
                                                               y2=second_closest_coords[1])

            if distance_closest_coords_new <= distance_second_closest_coords_new:
                next_point_coords = closest_coords
                prev_point_coords = second_closest_coords
            else:
                next_point_coords = second_closest_coords
                prev_point_coords = closest_coords

            return [next_point_coords, prev_point_coords]

        def racing_direction_diff(closest_coords, second_closest_coords, car_coords, heading):

            # Calculate the direction of the center line based on the closest waypoints
            next_point, prev_point = next_prev_racing_point(closest_coords,
                                                            second_closest_coords,
                                                            car_coords,
                                                            heading)

            # Calculate the direction in radius, arctan2(dy, dx), the result is (-pi, pi) in radians
            track_direction = math.atan2(
                next_point[1] - prev_point[1], next_point[0] - prev_point[0])

            # Convert to degree
            track_direction = math.degrees(track_direction)

            # Calculate the difference between the track direction and the heading direction of the car
            direction_diff = abs(track_direction - heading)
            if direction_diff > 180:
                direction_diff = 360 - direction_diff

            return direction_diff

        # Gives back indexes that lie between start and end index of a cyclical list 
        # (start index is included, end index is not)
        def indexes_cyclical(start, end, array_len):

            if end < start:
                end += array_len

            return [index % array_len for index in range(start, end)]

        # Calculate how long car would take for entire lap, if it continued like it did until now
        def projected_time(first_index, closest_index, step_count, times_list):

            # Calculate how much time has passed since start
            current_actual_time = (step_count-1) / 15

            # Calculate which indexes were already passed
            indexes_traveled = indexes_cyclical(first_index, closest_index, len(times_list))

            # Calculate how much time should have passed if car would have followed optimals
            current_expected_time = sum([times_list[i] for i in indexes_traveled])

            # Calculate how long one entire lap takes if car follows optimals
            total_expected_time = sum(times_list)

            # Calculate how long car would take for entire lap, if it continued like it did until now
            try:
                projected_time = (current_actual_time/current_expected_time) * total_expected_time
            except:
                projected_time = 9999

            return projected_time

        #################### RACING LINE ######################

        # Optimal racing line for the Spain track
        # Each row: [x,y,speed,timeFromPreviousPoint]
        racing_track = [[3.06161, 0.68677, 3.74442, 0.0393],
                        [3.21001, 0.68377, 4.0, 0.03711],
                        [3.35928, 0.68294, 4.0, 0.03732],
                        [3.50903, 0.68319, 4.0, 0.03744],
                        [3.65879, 0.68342, 4.0, 0.03744],
                        [3.80856, 0.68352, 4.0, 0.03744],
                        [3.95832, 0.68357, 4.0, 0.03744],
                        [4.10808, 0.68362, 4.0, 0.03744],
                        [4.25783, 0.68367, 4.0, 0.03744],
                        [4.40759, 0.68373, 4.0, 0.03744],
                        [4.55735, 0.68378, 4.0, 0.03744],
                        [4.70711, 0.68384, 4.0, 0.03744],
                        [4.85687, 0.68389, 4.0, 0.03744],
                        [5.00663, 0.68395, 4.0, 0.03744],
                        [5.15639, 0.684, 4.0, 0.03744],
                        [5.30615, 0.68405, 3.22898, 0.04638],
                        [5.45591, 0.68412, 2.62295, 0.0571],
                        [5.6052, 0.68637, 2.27014, 0.06577],
                        [5.75336, 0.69298, 2.0277, 0.07314],
                        [5.89948, 0.70601, 1.85457, 0.0791],
                        [6.04243, 0.72725, 1.72178, 0.08394],
                        [6.18086, 0.75815, 1.61603, 0.08777],
                        [6.31322, 0.79969, 1.5308, 0.09063],
                        [6.4379, 0.85238, 1.46666, 0.09228],
                        [6.55319, 0.91623, 1.41392, 0.09321],
                        [6.65747, 0.99078, 1.37672, 0.09311],
                        [6.74935, 1.07509, 1.34399, 0.09278],
                        [6.82764, 1.16789, 1.31877, 0.09207],
                        [6.89156, 1.26768, 1.30247, 0.09098],
                        [6.94047, 1.37287, 1.3, 0.08924],
                        [6.97395, 1.48184, 1.3, 0.08769],
                        [6.99186, 1.59294, 1.3, 0.08656],
                        [6.99436, 1.70456, 1.30355, 0.08565],
                        [6.98174, 1.81521, 1.31429, 0.08474],
                        [6.95439, 1.9235, 1.33542, 0.08363],
                        [6.91288, 2.02816, 1.3653, 0.08247],
                        [6.85783, 2.12806, 1.40448, 0.08121],
                        [6.78999, 2.22221, 1.456, 0.0797],
                        [6.71017, 2.30979, 1.52408, 0.07775],
                        [6.61937, 2.39021, 1.61381, 0.07516],
                        [6.5187, 2.46318, 1.73008, 0.07187],
                        [6.40937, 2.52873, 1.88377, 0.06767],
                        [6.29267, 2.58724, 2.09798, 0.06222],
                        [6.16999, 2.6395, 2.39147, 0.05576],
                        [6.04267, 2.68656, 2.87798, 0.04716],
                        [5.91212, 2.72979, 3.86678, 0.03556],
                        [5.77978, 2.77083, 4.0, 0.03464],
                        [5.64544, 2.8119, 4.0, 0.03512],
                        [5.51145, 2.85376, 3.82962, 0.03665],
                        [5.3781, 2.89701, 3.44945, 0.04064],
                        [5.24564, 2.94221, 3.16583, 0.04421],
                        [5.11433, 2.98994, 2.93187, 0.04765],
                        [4.98444, 3.04075, 2.73886, 0.05092],
                        [4.85625, 3.0952, 2.73886, 0.05085],
                        [4.73004, 3.15388, 2.73886, 0.05082],
                        [4.60613, 3.2174, 3.07891, 0.04522],
                        [4.48407, 3.28473, 3.27297, 0.04259],
                        [4.36368, 3.35543, 3.50432, 0.03984],
                        [4.24478, 3.42909, 3.78966, 0.03691],
                        [4.12717, 3.50527, 4.0, 0.03503],
                        [4.01066, 3.58355, 4.0, 0.03509],
                        [3.89505, 3.66352, 4.0, 0.03514],
                        [3.78013, 3.74475, 4.0, 0.03518],
                        [3.66567, 3.82685, 3.39066, 0.04154],
                        [3.5516, 3.90725, 2.94402, 0.0474],
                        [3.43648, 3.98557, 2.64259, 0.05269],
                        [3.31978, 4.06075, 2.4331, 0.05706],
                        [3.20102, 4.13181, 2.27046, 0.06095],
                        [3.07977, 4.19777, 2.14382, 0.06439],
                        [2.95566, 4.25772, 2.04227, 0.06749],
                        [2.82843, 4.31071, 1.9623, 0.07024],
                        [2.69794, 4.35576, 1.90059, 0.07264],
                        [2.56428, 4.39187, 1.84138, 0.07519],
                        [2.42787, 4.41799, 1.79324, 0.07745],
                        [2.28961, 4.4332, 1.75064, 0.07946],
                        [2.15081, 4.43663, 1.71257, 0.08107],
                        [2.01321, 4.42775, 1.67644, 0.08225],
                        [1.87874, 4.40646, 1.64606, 0.08271],
                        [1.74917, 4.37305, 1.61588, 0.08281],
                        [1.62598, 4.32809, 1.58355, 0.08282],
                        [1.51021, 4.27228, 1.55218, 0.0828],
                        [1.40264, 4.20629, 1.51941, 0.08305],
                        [1.30393, 4.13066, 1.48404, 0.0838],
                        [1.21469, 4.04583, 1.48404, 0.08297],
                        [1.13567, 3.9521, 1.48404, 0.0826],
                        [1.06785, 3.84971, 1.55732, 0.07886],
                        [1.01074, 3.73994, 1.66848, 0.07416],
                        [0.96348, 3.62402, 1.73579, 0.07212],
                        [0.92587, 3.50262, 1.81147, 0.07016],
                        [0.89769, 3.37633, 1.89223, 0.06838],
                        [0.87876, 3.24567, 1.98315, 0.06657],
                        [0.86884, 3.11115, 2.08712, 0.06463],
                        [0.86762, 2.97328, 2.20764, 0.06245],
                        [0.87473, 2.83258, 2.34568, 0.06006],
                        [0.88965, 2.68954, 2.50627, 0.05738],
                        [0.91182, 2.54468, 2.70364, 0.0542],
                        [0.94049, 2.39854, 2.64716, 0.05626],
                        [0.97426, 2.2539, 2.46579, 0.06024],
                        [1.01276, 2.11285, 2.31619, 0.06313],
                        [1.0565, 1.97596, 2.18996, 0.06562],
                        [1.10591, 1.84381, 2.08102, 0.0678],
                        [1.16134, 1.71689, 1.98561, 0.06975],
                        [1.22306, 1.59567, 1.89765, 0.07168],
                        [1.29132, 1.48063, 1.81782, 0.07359],
                        [1.36632, 1.37224, 1.74344, 0.0756],
                        [1.44827, 1.27102, 1.67216, 0.07788],
                        [1.5374, 1.17759, 1.67216, 0.07722],
                        [1.63399, 1.09269, 1.67216, 0.0769],
                        [1.73834, 1.01727, 1.74997, 0.07357],
                        [1.84956, 0.95083, 1.83011, 0.07079],
                        [1.96696, 0.89297, 1.91824, 0.06823],
                        [2.08992, 0.84331, 2.01835, 0.0657],
                        [2.21789, 0.80149, 2.13402, 0.06309],
                        [2.35034, 0.76708, 2.26868, 0.06032],
                        [2.48675, 0.73963, 2.42981, 0.05727],
                        [2.62661, 0.71857, 2.63071, 0.05376],
                        [2.7694, 0.70326, 2.88942, 0.0497],
                        [2.91457, 0.69295, 3.23562, 0.04498]]

        ################## INPUT PARAMETERS ###################

        # Read all input parameters
        all_wheels_on_track = params['all_wheels_on_track']
        x = params['x']
        y = params['y']
        distance_from_center = params['distance_from_center']
        is_left_of_center = params['is_left_of_center']
        heading = params['heading']
        progress = params['progress']
        steps = params['steps']
        speed = params['speed']
        steering_angle = params['steering_angle']
        track_width = params['track_width']
        waypoints = params['waypoints']
        closest_waypoints = params['closest_waypoints']
        is_offtrack = params['is_offtrack']

        ############### OPTIMAL X,Y,SPEED,TIME ################

        # Get closest indexes for racing line (and distances to all points on racing line)
        closest_index, second_closest_index = closest_2_racing_points_index(
            racing_track, [x, y])

        # Get optimal [x, y, speed, time] for closest and second closest index
        optimals = racing_track[closest_index]
        optimals_second = racing_track[second_closest_index]

        # Save first racingpoint of episode for later
        if self.verbose == True:
            self.first_racingpoint_index = 0 # this is just for testing purposes
        if steps == 1:
            self.first_racingpoint_index = closest_index

        ################ REWARD AND PUNISHMENT ################

        ## Define the default reward ##
        reward = 1

        ## Reward if car goes close to optimal racing line ##
        DISTANCE_MULTIPLE = 1
        dist = dist_to_racing_line(optimals[0:2], optimals_second[0:2], [x, y])
        distance_reward = max(1e-3, 1 - (dist/(track_width*0.5)))
        reward += distance_reward * DISTANCE_MULTIPLE

        ## Reward if speed is close to optimal speed ##
        SPEED_DIFF_NO_REWARD = 1
        SPEED_MULTIPLE = 2
        speed_diff = abs(optimals[2]-speed)
        if speed_diff <= SPEED_DIFF_NO_REWARD:
            # we use quadratic punishment (not linear) bc we're not as confident with the optimal speed
            # so, we do not punish small deviations from optimal speed
            speed_reward = (1 - (speed_diff/(SPEED_DIFF_NO_REWARD))**2)**2
        else:
            speed_reward = 0
        reward += speed_reward * SPEED_MULTIPLE

        # Reward if less steps
        REWARD_PER_STEP_FOR_FASTEST_TIME = 1 
        STANDARD_TIME = 37
        FASTEST_TIME = 27
        times_list = [row[3] for row in racing_track]
        projected_time = projected_time(self.first_racingpoint_index, closest_index, steps, times_list)
        try:
            steps_prediction = projected_time * 15 + 1
            reward_prediction = max(1e-3, (-REWARD_PER_STEP_FOR_FASTEST_TIME*(FASTEST_TIME) /
                                           (STANDARD_TIME-FASTEST_TIME))*(steps_prediction-(STANDARD_TIME*15+1)))
            steps_reward = min(REWARD_PER_STEP_FOR_FASTEST_TIME, reward_prediction / steps_prediction)
        except:
            steps_reward = 0
        reward += steps_reward

        # Zero reward if obviously wrong direction (e.g. spin)
        direction_diff = racing_direction_diff(
            optimals[0:2], optimals_second[0:2], [x, y], heading)
        if direction_diff > 30:
            reward = 1e-3
            
        # Zero reward of obviously too slow
        speed_diff_zero = optimals[2]-speed
        if speed_diff_zero > 0.5:
            reward = 1e-3
            
        ## Incentive for finishing the lap in less steps ##
        REWARD_FOR_FASTEST_TIME = 1500 # should be adapted to track length and other rewards
        STANDARD_TIME = 37  # seconds (time that is easily done by model)
        FASTEST_TIME = 27  # seconds (best time of 1st place on the track)
        if progress == 100:
            finish_reward = max(1e-3, (-REWARD_FOR_FASTEST_TIME /
                      (15*(STANDARD_TIME-FASTEST_TIME)))*(steps-STANDARD_TIME*15))
        else:
            finish_reward = 0
        reward += finish_reward
        
        ## Zero reward if off track ##
        if all_wheels_on_track == False:
            reward = 1e-3

        ####################### VERBOSE #######################
        
        if self.verbose == True:
            print("Closest index: %i" % closest_index)
            print("Distance to racing line: %f" % dist)
            print("=== Distance reward (w/out multiple): %f ===" % (distance_reward))
            print("Optimal speed: %f" % optimals[2])
            print("Speed difference: %f" % speed_diff)
            print("=== Speed reward (w/out multiple): %f ===" % speed_reward)
            print("Direction difference: %f" % direction_diff)
            print("Predicted time: %f" % projected_time)
            print("=== Steps reward: %f ===" % steps_reward)
            print("=== Finish reward: %f ===" % finish_reward)
            
        #################### RETURN REWARD ####################
        
        # Always return a float value
        return float(reward)


reward_object = Reward() # add parameter verbose=True to get noisy output for testing


def reward_function(params):
    return reward_object.reward_function(params)
