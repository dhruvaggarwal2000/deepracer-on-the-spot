import numpy as np
import math

turn_waypoints = None
turn_waypoints_map = None

def angle_between(v1, v2):
    # Calculate the angle between two vectors
    angle = np.arctan2(v2[1], v2[0]) - np.arctan2(v1[1], v1[0])
    return np.degrees(angle)

def find_turns(coords):
    turns = []
    for i in range(1, len(coords) - 1):
        p0 = coords[i - 1]
        p1 = coords[i]
        p2 = coords[i + 1]
        
        # Vectors from p0 to p1 and p1 to p2
        v1 = (p1[0] - p0[0], p1[1] - p0[1])
        v2 = (p2[0] - p1[0], p2[1] - p1[1])
        
        # Calculate the angle between the vectors
        angle = angle_between(v1, v2)
        
        # Normalize the angle to the range [-180, 180]
        angle = (angle + 180) % 360 - 180
        
        # Define threshold for detecting turns
        if abs(angle) >= 5:  # threshold for a significant turn
            turns.append(i) 
    return turns

def calculate_heading(p1, p2):
    p0 = (p1[0] + 1, p1[1])

    # Vectors from p1 to p0 and p1 to p2
    v1 = (p0[0] - p1[0], p0[1] - p1[1])
    v2 = (p2[0] - p1[0], p2[1] - p1[1])
    
    # Calculate the angle between the vectors
    return angle_between(v1, v2)

def identify_u_turns(turn_waypoints, waypoint, waypoint_heading):
    if (waypoint in turn_waypoints and waypoint_heading < 0):
        return True
    else:
        return False

def get_waypoints_for_all_turn_types(turn_waypoints, track_coords):
    turn_waypoints_dict = {
        'right': [],
        'left': [],
        'straight': []
    }
    for i in turn_waypoints:
        p0 = track_coords[i-1]
        p1 = track_coords[i % len(track_coords)]
        p2 = track_coords[i % len(track_coords) + 1]
        heading_1 = calculate_heading(p0, p1) % 360
        heading_2 = calculate_heading(p1, p2) % 360

        if(heading_1 > heading_2): # Right turn
            turn_waypoints_dict['right'] +=[i]
        elif(heading_1 < heading_2): # Left turn
            turn_waypoints_dict['left'] +=[i]
        else: # No turn
            turn_waypoints_dict['straight'] +=[i]

    return turn_waypoints_dict

def get_coords_based_on_angle(x, y, angle, distance):
    x_value = float(x + np.cos([np.deg2rad(angle)]) * distance)
    y_value = float(y + np.sin([np.deg2rad(angle)]) * distance)
    return (x_value, y_value)

def get_heading_angle(first_point, second_point):
    p0 = (first_point[0] + 1, first_point[1])
    p1 = first_point
    p2 = second_point
        
    # Vectors from p0 to p1 and p1 to p2
    v1 = (p0[0] - p1[0], p0[1] - p1[1])
    v2 = (p2[0] - p1[0], p2[1] - p1[1])
        
    # Calculate the angle between the vectors
    angle = angle_between(v1, v2)

    angle_360 = angle % 360

    return angle_360

def get_heading_angle_normalized(first_point, second_point):
    p0 = (first_point[0] + 1, first_point[1])
    p1 = first_point
    p2 = second_point
        
    # Vectors from p0 to p1 and p1 to p2
    v1 = (p0[0] - p1[0], p0[1] - p1[1])
    v2 = (p2[0] - p1[0], p2[1] - p1[1])
        
    # Calculate the angle between the vectors
    angle = angle_between(v1, v2)

    return angle

def get_next_turn_waypoint_turn_type(waypoint, turn_waypoints, turn_waypoints_map):
    turn_waypoints.sort()
    next_turn_waypoint = 0
    prev_turn_waypoint = 0
    next_turn_type = ''
    prev_turn_type = ''
    for i in turn_waypoints:
        if(waypoint < i):
            next_turn_waypoint = i
            break
        prev_turn_waypoint = i
    if (next_turn_waypoint in turn_waypoints_map['right']):
        next_turn_type = 'right'
    else:
        next_turn_type = 'left'
    if (prev_turn_waypoint in turn_waypoints_map['right']):
        prev_turn_type = 'right'
    else:
        prev_turn_type = 'left'
    return next_turn_waypoint, prev_turn_waypoint, next_turn_type, prev_turn_type

def get_coords_on_center_line(x: float, y: float, heading: float, distance_from_center: float, is_left_of_center: bool):
    angle = heading
    if (is_left_of_center):
        angle -= 90
    else:
        angle += 90
    
    return get_coords_based_on_angle(x, y, angle, distance_from_center)  

def get_turn_end_waypoints(waypoint: int):
    forward = 0
    backward = 0
    while (waypoint + forward in turn_waypoints):
        forward += 1
    while (waypoint + backward in turn_waypoints):
        backward -= 1

    return (waypoint + backward + 1, waypoint + forward - 1)

def percentage_distance_covered_on_turn(waypoint, car_position, track_coords):
    first_waypoint, last_waypoint = get_turn_end_waypoints(waypoint)
    p1 = track_coords[first_waypoint - 1]
    p2 = track_coords[last_waypoint - 1]
    p3 = car_position
    distance = calculate_distance_between_points(p1, p2)
    car_distance = calculate_distance_between_points(p2, p3)
    return 1 - float (car_distance / distance)

def calculate_distance_between_points(point_1, point_2):
    x1, y1 = point_1
    x2, y2 = point_2
    return math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)

def get_optimal_path_based_on_params(params):
    x = params['x']
    y = params['y']
    distance_from_center = params['distance_from_center']
    is_left_of_center = params['is_left_of_center']
    closest_waypoints = params['closest_waypoints']
    prev_closest_point = closest_waypoints[0]
    next_closest_point = closest_waypoints[1]
    waypoints_coordinates = params['waypoints']
    track_width = params['track_width']
    waypoints_heading = get_heading_angle(waypoints_coordinates[prev_closest_point-1], waypoints_coordinates[next_closest_point-1])
    waypoints_heading_normalized = get_heading_angle_normalized(waypoints_coordinates[prev_closest_point-1], waypoints_coordinates[next_closest_point-1])

    center_line_coords = get_coords_on_center_line(x, y, waypoints_heading, distance_from_center, is_left_of_center)
    next_turn_waypoint_next_waypoint, prev_turn_waypoint_next_waypoint, next_turn_type_next_waypoint, prev_turn_type_next_waypoint = get_next_turn_waypoint_turn_type(next_closest_point, turn_waypoints, turn_waypoints_map)
    next_turn_waypoint_prev_waypoint, prev_turn_waypoint_prev_waypoint, next_turn_type_prev_waypoint, prev_turn_type_prev_waypoint = get_next_turn_waypoint_turn_type(prev_closest_point, turn_waypoints, turn_waypoints_map)
    
    is_straight_road = False
    is_switching_side = False

    if prev_turn_waypoint_prev_waypoint == 0 or next_turn_waypoint_next_waypoint == 0:
        optimal_path_coords = get_coords_based_on_angle(center_line_coords[0], center_line_coords[1], waypoints_heading + 90, track_width / 2)
    elif (prev_closest_point in turn_waypoints and not identify_u_turns(turn_waypoints, next_closest_point, waypoints_heading_normalized) and waypoints_heading_normalized > 90):
        if next_turn_type_next_waypoint == 'left':
            optimal_path_coords = get_coords_based_on_angle(center_line_coords[0], center_line_coords[1], waypoints_heading + 90, track_width / 2)
    elif (prev_closest_point in turn_waypoints and not identify_u_turns(turn_waypoints, next_closest_point, waypoints_heading_normalized) and waypoints_heading_normalized > 0):
        if next_turn_type_next_waypoint == 'left':
            optimal_path_coords = get_coords_based_on_angle(center_line_coords[0], center_line_coords[1], waypoints_heading + 90, track_width / 2)
    elif prev_turn_type_next_waypoint == next_turn_type_next_waypoint and next_turn_type_next_waypoint == 'left':
        optimal_path_coords = get_coords_based_on_angle(center_line_coords[0], center_line_coords[1], waypoints_heading + 90, track_width / 2)
    elif prev_turn_type_next_waypoint == next_turn_type_next_waypoint and next_turn_type_next_waypoint == 'right':
        optimal_path_coords = get_coords_based_on_angle(center_line_coords[0], center_line_coords[1], waypoints_heading - 90, track_width / 2)
    else:
        next_turn_coords = waypoints_coordinates[next_turn_waypoint_next_waypoint - 1]
        prev_turn_coords = waypoints_coordinates[prev_turn_waypoint_next_waypoint - 1]
        total_distance = calculate_distance_between_points(prev_turn_coords, next_turn_coords)
        current_distance = calculate_distance_between_points(center_line_coords, next_turn_coords)
        reqd_distance = current_distance / total_distance * track_width
        angle = waypoints_heading
        reqd_distance_from_center = reqd_distance
        if next_turn_type_next_waypoint == 'right':
            if reqd_distance > track_width / 2:
                angle = waypoints_heading - 90
                reqd_distance_from_center = track_width / 2 - reqd_distance
            else:
                angle = waypoints_heading + 90
                reqd_distance_from_center = reqd_distance - track_width / 2
        else:
            if reqd_distance > track_width / 2:
                angle = waypoints_heading - 90
                reqd_distance_from_center = reqd_distance - track_width / 2 
            else:
                angle = waypoints_heading + 90
                reqd_distance_from_center = track_width / 2 - reqd_distance
        optimal_path_coords = get_coords_based_on_angle(center_line_coords[0], center_line_coords[1], angle, reqd_distance_from_center)
        is_switching_side = True
    
    if not next_closest_point in turn_waypoints:
        is_straight_road = True
    
    return optimal_path_coords, is_straight_road, is_switching_side

def reward_function(params):
    global turn_waypoints, turn_waypoints_map
    waypoints_coordinates = params['waypoints']
    speed = params['speed']
    if turn_waypoints == None:
        turn_waypoints = find_turns(waypoints_coordinates)
    if turn_waypoints_map == None:
        turn_waypoints_map = get_waypoints_for_all_turn_types(turn_waypoints, waypoints_coordinates)
    car_x = params['x']
    car_y = params['y']
    optimal_path_coords, is_straight_road, is_switching_side = get_optimal_path_based_on_params(params)
    distance = calculate_distance_between_points(optimal_path_coords, (car_x, car_y))
    if distance <= params['track_width']/2:
        reward = 1.0
    else:
        reward = 1e-3

    speed_reward = 0
    if not is_straight_road:
        if speed < 1.0:
            speed_reward = 0.5
        else:
            speed_reward = 1.0
    else:
        speed_reward = speed

    reward *= speed_reward
    
    if is_straight_road and not is_switching_side and abs(params['steering_angle']) > 5.0:
        reward = 1e-3
    
    if params['is_offtrack']:
        reward = 1e-3
        
    print(params)
    print('reward: ', reward)
    
    return float(reward)