import math
def reward_function(params):
    
    track_width = params['track_width']
    distance_from_center = params['distance_from_center']
    steering = abs(params['steering_angle'])
    direction_stearing=params['steering_angle']
    speed = params['speed']
    steps = params['steps']
    progress = params['progress']
    all_wheels_on_track = params['all_wheels_on_track']
    ABS_STEERING_THRESHOLD = 15
    SPEED_TRESHOLD = 5
    TOTAL_NUM_STEPS = 85
    
    waypoints = params['waypoints']
    closest_waypoints = params['closest_waypoints']
    heading = params['heading']
    
    reward = 1.0
        
    if progress == 100:
        reward += 100
    
    next_point = waypoints[closest_waypoints[1]]
    prev_point = waypoints[closest_waypoints[0]]
    
    track_direction = math.atan2(next_point[1] - prev_point[1], next_point[0] - prev_point[0]) 
    
    track_direction = math.degrees(track_direction)
    
    direction_diff = abs(track_direction - heading)
    
    DIRECTION_THRESHOLD = 10.0
    
    malus=1
    
    if direction_diff > DIRECTION_THRESHOLD:
        malus=1-(direction_diff/50)
        if malus<0 or malus>1:
            malus = 0
        reward *= malus
    
    return reward