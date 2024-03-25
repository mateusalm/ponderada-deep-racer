import math

class Reward:
    def __init__(self, verbose=False, track_time=False):
        self.prev_steering_angle = 0
        
    def reward_function(self, params):
        prev_steering_angle = self.prev_steering_angle
        steering_angle = params['steering_angle']
        self.prev_steering_angle = steering_angle
        steering_diff = abs(steering_angle - prev_steering_angle)
        reward_steering_smoothness = math.exp(-0.5 * steering_diff)

        track_width = params['track_width']
        distance_from_center = params['distance_from_center']
        reward_distance_from_center = 1.0 - (distance_from_center / (0.5 * track_width))
        
        weight_smoothness = 0.6
        weight_distance = 0.4
        
        weighted_reward = (weight_smoothness * reward_steering_smoothness) + (weight_distance * reward_distance_from_center)

        return weighted_reward

reward_obj = Reward()

def reward_function(params):
    reward = reward_obj.reward_function(params)
    return float(reward)
