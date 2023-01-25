import numpy as np


# todo: fault types - noise, spikes, packet drop, bit_flip, freeze


class Fault_injector:
    """Module to inject fault ito robotic system
    Args:
        Node (rclpy.node): rclpy node
    """
    def __init__(self):
        pass

    def noise(self, data, mean, sd, samples):
        '''
        callback function to return a array of random values

        '''
        self.noise_val = np.random.normal(data,sd,samples)
        return self.noise_val
    def offset(self, data):
        '''
        callback function to return offset value
        '''
        return data