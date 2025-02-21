import gymnasium as gym
from gymnasium import spaces
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Wrench

class CartPoleRLEnv(gym.Env):
    def __init__(self):
        super(CartPoleRLEnv, self).__init__()
        
        # Define action and observation spaces
        self.action_space = spaces.Discrete(3)  # 3 actions: left, neutral, right
        self.observation_space = spaces.Box(
            low=np.array([-2.5, -np.inf, -np.pi, -np.inf]),
            high=np.array([2.5, np.inf, np.pi, np.inf]),
            dtype=np.float32
        )
        
        # Initialize ROS2 node
        rclpy.init()
        self.node = Node('cart_pole_rl_env')
        
        # Publishers and Subscribers
        self.force_pub = self.node.create_publisher(Wrench, '/cart_force', 10)
        self.state_sub = self.node.create_subscription(
            Float64MultiArray,
            '/cart_pole_states',
            self._state_callback,
            10
        )
        
        # Initialize state variables
        self.state = np.zeros(4, dtype=np.float32)
        self.reward = 0.0
        self.done = False

    def _state_callback(self, msg):
        # Update state from simulation
        self.state = np.array(msg.data, dtype=np.float32)

    def step(self, action):
        # Convert action to force
        force = 0.0
        if action == 0:
            force = -20.0  # Left
        elif action == 1:
            force = 0.0    # Neutral
        elif action == 2:
            force = 20.0   # Right
        
        # Publish force command
        wrench_msg = Wrench()
        wrench_msg.force.x = force
        self.force_pub.publish(wrench_msg)
        
        # Calculate reward
        x, x_dot, theta, theta_dot = self.state
        reward = 10 - (x**2 + theta**2 + 0.1*x_dot**2 + 0.1*theta_dot**2)
        
        # Check termination
        if abs(x) > 2.5 or abs(theta) > 0.5:
            self.done = True
        
        return self.state, reward, self.done, {}

    def reset(self):
        # Reset simulation (you may need a service call here)
        self.done = False
        self.state = np.zeros(4, dtype=np.float32)
        return self.state

    def close(self):
        rclpy.shutdown()