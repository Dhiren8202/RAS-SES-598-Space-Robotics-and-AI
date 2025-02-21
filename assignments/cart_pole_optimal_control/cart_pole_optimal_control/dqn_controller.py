import os
from stable_baselines3 import DQN
from .cart_pole_rl_env import CartPoleRLEnv

env = CartPoleRLEnv()
model = DQN(
    "MlpPolicy",
    env,
    verbose=1,
    learning_rate=1e-4,
    buffer_size=10000,
    batch_size=64,
    gamma=0.99,
    exploration_fraction=0.2,
    exploration_initial_eps=1.0,
    exploration_final_eps=0.1,
    tensorboard_log="./dqn_logs/"
)
model.learn(total_timesteps=100000)
model.save("dqn_cartpole")
env.close()