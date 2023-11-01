import gymnasium as gym
import roslibpy
import numpy as np
import time

from ddpg import DDPG
from replay_memory import ReplayMemory, Transition
from noise import OrnsteinUhlenbeckActionNoise

import torch
from tqdm import tqdm


class RLEnv(gym.Env):
  def __init__(self) -> None:
    super().__init__()
    self.action_space = gym.spaces.Box(-1, 1, (14,), np.int8)
    self.observation_space = gym.spaces.Box(-np.inf, np.inf, (17,), np.float32)
    self.ros_state = [0.0 for _ in range(19)]
    self.initial_ros_state = None
    self.__next_state_ready = False
    self.client = roslibpy.Ros('localhost', port=9090)
    self.client.run()
    self.listener = roslibpy.Topic(self.client, '/state', 'std_msgs/Float32MultiArray')
    self.listener.subscribe(self.handle_sim_state)
    self.publisher = roslibpy.Topic(self.client, '/control', 'std_msgs/Int8MultiArray')
    self.reset_publisher = roslibpy.Topic(self.client, '/reset', 'std_msgs/Empty')
  
  def handle_sim_state(self, msg):
    self.ros_state = msg["data"]
    self.__next_state_ready = True
  
  def _get_obs(self):
    while not self.__next_state_ready:
      pass
    self.__next_state_ready = False
    
    return np.array([
      self.ros_state[0], # x position of object
      self.ros_state[1], # y
      self.ros_state[2], # z
      
      # Below are joint positions for robot links
      self.ros_state[5],
      self.ros_state[6],
      self.ros_state[7],
      self.ros_state[8],
      self.ros_state[9],
      self.ros_state[10],
      self.ros_state[11],
      self.ros_state[12],
      self.ros_state[13],
      self.ros_state[14],
      self.ros_state[15],
      self.ros_state[16],
      self.ros_state[17],
      self.ros_state[18],
      ], dtype=np.float32)
    
  def reset(self, seed=None):
    super().reset(seed=seed)
    self.reset_publisher.publish(roslibpy.Message({}))
    time.sleep(1)
    observation = self._get_obs()
    info = {}
    return observation, info
  
  def step(self, action):
    action = action.tolist()
    action = [int(i) for i in action]
    self.publisher.publish(roslibpy.Message({'data': action}))
    self.__next_state_ready = False
    
    observation = self._get_obs()
    terminated = self.ros_state[3] != 0
    reward = self.ros_state[4]

    return observation, reward, terminated, False, {}


if __name__ == '__main__':
  env = RLEnv()
  # check_env(env)
  
  device = "cuda" if torch.cuda.is_available() else "cpu"
  
  model = DDPG(0.99, 
               0.0001, 
               [400, 300], 
               env.observation_space.shape[0], 
               env.action_space,
               checkpoint_dir="rl_agent\checkpoints"
               )
  memory = ReplayMemory(1e6)
  num_actions = env.action_space.shape[-1]
  ou_noise = OrnsteinUhlenbeckActionNoise(mu=np.zeros(num_actions), sigma=0.2 * np.ones(num_actions))
  epochs = 2_000
  batch_size = 64
  
  TRAIN = True
  LOAD_MODEL = True
  
  rewards, policy_losses, value_losses, mean_test_rewards = [], [], [], []
  epoch = 0
  t = 0
  best_mean_reward = -np.inf
  
  time_last_checkpoint = time.time()
    
  if LOAD_MODEL:
    start_step, memory = model.load_checkpoint("rl_agent\checkpoints\ep_92.pth.tar")
  if not TRAIN:
    ou_noise = None
    
  loop = tqdm(range(epochs))
    
  for i, _ in enumerate(loop):
    if ou_noise is not None:
      ou_noise.reset()
    epoch_return = 0
    
    state, info = env.reset()
    
    state = torch.Tensor(state).to(device)
    
    while True:
      action = model.calc_action(state, ou_noise)
      next_state, reward, terminated, truncated, _ = env.step(action.cpu().numpy())
      
      epoch_return += reward
      
      mask = torch.Tensor([terminated or truncated]).to(device)
      reward = torch.Tensor([reward]).to(device)
      next_state = torch.Tensor(next_state).to(device)

      memory.push(state, action, mask, next_state, reward)

      state = next_state

      epoch_value_loss = 0
      epoch_policy_loss = 0

      if len(memory) > batch_size:
          transitions = memory.sample(batch_size)
          # Transpose the batch
          # (see http://stackoverflow.com/a/19343/3343043 for detailed explanation).
          batch = Transition(*zip(*transitions))

          # Update actor and critic according to the batch
          value_loss, policy_loss = model.update_params(batch)

          epoch_value_loss += value_loss
          epoch_policy_loss += policy_loss

      if terminated or truncated:
          break
      
    loop.set_postfix_str(f"Reward: {epoch_return}")
    
    rewards.append(epoch_return)
    value_losses.append(epoch_value_loss)
    policy_losses.append(epoch_policy_loss)

    if round(np.mean(rewards[-3:]), 3) >= best_mean_reward:
      print(f"Best Mean Reward updated from {best_mean_reward} to {round(np.mean(rewards[-3:]), 3)}")
      best_mean_reward = round(np.mean(rewards[-3:]), 3)
      model.save_checkpoint(i, memory)
      time_last_checkpoint = time.time()
      print('Saved model at {}'.format(time.strftime('%a, %d %b %Y %H:%M:%S GMT', time.localtime())))
    
    
    # action, _states = model.predict(observation)
    # obs, rewards, terminated, truncated, _ = env.step(action)
    
    # if terminated:
    #   break
  
  
  # while True:
  #   observation, reward, terminated, something, doesntmatter = env.step(np.random.random_integers(-1, 1, (env.action_space.shape)).tolist())
  #   print(reward)
  #   if terminated:
  #     break