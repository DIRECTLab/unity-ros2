import gymnasium as gym
import roslibpy
import numpy as np

class RLEnv(gym.Env):
  def __init__(self) -> None:
    super().__init__()
    self.action_space = gym.spaces.Box(-1, 1, (14,), np.int8)
    self.state_space = gym.spaces.Box(-np.inf, np.inf, (17,), np.float32)
    self.ros_state = [0.0 for _ in range(19)]
    self.initial_ros_state = None
    self.__next_state_ready = False
    self.client = roslibpy.Ros('localhost', port=9090)
    self.client.run()
    self.listener = roslibpy.Topic(self.client, '/state', 'std_msgs/Float32MultiArray')
    self.listener.subscribe(self.handle_sim_state)
    self.publisher = roslibpy.Topic(self.client, '/control', 'std_msgs/Int8MultiArray')
  
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
    observation = self._get_obs()
    info = None
    return observation, info
  
  def step(self, action):
    self.publisher.publish(roslibpy.Message({'data': action}))
    self.__next_state_ready = False
    
    observation = self._get_obs()
    terminated = self.ros_state[3] != 0
    reward = self.ros_state[4]

    return observation, reward, terminated, False, None


if __name__ == '__main__':
  env = RLEnv()
  
  observation, info = env.reset()
  
  while True:
    observation, reward, terminated, something, doesntmatter = env.step(np.random.random_integers(-1, 1, (env.action_space.shape)).tolist())
    print(reward)
    if terminated:
      break