import gym

class PavsTimeLimitWrapper(gym.Wrapper):
   
    def __init__(self, env, max_steps=100):
       
        super(PavsTimeLimitWrapper, self).__init__(env)
        self.max_steps = max_steps
        self.current_step = 0
  
    def reset(self):
        self.current_step = 0
        return self.env.reset()

    def step(self, action):
        self.current_step += 1
        obs, reward, done, info = self.env.step(action)
        
        if self.current_step >= self.max_steps:
            done = True
            info['time_limit_reached'] = True
            info['is_success'] = 0.0  # Assuming you want to add 'is_success' here

        return obs, reward, done, info
