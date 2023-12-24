import numpy as np
from pettingzoo.mpe import simple_adversary_v3

env = simple_adversary_v3.env(render_mode="rgb_array", continuous_actions=False, max_cycles=5)
env.reset()

def print_info(env):
    env.reset()
    print('env', env)
    print('numver of agents', env.num_agents)
    print('possible agent', env.possible_agents)
    print('agents name', env.agents)
    print('observation spavce', env.observation_spaces) ##dont use this its depricated
    print('observation space1 :', env.observation_space('adversary_0')) #used agent name and below used agents index from agent list
    print('observation space2 :', env.observation_space(env.agents[0]))
    print('observation space :', env.observation_space)
    print('observation size :', env.observation_space(env.agents[0]).shape[0])
    print('action space: ', env.action_space(env.agents[0]))
    print('num_action: ', env.action_space(env.agents[0]).n)
    #print('n actions', env.action_space.values())

def get_obs_reward():
    next_obs, info = env.reset(seed=None)
    print("Nxt_observation : ",next_obs)
    for agent in env.agents:
        observation, reward, termination, truncat, info = env.last()
        action = env.action_space(agent).sample()
        print(f"Agent: {agent} -> action: {action}")
        t = env.step(action)
        print(observation)
        print("rewards: ",env.rewards)
        # print(termination)
        # print(truncat)

def get_env_obs(env):
    obs=[]
    for agent in env.agents:
        observation, reward, termination, truncat, info = env.last()
        obs.append(observation)
    print(np.array(obs))

def take_action(env, action):
    act = np.argmax(action, axis=1)
    i=0
    for agent in env.agents:
        #action =env.action_space(agent)

        print(act[i])
        env.step(act[i])
        i+=1
        


def get_env_rwd(env):
    rwd=[]

    for agent in env.agents:
        observation, reward, termination, truncat, info = env.last()
        rwd.append(reward)
    print(np.array(rwd))

# get_env_obs(env)
# no_op = np.array([1,0,0,0,0])
# no_op1 = np.array([0,0,1,0,0])
# no_op2 = np.array([0,0,0,0,1])

# action = [no_op, no_op1, no_op2]

# take_action(env, action)
# get_env_rwd(env)

env.reset()
for agent in env.agents:
    observation, reward, termination, truncation, info = env.last()
    state = env.render()
    print(state)
    if termination or truncation:
        action = None
    else:
        action = env.action_space(agent).sample() # this is where you would insert your policy
        
    env.step(action) 
env.close()
