import numpy as np

num_states = 10  
num_actions = 2   
start_state = 2     
goal_state = 8  

# 初始化Q表
q_table = np.zeros((num_states, num_actions))

# 超参数
learning_rate = 0.1
discount_factor = 0.9
epsilon = 0.1
episodes = 1000

# 存储每个episode的总奖励
rewards_history = []

# Q学习训练
for episode in range(episodes):
    state = start_state
    total_reward = 0
    done = False
    
    while not done:
        # ε-贪婪
        if np.random.uniform(0, 1) < epsilon:
            action = np.random.choice(num_actions)  # explore
        else:
            action = np.argmax(q_table[state])      # exploit
        
        if action == 0: 
            next_state = max(0, state - 1)
        else: 
            next_state = min(num_states - 1, state + 1)
        
        # 计算奖励
        if next_state == goal_state:
            reward = 10
            done = True
        else:
            reward = -1
        
        # Q学习更新
        if done:
            target = reward
        else:
            target = reward + discount_factor * np.max(q_table[next_state])
        
        q_table[state, action] += learning_rate * (target - q_table[state, action])
        
        # 转移到下一个状态
        state = next_state
        total_reward += reward
    
    rewards_history.append(total_reward)
    
    if (episode + 1) % 10 == 0:
        print(f"Episode {episode+1}/{episodes}, Total Reward: {total_reward}")