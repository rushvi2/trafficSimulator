import numpy as np

class QLearningAgent:
    def __init__(self, state_size, action_size=3, learning_rate=0.1, discount_factor=0.9, exploration_rate=1.0):
        self.state_size = state_size
        self.action_size = action_size  
        self.learning_rate = learning_rate
        self.discount_factor = discount_factor
        self.exploration_rate = exploration_rate
        self.exploration_decay = 0.995
        self.q_table = np.zeros((state_size, action_size))

    def choose_action(self, state):
        if np.random.rand() < self.exploration_rate:
            return np.random.choice(self.action_size) 
        return np.argmax(self.q_table[state]) 
        
    def update_q_table(self, state, action, reward, next_state):
        if state >= self.state_size or next_state >= self.state_size:
            print(f"state out of bounds")
            return

        best_next_action = np.argmax(self.q_table[next_state])
        td_target = reward + self.discount_factor * self.q_table[next_state][best_next_action]
        td_error = td_target - self.q_table[state][action]
        self.q_table[state][action] += self.learning_rate * td_error


    def decay_exploration(self):
        self.exploration_rate = max(0.01, self.exploration_rate * self.exploration_decay)