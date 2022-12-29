import numpy as np
import matplotlib.pyplot as plt
import seaborn as sns
import itertools as it
import time


# Maze
def parking_lot_maze():

    maze = np.ones((3,30))
    maze[0,9] = 0
    maze[0,10] = 0
    maze[1,20] = 0
    maze[1,21] = 0
    return maze


def reward():
    r = np.zeros((3,30))

    r[0,9] = -1
    r[0,10] = -1
    r[1,20] = -1
    r[1,21] = -1

    return r

plt.matshow(parking_lot_maze())
#plt.savefig('parking_lot.jpg')
#plt.matshow(reward())
#plt.grid()plt.grid()
plt.show()

# 初始化
r = reward()
maze = parking_lot_maze()

### TD Learning

def V(s):
    return values[s[0], s[1]]

def R(s):
    return r[s[0], s[1]]

def get_visited(s):
    return visited[s[0], s[1]]

def inc_visited(s):
    visited[s[0], s[1]] += 1.0

def set_V(s, value):
    values[s[0], s[1]] = value

def in_maze(s):
    return s[0] >= 0 and s[0] < maze.shape[0] and s[1] >= 0 and s[1] < maze.shape[1]

def is_free(s):
    return maze[s[0], s[1]] == 1

def valid_state(s):
    return in_maze(s) and is_free(s)

def sigmoid(x):
    return 1 / (1 + np.exp(-x))

def softmax(values, beta=10):
    values = np.exp(values) * beta
    return values / np.sum(values)

def policy(s):  # for TD learning
    possible_next_states = [s + d for d in np.array([[1, 0], [-1, 0], [0, 1], [0, -1]])] # 上下左右
    filtered = list(filter(valid_state, possible_next_states))
    vs = np.array([V(s) for s in filtered])
    p = softmax(vs)
    return filtered[np.random.choice(range(len(filtered)), 1, p=p)[0]]


def alpha(s):
    return 1 / (get_visited(s) + 1)

'''
# TD算法
s_0 = np.array([2, 0])

maze = np.array([
    [1, 1, 1, 1],
    [1, 0, 1, 1],
    [1, 0, 1, 1],
    [1, 1, 1, 1]
])
r = np.array([
    [0, 0, 0, 0],
    [0, 0, 0, 1],
    [0, 0, 0, -1],
    [0, 0, 0, 0]
])

values = np.zeros(maze.shape)
visited = np.zeros(maze.shape)

s_new = s_0
s = None

alpha = 0.1
gamma = 0.9

for i in range(500):
    s = s_new
    s_new = policy(s)
    if get_visited(s_new) == 0:
        set_V(s_new, R(s_new))
    inc_visited(s_new)

    new_V = V(s) + alpha * (R(s_new) + gamma * V(s_new) - V(s))
    set_V(s, new_V)
    if R(s_new) != 0:
        s_new = s_0

np.round(values, 3)
sns.heatmap(values, annot=True)

'''

### Successor Representation

def state2idx(state):
    for i in range(len(all_states)):
        if np.all(all_states[i] == state):
            return i

def M(s, s_new):
    idx_s = state2idx(s)
    idx_s_new = state2idx(s_new)
    return sr_matrix[idx_s, idx_s_new]

def set_M(s, s_new, value, sr_matrix):
    idx_s = state2idx(s)
    idx_s_new = state2idx(s_new)
    sr_matrix[idx_s, idx_s_new] = value

def random_policy(s, beta=5): # for SR
    possible_next_states = [s + d for d in np.array([[1, 0], [-1, 0], [0, 1], [0, -1], [1, -1], [1, 1], [-1, 1], [-1 ,-1]])]
    filtered = list(filter(valid_state, possible_next_states))
    vs = np.array([1 for s in filtered])
    p = softmax(vs, beta)
    return filtered[np.random.choice(range(len(filtered)), 1, p=p)[0]]

def same_state(s1, s2):
    return np.all(s1 == s2)

# 初始状态值
s_0 = np.array([0, 0])

values = np.zeros(maze.shape, dtype=np.float)
visited = np.zeros(maze.shape)
all_states = [np.array(s) for s in it.product(range(maze.shape[0]), range(maze.shape[1]))]
n_states = maze.shape[0] * maze.shape[1]
sr_matrix = np.eye(len(all_states), dtype=np.float)

s_new = s_0
s = None

alpha = 0.1
gamma = 0.9
steps = 30000
times2goal = 0
collap = 0

tic = time.clock()
for i in range(steps):
    s = s_new
    s_new = random_policy(s)
    if not is_free(s_new):
        collap = collap + 1
    inc_visited(s_new)
    next_sr_matrix = sr_matrix.copy()

    for s_prime in all_states:
        I = 1 if same_state(s, s_prime) else 0
        new_M = M(s, s_prime) + alpha * (I + gamma * M(s_new, s_prime) - M(s, s_prime))
        set_M(s, s_prime, new_M, next_sr_matrix)
    sr_matrix = next_sr_matrix

    if R(s_new) != 0:
        times2goal = times2goal + 1
        s_new = s_0

    if i % 1000 == 0:
        print('Break Point', i)
        print(time.clock())
        #np.save('D:/pyproject/predictive map for SR/for_lanechange/visited_{}.npy'.format(i), visited)
        #np.save('D:/pyproject/predictive map for SR/for_lanechange/sr_matrix_{}.npy'.format(i),sr_matrix)

toc = time.clock()
time = toc-tic
print('花费的总时间：',time)
print('碰撞：',collap)
print('找到目标的次数：', times2goal)

sns.heatmap(visited)
plt.savefig('D:/pyproject/predictive map for SR/for_lanechange/visited_{}.jpg'.format(steps))

sns.heatmap(sr_matrix)
plt.savefig('D:/pyproject/predictive map for SR/for_lanechange/sr_matrix_{}.jpg'.format(steps))
np.save('D:/pyproject/predictive map for SR/for_lanechange/sr_matrix_{}.npy'.format(steps),sr_matrix)

#plt.show()
