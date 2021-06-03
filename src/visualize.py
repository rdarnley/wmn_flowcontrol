# Plot figures

from matplotlib import pyplot as plt

import numpy as np

######

# x = [10, 20, 30, 40, 50, 60, 70, 80, 90, 100]

# fg =    [4.90e5,    1.05e6, 1.61e6, 2.17e6, 2.73e6, 3.29e6, 3.85e6, 4.41e6, 4.97e6, 5.53e6]
# dare =  [5.21e5,    1.11e6, 1.71e6, 2.30e6, 2.90e6, 3.49e6, 4.08e6, 4.68e6, 5.27e6, 5.86e6]
# ct =    [5.12e5,    1.09e6, 1.68e6, 2.26e6, 2.84e6, 3.42e6, 4.01e6, 4.59e6, 5.17e6, 5.76e6]

# plt.figure()

# # plt.plot(x, fg, 'r', label='FG')
# # plt.plot(x, dare, 'g', label='DARE')
# # plt.plot(x, ct, 'k', label='CT')

# plt.semilogy(x, fg, 'r', label='FG')
# plt.semilogy(x, dare, 'g', label='DARE')
# plt.semilogy(x, ct, 'k', label='CT')

# plt.ylim([4e5, 6e6])

# plt.xlabel('Number of Nodes')
# plt.ylabel('Cost')
# plt.title('Nodes vs Cost')

# plt.legend()
# plt.show()

# #########

# x = [10, 20, 30, 40, 50, 60, 70, 80, 90, 100]

# fg =    [0.017, 0.011,  0.024,  0.029,  0.030,  0.028,  0.035,  0.035,  0.037,  0.042]
# dare =  [0.003, 0.011,  0.033,  0.063,  0.109,  0.190,  0.364,  0.493,  0.601,  0.891]
# ct =    [0.004, 0.010,  0.025,  0.046,  0.082,  0.137,  0.258,  0.310,  0.434,  0.574]

# plt.figure()

# plt.plot(x, fg, 'r', label='FG')
# plt.plot(x, dare, 'g', label='DARE')
# plt.plot(x, ct, 'k', label='CT')

# plt.xlabel('Number of Nodes')
# plt.ylabel('Time (s)')
# plt.title('Nodes vs Time')

# plt.legend()
# plt.show()

#######

# x = [0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1.0]

# fg =    [2.73e6,    2.73e6, 2.73e6, 2.73e6, 2.73e6, 2.73e6, 2.72e6, 2.72e6, 2.71e6, 2.51e6]
# dare =  [2.90e6,    2.90e6, 2.90e6, 2.90e6, 2.90e6, 2.88e6, 2.87e6, 2.86e6, 2.82e6, 2.80e6]
# ct =    [2.89e6,    2.88e6, 2.87e6, 2.85e6, 2.84e6, 2.82e6, 2.81e6, 2.79e6, 2.77e6, 9.40e6]

# plt.figure()

# # plt.plot(x, fg, 'r', label='FG')
# # plt.plot(x, dare, 'g', label='DARE')
# # plt.plot(x, ct, 'k', label='CT')

# plt.semilogy(x, fg, 'r', label='FG')
# plt.semilogy(x, dare, 'g', label='DARE')
# plt.semilogy(x, ct, 'k', label='CT')

# plt.ylim([2e6, 3e6])


# plt.xlabel('Actuation Ratio')
# plt.ylabel('Cost')
# plt.title('Actuation Ratio vs Cost')

# plt.legend()
# plt.show()

# #######

# x = [0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1.0]

# fg =    [0.022, 0.024,  0.028,  0.028,  0.021,  0.021,  0.018,  0.024,  0.024,  0.023]
# dare =  [0.083, 0.085,  0.107,  0.098,  0.120,  0.179,  0.162,  0.172,  0.191,  0.224]
# ct =    [0.058, 0.062,  0.068,  0.072,  0.123,  0.137,  0.119,  0.128,  0.190,  0.441]

# plt.figure()

# plt.plot(x, fg, 'r', label='FG')
# plt.plot(x, dare, 'g', label='DARE')
# plt.plot(x, ct, 'k', label='CT')

# plt.xlabel('Actuation Ratio')
# plt.ylabel('Time (s)')
# plt.title('Actuation Ratio vs Time')

# plt.legend()
# plt.show()

########

import yaml

filename = "../config/node_topology.yaml"

with open(filename) as file:
    dictionary = yaml.full_load(file)

nodes = dictionary["Nodes"]

print(nodes)

plt.figure()

for k, v in nodes.items():

    plt.scatter(v[0], v[1], c='r')

plt.show()