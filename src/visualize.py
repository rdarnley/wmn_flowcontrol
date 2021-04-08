# Plot figures

from matplotlib import pyplot as plt

import numpy as np

######

# x = [10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 120]

# fg = [2.70e6, 3.65e6, 4.59e6, 5.53e6, 6.48e6, 7.42e6, 8.36e6, 9.30e6, 1.02e7, 1.12e7, 1.31e7]
# dare = [2.69e6, 3.63e6, 4.57e6, 5.50e6, 6.44e6, 7.38e6, 8.31e6, 9.25e6, 1.02e7, 1.11e7, 1.30e7]
# ct = [2.66e6, 3.55e6, 4.44e6, 5.33e6, 6.23e6, 7.12e6, 8.01e6, 8.91e6, 9.80e6, 1.07e7, 1.25e7]

# plt.figure()

# # plt.plot(x, fg, 'r', label='FG')
# # plt.plot(x, dare, 'g', label='DARE')
# # plt.plot(x, ct, 'k', label='CT')

# plt.semilogy(x, fg, 'r', label='FG')
# plt.semilogy(x, dare, 'g', label='DARE')
# plt.semilogy(x, ct, 'k', label='CT')

# plt.xlabel('Number of Nodes')
# plt.ylabel('Cost')
# plt.title('Nodes vs Cost')

# plt.legend()
# plt.show()

#########

# x = [10, 20, 30, 40, 50, 60, 70, 80, 90, 100, 120]

# fg = [0.0155, 0.0337, 0.0480, 0.1370, 0.1338, 0.2032, 0.2087, 0.2735, 0.2735, 0.3251, 0.4042]
# dare = [0.0102, 0.0452, 0.1270, 0.2590, 0.4863, 0.8362, 1.3286, 1.9681, 2.7160, 3.9898, 6.5275]
# ct = [0.0119, 0.0409, 0.1047, 0.2116, 0.3709, 0.6182, 0.9644, 1.3764, 1.8485, 2.5237, 4.2524]

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

# fg = [6.48e6, 6.48e6, 6.48e6, 6.48e6, 6.48e6, 5.51e6, 5.02e6, 4.72e6, 4.09e6, 3.27e6]
# dare = [6.46e6, 6.46e6, 6.45e6, 6.45e6, 6.44e6, 5.45e6, 4.99e6, 4.59e6, 3.84e6, 2.41e6]
# ct = [6.43e6, 6.38e6, 6.33e6, 6.28e6, 6.23e6, 5.21e6, 4.67e6, 4.33e6, 3.64e6, 4.27e6]

# plt.figure()

# # plt.plot(x, fg, 'r', label='FG')
# # plt.plot(x, dare, 'g', label='DARE')
# # plt.plot(x, ct, 'k', label='CT')

# plt.semilogy(x, fg, 'r', label='FG')
# plt.semilogy(x, dare, 'g', label='DARE')
# plt.semilogy(x, ct, 'k', label='CT')


# plt.xlabel('Actuation Ratio')
# plt.ylabel('Cost')
# plt.title('Actuation Ratio vs Cost')

# plt.legend()
# plt.show()

#######

x = [0.1,0.2,0.3,0.4,0.5,0.6,0.7,0.8,0.9,1.0]

fg = [0.1293, 0.1485, 0.1962, 0.1296, 0.1394, 0.2002, 0.1884, 0.1976, 0.2006, 0.2054]
dare = [0.3311, 0.3727, 0.4139, 0.4503, 0.4886, 0.6453, 0.7225, 0.7758, 0.8781, 0.9843]
ct = [0.2625, 0.2824, 0.3110, 0.3382, 0.3731, 0.4197, 0.5362, 0.6259, 0.9614, 2.2999]

plt.figure()

plt.plot(x, fg, 'r', label='FG')
plt.plot(x, dare, 'g', label='DARE')
plt.plot(x, ct, 'k', label='CT')

plt.xlabel('Number of Nodes')
plt.ylabel('Cost (*10^6)')
plt.title('Nodes vs Cost')

plt.legend()
plt.show()