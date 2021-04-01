# Plot figures

from matplotlib import pyplot as plt

x = [0.1, 0.2, 0.3, 0.4, 0.5, 0.6, 0.7, 0.8, 0.9, 1.0]

fg = [0.0587469, 0.067577, 0.068329, 0.0741558, 0.0762391, 0.0799957, 0.0897127, 0.19946, 0.246546, 0.656453]
dare = [0.280217, 0.343442, 0.416074, 0.461773, 0.545092, 0.955396, 1.15771, 1.30285, 1.55064, 1.95332]

plt.figure()

plt.plot(x, fg, 'r', label='FG')
plt.plot(x, dare, 'g', label='DARE')

plt.xlabel('Actuation Ratio (# Control / # Nodes)')
plt.ylabel('Time (s)')
plt.title('Actuation Ratio (0.5) vs. Time')

plt.legend()
plt.show()



x = [10, 20, 50, 100, 200]

fg = [0.0382579, 0.064372, 0.0748255, 0.15406, 0.33891]
dare = [0.0169104, 0.0525091, 0.550556, 4.68078, 31.7329]

plt.figure()

plt.plot(x, fg, 'r', label='FG')
plt.plot(x, dare, 'g', label='DARE')

plt.xlabel('Number of Nodes')
plt.ylabel('Time (s)')
plt.title('Number of Total Nodes vs. Time')

plt.legend()
plt.show()