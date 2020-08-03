import csv
import matplotlib.pyplot as plt

#plt.plot(x, y)
#plt.plot(z, t)
#plt.show()

with open('pos') as csvfile:
    data = list(csv.reader(csvfile))

x = []
y = []
for point in data:
	x.append(float(point[0]))
	y.append(float(point[1]))
plt.plot(x, y)
plt.show()

