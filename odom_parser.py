import numpy
import matplotlib.pyplot as plt 
import math

f = open("odom_7.log", "r")
lines = f.readlines()



x_vals_physical = []
y_vals_physical = []
speed_physical = []
ts_physical = []
for i in range(len(lines)):

    # if 'position' in lines[i]:
    x_vals_physical.append(float(lines[i].split('x: ')[1].split(' ')[0]))
    y_vals_physical.append(float(lines[i].split('y: ')[1].split(' ')[0]))
    ts_physical.append(float(lines[i].split('timestamp: ')[1].split('\n')[0]))
    # if 'linear' in lines[i]:
    #     speed_physical.append(float(lines[i+1].split('x: ')[1]))  



f = open("odom_8.log", "r")
lines = f.readlines()


x_vals_digital = []
y_vals_digital = []
speed_digital = []
ts_digital = []
for i in range(len(lines)):

    # if 'position' in lines[i]:
    x_vals_digital.append(float(lines[i].split('x: ')[1].split(' ')[0]))
    y_vals_digital.append(float(lines[i].split('y: ')[1].split(' ')[0]))
    ts_digital.append(float(lines[i].split('timestamp: ')[1].split('\n')[0]))

    # if 'linear' in lines[i]:
    #     speed_physical.append(float(lines[i+1].split('x: ')[1]))  
 

eucl_dist_dig = []
for i in range(len(x_vals_digital)):
    x_vals_sqrd = math.pow(x_vals_digital[i], 2)
    y_vals_sqrd = math.pow(y_vals_digital[i], 2)
    eucl_dist_dig.append(math.sqrt(x_vals_sqrd+y_vals_sqrd))
eucl_dist_phys = []
for i in range(len(x_vals_physical)):
    x_vals_sqrd = math.pow(x_vals_physical[i], 2)
    y_vals_sqrd = math.pow(y_vals_physical[i], 2)
    eucl_dist_phys.append(math.sqrt(x_vals_sqrd+y_vals_sqrd))


file = open("Euclidean_digital.txt", "w")
file1 = open("Euclidean_physical.txt", "w")

for i in range(len(ts_digital)):
    file.write(f"{eucl_dist_dig[i]}\t{ts_digital[i]}\n")

for i in range(len(ts_physical)):
    file1.write(f"{eucl_dist_phys[i]}\t{ts_physical[i]}\n")

plt.plot(ts_digital, eucl_dist_dig, label = 'euc digital')
plt.plot(ts_physical, eucl_dist_phys, label = 'euc physical')

plt.legend()

plt.show()