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
    eucl_dist_dig.append((math.sqrt(x_vals_sqrd+y_vals_sqrd), ts_digital[i]))
eucl_dist_phys = []
for i in range(len(x_vals_physical)):
    x_vals_sqrd = math.pow(x_vals_physical[i], 2)
    y_vals_sqrd = math.pow(y_vals_physical[i], 2)
    eucl_dist_phys.append((math.sqrt(x_vals_sqrd+y_vals_sqrd), ts_physical[i]))


speed_dig = []
timestamps_dig = []
for i in range(len(eucl_dist_dig) - 80):
    speed_dig.append((eucl_dist_dig[i+80][0] - eucl_dist_dig[i][0])/(eucl_dist_dig[i+80][1] - eucl_dist_dig[i][1]))
    timestamps_dig.append(eucl_dist_dig[i+80][1])

speed_phys = []
timestamps_phys = []
for i in range(len(eucl_dist_phys) - 80):
    pos2 = eucl_dist_phys[i+80][0]
    pos1 = eucl_dist_phys[i][0]

    t2 = eucl_dist_phys[i+80][1]
    t1 = eucl_dist_phys[i][1]

    time = t2 - t1

    pos = pos2 - pos1 
    speed_phys.append((pos/time))
    timestamps_phys.append(t2)

# file = open("Speed_data_digital.txt", "w")
# file1 = open("Speed_data_physical.txt", "w")

# for i in range(len(timestamps_dig)):
#     file.write(f"{speed_dig[i]}\t{timestamps_dig[i]}\n")

# for i in range(len(timestamps_phys)):
#     file1.write(f"{speed_phys[i]}\t{timestamps_phys[i]}\n")

error_dig = []
error_phys = []
for i in range(0, len(speed_dig), 4):
    error_dig.append((speed_dig[i], timestamps_dig[i]))

for i in range(0, len(speed_phys), 3):
    error_phys.append((speed_phys[i], timestamps_phys[i]))


print(len(error_dig))
print(len(error_phys))
final_error = 0

for i in range(len(error_phys)):
    print(f"Example:\n{error_phys[i][0]}\t{error_phys[i][1]}\n{error_dig[i][0]}\t{error_dig[i][1]}")
    if error_phys[i][0] != 0:
        error = (abs(abs(error_dig[i][0]) - abs(error_phys[i][0]))/abs(error_dig[i][0]))
        #print(error)
        final_error += error


final_error = final_error/(len(error_phys))

print(f"Average error = {final_error}")

# plt.plot(timestamps_dig, speed_dig, label = ' speed_dig')
# plt.plot(timestamps_phys,speed_phys, label = ' speed_phys')

# plt.legend()

# plt.show()