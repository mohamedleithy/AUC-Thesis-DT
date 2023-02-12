from fmpy import *
#from fmpy.utill import plot_result
from pyfmi import *
import matplotlib.pyplot as plt
import numpy as np
import matplotlib.animation as animation
fig,(ax1,ax2,ax3,ax4)=plt.subplots(nrows=4,ncols=1 )

def fmu(path,force):
    fmu = path
    dump(fmu)
    #read the model description
    model_description = read_model_description(fmu)
    vrs = {}
    for variable in model_description.modelVariables:
        vrs[variable.name] = variable.valueReference
    print(vrs)
    # Force = force
    # dtype = [('expseu_.Force', np.float)]
    # signals = np.array([Force], dtype=dtype)
    # result = simulate_fmu(fmu, stop_time=0.5, input=signals)
    # print(result)
    # return result
    
# def pushing_into_file(results):
#     for result in results:
#         front_left = open("front_left.txt", 'a+')
#         front_left.write(f"{result[0]},{result[1]} \n")
#         front_left.close()
#         front_right = open("front_right.txt", 'a+')
#         front_right.write(f"{result[0]},{result[2]} \n")
#         front_right.close()
#         rear_left = open("rear_left.txt", 'a+')
#         rear_left.write(f"{result[0]},{result[3]} \n")
#         rear_left.close()
#         rear_right = open("rear_right.txt", 'a+')
#         rear_right.write(f"{result[0]},{result[4]} \n")
#         rear_right.close()

def plotting(axis,lines):
    xs = []
    ys = []
    for line in lines:
        if len(line) > 1:
            x, y = line.split(',')
            xs.append(float(x))
            ys.append(float(y))
    axis.clear()
    axis.set_xlim(-2, 26)
    axis.set_ylim(0, 3)
    plt.ylabel('Braking Force')
    plt.xlabel('Time')
    axis.plot(xs, ys)

def animate(i,axis,path):
    graph_data = open(path, 'r').read()
    lines = graph_data.split('\n')
    plotting(axis,lines)

fmu("/home/g02-f22/Downloads/source_code/AUC-Thesis-DT/FMU/BrakingSystem.fmu",50)
#pushing_into_file(fmu("/home/g02-f22/Downloads/ToSend/FMI/BrakingSystem.fmu",50))
# ani_1 = animation.FuncAnimation(fig,animate(ax1,"front_left_path"), interval=1000)
# ani_2 = animation.FuncAnimation(fig,animate(ax1,"front_right_path"), interval=1000)
# ani_3 = animation.FuncAnimation(fig,animate(ax1,"rear_left_path"), interval=1000)
# ani_4 = animation.FuncAnimation(fig,animate(ax1,"rear_right_path"), interval=1000)