from datetime import datetime
import rospy
from std_msgs.msg import String
from subprocess import call
import socketio
import json
import re
import math
import message_converter
import subprocess
# from django.db import connections
import mysql.connector
import time
import os 
import subprocess
#Kinesis SDK
import boto3

from fmpy import *
#from fmpy.utill import plot_result
import numpy as np



import os
os.environ["SC1D_LICENSING_TYPE"] = "ugs"
os.environ["LD_LIBRARY_PATH"] = "/AUC-Thesis-DT/FMU/minimalAmesim/lnx_x64"
os.environ["SPLM_LICENSE_SERVER"]="28000@orw-sddlic1"

def fmu(path, setSpeed, posError, headingError):
    fmu = path
    dump(fmu)
    #read the model description
    model_description = read_model_description(fmu)
    vrs = {}
    for variable in model_description.modelVariables:
        vrs[variable.name] = variable.valueReference
    print(vrs)

    start_values = [setSpeed, posError, headingError]
    dtype = [('expseu_.SetSpeed_mps', float), ('expseu_.PosError_m', float), ('expseu_.HeadingError_rad', float)]

    signals = np.array(start_values, dtype=dtype)


    result = simulate_fmu(fmu, stop_time=0.5, input=signals)
    print(result)
    return result
    
def pushing_into_file(results):
    for result in results:
        PositionX_m = open("PositionX_m.txt", 'a+')
        PositionX_m.write(f"{result[0]},{result[1]} \n")
        PositionX_m.close()
        PositionY_m = open("PositionY_m.txt", 'a+')
        PositionY_m.write(f"{result[0]},{result[2]} \n")
        PositionY_m.close()
        PositionZ_m = open("PositionZ_m.txt", 'a+')
        PositionZ_m.write(f"{result[0]},{result[3]} \n")
        PositionZ_m.close()
        Roll_deg = open("Roll_deg.txt", 'a+')
        Roll_deg.write(f"{result[0]},{result[4]} \n")
        Roll_deg.close()
        Pitch_deg = open("Pitch_deg.txt", 'a+')
        Pitch_deg.write(f"{result[0]},{result[5]} \n")
        Pitch_deg.close()
        Yaw_deg = open("Yaw_deg.txt", 'a+')
        Yaw_deg.write(f"{result[0]},{result[6]} \n")
        Yaw_deg.close()
        WheelSpeedLeft_RPM = open("WheelSpeedLeft_RPM.txt", 'a+')
        WheelSpeedLeft_RPM.write(f"{result[0]},{result[7]} \n")
        WheelSpeedLeft_RPM.close()
        WheelSpeedRight_RPM = open("WheelSpeedRight_RPM.txt", 'a+')
        WheelSpeedRight_RPM.write(f"{result[0]},{result[8]} \n")
        WheelSpeedRight_RPM.close()




kinesis_client = boto3.client('kinesis')


# data = {"sensor_name": "Accelerometer1",
# "linear_acceleartion_x": 1,
#  "linear_accelaration_y": 1,
#  "linear_acceleration_z": 1,
#  "magnitude": 7}

# data = {"sensor_name": "Speed1",
#   "linear_x" : 2,
# "linear_y" : 3,
# "linear_z" : 4,
# "angular_x" : 1,
# "angular_y" : 1,
# "angular_z" : 1,
# "magnitude": 0.2}

# data = { "sensor_name": "Position",
# "position_x":5000,
# "position_y":2000,
# "position_z":3000
# }


# def dbInit(db_host,db_user,db_password,data_base,server_addr):
#     server_addr = server_addr
#     server_port = 8000
#     mydb = mysql.connector.connect(
#     host=db_host,
#     user= db_user,
#     password=db_password,
#     database=data_base,
#     autocommit=True,
#     port = 3306
#     )
#     cursor = mydb.cursor()
#     print('[INFO] Connecting to server http://{}:{}...'.format(server_addr, server_port))
#     return cursor, mydb

# def dbSave(data):
#   cursor, mydb = dbInit("127.0.0.1","root","root","velanalytics","0.0.0.0")

  # cursor = connections["default"].cursor()
  # sql_statement = f"""INSERT INTO  {data["sensor_name"]}
  #               ( `time`,
  #               `value`)
  #               VALUES
  #               ({data["time"]},{data["magnitude"]});
  #               """
  # cursor.execute(sql_statement)

def callback(data):
#     # rospy.loginfo(rospy.get_caller_id() + "I heard %s", data.data)
#     #Send the data you got from sensors throgh ros node to the dashboard to update the graphs by emitting an event
  # sio.emit('sensedData', data ,namespace='/dashboard')
  data = message_converter.convert_ros_message_to_dictionary(data)

  data = json.loads(data['data'])
  

  data = preprocessing(data)
  print(data)
  # cursor, mydb = dbInit("127.0.0.1","root","root","velanalytics","0.0.0.0")

  # cursor = connections["default"].cursor()
                
  # cursor.execute(sql_statement)

  #timestamp here ----------------------


  obj = time.gmtime(0)
  epoch = time.asctime(obj)
  print("The time is:",epoch)
  curr_time = round(time.time())
  print("Milliseconds since epoch:",curr_time)

def preprocessing(data):
    global i 
    i+=1
    print(i)
    data["time"] = json.dumps(datetime.now(), default=str)
    flag = False
    if(bool(re.search("^Accelerometer", data["sensor_name"]))):
      print("accelerometer")
      flag = True

      # dbSave(data)    

    elif(bool(re.search("^Speed", data["sensor_name"]))):
      print("speed")
      data = {"sensor_name" : data["sensor_name"],
              "magnitude" : data['magnitude'],
              "time": json.dumps(datetime.now(), default=str)
            }
      # subprocess.run(['aws', 'cloudwatch', 'put-metric-data', '--metric-name', 'Speed', '--namespace', 'Turtlebot3', '--unit', 'mps', '--value', f'''{data['magnitude']}'''])
      
      speed = float(data['magnitude'])/10
      pushing_into_file(fmu("/AUC-Thesis-DT/FMU/AGV_turtle_electric_drive_Prescan_export.fmu", speed, 0.1, 0.1))
      
      response = kinesis_client.put_record(StreamName='turtlebot', Data=f'''{{"Speed":"{data['magnitude']}"}}''',PartitionKey='123',)
      print(response)
      flag = True
      

    if(bool(re.search("^Position", data["sensor_name"]))):
      print("position")


    else:  
      if(i==30):      
        # dbSave(data)
        i=0


    return data
def listener():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('listener', anonymous=True)

    rospy.Subscriber("metric", String, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()





if __name__ == '__main__':
    global i 
    i=0
    listener()