from pydoc import visiblename
from typing import Counter
import json
from apps.home.views import sio
from src.local_cloud.DynamicDB import Singleton
from django.shortcuts import render

dynamic_DB = Singleton()

camera = False
gps = False
actuate = False

def rcv(jsonfile):
    print(jsonfile)
    global jsonResponse
    jsonResponse = json.loads(jsonfile.decode('utf-8'))
   
    global camera
    camera = jsonResponse.pop("Camera")
    
    sensorNum = jsonResponse.pop("SensorsNum")
    
    global actuate
    actuate = jsonResponse.pop("Actuate")
    # print("actuate", actuate)

    global gps
    gps = jsonResponse.pop("GPS")

    # not yet used
    global visualization
    visualization = jsonResponse.pop("Visualization")
    global classification
    classification = jsonResponse.pop("Classification")

    global anomalyDetection
    anomalyDetection = jsonResponse.pop("AnomalyDetection")
    

    countsForEachSensor = getCountForEachSensor()

    for key in countsForEachSensor:
        for j in range(countsForEachSensor[key]):
            dynamic_DB.create_Table(key+str(j+1))
    
    print("data received and table created")
    print(jsonResponse.values())
    

def getCountForEachSensor():
    countsForEachSensor = Counter(jsonResponse.values())
    print(countsForEachSensor)
    return countsForEachSensor
    

def actuateData():
    return {'Camera': bool(camera),
            'Actuate': bool(actuate),
            'GPS': bool(gps)}
    
def getAnomalyData():
    return {'AnomalyDetection': bool(anomalyDetection)}



