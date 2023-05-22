# -*- coding: utf-8 -*-
from __future__ import unicode_literals

from django.http import HttpResponse
from django.template import loader

from django.shortcuts import redirect #render
from django.template.response import TemplateResponse as render
from django.http import HttpResponse , JsonResponse
from src.local_cloud.Query_Search import select_data,create_DB
import json as simplejson
from apps.home.views import sio
from src.local_cloud.qt_rcv import rcv, getCountForEachSensor, getAnomalyData
import json
from apps.sense.AnomalyDetector  import detect_outliers #todo



#-----------------------historicaldata---------------------#

def connectionState(state):
	if state == 0:
		global isConnected
		isConnected = False
	elif state == 1:
		isConnected = True

connectionState(0) #setting connection with QT at first with false to render page without charts

def main(request):
	return render(request, "home/dashboard_layout.html")




def show_data(request):
	if request.is_ajax():
		ranges_dict = {"FromDate":request.POST["FromDate"],
					   "ToDate" : request.POST["ToDate"]
		}
		data = select_data(ranges_dict)
		json_stuff = simplejson.dumps(data, indent=4, sort_keys=True, default=str) 
		return HttpResponse(json_stuff)
	else:
		data = None
		sensorInfo = None
		#data = select_data(None)
		#sensorInfo = select_data(None)
		json_stuff = simplejson.dumps(data, indent=4, sort_keys=True, default=str) 
		if not isConnected:
			return render(request,"sense/historicaldata.html",{"dataJS":json_stuff, 'data': json.dumps(sensorInfo)})
		# Fetching sensors from QT
		sensorInfo = getCountForEachSensor()
		return render(request,"sense/historicaldata.html",{"dataJS":json_stuff, 'data': json.dumps(sensorInfo)})


def real_data(request):
	if not isConnected:
		return render(request,"sense/real_data.html")

	sensorInfo = getCountForEachSensor()
	return render(request,"sense/real_data.html",{'data': json.dumps(sensorInfo), 'anomaly': json.dumps(getAnomalyData())})

def anomaly_detection(request):
	if request.is_ajax():
		graphInfo = json.loads(request.POST.get("graphInfo"))
		result = detect_outliers(graphInfo)
		return JsonResponse(result, safe=False)


def Create_DynamicDBSocketFunc(data):
	rcv(data)
	connectionState(1)


@sio.on('DB_Info', namespace='/dynamicDB')
def Create_DynamicDB(sid, data):
    Create_DynamicDBSocketFunc(data)








