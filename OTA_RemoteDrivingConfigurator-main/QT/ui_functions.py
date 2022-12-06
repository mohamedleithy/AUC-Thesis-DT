import json
import socketio
import os
import webbrowser


dir_path = "/home/g2f/Desktop/Thesis/AUC-Thesis-DT/RemoteDrivingDashboard-master/docker-compose.yml" #
os.system("docker-compose -f " + dir_path + " up -d")
# to run the cloudconnect scripts in the background
# the scripts can be found in the ROS directory attached with the project
os.system("nohup python3 /home/g2f/Desktop/Thesis/AUC-Thesis-DT/ROS-master/cloudconnect/Reactive.py > output.log &")
os.system("nohup python3 /home/g2f/Desktop/Thesis/AUC-Thesis-DT/ROS-master/cloudconnect/Streaming.py > output.log &")
os.system("nohup python3 /home/g2f/Desktop/Thesis/AUC-Thesis-DT/ROS-master/cloudconnect/TeleOperations.py > output.log &")

sio = socketio.Client()

@sio.on('connect', namespace='/dynamicDB')
def on_connect():
    print("I'm connected to the /dynamicDB namespace!")

#Sending Sensors Info
def send_data(data):
    if(not sio.connected):
        sio.connect('http://localhost:8000', namespaces=['/dynamicDB'], wait_timeout=60)
    data = str.encode(json.dumps(data))
    print(data)
    sio.emit('DB_Info', data, namespace='/dynamicDB')
    url = "http://localhost:8000/"
    webbrowser.open(url, new=0, autoraise=True)



