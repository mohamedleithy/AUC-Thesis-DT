# QT Configurator

The Configurator is a desktop application used for customizing the OTA remote driving dashboard according to the user's input
## Project Demo
The following link demonstrates in a video the whole project process using the configurator

[Project Demo](https://drive.google.com/file/d/1iktGOEk-YixxjGIhwBbPbenZhpoLa4H1/view)

## Paths Setup
The following lines should be adjusted according to the location of the files on the system. 

These lines are located in QT/ui_functions.py script

```python
dir_path = "~/Downloads/dashboardCode/RemoteDrivingDashboard-master/docker-compose.yml" 
os.system("docker-compose -f " + dir_path + " up -d")
os.system("nohup python3 ~/RemoteDrivingQtApp/Designs/cloudconnect/reactive.py > output.log &")
os.system("nohup python3 ~/RemoteDrivingQtApp/Designs/cloudconnect/stream.py > output.log &")
os.system("nohup python3 ~/RemoteDrivingQtApp/Designs/cloudconnect/teleop.py > output.log &")
```
The files mentioned are the following:
* docker-compose.yml in the [dashboard](https://github.com/SaraAkmal/RemoteDrivingDashboard) application 
* reactive.py, stream.py and teleop.py in [ROS](https://github.com/islamashraf98/ROS) application


## How to Run the Application
Run the QtGUI.py script located in Designs folder.

## How to Edit the QT Design
If there are changes that need to take place in the design you can change the QtApp.ui by using the .ui file in the QT designer and then generate the GUI python code with the following command:
```bash
pyuic5 -x QtApp.ui -o QtGUI.py
```

## Project Features and Details
[Project Thesis](https://github.com/SaraAkmal/RemoteDrivingDashboard/blob/master/Cloud-Connected%20AGV%20Thesis.pdf)

[Project Presentation](https://github.com/SaraAkmal/RemoteDrivingDashboard/blob/master/Cloud-Connected%20AGV%20Presentation.pptx)

## Configurator Preview
<img src="https://github.com/SaraAkmal/OTA_RemoteDrivingConfigurator/blob/main/Preview%20Snippet.gif"/>

