# PROB-Lab


## um eine Karte zu erstellen

1. gazebo und rviz öffnen (in rviz alle relevanten topics anzeigen lassen)

2. Für das erstellen einer Map mit SLAM:
```bash
rosrun gmapping slam_gmapping
```

3. Teleop Befehl um den Roboter mit der Tastatur zu steuern:
```bash
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

4. den Roboter bewegen und die Karte aufnehmen

5. Wenn die Karte fertig ist --> Speichern der Map:

(speichert eine Map dort wo ich gerade im terminal bin)

(wirft ein map.yaml und ein map.png aus)
```bash
rosrun map_server map_saver -f <dateiname>
```

Zum Beispiel:
```bash
rosrun map_server map_saver -f map
```
