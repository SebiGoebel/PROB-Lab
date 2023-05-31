# PROB-Lab

Teleop Befehl um den Roboter mit der Tastatur zu steuern:
```bash
roslaunch turtlebot3_teleop turtlebot3_teleop_key.launch
```

Für das erstellen einer Map mit SLAM:
```bash
rosrun gmapping slam_gmapping
```

Zum speichern einer Map:

(speichert eine Map dort wo ich gerade im terminal bin)

(wirft ein map.yaml und ein map.png aus)
```bash
rosrun map_server map_saver -f <**dateiname**>
```

Zum Beispiel:
```bash
rosrun map_server map_saver -f map
```
