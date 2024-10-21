# Webots Installation und Setup-Anleitung 
## 1. Webots Installation

Lade den Webots-Installer herunter und folge den Installationsanweisungen:  
[Webots](https://cyberbotics.com/)

Für erste Schritte und eine Einführung in Webots:  
[Getting Started Guide](https://cyberbotics.com/doc/guide/getting-started-with-webots)

# 2. Den THA Branch dieses repositories clonen/herunterladen

Stelle sicher, dass du den **tha** Branch dieses Repositories klonst oder herunterlädst:

```bash
git clone -b tha https://github.com/DynamicSwarms/crazywebotsworld.git
```

❗ Achte darauf beim schließen/verändern der Welt diese nicht abzuspeichern, sodass Crazyflie und Zauberstab immer in Ursprungsposition starten. 

# 3. Controller für Crazyflie und Zauberstab (Wand) bauen

### Crazyflie Controller
1. Rechtsklick auf die Crazyflie im Scene-Tree und wähle **Edit Controller**.
2. Klicke auf das Zahnradsymbol über dem Codefenster, um den Controller zu bauen.
3. Wenn "Reset World?" erscheint, wähle **Reset**.

### Zauberstab Controller (Controllable Wand):
1. Wiederhole die oben genannten Schritte für den Zauberstab.

# 4. Externen Controller starten

Hilfreich, nicht notwendig:
[Externe Controller ausführen](https://cyberbotics.com/doc/guide/running-extern-robot-controllers?tab-os=windows)


### Windows:
1. Kopiere die Datei `webots-controller.exe` aus dem Pfad `C:\Program Files\Webots\msys64\mingw64\bin` in das `crazywebotsworld` Verzeichnis.
2. Controller starten:

   ```bash
   .\webots-controller.exe --protocol=tcp --ip-address=127.0.0.1 --robot-name=cf0_ros_ctrl .\crazyflie-example.py
   ```

### Linux

1. Setze den `WEBOTS_HOME` Pfad: 
    ```bash
    export WEBOTS_HOME=/home/-username-/webots`
    ```

2. Controller starten: 
    ```bash
    $WEBOTS_HOME/webots-controller --protocol=tcp --ip-address=127.0.0.1 --robot-name=cf0_ros_ctrl crazyflie-example.py
    ```

### macOS
1. Setze den `WEBOTS_HOME` Pfad: 
    ```bash
    export WEBOTS_HOME=/home/-username-/webots`
    ```
2. Controller starten: 
    ```bash
    $WEBOTS_HOME/Contents/MacOS/webots-controller --protocol=tcp --ip-address=127.0.0.1 --robot-name=cf0_ros_ctrl crazyflie-example.py
    ```







