# Motion transfer system, from a physical camera to a virtual one

In order to run the motion transfer system, you will need both Python 3.9.2 + dependencies, and Unity installed in your computer.

1. Clone the repository in your preferred folder.

```git
cd <preferred folder path>
git clone https://github.com/mcroldan/TFG.git
```

2. You will download two folders. Python folder will have the movement detection module's scripts. First, you will have to calibrate your camera using this: https://github.com/opencv/opencv/blob/4.x/doc/pattern.png .
Print the chessboard and take 10+ pictures of it from different angles (you can find more information here https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html ). Once you take the 10+ photos, copy them into
`/Python/calibration` and run the following command:

```
python camera_calibration.py -s <length in meters of side length of printed chessboard>
This will generate a file called calibration_chessboard.yaml.
```

3. Once you have the calibration yaml, run the movement detection script with the following command:

```
python detectArucoPoseRelative.py -s <length in meters of side length of ArUcos> -c <path to calibration yaml file>
```

4. Open Unity and load the Unity/Tracking folder as a project.

5. If you see a `Untitled` scene instead of the `ArUcos_scene` one, go File -> Open Scene -> ..\Unity\Tracking\Assets\Scenes\ArUcos_scene.unity

6. Run the program using the "Play" button above the environment camera.


7. You may change the way you see the results clicking into `TrackingSystem` object and changing `TRACE_MOVEMENT` field on its script. You may also rotate the camera when not in CAMERA_MOVEMENT mode holding right click and WASD.

![image](https://user-images.githubusercontent.com/62695677/207685557-84adb958-2609-4d7a-a86e-378a233d169b.png)
