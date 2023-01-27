## Build with CMake

```bash
# create a build directory
mkdir build
# generate the build files and run the build
cmake -S . -B build/
cmake --build build/
```

cd ~/packages/listeners_commander/

With test Simulation:

```
#Coammander (with correct IP-Adress of Simulation)
./build/interface_commander 192.168.0.192 9999
#Listeners(with correct IP-Adress of Simulation)
./build/interface_lidar 192.168.0.192 9997
./build/interface_odometry 192.168.0.192 9998
```
#starting order:
1) Simulation
2) Commander
3) Listener Lidar
4) Listener Odometrie


Live System:

```
#Coammander (with correct IP-Adress of Simulation)
./build/interface_commander  <Server IP> <landmark-x> <landmark-y>
#Listeners(with correct IP-Adress of Simulation)
./build/interface_lidar <Server IP> [<Landmark Radius [m]>] [<Max. Scan Range [m]>]
./build/interface_odometry  <Server IP> [<Echo Port>]
```
#starting order:
1) Commander
2) Listener Lidar
3) Listener Odometrie

#Landmarks:
Line:       x = 0.5     y = 0.24
Circle:     x = 0.35    y = 0.25
Square:     x = -0.23   y = -0.19
Triangle:   x = -0.8    y = -0.48
___________________________________________________________________________________________________________________________



REFERENCES/QUELLEN:

Shared Memory:
[1] Moodle-Course: Advanced Programming for Robots (MRE-VZ-1-WS2022-vAPR)
[2] Jacob Sorber: How to Set up Shared Memory in Your Linux and MacOS Programs. (shmget, shmat, shmdt, shmctl, ftok) - (https://www.youtube.com/watch?v=WgVSq-sgHOc&list=PLvXqHwX2HFAI-zCw8JQyWBysP1VSr2Plw&index=1&t=539s)

Sempahores:
[1] Moodle-Course: Advanced Programming for Robots (MRE-VZ-1-WS2022-vAPR)
[3] Jacob Sorber: What is a semaphore? How do they work? (Example in C) - (https://www.youtube.com/watch?v=ukM_zzrIeXs&list=PLvXqHwX2HFAI-zCw8JQyWBysP1VSr2Plw&index=2)
[4] https://pages.cs.wisc.edu/~remzi/OSTEP/threads-sema.pdf


TCP-IP Connection:
[1] Moodle-Course: Advanced Programming for Robots (MRE-VZ-1-WS2022-vAPR)

Controller:
[1]Moodle-Course: MOR (BMR 6)
[2s]https://www.arxterra.com/lecture-6-pid-controllers/
[3]https://tttapa.github.io/Pages/Arduino/Control-Theory/Motor-Fader/PID-Cpp-Implementation.html