
# Delivery System / C-Truck project with Gradle

## About this project

This repository stores a template project about `C-Truck`. You will use this project to get started and will add your developed software to this project. 

## Getting Started

To develop software you need following things: 
- Development tools installed as per instructions (IntelliJIDEA and Git/GitHub/SourceTree are must, Gradle is optional).
- Learn (reading provided documentation) about capabilities of motors and sensors and how you can control them with your Java code. See section: Documenatation of system for links to this documentation and to see different code examples. 

To run software you need following things: 
- Your truck (this is where your software will execute - not on your computer!) 
- One computer per team with all development tools installed (here Gradle is necessary as well). 

## How to run your code on truck (when you have access to truck in Lab)

The project includes latest dependencies and an example ready to be deployed on Container Truck using the `C-Truck` library from `CONNEX-AB-Delivery-System`. The project includes some tasks to reduce the time to deploy on your robot.

To start, change the `brickHost`, `brickUser` and `brickPassword` properties in `config.gradle`. Then, continue to the brick setup.

Steps to connect to Truck: 
1) Switch on Truck and wait for OS to load.
2) To start, check the `brickHost`, `brickUser` and `brickPassword` properties in `config.gradle`.
2) Especially, check the IP of Truck (by default for this truck it should be 192.168.122.223 - if not, update the file `config.gradle`):

```
// Brick connection parameters //
    brickHost = "192.168.122.223"
    brickUser = "robot"
    brickPassword = "maker"
```

3) Open Terminal or Command Line. Locate to Project file folder (use commands cd <project directory>). 
4) Connect your computer network to BTH (you don't need to login into this network, just connect). 
5) Now you can use the Java IDE to launch the task or execute them from the terminal

```
./gradlew deployRun
```

5b) Some other tasks associated to deploy on your robot are:

- `testConnection` - Test connection to the brick.
- `deploy` - Deploy a new build of the program to the brick.
- `deployRun` - Deploy a new build of the program to the brick and then run it.
- `run` - Run the program that is currently loaded on the brick.
- `undeploy` - Remove previously uploaded JAR.
- `pkillJava` - Kill running Java instances.
- `fatJar` - Build a fat JAR with all dependencies included inside.

# About C-Truck

motors
- [one EV3 Large Motor] for driving to run front wheel drive system. Documentation here: <a href="http://ev3dev-lang-java.github.io/docs/api/latest/ev3dev-lang-java/ev3dev/actuators/lego/motors/EV3LargeRegulatedMotor.html">Javadocs</a>
- [one EV3 Medium Motor] for steering to run back wheel steering system. Documentation here: <a href="http://ev3dev-lang-java.github.io/docs/api/latest/ev3dev-lang-java/ev3dev/actuators/lego/motors/EV3MediumRegulatedMotor.html">Javadocs</a>
- [one EV3 Large Motor] for lifting crane. Documentation here: <a href="http://ev3dev-lang-java.github.io/docs/api/latest/ev3dev-lang-java/ev3dev/actuators/lego/motors/EV3LargeRegulatedMotor.html">Javadocs</a>
- [one EV3 Medium Motor] for grabbing container. Documentation here: <a href="http://ev3dev-lang-java.github.io/docs/api/latest/ev3dev-lang-java/ev3dev/actuators/lego/motors/EV3MediumRegulatedMotor.html">Javadocs</a>


sensors
- [one EV3 Infrared Sensor] for measuring distance to detect obstacles. Documentation here: <a href="http://ev3dev-lang-java.github.io/docs/api/latest/ev3dev-lang-java/ev3dev/sensors/ev3/EV3IRSensor.html">Javadocs</a> and <a href="http://docs.ev3dev.org/projects/lego-linux-drivers/en/ev3dev-jessie/sensor_data.html#lego-ev3-ir">Sensor capabilities</a>
- [one mindstormsLineReaderV2 Sensor Line] for measuring line reflection to follow the line. Documentation here: <a href="https://github.com/CONNEX-AB-Delivery-System/DS-CSLTruck/blob/master/src/main/java/base/LineReaderV2.java">Javadocs</a> and <a href="http://docs.ev3dev.org/projects/lego-linux-drivers/en/ev3dev-jessie/sensor_data.html#ms-line-leader">Sensor capabilities</a> If you plan implement line following, stoping at start and end <a href="http://www.mindsensors.com/pdfs/LineLeader-v2-User-Guide.pdf">User Guide</a>.



# Documenatation of system

## General information

LEGO brick is running on Debian-based operating system ev3dev: https://github.com/ev3dev (for more info see ev3dev links).

And is programmed in JAVA: http://ev3dev-lang-java.github.io/#/. JAVA programms are deployed on brick by using Gradle,
to see how it is done, follow this link: http://ev3dev-lang-java.github.io/docs/support/getting_started/create-your-first-project.html.

## Examples

Exist several examples ready to use here:

https://github.com/ev3dev-lang-java/examples

Another Git repo for example source code available here: https://github.com/ev3dev-lang-java/template_project_gradle).

In order to modify examples, current full APIs are:

http://ev3dev-lang-java.github.io/docs/api/latest/index.html

You mostly will use EV3 Sensors in package: ev3dev.sensors.ev3 <br />
And classes: EV3ColorSensor, EV3IRSensor, EV3TouchSensor, EV3UltrasonicSensor <br />
*note: we also will use custom LineReaderV2 class, documentation here: TODO: LINK <br />


You mostly will use EV3 Motors in package: ev3dev.actuators.lego.motors <br />
And classes: EV3LargeRegulatedMotor, EV3MediumRegulatedMotor

## Sensors and Motors

For ev3dev OS capabilities on EV3Brick and BrickPI3, you can read here: http://docs.ev3dev.org/en/ev3dev-jessie/

You can learn about sensor capabilities here: http://docs.ev3dev.org/projects/lego-linux-drivers/en/ev3dev-jessie/sensor_data.html

You can learn about sensor capabilities here: http://docs.ev3dev.org/projects/lego-linux-drivers/en/ev3dev-jessie/motor_data.html

Hint: value0 -> value(0) in Java.

## Network

If necessary, to set-up wifi, you can access robot through ssh and then use "connman", described here (Section: Connecting to an open access point):  https://wiki.archlinux.org/index.php/ConnMan#Connecting_to_an_open_access_point


## Configuration

You can change the project configuration in `config.gradle`.
- `mainClass` - Main class of your program.
- `brickHost` - IP address of your brick.
- `brickUser` - Username on your brick.
- `brickPassword` - Password for the `brickUser`.
- `opencv` - Whether to include OpenCV libraries.
- `rxtx` - Whether to include RXTX library.
- `userClasspath` - List of additional URLs for runtime Java classpath.
- `useTime` - Can be used to measure the time for which the program runs.
- `useBrickman` - Should be used when running the program on a brick with a display.
- `useSudo` - If true, the program is launched under root.
- `jvmFlags` - Flags for Java Virtual Machine.
  - `-Xlog:class+load=info,class+unload=info ` - Display the debugging info for class loading.
  - `-Xshare:on ` - Enable Class Data Sharing (recommended).
  - `-Dcom.sun.management.jmxremote=true -Dcom.sun.management.jmxremote.port=7091 -Dcom.sun.management.jmxremote.authenticate=false -Dcom.sun.management.jmxremote.ssl=false ` - Enable JMX agent.
  - `-noverify ` - Do not verify class bytecode. This can be used to speed up startup time, but it is also a potential big security risk.
  - `-XX:TieredStopAtLevel=1 ` - Do not perform many optimizations. This can be used to speed up startup time.
  - `-Djava.security.egd=file:/dev/./urandom ` - This can be used to speed up random number generation.
- `slimJar` - Whether to generate a small JAR with external dependencies, or rather to generate a fat jar with embedded dependencies.
- `useEmbeddedPaths` - Whehter to use classpath and mainclass embedded in JAR or if they should be supplied on the command line.

## Gradle Tasks

The project has some tasks developed to interact in 3 areas:

- EV3Dev-lang-java
- EV3Dev
- Installer


You can use the Java IDE to launch the tasks or you can execute them from the terminal:
```bash
./gradlew deployRun
```

### EV3Dev-lang-java tasks
- `testConnection` - Test connection to the brick.
- `deploy` - Deploy a new build of the program to the brick.
- `deployRun` - Deploy a new build of the program to the brick and then run it.
- `run` - Run the program that is currently loaded on the brick.
- `undeploy` - Remove previously uploaded JAR.
- `pkillJava` - Kill running Java instances.
- `fatJar` - Build a fat JAR with all dependencies included inside.

### Installer tasks
- `helpInstall` - Print the installer help.
- `getInstaller` - Download the installer to the brick.
- `installJava` - Install Java on the brick.
- `installOpenCV` - Install OpenCV libraries on the brick.
- `installRXTX` - Install RXTX library on the brick.
- `installJavaLibraries` - Install EV3Dev-lang-java libraries on the brick.
- `javaVersion` - Print Java version which is present on the brick.
- `updateAPT` - Update APT package cache.

### EV3Dev tasks
- `ev3devInfo` - Get system summary from `ev3dev-sysinfo -m`.
- `free` - Print free memory summary.
- `getDebianDistro` - Get Debian version information from `/etc/os-release`.
- `ps` - Print list of running processes.
- `stopBluetooth`/`restartBluetooth` - Stop/restart the Bluetooth service.
- `stopBrickman`/`restartBrickman` - Stop/restart the Brickman service.
- `stopNmbd`/`restartNmbd` - Stop/restart the NMBD service.
- `stopNtp`/`restartNtp` - Stop/restart the NTP service.
- `shutdown` - Shut down the brick.

## Javadocs

The project has the following technical documentation

http://ev3dev-lang-java.github.io/docs/api/latest/index.html

## Examples

Exist several examples ready to use here:

https://github.com/ev3dev-lang-java/examples

## Issues

If you have any problem or doubt, use the main projet.

https://github.com/ev3dev-lang-java/ev3dev-lang-java/issues
