#!/bin/sh

cat /home/robot/java/splash.txt
java -Xms64m -Xmx64m -XX:+UseSerialGC -Xshare:off -noverify -cp "/home/robot/java/programs/template_project_gradle-2.5.3-all.jar" example.MyFirstRobot
exit $?
