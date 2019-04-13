package example;

import ev3dev.actuators.lego.motors.EV3LargeRegulatedMotor;
import ev3dev.actuators.lego.motors.EV3MediumRegulatedMotor;
import ev3dev.sensors.Battery;
import ev3dev.sensors.ev3.EV3ColorSensor;
import ev3dev.sensors.ev3.EV3IRSensor;
import ev3dev.sensors.ev3.EV3TouchSensor;
import lejos.hardware.port.MotorPort;
import lejos.hardware.port.SensorPort;
import lejos.hardware.sensor.SensorMode;
import lejos.robotics.SampleProvider;
import lejos.utility.Delay;

public class CTruck {

    private static boolean safeRotateTo(EV3MediumRegulatedMotor motor, int rotationAngle, int rotationSpeed, String name) {
        int count = 0;
        int tachocount = 0;
        int prevtachocount;
        boolean rotationDirectionForward;

        //constant for deciding if there enter speedy or precision rotation
        int minRotationConstant = 200;
        int precisionAdjustmentSpeed = 100;

        boolean stopFlag = false;
        int stopLiftConstant = rotationAngle/(rotationSpeed/20)+50;


        //check direction of rotation
        if (tachocount < rotationAngle) {
            rotationDirectionForward = true;
        }
        else {
            rotationDirectionForward = false;
        }

        count = 0;
        System.out.println(name + "$ -rotate motor -angle: " + rotationAngle);
        motor.setSpeed(rotationSpeed);
        motor.rotate(rotationAngle, true);
        prevtachocount = 0;
        tachocount = motor.getTachoCount();

        //enter main loop only if there is enough angle to rotate, else go for adjust mode
        if (rotationAngle > minRotationConstant) {

            //rotationAngle-50 ensures that we start to stop before reaching rotation angle, to allow motor time to stop
            //Fail safe check - motor.isMoving() checks if motor is rotating, if not, exit
            //Fail safe check - stopLiftConstant - for whatever reason motor doesn't stop and while is looping, stopLiftConstant ensures that there will be timeout

            while (tachocount < (rotationAngle-(minRotationConstant/2)) && (motor.isMoving() && count <= stopLiftConstant ) ) {

                prevtachocount = tachocount;
                tachocount = motor.getTachoCount();

                //Fail safe check - check if we are going in right direction, based on rotationDirectionForward variable
                if (rotationDirectionForward) {
                    if (prevtachocount > tachocount) {
                        count = stopLiftConstant;
                        stopFlag = true;
                    }
                }
                else {
                    if (prevtachocount < tachocount) {
                        count = stopLiftConstant;
                        stopFlag = true;
                    }
                }

                count++;
                Delay.msDelay(20);
            }

            motor.stop();
            Delay.msDelay(100); //thread sleep to allow motor time to coast to stop.

            tachocount = motor.getTachoCount();
            System.out.println(name + "$ -motor rotated -angle: " + rotationAngle + " -tacos: " + tachocount);
        }

        //check if previous operation was successful, if not, don't do adjustment
        if (!stopFlag) {
            //adjust mode, does final adjustment of rotation in slower speed for more precision
            if (tachocount < rotationAngle) {
                System.out.println(name + "$ -adjust motor position -for angle: " + (rotationAngle-tachocount));
                motor.setSpeed(precisionAdjustmentSpeed);
                motor.rotate((rotationAngle-tachocount), true);
                while ((tachocount < rotationAngle) && motor.isMoving()) {
                    Delay.msDelay(10);
                }
            }
            motor.stop();
            Delay.msDelay(100);
            System.out.println(name + "$ -motor rotated -final tacos: " + motor.getTachoCount());
        }

        if (stopFlag) {
            return false;
        }
        else {
            return true;
        }
    }

    private static boolean safeRotateTo(EV3LargeRegulatedMotor motor, int rotationAngle, int rotationSpeed, String name) {

        int count = 0;
        int tachocount = motor.getTachoCount();
        int prevtachocount;
        boolean rotationDirectionForward;

        //constant for deciding if there enter speedy or precision rotation
        int minRotationConstant = 200;
        int precisionAdjustmentSpeed = 100;

        boolean stopFlag = false;
        int stopLiftConstant = rotationAngle/(rotationSpeed/40)+50;


        //check direction of rotation
        if (rotationAngle > tachocount) {
            rotationDirectionForward = true;
        }
        else {
            rotationDirectionForward = false;
        }

        //enter main loop only if there is enough angle to rotate, else go for adjust mode
        if (Math.abs(rotationAngle) > minRotationConstant) {

            count = 0;
            System.out.println(name + "$ -rotate motor -angle: " + rotationAngle + " -direction: " + rotationDirectionForward);
            motor.setSpeed(rotationSpeed);
            motor.rotateTo(rotationAngle, true);
            tachocount = motor.getTachoCount();

            //rotationAngle-50 ensures that we start to stop before reaching rotation angle, to allow motor time to stop
            //Fail safe check - motor.isMoving() checks if motor is rotating, if not, exit
            //Fail safe check - stopLiftConstant - for whatever reason motor doesn't stop and while is looping, stopLiftConstant ensures that there will be timeout

            if (rotationDirectionForward) {
                while (tachocount < (rotationAngle-(minRotationConstant/2)) && (motor.isMoving() && count <= stopLiftConstant ) ) {
                    prevtachocount = tachocount;
                    //Fail safe check - check if we are going in right direction, based on rotationDirectionForward variable
                    if (prevtachocount > tachocount) {
                        count = stopLiftConstant;
                        stopFlag = true;
                    }
                    count++;
                    Delay.msDelay(10);
                    tachocount = motor.getTachoCount();
                }
            } else {
                while (tachocount > (rotationAngle+(minRotationConstant/2)) && (motor.isMoving() && count <= stopLiftConstant ) ) {

                    prevtachocount = tachocount;
                    //Fail safe check - check if we are going in right direction, based on rotationDirectionForward variable

                    if (prevtachocount < tachocount) {
                        count = stopLiftConstant;
                        stopFlag = true;
                    }
                    count++;
                    Delay.msDelay(10);
                    tachocount = motor.getTachoCount();
                }
            }

            motor.stop();
            Delay.msDelay(100); //thread sleep to allow motor time to coast to stop.

            tachocount = motor.getTachoCount();
            System.out.println(name + "$ -crane motor -angle: " + rotationAngle + " -tacos: " + tachocount);
        }

        //check if previous operation was successful, if not, don't do adjustment
        if (!stopFlag) {
            //adjust mode, does final adjustment of rotation in slower speed for more precision
            motor.setSpeed(precisionAdjustmentSpeed);
            System.out.println(name + "$ -adjust motor position -for angle: " + (rotationAngle - tachocount) + " -direction: " + rotationDirectionForward);
            motor.rotate((rotationAngle - tachocount), true);

            if (rotationDirectionForward) {
                if (tachocount < rotationAngle) {
                    while ((tachocount < rotationAngle) && motor.isMoving()) {
                        Delay.msDelay(10);
                    }
                }
            }
            else {
                if (tachocount > rotationAngle) {
                    while ((tachocount > rotationAngle) && motor.isMoving()) {
                        Delay.msDelay(10);
                    }
                }
            }
            motor.stop();
            Delay.msDelay(100);
            System.out.println(name + "$ -crane rotated -final tacos: " + motor.getTachoCount());
        }

        if (stopFlag) {
            return false;
        }
        else {
            return true;
        }
    }


    private static boolean rotateToZero(EV3MediumRegulatedMotor motor, int rotationSpeed, String name) {

        int tachocount;


        System.out.println(name + "$ -return motor to starting position -tacos-" + motor.getTachoCount());
        motor.setSpeed(rotationSpeed);
        motor.rotateTo(0, true);
        while (motor.isMoving()) {
            Delay.msDelay(50);
        }
        motor.stop();
        Delay.msDelay(100);

        tachocount = motor.getTachoCount();
        System.out.println(name + "$ -check for final adjustment -tacos-" + tachocount);
        motor.setSpeed(100);
        motor.rotate(0-tachocount, true);
        while (motor.isMoving()) {
            Delay.msDelay(10);
        }
        System.out.println(name + "$ -motor returned to starting position -tacos-" + motor.getTachoCount());

        return true;
    }

    private static boolean rotateToZero(EV3LargeRegulatedMotor motor, int rotationSpeed, String name) {

        int tachocount;


        System.out.println(name + "$ -return motor to starting position -tacos-" + motor.getTachoCount());
        motor.setSpeed(rotationSpeed);
        motor.rotateTo(0, true);
        while (motor.isMoving()) {
            Delay.msDelay(20);
        }
        motor.stop();
        Delay.msDelay(100);

        tachocount = motor.getTachoCount();
        System.out.println(name + "$ -check for final adjustment -tacos-" + tachocount);
        motor.setSpeed(200);
        motor.rotate(0-tachocount, true);
        while (motor.isMoving()) {
            Delay.msDelay(10);
        }
        System.out.println(name + "$ -motor returned to starting position -tacos-" + motor.getTachoCount());

        return true;
    }



    //motor for drive forwards and backwards - connected to motor port D //TODO: port?
    public static EV3LargeRegulatedMotor motorDrive;
    //motor for steering - connected to motor port C //TODO: port?
    public static EV3MediumRegulatedMotor motorSteer;

    //motor for crane lifting - connected to motor port //TODO: port?
    public static EV3LargeRegulatedMotor craneLift;
    //motor for grabber - connected to motor port A
    public static EV3MediumRegulatedMotor craneGrabber;

    //sensor for proximity - connect to sensor port S1
    public static EV3IRSensor sensorProximity;
    //sensor for line reading - connected to sensor port S4
    public static LineReaderV2 lineReader;

    public static void main(final String[] args){

        int[] values = new int[8];
        int pidPrev;
        int pidValue;
        int pidAverage;
        int stop;
        int pidflag;
        int count = 0;
        int tachocount = 0;
        int prevtachocount;
        int motorDriveSpeed = 50;

        int craneLiftSpeed = 500;
        int lowerCraneConstant = 7500;
        int grabberConstant = 9500;
        int craneLiftDriveConstant = 2000;
        int stopLiftConstant = 200;

        //initialize all motors here
        motorDrive = new EV3LargeRegulatedMotor(MotorPort.B);
        motorSteer = new EV3MediumRegulatedMotor(MotorPort.C);
        craneLift = new EV3LargeRegulatedMotor(MotorPort.D);
        craneGrabber = new EV3MediumRegulatedMotor(MotorPort.A);
        System.out.println("Motor initialized");
        //initialize all sensors here
        lineReader = new LineReaderV2(SensorPort.S4);
        sensorProximity = new EV3IRSensor(SensorPort.S1);
        System.out.println("Sensors initialized");

        System.out.println("Checking Battery");
        System.out.println("Votage: " + Battery.getInstance().getVoltage());

        safeRotateTo(craneLift, lowerCraneConstant,500,"craneLift");

        safeRotateTo(craneGrabber, grabberConstant,500,"craneGrabber");

        safeRotateTo(craneLift, craneLiftDriveConstant,500,"craneLift");

        Delay.msDelay(1000);


        Delay.msDelay(1000);
/*        motorSteer.setSpeed(300);
        motorSteer.rotateTo(-80, true);
        Delay.msDelay(200);

        safeRotateTo(motorDrive, 1800,200,"motorDrive");

        Delay.msDelay(500);

        motorSteer.setSpeed(300);
        motorSteer.rotateTo(80, true);
        Delay.msDelay(200);
        safeRotateTo(motorDrive, 0,200,"motorDrive");

        Delay.msDelay(500);

        motorSteer.setSpeed(300);
        motorSteer.rotateTo(0, true);
        Delay.msDelay(200);

        Delay.msDelay(1000); */

        lineReader.wake();
        values = lineReader.getCALValues();
        Delay.msDelay(200);
        values = lineReader.getCALValues();
        pidValue = NewPID.calculatePID(values, 60);

        System.out.print(" V0: " + values[0]);
        System.out.print(" V1: " + values[1]);
        System.out.print(" V2: " + values[2]);
        System.out.print(" V3: " + values[3]);
        System.out.print(" V4: " + values[4]);
        System.out.print(" V5: " + values[5]);
        System.out.print(" V6: " + values[6]);
        System.out.print(" V7: " + values[7]);
        System.out.println(" pidValue: " + pidValue);

        //start moving only if truck is on line
        if (pidValue < 25 & pidValue > -25) {
            System.out.println("move: " + pidValue);
            motorDriveSpeed = 100;
            motorDrive.setSpeed(motorDriveSpeed);
            motorDrive.backward();
            motorSteer.setSpeed(500);
        }

        int stopnum = 100;
        pidAverage = 0;

        for(int i = 0; i < stopnum; i++){

            pidPrev = pidAverage;
            pidValue = 0;
            pidAverage = 0;
            pidflag = 0;

            for (int j = 0; j <= 2; j++)  {
                values = lineReader.getCALValues();
                System.out.print(" V0: " + values[0]);
                System.out.print(" V1: " + values[1]);
                System.out.print(" V2: " + values[2]);
                System.out.print(" V3: " + values[3]);
                System.out.print(" V4: " + values[4]);
                System.out.print(" V5: " + values[5]);
                System.out.print(" V6: " + values[6]);
                System.out.print(" V7: " + values[7]);

                pidValue = NewPID.calculatePID(values, 60);
                System.out.println(" -i" + i +" pidValue: " + pidValue);

                if (pidValue == -100) {
                    pidflag = 1;
                }
                else {
                    pidAverage = pidAverage + pidValue;
                }
            }

            if (pidflag == 1) {
                pidAverage = pidPrev;
            }
            else {
                pidValue = pidAverage / 2;
            }


            stop = (values[0] + values[1] + values[2] + values [3] + values [4] + values[5] + values[6] + values[7]) / 8;



            if ((stop < 35))
            {
                System.out.println("followTheLine$ Stop: " + stop);
                motorDrive.stop();
                i = stopnum;
            }
            else {

                if (pidValue > 69) {
                    motorSteer.rotateTo(-69, true);
                }
                else if (pidValue < -69) {
                    motorSteer.rotateTo(69, true);
                }
                else {
                    motorSteer.rotateTo(-pidValue, true);
                }

                Delay.msDelay(40);

                if (((pidValue < -20) | (pidValue > 20)) && (motorDriveSpeed == 100)) {
                    motorDriveSpeed = 50;
                    motorDrive.setSpeed(motorDriveSpeed);
                }
                if (((pidValue > -20) & (pidValue < 20)) && (motorDriveSpeed == 50)) {
                    motorDriveSpeed = 100;
                    motorDrive.setSpeed(motorDriveSpeed);
                }

            }

        }

        motorDrive.stop(true);

        motorSteer.setSpeed(50);
        motorSteer.rotateTo(0, true);
        System.out.println(motorSteer.getTachoCount());

        Delay.msDelay(500);

        safeRotateTo(craneLift, lowerCraneConstant,500,"craneLift");

        rotateToZero(craneGrabber, craneLiftSpeed, "craneGrabber");

        rotateToZero(craneLift, craneLiftSpeed, "craneLift");

        lineReader.sleep();


        System.exit(0);
    }
}
