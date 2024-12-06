package org.firstinspires.ftc.teamcode.intothedeep;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Non Bucket Red", group = "RedAuton")
public class NonBucketRed extends LinearOpMode {

    int initialWaitTime = 3000;
    protected RobotHardware robot = new RobotHardware(this);

    @Override
    public void runOpMode() {

        Utility.initializeRobot(robot);


        waitForStart();

        hangOnRobotSpecimen();
//        parkRobot();
    }

    protected void parkRobot() {

        Utility.encoderDrive(robot, Utility.Direction.BACKWARD,0.7, 20);
        Utility.turnToPID(robot, 0);
        Utility.encoderDrive(robot, Utility.Direction.RIGHT,0.7, 40);
        Utility.turnToPID(robot, 0);
    }

    protected void hangOnRobotSpecimen() {

        Utility.encoderDrive(robot, Utility.Direction.FORWARD,0.7, 20);
        Utility.turnToPID(robot, 0);
        Utility.encoderDrive(robot, Utility.Direction.LEFT,0.5, 8);
        Utility.turnToPID(robot, 0);

        hangSpecimen();

        Utility.slideSpecimenIntake(robot, Utility.Stage.ZERO, 1.0);
        Utility.turnToPID(robot, -90);

        Utility.encoderDrive(robot, Utility.Direction.FORWARD,0.5, 41);
        Utility.turnToPID(robot, 180);
        Utility.encoderDrive(robot, Utility.Direction.FORWARD,0.5, 14);
        Utility.encoderDrive(robot, Utility.Direction.FORWARD,0.2, 8);
        Utility.turnToPID(robot, -180);

        robot.getSpecimenIntakeServo().setPosition(Constants.SPECIMEN_INTAKE_SERVO_CLOSE_POSITION);
        sleep(200);
        Utility.slideSpecimenIntake(robot, Utility.Stage.ONE, 1.0);

        Utility.encoderDrive(robot, Utility.Direction.BACKWARD,0.5, 6);
        Utility.turnToPID(robot, 90);
        Utility.encoderDrive(robot, Utility.Direction.FORWARD,0.5, 36);
        Utility.turnToPID(robot, 0);
        Utility.encoderDrive(robot, Utility.Direction.FORWARD,0.5, 10);
        Utility.turnToPID(robot, 0);

        hangSpecimen();

        Utility.slideSpecimenIntake(robot, Utility.Stage.HOME, 1.0);
        Utility.turnToPID(robot, 0);

        Utility.encoderDrive(robot, Utility.Direction.RIGHT,0.5, 34);
        Utility.turnToPID(robot, 0);
        Utility.encoderDrive(robot, Utility.Direction.FORWARD,0.5, 24);
        Utility.turnToPID(robot, 180);
        Utility.encoderDrive(robot, Utility.Direction.LEFT,0.5, 10);
        Utility.turnToPID(robot, 180);

        Utility.encoderDrive(robot, Utility.Direction.FORWARD,0.5, 40);
        Utility.turnToPID(robot, 180);
        Utility.encoderDrive(robot, Utility.Direction.BACKWARD,0.5, 40);
        Utility.turnToPID(robot, 180);
        Utility.encoderDrive(robot, Utility.Direction.LEFT,0.5, 8);
        Utility.turnToPID(robot, 180);
        Utility.encoderDrive(robot, Utility.Direction.FORWARD,0.5,40);
        Utility.turnToPID(robot, 0);

    }

    protected void hangSpecimen() {

        Utility.slideSpecimenIntake(robot, Utility.Stage.THREE, 1.0);
        Utility.encoderDrive(robot, Utility.Direction.FORWARD,0.5, 5);
        Utility.slideSpecimenIntake(robot, Utility.Stage.TWO, 1.0);
        robot.getSpecimenIntakeServo().setPosition(Constants.SPECIMEN_INTAKE_SERVO_OPEN_POSITION);
        Utility.encoderDrive(robot, Utility.Direction.BACKWARD,0.7, 5);
    }
}
