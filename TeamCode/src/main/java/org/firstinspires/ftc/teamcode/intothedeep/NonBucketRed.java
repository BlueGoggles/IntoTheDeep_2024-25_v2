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

        Utility.encoderDrive(robot, Utility.Direction.FORWARD,0.7, 19);
        Utility.turnToPID(robot, 0);
        Utility.encoderDrive(robot, Utility.Direction.LEFT,0.5, 8);
        Utility.turnToPID(robot, 0);

        hangSpecimen();

        Utility.slideSpecimenIntake(robot, Utility.Stage.ZERO, 1.0);
        Utility.turnToPID(robot, -105);

        Utility.encoderDrive(robot, Utility.Direction.FORWARD,0.5, 44);
        Utility.turnToPID(robot, 180);
        Utility.encoderDrive(robot, Utility.Direction.FORWARD,0.2, 6);
        Utility.turnToPID(robot, -180);

        robot.getSpecimenIntakeServo().setPosition(Constants.SPECIMEN_INTAKE_SERVO_CLOSE_POSITION);
        sleep(200);
        Utility.slideSpecimenIntake(robot, Utility.Stage.ONE, 1.0);

        Utility.encoderDrive(robot, Utility.Direction.BACKWARD,0.2, 6);
        Utility.turnToPID(robot, 75);
        Utility.encoderDrive(robot, Utility.Direction.FORWARD,0.5, 44);
        Utility.turnToPID(robot, 0);
        Utility.encoderDrive(robot, Utility.Direction.RIGHT,0.2, 3);

        hangSpecimen();

        Utility.slideSpecimenIntake(robot, Utility.Stage.ZERO, 1.0);
        Utility.turnToPID(robot, -90);

        Utility.encoderDrive(robot, Utility.Direction.FORWARD,0.5, 30);
        Utility.turnToPID(robot, 180);
        Utility.encoderDrive(robot, Utility.Direction.BACKWARD,0.7, 26);
        Utility.turnToPID(robot, 180);
        Utility.encoderDrive(robot, Utility.Direction.LEFT,0.5, 9.5);
        Utility.turnToPID(robot, 180);

        Utility.encoderDrive(robot, Utility.Direction.FORWARD,0.8, 37);
        Utility.turnToPID(robot, 180);

        // Second Push Start
        Utility.encoderDrive(robot, Utility.Direction.BACKWARD,0.7, 43);
        Utility.turnToPID(robot, 180);
        Utility.encoderDrive(robot, Utility.Direction.LEFT,0.5, 11);
        Utility.turnToPID(robot, 180);
        Utility.encoderDrive(robot, Utility.Direction.FORWARD,0.8,43);
        // Second Push End

        Utility.encoderDrive(robot, Utility.Direction.BACKWARD,0.7,1);
        Utility.turnToPID(robot, 0);
        Utility.turnToPID(robot, 0);
        Utility.slideSpecimenIntake(robot, Utility.Stage.HOME, 1.0);
        Utility.encoderDrive(robot, Utility.Direction.BACKWARD,0.7,3);
        Utility.turnToPID(robot, 0);
    }

    protected void hangSpecimen() {

        Utility.slideSpecimenIntake(robot, Utility.Stage.THREE, 1.0);
        Utility.encoderDrive(robot, Utility.Direction.FORWARD,0.5, 5);
        Utility.slideSpecimenIntake(robot, Utility.Stage.TWO, 1.0);
        robot.getSpecimenIntakeServo().setPosition(Constants.SPECIMEN_INTAKE_SERVO_OPEN_POSITION);
        Utility.encoderDrive(robot, Utility.Direction.BACKWARD,0.5, 5);
    }
}
