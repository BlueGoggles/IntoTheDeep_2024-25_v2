package org.firstinspires.ftc.teamcode.intothedeep;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous(name = "Non Bucket Red", group = "RedAuton")
public class NonBucketRed extends LinearOpMode {

    int initialWaitTime = 3000;
    protected RobotHardware robot = new RobotHardware(this);

    @Override
    public void runOpMode() {

        Utility.initializeRobot(robot);

        waitForStart();

        deliverOnRobotSample();
        parkRobot();
    }

    protected void parkRobot() {

        Utility.encoderDrive(robot, Utility.Direction.RIGHT, 0.5,70);
    }
    protected void deliverOnRobotSample() {

        sleep(initialWaitTime);
        Utility.encoderDrive(robot, Utility.Direction.FORWARD, 0.5, 4);
        Utility.encoderDrive(robot, Utility.Direction.LEFT, 0.5,46);
        Utility.turnToPID(robot, -45);
        Utility.encoderDrive(robot, Utility.Direction.BACKWARD, 0.5,5);

        deliverSample();
    }

    protected void deliverSample() {
        robot.getShoulderServo().setPosition(0.5);
        sleep(350);
        Utility.slide(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED);
        Utility.turnIntakePan(robot, Constants.INTAKE_PAN_MOTOR_HOME_POSITION);

        robot.getLeftSlideServo().setPosition(Constants.SLIDE_SERVO_DELIVERY_POSITION);
        sleep(1000);
        robot.getLeftSlideServo().setPosition(Constants.SLIDE_SERVO_HOME_POSITION);

        Utility.slide(robot, Utility.Direction.BACKWARD, Constants.AUTON_DRIVE_SPEED);

        Utility.encoderDrive(robot, Utility.Direction.FORWARD, 0.5,5);
        Utility.turnToPID(robot, 0);
    }
}
