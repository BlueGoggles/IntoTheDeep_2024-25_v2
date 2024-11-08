package org.firstinspires.ftc.teamcode.intothedeep;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Bucket Red", group = "RedAuton")
public class BucketRed extends LinearOpMode {

    protected RobotHardware robot = new RobotHardware(this);

    @Override
    public void runOpMode() {

        Utility.initializeRobot(robot);

        waitForStart();

        deliverOnRobotSample();
        deliverRightSample();
//        deliverCenterSample();
//        deliverLeftSample();
        parkRobot();

    }

    protected void parkRobot() {

        robot.getIntakePanServo().setPosition(Constants.INTAKE_PAN_SERVO_HOME_POSITION);
//        sleep(2000);

        Utility.encoderDrive(robot, Utility.Direction.FORWARD, 20);
        robot.getShoulderServo().setPosition(0.55);
        Utility.turnToPID(robot, 90);
        Utility.encoderDrive(robot, Utility.Direction.BACKWARD, 8);
    }

    protected void deliverLeftSample() {

    }

    protected void deliverCenterSample() {

    }

    protected void deliverRightSample() {
//        Utility.encoderDrive(robot, Utility.Direction.BACKWARD, 3);

        robot.getShoulderServo().setPosition(Constants.SHOULDER_SERVO_PICKUP_POSITION);
        robot.getIntakePanServo().setPosition(Constants.INTAKE_PAN_SERVO_PICKUP_POSITION);

        robot.getFrontIntakeServo().setPosition(Constants.INTAKE_SERVO_IN_POSITION);
        robot.getBackIntakeServo().setPosition(Constants.INTAKE_SERVO_IN_POSITION);
        sleep(4000);

        robot.getIntakePanServo().setPosition(Constants.INTAKE_PAN_SERVO_CARRY_POSITION);
        robot.getShoulderServo().setPosition(0.45);
        robot.getIntakePanServo().setPosition(Constants.INTAKE_PAN_SERVO_PICKUP_POSITION);
        robot.getShoulderServo().setPosition(Constants.SHOULDER_SERVO_DELIVERY_POSITION);

        sleep(2000);

        Utility.turnToPID(robot, -45);
        Utility.encoderDrive(robot, Utility.Direction.BACKWARD, 10);

        deliverSample();



//        telemetry.addData("Shoulder Servo: ", robot.getShoulderServo().getPosition());
//        telemetry.addData("Slide Servo: ", robot.getLeftSlideServo().getPosition());
//
//        telemetry.addData("Slide Motor position: ", robot.getLeftSlide().getCurrentPosition());
//
//        telemetry.update();
//        sleep(10000);
    }

    protected void deliverOnRobotSample() {

        Utility.encoderDrive(robot, Utility.Direction.FORWARD, 4);
        Utility.encoderDrive(robot, Utility.Direction.LEFT, 12);
        Utility.turnToPID(robot, -45);
        Utility.encoderDrive(robot, Utility.Direction.BACKWARD, 5);

        deliverSample();
    }

    protected void deliverSample() {
        robot.getShoulderServo().setPosition(0.5);
        sleep(350);
        Utility.slide(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED);

        robot.getLeftSlideServo().setPosition(Constants.SLIDE_SERVO_DELIVERY_POSITION);
        sleep(1500);
        robot.getLeftSlideServo().setPosition(Constants.SLIDE_SERVO_HOME_POSITION);

        robot.getLeftSlideServo().setPosition(Constants.SLIDE_SERVO_HOME_POSITION);
        Utility.slide(robot, Utility.Direction.BACKWARD, Constants.MAX_POWER);

        Utility.encoderDrive(robot, Utility.Direction.FORWARD, 10);
        Utility.turnToPID(robot, 0);
    }
}