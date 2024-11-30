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

        Utility.encoderDrive(robot, Utility.Direction.BACKWARD, 25);
        sleep(5000);
//        deliverOnRobotSample();
//        deliverRightSample();
//        deliverCenterSample();
//        deliverLeftSample();
//        parkRobot();

    }

    protected void parkRobot() {

//        Utility.turnIntakePan(robot, Constants.INTAKE_PAN_MOTOR_HOME_POSITION);
//        sleep(2000);

        Utility.encoderDrive(robot, Utility.Direction.FORWARD, 20);
        robot.getShoulderServo().setPosition(0.45);
        Utility.turnToPID(robot, 90);
        Utility.encoderDrive(robot, Utility.Direction.BACKWARD, 8);
    }

    protected void deliverLeftSample() {

    }

    protected void deliverCenterSample() {

    }

    protected void deliverRightSample() {
        Utility.encoderDrive(robot, Utility.Direction.FORWARD, 1.5);
        Utility.encoderDrive(robot, Utility.Direction.RIGHT, 1.5);

        robot.getShoulderServo().setPosition(Constants.SHOULDER_SERVO_PICKUP_POSITION);
//        Utility.turnIntakePan(robot, Constants.INTAKE_PAN_MOTOR_PICKUP_POSITION);

        robot.getFrontIntakeServo().setPosition(Constants.INTAKE_SERVO_IN_POSITION);
        robot.getBackIntakeServo().setPosition(Constants.INTAKE_SERVO_IN_POSITION);
        Utility.encoderDrive(robot, Utility.Direction.FORWARD, 0.5);
        sleep(2000);

//        Utility.turnIntakePan(robot, Constants.INTAKE_PAN_MOTOR_CARRY_POSITION);
        robot.getShoulderServo().setPosition(0.45);
        sleep(500);
//        Utility.turnIntakePan(robot, Constants.INTAKE_PAN_MOTOR_PICKUP_POSITION);
        robot.getShoulderServo().setPosition(Constants.SHOULDER_SERVO_DELIVERY_POSITION);

        robot.getFrontIntakeServo().setPosition(Constants.INTAKE_SERVO_STOP_POSITION);
        robot.getBackIntakeServo().setPosition(Constants.INTAKE_SERVO_STOP_POSITION);

        Utility.turnToPID(robot, -45);
        Utility.encoderDrive(robot, Utility.Direction.BACKWARD, 10);

        sleep(500);

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
        Utility.encoderDrive(robot, Utility.Direction.BACKWARD, 0.3, 7);

        deliverSample();
    }

    protected void deliverSample() {
        robot.getShoulderServo().setPosition(0.5);
        sleep(350);
//        Utility.slide(robot, Utility.Direction.FORWARD, Constants.AUTON_DRIVE_SPEED);
//        Utility.turnIntakePan(robot, Constants.INTAKE_PAN_MOTOR_HOME_POSITION);

        robot.getLeftSlideServo().setPosition(Constants.SLIDE_SERVO_DELIVERY_POSITION);
        sleep(1000);
        robot.getLeftSlideServo().setPosition(Constants.SLIDE_SERVO_HOME_POSITION);

//        Utility.slide(robot, Utility.Direction.BACKWARD, Constants.AUTON_DRIVE_SPEED);

        Utility.encoderDrive(robot, Utility.Direction.FORWARD, 8);
        Utility.turnToPID(robot, 0);
    }
}
