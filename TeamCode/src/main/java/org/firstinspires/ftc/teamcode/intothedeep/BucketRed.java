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

        Utility.turnToPID(robot, -27);

        Utility.encoderDrive(robot, Utility.Direction.FORWARD, 46);
        Utility.turnToPID(robot, 0);
        Utility.rightSlide(robot, Utility.Stage.HOME,1.0);
    }

    protected void deliverLeftSample() {

    }

    protected void deliverCenterSample() {

    }

    protected void deliverRightSample() {
        Utility.encoderDrive(robot, Utility.Direction.FORWARD, 0.5, 9);
        Utility.turnToPID(robot, 0);
        Utility.encoderDrive(robot, Utility.Direction.BACKWARD, 0.5, 2);
        Utility.encoderDrive(robot, Utility.Direction.LEFT, 0.5, 1);

        robot.getShoulderServo().setPosition(0.535);
        robot.getElbowServo().setPosition(0.0);
        robot.getWristServo().setPosition(Constants.WRIST_SERVO_90_POSITION);
        sleep(1300);
        robot.getShoulderServo().setPosition(0.547);
        sleep(500);
        robot.getFingerServo().setPosition(Constants.FINGER_SERVO_RUN_POSITION);
        sleep(1000);
        transferSample(1400);
        Utility.turnToPID(robot, -45);
        Utility.encoderDrive(robot, Utility.Direction.BACKWARD, 0.5, 8);

        deliverSample();
   }

    protected void deliverOnRobotSample() {

        transferSample(650);

        Utility.encoderDrive(robot, Utility.Direction.FORWARD, 6);
        Utility.encoderDrive(robot, Utility.Direction.LEFT, 16);
        Utility.turnToPID(robot, -45);
        Utility.encoderDrive(robot, Utility.Direction.BACKWARD, 0.3, 3);

        deliverSample();
    }

    protected void deliverSample() {

        Utility.rightSlide(robot, Utility.Stage.THREE,1.0);
        Utility.encoderDrive(robot, Utility.Direction.BACKWARD, 0.3, 3);
        robot.getOuttakePanServo().setPosition(Constants.OUTTAKE_PAN_SERVO_DELIVERY_POSITION);
        sleep(1200);
        Utility.encoderDrive(robot, Utility.Direction.FORWARD, 0.3, 3);
        robot.getOuttakePanServo().setPosition(Constants.OUTTAKE_PAN_SERVO_RECEIVE_POSITION);
        Utility.rightSlide(robot, Utility.Stage.ZERO,1.0);
    }

    protected void transferSample(int transferDelay) {

        robot.getOuttakePanServo().setPosition(Constants.OUTTAKE_PAN_SERVO_RECEIVE_POSITION);
        robot.getShoulderServo().setPosition(0.44);
        sleep(500);
        robot.getElbowServo().setPosition(Constants.ELBOW_SERVO_DELIVERY_POSITION);
        sleep(500);
        robot.getShoulderServo().setPosition(Constants.SHOULDER_SERVO_DELIVERY_POSITION);

        robot.getWristServo().setPosition(Constants.WRIST_SERVO_90_POSITION);

        sleep(transferDelay);

        robot.getFingerServo().setPosition(Constants.FINGER_SERVO_RUN_OPPOSITE_POSITION);
        sleep(100);
        robot.getFingerServo().setPosition(Constants.FINGER_SERVO_STOP_POSITION);
        sleep(300);
        robot.getShoulderServo().setPosition(0.4);
        sleep(300);
        robot.getElbowServo().setPosition(Constants.ELBOW_SERVO_HOME_POSITION);
        robot.getShoulderServo().setPosition(Constants.SHOULDER_SERVO_HOME_POSITION);
    }
}
