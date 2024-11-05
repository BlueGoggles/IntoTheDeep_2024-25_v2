package org.firstinspires.ftc.teamcode.intothedeep;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;

@Autonomous(name = "Bucket Red", group = "RedAuton")
public class BucketRed extends LinearOpMode {

    protected RobotHardware robot = new RobotHardware(this);

    @Override
    public void runOpMode() {

        Utility.initializeRobot(robot);
        waitForStart();

//        robot.getBackIntakeServo().setPosition(0.0);
//        robot.getFrontIntakeServo().setPosition(0.0);



        // Drive towards object
        moveToObject(robot);

    }

    protected void moveToObject(RobotHardware robot) {

//        Utility.encoderDrive(robot, Utility.Direction.FORWARD, 10);
//        Utility.encoderDrive(robot, Utility.Direction.RIGHT, 10);
//        Utility.encoderDrive(robot, Utility.Direction.BACKWARD, 10);
//        Utility.encoderDrive(robot, Utility.Direction.LEFT, 10);
//        Utility.turnToPID(robot, 90);
//        Utility.turnToPID(robot, -90);
//
//        sleep(5000);
//        for (int i = 0; i < 90; i+=10)

//        robot.setZeroPowerBehaviorForSlide();
        sleep(5000);

        sleep(5000);
//        Utility.turn(robot, Utility.Direction.FORWARD, 3, 0.5);
        robot.setZeroPowerBehaviorForTurn();
//        Utility.encoderDrive(robot, Utility.Direction.FORWARD, 10);

    }
}