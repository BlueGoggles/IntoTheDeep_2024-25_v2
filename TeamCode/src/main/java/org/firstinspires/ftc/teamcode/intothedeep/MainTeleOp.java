package org.firstinspires.ftc.teamcode.intothedeep;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

@TeleOp(name = "MainTeleOp")
public class MainTeleOp extends LinearOpMode {

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @Override
    public void runOpMode() {
        double FL_Power;
        double FR_Power;
        double BL_Power;
        double BR_Power;
        double Gain_X;
        double Gain_Y;
        double Gain_Z;
        double Max;
        double Deadband;
        double M;
        double B;
        double Z_;
        double KD;
        double Kp;
        int Target_Angle;
        double Z__Max;
        double Joystick_X;
        double Joystick_Y;
        double Joystick_Z;
        YawPitchRollAngles Orientation2;
        double Theta_Actual;
        AngularVelocity Theta_Velocity;
        double Speed;
        double Theta_Request;
        double Theta_Command;
        double Error2;
        boolean enableManualOverride;
        double teleOpSpeed;

        // By default we don't want to allow the lead screw to run until it has been released.
        boolean allowLeadScrew = false;
        // By default we don't want to allow the drone to be launched until the timer has elapsed.
        boolean allowDroneLauncher = false;

        // Create the timer to lock out the drone launcher.
        //ElapsedTime droneLauncherWaitTimer = new ElapsedTime();

        RobotHardware robot = new RobotHardware(this);
        robot.initialize();
        robot.initializeIMU();
//        robot.initializeDroneLauncher();
        sleep(400);

        FL_Power = 0;
        FR_Power = 0;
        BL_Power = 0;
        BR_Power = 0;
        Gain_X = 1;
        Gain_Y = 1;
        Gain_Z = 1;
        Max = 0;
        Deadband = 0.05;
        M = 0;
        B = 0;
        Z_ = 0;
        KD = 0.003;
        Kp = 0.024;
        Target_Angle = 0;
        Z__Max = 0.75;
        enableManualOverride = false;
        teleOpSpeed = 0.0;
        robot.getImu().resetYaw();

        waitForStart();

        if (opModeIsActive()) {
            // Reset the timer to start after "Start" is pressed.
            //droneLauncherWaitTimer.reset();

            while (opModeIsActive()) {
                Joystick_X = gamepad1.right_stick_x;
                Joystick_Y = -1 * gamepad1.right_stick_y;
                Joystick_Z = gamepad1.left_stick_x;
//                M = 1 / (1 - Deadband);
//                B = -Deadband / (1 - Deadband);
//                if (Math.abs(Joystick_X) > Deadband) {
//                    Joystick_X = (float) (M * Joystick_X + B);
//                } else {
//                    Joystick_X = 0;
//                }
//                if (Math.abs(Joystick_Y) > Deadband) {
//                    Joystick_Y = (float) (M * Joystick_Y + B);
//                } else {
//                    Joystick_Y = 0;
//                }
//                if (Math.abs(Joystick_Z) > Deadband) {
//                    Joystick_Z = (float) (M * Joystick_Z + B);
//                } else {
//                    Joystick_Z = 0;
//                }
//                Orientation2 = robot.getImu().getRobotYawPitchRollAngles();
//                Theta_Actual = Double.parseDouble(JavaUtil.formatNumber(Orientation2.getYaw(AngleUnit.DEGREES), 2));
//                Theta_Velocity = robot.getImu().getRobotAngularVelocity(AngleUnit.DEGREES);
//                Speed = Math.sqrt(Math.pow(Joystick_Y, 2) + Math.pow(Joystick_X, 2));
//                Theta_Request = Math.atan2(Joystick_Y, Joystick_X) / Math.PI * 180;
//                Theta_Command = Theta_Request - (90 - Theta_Actual);
//                if (gamepad1.dpad_up) {
//                    Target_Angle = 0;
//                }
//                if (gamepad1.dpad_right) {
//                    Target_Angle = -90;
//                }
//                if (gamepad1.dpad_left) {
//                    Target_Angle = 90;
//                }
//                if (gamepad1.dpad_down) {
//                    if (Theta_Actual < 0) {
//                        Target_Angle = -180;
//                    } else {
//                        Target_Angle = 180;
//                    }
//                }
//                if (Math.abs(Target_Angle - Theta_Actual) < 180) {
//                    Error2 = (int) (Target_Angle - Theta_Actual);
//                } else {
//                    if (Target_Angle - Theta_Actual < 0) {
//                        Error2 = (int) (Target_Angle - (Theta_Actual - 360));
//                    } else {
//                        Error2 = (int) (Target_Angle - (Theta_Actual + 360));
//                    }
//                }
//                Z_ = (Error2 * Kp - KD * Theta_Velocity.zRotationRate);
//                if (Math.abs(Z_) > Z__Max) {
//                    Z_ = (Z__Max * (Z_ / Math.abs(Z_)));
//                }
//
//                if( enableManualOverride ) {
//                    // Leave Joystick_Z alone.
//                } else {
//                    Joystick_Z = -Z_;
//                }
//
//                Joystick_X = (Math.sin(Theta_Command / 180 * Math.PI) * Speed);
//                Joystick_Y = (Math.cos(Theta_Command / 180 * Math.PI) * Speed);
                double wheelMotorSpeed = 0.5;
                FL_Power = wheelMotorSpeed * (-Gain_X * Joystick_X - (Gain_Y * Joystick_Y + Gain_Z * Joystick_Z));
                FR_Power = wheelMotorSpeed * (-Gain_X * Joystick_X + (Gain_Y * Joystick_Y - Gain_Z * Joystick_Z));
                BL_Power = wheelMotorSpeed * (Gain_X * Joystick_X - (Gain_Y * Joystick_Y + Gain_Z * Joystick_Z));
                BR_Power = wheelMotorSpeed * (Gain_X * Joystick_X + (Gain_Y * Joystick_Y - Gain_Z * Joystick_Z));
//                if (Math.abs(FR_Power) > Math.abs(FL_Power)) {
//                    Max = Math.abs(FR_Power);
//                } else {
//                    Max = Math.abs(FL_Power);
//                }
//                if (Math.abs(BL_Power) > Max) {
//                    Max = Math.abs(BL_Power);
//                }
//                if (Math.abs(BR_Power) > Max) {
//                    Max = Math.abs(BR_Power);
//                }
//
//                if (gamepad1.left_trigger > Constants.ZERO_POWER) {
//                    teleOpSpeed = Constants.TELEOP_MODIFIED_SPEED;
//                } else {
//                    teleOpSpeed = Constants.TELEOP_DEFAULT_SPEED;
//                }
//
//                if (Max > teleOpSpeed) {
//                    FR_Power = (FR_Power * teleOpSpeed) / Max;
//                    FL_Power = (FL_Power * teleOpSpeed) / Max;
//                    BR_Power = (BR_Power * teleOpSpeed) / Max;
//                    BL_Power = (BL_Power * teleOpSpeed) / Max;
//                }

                robot.setMotorPowers(-FL_Power, FR_Power, -BL_Power, BR_Power);


                // NOTE: This program is single threaded right now. So we can't do multiple operations at once.

                if (gamepad1.a) {
                    robot.getLeftSlideServo().setPosition(Constants.SLIDE_SERVO_HOME_POSITION);
                }

                if (gamepad1.b) {
                    robot.getLeftSlideServo().setPosition(Constants.SLIDE_SERVO_DELIVERY_POSITION);
                }

                if (gamepad1.x) {
                    robot.getShoulderServo().setPosition(0.5);
                    sleep(200);
                    Utility.slide(robot, Utility.Direction.FORWARD, 1.0);
                }

                if (gamepad1.y) {
                    robot.getLeftSlideServo().setPosition(Constants.SLIDE_SERVO_HOME_POSITION);
                    Utility.slide(robot, Utility.Direction.BACKWARD, 1.0);
                    robot.getIntakePanServo().setPosition(0.0);
                }

//                if (gamepad2.y) {
//                    robot.getLeftSlideServo().setPosition(Constants.SLIDE_SERVO_DELIVERY_POSITION);
//                    robot.getRightSlideServo().setPosition(Constants.SLIDE_SERVO_DELIVERY_POSITION);
//                }

                if (gamepad2.a) {
                    robot.getIntakePanServo().setPosition(Constants.INTAKE_PAN_SERVO_PICKUP_POSITION);
                }

                if (gamepad2.b) {
                    robot.getIntakePanServo().setPosition(Constants.INTAKE_PAN_SERVO_CARRY_POSITION);
                }

                if (gamepad2.y) {
                    robot.getIntakePanServo().setPosition(Constants.INTAKE_PAN_SERVO_DELIVERY_POSITION);
                }

                double intakeServoPosition = 0.5;

                if (gamepad2.left_bumper) {
                    intakeServoPosition = Constants.INTAKE_SERVO_OUT_POSITION;
                } else if (gamepad2.right_bumper) {
                    intakeServoPosition = Constants.INTAKE_SERVO_IN_POSITION;
                }

                robot.getFrontIntakeServo().setPosition(intakeServoPosition);
                robot.getBackIntakeServo().setPosition(intakeServoPosition);

                if (gamepad2.dpad_down) {
//                    robot.getIntakePanServo().setPosition(Constants.INTAKE_PAN_SERVO_CARRY_POSITION);
                    robot.getShoulderServo().setPosition(Constants.SHOULDER_SERVO_PICKUP_POSITION);
                }

                if (gamepad2.dpad_right) {
                    robot.getIntakePanServo().setPosition(Constants.INTAKE_PAN_SERVO_CARRY_POSITION);
                    robot.getShoulderServo().setPosition(0.45);
                    robot.getIntakePanServo().setPosition(Constants.INTAKE_PAN_SERVO_PICKUP_POSITION);
                    robot.getShoulderServo().setPosition(Constants.SHOULDER_SERVO_DELIVERY_POSITION);
                }


                if (gamepad2.dpad_up) {
                    robot.getShoulderServo().setPosition(0.5);
                }

//                if( gamepad2.left_trigger > 0.5 ) {
//                    Utility.slide(robot, Utility.Direction.BACKWARD, 50, 1.0);
//                }
//
//                if( gamepad2.right_trigger > 0.5 ) {
//                    Utility.slide(robot, Utility.Direction.FORWARD, 50,1.0);
//                }
//
//                if( gamepad2.start) {
//                    Utility.slide(robot, Utility.Direction.BACKWARD, 100, 1.0);
//                }
//
//                if( gamepad2.back) {
//                    Utility.slide(robot, Utility.Direction.FORWARD, 100,1.0);
//                }
//
//                if ( gamepad1.right_bumper ) {
//                    Utility.turn(robot, Utility.Direction.FORWARD, 8,0.1);
//                }
//
//                if ( gamepad1.left_bumper) {
//                    Utility.turn(robot, Utility.Direction.BACKWARD, 8, 0.1);
//                }

                // Press this button to reset the yaw during Teleop. Only allow this to happen if we are in manual mode.
                if (gamepad1.y && enableManualOverride) {
                    robot.getImu().resetYaw();
                }

                telemetry.addData("Front Left Power: ", robot.getLeftFront().getPower());
                telemetry.addData("Front Right Power: ", robot.getRightFront().getPower());
                telemetry.addData("Back Left Power: ", robot.getLeftBack().getPower());
                telemetry.addData("Back Right Power: ", robot.getRightBack().getPower());

                telemetry.addData("Slide Motor position: ", robot.getLeftSlide().getCurrentPosition());
                telemetry.addData("Front Intake Servo: ", robot.getFrontIntakeServo().getPosition());
                telemetry.addData("Back Intake Servo: ", robot.getBackIntakeServo().getPosition());

                telemetry.update();
            }
        }
    }
}