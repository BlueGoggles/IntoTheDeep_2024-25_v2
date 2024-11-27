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
                Joystick_X = -1 * gamepad1.right_stick_x;
                Joystick_Y = -1 * gamepad1.right_stick_y;
                Joystick_Z = gamepad1.left_stick_x;
                M = 1 / (1 - Deadband);
                B = -Deadband / (1 - Deadband);
                if (Math.abs(Joystick_X) > Deadband) {
                    Joystick_X = (float) (M * Joystick_X + B);
                } else {
                    Joystick_X = 0;
                }
                if (Math.abs(Joystick_Y) > Deadband) {
                    Joystick_Y = (float) (M * Joystick_Y + B);
                } else {
                    Joystick_Y = 0;
                }
                if (Math.abs(Joystick_Z) > Deadband) {
                    Joystick_Z = (float) (M * Joystick_Z + B);
                } else {
                    Joystick_Z = 0;
                }
                Orientation2 = robot.getImu().getRobotYawPitchRollAngles();
                Theta_Actual = Double.parseDouble(JavaUtil.formatNumber(Orientation2.getYaw(AngleUnit.DEGREES), 2));
                Theta_Velocity = robot.getImu().getRobotAngularVelocity(AngleUnit.DEGREES);
                Speed = Math.sqrt(Math.pow(Joystick_Y, 2) + Math.pow(Joystick_X, 2));
                Theta_Request = Math.atan2(Joystick_Y, Joystick_X) / Math.PI * 180;
                Theta_Command = Theta_Request - (90 - Theta_Actual);
                if (gamepad1.dpad_up) {
                    Target_Angle = 0;
                }
                if (gamepad1.dpad_right) {
                    Target_Angle = -90;
                }
                if (gamepad1.dpad_left) {
                    Target_Angle = 90;
                }
                if (gamepad1.dpad_down) {
                    if (Theta_Actual < 0) {
                        Target_Angle = -180;
                    } else {
                        Target_Angle = 180;
                    }
                }
                if (Math.abs(Target_Angle - Theta_Actual) < 180) {
                    Error2 = (int) (Target_Angle - Theta_Actual);
                } else {
                    if (Target_Angle - Theta_Actual < 0) {
                        Error2 = (int) (Target_Angle - (Theta_Actual - 360));
                    } else {
                        Error2 = (int) (Target_Angle - (Theta_Actual + 360));
                    }
                }
                Z_ = (Error2 * Kp - KD * Theta_Velocity.zRotationRate);
                if (Math.abs(Z_) > Z__Max) {
                    Z_ = (Z__Max * (Z_ / Math.abs(Z_)));
                }

                if( enableManualOverride ) {
                    // Leave Joystick_Z alone.
                } else {
                    Joystick_Z = -Z_;
                }

                Joystick_X = (Math.sin(Theta_Command / 180 * Math.PI) * Speed);
                Joystick_Y = (Math.cos(Theta_Command / 180 * Math.PI) * Speed);

                FL_Power = (-Gain_X * Joystick_X - (Gain_Y * Joystick_Y + Gain_Z * Joystick_Z));
                FR_Power = (-Gain_X * Joystick_X + (Gain_Y * Joystick_Y - Gain_Z * Joystick_Z));
                BL_Power = (Gain_X * Joystick_X - (Gain_Y * Joystick_Y + Gain_Z * Joystick_Z));
                BR_Power = (Gain_X * Joystick_X + (Gain_Y * Joystick_Y - Gain_Z * Joystick_Z));
                if (Math.abs(FR_Power) > Math.abs(FL_Power)) {
                    Max = Math.abs(FR_Power);
                } else {
                    Max = Math.abs(FL_Power);
                }
                if (Math.abs(BL_Power) > Max) {
                    Max = Math.abs(BL_Power);
                }
                if (Math.abs(BR_Power) > Max) {
                    Max = Math.abs(BR_Power);
                }

                if (gamepad1.left_trigger > Constants.ZERO_POWER) {
                    teleOpSpeed = Constants.TELEOP_MODIFIED_SPEED;
                } else {
                    teleOpSpeed = Constants.TELEOP_DEFAULT_SPEED;
                }

                if (Max > teleOpSpeed) {
                    FR_Power = (FR_Power * teleOpSpeed) / Max;
                    FL_Power = (FL_Power * teleOpSpeed) / Max;
                    BR_Power = (BR_Power * teleOpSpeed) / Max;
                    BL_Power = (BL_Power * teleOpSpeed) / Max;
                }

                robot.setMotorPowers(-FL_Power, FR_Power, -BL_Power, BR_Power);


                // NOTE: This program is single threaded right now. So we can't do multiple operations at once.

                if (gamepad2.right_bumper) {
                    Utility.slideSpecimenIntake(robot, Utility.Stage.TWO,0.8);
                }

                if (gamepad2.left_bumper) {
                    Utility.slideSpecimenIntake(robot, Utility.Stage.ZERO,0.8);
                }

                if (gamepad2.left_trigger > 0.5) {
                    Utility.slideSpecimenIntake(robot, Utility.Stage.ONE,0.8);
                }

                if (gamepad1.right_bumper) {
                    Utility.slide(robot, Utility.Direction.FORWARD,0.5);
                }

                if (gamepad1.left_bumper) {
                    Utility.slide(robot, Utility.Direction.BACKWARD,0.5);
                }

                if (gamepad1.a) {
                    robot.getSpecimenIntakeServo().setPosition(Constants.SPECIMEN_INTAKE_SERVO_OPEN_POSITION);
                }

                if (gamepad1.b) {
                    robot.getSpecimenIntakeServo().setPosition(Constants.SPECIMEN_INTAKE_SERVO_CLOSE_POSITION);
                }

                if (gamepad2.a) {
                    robot.getFingerServo().setPosition(Constants.FINGER_SERVO_RUN_POSITION);
                }

                if (gamepad2.b) {
                    robot.getFingerServo().setPosition(Constants.FINGER_SERVO_RUN_OPPOSITE_POSITION);
                    sleep(300);
                    robot.getFingerServo().setPosition(Constants.FINGER_SERVO_STOP_POSITION);
                    robot.getShoulderServo().setPosition(0.6);
                    sleep(300);
                    robot.getElbowServo().setPosition(Constants.ELBOW_SERVO_HOME_POSITION);
                    robot.getShoulderServo().setPosition(Constants.SHOULDER_SERVO_HOME_POSITION);
                }

                if (gamepad2.back) {
                    robot.getFingerServo().setPosition(Constants.FINGER_SERVO_STOP_POSITION);
                }

                if (gamepad2.start) {
                    robot.getShoulderServo().setPosition(Constants.SHOULDER_SERVO_HOME_POSITION);
                }

                if (gamepad2.x) {
                    robot.getShoulderServo().setPosition(Constants.SHOULDER_SERVO_PICKUP_POSITION);
                    robot.getElbowServo().setPosition(Constants.ELBOW_SERVO_PICKUP_POSITION);
                }

                if (gamepad2.y) {
                    robot.getShoulderServo().setPosition(Constants.SHOULDER_SERVO_DELIVERY_POSITION);
                    robot.getElbowServo().setPosition(Constants.ELBOW_SERVO_DELIVERY_POSITION);
                    robot.getWristServo().setPosition(Constants.WRIST_SERVO_HOME_POSITION);
                }
                if (gamepad2.dpad_down) {
                    robot.getWristServo().setPosition(Constants.WRIST_SERVO_HOME_POSITION);
                }
                if (gamepad2.dpad_right) {
                    robot.getWristServo().setPosition(Constants.WRIST_SERVO_45_POSITION);
                }
                if (gamepad2.dpad_up) {
                    robot.getWristServo().setPosition(Constants.WRIST_SERVO_90_POSITION);
                }
                if (gamepad2.dpad_left) {
                    robot.getWristServo().setPosition(Constants.WRIST_SERVO_180_POSITION);
                }


//                if (gamepad1.a) {
//                    robot.getLeftSlideServo().setPosition(Constants.SLIDE_SERVO_HOME_POSITION);
//                }
//
//                if (gamepad1.b) {
//                    robot.getLeftSlideServo().setPosition(Constants.SLIDE_SERVO_DELIVERY_POSITION);
//                }
//
//                if (gamepad1.x) {
//                    robot.getShoulderServo().setPosition(0.5);
//                    sleep(350);
//                    Utility.slide(robot, Utility.Direction.FORWARD, Constants.MAX_POWER);
////                    Utility.turnIntakePan(robot, Constants.INTAKE_PAN_MOTOR_HOME_POSITION);
//                }
//
//                if (gamepad1.y) {
//                    robot.getLeftSlideServo().setPosition(Constants.SLIDE_SERVO_HOME_POSITION);
//                    Utility.slide(robot, Utility.Direction.BACKWARD, Constants.MAX_POWER);
//                }
//
//                if (gamepad2.a) {
////                    Utility.turnIntakePan(robot, Constants.INTAKE_PAN_MOTOR_PICKUP_POSITION);
//                }
//
//                if (gamepad2.b) {
////                    robot.getFrontIntakeServo().setPosition(Constants.INTAKE_SERVO_IN_POSITION);
////                    robot.getBackIntakeServo().setPosition(Constants.INTAKE_SERVO_IN_POSITION);
////                    Utility.turnIntakePan(robot, Constants.INTAKE_PAN_MOTOR_CARRY_POSITION);
//                }
//
//                if (gamepad2.left_trigger > 0.5) {
////                    Utility.turnIntakePan(robot, 50, true);
//                }
//
//                if (gamepad2.right_trigger > 0.5) {
////                    Utility.turnIntakePan(robot, -50, true);
//                }
//
//                double intakeServoPosition = 0.5;
//
////                if (gamepad2.left_bumper) {
////                    intakeServoPosition = Constants.INTAKE_SERVO_OUT_POSITION;
////                } else if (gamepad2.right_bumper) {
////                    intakeServoPosition = Constants.INTAKE_SERVO_IN_POSITION;
////                }
//
//
//
////                robot.getFrontIntakeServo().setPosition(intakeServoPosition);
////                robot.getBackIntakeServo().setPosition(intakeServoPosition);
//
//                if (gamepad2.dpad_down) {
////                    Utility.turnIntakePan(robot, Constants.INTAKE_PAN_MOTOR_CARRY_POSITION);
//                    robot.getShoulderServo().setPosition(Constants.SHOULDER_SERVO_PICKUP_POSITION);
//                }
//
//                if (gamepad2.dpad_right) {
////                    Utility.turnIntakePan(robot, Constants.INTAKE_PAN_MOTOR_CARRY_POSITION);
//                    robot.getShoulderServo().setPosition(0.45);
//                    sleep(500);
////                    Utility.turnIntakePan(robot, Constants.INTAKE_PAN_MOTOR_PICKUP_POSITION);
//                    robot.getShoulderServo().setPosition(Constants.SHOULDER_SERVO_DELIVERY_POSITION);
////                    robot.getFrontIntakeServo().setPosition(Constants.INTAKE_SERVO_STOP_POSITION);
////                    robot.getBackIntakeServo().setPosition(Constants.INTAKE_SERVO_STOP_POSITION);
//                }
//
//
//                if (gamepad2.dpad_up) {
//                    robot.getShoulderServo().setPosition(0.5);
//                }

                // Press this button to reset the yaw during Teleop. Only allow this to happen if we are in manual mode.
                if (gamepad1.y && enableManualOverride) {
                    robot.getImu().resetYaw();
                }

                telemetry.addData("Front Left Power: ", robot.getLeftFront().getPower());
                telemetry.addData("Front Right Power: ", robot.getRightFront().getPower());
                telemetry.addData("Back Left Power: ", robot.getLeftBack().getPower());
                telemetry.addData("Back Right Power: ", robot.getRightBack().getPower());

                telemetry.addData("Left Slide Motor position: ", robot.getLeftSlide().getCurrentPosition());
                telemetry.addData("Right Slide Motor position: ", robot.getRightSlide().getCurrentPosition());

                telemetry.addData("Specimen Intake Motor position: ", robot.getSpecimenIntakeMotor().getCurrentPosition());

//                telemetry.addData("Front Intake Servo: ", robot.getFrontIntakeServo().getPosition());
//                telemetry.addData("Back Intake Servo: ", robot.getBackIntakeServo().getPosition());
//                telemetry.addData("Shoulder Servo: ", robot.getShoulderServo().getPosition());
//                telemetry.addData("Slide Servo: ", robot.getLeftSlideServo().getPosition());

                telemetry.update();
            }
        }
    }
}