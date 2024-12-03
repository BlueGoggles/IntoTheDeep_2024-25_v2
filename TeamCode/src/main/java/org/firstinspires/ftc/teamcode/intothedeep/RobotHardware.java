package org.firstinspires.ftc.teamcode.intothedeep;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;

public class RobotHardware {

    private final LinearOpMode myOpMode;

    private DcMotorEx leftFront = null;
    private DcMotorEx rightFront = null;
    private DcMotorEx leftBack = null;
    private DcMotorEx rightBack = null;
    private DcMotorEx leftSlide = null;
    private DcMotorEx rightSlide = null;
    private DcMotorEx specimenIntakeMotor = null;

    //    private DcMotorEx viperSlide = null;
//    private DcMotorEx leadScrew = null;
//    private Servo rightSlideServo = null;
    private Servo leftSlideServo = null;
    private Servo shoulderServo = null;
    private Servo elbowServo = null;
    private Servo wristServo = null;
    private Servo fingerServo = null;

    private Servo frontIntakeServo = null;
    private Servo backIntakeServo = null;
    private Servo intakePanServo = null;

    private Servo specimenIntakeServo = null;
    private Servo outtakePanServo = null;

    public static int LEFT_SLIDE_LOWER_LIMIT = 0;
    public static int LEFT_SLIDE_UPPER_LIMIT = 0;

    public static int RIGHT_SLIDE_LOWER_LIMIT = 0;
    public static int RIGHT_SLIDE_UPPER_LIMIT = 0;

    public static int SPECIMEN_INTAKE_MOTOR_SLIDE_LOWER_LIMIT = 0;
    public static int SPECIMEN_INTAKE_MOTOR_SLIDE_UPPER_LIMIT = 0;

    private IMU imu = null;

    private OpenCvCamera controlHubCam = null; // Use OpenCvCamera class from FTC SDK

    private AprilTagProcessor aprilTag = null; // Used for managing the AprilTag detection process.
    private VisionPortal visionPortal = null; // Used to manage the video source.

    // Calculate the COUNTS_PER_INCH for your specific drive train.
    // Go to your motor vendor website to determine your motor's COUNTS_PER_MOTOR_REV
    // For external drive gearing, set DRIVE_GEAR_REDUCTION as needed.
    // For example, use a value of 2.0 for a 12-tooth spur gear driving a 24-tooth spur gear.
    // This is gearing DOWN for less speed and more torque.
    // For gearing UP, use a gear ratio less than 1.0. Note this will affect the direction of wheel rotation.
    static final double COUNTS_PER_INCH = (Constants.COUNTS_PER_MOTOR_REV * Constants.DRIVE_GEAR_REDUCTION) / (Constants.WHEEL_DIAMETER_INCHES * 3.1415);

    // Define a constructor that allows the OpMode to pass a reference to itself.
    public RobotHardware(LinearOpMode opMode) {
        myOpMode = opMode;
    }

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     * <p>
     * All of the hardware devices are accessed via the hardware map, and initialized.
     */
    public void initialize()    {

        // Drive motors
        leftFront  = myOpMode.hardwareMap.get(DcMotorEx.class, Constants.DEVICE_FRONT_LEFT);
        rightFront = myOpMode.hardwareMap.get(DcMotorEx.class, Constants.DEVICE_FRONT_RIGHT);
        leftBack  = myOpMode.hardwareMap.get(DcMotorEx.class, Constants.DEVICE_BACK_LEFT);
        rightBack = myOpMode.hardwareMap.get(DcMotorEx.class, Constants.DEVICE_BACK_RIGHT);
        // Expansion hub motors
        leftSlide  = myOpMode.hardwareMap.get(DcMotorEx.class, Constants.DEVICE_LEFT_SLIDE);
        rightSlide  = myOpMode.hardwareMap.get(DcMotorEx.class, Constants.DEVICE_RIGHT_SLIDE);

        specimenIntakeMotor  = myOpMode.hardwareMap.get(DcMotorEx.class, Constants.SPECIMEN_INTAKE_MOTOR);

        getLeftFront().setDirection(DcMotorEx.Direction.REVERSE);
        getRightFront().setDirection(DcMotorEx.Direction.FORWARD);
        getLeftBack().setDirection(DcMotorEx.Direction.REVERSE);
        getRightBack().setDirection(DcMotorEx.Direction.FORWARD);

        getLeftSlide().setDirection(DcMotorEx.Direction.REVERSE);
        getRightSlide().setDirection(DcMotorEx.Direction.FORWARD);

        getSpecimenIntakeMotor().setDirection(DcMotorEx.Direction.REVERSE);


        // Servos
        shoulderServo  = myOpMode.hardwareMap.get(Servo.class, Constants.DEVICE_SHOULDER_SERVO);
        elbowServo  = myOpMode.hardwareMap.get(Servo.class, Constants.DEVICE_ELBOW_SERVO);
        wristServo  = myOpMode.hardwareMap.get(Servo.class, Constants.DEVICE_WRIST_SERVO);
        fingerServo  = myOpMode.hardwareMap.get(Servo.class, Constants.DEVICE_FINGER_SERVO);

        specimenIntakeServo  = myOpMode.hardwareMap.get(Servo.class, Constants.SPECIMEN_INTAKE_SERVO);
        outtakePanServo  = myOpMode.hardwareMap.get(Servo.class, Constants.OUTTAKE_PAN_SERVO);

        getElbowServo().setDirection(Servo.Direction.REVERSE);
        getElbowServo().setPosition(Constants.ELBOW_SERVO_HOME_POSITION);
        myOpMode.sleep(300);

        getShoulderServo().setDirection(Servo.Direction.REVERSE);
        getShoulderServo().setPosition(Constants.SHOULDER_SERVO_HOME_POSITION);

        getWristServo().setDirection(Servo.Direction.REVERSE);
        getWristServo().setPosition(Constants.WRIST_SERVO_90_POSITION);

        getFingerServo().setDirection(Servo.Direction.REVERSE);
        getFingerServo().setPosition(Constants.FINGER_SERVO_STOP_POSITION);

        getSpecimenIntakeServo().setDirection(Servo.Direction.REVERSE);
        getSpecimenIntakeServo().setPosition(Constants.SPECIMEN_INTAKE_SERVO_CLOSE_POSITION);

        getOuttakePanServo().setDirection(Servo.Direction.REVERSE);
        getOuttakePanServo().setPosition(Constants.OUTTAKE_PAN_SERVO_HOME_POSITION);

        setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);


        setMotorPowers(Constants.ZERO_POWER);
        getLeftSlide().setPower(Constants.ZERO_POWER);
        getRightSlide().setPower(Constants.ZERO_POWER);
        getSpecimenIntakeMotor().setPower(Constants.ZERO_POWER);

        LEFT_SLIDE_LOWER_LIMIT = getLeftSlide().getCurrentPosition();
        LEFT_SLIDE_UPPER_LIMIT = LEFT_SLIDE_LOWER_LIMIT + Constants.SLIDE_TIX_COUNT;

        RIGHT_SLIDE_LOWER_LIMIT = getRightSlide().getCurrentPosition();
        RIGHT_SLIDE_UPPER_LIMIT = RIGHT_SLIDE_LOWER_LIMIT + Constants.SLIDE_TIX_COUNT;

        SPECIMEN_INTAKE_MOTOR_SLIDE_LOWER_LIMIT = getSpecimenIntakeMotor().getCurrentPosition();
        SPECIMEN_INTAKE_MOTOR_SLIDE_UPPER_LIMIT = SPECIMEN_INTAKE_MOTOR_SLIDE_LOWER_LIMIT + Constants.SPECIMEN_INTAKE_MOTOR_SLIDE_TIX_COUNT;

        Utility.slide(this, Utility.Stage.ZERO,0.5);
        Utility.slideSpecimenIntake(this, Utility.Stage.ZERO,0.8);


        getLeftSlide().setPositionPIDFCoefficients(5.0);
        getRightSlide().setPositionPIDFCoefficients(5.0);

        getSpecimenIntakeMotor().setPositionPIDFCoefficients(5.0);
    }

    public void setMotorPowers(double leftFront, double rightFront, double leftBack, double rightBack) {
        this.getLeftFront().setPower(leftFront);
        this.getRightFront().setPower(rightFront);
        this.getLeftBack().setPower(leftBack);
        this.getRightBack().setPower(rightBack);
    }

    public void setMotorPowersForSlide(double speed) {
        this.getLeftSlide().setPower(speed);
        this.getRightSlide().setPower(speed);
    }

    public void setMotorPowersForSpecimenIntakeMotor(double speed) {
        this.getSpecimenIntakeMotor().setPower(speed);
    }

    public void setMotorPowers(double allWhealPower) {
        setMotorPowers(allWhealPower, allWhealPower, allWhealPower, allWhealPower);
    }

    public void setMode(DcMotorEx.RunMode mode) {
        getLeftFront().setMode(mode);
        getRightFront().setMode(mode);
        getLeftBack().setMode(mode);
        getRightBack().setMode(mode);
    }

    public void setModeForSlide(DcMotorEx.RunMode mode) {
        getLeftSlide().setMode(mode);
        getRightSlide().setMode(mode);
    }

    public void setModeForSpecimenIntakeMotor(DcMotorEx.RunMode mode) {
        getSpecimenIntakeMotor().setMode(mode);
    }

    public void setZeroPowerBehavior() {
        getLeftFront().setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        getRightFront().setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        getLeftBack().setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        getRightBack().setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void setZeroPowerBehaviorForSlide() {
        getLeftSlide().setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        getRightSlide().setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void setZeroPowerBehaviorForSpecimenIntakeMotor() {
        getSpecimenIntakeMotor().setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
    }

    public void setTargetPositionForSlide(Utility.Stage stage) {

        // Determine new target position, and pass to motor controller
        if (stage == Utility.Stage.ZERO) {
            getLeftSlide().setTargetPosition(LEFT_SLIDE_LOWER_LIMIT + 50);
            getRightSlide().setTargetPosition(RIGHT_SLIDE_LOWER_LIMIT + 50);
        } else if (stage == Utility.Stage.ONE) {
            getLeftSlide().setTargetPosition(LEFT_SLIDE_LOWER_LIMIT + 1950);
            getRightSlide().setTargetPosition(LEFT_SLIDE_LOWER_LIMIT + 1950);
        } else if (stage == Utility.Stage.TWO) {
            getLeftSlide().setTargetPosition(LEFT_SLIDE_LOWER_LIMIT + 2450);
            getRightSlide().setTargetPosition(LEFT_SLIDE_LOWER_LIMIT + 2450);
        } else if (stage == Utility.Stage.THREE) {
            getLeftSlide().setTargetPosition(LEFT_SLIDE_UPPER_LIMIT);
            getRightSlide().setTargetPosition(RIGHT_SLIDE_UPPER_LIMIT);
        }
    }

    public void setTargetPositionForSpecimenIntakeMotor(Utility.Stage stage) {

        if (stage == Utility.Stage.ZERO) {
            getSpecimenIntakeMotor().setTargetPosition(SPECIMEN_INTAKE_MOTOR_SLIDE_LOWER_LIMIT + 100);
        } else if (stage == Utility.Stage.ONE) {
            getSpecimenIntakeMotor().setTargetPosition(SPECIMEN_INTAKE_MOTOR_SLIDE_LOWER_LIMIT + 200);
        } else if (stage == Utility.Stage.TWO) {
            getSpecimenIntakeMotor().setTargetPosition(SPECIMEN_INTAKE_MOTOR_SLIDE_UPPER_LIMIT - 685);
        } else if (stage == Utility.Stage.THREE) {
            getSpecimenIntakeMotor().setTargetPosition(SPECIMEN_INTAKE_MOTOR_SLIDE_UPPER_LIMIT);
        }
    }

    public void setTargetPosition(Utility.Direction direction, double leftFrontInches, double rightFrontInches, double leftBackInches, double rightBackInches) {

        int leftFrontTarget = getLeftFront().getCurrentPosition();
        int rightFrontTarget= getRightFront().getCurrentPosition();
        int leftBackTarget = getLeftBack().getCurrentPosition();
        int rightBackTarget = getRightBack().getCurrentPosition();

        // Determine new target position, and pass to motor controller
        if (direction == Utility.Direction.FORWARD) {
            leftFrontTarget = leftFrontTarget + (int) (leftFrontInches * COUNTS_PER_INCH);
            rightFrontTarget = rightFrontTarget + (int) (rightFrontInches * COUNTS_PER_INCH);
            leftBackTarget = leftBackTarget + (int) (leftBackInches * COUNTS_PER_INCH);
            rightBackTarget = rightBackTarget + (int) (rightBackInches * COUNTS_PER_INCH);
        } else if (direction == Utility.Direction.BACKWARD) {
            leftFrontTarget = leftFrontTarget - (int) (leftFrontInches * COUNTS_PER_INCH);
            rightFrontTarget = rightFrontTarget - (int) (rightFrontInches * COUNTS_PER_INCH);
            leftBackTarget = leftBackTarget - (int) (leftBackInches * COUNTS_PER_INCH);
            rightBackTarget = rightBackTarget - (int) (rightBackInches * COUNTS_PER_INCH);
        } else if (direction == Utility.Direction.LEFT) {
            leftFrontTarget = leftFrontTarget - (int) (leftFrontInches * COUNTS_PER_INCH);
            rightFrontTarget = rightFrontTarget + (int) (rightFrontInches * COUNTS_PER_INCH);
            leftBackTarget = leftBackTarget + (int) (leftBackInches * COUNTS_PER_INCH);
            rightBackTarget = rightBackTarget - (int) (rightBackInches * COUNTS_PER_INCH);
        } else if (direction == Utility.Direction.RIGHT) {
            leftFrontTarget = leftFrontTarget + (int) (leftFrontInches * COUNTS_PER_INCH);
            rightFrontTarget = rightFrontTarget - (int) (rightFrontInches * COUNTS_PER_INCH);
            leftBackTarget = leftBackTarget - (int) (leftBackInches * COUNTS_PER_INCH);
            rightBackTarget = rightBackTarget + (int) (rightBackInches * COUNTS_PER_INCH);
        }

        // Set the Target Position
        getLeftFront().setTargetPosition(leftFrontTarget);
        getRightFront().setTargetPosition(rightFrontTarget);
        getLeftBack().setTargetPosition(leftBackTarget);
        getRightBack().setTargetPosition(rightBackTarget);
    }

    public void setTargetPosition(Utility.Direction direction, double inches) {

        setTargetPosition(direction, inches, inches, inches, inches);
    }

    public void initializeIMU() {

        setZeroPowerBehavior();

        // Control Hub IMU Parameters
//        RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
//        RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection = RevHubOrientationOnRobot.UsbFacingDirection.BACKWARD;
//        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoFacingDirection, usbFacingDirection);

        // Expansion Hub IMU Parameters
        RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection = RevHubOrientationOnRobot.UsbFacingDirection.LEFT;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoFacingDirection, usbFacingDirection);

        IMU.Parameters parameters = new IMU.Parameters(orientationOnRobot);

        imu = myOpMode.hardwareMap.get(IMU.class, Constants.DEVICE_IMU);
        imu.initialize(parameters);
        imu.resetYaw();
    }

    public double getCurrentHeading() {
        YawPitchRollAngles orientation = getImu().getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    public double getAbsoluteAngle() {
        return getImu().getRobotOrientation(AxesReference.INTRINSIC, AxesOrder.ZXY, AngleUnit.DEGREES).firstAngle;
    }

    public void initializeOpenCV(OpenCvPipeline pipeline) {

        // Create an instance of the camera
        int cameraMonitorViewId = myOpMode.hardwareMap.appContext.getResources().getIdentifier(
                "cameraMonitorViewId", "id", myOpMode.hardwareMap.appContext.getPackageName());

        // Use OpenCvCameraFactory class from FTC SDK to create camera instance
        this.controlHubCam = OpenCvCameraFactory.getInstance().createWebcam(
                myOpMode.hardwareMap.get(WebcamName.class, Constants.DEVICE_CAMERA), cameraMonitorViewId);

        this.controlHubCam.setPipeline(pipeline);

        this.controlHubCam.openCameraDevice();
        this.controlHubCam.startStreaming(Constants.CAMERA_WIDTH, Constants.CAMERA_HEIGHT, OpenCvCameraRotation.UPRIGHT);
    }

    public void releaseResourcesForOpenCV() {

        getControlHubCam().stopRecordingPipeline();
        getControlHubCam().stopStreaming();
        getControlHubCam().closeCameraDevice();
    }

    public void initializeAprilTag() {

        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.
        visionPortal = new VisionPortal.Builder()
                .setCamera(myOpMode.hardwareMap.get(WebcamName.class, Constants.DEVICE_CAMERA))
                .addProcessor(aprilTag)
                .build();
    }

    public LinearOpMode getMyOpMode() {
        return this.myOpMode;
    }

    public DcMotorEx getLeftFront() {
        return this.leftFront;
    }

    public DcMotorEx getRightFront() {
        return this.rightFront;
    }

    public DcMotorEx getLeftBack() {
        return this.leftBack;
    }

    public DcMotorEx getRightBack() {
        return this.rightBack;
    }

    public DcMotorEx getRightSlide() {
        return this.rightSlide;
    }

    public DcMotorEx getLeftSlide() {
        return this.leftSlide;
    }

//    public Servo getRightSlideServo() {
//        return this.rightSlideServo;
//    }


    public DcMotorEx getSpecimenIntakeMotor() {
        return specimenIntakeMotor;
    }

    public Servo getShoulderServo() {
        return this.shoulderServo;
    }

    public Servo getElbowServo() {
        return elbowServo;
    }

    public Servo getWristServo() {
        return wristServo;
    }

    public Servo getFingerServo() {
        return fingerServo;
    }

    public Servo getLeftSlideServo() {
        return this.leftSlideServo;
    }

    public Servo getFrontIntakeServo() {
        return this.frontIntakeServo;
    }

    public Servo getBackIntakeServo() {
        return this.backIntakeServo;
    }

    public Servo getIntakePanServo() {
        return intakePanServo;
    }

    public IMU getImu() {
        return this.imu;
    }

    public Servo getSpecimenIntakeServo() {
        return specimenIntakeServo;
    }

    public Servo getOuttakePanServo() {
        return outtakePanServo;
    }

    public OpenCvCamera getControlHubCam() {
        return this.controlHubCam;
    }

    public AprilTagProcessor getAprilTag() {
        return this.aprilTag;
    }

    public VisionPortal getVisionPortal() {
        return this.visionPortal;
    }
}
