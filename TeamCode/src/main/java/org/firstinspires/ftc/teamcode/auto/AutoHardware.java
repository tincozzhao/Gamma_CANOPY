
package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

public abstract class AutoHardware extends LinearOpMode {

    // Drivetrain motors
    protected DcMotor backleftDrive = null;
    protected DcMotor backrightDrive = null;
    protected DcMotor frontleftDrive = null;
    protected DcMotor frontrightDrive = null;

    // Hogback motor - supports velocity control
    protected DcMotorEx hogback = null;

    // Intake motor
    protected DcMotor intake = null;

    // Pusher servos
    public Servo pusher0 = null;
    public Servo pusher1 = null;

    // Timer
    protected ElapsedTime runtime = new ElapsedTime();

    // Motor encoder constants
    protected static final double COUNTS_PER_MOTOR_REV = 383;
    protected static final double DRIVE_GEAR_REDUCTION = 1.0;
    protected static final double WHEEL_DIAMETER_INCHES = 4.0;

    protected static final double COUNTS_PER_INCH =
            (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                    (WHEEL_DIAMETER_INCHES * 3.1415);

    // IMU control
    public IMU imu;

    public YawPitchRollAngles orientation0;
    public AngularVelocity angularVelocity0;
    public double yaw0;

    public double ticksPerInch = 31.3;
    public double ticksPerDegree = 12;

    // Vision declarations
    protected static final boolean USE_WEBCAM = true;

    protected static final String TFOD_MODEL_ASSET =
            "model_cylinder2_20231125.tflite";

    protected static final String TFOD_MODEL_FILE =
            "/sdcard/FIRST/tflitemodels/myCustomModel.tflite";

    protected static final String[] LABELS = {
            "cylinder"
    };

    protected VisionPortal visionPortal;
    protected AprilTagProcessor aprilTag;


    // Initialize all hardware
    public void initAll() {
        initServo();
        initMotor();
        initIMU();
    }


    // Initialize pusher servos
    public void initServo() {

        pusher0 = hardwareMap.get(Servo.class, "pusher0");
        pusher1 = hardwareMap.get(Servo.class, "pusher1");
    }


    // Initialize hogback
    public void initHogback() {

        hogback = hardwareMap.get(DcMotorEx.class, "hogback");

        hogback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        hogback.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);
    }


    // Initialize intake
    public void initIntake() {

        intake = hardwareMap.get(DcMotor.class, "intake");

        intake.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intake.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);
    }


    // Initialize drivetrain and motors
    public void initMotor() {

        backleftDrive = hardwareMap.get(DcMotor.class, "motorbl");
        backrightDrive = hardwareMap.get(DcMotor.class, "motorbr");
        frontleftDrive = hardwareMap.get(DcMotor.class, "motorfl");
        frontrightDrive = hardwareMap.get(DcMotor.class, "motorfr");

        // Drivetrain braking
        frontrightDrive.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);

        frontleftDrive.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);

        backrightDrive.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);

        backleftDrive.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE);

        // Motor directions
        frontrightDrive.setDirection(DcMotor.Direction.FORWARD);
        frontleftDrive.setDirection(DcMotor.Direction.REVERSE);

        // Initialize other motors
        initHogback();
        initIntake();
    }


    // Encoder drivetrain movement
    public void encoderDrive(
            double speed,
            double leftInches,
            double rightInches,
            double timeoutS) {

        int newBackLeftTarget;
        int newBackRightTarget;
        int newFrontLeftTarget;
        int newFrontRightTarget;

        if (opModeIsActive()) {

            newBackLeftTarget = backleftDrive.getCurrentPosition()
                    + (int) (leftInches * COUNTS_PER_INCH);

            newBackRightTarget = backrightDrive.getCurrentPosition()
                    + (int) (rightInches * COUNTS_PER_INCH);

            newFrontLeftTarget = frontleftDrive.getCurrentPosition()
                    + (int) (leftInches * COUNTS_PER_INCH);

            newFrontRightTarget = frontrightDrive.getCurrentPosition()
                    + (int) (rightInches * COUNTS_PER_INCH);

            backleftDrive.setTargetPosition(newBackLeftTarget);
            backrightDrive.setTargetPosition(newBackRightTarget);
            frontleftDrive.setTargetPosition(newFrontLeftTarget);
            frontrightDrive.setTargetPosition(newFrontRightTarget);

            backleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            backrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            frontrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();

            backleftDrive.setPower(Math.abs(speed));
            backrightDrive.setPower(Math.abs(speed));
            frontleftDrive.setPower(Math.abs(speed));
            frontrightDrive.setPower(Math.abs(speed));

            while (opModeIsActive()
                    && runtime.seconds() < timeoutS
                    && backleftDrive.isBusy()
                    && backrightDrive.isBusy()) {

                telemetry.addData("Running to", "%7d :%7d",
                        newBackLeftTarget, newBackRightTarget);

                telemetry.addData("Currently at", "at %7d :%7d",
                        backleftDrive.getCurrentPosition(),
                        backrightDrive.getCurrentPosition());

                telemetry.update();
                idle();
            }

            // Stop drivetrain
            backleftDrive.setPower(0);
            backrightDrive.setPower(0);
            frontleftDrive.setPower(0);
            frontrightDrive.setPower(0);

            // Restore encoder mode
            backleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            backrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

            sleep(250);
        }
    }


    // Reset drivetrain encoders
    public void setAutoDriveMotorMode() {

        frontleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backrightDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backleftDrive.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        frontleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backrightDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backleftDrive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    // Initialize IMU
    public void initIMU() {

        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.LEFT;

        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.UP;

        RevHubOrientationOnRobot orientationOnRobot =
                new RevHubOrientationOnRobot(logoDirection, usbDirection);

        imu = hardwareMap.get(IMU.class, "imu");

        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

        telemetry.addLine(String.format(
                "INIT Yaw: %.1f\n", getCurrentYaw()
        ));
        telemetry.update();

        orientation0 = imu.getRobotYawPitchRollAngles();

        angularVelocity0 =
                imu.getRobotAngularVelocity(AngleUnit.DEGREES);

        imu.resetYaw();
        yaw0 = getCurrentYaw();
    }


    // Get current robot yaw
    public double getCurrentYaw() {

        return imu.getRobotYawPitchRollAngles()
                .getYaw(AngleUnit.DEGREES);
    }


    // Turn robot to target yaw
    public void turnToTargetYaw(
            double targetYawDegree,
            double power,
            long maxAllowedTimeInMills) {

        long timeBegin;
        long timeCurrent;

        double currentYaw = getCurrentYaw();

        int ticks;
        int tickDirection;

        double factor = 1.0;

        double diffYaw = Math.abs(currentYaw - targetYawDegree);

        telemetry.addLine(String.format(
                "\nCurrentYaw=%.2f\nTargetYaw=%.2f\nYaw0=%.2f",
                currentYaw, targetYawDegree, yaw0
        ));
        telemetry.update();

        timeBegin = timeCurrent = System.currentTimeMillis();

        while (diffYaw > 2
                && opModeIsActive()
                && (timeCurrent - timeBegin) < maxAllowedTimeInMills) {

            ticks = (int) (diffYaw * ticksPerDegree);

            if (ticks > 100) {
                ticks = 100;
            }

            tickDirection = (currentYaw < targetYawDegree) ? -1 : 1;

            if (ticks < 5) {
                break;
            }

            if (diffYaw > 5) {
                factor = 1.0;
            } else {
                factor = diffYaw / 5;
            }

            driveMotors(
                    tickDirection * ticks,
                    tickDirection * ticks,
                    -(tickDirection * ticks),
                    -(tickDirection * ticks),
                    power * factor,
                    false,
                    0
            );

            sleep(30);

            currentYaw = getCurrentYaw();
            timeCurrent = System.currentTimeMillis();
            diffYaw = Math.abs(currentYaw - targetYawDegree);

            telemetry.addLine(String.format(
                    "\nCurrentYaw=%.2f\nTargetYaw=%.2f\nTimeLapsed=%.2f ms",
                    currentYaw,
                    targetYawDegree,
                    (double) (timeCurrent - timeBegin)
            ));
            telemetry.update();
        }
    }


    // Drive motors using encoder targets
    public void driveMotors(
            int flTarget,
            int blTarget,
            int frTarget,
            int brTarget,
            double power,
            boolean bKeepYaw,
            double targetYaw) {

        double currentYaw;
        double powerDeltaPct;
        double powerL;
        double powerR;

        double leftRatioToCounterCOG = 0.95;

        int direction;

        frontleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backleftDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backrightDrive.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontleftDrive.setTargetPosition(flTarget);
        backleftDrive.setTargetPosition(blTarget);
        frontrightDrive.setTargetPosition(frTarget);
        backrightDrive.setTargetPosition(brTarget);

        frontleftDrive.setPower(power * leftRatioToCounterCOG);
        backleftDrive.setPower(power * leftRatioToCounterCOG);
        frontrightDrive.setPower(power);
        backrightDrive.setPower(power);

        frontleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backleftDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backrightDrive.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        if (!(flTarget == blTarget
                && flTarget == frTarget
                && flTarget == brTarget)) {
            bKeepYaw = false;
        }

        direction = (flTarget > 0) ? 1 : -1;

        while (opModeIsActive()
                && frontleftDrive.isBusy()
                && backleftDrive.isBusy()
                && frontrightDrive.isBusy()
                && backrightDrive.isBusy()) {

            if (bKeepYaw) {

                currentYaw = imu.getRobotYawPitchRollAngles()
                        .getYaw(AngleUnit.DEGREES);

                if (Math.abs(currentYaw - targetYaw) > 2.0) {
                    powerDeltaPct = 0.25;
                } else {
                    powerDeltaPct =
                            Math.abs(currentYaw - targetYaw) / 2.0 * 0.25;
                }

                if (currentYaw < targetYaw) {
                    powerL = power * (1 - direction * powerDeltaPct);
                    powerR = power * (1 + direction * powerDeltaPct);
                } else {
                    powerL = power * (1 + direction * powerDeltaPct);
                    powerR = power * (1 - direction * powerDeltaPct);
                }

                if (powerL > 1.0) {
                    powerL = 1.0;
                }

                if (powerR > 1.0) {
                    powerR = 1.0;
                }

                frontleftDrive.setPower(powerL);
                backleftDrive.setPower(powerL);
                frontrightDrive.setPower(powerR);
                backrightDrive.setPower(powerR);
            }

            idle();
        }

        frontleftDrive.setPower(0);
        backleftDrive.setPower(0);
        frontrightDrive.setPower(0);
        backrightDrive.setPower(0);
    }
}
