package org.firstinspires.ftc.teamcode.common.hardware;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class RobotHardware extends LinearOpMode {
    public enum LaunchState {
        IDLE, OPEN, CLOSED
    }

    // HardwareMap reference
    public HardwareMap hwMap = null;

    // Drivetrain motors (Dual-name support: motorfl/motorfr/motorbl/motorbr AND frontleftDrive/frontrightDrive/backleftDrive/backrightDrive)
    public DcMotor motorfl = null;
    public DcMotor motorfr = null;
    public DcMotor motorbl = null;
    public DcMotor motorbr = null;

    public DcMotor frontleftDrive = null;
    public DcMotor frontrightDrive = null;
    public DcMotor backleftDrive = null;
    public DcMotor backrightDrive = null;

    // Mechanisms
    public DcMotorEx hogback = null;
    public Servo gate = null;
    public DcMotor intake1 = null;
    public DcMotor intake2 = null;
    public IMU imu;

    // Orientation and timer
    public YawPitchRollAngles orientation0;
    public AngularVelocity angularVelocity0;
    public double yaw0;
    public ElapsedTime runtime = new ElapsedTime();

    // Encoder and drive math constants
    public static final double COUNTS_PER_MOTOR_REV = 383;
    public static final double DRIVE_GEAR_REDUCTION = 1.0;
    public static final double WHEEL_DIAMETER_INCHES = 4.0;
    public static final double COUNTS_PER_INCH =
            (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
                    (WHEEL_DIAMETER_INCHES * 3.1415);

    public double ticksPerInch = 31.3;
    public double ticksPerDegree = 12;

    public RobotHardware() {}

    @Override
    public void runOpMode() throws InterruptedException {
        // Stub for LinearOpMode compatibility
    }

    /**
     * Initialize all the robot's hardware.
     */
    public void init(HardwareMap ahwMap) {
        hwMap = ahwMap;

        // Define and Initialize Motors
        motorfl = hwMap.get(DcMotor.class, "motorfl");
        motorfr = hwMap.get(DcMotor.class, "motorfr");
        motorbl = hwMap.get(DcMotor.class, "motorbl");
        motorbr = hwMap.get(DcMotor.class, "motorbr");

        // Set aliases for AutoHardware compatibility
        frontleftDrive = motorfl;
        frontrightDrive = motorfr;
        backleftDrive = motorbl;
        backrightDrive = motorbr;

        hogback = hwMap.get(DcMotorEx.class, "hogback");
        intake1 = hwMap.get(DcMotor.class, "intake1");
        intake2 = hwMap.get(DcMotor.class, "intake2");
        gate = hwMap.get(Servo.class, "gate");

        // Set Brake zero power behavior
        motorfr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorfl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorbr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorbl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hogback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorfl.setDirection(DcMotor.Direction.REVERSE);
        motorbl.setDirection(DcMotor.Direction.REVERSE);
        motorfr.setDirection(DcMotor.Direction.FORWARD);
        motorbr.setDirection(DcMotor.Direction.FORWARD);

        gate.setPosition(0.0);

        setDrivetrainMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        hogback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize IMU in the control hub
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.UP;
        RevHubOrientationOnRobot.UsbFacingDirection  usbDirection  = RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu = hwMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

        orientation0 = imu.getRobotYawPitchRollAngles();
        angularVelocity0 = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
        yaw0 = orientation0.getYaw(AngleUnit.DEGREES);
    }

    public void initAll() {
        init(hardwareMap);
    }

    public void initServo() {
        gate = hwMap.get(Servo.class, "gate");
    }

    public void initHogback() {
        hogback = hwMap.get(DcMotorEx.class, "hogback");
        hogback.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        hogback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void initIntakes() {
        intake1 = hwMap.get(DcMotor.class, "intake1");
        intake2 = hwMap.get(DcMotor.class, "intake2");
        intake1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intake1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        intake2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void initMotor() {
        init(hwMap);
    }

    public void setDrivetrainMode(DcMotor.RunMode mode) {
        motorfl.setMode(mode);
        motorfr.setMode(mode);
        motorbl.setMode(mode);
        motorbr.setMode(mode);
    }

    public double getCurrentYaw() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public void setAutoDriveMotorMode() {
        motorfl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorfr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorbl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorbr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorfl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorfr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorbl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorbr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorfl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorfr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorbl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorbr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    public void setDriveForward(double fl, double fr, double bl, double br) {
        setDrivePower(fl, fr, bl, br);
    }

    public void setDrivePower(double fl, double fr, double bl, double br) {
        motorfl.setPower(Math.max(-1.0, Math.min(1.0, fl)));
        motorfr.setPower(Math.max(-1.0, Math.min(1.0, fr)));
        motorbl.setPower(Math.max(-1.0, Math.min(1.0, bl)));
        motorbr.setPower(Math.max(-1.0, Math.min(1.0, br)));
    }

    public void setAllDrivePower(double p) {
        setDrivePower(p, p, p, p);
    }

    public void encoderDrive(double speed, double leftInches, double rightInches, double timeoutS) {
        int newBackLeftTarget;
        int newBackRightTarget;
        int newFrontLeftTarget;
        int newFrontRightTarget;

        if (opModeIsActive()) {
            newBackLeftTarget = motorbl.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newBackRightTarget = motorbr.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);
            newFrontLeftTarget = motorfl.getCurrentPosition() + (int) (leftInches * COUNTS_PER_INCH);
            newFrontRightTarget = motorfr.getCurrentPosition() + (int) (rightInches * COUNTS_PER_INCH);

            motorbl.setTargetPosition(newBackLeftTarget);
            motorbr.setTargetPosition(newBackRightTarget);
            motorfl.setTargetPosition(newFrontLeftTarget);
            motorfr.setTargetPosition(newFrontRightTarget);

            motorbl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorbr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorfl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            motorfr.setMode(DcMotor.RunMode.RUN_TO_POSITION);

            runtime.reset();

            motorbl.setPower(Math.abs(speed));
            motorbr.setPower(Math.abs(speed));
            motorfl.setPower(Math.abs(speed));
            motorfr.setPower(Math.abs(speed));

            while (opModeIsActive() && runtime.seconds() < timeoutS && motorbl.isBusy() && motorbr.isBusy()) {
                telemetry.addData("Running to", "%7d :%7d", newBackLeftTarget, newBackRightTarget);
                telemetry.addData("Currently at", "at %7d :%7d", motorbl.getCurrentPosition(), motorbr.getCurrentPosition());
                telemetry.update();
                idle();
            }

            motorbl.setPower(0);
            motorbr.setPower(0);
            motorfl.setPower(0);
            motorfr.setPower(0);

            setDrivetrainMode(DcMotor.RunMode.RUN_USING_ENCODER);
            sleep(250);
        }
    }

    public void initIMU() {
        if (hwMap == null) return;
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.LEFT;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu = hwMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();
        orientation0 = imu.getRobotYawPitchRollAngles();
        angularVelocity0 = imu.getRobotAngularVelocity(AngleUnit.DEGREES);
        imu.resetYaw();
        yaw0 = getCurrentYaw();
    }

    public void turnToTargetYaw(double targetYawDegree, double power, long maxAllowedTimeInMills) {
        long timeBegin = System.currentTimeMillis();
        double currentYaw = getCurrentYaw();
        double diffYaw = Math.abs(currentYaw - targetYawDegree);

        while (diffYaw > 2 && opModeIsActive() && (System.currentTimeMillis() - timeBegin) < maxAllowedTimeInMills) {
            int ticks = (int) (diffYaw * ticksPerDegree);
            if (ticks > 100) ticks = 100;
            int tickDirection = (currentYaw < targetYawDegree) ? -1 : 1;
            if (ticks < 5) break;

            double factor = (diffYaw > 5) ? 1.0 : diffYaw / 5;

            driveMotors(tickDirection * ticks, tickDirection * ticks, -(tickDirection * ticks), -(tickDirection * ticks), power * factor, false, 0);
            sleep(30);

            currentYaw = getCurrentYaw();
            diffYaw = Math.abs(currentYaw - targetYawDegree);
        }
    }

    public void driveMotors(int flTarget, int blTarget, int frTarget, int brTarget, double power, boolean bKeepYaw, double targetYaw) {
        motorfl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorbl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorfr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorbr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorfl.setTargetPosition(flTarget);
        motorbl.setTargetPosition(blTarget);
        motorfr.setTargetPosition(frTarget);
        motorbr.setTargetPosition(brTarget);

        double leftRatio = 0.95;
        motorfl.setPower(power * leftRatio);
        motorbl.setPower(power * leftRatio);
        motorfr.setPower(power);
        motorbr.setPower(power);

        motorfl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorbl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorfr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        motorbr.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        while (opModeIsActive() && motorfl.isBusy() && motorbl.isBusy() && motorfr.isBusy() && motorbr.isBusy()) {
            idle();
        }

        motorfl.setPower(0);
        motorbl.setPower(0);
        motorfr.setPower(0);
        motorbr.setPower(0);
    }
}