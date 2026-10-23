
package org.firstinspires.ftc.teamcode.common.hardware;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngularVelocity;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;

public class RobotHardware {

    // Hardware map
    HardwareMap hwMap = null;

    // Drive motors
    public DcMotor motorfl = null;
    public DcMotor motorfr = null;
    public DcMotor motorbr = null;
    public DcMotor motorbl = null;

    // Mechanism motors
    public DcMotor intake = null;
    public DcMotor hogback = null;

    // Pusher
    public CRServo pusher0 = null;

    // IMU
    public IMU imu;

    // Initial robot orientation
    public YawPitchRollAngles orientation0;
    public AngularVelocity angularVelocity0;
    public double yaw0;

    // Constructor
    public RobotHardware() {
    }

    // Initialize robot hardware
    public void init(HardwareMap ahwMap) {

        // Save hardware map
        hwMap = ahwMap;

        // Initialize drive motors
        motorfl = hwMap.get(DcMotor.class, "motorfl");
        motorfr = hwMap.get(DcMotor.class, "motorfr");
        motorbl = hwMap.get(DcMotor.class, "motorbl");
        motorbr = hwMap.get(DcMotor.class, "motorbr");

        // Initialize hogback
        hogback = hwMap.get(DcMotor.class, "hogback");

        // Initialize intake
        intake = hwMap.get(DcMotor.class, "intake");
        intake.setDirection(DcMotor.Direction.REVERSE);
        intake.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        intake.setZeroPowerBehavior(
                DcMotor.ZeroPowerBehavior.BRAKE
        );

        // Initialize pusher
        pusher0 = hwMap.get(CRServo.class, "pusher0");

        // Set drive motor braking
        motorfr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorfl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorbr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorbl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        hogback.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Set drive motor directions
        motorfr.setDirection(DcMotor.Direction.REVERSE);
        motorbr.setDirection(DcMotor.Direction.REVERSE);

        // Stop pusher during initialization
        pusher0.setPower(0);

        // Set drivetrain mode
        setDrivetrainMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        // Initialize IMU
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection =
                RevHubOrientationOnRobot.LogoFacingDirection.UP;

        RevHubOrientationOnRobot.UsbFacingDirection usbDirection =
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD;

        RevHubOrientationOnRobot orientationOnRobot =
                new RevHubOrientationOnRobot(
                        logoDirection,
                        usbDirection
                );

        imu = hwMap.get(IMU.class, "imu");
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        imu.resetYaw();

        // Retrieve initial rotational angles and velocities
        orientation0 = imu.getRobotYawPitchRollAngles();
        angularVelocity0 =
                imu.getRobotAngularVelocity(AngleUnit.DEGREES);

        yaw0 = orientation0.getYaw(AngleUnit.DEGREES);
    }

    // Set motors for autonomous driving
    public void setAutoDriveMotorMode() {

        motorfr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorfl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorbr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motorbl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        motorfr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorfl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorbr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motorbl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        motorfr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorfl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorbr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorbl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    // Get current robot yaw
    public double getCurrentYaw() {
        return imu.getRobotYawPitchRollAngles()
                .getYaw(AngleUnit.DEGREES);
    }

    // Set drivetrain run mode
    public void setDrivetrainMode(DcMotor.RunMode mode) {
        motorfl.setMode(mode);
        motorfr.setMode(mode);
        motorbl.setMode(mode);
        motorbr.setMode(mode);
    }

    // Drive forward
    public void setDriveForward(
            double fl, double fr, double bl, double br) {

        fl = Math.max(-1.0, Math.min(1.0, fl));
        fr = Math.max(-1.0, Math.min(1.0, fr));
        bl = Math.max(-1.0, Math.min(1.0, bl));
        br = Math.max(-1.0, Math.min(1.0, br));

        motorfl.setPower(-fl);
        motorfr.setPower(fr);
        motorbl.setPower(bl);
        motorbr.setPower(br);
    }

    // Set drivetrain power
    public void setDrivePower(
            double fl, double fr, double bl, double br) {

        fl = Math.max(-1.0, Math.min(1.0, fl));
        fr = Math.max(-1.0, Math.min(1.0, fr));
        bl = Math.max(-1.0, Math.min(1.0, bl));
        br = Math.max(-1.0, Math.min(1.0, br));

        motorfl.setPower(fl);
        motorfr.setPower(fr);
        motorbl.setPower(bl);
        motorbr.setPower(br);
    }

    // Set all drive motors to the same power
    public void setAllDrivePower(double power) {
        setDrivePower(power, power, power, power);
    }
}
