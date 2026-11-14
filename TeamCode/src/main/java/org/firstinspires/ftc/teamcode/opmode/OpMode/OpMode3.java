package org.firstinspires.ftc.teamcode.opmode.OpMode;

import static org.firstinspires.ftc.teamcode.common.hardware.BotCoefficients.HOGBACK_MIN_VELOCITY;
import static org.firstinspires.ftc.teamcode.common.hardware.BotCoefficients.HOGBACK_TARGET_VELOCITY;
import static org.firstinspires.ftc.teamcode.common.hardware.RobotHardware.LaunchState.IDLE;

import android.annotation.SuppressLint;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.Limelight3A;

import org.firstinspires.ftc.teamcode.common.hardware.BotCoefficients;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
//a
@TeleOp(name = "OpMode3")

public class OpMode3 extends LinearOpMode {
    public ElapsedTime mRunTime = new ElapsedTime();

    RobotHardware robot = new RobotHardware();

    private int sleepMs1 = 0;
    private boolean bMoveUpHogback = false;
    private boolean wasPusherMoving = false;
    RobotHardware.LaunchState launchState = null;

    public static final double GATE_OPEN = 1.0;
    public static final double GATE_CLOSED = 0.45; // Fully closed position

    /**
     * This function is executed when this Op Mode is selected from the Driver Station.
     */
    @SuppressLint("DefaultLocale")
    @Override
    public void runOpMode() throws InterruptedException {

        robot.init(hardwareMap);
        telemetry.addData("Status", "Initialized");
        telemetry.setMsTransmissionInterval(11);


        // Put initialization blocks here.

        waitForStart();
        while (opModeIsActive()) {
            double horizontal = -1.0 * gamepad1.right_stick_x * 0.6;
            double vertical = gamepad1.right_stick_y * 0.6;
            double turn = -1.0 * gamepad1.left_stick_x * 0.6;

            double flPower = vertical - turn - horizontal;
            double frPower = vertical + turn - horizontal;
            double blPower = vertical - turn + horizontal;
            double brPower = vertical + turn + horizontal;
            double scaling = Math.max(1.0,
                    Math.max(Math.max(Math.abs(flPower), Math.abs(frPower)),
                            Math.max(Math.abs(blPower), Math.abs(brPower))));
            flPower = flPower / scaling;
            frPower = frPower / scaling;
            blPower = blPower / scaling;
            brPower = brPower / scaling;
            robot.setDrivePower(flPower, frPower, blPower, brPower);

            telemetry.addLine(String.format("FL: %d \nBL %d \nFR: %d \nBR: %d ",
                    robot.motorfl.getCurrentPosition(),
                    robot.motorbl.getCurrentPosition(),
                    robot.motorfr.getCurrentPosition(),
                    robot.motorbr.getCurrentPosition()
            ));


            // 1. Moving Forward
            if (gamepad2.right_stick_y > 0.5 ) {
                robot.hogback.setPower(BotCoefficients.HOGBACK_POWERUP);
//                currentVelocity = robot.hogback.getVelocity();
//                while (robot.hogback.getVelocity() < BotCoefficients.HOGBACK_MIN_VELOCITY) {

            }
// 2. Moving Backward
            else if (gamepad2.right_stick_y < -0.5) {
                robot.hogback.setPower(BotCoefficients.HOGBACK_POWERDOWN);
            }
// 3. Joystick is back in the neutral/original position
            else {
                robot.hogback.setPower(BotCoefficients.HOGBACK_STOP); // Sets power to 0.0
            }

            // 1. Intake Start
            if (gamepad2.a) {
                robot.intake1.setPower(BotCoefficients.INTAKE_POWER);
                robot.intake2.setPower(BotCoefficients.INTAKE_POWER);
            }
// 2. Moving Backward
            else if (gamepad2.x) {
                robot.intake1.setPower(BotCoefficients.INTAKE_REVERSE);
                robot.intake2.setPower(BotCoefficients.INTAKE_REVERSE);
            }


// 3. Joystick is back in the neutral/original position
            else if (gamepad2.b) {
                robot.intake1.setPower(BotCoefficients.INTAKE_STOP); // Sets power to 0.0
                robot.intake2.setPower(BotCoefficients.INTAKE_STOP);
            }

            if (gamepad2.dpad_right) {
                robot.gate.setPosition(GATE_OPEN);
            } else if (gamepad2.dpad_left) {
                robot.gate.setPosition(GATE_CLOSED);
            }

            // Launch state machine
            launch(gamepad2.right_stick_y < -0.5);

            telemetry.addData("Hogback Velocity", robot.hogback.getVelocity());
            telemetry.addData("Launch State", launchState);
            telemetry.update();
        }
    }

    public void launch(boolean shotRequested) {
        if (launchState == null) {
            launchState = IDLE;
        }

        switch (launchState) {
            case IDLE:
                robot.gate.setPosition(GATE_CLOSED);
                robot.hogback.setPower(BotCoefficients.HOGBACK_STOP);

                if (shotRequested) {
                    launchState = RobotHardware.LaunchState.CLOSED;
                }
                break;

            case CLOSED:
                if (!shotRequested) {
                    robot.gate.setPosition(GATE_CLOSED);
                    robot.hogback.setPower(BotCoefficients.HOGBACK_STOP);
                    launchState = IDLE;
                    break;
                }

                robot.gate.setPosition(GATE_CLOSED);
                robot.hogback.setVelocity(HOGBACK_TARGET_VELOCITY);

                if (robot.hogback.getVelocity() >= HOGBACK_MIN_VELOCITY) {
                    robot.gate.setPosition(GATE_OPEN);
                    launchState = RobotHardware.LaunchState.OPEN;
                }
                break;

            case OPEN:
                if (!shotRequested) {
                    robot.gate.setPosition(GATE_CLOSED);
                    robot.hogback.setPower(BotCoefficients.HOGBACK_STOP);
                    launchState = IDLE;
                    break;
                }

                robot.hogback.setVelocity(HOGBACK_TARGET_VELOCITY);

                if (robot.hogback.getVelocity() < HOGBACK_MIN_VELOCITY) {
                    robot.gate.setPosition(GATE_CLOSED);
                    launchState = RobotHardware.LaunchState.CLOSED;
                } else {
                    robot.gate.setPosition(GATE_OPEN);
                }
                break;
        }
    }
}





