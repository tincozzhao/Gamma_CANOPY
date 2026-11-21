package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "intaketest", group = "Robot")
public class intaketest extends AutoHardware {

    public static final double HOGBACK_POWERUP = -0.8;
    public static final double HOGBACK_POWERDOWN = 0.8;
    public static final double HOGBACK_STOP = 0.0;
    public static final double AUTO_TARGET_VELOCITY = -1200;

    public static final double AUTO_MIN_VELOCITY = -1100;
    public static final double INTAKE_POWER = 0.4;
    public static final double INTAKE_REVERSE = -0.4;
    public static final double INTAKE_STOP = 0.0;

    public static final double HOGBACK_TARGET_VELOCITY = -1050;
    public static final double HOGBACK_MIN_VELOCITY = -950;

    public static final double GATE_OPEN = 1.0;
    public static final double GATE_CLOSED = 0.45;

    @Override
    public void runOpMode() throws InterruptedException {

        initAll();

        // Initial state
        hogback.setVelocity(0);
        intake1.setPower(0);
        intake2.setPower(0);
        gate.setPosition(GATE_CLOSED);

        telemetry.addData("Status", "Ready");
        telemetry.update();

        waitForStart();

        if (!opModeIsActive()) {
            return;
        }

        try {

            // Start Hogback
            hogback.setVelocity(AUTO_TARGET_VELOCITY);

            double startTime = getRuntime();
            boolean hogbackReady = false;

            // Wait for Hogback to reach target velocity
            while (opModeIsActive()
                    && getRuntime() - startTime < 4.0) {

                double velocity = hogback.getVelocity();

                telemetry.addData("Hogback Velocity", velocity);
                telemetry.addData("Target Velocity",
                        AUTO_TARGET_VELOCITY);

                if (velocity <= AUTO_MIN_VELOCITY) {
                    hogbackReady = true;
                    break;
                }

                telemetry.addData("Status", "Waiting for Hogback");
                telemetry.update();

                idle();
            }

            if (opModeIsActive() && hogbackReady) {

                telemetry.addData("Status", "Hogback Ready");
                telemetry.update();

                // Start intake
                intake1.setPower(INTAKE_POWER);
                intake2.setPower(INTAKE_POWER);

                sleep(2000);

                if (opModeIsActive()) {

                    // Open gate
                    gate.setPosition(GATE_OPEN);

                    telemetry.addData("Status", "Feeding pollen");
                    telemetry.update();

                    sleep(5000);
                }

            } else if (opModeIsActive()) {

                telemetry.addData("Status",
                        "ERROR: Hogback failed to reach velocity");
                telemetry.update();
            }

        } finally {

            // Stop all motors
            hogback.setVelocity(0);
            intake1.setPower(INTAKE_STOP);
            intake2.setPower(INTAKE_STOP);

            // Close gate
            gate.setPosition(GATE_CLOSED);

            telemetry.addData("Status", "Stopped");
            telemetry.update();
        }
    }
}
