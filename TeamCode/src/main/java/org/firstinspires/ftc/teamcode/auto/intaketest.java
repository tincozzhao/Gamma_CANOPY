package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "intaketest", group = "Robot")
public class intaketest extends AutoHardware {

    public static final double HOGBACK_POWERUP = -0.8;
    public static final double HOGBACK_POWERDOWN = 0.8;
    public static final double HOGBACK_STOP = 0.0;

    public static final double INTAKE_POWER = 0.4;
    public static final double INTAKE_REVERSE = -0.4;

    public static final double INTAKE_STOP = 0.0;

    public static final double HOGBACK_TARGET_VELOCITY = -1050;
    public static final double HOGBACK_MIN_VELOCITY = -950;

    public static final double GATE_OPEN = 1.0;
    public static final double GATE_CLOSED = 0.45; // Fully closed position



    @Override
    public void runOpMode() throws InterruptedException {

        initAll();

        telemetry.addData("Status", "Ready");
        telemetry.update();

        waitForStart();

        if (opModeIsActive()) {

            // Start hogback
            hogback.setVelocity(HOGBACK_TARGET_VELOCITY);

            double startTime = getRuntime();

            // Wait for hogback to reach target velocity
            while (opModeIsActive()
                    && hogback.getVelocity() > HOGBACK_MIN_VELOCITY
                    && getRuntime() - startTime < 4.0) {

                telemetry.addData("Hogback Velocity",
                        hogback.getVelocity());
                telemetry.update();

                idle();
            }
//
            if (opModeIsActive() && hogback.getVelocity() <= HOGBACK_MIN_VELOCITY) {

                intake1.setPower(INTAKE_POWER);
                intake2.setPower(INTAKE_POWER);
                sleep(2000);
                gate.setPosition(GATE_OPEN);

                telemetry.addData("Status", "Feeding pollen");
                telemetry.update();

                sleep(5000);

                intake1.setPower(INTAKE_STOP);
                intake2.setPower(INTAKE_STOP);
                gate.setPosition(GATE_CLOSED);
                hogback.setVelocity(0);

                telemetry.addData("Status", "Done");
                telemetry.update();
            }
        }
    }
}