package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "intaketest", group = "Robot")
public class intaketest extends AutoHardware {

    private static final double HOGBACK_TARGET_VELOCITY = 1600;
    private static final double HOGBACK_MIN_VELOCITY = 1500;

    private static final double PUSHER0_OPEN = 0.5;
    private static final double PUSHER0_CLOSED = 0.0;

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
                    && hogback.getVelocity() < HOGBACK_MIN_VELOCITY
                    && getRuntime() - startTime < 4.0) {

                telemetry.addData("Hogback Velocity",
                        hogback.getVelocity());
                telemetry.update();

                idle();
            }
            //
            if (opModeIsActive()
                    && hogback.getVelocity() >= HOGBACK_MIN_VELOCITY) {

                // Open pusher0
                pusher0.setPosition(PUSHER0_OPEN);

                // Activate pusher1
                pusher1.setPosition(1.0);

                // Start intake
                intake.setPower(0.3);

                telemetry.addData("Status", "Feeding pollen");
                telemetry.update();

                sleep(5000);

                // Close pusher0
                pusher0.setPosition(PUSHER0_CLOSED);
            }

            // Stop all mechanisms
            pusher1.setPosition(0);
            intake.setPower(0);
            hogback.setVelocity(0);

            telemetry.addData("Status", "Done");
            telemetry.update();
        }
    }
}