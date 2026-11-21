package org.firstinspires.ftc.teamcode.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;
import static org.firstinspires.ftc.teamcode.common.hardware.BotCoefficients.*;

@Autonomous(name = "intaketest", group = "Robot")
public class intaketest extends RobotHardware {

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