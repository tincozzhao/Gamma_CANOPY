package org.firstinspires.ftc.teamcode.auto;
//test


import static org.firstinspires.ftc.teamcode.common.hardware.BotCoefficients.AUTO_SLIDER_UP;
import static org.firstinspires.ftc.teamcode.common.hardware.BotCoefficients.SLIDER_DOWN_POWER;
import static org.firstinspires.ftc.teamcode.common.hardware.BotCoefficients.SLIDER_HOLD_POWER;
import static org.firstinspires.ftc.teamcode.common.hardware.BotCoefficients.grabberXClose;
import static org.firstinspires.ftc.teamcode.common.hardware.BotCoefficients.grabberXOpen;
import static org.firstinspires.ftc.teamcode.common.hardware.BotCoefficients.grabberXtiltDown;
import static org.firstinspires.ftc.teamcode.common.hardware.BotCoefficients.grabberXtiltUp;
import static org.firstinspires.ftc.teamcode.common.hardware.BotCoefficients.grabberYClose;
import static org.firstinspires.ftc.teamcode.common.hardware.BotCoefficients.grabberYOpen;
import static org.firstinspires.ftc.teamcode.common.hardware.BotCoefficients.grabberYtiltDown;
import static org.firstinspires.ftc.teamcode.common.hardware.BotCoefficients.grabberYtiltUp;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.common.hardware.RobotHardware;

//ignore this for now
@Autonomous(name="HighBasket_netZone2")
public class HighBasket_netZone2 extends LinearOpMode {
    RobotHardware robot = new RobotHardware();
    // Motor encoder parameter
    double ticksPerInch = 31.3;
    double ticksPerDegree = 15.6;


    @Override
    public void runOpMode() throws InterruptedException {
        robot.init(hardwareMap);
//.
        //reset encoder
        robot.setAutoDriveMotorMode();

        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {

           // turnToTargetYaw(robot.yaw0 -90, 0.3, 10000);
            //robot.tiltServo.setPosition(0.65);




            int forwardTicks = -250; //forward
            driveMotors(forwardTicks,forwardTicks,forwardTicks,forwardTicks, 0.5, false, robot.yaw0);
            sleep (100);

            forwardTicks = 1000; // strafe left
            driveMotors(forwardTicks,-forwardTicks,-forwardTicks,forwardTicks, 0.5, false, robot.yaw0);
            sleep (100);

            //encoder drive BACKWARD
           //forwardTicks = 100;
           //driveMotors(forwardTicks,forwardTicks,forwardTicks,forwardTicks, 0.5, false, robot.yaw0);
          // sleep (2000);
           //ticks 4000/97 ticks each inch

            forwardTicks = -400; //rotate clockwise
            driveMotors(forwardTicks,forwardTicks,-forwardTicks,-forwardTicks, 0.5, false, robot.yaw0);
            sleep (100);

            //robot.liftArm.(BotCoefficients.SLIDER_TOP_POSITION);
           // sleep(4000);

            forwardTicks = 400; //rotate counter clockwise
            driveMotors(forwardTicks,forwardTicks,-forwardTicks,-forwardTicks, 0.5, false, robot.yaw0);
            sleep (100);

           forwardTicks = -400; // strafe rt
            driveMotors(forwardTicks,-forwardTicks,-forwardTicks,forwardTicks, 0.6, false, robot.yaw0);
            sleep (100);

            forwardTicks = -900; //forward
            driveMotors(forwardTicks,forwardTicks,forwardTicks,forwardTicks, 0.5, false, robot.yaw0);
            sleep (100);


            forwardTicks = 880; //backward
            driveMotors(forwardTicks,forwardTicks,forwardTicks,forwardTicks, 0.5, false, robot.yaw0);
            sleep (100);

            forwardTicks = 395; // strafe left
            driveMotors(forwardTicks,-forwardTicks,-forwardTicks,forwardTicks, 0.6, false, robot.yaw0);
            sleep (100);

            forwardTicks = -400; //rotate clockwise
            driveMotors(forwardTicks,forwardTicks,-forwardTicks,-forwardTicks, 0.5, false, robot.yaw0);
            sleep (100);

            forwardTicks = 400; //rotate counter clockwise
            driveMotors(forwardTicks,forwardTicks,-forwardTicks,-forwardTicks, 0.5, false, robot.yaw0);
            sleep (100);

            forwardTicks = -850; // strafe rt
            driveMotors(forwardTicks,-forwardTicks,-forwardTicks,forwardTicks, 0.6, false, robot.yaw0);
            sleep (100);

            forwardTicks = -1900; //forward
            driveMotors(forwardTicks,forwardTicks,forwardTicks,forwardTicks, 0.5, false, robot.yaw0);
            sleep (100);

            forwardTicks = 1000; //rotate counter clockwise
            driveMotors(forwardTicks,forwardTicks,-forwardTicks,-forwardTicks, 0.5, false, robot.yaw0);
            sleep (100);

            forwardTicks = 750; //forward
            driveMotors(forwardTicks,forwardTicks,forwardTicks,forwardTicks, 0.5, false, robot.yaw0);
            sleep (100);

            /* forwardTicks = 1000; //rotate counter clockwise
            driveMotors(forwardTicks,forwardTicks,-forwardTicks,-forwardTicks, 0.5, false, robot.yaw0);
            sleep (1500);

            forwardTicks = 700; //forward
            driveMotors(forwardTicks,forwardTicks,forwardTicks,forwardTicks, 0.5, false, robot.yaw0);
            sleep (1000);

            */
            //robot.grabberYtilt.setPosition(grabberYtiltUp);
          //  sleep(2700);







          //  forwardTicks = 1000; //forwards
        //    driveMotors(forwardTicks,forwardTicks,forwardTicks,forwardTicks, 0.5, false, robot.yaw0);
         //   sleep (2000);






           // robot.liftArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            //robot.liftArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            //robot.liftArm.setPower(Math.abs(BotCoefficients.SLIDER_UP_SPEED));
            //sleep(4000);

           // turnToTargetYaw(90, 0.5, 1000);



         //   forwardTicks = -1000;
           // driveMotors(forwardTicks,forwardTicks,forwardTicks,forwardTicks, 0.3, true, robot.yaw0);

          //  sleep(1000);



        }


    }

    private void driveMotors(int flTarget, int blTarget, int frTarget, int brTarget,
                             double power,
                             boolean bKeepYaw, double targetYaw){
        double currentYaw, diffYaw;
        double powerDeltaPct, powerL, powerR;
        int direction;

        robot.motorfl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorbl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorfr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorbr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.motorfl.setTargetPosition(flTarget);
        robot.motorbl.setTargetPosition(blTarget);
        robot.motorfr.setTargetPosition(frTarget);
        robot.motorbr.setTargetPosition(brTarget);

        robot.motorfl.setPower(power);
        robot.motorbl.setPower(power);
        robot.motorfr.setPower(power);
        robot.motorbr.setPower(power);

        robot.motorfl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorbl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorfr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorbr.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Defensive programming.
        // Use bKeepYaw only when all targets are the same, meaning moving in a straight line
        if (! ((flTarget == blTarget)
                && (flTarget == frTarget)
                && (flTarget == brTarget)) )
            bKeepYaw = false;
        direction = (flTarget > 0) ? 1 : -1;
        while(opModeIsActive() &&
                (robot.motorfl.isBusy() &&
                        robot.motorbl.isBusy() &&
                        robot.motorfr.isBusy() &&
                        robot.motorbr.isBusy())){
            if (bKeepYaw) {

                currentYaw = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                if (Math.abs(currentYaw - targetYaw) > 2.0)
                    powerDeltaPct = 0.25;
                else
                    powerDeltaPct = Math.abs(currentYaw - targetYaw) / 2.0 * 0.25;
                if (currentYaw < targetYaw) {
                    powerL = power * (1 - direction * powerDeltaPct);
                    powerR = power * (1 + direction * powerDeltaPct);
                }
                else {
                    powerL = power * (1 + direction * powerDeltaPct);
                    powerR = power * (1 - direction * powerDeltaPct);
                }
                if (powerL > 1.0)
                    powerL = 1.0;
                if (powerR > 1.0)
                    powerR = 1.0;
                robot.motorfl.setPower(powerL);
                robot.motorbl.setPower(powerL);
                robot.motorfr.setPower(powerR);
                robot.motorbr.setPower(powerR);
            }
            idle();
        }

        robot.motorfl.setPower(0);
        robot.motorbl.setPower(0);
        robot.motorfr.setPower(0);
        robot.motorbr.setPower(0);
    }

    private void nonEncoderDrive(){

    }

    private void driveStrafe(int flTarget, int blTarget, int frTarget, int brTarget,
                             double power,
                             boolean bKeepYaw, double targetYaw){
        double currentYaw, diffYaw;
        double powerDeltaPct, powerL, powerR;
        int direction;

        robot.motorfl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorbl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorfr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.motorbr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.motorfl.setTargetPosition(flTarget);
        robot.motorbl.setTargetPosition(blTarget);
        robot.motorfr.setTargetPosition(frTarget);
        robot.motorbr.setTargetPosition(brTarget);

        robot.motorfl.setPower(power);
        robot.motorbl.setPower(power);
        robot.motorfr.setPower(power);
        robot.motorbr.setPower(power);

        robot.motorfl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorbl.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorfr.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.motorbr.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        // Defensive programming.
        // Use bKeepYaw only when all targets are the same, meaning moving in a straight line
        if (! ((flTarget == blTarget)
                && (flTarget == frTarget)
                && (flTarget == brTarget)) )
            bKeepYaw = false;
        direction = (flTarget > 0) ? 1 : -1;
        while(opModeIsActive() &&
                (robot.motorfl.isBusy() &&
                        robot.motorbl.isBusy() &&
                        robot.motorfr.isBusy() &&
                        robot.motorbr.isBusy())){
            if (bKeepYaw) {

                currentYaw = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
                if (Math.abs(currentYaw - targetYaw) > 2.0)
                    powerDeltaPct = 0.25;
                else
                    powerDeltaPct = Math.abs(currentYaw - targetYaw) / 2.0 * 0.25;
                if (currentYaw < targetYaw) {
                    powerL = power * (1 - direction * powerDeltaPct);
                    powerR = power * (1 + direction * powerDeltaPct);
                }
                else {
                    powerL = power * (1 + direction * powerDeltaPct);
                    powerR = power * (1 - direction * powerDeltaPct);
                }
                if (powerL > 1.0)
                    powerL = 1.0;
                if (powerR > 1.0)
                    powerR = 1.0;
                robot.motorfl.setPower(powerL);
                robot.motorbl.setPower(powerL);
                robot.motorfr.setPower(powerR);
                robot.motorbr.setPower(powerR);
            }
            idle();
        }

        robot.motorfl.setPower(0);
        robot.motorbl.setPower(0);
        robot.motorfr.setPower(0);
        robot.motorbr.setPower(0);
    }


    private void turnToTargetYaw(double targetYawDegree, double power, long maxAllowedTimeInMills){
        long timeBegin, timeCurrent;
        double currentYaw = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);;
        int ticks, tickDirection;
        double factor = 1.0;

        double diffYaw = Math.abs(currentYaw - targetYawDegree);
        telemetry.addLine(String.format("\nCurrentYaw=%.2f\nTargetYaw=%.2f", currentYaw, targetYawDegree));
        telemetry.update();

        timeBegin = timeCurrent = System.currentTimeMillis();
        while (diffYaw > 0.5
                && opModeIsActive()
                && ((timeCurrent-timeBegin) < maxAllowedTimeInMills)) {
            ticks = (int) (diffYaw * ticksPerDegree);
            if (ticks > 200)
                ticks = 200;

            tickDirection = (currentYaw < targetYawDegree) ? -1 : 1;
            if (ticks < 1)
                break;
            if (diffYaw > 3)
                factor = 1.0;
            else
                factor = diffYaw / 3;
            driveMotors(
                    (int)(tickDirection * ticks),
                    (int)(tickDirection * ticks),
                    -(int)(tickDirection * ticks),
                    -(int)(tickDirection * ticks),
                    power * factor, false, 0);
            currentYaw = robot.imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
            timeCurrent = System.currentTimeMillis();
            diffYaw = Math.abs(currentYaw - targetYawDegree);

            telemetry.addLine(String.format("\nCurrentYaw=%.2f\nTargetYaw=%.2f\nTimeLapsed=%.2f ms",
                    currentYaw, targetYawDegree, (double)(timeCurrent-timeBegin)));
            telemetry.update();
        }
    }

    private void deployPreloadedPixel1(int timeIntervalMs) {
        // Deploy preloaded pixel 1
        //   robot.autoPixel.setPosition(1.0);
        sleep(timeIntervalMs);
        //   robot.autoPixel.setPosition(0.5);
        sleep(timeIntervalMs);

    }

}