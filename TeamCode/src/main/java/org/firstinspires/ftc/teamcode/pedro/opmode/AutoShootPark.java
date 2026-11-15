package org.firstinspires.ftc.teamcode.pedro.opmode;

import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.pedropathing.paths.Path;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.pedro.LeaveParkRoute;
import org.firstinspires.ftc.teamcode.pedro.LeaveParkRoute.Alliance;
import org.firstinspires.ftc.teamcode.pedro.ShootParkRoute;
import org.firstinspires.ftc.teamcode.pedro.PedroRobot;
import org.firstinspires.ftc.teamcode.pedro.TwoPollenFeed;

/**
 * ShootParkAuto is the abstract base class for autonomous routines featuring a front-facing
 * shooter, two timed feed attempts, reverse corridor pathing, and loading-zone parking.
 */
public abstract class AutoShootPark extends LinearOpMode {
    /**
     * Subclasses must provide their alliance (RED or BLUE).
     *
     * @return the Alliance color
     */
    protected abstract Alliance alliance();

    /**
     * Calculates the absolute heading error in radians between two Poses.
     *
     * @param a first Pose
     * @param b second Pose
     * @return absolute heading difference in radians
     */
    private static double headingError(Pose a, Pose b) {
        return Math.abs(Math.atan2(Math.sin(a.heading() - b.heading()), Math.cos(a.heading() - b.heading())));
    }

    /**
     * Follows a single path leg with timeout and target tolerance checks.
     *
     * @param robot              PedroRobot instance
     * @param follower           Pedro Follower instance
     * @param path               Path to follow
     * @param target             target destination Pose
     * @param autoTime           overall autonomous elapsed time
     * @param deadline           match time deadline for the routine
     * @param stage              description string of the current stage
     * @param positionTolerance  positional tolerance in inches
     * @param angleTolerance     heading tolerance in radians
     * @return true if target was successfully reached within tolerance
     */
    private boolean followLeg(PedroRobot robot, Follower follower, Path path, Pose target,
                              ElapsedTime autoTime, double deadline, String stage, double positionTolerance, double angleTolerance) {
        if (!opModeIsActive() || autoTime.seconds() >= deadline) return false;
        robot.follow(path);
        ElapsedTime segmentTime = new ElapsedTime();
        while (opModeIsActive() && follower.isBusy() && autoTime.seconds() < deadline
                && segmentTime.seconds() < ShootParkRoute.SEGMENT_TIMEOUT) {
            robot.update();
            telemetry.addData("Stage", stage);
            telemetry.addData("Pose", follower.pose());
            telemetry.addData("Auto elapsed", "%.1f s", autoTime.seconds());
            telemetry.update();
            idle();
        }
        if (!opModeIsActive()) return false;
        boolean reached = !follower.isBusy()
                && follower.pose().distance(target) <= positionTolerance
                && headingError(follower.pose(), target) <= angleTolerance;
        if (!reached) robot.stop();
        return reached;
    }

    /**
     * Executes the shooting sequence using the TwoPollenFeed state machine while holding position.
     *
     * @param robot     PedroRobot instance
     * @param follower  Pedro Follower instance
     * @param target    shooting target Pose to hold
     * @param autoTime  overall autonomous elapsed time
     * @param deadline  maximum deadline timestamp for shooting
     * @return status description string
     */
    private String shoot(PedroRobot robot, Follower follower, Pose target,
                         ElapsedTime autoTime, double deadline) {
        TwoPollenFeed feed = new TwoPollenFeed();
        feed.start(autoTime.seconds());
        follower.hold(target, true);
        TwoPollenFeed.Output out = null;
        try {
            while (opModeIsActive() && autoTime.seconds() < deadline) {
                robot.update();
                // Check if position, heading, linear speed, and angular speed are stable for shooting
                boolean aimStable = follower.pose().distance(target) <= ShootParkRoute.SHOOT_POSITION_TOLERANCE
                        && headingError(follower.pose(), target) <= ShootParkRoute.SHOOT_HEADING_TOLERANCE
                        && follower.velocity().linSpeed() <= ShootParkRoute.SHOOT_MAX_LINEAR_SPEED
                        && Math.abs(follower.velocity().omega) <= ShootParkRoute.SHOOT_MAX_ANGULAR_SPEED;
                out = feed.update(autoTime.seconds(), robot.hardware.hogback.getVelocity(), aimStable);
                if (out.shooterOn) robot.spinShooter();
                if (out.gateOpen) robot.openGate(); else robot.closeGate();
                if (out.intakesOn) robot.intakeIn(); else robot.intakeStop();
                if (!out.shooterOn) robot.stopShooter();
                telemetry.addData("Stage", "Shooting: %s", out.state);
                telemetry.addData("Timed feed pulses completed", "%d / 2", out.completedFeedPulses);
                telemetry.addData("Hogback velocity", robot.hardware.hogback.getVelocity());
                telemetry.addData("Aim settled", aimStable);
                telemetry.addData("Return starts by", "%.1f s", deadline);
                telemetry.update();
                if (out.terminal()) break;
                idle();
            }
            if (out != null && out.state == TwoPollenFeed.State.COMPLETE) {
                return "Two timed feed pulses completed (pollen count unconfirmed)";
            }
            if (!opModeIsActive()) return "Stopped by Driver Station";
            if (out != null && out.state == TwoPollenFeed.State.FAILED) return out.reason;
            feed.abort("Return-time budget reached");
            return "Shooting stopped to reserve return time";
        } finally {
            // Mechanisms are stopped before the first return path is allowed to start.
            robot.stop();
        }
    }

    /**
     * Executes the full Shoot Two + Park autonomous routine lifecycle.
     *
     * @throws InterruptedException if thread is interrupted
     */
    @Override public final void runOpMode() throws InterruptedException {
        PedroRobot robot = new PedroRobot(hardwareMap);
        try {
            Pose[] outbound = ShootParkRoute.outboundPoses(alliance());
            Pose[] returning = ShootParkRoute.returnPoses(alliance());
            Path[] outPaths = ShootParkRoute.paths(outbound);
            Path[] returnPaths = ShootParkRoute.paths(returning);
            Follower follower;
            try { follower = robot.enableAutonomous(); }
            catch (IllegalStateException e) {
                while (opModeInInit()) {
                    telemetry.addData("Setup required", e.getMessage());
                    telemetry.addLine("See SHOOT_PARK.md"); telemetry.update(); idle();
                }
                return;
            }
            follower.setPose(outbound[0]);
            while (opModeInInit()) {
                telemetry.addData("Auto", "%s Shoot Two + Park", alliance());
                telemetry.addData("Start center", outbound[0]);
                telemetry.addData("Front-facing shooting pose", outbound[3]);
                telemetry.addLine("Two timed feed pulses; no pollen-count sensor.");
                telemetry.addLine("Feed timing and physical aim require tuning. See SHOOT_PARK.md.");
                telemetry.update(); idle();
            }
            waitForStart();
            if (isStopRequested()) return;
            ElapsedTime autoTime = new ElapsedTime();
            boolean reachedShoot = true;
            // Execute outbound path legs to the shooting zone
            for (int i = 0; i < outPaths.length; i++) {
                if (!followLeg(robot, follower, outPaths[i], outbound[i + 1], autoTime,
                        ShootParkRoute.OUTBOUND_DEADLINE, "Drive to shooting zone " + (i + 1),
                        LeaveParkRoute.POSITION_TOLERANCE, LeaveParkRoute.HEADING_TOLERANCE)) {
                    reachedShoot = false; break;
                }
            }
            String result;
            boolean parked = false;
            if (reachedShoot && opModeIsActive()) {
                double arrival = autoTime.seconds();
                double shootDeadline = ShootParkRoute.shootingDeadline(arrival, alliance());
                if (shootDeadline - arrival >= ShootParkRoute.MIN_SHOOT_WINDOW) {
                    result = shoot(robot, follower, outbound[3], autoTime, shootDeadline);
                } else {
                    robot.stop();
                    result = "Skipped shooting to reserve return time";
                }
                parked = true;
                // Execute return and parking path legs
                for (int i = 0; i < returnPaths.length; i++) {
                    if (!followLeg(robot, follower, returnPaths[i], returning[i + 1], autoTime,
                            ShootParkRoute.POWERED_AUTO_DEADLINE, "Return and park " + (i + 1),
                            i == returnPaths.length - 1 ? ShootParkRoute.parkPositionTolerance(alliance())
                                    : LeaveParkRoute.POSITION_TOLERANCE,
                            i == returnPaths.length - 1 ? ShootParkRoute.parkHeadingTolerance(alliance())
                                    : LeaveParkRoute.HEADING_TOLERANCE)) {
                        parked = false; result += "; return path did not finish"; break;
                    }
                }
            } else result = "Navigation did not reach shooting pose; stopped without feeding";
            robot.stop();
            // Post-stop telemetry display loop
            while (opModeIsActive()) {
                // Track any post-stop drift without commanding drive or shooter power.
                robot.localizer.update();
                telemetry.addData("Result", result);
                telemetry.addData("Park path completed", parked);
                telemetry.addData("Current estimated pose", robot.localizer.pose());
                telemetry.update(); idle();
            }
        } finally { robot.stop(); }
    }
}
