package org.firstinspires.ftc.teamcode.pedro.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.pedro.LeaveParkRoute.Alliance;

/**
 * Autonomous OpMode for executing the Red Alliance Leave + Park routine.
 */
@Autonomous(name = "Red Leave + Park", group = "Pedro")
public final class RedLeavePark extends AutoLeavePark {
    @Override protected Alliance alliance() { return Alliance.RED; }
}
