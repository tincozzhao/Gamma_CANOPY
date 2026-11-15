package org.firstinspires.ftc.teamcode.pedro.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.pedro.LeaveParkRoute.Alliance;

/**
 * Autonomous OpMode for executing the Red Alliance Shoot Two + Park routine.
 */
@Autonomous(name = "Red Shoot Two + Park", group = "Pedro")
public final class RedShootPark extends AutoShootPark {
    @Override protected Alliance alliance() { return Alliance.RED; }
}
