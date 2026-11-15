package org.firstinspires.ftc.teamcode.pedro.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.pedro.LeaveParkRoute.Alliance;

/**
 * Autonomous OpMode for executing the Blue Alliance Shoot Two + Park routine.
 */
@Autonomous(name = "Blue Shoot Two + Park", group = "Pedro")
public final class BlueShootPark extends AutoShootPark {
    @Override protected Alliance alliance() { return Alliance.BLUE; }
}
