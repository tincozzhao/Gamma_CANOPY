package org.firstinspires.ftc.teamcode.pedro.opmode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.teamcode.pedro.LeaveParkRoute.Alliance;

/**
 * Autonomous OpMode for executing the Blue Alliance Leave + Park routine.
 */
@Autonomous(name = "Blue Leave + Park", group = "Pedro")
public final class BlueLeavePark extends AutoLeavePark {
    @Override protected Alliance alliance() { return Alliance.BLUE; }
}
