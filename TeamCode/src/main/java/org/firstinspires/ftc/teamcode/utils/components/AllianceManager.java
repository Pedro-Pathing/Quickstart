package org.firstinspires.ftc.teamcode.utils.components;
import org.firstinspires.ftc.teamcode.utils.Alliance;

import dev.nextftc.core.components.Component;
import dev.nextftc.ftc.ActiveOpMode;

public class AllianceManager implements Component {

    public static final AllianceManager INSTANCE = new AllianceManager();
    public static Alliance currentAlliance = Alliance.BLUE;

    @Override public void preWaitForStart() {
        ActiveOpMode.telemetry().addData("Alliance", currentAlliance);
        ActiveOpMode.telemetry().addLine("Press 'X' to toggle current alliance");
        if (ActiveOpMode.gamepad1().xWasPressed()) {
            if (currentAlliance == Alliance.BLUE) {
                currentAlliance = Alliance.RED;
            } else {
                currentAlliance = Alliance.BLUE;
            }
        }
    }

    public static Alliance getCurrentAlliance(){
        return currentAlliance;
    }
    public static void setCurrentAlliance(Alliance newAlliance){ currentAlliance = newAlliance; }
}
