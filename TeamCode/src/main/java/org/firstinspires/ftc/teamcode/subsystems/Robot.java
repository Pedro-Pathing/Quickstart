package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.core.subsystems.SubsystemGroup;

public class Robot extends SubsystemGroup {
    public static final Robot INSTANCE = new Robot();

    private Robot() {
        super(
                Drive.INSTANCE,
                Intake.INSTANCE,
                Outtake.INSTANCE
        );
    }
}
