package org.firstinspires.ftc.teamcode.subsystems;

public interface Subsystem {
    void update();
    void init();

    //Adding default stop and logPsiKitData methods... to be implemented in robot/main if we need them
    default void stop() {}

    default void logPsiKitData() {}
}