package org.firstinspires.ftc.teamcode.prototyping.SOTM;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.configurables.annotations.IgnoreConfigurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.seattlesolvers.solverslib.geometry.Translation2d;
import com.seattlesolvers.solverslib.geometry.Vector2d;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

@TeleOp(name = "SOTMTest")
@Configurable

public class SOTMTest extends LinearOpMode {
    @IgnoreConfigurable
    static TelemetryManager telemetryM;
    public static double botX=70;
    public static double botY=70;
    public static double botHeading=0;
    public static double botVelX=0;
    public static double botvelY=0;
    public static double botAccelX=0;
    public static double botAccelY=0;
    public static double launcherVelocity = 0;
    public static double goalX = 0;
    public static double goalY = 144;
    private MotorEx launcher;
    SOTM sotm = new SOTM();



    @Override
    public void runOpMode() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        while (opModeInInit()) {
            telemetryM.addLine("This time i'm really gonna do it.");
            telemetryM.update();
        }
        if (opModeIsActive()) {
            sotm.init();
            while (opModeIsActive()) {


                sotm.calculateSOTM(
                        new Pose(botX, botY, Math.toRadians(botHeading)),
                        new Translation2d(goalX, goalY),
                        new Vector2d(botVelX, botvelY),
                        new Vector2d(botAccelX, botAccelY),
                        /*launcher.getVelocity()*/launcherVelocity,
                        telemetryM);

                telemetryM.addData("Adjusted Goal X", sotm.virtualGoalX);
                telemetryM.addData("Adjusted Goal Y", sotm.virtualGoalY);
                telemetryM.addData("Old Distance",sotm.dist);
                telemetryM.addData("New distance", sotm.newdist);
                telemetryM.addData("Required Angle", sotm.angle);

                telemetryM.update();
            }
        }
    }
}
