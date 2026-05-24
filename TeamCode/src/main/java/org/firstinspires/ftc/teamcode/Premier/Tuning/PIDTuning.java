package org.firstinspires.ftc.teamcode.Premier.Tuning;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

@TeleOp(name = "PIDTuning", group = "Tuning")
@Configurable
public class PIDTuning extends LinearOpMode {

    //configurables
    public static double ks;
    public static double kv;

    public static double kp;
    public static double ki;
    public static double kd;

    public static double targetVelocity = 3500;
    public static double errorThreshold = 100;

    //non configurables
    private MotorEx launcher1, launcher2;
    private ElapsedTime spinUpTimer;

    @Override
    public void runOpMode() {



        while (opModeInInit()) {
            telemetry.addLine("------------------------------------");
            telemetry.addLine("|             PID TUNER            |");
            telemetry.addLine("| Feed Forward Coefficients:                |");
            telemetry.addData("| kS: ", "%.2f - kV: ", ks, "%.2f", kv);
            telemetry.addData("| kP: ", "%.2f - kI: ", kp, "%.2f - kD: ", ki, "%.2f", kd);
            telemetry.addData("| Target Velocity: ", "%.2f - Error Threshold: ", targetVelocity, "%.2f", errorThreshold);
            telemetry.addLine("------------------------------------");
            telemetry.update();

        }
        if (opModeIsActive()) {
            initMotors();

            while (opModeIsActive()) {
                if (launcher2.getVelocity() < targetVelocity + errorThreshold && launcher2.getVelocity() > targetVelocity - errorThreshold) {
                    double time = spinUpTimer.milliseconds();
                    telemetry.addData("Sped up in: ", "%.5f milliseconds", time);
                    telemetry.update();
                    launcher2.setVelocity(0);
                    launcher1.setVelocity(0);
                }
            }
        }

    }

    private void initMotors() {
        launcher1 = new MotorEx(hardwareMap, "launcherMotor1", Motor.GoBILDA.BARE);
        launcher2 = new MotorEx(hardwareMap, "launcherMotor2", Motor.GoBILDA.BARE);

        launcher1.setVeloCoefficients(kp,ki,kd);
        launcher1.setFeedforwardCoefficients(ks,kv);
        launcher1.setRunMode(Motor.RunMode.VelocityControl);

        launcher2.setInverted(true);
        launcher2.setVeloCoefficients(kp,ki,kd);
        launcher2.setFeedforwardCoefficients(ks,kv);
        launcher2.setRunMode(Motor.RunMode.VelocityControl);
    }

    private void runTest() {
        launcher1.setVelocity(targetVelocity);
        launcher2.setVelocity(targetVelocity);
        spinUpTimer = new ElapsedTime();
    }
}
