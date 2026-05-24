package org.firstinspires.ftc.teamcode.Premier.Tuning;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.seattlesolvers.solverslib.hardware.motors.Motor;
import com.seattlesolvers.solverslib.hardware.motors.MotorEx;

@TeleOp(name = "DrivetrainSetup", group = "Tuning")
@Configurable
public class DrivetrainSetup extends LinearOpMode {

    //non configurables
    private MotorEx test;

    @Override
    public void runOpMode() {



        while (opModeInInit()) {


        }
        if (opModeIsActive()) {
            test.set(1000);
        }

    }

    private void initMotors() {
        test = new MotorEx(hardwareMap, "test", Motor.GoBILDA.BARE);

        test.setRunMode(Motor.RunMode.RawPower);
    }


}
