package org.firstinspires.ftc.teamcode.Testes;

import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Autonomo.pedro.AutoPath;
import org.firstinspires.ftc.teamcode.Autonomo.pedro.Constants;
import org.firstinspires.ftc.teamcode.Mech.FieldCentric;
import org.firstinspires.ftc.teamcode.Mech.TurnOn_TurnOff;

@TeleOp(name = "AutoPath&FieldCentric")
public class TestFieldAutoPath extends LinearOpMode {

    boolean lastOptions = false;
    boolean lastA = false, Hold = true;
    boolean lastB = false, DriveMode = false;
    Pose lastPose;

    @Override
    public void runOpMode(){

        Follower follower  = Constants.create(hardwareMap);
        follower.update();
        FieldCentric field = new FieldCentric(follower);
        field.resetHeading();
        TurnOn_TurnOff On_button = new TurnOn_TurnOff();
        AutoPath autoPath = new AutoPath(follower);
        double XtoAdjust = 0, YtoAdjust = 0;

        waitForStart();
        while (opModeIsActive()){

            double x  = gamepad1.left_stick_x;
            double y  = -gamepad1.left_stick_y;
            double rx = gamepad1.right_stick_x;
            Pose pose = follower.pose();

            boolean options = gamepad1.options;

            if (options && !lastOptions){
                field.resetHeading();
            }


            //Button's
            boolean A = gamepad1.a;
            Hold = On_button.IsActivate(A, lastA, Hold);

            boolean B = gamepad1.b;
            DriveMode = On_button.IsActivate(B, lastB, DriveMode);

            lastA = A; lastB = B;


            if (Hold && !lastA){

                XtoAdjust = Math.round(pose.x());
                YtoAdjust = Math.round(pose.y());
                lastPose = pose;
            }


            autoPath.GoToPosition(
                    gamepad1.dpad_left, gamepad1.dpad_right, gamepad1.dpad_down, gamepad1.dpad_up,
                    false, false, false, false);

            if (Hold){
                autoPath.HoldPosition(follower, lastPose, XtoAdjust,
                        YtoAdjust, pose.x(), pose.y());
            }
            else if (!gamepad1.dpad_left && !gamepad1.dpad_right
                    && !gamepad1.dpad_up && !gamepad1.dpad_down ) {
                DrivePowers power = DriveMode ? field.powers(x,y,rx) :new DrivePowers(x, y, rx);
                follower.manual(power);
            }


            DrivePowers powers = DriveMode ? field.powers(x,y,rx) :new DrivePowers(x, y, rx);
            follower.manual(powers);
            follower.update();


            telemetry.addData("Mode", DriveMode ? "Centric" : "Normal");
            telemetry.addLine("====================================");
            telemetry.addData("Travado", Hold ? "Lock" : "Normal");


        }
    }
}
