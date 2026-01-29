package org.firstinspires.ftc.teamcode.Testing;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Mechanics.Color;
import org.firstinspires.ftc.teamcode.Mechanics.Robot;
import org.firstinspires.ftc.teamcode.Mechanics.Shooter;
import org.firstinspires.ftc.teamcode.Mechanics.Spind;
import org.firstinspires.ftc.teamcode.Mechanics.Vision;

@TeleOp(name = "Spindexer Test")
public class SpindTest extends OpMode {
    private double slot;
    private boolean yes = true;
    private boolean yes2 = true;
    Timer sigma;
    Vision camera = new Vision();
    @Override
    public void loop() {
        if (gamepad1.a) {
            Spind.setSpindToColor(Color.DetectedColor.PURPLE);
        }
//        if(gamepad1.b){
//            try {
//                Spind.Launch3Balls("PPG");
//            } catch (InterruptedException e) {
//                throw new RuntimeException(e);
//            }
//        }
        if(gamepad1.b){
            Spind.spinTheDexer(0.5);
            Spind.updateBallList(new Timer(),0);
        }
        if (gamepad1.x && yes) {
            sigma.resetTimer();
            yes = false;
        }else if(gamepad1.x){
            //Spind.updateBallList(sigma,0.75);
        }
        else if(!gamepad1.x)
            yes = true;

        if (gamepad1.a && yes2){
            sigma.resetTimer();
            yes2 = false;
        }
        else if (gamepad1.a) {
            try {
                Spind.Launch3Balls(sigma,.75,1);
            } catch (InterruptedException e) {
                throw new RuntimeException(e);
            }
        }
        else if (!gamepad1.a)
            yes2 = true;

        if (gamepad1.y) {
            Shooter.setPower(1);
        }

        Shooter.autoShotHood(72, 72, 0, true);

        //Spind.spinTheDexer(slot);
        for(int i =0;i<3;i++) {
            if(Spind.ballList[i]== Color.DetectedColor.PURPLE)
                telemetry.addData("list " + i, "purple");
            else if(Spind.ballList[i]== Color.DetectedColor.GREEN)
                telemetry.addData("list " + i, "green");
            else if(Spind.ballList[i]== Color.DetectedColor.UNKNOWN)
                telemetry.addData("list " + i, "no ball");
        }
        telemetry.update();
    }

    /** This method is called once at the init of the OpMode. **/
    @Override
    public void init() {
        Robot.init(hardwareMap);
        camera.initAprilTag(hardwareMap);
        try {
            camera.setManualExposure(6, 250, telemetry);
        } catch (InterruptedException e) {
            throw new RuntimeException(e);
        }
        sigma = new Timer();
    }

    /** This method is called continuously after Init while waiting for "play". **/
    @Override
    public void init_loop() {
        camera.findMotif();
    }

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {

    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {

    }
}
