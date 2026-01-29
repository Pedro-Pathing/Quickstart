package org.firstinspires.ftc.teamcode.Mechanics;

import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;

import static org.firstinspires.ftc.teamcode.Mechanics.Robot.intake;
import static org.firstinspires.ftc.teamcode.Mechanics.Robot.spindexer;
import static org.firstinspires.ftc.teamcode.Mechanics.Robot.transfer;

public class Spind {
    public static double spindexerAngle = 0, targetAngle = 0;
    public static Color.DetectedColor[] ballList = {Color.DetectedColor.UNKNOWN, Color.DetectedColor.UNKNOWN, Color.DetectedColor.UNKNOWN};
    public static boolean[] launchedBalls = {false,false,false};

    private static boolean p1, p2, p3;
    public static boolean spinTheDexer(double slot) {
        targetAngle = Constants.CPR312 / 3 * slot;
        spindexer.setTargetPosition((int)Math.round(targetAngle));
        spindexer.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        spindexer.setPower(1);

        spindexerAngle = spindexer.getCurrentPosition();

        intake.setPower(-.5);

        if (Math.abs(spindexerAngle - targetAngle) <= 10) {
            spindexer.setPower(0);
            intake.setPower(0);
            return true;
        }
        return false;
    }
    public static boolean spinTheDexer(double slot, int intaking) {
        targetAngle = Constants.CPR312 / 3 * slot;
        spindexer.setTargetPosition((int)Math.round(targetAngle));
        spindexer.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        spindexer.setPower(1);

        spindexerAngle = spindexer.getCurrentPosition();


        if (Math.abs(spindexerAngle - targetAngle) <= 10) {
            spindexer.setPower(0);
            return true;
        }
        return false;
    }
    public static boolean spinTheDexer(double slot, boolean Teleop) {
        targetAngle = Constants.CPR312 / 3 * slot;
        spindexer.setTargetPosition((int)Math.round(targetAngle));
        spindexer.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        spindexer.setPower(1);

        spindexerAngle = spindexer.getCurrentPosition();


        if (Math.abs(spindexerAngle - targetAngle) <= 10) {
            spindexer.setPower(0);
            return true;
        }
        return false;
    }
    public static boolean setSpindToColor(Color.DetectedColor color) {
        int index=-1;
        for(int i=0;i<3;i++){
            if (ballList[i]==color) {
                index = i;
                break;
            }
        }
        if(index==-1)
            return true;
        return spinTheDexer(index);
    }

    public static boolean intaking(Timer timer,double timeBetweenSpins){
        if(timer.getElapsedTimeSeconds()>4*timeBetweenSpins){
            spinTheDexer(0, 1);
            return true;
        }
        intake.setPower(-1);
        int index = (int)(timer.getElapsedTimeSeconds()/timeBetweenSpins);
        spinTheDexer(index + .25, 1);
        return false;
    }
    public static boolean updateBallList(Timer timer,double timeBetweenSpins) {
        if(timer.getElapsedTimeSeconds()>3*timeBetweenSpins){
            intake.setPower(0);
            return true;
        }
        intake.setPower(-0.5);
        int index=(int)(timer.getElapsedTimeSeconds()/timeBetweenSpins);
        if(spinTheDexer(index)&&Color.getColor()!= Color.DetectedColor.UNKNOWN) {
            ballList[index] = Color.getColor();
        }
        return false;
    }
    public static int getSigmaPosition() {
        return (int)Math.round(spindexer.getCurrentPosition() / Constants.CPR312 * 3);
    }
    public static boolean Launch3Balls(Timer timer,String motif,double timeBetweenShots) throws InterruptedException {
        if(timer.getElapsedTimeSeconds()>(3*timeBetweenShots)) {
            intake.setPower(0);
            Shooter.setPower(0);
            for(int i =0;i<3;i++)
                launchedBalls[i]=false;
            return true;
        }
        intake.setPower(-.5);
        int index = (int)(timer.getElapsedTimeSeconds()/timeBetweenShots);
        String[] motifList = motif.split("");
        if(motifList[index].equals("P")){
            if(setSpindToColor(Color.DetectedColor.PURPLE)) {
                launchedBalls[index]=true;
                transfer.setPosition(0.9);
                Thread.sleep(100);
                transfer.setPosition(0.4);
            }
        }
        else if(motifList[index].equals("G")){
            if(setSpindToColor(Color.DetectedColor.GREEN)) {
                launchedBalls[index]=true;
                transfer.setPosition(0.9);
                Thread.sleep(100);
                transfer.setPosition(0.4);
            }
        }
        for(int i =0;i<3;i++){
            if(!launchedBalls[i]){
                spinTheDexer(i);
                transfer.setPosition(0.9);
                Thread.sleep(100);
                transfer.setPosition(0.4);
            }
        }
        return false;
    }
    public static boolean Launch3Balls(Timer timer, double timeBetweenShots,double shooterSpeedTime) throws InterruptedException {
        if (timer.getElapsedTimeSeconds() < .1) {
            p1 = false;
            p2 = false;
            p3 = false;
            System.out.println("please?");
        }
        intake.setPower(-.5);
        if(timer.getElapsedTimeSeconds()<shooterSpeedTime)
            return false;


        System.out.println("Seriously? just here");
        if (!p1 && spinTheDexer(0)) {
            System.out.println("at least we're here");
            transfer.setPosition(.9);
            Thread.sleep(100);
            transfer.setPosition(.4);
            Thread.sleep(75);
            p1 = true;
        }

        if (!p2 && p1 && spinTheDexer(1)) {
            System.out.println("one step closer to freedom");
            transfer.setPosition(.9);
            Thread.sleep(100);
            transfer.setPosition(.4);
            Thread.sleep(75);
            p2 = true;
        }

        if (!p3 && p1 && p2 && spinTheDexer(2)) {
            System.out.println("freedom");
            transfer.setPosition(.9);
            Thread.sleep(100);
            transfer.setPosition(.4);
            Thread.sleep(75);
            p3 = true;
        }

        return p1 && p2 && p3;
    }
}

