package org.firstinspires.ftc.teamcode;


import com.pedropathing.follower.Follower;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.pedro.Constants;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;
import com.qualcomm.robotcore.util.Range;


@TeleOp
public class teleTest extends OpMode {


    robotConstats rConstats = new robotConstats();



    double vertical;
    double horizontal;
    double pivot;
    double[] stepsizes = {10.0, 1.0, 0.1, 0.001, 0.0001};
    int stepindex = 1;

    Pose goal = new Pose(84, 91, Math.toRadians(0));
    double angleTolerance = 4;
    double KP = .5;
    double KD = .5;
    double curTime = 0;
    double lastTime = 0;
    double lastError = 0;


    private Follower follower;




    @Override
    public void init() {


        rConstats.init(hardwareMap);

        follower = Constants.create(hardwareMap);

    }

    @Override
    public void start(){
        follower.setPose(OpModeStorage.autonomousEndPose);
        follower.update();

        resetRuntime();
        curTime = getRuntime();
    }


    @Override
    public void loop() {




        if (gamepad1.dpadRightWasPressed()){
            if (!rConstats.getFlowerMech()) {
                rConstats.flowerMechDown(true);
            }
            else {
                rConstats.flowerMechDown(false);
            }
        }


        if (gamepad1.dpadDownWasPressed()) {
            if (!rConstats.shooterOn) {
                rConstats.shooterOn(true);
            }
            else {
                rConstats.shooterOn(false);
            }
        }

        if (gamepad1.right_bumper) {
            rConstats.intakeOn(true);
        }
        else {
            rConstats.intakeOn(false);
        }
        rConstats.transferStaller();


        if (gamepad1.left_bumper) {
            rConstats.intakeOut();
        }

        if (gamepad1.right_trigger > .2){
            rConstats.shooting(true);
        }
        else{
            rConstats.shooting(false);
        }



        vertical = -gamepad1.left_stick_y;
        horizontal = -gamepad1.left_stick_x;
        pivot = -gamepad1.right_stick_x;

        double headingTrueDegree = Math.toDegrees(follower.pose().heading());





        /*if (follower.pose().y() > goal.y()) {
            angleToTheGoal = ((((Math.toDegrees(Math.atan((((goal.x() - follower.pose().x()) / (goal.y() - follower.pose().x()))))))) * -1) + 270);
        }
        else {
            angleToTheGoal = ((((Math.toDegrees(Math.atan((((goal.x() - follower.pose().x()) / (goal.y() - follower.pose().x()))))))) * -1) + 90);
        }
        */
        double angleToTheGoal = Math.toDegrees(Math.atan2(goal.y()-follower.pose().y(), goal.x() - follower.pose().x()));

        double error = ((angleToTheGoal - headingTrueDegree + 540) % 360 + 360) % 360 - 180;
        //-----------------
        if (gamepad1.left_trigger > 0.2) {


            if (Math.abs(error) < angleTolerance) {
                pivot = 0;
            }
            else {
                double pTerm = error * KP;

                curTime = getRuntime();
                double DT = curTime - lastTime;
                double dterm = ((error - lastError)/ DT) * KD;

                pivot = -1 * (Range.clip(pTerm + dterm, -0.7, 0.7));

                lastError = error;
                lastTime = curTime;
            }
        }
        else {
            lastError = 0;
            lastTime = getRuntime();
        }

        //follower.manual(vertical,horizontal,pivot);
        DrivePowers powers = ManualDrive.fieldCentric(vertical,horizontal,pivot, follower.pose().heading());
        ManualDrive.driveOrHold(follower,powers);
        //ManualDrive.driveOrHold(follower,vertical,horizontal,pivot);



        if (gamepad1.dpadUpWasPressed()) {
            Pose cornerPose = new Pose(8.5, 133, Math.toRadians(90));
            follower.setPose(cornerPose); // overrides our pose
        }

        follower.update();
        Pose robotPose = follower.pose();
        rConstats.callUpdate(robotPose);


        double curShootaPowwa = rConstats.shooter.getVelocity();
        double shootaError = rConstats.shooterSpeed - curShootaPowwa;

        telemetry.addData("target powwa", rConstats.shooterSpeed);
        telemetry.addData("current powwa", "%.2f", curShootaPowwa);
        telemetry.addData("Error", "%.2f", shootaError);
        telemetry.addLine("-----------------------------------");
        telemetry.addData("P tune", "%.4f (D-Pad U/D)", rConstats.P);
        telemetry.addData("F tune", "%.4f (D-Pad L/R)", rConstats.F);
        telemetry.addData("step size", "%.4f (B Button)", stepsizes[stepindex]);
        telemetry.addLine("-----------------------------------");
        telemetry.addData("gate position", rConstats.gate.getPosition());
        telemetry.addData("intake speed", rConstats.intakePower);
        telemetry.addData("transfer speed", rConstats.transferPower);
        telemetry.addLine("-----------------------------------");
        telemetry.addData("transfer over current", rConstats.transfer.getCurrent(CurrentUnit.AMPS));
        telemetry.addData("Robot X", robotPose.x());
        telemetry.addData("Robot Y", robotPose.y());
        telemetry.addData("Robot Heading", Math.toDegrees(robotPose.heading()));
        telemetry.addData("error", error);
        telemetry.update();
    }
}




