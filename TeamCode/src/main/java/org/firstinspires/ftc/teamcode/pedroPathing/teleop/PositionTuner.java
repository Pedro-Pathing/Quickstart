package org.firstinspires.ftc.teamcode.pedroPathing.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.wpilibcontroller.ProfiledPIDController;
import com.arcrobotics.ftclib.trajectory.TrapezoidProfile;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;


@Config
@TeleOp(name = "positionTuner", group = "Tuning")
public class PositionTuner extends LinearOpMode {
    public static double kP = 0;
    public static double kI = 0;
    public static double kD = 0;

    public static double maxV = 0;

    public static double maxA = 0;

    public static double targetPos = 0;

    private DcMotorEx motor;

    private FtcDashboard dashboard = FtcDashboard.getInstance();

    private ProfiledPIDController PIDController = new ProfiledPIDController(0,0,0,new TrapezoidProfile.Constraints(maxV, maxA));

    private ElapsedTime timer = new ElapsedTime();

    @Override
    public void runOpMode(){
        motor = hardwareMap.get(DcMotorEx.class,"Turret");
        motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        motor.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addLine("Tuner Ready");
        telemetry.update();

        waitForStart();
        timer.reset();

        while (opModeIsActive()) {
            PIDController.setPID(kP, kI, kD);
            PIDController.setConstraints(new TrapezoidProfile.Constraints(maxV, maxA));
            double power = PIDController.calculate(motor.getCurrentPosition(), targetPos);
            motor.setPower(power);

            telemetry.addData("Target", targetPos);
            telemetry.addData("Current position", motor.getCurrentPosition());
            telemetry.update();
        }
    }

}