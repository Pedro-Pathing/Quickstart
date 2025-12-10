package org.firstinspires.ftc.teamcode.pedroPathing.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

@Config
@TeleOp(name = "FlywheelVelocityTuner", group = "Tuning")
public class FlywheelVelocityTuner extends LinearOpMode {

    public static double targetVelocity = 1500;
    public static String motorType = "intake";
    public static double fP = 0.002;
    public static double fI = 0.0;
    public static double fD = 0.0;
    public static double fK = 0.001;

    private DcMotorEx controlledMotor;
    private FtcDashboard dashboard = FtcDashboard.getInstance();
    private Methods methods = new Methods();

    @Override
    public void runOpMode() {

        controlledMotor = hardwareMap.get(DcMotorEx.class, motorType);
        controlledMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        controlledMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        controlledMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        controlledMotor.setDirection(DcMotorSimple.Direction.REVERSE);

        telemetry.addLine("Velocity Tuner Ready");
        telemetry.update();

        String lastMotorType = motorType;

        waitForStart();
        methods.resetPID();

        while (opModeIsActive()) {

            if (!motorType.equals(lastMotorType)) {
                lastMotorType = motorType;

                controlledMotor = hardwareMap.get(DcMotorEx.class, motorType);
                controlledMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                controlledMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                controlledMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            }

            switch (motorType) {

                case "intake":
                    Values.intake_Values.iP = fP;
                    Values.intake_Values.iI = fI;
                    Values.intake_Values.iD = fD;
                    Values.intake_Values.iK = fK;
                    controlledMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    Values.intake_Values.intakePIDController.setPIDF(fP, fI, fD, fK);
                    break;

                case "flywheel":
                    Values.flywheel_Values.fP = fP;
                    Values.flywheel_Values.fI = fI;
                    Values.flywheel_Values.fD = fD;
                    Values.flywheel_Values.fF = fK;
                    controlledMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    Values.flywheel_Values.flywheelPIDController.setPIDF(fP, fI, fD, fK);
                    break;

                case "transfer":
                    Values.transfer_Values.trP = fP;
                    Values.transfer_Values.trI = fI;
                    Values.transfer_Values.trD = fD;
                    Values.transfer_Values.trF = fK;
                    controlledMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
                    Values.transfer_Values.transferPIDController.setPIDF(fP, fI, fD, fK);
                    break;

            }

            TelemetryPacket packet = new TelemetryPacket();
            packet.put("MotorType", motorType);
            packet.put("TargetVelocity", targetVelocity);
            packet.put("MeasuredVelocity", methods.velocity_PID(controlledMotor, targetVelocity, motorType));
            packet.put("Power", controlledMotor.getPower());
            dashboard.sendTelemetryPacket(packet);

            telemetry.update();
        }

        controlledMotor.setPower(0);
    }
}
