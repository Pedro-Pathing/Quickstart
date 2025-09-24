package org.firstinspires.ftc.teamcode.pedroPathing;


import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.hardware.limelightvision.LLResultTypes;
import com.qualcomm.hardware.limelightvision.LLStatus;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;


import org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

import java.util.Deque;
import java.util.LinkedList;
@TeleOp
public class LimeLight extends OpMode {
    Gamepad lastGamepad1 = new Gamepad(), lastGamepad2 = new Gamepad();
    Deque<Gamepad> gamepad1History = new LinkedList<>(), gamepad2History = new LinkedList<>();

    DcMotor motorFrontLeft;
    DcMotor motorBackLeft;
    DcMotor motorFrontRight;
    DcMotor motorBackRight;
    Limelight3A limelight;
    IMU

    @Override
    // runs on init press
    public void init() {
        // define and init robot
        motorFrontLeft = (DcMotorEx) hardwareMap.dcMotor.get("leftFront");
        motorBackLeft = (DcMotorEx) hardwareMap.dcMotor.get("leftBack");
        motorFrontRight = (DcMotorEx) hardwareMap.dcMotor.get("rightFront");
        motorBackRight = (DcMotorEx) hardwareMap.dcMotor.get("rightBack");

        motorFrontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorFrontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        motorBackRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        motorFrontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        motorBackLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        limelight = hardwareMap.get(Limelight3A.class, "limelight");
        limelight.setPollRateHz(100); // This sets how often we ask Limelight for data (100 times per second)
        limelight.start();


    }

    @Override
    // runs on start press
    public void start() {
        // run everything to start positions

    }

    @Override
    // loops after start press
    public void loop() {
        if (gamepad2.start || gamepad1.start) return;
        double y = -gamepad1.left_stick_y;
        double x = gamepad1.left_stick_x;
        double rx = gamepad1.right_stick_x;

        if (!gamepad1.right_bumper){
            motorFrontLeft.setPower(y + x + rx);
            motorBackLeft.setPower(y - x + rx);
            motorFrontRight.setPower(y - x - rx);
            motorBackRight.setPower(y + x - rx);
        }
        else {
            motorFrontLeft.setPower(0.3 * (y + x + rx));
            motorBackLeft.setPower(0.3 * (y - x + rx));
            motorFrontRight.setPower(0.3 * (y - x - rx));
            motorBackRight.setPower(0.3 * (y + x - rx));
        }

        LLResult result = limelight.getLatestResult();
        if (result != null && result.isValid()) {
            double tx = result.getTx(); // How far left or right the target is (degrees)
            double ty = result.getTy(); // How far up or down the target is (degrees)
            double ta = result.getTa(); // How big the target looks (0%-100% of the image)

            telemetry.addData("Target X", tx);
            telemetry.addData("Target Y", ty);
            telemetry.addData("Target Area", ta);
        } else {
            telemetry.addData("Limelight", "No Targets");
        }

        // Sending numbers to Python
        double[] inputs = {1.0, 2.0, 3.0, 4.0, 5.0, 6.0, 7.0, 8.0};
        limelight.updatePythonInputs(inputs);

        // Getting numbers from Python
        double[] pythonOutputs = result.getPythonOutput();
        if (pythonOutputs != null && pythonOutputs.length > 0) {
            double firstOutput = pythonOutputs[0];
            telemetry.addData("Python output:", firstOutput);
        }

        double robotYaw = imu.getAngularOrientation().firstAngle;
        limelight.updateRobotOrientation(robotYaw);
        if (result != null && result.isValid()) {
            Pose3D botpose_mt2 = result.getBotpose_MT2();
            if (botpose_mt2 != null) {
                double x = botpose_mt2.getPosition().x;
                double y = botpose_mt2.getPosition().y;
                telemetry.addData("MT2 Location:", "(" + x + ", " + y + ")");
            }
        }

    }


    }

