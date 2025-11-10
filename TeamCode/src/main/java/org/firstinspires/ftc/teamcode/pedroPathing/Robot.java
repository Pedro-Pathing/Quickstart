package org.firstinspires.ftc.teamcode.pedroPathing;

import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

@Configurable
public class Robot {

    public DcMotorEx leftFront, rightFront, leftBack, rightBack;
    public DcMotorEx launcher1, launcher2;
    public DcMotorEx intake;

    public CRServo spindexer;

    public ServoImplEx bumper;


    public static double bumperRest;
    public static double bumperUp;

    public Robot (HardwareMap hardwareMap) {

        rightBack = hardwareMap.get(DcMotorEx.class, "rightBack");
        leftBack = hardwareMap.get(DcMotorEx.class, "leftBack");
        rightFront = hardwareMap.get(DcMotorEx.class, "rightFront");
        leftFront = hardwareMap.get(DcMotorEx.class, "leftFront");

        rightFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftFront.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        leftBack.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
        rightFront.setDirection(DcMotorSimple.Direction.REVERSE);
        rightBack.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.FORWARD);
        leftBack.setDirection(DcMotorSimple.Direction.FORWARD);

        launcher1 = (DcMotorEx) hardwareMap.dcMotor.get("launcher1");
        launcher2 = (DcMotorEx) hardwareMap.dcMotor.get("launcher2");

        launcher1.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcher1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcher1.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        launcher1.setDirection(DcMotorSimple.Direction.REVERSE);

        launcher2.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        launcher2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        launcher2.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intake = hardwareMap.get(DcMotorEx.class, "intake");

        spindexer = hardwareMap.get(CRServo.class, "spindexer");

        bumper = hardwareMap.get(ServoImplEx.class, "bumper");

    }






}
