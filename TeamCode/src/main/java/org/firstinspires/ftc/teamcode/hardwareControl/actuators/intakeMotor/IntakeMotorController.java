package org.firstinspires.ftc.teamcode.hardwareControl.actuators.intakeMotor;


import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.hardwareConfig.actuators.intakeMotor.IntakeMotorConstants;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.common.SimpleVelocityController;


public class IntakeMotorController {

    private DcMotorEx intakeMotor;

    private IntakeMotorConstants intakeMotorConstants;

    private double TOLERABLE_ERROR;
    private double targetVelocity;
    private double lastPower = 0.0;

    // --- Effort-based stuck detection ---
    private boolean isStuck = false;

    private static final double STUCK_CURRENT = 2.5;   // amps of large effort
    private static final long STUCK_TIME_MS = 200;

    private long effortStartTime = -1;

    // --- Unstick ---
    private static final long UNSTICK_DURATION_MS = 380;
    private static final double UNSTICK_REVERSE_POWER = -0.5;

    private boolean unsticking = false;
    private long unstickStartTime = -1;



    /**
     * velocity gain
     */
    private double Kp = 0.0005;

    private SimpleVelocityController velocityController;
    private boolean initialized = false;

    private static final IntakeMotorController INSTANCE = new IntakeMotorController();
    private Telemetry telemetry;

    private IntakeMotorController() {}

    public static IntakeMotorController getInstance() {
        return INSTANCE;
    }


    public void initialize(HardwareMap hardwareMap, Telemetry telemetry) {
        if (initialized) return;

        setupConstants();
        this.telemetry = telemetry;

        initializeMotor(hardwareMap);
        initializeConstants();
        initializeVelControl();

        initialized = true;
    }

    private void setupConstants() {
        intakeMotorConstants = new IntakeMotorConstants();
    }
    
    private void initializeMotor(HardwareMap hardwareMap) {
        intakeMotor = hardwareMap.get(DcMotorEx.class, intakeMotorConstants.motorName);

        MotorConfigurationType type = intakeMotor.getMotorType().clone();
        type.setTicksPerRev(intakeMotorConstants.ticksPerRev);
        type.setGearing(intakeMotorConstants.gearing);
        type.setAchieveableMaxRPMFraction(intakeMotorConstants.achievableMaxRPMFraction);

        intakeMotor.setMotorType(type);
        intakeMotor.setMode(intakeMotorConstants.resetMode);
        intakeMotor.setMode(intakeMotorConstants.mode);
        intakeMotor.setDirection(intakeMotorConstants.motorDirection);
    }

    private void initializeConstants() {
        TOLERABLE_ERROR = intakeMotorConstants.tolerableError;
    }

    private void initializeVelControl() {
        velocityController = new SimpleVelocityController(Kp, telemetry);
    }

    /** RPM */
    public double getCurrentVelocity() {
        return intakeMotor.getVelocity(AngleUnit.DEGREES) / 6.0;
    }

    public double getTargetVelocity() {
        return targetVelocity;
    }

    public void spinToTargetVelocity(double newTargetVelocity) {
        if (newTargetVelocity != targetVelocity) {
            velocityController.reset();
            targetVelocity = newTargetVelocity;
        }

        update();
    }

    public boolean isOnTarget() {
        return Math.abs(targetVelocity - getCurrentVelocity()) <= TOLERABLE_ERROR;
    }

    public void reset() {
        if (!initialized) return;

        velocityController.reset();
        intakeMotor.setMode(intakeMotorConstants.resetMode);
        intakeMotor.setMode(intakeMotorConstants.mode);
        intakeMotor.setDirection(intakeMotorConstants.motorDirection);
        intakeMotor.setZeroPowerBehavior(intakeMotorConstants.zeroPowerBehavior);

        initialized = false;
    }

    public boolean isStuck() {
        return isStuck;
    }

    public boolean isUnsticking() {
        return unsticking;
    }

    public void stop() {
        spinToTargetVelocity(0.0);
        intakeMotor.setPower(0.0);
    }

    public void clearStuck() {
        isStuck = false;
        effortStartTime = -1;
    }

    private void detectStuck(double current) {
        if (Math.abs(targetVelocity) < 10) {
            clearStuck();
            return;
        }

        boolean highEffort =
                Math.abs(current) > STUCK_CURRENT;

        if (highEffort) {
            if (effortStartTime < 0) {
                effortStartTime = System.currentTimeMillis();
            } else if (System.currentTimeMillis() - effortStartTime > STUCK_TIME_MS) {
                isStuck = true;
            }
        } else {
            clearStuck();
        }
    }

    private void startUnstick() {
        unsticking = true;
        unstickStartTime = System.currentTimeMillis();
        velocityController.reset();
        intakeMotor.setPower(UNSTICK_REVERSE_POWER);
    }


    public void update() {

        if (unsticking) {
            telemetry.addData("Intake Unsticking", true);
            if (System.currentTimeMillis() - unstickStartTime >= UNSTICK_DURATION_MS) {
                unsticking = false;
                clearStuck();
                velocityController.reset();
            } else {
                return; // hold reverse pulse
            }
        }

        double currentVelocity = getCurrentVelocity();

        double power = velocityController.calculate(targetVelocity, currentVelocity);
        double intakeCurrent = intakeMotor.getCurrent(CurrentUnit.AMPS);

        if (Math.abs(lastPower - power) > 0.02) {
            intakeMotor.setPower(power);
            lastPower = power;
        }

//        telemetry.addData("Intake Target RPM", targetVelocity);
//        telemetry.addData("Intake Current RPM", currentVelocity);
//        telemetry.addData("Intake Power", power);
//        telemetry.addData("Intake Current", intakeCurrent);


        detectStuck(intakeCurrent);

        if (isStuck && !unsticking) {
            startUnstick();
        }
    }
}

