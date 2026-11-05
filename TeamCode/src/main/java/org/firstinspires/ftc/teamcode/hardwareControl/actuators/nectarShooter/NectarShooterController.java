package org.firstinspires.ftc.teamcode.hardwareControl.actuators.nectarShooter;


import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.typecontainers.MotorConfigurationType;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.hardwareConfig.actuators.nectarShooter.NectarShooterConstants;
import org.firstinspires.ftc.teamcode.hardwareConfig.actuators.nectarShooter.NectarShooterTBHControllerConstants;
import org.firstinspires.ftc.teamcode.hardwareControl.actuators.common.TBHController;

public class NectarShooterController {

    private DcMotorEx frontShooterMotor;
    private DcMotorEx rearShooterMotor;

    NectarShooterConstants shooterConstants;
    NectarShooterTBHControllerConstants tbhConstants;

    private double START_VELOCITY;
    private double TOLERABLE_ERROR;
    private double targetVelocity;


    private double FRONT_Kp;
    private double REAR_Kp;
    private double FRONT_Kf_a;
    private double FRONT_Kf_b;
    private double FRONT_Kf_c;
    private double REAR_Kf_a;
    private double REAR_Kf_b;
    private double REAR_Kf_c;

    /// motion control
    private TBHController frontTbhController;
    private TBHController rearTbhController;

    private double currentFrontVelocity;
    private double currentRearVelocity;

    private double FRONTLastPower = 0.0;
    private double REARLastPower = 0.0;

    ///

    private boolean initialized = false;

    // Private static instance (eager initialization)
    private static final NectarShooterController INSTANCE = new NectarShooterController();
    private Telemetry telemetry;

    // Private constructor to prevent instantiation
    private NectarShooterController() {

    }

    // Public method to access the singleton instance
    public static NectarShooterController getInstance() {
        return INSTANCE;
    }


    // Initialization method — must be called once at the beginning
    public void initialize(HardwareMap hardwareMap, Telemetry telemetry) {
        if (initialized) {
            return;
        }
        this.telemetry = telemetry;
        setupConstants();

        initializeMotor(hardwareMap);
        initializeLocalVariablesWithConstants();
        initializeTBHController();

        initialized = true;
    }

    private void setupConstants() {
        shooterConstants = new NectarShooterConstants();
    }

    private void initializeMotor(HardwareMap hardwareMap){
        frontShooterMotor = hardwareMap.get(DcMotorEx.class, shooterConstants.frontMotorName);
        MotorConfigurationType motorConfigurationType = frontShooterMotor.getMotorType().clone();
        rearShooterMotor = hardwareMap.get(DcMotorEx.class, shooterConstants.rearMotorName);
        motorConfigurationType.setTicksPerRev(shooterConstants.ticksPerRev);
        motorConfigurationType.setGearing(shooterConstants.gearing);
        motorConfigurationType.setAchieveableMaxRPMFraction(shooterConstants.achievableMaxRPMFraction);

        frontShooterMotor.setMotorType(motorConfigurationType);
        frontShooterMotor.setMode(shooterConstants.resetMode);
        frontShooterMotor.setMode(shooterConstants.mode);
        frontShooterMotor.setDirection(shooterConstants.frontMotorDirection);
        frontShooterMotor.setPower(0.0);

        rearShooterMotor.setMotorType(motorConfigurationType);
        rearShooterMotor.setMode(shooterConstants.resetMode);
        rearShooterMotor.setMode(shooterConstants.mode);
        rearShooterMotor.setDirection(shooterConstants.rearMotorDirection);
        rearShooterMotor.setPower(0.0);
    }

    private void initializeLocalVariablesWithConstants(){
        tbhConstants = new NectarShooterTBHControllerConstants();

        START_VELOCITY = shooterConstants.startPosition;
        TOLERABLE_ERROR = shooterConstants.tolerableError;

        FRONT_Kp = tbhConstants.FRONT_Kp;
        REAR_Kp = tbhConstants.REAR_Kp;

        FRONT_Kf_a = tbhConstants.FRONT_Kf_a;
        FRONT_Kf_b = tbhConstants.FRONT_Kf_b;
        FRONT_Kf_c = tbhConstants.FRONT_Kf_c;

        REAR_Kf_a = tbhConstants.REAR_Kf_a;
        REAR_Kf_b = tbhConstants.REAR_Kf_b;
        REAR_Kf_c = tbhConstants.REAR_Kf_c;
    }

    private void initializeTBHController(){

        frontTbhController = new TBHController(FRONT_Kp, FRONT_Kf_a, FRONT_Kf_b, FRONT_Kf_c, telemetry);
        rearTbhController = new TBHController(REAR_Kp, REAR_Kf_a, REAR_Kf_b, REAR_Kf_c, telemetry);
    }


    public void reset() {
        if(initialized) {
            if (!isOnTarget()) {
                this.spinToTargetVelocity(START_VELOCITY);
            } else {
                frontShooterMotor.setMode(shooterConstants.resetMode);
                frontShooterMotor.setMode(shooterConstants.mode);
                frontShooterMotor.setDirection(shooterConstants.frontMotorDirection);
                frontShooterMotor.setZeroPowerBehavior(shooterConstants.frontMotorZeroPowerBehavior);

                rearShooterMotor.setMode(shooterConstants.resetMode);
                rearShooterMotor.setMode(shooterConstants.mode);
                rearShooterMotor.setDirection(shooterConstants.rearMotorDirection);
                rearShooterMotor.setZeroPowerBehavior(shooterConstants.rearMotorZeroPowerBehavior);


                targetVelocity = 0.0;
            }
            initialized = false;
        }
    }


    /**
     * Gets the velocity of the shooter motor
     * @return returns rotations per minute
     */
    public double getFrontCurrentVelocity() {
        return frontShooterMotor.getVelocity(AngleUnit.DEGREES) / 6;
    }

    /**
     * Gets the velocity of the shooter motor
     * @return returns rotations per minute
     */
    public double getRearCurrentVelocity() {
        return rearShooterMotor.getVelocity(AngleUnit.DEGREES) / 6;
    }

    private void setFrontPower(double power) {
        frontShooterMotor.setPower(power);
    }

    private void setRearPower(double power) {
        rearShooterMotor.setPower(power);
    }

    public void stop() {
        spinToTargetVelocity(0.0);
    }

    public void spinToTargetVelocity(double newTargetVelocity){


        if (newTargetVelocity == 0.0 && targetVelocity != 0.0) {
            frontShooterMotor.setPower(0.0);
            rearShooterMotor.setPower(0.0);
        }

        targetVelocity = newTargetVelocity;

        if (targetVelocity != 0.0) {
            update();
        }
    }

    public boolean isOnTarget(){
        currentFrontVelocity = getFrontCurrentVelocity();
        currentRearVelocity = getRearCurrentVelocity();

        return ((Math.abs(targetVelocity - currentFrontVelocity) <= TOLERABLE_ERROR)
                && (Math.abs(targetVelocity - currentRearVelocity) <= TOLERABLE_ERROR));
    }

    public void update(){
        if (targetVelocity == 0.0) {
            FRONTLastPower = 0.0;
            REARLastPower = 0.0;
            return;
        }

        currentFrontVelocity = getFrontCurrentVelocity();
        currentRearVelocity = getRearCurrentVelocity();

        // Calculate motor tbhPower using TBH Spinning the front motor faster so that it can keep the right momentum
        double FRONTtbhPower = frontTbhController.calculate(targetVelocity, currentFrontVelocity);
        double REARtbhPower = rearTbhController.calculate(targetVelocity, currentRearVelocity);

        telemetry.addData("[ShooterController] CurrentFrontVelocity", getFrontCurrentVelocity());
        telemetry.addData("[ShooterController] CurrentRearVelocity", getRearCurrentVelocity());
        telemetry.addData("[ShooterController] TargetVelocity", targetVelocity);


        //Apply power to the motor if this is the first loop
        //Or if it's substantially different than what the motor is currently running at.
        //Otherwise save time by ignoring small changes in power.
        if ((Math.abs(FRONTtbhPower - FRONTLastPower) >= 0.008)) {
            // Apply FRONTtbhPower to motor
            frontShooterMotor.setPower(FRONTtbhPower);
            FRONTLastPower = FRONTtbhPower;
        }

        if ((Math.abs(REARtbhPower - REARLastPower) >= 0.008)) {
            // Apply FRONTtbhPower to motor
            rearShooterMotor.setPower(REARtbhPower);
            REARLastPower = REARtbhPower;
        }
        telemetry.addData("[ShooterController] SentFrontpower:", FRONTtbhPower);
        telemetry.addData("[ShooterController] SentRearpower:", REARtbhPower);

    }
}
