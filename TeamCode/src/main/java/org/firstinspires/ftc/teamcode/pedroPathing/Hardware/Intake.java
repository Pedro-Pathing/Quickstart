package org.firstinspires.ftc.teamcode.pedroPathing.Hardware;
import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import androidx.annotation.NonNull;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Intake {

    private DcMotorEx IntakeBall;
    private  ElapsedTime statetimer = new ElapsedTime();

    private enum Intakeetat {
        IDLE,
        RAMASSAGE,
        EJECTION,
    }
    private Intakeetat intakeState;

    // --- constante du moteur d'intake
    private double intake_reverse = -2000;

    private double intake_fast= 3000;

    // parametre de bourrage

    private final double MINRPM = 120;
    private final int tempsblocage = 300;
    private Indexeur indexeur;


    private static final int TICKS_PER_REV_6000 = 28; // GoBilda 5203 ratio 1:1
    public void init(@NonNull HardwareMap hwMap) {

        IntakeBall = hwMap.get(DcMotorEx.class, "IntakeBall");
        IntakeBall.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        IntakeBall.setDirection(DcMotor.Direction.REVERSE);
        intakeState = Intakeetat.IDLE;
    }

    public Intake(Indexeur indexeur) { this.indexeur = indexeur; }

    public void update() {
        int ballcomptage = indexeur.getBalles();

        switch (intakeState) {
            case IDLE:
                setIntakeBallTargetRPM(0);
                if (ballcomptage == 0) {
                    statetimer.reset();
                    intakeState = Intakeetat.RAMASSAGE;
                }
                break;
            case RAMASSAGE:
                setIntakeBallTargetRPM(intake_fast);

                double rpm = IntakeBall.getVelocity()*60/TICKS_PER_REV_6000;

                if (ballcomptage == 3) {
                    intakeState = Intakeetat.IDLE;
                    break;

                }

                if (rpm<MINRPM && statetimer.milliseconds()>tempsblocage){
                    intakeState = Intakeetat.EJECTION;
                }
                break;
            case EJECTION:
                setIntakeBallTargetRPM(intake_reverse);
                if (statetimer.milliseconds()>500 && ballcomptage >3){
                intakeState = Intakeetat.IDLE;
                break;
              } else if (statetimer.milliseconds() > 500 && ballcomptage < 3) {
                    statetimer.reset();
                    intakeState = Intakeetat.RAMASSAGE;
                }
            }
            break;


        }

        public void setIntakeBallTargetRPM ( double targetRPM){
            // Conversion RPM -> ticks/sec
                double targetTicksPerSec = (targetRPM * TICKS_PER_REV_6000) / 60.0;
                IntakeBall.setVelocity(targetTicksPerSec);

            }
        }