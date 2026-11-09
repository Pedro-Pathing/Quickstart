package org.firstinspires.ftc.teamcode.Mech;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Shooter {

    DcMotorEx shooter;
    VoltageSensor battery;
    ElapsedTime timer = new ElapsedTime();

    private final double p = 60;

    static final double BasePotence = 16.67;
    static final double PorcentOfBatteryOf = 12.97;

    double lastF = BasePotence;
    double cachedVolts;

    private double lastTuneP = -1, lastTuneI = -1, lastTuneD = -1, lastTuneF = -1;


    // Sem sensor: o init busca o sensor de tensao sozinho
    public Shooter(){
        this(null);
    }

    // Com sensor: usa o sensor passado pelo OpMode
    public Shooter(VoltageSensor battery){
        this.battery = battery;
    }

    public void init(HardwareMap hardwareMap){

        shooter = hardwareMap.get(DcMotorEx.class, "shooter");
        shooter.setDirection(DcMotorEx.Direction.FORWARD);
        shooter.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        shooter.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        shooter.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        shooter.setVelocityPIDFCoefficients(p, 0, 0, BasePotence);

        // So busca o sensor se nenhum foi passado no construtor
        if (battery == null && hardwareMap.voltageSensor.iterator().hasNext()){
            battery = hardwareMap.voltageSensor.iterator().next();
        }
        cachedVolts = (battery != null) ? battery.getVoltage() : PorcentOfBatteryOf;
        timer.reset();
    }

    public void Activate(boolean RT_ON, double TargetVelocity){

        if (RT_ON){

            if (timer.milliseconds() > 500){
                if (battery != null){
                    double v = battery.getVoltage();
                    // Ignora leitura invalida ou queda brusca de tensao
                    if (v > 7.0){
                        // Filtro: suaviza as quedas momentaneas ao acelerar o shooter
                        cachedVolts = 0.8 * cachedVolts + 0.2 * v;
                    }
                }
                timer.reset();
            }

            double f = BasePotence * (PorcentOfBatteryOf/cachedVolts);

            if(Math.abs(f - lastF) > 0.05){

                shooter.setVelocityPIDFCoefficients(p, 0,0, f);
                lastF = f;
            }

            shooter.setVelocity(TargetVelocity);
        }
        else{
            shooter.setVelocity(0);
        }

    }

    public void Tune(double P, double I, double D, double F, double rpm){
        if (P != lastTuneP || I != lastTuneI || D != lastTuneD || F != lastTuneF) {
            shooter.setVelocityPIDFCoefficients(P, I, D, F);
            lastTuneP = P;
            lastTuneI = I;
            lastTuneD = D;
            lastTuneF = F;
        }
        shooter.setVelocity(rpm);
    }

    public double getVelocity(){ return shooter.getVelocity();}

    public void ActivateTu(boolean RT_ON, double TargetVelocity){

        if (RT_ON){

            if (timer.milliseconds() > 500){
                if (battery != null){
                    double v = battery.getVoltage();
                    // Ignora leitura invalida ou queda brusca de tensao
                    if (v > 7.0){
                        // Filtro: suaviza as quedas momentaneas ao acelerar o shooter
                        cachedVolts = 0.8 * cachedVolts + 0.2 * v;
                    }
                }
                timer.reset();
            }

            double f = BasePotence * (PorcentOfBatteryOf/cachedVolts);

            if(Math.abs(f - lastF) > 0.05){

                shooter.setVelocityPIDFCoefficients(p, 0,0, f);
                lastF = f;
            }

            shooter.setVelocity(TargetVelocity);
        }
        else{
            shooter.setVelocity(0);
        }

    }
}
