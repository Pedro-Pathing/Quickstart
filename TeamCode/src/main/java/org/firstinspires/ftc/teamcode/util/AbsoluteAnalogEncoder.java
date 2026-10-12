package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.AnalogInput;

public class AbsoluteAnalogEncoder {
    public static double DEFAULT_RANGE = 3.3;
    public double gearRatio;
    private final AnalogInput encoder;
    private double offset;
    private final double analogRange;
    private boolean inverted;

    public AbsoluteAnalogEncoder(AnalogInput enc){this(enc, DEFAULT_RANGE);}
    public AbsoluteAnalogEncoder(AnalogInput enc, double aRange){this(enc,aRange, 0, 1.0);}
    public AbsoluteAnalogEncoder(AnalogInput enc, double aRange, double off, double ratio){
        encoder = enc;
        analogRange = aRange;
        offset = off;
        inverted = false;
        gearRatio = ratio;
    }
    public AbsoluteAnalogEncoder zero(double off){
        offset = off;
        return this;
    }
    public AbsoluteAnalogEncoder setInverted(boolean invert){
        inverted = invert;
        return this;
    }
    public boolean getDirection() {
        return inverted;
    }

    private double pastPosition = 1;
    public double getCurrentPosition() {
        double pos = ((!inverted ? 1 - getVoltage() / analogRange : getVoltage() / analogRange) * 360) - offset; //Degreez
        pos = (pos % 360 + 360) % 360;
        pastPosition = pos * gearRatio;
        return pastPosition;
    }

    public AnalogInput getEncoder() {
        return encoder;
    }

    public double getVoltage(){
        return encoder.getVoltage();
    }
}
