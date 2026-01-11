package org.firstinspires.ftc.teamcode.navigation;

import com.qualcomm.hardware.dfrobot.HuskyLens;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import java.util.concurrent.TimeUnit;

import java.util.Map.Entry;

public class camerahusky {
    HuskyLens.Block tag;


    private final int READ_PERIOD = 1;
    private final double TAG_REAL_WIDTH_CM = 20.5;
    private final double FOV_DEGREES = 60.0;
    private final int IMAGE_WIDTH = 320;
    private final double K = 400.0;

    private HuskyLens huskyLensDevice;
    private Deadline rateLimit;

    public void init(HardwareMap hardwareMap) {
        huskyLensDevice = hardwareMap.get(HuskyLens.class, "huskylens");
        rateLimit = new Deadline(READ_PERIOD, TimeUnit.SECONDS);
        rateLimit.expire();
        huskyLensDevice.selectAlgorithm(HuskyLens.Algorithm.TAG_RECOGNITION);
    }

    public boolean isConnected() {
        return huskyLensDevice != null && huskyLensDevice.knock();
    }

    public boolean canRead() {
        return rateLimit.hasExpired();
    }

    public void resetRateLimit() {
        rateLimit.reset();
    }

    public boolean hasTag() {
        return huskyLensDevice.blocks().length > 0;
    }

    public double getEstimatedDistanceCm() {
        HuskyLens.Block[] blocks = huskyLensDevice.blocks();
        if (blocks.length > 0) {
            int tagWidthPixels = blocks[0].width;
            return (K * TAG_REAL_WIDTH_CM) / tagWidthPixels;
        }
        return -1;
    }

    public double getEstimatedAngleDegrees() {
        HuskyLens.Block[] blocks = huskyLensDevice.blocks();
        if (blocks.length > 0) {
            int tagX = blocks[0].x;
            int pixelOffset = tagX - (IMAGE_WIDTH / 2);
            return (pixelOffset * FOV_DEGREES) / IMAGE_WIDTH;
        }
        return 0;
    }


}