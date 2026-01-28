package org.firstinspires.ftc.teamcode.pedroPathing.navigation;

import com.qualcomm.hardware.dfrobot.HuskyLens;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.internal.system.Deadline;
import java.util.concurrent.TimeUnit;

public class Camerahusky {
    HuskyLens.Block tag;


    private final int READ_PERIOD = 1;
    private final double TAG_REAL_WIDTH_CM = 20.5;
    private final double FOV_DEGREES = 60.0;
    private final int IMAGE_WIDTH = 320;
    private final double K = 400.0;

    private HuskyLens huskyLensDevice;
    private Deadline rateLimit;

    private double[] buffer = new double[5];
    private int index = 0;
    private int count = 0;


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

    public double getDistanceFiableCm() {
        double cam = getEstimatedDistanceCm();
        if (cam < 0) return -1;

        // Correction en cm (calibration)
        double reel = 0.7078 * cam - 13.25;
        //double reel = 0.5096995 * Math.pow(cam, 1.023373);

        // Buffer circulaire de 5 valeurs
        buffer[index] = reel;
        index = (index + 1) % 5;
        if (count < 5) count++;

        // Pas encore 5 valeurs → pas fiable
        if (count < 5) return -1;

        // Cherche un groupe de ≥3 mesures dans ±5 cm
        for (int i = 0; i < 5; i++) {
            int c = 1;
            double somme = buffer[i];

            for (int j = 0; j < 5; j++) {
                if (i != j && Math.abs(buffer[j] - buffer[i]) <= 5.0) {
                    somme += buffer[j];
                    c++;
                }
            }

            if (c >= 3) {
                return somme / c;  // Renvoie UN SEUL double
            }
        }
        return -1;
    }
}





