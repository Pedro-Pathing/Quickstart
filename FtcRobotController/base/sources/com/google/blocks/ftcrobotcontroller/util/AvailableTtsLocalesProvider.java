package com.google.blocks.ftcrobotcontroller.util;

import android.os.Looper;
import com.qualcomm.robotcore.util.RobotLog;
import java.util.HashSet;
import java.util.Locale;
import java.util.Set;
import org.firstinspires.ftc.robotcore.external.android.AndroidTextToSpeech;

/* JADX INFO: loaded from: classes8.dex */
public class AvailableTtsLocalesProvider {
    private static final String TAG = "AvailableTtsLocalesProvider";
    private static final AvailableTtsLocalesProvider instance = new AvailableTtsLocalesProvider();
    private Set<Locale> availableTtsLocales = null;

    public static AvailableTtsLocalesProvider getInstance() {
        return instance;
    }

    public synchronized Set<Locale> getAvailableTtsLocales() {
        if (Looper.getMainLooper().getThread() == Thread.currentThread()) {
            RobotLog.ee(TAG, "AvailableTtsLocalesProvider used from Android Main Thread. This is not allowed.");
            throw new RuntimeException("AvailableTtsLocalesProvider used from Android Main Thread. This is not allowed.");
        }
        if (this.availableTtsLocales == null) {
            this.availableTtsLocales = new HashSet();
            AndroidTextToSpeech textToSpeech = new AndroidTextToSpeech();
            textToSpeech.initialize();
            for (Locale locale : Locale.getAvailableLocales()) {
                if (textToSpeech.isLocaleAvailable(locale)) {
                    this.availableTtsLocales.add(locale);
                }
            }
            textToSpeech.close();
        }
        return this.availableTtsLocales;
    }
}
