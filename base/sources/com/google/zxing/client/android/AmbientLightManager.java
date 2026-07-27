package com.google.zxing.client.android;

import android.content.Context;
import android.hardware.Sensor;
import android.hardware.SensorEvent;
import android.hardware.SensorEventListener;
import android.hardware.SensorManager;
import android.os.Handler;
import com.journeyapps.barcodescanner.camera.CameraManager;
import com.journeyapps.barcodescanner.camera.CameraSettings;

/* JADX INFO: loaded from: classes8.dex */
public final class AmbientLightManager implements SensorEventListener {
    private static final float BRIGHT_ENOUGH_LUX = 450.0f;
    private static final float TOO_DARK_LUX = 45.0f;
    private CameraManager cameraManager;
    private CameraSettings cameraSettings;
    private Context context;
    private Handler handler = new Handler();
    private Sensor lightSensor;

    public AmbientLightManager(Context context, CameraManager cameraManager, CameraSettings settings) {
        this.context = context;
        this.cameraManager = cameraManager;
        this.cameraSettings = settings;
    }

    public void start() {
        if (this.cameraSettings.isAutoTorchEnabled()) {
            SensorManager sensorManager = (SensorManager) this.context.getSystemService("sensor");
            this.lightSensor = sensorManager.getDefaultSensor(5);
            if (this.lightSensor != null) {
                sensorManager.registerListener(this, this.lightSensor, 3);
            }
        }
    }

    public void stop() {
        if (this.lightSensor != null) {
            SensorManager sensorManager = (SensorManager) this.context.getSystemService("sensor");
            sensorManager.unregisterListener(this);
            this.lightSensor = null;
        }
    }

    private void setTorch(final boolean on) {
        this.handler.post(new Runnable() { // from class: com.google.zxing.client.android.AmbientLightManager$$ExternalSyntheticLambda0
            @Override // java.lang.Runnable
            public final void run() {
                this.f$0.m136lambda$setTorch$0$comgooglezxingclientandroidAmbientLightManager(on);
            }
        });
    }

    /* JADX INFO: renamed from: lambda$setTorch$0$com-google-zxing-client-android-AmbientLightManager, reason: not valid java name */
    /* synthetic */ void m136lambda$setTorch$0$comgooglezxingclientandroidAmbientLightManager(boolean on) {
        this.cameraManager.setTorch(on);
    }

    @Override // android.hardware.SensorEventListener
    public void onSensorChanged(SensorEvent sensorEvent) {
        float ambientLightLux = sensorEvent.values[0];
        if (this.cameraManager != null) {
            if (ambientLightLux <= TOO_DARK_LUX) {
                setTorch(true);
            } else if (ambientLightLux >= BRIGHT_ENOUGH_LUX) {
                setTorch(false);
            }
        }
    }

    @Override // android.hardware.SensorEventListener
    public void onAccuracyChanged(Sensor sensor, int accuracy) {
    }
}
