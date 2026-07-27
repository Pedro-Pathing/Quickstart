package com.google.zxing.client.android;

import android.content.BroadcastReceiver;
import android.content.Context;
import android.content.Intent;
import android.content.IntentFilter;
import android.os.Handler;

/* JADX INFO: loaded from: classes8.dex */
public final class InactivityTimer {
    private static final long INACTIVITY_DELAY_MS = 300000;
    private static final String TAG = InactivityTimer.class.getSimpleName();
    private Runnable callback;
    private final Context context;
    private boolean onBattery;
    private boolean registered = false;
    private final BroadcastReceiver powerStatusReceiver = new PowerStatusReceiver();
    private Handler handler = new Handler();

    public InactivityTimer(Context context, Runnable callback) {
        this.context = context;
        this.callback = callback;
    }

    public void activity() {
        cancelCallback();
        if (this.onBattery) {
            this.handler.postDelayed(this.callback, INACTIVITY_DELAY_MS);
        }
    }

    public void start() {
        registerReceiver();
        activity();
    }

    public void cancel() {
        cancelCallback();
        unregisterReceiver();
    }

    private void unregisterReceiver() {
        if (this.registered) {
            this.context.unregisterReceiver(this.powerStatusReceiver);
            this.registered = false;
        }
    }

    private void registerReceiver() {
        if (!this.registered) {
            this.context.registerReceiver(this.powerStatusReceiver, new IntentFilter("android.intent.action.BATTERY_CHANGED"));
            this.registered = true;
        }
    }

    private void cancelCallback() {
        this.handler.removeCallbacksAndMessages(null);
    }

    /* JADX INFO: Access modifiers changed from: private */
    public void onBattery(boolean onBattery) {
        this.onBattery = onBattery;
        if (this.registered) {
            activity();
        }
    }

    /* JADX INFO: Access modifiers changed from: private */
    final class PowerStatusReceiver extends BroadcastReceiver {
        private PowerStatusReceiver() {
        }

        @Override // android.content.BroadcastReceiver
        public void onReceive(Context context, Intent intent) {
            if ("android.intent.action.BATTERY_CHANGED".equals(intent.getAction())) {
                final boolean onBatteryNow = intent.getIntExtra("plugged", -1) <= 0;
                InactivityTimer.this.handler.post(new Runnable() { // from class: com.google.zxing.client.android.InactivityTimer$PowerStatusReceiver$$ExternalSyntheticLambda0
                    @Override // java.lang.Runnable
                    public final void run() {
                        this.f$0.m137lambda$onReceive$0$comgooglezxingclientandroidInactivityTimer$PowerStatusReceiver(onBatteryNow);
                    }
                });
            }
        }

        /* JADX INFO: renamed from: lambda$onReceive$0$com-google-zxing-client-android-InactivityTimer$PowerStatusReceiver, reason: not valid java name */
        /* synthetic */ void m137lambda$onReceive$0$comgooglezxingclientandroidInactivityTimer$PowerStatusReceiver(boolean onBatteryNow) {
            InactivityTimer.this.onBattery(onBatteryNow);
        }
    }
}
