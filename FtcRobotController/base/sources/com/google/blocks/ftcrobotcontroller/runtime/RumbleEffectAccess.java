package com.google.blocks.ftcrobotcontroller.runtime;

import android.webkit.JavascriptInterface;
import com.qualcomm.robotcore.hardware.Gamepad;

/* JADX INFO: loaded from: classes8.dex */
class RumbleEffectAccess extends Access {
    RumbleEffectAccess(BlocksOpMode blocksOpMode, String identifier) {
        super(blocksOpMode, identifier, "RumbleEffect");
    }

    private Gamepad.RumbleEffect.Builder checkRumbleEffectBuilder(Object rumbleEffectBuilderArg) {
        return (Gamepad.RumbleEffect.Builder) checkArg(rumbleEffectBuilderArg, Gamepad.RumbleEffect.Builder.class, "rumbleEffectBuilder");
    }

    @JavascriptInterface
    @Block(classes = {Gamepad.RumbleEffect.Builder.class}, constructor = true)
    public Gamepad.RumbleEffect.Builder createBuilder() {
        try {
            startBlockExecution(BlockType.CREATE, "");
            return new Gamepad.RumbleEffect.Builder();
        } catch (Throwable e) {
            try {
                this.blocksOpMode.handleFatalException(e);
                throw new AssertionError("impossible", e);
            } finally {
                endBlockExecution();
            }
        }
    }

    @JavascriptInterface
    @Block(classes = {Gamepad.RumbleEffect.Builder.class}, methodName = {"addStep"})
    public void addStep(Object rumbleEffectBuilderArg, double rumble1, double rumble2, int millis) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".addStep");
            Gamepad.RumbleEffect.Builder rumbleEffectBuilder = checkRumbleEffectBuilder(rumbleEffectBuilderArg);
            if (rumbleEffectBuilder != null) {
                rumbleEffectBuilder.addStep(rumble1, rumble2, millis);
            }
        } catch (Throwable e) {
            try {
                this.blocksOpMode.handleFatalException(e);
                throw new AssertionError("impossible", e);
            } finally {
                endBlockExecution();
            }
        }
    }

    @JavascriptInterface
    @Block(classes = {Gamepad.RumbleEffect.Builder.class}, methodName = {"build"})
    public Gamepad.RumbleEffect build(Object rumbleEffectBuilderArg) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".build");
            Gamepad.RumbleEffect.Builder rumbleEffectBuilder = checkRumbleEffectBuilder(rumbleEffectBuilderArg);
            if (rumbleEffectBuilder != null) {
                return rumbleEffectBuilder.build();
            }
            endBlockExecution();
            return null;
        } catch (Throwable e) {
            try {
                this.blocksOpMode.handleFatalException(e);
                throw new AssertionError("impossible", e);
            } finally {
                endBlockExecution();
            }
        }
    }
}
