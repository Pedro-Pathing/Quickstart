package com.google.blocks.ftcrobotcontroller.runtime;

import android.webkit.JavascriptInterface;
import com.qualcomm.robotcore.hardware.Gamepad;

/* JADX INFO: loaded from: classes8.dex */
class LedEffectAccess extends Access {
    LedEffectAccess(BlocksOpMode blocksOpMode, String identifier) {
        super(blocksOpMode, identifier, "LedEffect");
    }

    private Gamepad.LedEffect.Builder checkLedEffectBuilder(Object ledEffectBuilderArg) {
        return (Gamepad.LedEffect.Builder) checkArg(ledEffectBuilderArg, Gamepad.LedEffect.Builder.class, "ledEffectBuilder");
    }

    @JavascriptInterface
    @Block(classes = {Gamepad.LedEffect.Builder.class}, constructor = true)
    public Gamepad.LedEffect.Builder createBuilder() {
        try {
            startBlockExecution(BlockType.CREATE, "");
            return new Gamepad.LedEffect.Builder();
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
    @Block(classes = {Gamepad.LedEffect.Builder.class}, methodName = {"addStep"})
    public void addStep(Object ledEffectBuilderArg, double red, double green, double blue, int millis) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".addStep");
            Gamepad.LedEffect.Builder ledEffectBuilder = checkLedEffectBuilder(ledEffectBuilderArg);
            if (ledEffectBuilder != null) {
                ledEffectBuilder.addStep(red, green, blue, millis);
            }
        } finally {
        }
    }

    @JavascriptInterface
    @Block(classes = {Gamepad.LedEffect.Builder.class}, methodName = {"setRepeating"})
    public void setRepeating(Object ledEffectBuilderArg, boolean repeating) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".setRepeating");
            Gamepad.LedEffect.Builder ledEffectBuilder = checkLedEffectBuilder(ledEffectBuilderArg);
            if (ledEffectBuilder != null) {
                ledEffectBuilder.setRepeating(repeating);
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
    @Block(classes = {Gamepad.LedEffect.Builder.class}, methodName = {"build"})
    public Gamepad.LedEffect build(Object ledEffectBuilderArg) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".build");
            Gamepad.LedEffect.Builder ledEffectBuilder = checkLedEffectBuilder(ledEffectBuilderArg);
            if (ledEffectBuilder != null) {
                return ledEffectBuilder.build();
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
