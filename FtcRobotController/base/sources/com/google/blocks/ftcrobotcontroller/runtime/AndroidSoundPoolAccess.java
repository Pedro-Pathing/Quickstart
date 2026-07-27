package com.google.blocks.ftcrobotcontroller.runtime;

import android.webkit.JavascriptInterface;
import com.qualcomm.ftccommon.SoundPlayer;
import org.firstinspires.ftc.robotcore.external.android.AndroidSoundPool;

/* JADX INFO: loaded from: classes8.dex */
class AndroidSoundPoolAccess extends Access {
    private final AndroidSoundPool androidSoundPool;

    AndroidSoundPoolAccess(BlocksOpMode blocksOpMode, String identifier) {
        super(blocksOpMode, identifier, "AndroidSoundPool");
        this.androidSoundPool = new AndroidSoundPool();
    }

    @Override // com.google.blocks.ftcrobotcontroller.runtime.Access
    void close() {
        this.androidSoundPool.close();
    }

    @JavascriptInterface
    public void initialize() {
        try {
            startBlockExecution(BlockType.FUNCTION, ".initialize");
            this.androidSoundPool.initialize(SoundPlayer.getInstance());
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
    public boolean preloadSound(String soundName) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".preloadSound");
            try {
                if (!this.androidSoundPool.preloadSound(soundName)) {
                    reportWarning("Failed to preload " + soundName);
                } else {
                    endBlockExecution();
                    return true;
                }
            } catch (IllegalStateException e) {
                reportWarning(e.getMessage());
            }
            endBlockExecution();
            return false;
        } finally {
        }
    }

    @JavascriptInterface
    public void play(String soundName) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".play");
            try {
                if (!this.androidSoundPool.play(soundName)) {
                    reportWarning("Failed to load " + soundName);
                }
            } catch (IllegalStateException e) {
                reportWarning(e.getMessage());
            }
        } finally {
        }
    }

    @JavascriptInterface
    public void stop() {
        try {
            startBlockExecution(BlockType.FUNCTION, ".stop");
            this.androidSoundPool.stop();
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
    public float getVolume() {
        try {
            startBlockExecution(BlockType.GETTER, ".Volume");
            return this.androidSoundPool.getVolume();
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
    public void setVolume(float volume) {
        try {
            startBlockExecution(BlockType.SETTER, ".Volume");
            if (volume >= 0.0f && volume <= 1.0f) {
                this.androidSoundPool.setVolume(volume);
            } else {
                reportInvalidArg("", "a number between 0.0 and 1.0");
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
    public float getRate() {
        try {
            startBlockExecution(BlockType.GETTER, ".Rate");
            return this.androidSoundPool.getRate();
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
    public void setRate(float rate) {
        try {
            startBlockExecution(BlockType.SETTER, ".Rate");
            if (rate >= 0.5f && rate <= 2.0f) {
                this.androidSoundPool.setRate(rate);
            } else {
                reportInvalidArg("", "a number between 0.5 and 2.0");
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
    public int getLoop() {
        try {
            startBlockExecution(BlockType.GETTER, ".Loop");
            return this.androidSoundPool.getLoop();
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
    public void setLoop(int loop) {
        try {
            startBlockExecution(BlockType.SETTER, ".Loop");
            if (loop >= -1) {
                this.androidSoundPool.setLoop(loop);
            } else {
                reportInvalidArg("", "a number greater than or equal to -1");
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
}
