package com.google.blocks.ftcrobotcontroller.runtime;

import android.app.Activity;
import android.graphics.Color;
import android.webkit.JavascriptInterface;
import org.firstinspires.ftc.robotcore.external.JavaUtil;

/* JADX INFO: loaded from: classes8.dex */
class ColorAccess extends Access {
    private final Activity activity;

    ColorAccess(BlocksOpMode blocksOpMode, String identifier, Activity activity) {
        super(blocksOpMode, identifier, "Color");
        this.activity = activity;
    }

    @JavascriptInterface
    public double getRed(int color) {
        try {
            startBlockExecution(BlockType.GETTER, ".Red");
            return Color.red(color);
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
    public double getGreen(int color) {
        try {
            startBlockExecution(BlockType.GETTER, ".Green");
            return Color.green(color);
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
    public double getBlue(int color) {
        try {
            startBlockExecution(BlockType.GETTER, ".Blue");
            return Color.blue(color);
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
    public double getAlpha(int color) {
        try {
            startBlockExecution(BlockType.GETTER, ".Alpha");
            return Color.alpha(color);
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
    public float getHue(int color) {
        try {
            startBlockExecution(BlockType.GETTER, ".Hue");
            return JavaUtil.colorToHue(color);
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
    public float getSaturation(int color) {
        try {
            startBlockExecution(BlockType.GETTER, ".Saturation");
            return JavaUtil.colorToSaturation(color);
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
    public float getValue(int color) {
        try {
            startBlockExecution(BlockType.GETTER, ".Value");
            return JavaUtil.colorToValue(color);
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
    public int rgbToColor(int red, int green, int blue) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".rgbToColor");
            return Color.rgb(red, green, blue);
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
    public int argbToColor(int alpha, int red, int green, int blue) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".argbToColor");
            return Color.argb(alpha, red, green, blue);
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
    public int hsvToColor(float hue, float saturation, float value) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".hsvToColor");
            float[] array = {hue, saturation, value};
            return Color.HSVToColor(array);
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
    public int ahsvToColor(int alpha, float hue, float saturation, float value) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".ahsvToColor");
            float[] array = {hue, saturation, value};
            return Color.HSVToColor(alpha, array);
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
    public int textToColor(String text) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".textToColor");
            return Color.parseColor(text);
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
    public float rgbToHue(int red, int green, int blue) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".rgbToHue");
            return JavaUtil.rgbToHue(red, green, blue);
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
    public float rgbToSaturation(int red, int green, int blue) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".rgbToSaturation");
            return JavaUtil.rgbToSaturation(red, green, blue);
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
    public float rgbToValue(int red, int green, int blue) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".rgbToValue");
            return JavaUtil.rgbToValue(red, green, blue);
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
    public String toText(int color) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".toText");
            return JavaUtil.colorToText(color);
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
    public void showColor(int color) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".showColor");
            JavaUtil.showColor(this.activity, color);
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
