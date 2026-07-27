package com.google.blocks.ftcrobotcontroller.runtime;

import android.webkit.JavascriptInterface;
import com.qualcomm.robotcore.hardware.Gamepad;

/* JADX INFO: loaded from: classes8.dex */
class GamepadAccess extends Access {
    private final Gamepad gamepad;

    GamepadAccess(BlocksOpMode blocksOpMode, String identifier, Gamepad gamepad) {
        super(blocksOpMode, identifier, identifier);
        this.gamepad = gamepad;
    }

    @JavascriptInterface
    @Block(classes = {Gamepad.class}, fieldName = {"left_stick_x"})
    public float getLeftStickX() {
        try {
            startBlockExecution(BlockType.GETTER, ".LeftStickX");
            if (this.gamepad != null) {
                return this.gamepad.left_stick_x;
            }
            endBlockExecution();
            return 0.0f;
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
    @Block(classes = {Gamepad.class}, fieldName = {"left_stick_y"})
    public float getLeftStickY() {
        try {
            startBlockExecution(BlockType.GETTER, ".LeftStickY");
            if (this.gamepad != null) {
                return this.gamepad.left_stick_y;
            }
            endBlockExecution();
            return 0.0f;
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
    @Block(classes = {Gamepad.class}, fieldName = {"right_stick_x"})
    public float getRightStickX() {
        try {
            startBlockExecution(BlockType.GETTER, ".RightStickX");
            if (this.gamepad != null) {
                return this.gamepad.right_stick_x;
            }
            endBlockExecution();
            return 0.0f;
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
    @Block(classes = {Gamepad.class}, fieldName = {"right_stick_y"})
    public float getRightStickY() {
        try {
            startBlockExecution(BlockType.GETTER, ".RightStickY");
            if (this.gamepad != null) {
                return this.gamepad.right_stick_y;
            }
            endBlockExecution();
            return 0.0f;
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
    @Block(classes = {Gamepad.class}, fieldName = {"dpad_up"})
    public boolean getDpadUp() {
        try {
            startBlockExecution(BlockType.GETTER, ".DpadUp");
            if (this.gamepad != null) {
                return this.gamepad.dpad_up;
            }
            endBlockExecution();
            return false;
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
    @Block(classes = {Gamepad.class}, methodName = {"dpadUpWasPressed"})
    public boolean getDpadUpWasPressed() {
        try {
            startBlockExecution(BlockType.GETTER, ".DpadUpWasPressed");
            if (this.gamepad != null) {
                return this.gamepad.dpadUpWasPressed();
            }
            endBlockExecution();
            return false;
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
    @Block(classes = {Gamepad.class}, methodName = {"dpadUpWasReleased"})
    public boolean getDpadUpWasReleased() {
        try {
            startBlockExecution(BlockType.GETTER, ".DpadUpWasReleased");
            if (this.gamepad != null) {
                return this.gamepad.dpadUpWasReleased();
            }
            endBlockExecution();
            return false;
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
    @Block(classes = {Gamepad.class}, fieldName = {"dpad_down"})
    public boolean getDpadDown() {
        try {
            startBlockExecution(BlockType.GETTER, ".DpadDown");
            if (this.gamepad != null) {
                return this.gamepad.dpad_down;
            }
            endBlockExecution();
            return false;
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
    @Block(classes = {Gamepad.class}, methodName = {"dpadDownWasPressed"})
    public boolean getDpadDownWasPressed() {
        try {
            startBlockExecution(BlockType.GETTER, ".DpadDownWasPressed");
            if (this.gamepad != null) {
                return this.gamepad.dpadDownWasPressed();
            }
            endBlockExecution();
            return false;
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
    @Block(classes = {Gamepad.class}, methodName = {"dpadDownWasReleased"})
    public boolean getDpadDownWasReleased() {
        try {
            startBlockExecution(BlockType.GETTER, ".DpadDownWasReleased");
            if (this.gamepad != null) {
                return this.gamepad.dpadDownWasReleased();
            }
            endBlockExecution();
            return false;
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
    @Block(classes = {Gamepad.class}, fieldName = {"dpad_left"})
    public boolean getDpadLeft() {
        try {
            startBlockExecution(BlockType.GETTER, ".DpadLeft");
            if (this.gamepad != null) {
                return this.gamepad.dpad_left;
            }
            endBlockExecution();
            return false;
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
    @Block(classes = {Gamepad.class}, methodName = {"dpadLeftWasPressed"})
    public boolean getDpadLeftWasPressed() {
        try {
            startBlockExecution(BlockType.GETTER, ".DpadLeftWasPressed");
            if (this.gamepad != null) {
                return this.gamepad.dpadLeftWasPressed();
            }
            endBlockExecution();
            return false;
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
    @Block(classes = {Gamepad.class}, methodName = {"dpadLeftWasReleased"})
    public boolean getDpadLeftWasReleased() {
        try {
            startBlockExecution(BlockType.GETTER, ".DpadLeftWasReleased");
            if (this.gamepad != null) {
                return this.gamepad.dpadLeftWasReleased();
            }
            endBlockExecution();
            return false;
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
    @Block(classes = {Gamepad.class}, fieldName = {"dpad_right"})
    public boolean getDpadRight() {
        try {
            startBlockExecution(BlockType.GETTER, ".DpadRight");
            if (this.gamepad != null) {
                return this.gamepad.dpad_right;
            }
            endBlockExecution();
            return false;
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
    @Block(classes = {Gamepad.class}, methodName = {"dpadRightWasPressed"})
    public boolean getDpadRightWasPressed() {
        try {
            startBlockExecution(BlockType.GETTER, ".DpadRightWasPressed");
            if (this.gamepad != null) {
                return this.gamepad.dpadRightWasPressed();
            }
            endBlockExecution();
            return false;
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
    @Block(classes = {Gamepad.class}, methodName = {"dpadRightWasReleased"})
    public boolean getDpadRightWasReleased() {
        try {
            startBlockExecution(BlockType.GETTER, ".DpadRightWasReleased");
            if (this.gamepad != null) {
                return this.gamepad.dpadRightWasReleased();
            }
            endBlockExecution();
            return false;
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
    @Block(classes = {Gamepad.class}, fieldName = {"a"})
    public boolean getA() {
        try {
            startBlockExecution(BlockType.GETTER, ".A");
            if (this.gamepad != null) {
                return this.gamepad.a;
            }
            endBlockExecution();
            return false;
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
    @Block(classes = {Gamepad.class}, methodName = {"aWasPressed"})
    public boolean getAWasPressed() {
        try {
            startBlockExecution(BlockType.GETTER, ".AWasPressed");
            if (this.gamepad != null) {
                return this.gamepad.aWasPressed();
            }
            endBlockExecution();
            return false;
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
    @Block(classes = {Gamepad.class}, methodName = {"aWasReleased"})
    public boolean getAWasReleased() {
        try {
            startBlockExecution(BlockType.GETTER, ".AWasReleased");
            if (this.gamepad != null) {
                return this.gamepad.aWasReleased();
            }
            endBlockExecution();
            return false;
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
    @Block(classes = {Gamepad.class}, fieldName = {"b"})
    public boolean getB() {
        try {
            startBlockExecution(BlockType.GETTER, ".B");
            if (this.gamepad != null) {
                return this.gamepad.b;
            }
            endBlockExecution();
            return false;
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
    @Block(classes = {Gamepad.class}, methodName = {"bWasPressed"})
    public boolean getBWasPressed() {
        try {
            startBlockExecution(BlockType.GETTER, ".BWasPressed");
            if (this.gamepad != null) {
                return this.gamepad.bWasPressed();
            }
            endBlockExecution();
            return false;
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
    @Block(classes = {Gamepad.class}, methodName = {"bWasReleased"})
    public boolean getBWasReleased() {
        try {
            startBlockExecution(BlockType.GETTER, ".BWasReleased");
            if (this.gamepad != null) {
                return this.gamepad.bWasReleased();
            }
            endBlockExecution();
            return false;
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
    @Block(classes = {Gamepad.class}, fieldName = {"x"})
    public boolean getX() {
        try {
            startBlockExecution(BlockType.GETTER, ".X");
            if (this.gamepad != null) {
                return this.gamepad.x;
            }
            endBlockExecution();
            return false;
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
    @Block(classes = {Gamepad.class}, methodName = {"xWasPressed"})
    public boolean getXWasPressed() {
        try {
            startBlockExecution(BlockType.GETTER, ".XWasPressed");
            if (this.gamepad != null) {
                return this.gamepad.xWasPressed();
            }
            endBlockExecution();
            return false;
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
    @Block(classes = {Gamepad.class}, methodName = {"xWasReleased"})
    public boolean getXWasReleased() {
        try {
            startBlockExecution(BlockType.GETTER, ".XWasReleased");
            if (this.gamepad != null) {
                return this.gamepad.xWasReleased();
            }
            endBlockExecution();
            return false;
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
    @Block(classes = {Gamepad.class}, fieldName = {"y"})
    public boolean getY() {
        try {
            startBlockExecution(BlockType.GETTER, ".Y");
            if (this.gamepad != null) {
                return this.gamepad.y;
            }
            endBlockExecution();
            return false;
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
    @Block(classes = {Gamepad.class}, methodName = {"yWasPressed"})
    public boolean getYWasPressed() {
        try {
            startBlockExecution(BlockType.GETTER, ".YWasPressed");
            if (this.gamepad != null) {
                return this.gamepad.yWasPressed();
            }
            endBlockExecution();
            return false;
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
    @Block(classes = {Gamepad.class}, methodName = {"yWasReleased"})
    public boolean getYWasReleased() {
        try {
            startBlockExecution(BlockType.GETTER, ".YWasReleased");
            if (this.gamepad != null) {
                return this.gamepad.yWasReleased();
            }
            endBlockExecution();
            return false;
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
    @Block(classes = {Gamepad.class}, fieldName = {"guide"})
    public boolean getGuide() {
        try {
            startBlockExecution(BlockType.GETTER, ".Guide");
            if (this.gamepad != null) {
                return this.gamepad.guide;
            }
            endBlockExecution();
            return false;
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
    @Block(classes = {Gamepad.class}, methodName = {"guideWasPressed"})
    public boolean getGuideWasPressed() {
        try {
            startBlockExecution(BlockType.GETTER, ".GuideWasPressed");
            if (this.gamepad != null) {
                return this.gamepad.guideWasPressed();
            }
            endBlockExecution();
            return false;
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
    @Block(classes = {Gamepad.class}, methodName = {"guideWasReleased"})
    public boolean getGuideWasReleased() {
        try {
            startBlockExecution(BlockType.GETTER, ".GuideWasReleased");
            if (this.gamepad != null) {
                return this.gamepad.guideWasReleased();
            }
            endBlockExecution();
            return false;
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
    @Block(classes = {Gamepad.class}, fieldName = {"start"})
    public boolean getStart() {
        try {
            startBlockExecution(BlockType.GETTER, ".Start");
            if (this.gamepad != null) {
                return this.gamepad.start;
            }
            endBlockExecution();
            return false;
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
    @Block(classes = {Gamepad.class}, methodName = {"startWasPressed"})
    public boolean getStartWasPressed() {
        try {
            startBlockExecution(BlockType.GETTER, ".StartWasPressed");
            if (this.gamepad != null) {
                return this.gamepad.startWasPressed();
            }
            endBlockExecution();
            return false;
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
    @Block(classes = {Gamepad.class}, methodName = {"startWasReleased"})
    public boolean getStartWasReleased() {
        try {
            startBlockExecution(BlockType.GETTER, ".StartWasReleased");
            if (this.gamepad != null) {
                return this.gamepad.startWasReleased();
            }
            endBlockExecution();
            return false;
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
    @Block(classes = {Gamepad.class}, fieldName = {"back"})
    public boolean getBack() {
        try {
            startBlockExecution(BlockType.GETTER, ".Back");
            if (this.gamepad != null) {
                return this.gamepad.back;
            }
            endBlockExecution();
            return false;
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
    @Block(classes = {Gamepad.class}, methodName = {"backWasPressed"})
    public boolean getBackWasPressed() {
        try {
            startBlockExecution(BlockType.GETTER, ".BackWasPressed");
            if (this.gamepad != null) {
                return this.gamepad.backWasPressed();
            }
            endBlockExecution();
            return false;
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
    @Block(classes = {Gamepad.class}, methodName = {"backWasReleased"})
    public boolean getBackWasReleased() {
        try {
            startBlockExecution(BlockType.GETTER, ".BackWasReleased");
            if (this.gamepad != null) {
                return this.gamepad.backWasReleased();
            }
            endBlockExecution();
            return false;
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
    @Block(classes = {Gamepad.class}, fieldName = {"left_bumper"})
    public boolean getLeftBumper() {
        try {
            startBlockExecution(BlockType.GETTER, ".LeftBumper");
            if (this.gamepad != null) {
                return this.gamepad.left_bumper;
            }
            endBlockExecution();
            return false;
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
    @Block(classes = {Gamepad.class}, methodName = {"leftBumperWasPressed"})
    public boolean getLeftBumperWasPressed() {
        try {
            startBlockExecution(BlockType.GETTER, ".LeftBumperWasPressed");
            if (this.gamepad != null) {
                return this.gamepad.leftBumperWasPressed();
            }
            endBlockExecution();
            return false;
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
    @Block(classes = {Gamepad.class}, methodName = {"leftBumperWasReleased"})
    public boolean getLeftBumperWasReleased() {
        try {
            startBlockExecution(BlockType.GETTER, ".LeftBumperWasReleased");
            if (this.gamepad != null) {
                return this.gamepad.leftBumperWasReleased();
            }
            endBlockExecution();
            return false;
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
    @Block(classes = {Gamepad.class}, fieldName = {"right_bumper"})
    public boolean getRightBumper() {
        try {
            startBlockExecution(BlockType.GETTER, ".RightBumper");
            if (this.gamepad != null) {
                return this.gamepad.right_bumper;
            }
            endBlockExecution();
            return false;
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
    @Block(classes = {Gamepad.class}, methodName = {"rightBumperWasPressed"})
    public boolean getRightBumperWasPressed() {
        try {
            startBlockExecution(BlockType.GETTER, ".RightBumperWasPressed");
            if (this.gamepad != null) {
                return this.gamepad.rightBumperWasPressed();
            }
            endBlockExecution();
            return false;
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
    @Block(classes = {Gamepad.class}, methodName = {"rightBumperWasReleased"})
    public boolean getRightBumperWasReleased() {
        try {
            startBlockExecution(BlockType.GETTER, ".RightBumperWasReleased");
            if (this.gamepad != null) {
                return this.gamepad.rightBumperWasReleased();
            }
            endBlockExecution();
            return false;
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
    @Block(classes = {Gamepad.class}, fieldName = {"left_stick_button"})
    public boolean getLeftStickButton() {
        try {
            startBlockExecution(BlockType.GETTER, ".LeftStickButton");
            if (this.gamepad != null) {
                return this.gamepad.left_stick_button;
            }
            endBlockExecution();
            return false;
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
    @Block(classes = {Gamepad.class}, methodName = {"leftStickButtonWasPressed"})
    public boolean getLeftStickButtonWasPressed() {
        try {
            startBlockExecution(BlockType.GETTER, ".LeftStickButtonWasPressed");
            if (this.gamepad != null) {
                return this.gamepad.leftStickButtonWasPressed();
            }
            endBlockExecution();
            return false;
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
    @Block(classes = {Gamepad.class}, methodName = {"leftStickButtonWasReleased"})
    public boolean getLeftStickButtonWasReleased() {
        try {
            startBlockExecution(BlockType.GETTER, ".LeftStickButtonWasReleased");
            if (this.gamepad != null) {
                return this.gamepad.leftStickButtonWasReleased();
            }
            endBlockExecution();
            return false;
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
    @Block(classes = {Gamepad.class}, fieldName = {"right_stick_button"})
    public boolean getRightStickButton() {
        try {
            startBlockExecution(BlockType.GETTER, ".RightStickButton");
            if (this.gamepad != null) {
                return this.gamepad.right_stick_button;
            }
            endBlockExecution();
            return false;
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
    @Block(classes = {Gamepad.class}, methodName = {"rightStickButtonWasPressed"})
    public boolean getRightStickButtonWasPressed() {
        try {
            startBlockExecution(BlockType.GETTER, ".RightStickButtonWasPressed");
            if (this.gamepad != null) {
                return this.gamepad.rightStickButtonWasPressed();
            }
            endBlockExecution();
            return false;
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
    @Block(classes = {Gamepad.class}, methodName = {"rightStickButtonWasReleased"})
    public boolean getRightStickButtonWasReleased() {
        try {
            startBlockExecution(BlockType.GETTER, ".RightStickButtonWasReleased");
            if (this.gamepad != null) {
                return this.gamepad.rightStickButtonWasReleased();
            }
            endBlockExecution();
            return false;
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
    @Block(classes = {Gamepad.class}, fieldName = {"left_trigger"})
    public float getLeftTrigger() {
        try {
            startBlockExecution(BlockType.GETTER, ".LeftTrigger");
            if (this.gamepad != null) {
                return this.gamepad.left_trigger;
            }
            endBlockExecution();
            return 0.0f;
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
    @Block(classes = {Gamepad.class}, fieldName = {"right_trigger"})
    public float getRightTrigger() {
        try {
            startBlockExecution(BlockType.GETTER, ".RightTrigger");
            if (this.gamepad != null) {
                return this.gamepad.right_trigger;
            }
            endBlockExecution();
            return 0.0f;
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
    @Block(classes = {Gamepad.class}, methodName = {"atRest"})
    public boolean getAtRest() {
        try {
            startBlockExecution(BlockType.GETTER, ".AtRest");
            if (this.gamepad != null) {
                return this.gamepad.atRest();
            }
            endBlockExecution();
            return false;
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
    @Block(classes = {Gamepad.class}, fieldName = {"circle"})
    public boolean getCircle() {
        try {
            startBlockExecution(BlockType.GETTER, ".Circle");
            if (this.gamepad != null) {
                return this.gamepad.circle;
            }
            endBlockExecution();
            return false;
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
    @Block(classes = {Gamepad.class}, methodName = {"circleWasPressed"})
    public boolean getCircleWasPressed() {
        try {
            startBlockExecution(BlockType.GETTER, ".CircleWasPressed");
            if (this.gamepad != null) {
                return this.gamepad.circleWasPressed();
            }
            endBlockExecution();
            return false;
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
    @Block(classes = {Gamepad.class}, methodName = {"circleWasReleased"})
    public boolean getCircleWasReleased() {
        try {
            startBlockExecution(BlockType.GETTER, ".CircleWasReleased");
            if (this.gamepad != null) {
                return this.gamepad.circleWasReleased();
            }
            endBlockExecution();
            return false;
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
    @Block(classes = {Gamepad.class}, fieldName = {"cross"})
    public boolean getCross() {
        try {
            startBlockExecution(BlockType.GETTER, ".Cross");
            if (this.gamepad != null) {
                return this.gamepad.cross;
            }
            endBlockExecution();
            return false;
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
    @Block(classes = {Gamepad.class}, methodName = {"crossWasPressed"})
    public boolean getCrossWasPressed() {
        try {
            startBlockExecution(BlockType.GETTER, ".CrossWasPressed");
            if (this.gamepad != null) {
                return this.gamepad.crossWasPressed();
            }
            endBlockExecution();
            return false;
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
    @Block(classes = {Gamepad.class}, methodName = {"crossWasReleased"})
    public boolean getCrossWasReleased() {
        try {
            startBlockExecution(BlockType.GETTER, ".CrossWasReleased");
            if (this.gamepad != null) {
                return this.gamepad.crossWasReleased();
            }
            endBlockExecution();
            return false;
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
    @Block(classes = {Gamepad.class}, fieldName = {"options"})
    public boolean getOptions() {
        try {
            startBlockExecution(BlockType.GETTER, ".Options");
            if (this.gamepad != null) {
                return this.gamepad.options;
            }
            endBlockExecution();
            return false;
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
    @Block(classes = {Gamepad.class}, methodName = {"optionsWasPressed"})
    public boolean getOptionsWasPressed() {
        try {
            startBlockExecution(BlockType.GETTER, ".OptionsWasPressed");
            if (this.gamepad != null) {
                return this.gamepad.optionsWasPressed();
            }
            endBlockExecution();
            return false;
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
    @Block(classes = {Gamepad.class}, methodName = {"optionsWasReleased"})
    public boolean getOptionsWasReleased() {
        try {
            startBlockExecution(BlockType.GETTER, ".OptionsWasReleased");
            if (this.gamepad != null) {
                return this.gamepad.optionsWasReleased();
            }
            endBlockExecution();
            return false;
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
    @Block(classes = {Gamepad.class}, fieldName = {"ps"})
    public boolean getPS() {
        try {
            startBlockExecution(BlockType.GETTER, ".PS");
            if (this.gamepad != null) {
                return this.gamepad.ps;
            }
            endBlockExecution();
            return false;
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
    @Block(classes = {Gamepad.class}, methodName = {"psWasPressed"})
    public boolean getPSWasPressed() {
        try {
            startBlockExecution(BlockType.GETTER, ".PSWasPressed");
            if (this.gamepad != null) {
                return this.gamepad.psWasPressed();
            }
            endBlockExecution();
            return false;
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
    @Block(classes = {Gamepad.class}, methodName = {"psWasReleased"})
    public boolean getPSWasReleased() {
        try {
            startBlockExecution(BlockType.GETTER, ".PSWasReleased");
            if (this.gamepad != null) {
                return this.gamepad.psWasReleased();
            }
            endBlockExecution();
            return false;
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
    @Block(classes = {Gamepad.class}, fieldName = {"share"})
    public boolean getShare() {
        try {
            startBlockExecution(BlockType.GETTER, ".Share");
            if (this.gamepad != null) {
                return this.gamepad.share;
            }
            endBlockExecution();
            return false;
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
    @Block(classes = {Gamepad.class}, methodName = {"shareWasPressed"})
    public boolean getShareWasPressed() {
        try {
            startBlockExecution(BlockType.GETTER, ".ShareWasPressed");
            if (this.gamepad != null) {
                return this.gamepad.shareWasPressed();
            }
            endBlockExecution();
            return false;
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
    @Block(classes = {Gamepad.class}, methodName = {"shareWasReleased"})
    public boolean getShareWasReleased() {
        try {
            startBlockExecution(BlockType.GETTER, ".ShareWasReleased");
            if (this.gamepad != null) {
                return this.gamepad.shareWasReleased();
            }
            endBlockExecution();
            return false;
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
    @Block(classes = {Gamepad.class}, fieldName = {"square"})
    public boolean getSquare() {
        try {
            startBlockExecution(BlockType.GETTER, ".Square");
            if (this.gamepad != null) {
                return this.gamepad.square;
            }
            endBlockExecution();
            return false;
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
    @Block(classes = {Gamepad.class}, methodName = {"squareWasPressed"})
    public boolean getSquareWasPressed() {
        try {
            startBlockExecution(BlockType.GETTER, ".SquareWasPressed");
            if (this.gamepad != null) {
                return this.gamepad.squareWasPressed();
            }
            endBlockExecution();
            return false;
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
    @Block(classes = {Gamepad.class}, methodName = {"squareWasReleased"})
    public boolean getSquareWasReleased() {
        try {
            startBlockExecution(BlockType.GETTER, ".SquareWasReleased");
            if (this.gamepad != null) {
                return this.gamepad.squareWasReleased();
            }
            endBlockExecution();
            return false;
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
    @Block(classes = {Gamepad.class}, fieldName = {"touchpad"})
    public boolean getTouchpad() {
        try {
            startBlockExecution(BlockType.GETTER, ".Touchpad");
            if (this.gamepad != null) {
                return this.gamepad.touchpad;
            }
            endBlockExecution();
            return false;
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
    @Block(classes = {Gamepad.class}, methodName = {"touchpadWasPressed"})
    public boolean getTouchpadWasPressed() {
        try {
            startBlockExecution(BlockType.GETTER, ".TouchpadWasPressed");
            if (this.gamepad != null) {
                return this.gamepad.touchpadWasPressed();
            }
            endBlockExecution();
            return false;
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
    @Block(classes = {Gamepad.class}, methodName = {"touchpadWasReleased"})
    public boolean getTouchpadWasReleased() {
        try {
            startBlockExecution(BlockType.GETTER, ".TouchpadWasReleased");
            if (this.gamepad != null) {
                return this.gamepad.touchpadWasReleased();
            }
            endBlockExecution();
            return false;
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
    @Block(classes = {Gamepad.class}, fieldName = {"touchpad_finger_1"})
    public boolean getTouchpadFinger1() {
        try {
            startBlockExecution(BlockType.GETTER, ".TouchpadFinger1");
            if (this.gamepad != null) {
                return this.gamepad.touchpad_finger_1;
            }
            endBlockExecution();
            return false;
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
    @Block(classes = {Gamepad.class}, fieldName = {"touchpad_finger_1_x"})
    public float getTouchpadFinger1X() {
        try {
            startBlockExecution(BlockType.GETTER, ".TouchpadFinger1X");
            if (this.gamepad != null) {
                return this.gamepad.touchpad_finger_1_x;
            }
            endBlockExecution();
            return 0.0f;
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
    @Block(classes = {Gamepad.class}, fieldName = {"touchpad_finger_1_y"})
    public float getTouchpadFinger1Y() {
        try {
            startBlockExecution(BlockType.GETTER, ".TouchpadFinger1Y");
            if (this.gamepad != null) {
                return this.gamepad.touchpad_finger_1_y;
            }
            endBlockExecution();
            return 0.0f;
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
    @Block(classes = {Gamepad.class}, fieldName = {"touchpad_finger_2"})
    public boolean getTouchpadFinger2() {
        try {
            startBlockExecution(BlockType.GETTER, ".TouchpadFinger2");
            if (this.gamepad != null) {
                return this.gamepad.touchpad_finger_2;
            }
            endBlockExecution();
            return false;
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
    @Block(classes = {Gamepad.class}, fieldName = {"touchpad_finger_2_x"})
    public float getTouchpadFinger2X() {
        try {
            startBlockExecution(BlockType.GETTER, ".TouchpadFinger2X");
            if (this.gamepad != null) {
                return this.gamepad.touchpad_finger_2_x;
            }
            endBlockExecution();
            return 0.0f;
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
    @Block(classes = {Gamepad.class}, fieldName = {"touchpad_finger_2_y"})
    public float getTouchpadFinger2Y() {
        try {
            startBlockExecution(BlockType.GETTER, ".TouchpadFinger2Y");
            if (this.gamepad != null) {
                return this.gamepad.touchpad_finger_2_y;
            }
            endBlockExecution();
            return 0.0f;
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
    @Block(classes = {Gamepad.class}, fieldName = {"triangle"})
    public boolean getTriangle() {
        try {
            startBlockExecution(BlockType.GETTER, ".Triangle");
            if (this.gamepad != null) {
                return this.gamepad.triangle;
            }
            endBlockExecution();
            return false;
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
    @Block(classes = {Gamepad.class}, methodName = {"triangleWasPressed"})
    public boolean getTriangleWasPressed() {
        try {
            startBlockExecution(BlockType.GETTER, ".TriangleWasPressed");
            if (this.gamepad != null) {
                return this.gamepad.triangleWasPressed();
            }
            endBlockExecution();
            return false;
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
    @Block(classes = {Gamepad.class}, methodName = {"triangleWasReleased"})
    public boolean getTriangleWasReleased() {
        try {
            startBlockExecution(BlockType.GETTER, ".TriangleWasReleased");
            if (this.gamepad != null) {
                return this.gamepad.triangleWasReleased();
            }
            endBlockExecution();
            return false;
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
    @Block(classes = {Gamepad.class}, methodName = {"resetEdgeDetection"})
    public void resetEdgeDetection() {
        try {
            startBlockExecution(BlockType.FUNCTION, ".resetEdgeDetection");
            if (this.gamepad != null) {
                this.gamepad.resetEdgeDetection();
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
    @Block(classes = {Gamepad.class}, methodName = {"rumble"})
    public void rumble_with1(int millis) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".rumble");
            if (this.gamepad != null) {
                this.gamepad.rumble(millis);
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
    @Block(classes = {Gamepad.class}, methodName = {"rumble"})
    public void rumble_with3(double rumble1, double rumble2, int millis) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".rumble");
            if (this.gamepad != null) {
                this.gamepad.rumble(rumble1, rumble2, millis);
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
    @Block(classes = {Gamepad.class}, methodName = {"stopRumble"})
    public void stopRumble() {
        try {
            startBlockExecution(BlockType.FUNCTION, ".stopRumble");
            if (this.gamepad != null) {
                this.gamepad.stopRumble();
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
    @Block(classes = {Gamepad.class}, methodName = {"rumbleBlips"})
    public void rumbleBlips(int count) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".rumbleBlips");
            if (this.gamepad != null) {
                this.gamepad.rumbleBlips(count);
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
    @Block(classes = {Gamepad.class}, methodName = {"isRumbling"})
    public boolean isRumbling() {
        try {
            startBlockExecution(BlockType.FUNCTION, ".isRumbling");
            if (this.gamepad != null) {
                return this.gamepad.isRumbling();
            }
            endBlockExecution();
            return false;
        } catch (Throwable e) {
            try {
                this.blocksOpMode.handleFatalException(e);
                throw new AssertionError("impossible", e);
            } finally {
                endBlockExecution();
            }
        }
    }

    private Gamepad.RumbleEffect checkRumbleEffect(Object rumbleEffectArg) {
        return (Gamepad.RumbleEffect) checkArg(rumbleEffectArg, Gamepad.RumbleEffect.class, "rumbleEffect");
    }

    @JavascriptInterface
    @Block(classes = {Gamepad.class}, methodName = {"runRumbleEffect"})
    public void runRumbleEffect(Object rumbleEffectArg) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".runRumbleEffect");
            Gamepad.RumbleEffect rumbleEffect = checkRumbleEffect(rumbleEffectArg);
            if (this.gamepad != null && rumbleEffect != null) {
                this.gamepad.runRumbleEffect(rumbleEffect);
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
    @Block(classes = {Gamepad.class}, methodName = {"setLedColor"})
    public void setLedColor(double r, double g, double b, int millis) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".setLedColor");
            if (this.gamepad != null) {
                this.gamepad.setLedColor(r, g, b, millis);
            }
        } finally {
        }
    }

    private Gamepad.LedEffect checkLedEffect(Object ledEffectArg) {
        return (Gamepad.LedEffect) checkArg(ledEffectArg, Gamepad.LedEffect.class, "ledEffect");
    }

    @JavascriptInterface
    @Block(classes = {Gamepad.class}, methodName = {"runLedEffect"})
    public void runLedEffect(Object ledEffectArg) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".runLedEffect");
            Gamepad.LedEffect ledEffect = checkLedEffect(ledEffectArg);
            if (this.gamepad != null && ledEffect != null) {
                this.gamepad.runLedEffect(ledEffect);
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
