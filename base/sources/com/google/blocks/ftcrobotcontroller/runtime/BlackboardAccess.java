package com.google.blocks.ftcrobotcontroller.runtime;

import android.webkit.JavascriptInterface;
import com.qualcomm.hardware.lynx.LynxServoController;
import com.qualcomm.robotcore.util.RobotLog;
import java.util.HashMap;
import java.util.Map;

/* JADX INFO: loaded from: classes8.dex */
class BlackboardAccess extends Access {
    private final Map<String, Object> blackboard;

    BlackboardAccess(BlocksOpMode blocksOpMode, String identifier, Map<String, Object> blackboard) {
        super(blocksOpMode, identifier, "Blackboard");
        this.blackboard = blackboard;
    }

    @JavascriptInterface
    @Block(classes = {HashMap.class, Map.class}, methodName = {"clear"})
    public void clear() {
        try {
            startBlockExecution(BlockType.FUNCTION, ".clear");
            this.blackboard.clear();
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
    @Block(classes = {HashMap.class, Map.class}, methodName = {"containsKey"})
    public boolean containsKey(String key) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".containsKey");
            return this.blackboard.containsKey(key);
        } catch (Throwable e) {
            try {
                e.printStackTrace();
                this.blocksOpMode.handleFatalException(e);
                throw new AssertionError("impossible", e);
            } finally {
                endBlockExecution();
            }
        }
    }

    @JavascriptInterface
    @Block(exclusiveToBlocks = true)
    public String getType(String label, String key) {
        try {
            startBlockExecution(BlockType.FUNCTION, "." + label);
            Object value = this.blackboard.get(key);
            return value == null ? "null" : value instanceof Boolean ? "boolean" : value instanceof Number ? "number" : value instanceof String ? "string" : "object";
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
    @Block(classes = {HashMap.class, Map.class}, methodName = {"get"})
    public Object get(String label, String key) {
        try {
            startBlockExecution(BlockType.FUNCTION, "." + label);
            return this.blackboard.get(key);
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
    @Block(classes = {HashMap.class, Map.class}, methodName = {"get"})
    public boolean getBoolean(String label, String key) {
        try {
            startBlockExecution(BlockType.FUNCTION, "." + label);
            Object value = this.blackboard.get(key);
            if (value instanceof Boolean) {
                return ((Boolean) value).booleanValue();
            }
            RobotLog.ww("BlackboardAccess", "Expected blackboard.get(" + key + ") to return a Boolean");
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
    @Block(classes = {HashMap.class, Map.class}, methodName = {"get"})
    public double getNumber(String label, String key) {
        try {
            startBlockExecution(BlockType.FUNCTION, "." + label);
            Object value = this.blackboard.get(key);
            if (value instanceof Number) {
                return ((Number) value).doubleValue();
            }
            RobotLog.ww("BlackboardAccess", "Expected blackboard.get(" + key + ") to return a Number");
            endBlockExecution();
            return LynxServoController.apiPositionFirst;
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
    @Block(classes = {HashMap.class, Map.class}, methodName = {"get"})
    public String getString(String label, String key) {
        try {
            startBlockExecution(BlockType.FUNCTION, "." + label);
            Object value = this.blackboard.get(key);
            if (value instanceof String) {
                return (String) value;
            }
            RobotLog.ww("BlackboardAccess", "Expected blackboard.get(" + key + ") to return a String");
            return "";
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
    @Block(classes = {HashMap.class, Map.class}, methodName = {"isEmpty"})
    public boolean isEmpty() {
        try {
            startBlockExecution(BlockType.FUNCTION, ".isEmpty");
            return this.blackboard.isEmpty();
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
    @Block(classes = {HashMap.class, Map.class}, methodName = {"put"})
    public void put(String key, Object value) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".put");
            this.blackboard.put(key, value);
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
    @Block(classes = {HashMap.class, Map.class}, methodName = {"put"})
    public void putBoolean(String key, boolean value) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".put");
            this.blackboard.put(key, Boolean.valueOf(value));
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
    @Block(classes = {HashMap.class, Map.class}, methodName = {"put"})
    public void putNumber(String key, double value) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".put");
            this.blackboard.put(key, Double.valueOf(value));
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
    @Block(classes = {HashMap.class, Map.class}, methodName = {"put"})
    public void putString(String key, String value) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".put");
            this.blackboard.put(key, value);
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
    @Block(classes = {HashMap.class, Map.class}, methodName = {"remove"})
    public Object remove(String key) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".remove");
            return this.blackboard.remove(key);
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
    @Block(classes = {HashMap.class, Map.class}, methodName = {"size"})
    public int size() {
        try {
            startBlockExecution(BlockType.FUNCTION, ".size");
            return this.blackboard.size();
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
