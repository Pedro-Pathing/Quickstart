package com.google.blocks.ftcrobotcontroller.runtime;

import android.webkit.JavascriptInterface;
import com.google.blocks.ftcrobotcontroller.hardware.HardwareUtil;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.collections.SimpleGson;
import org.firstinspires.ftc.robotcore.internal.opmode.BlocksClassFilter;

/* JADX INFO: loaded from: classes8.dex */
class MiscAccess extends Access {
    private HardwareMap hardwareMap;

    MiscAccess(BlocksOpMode blocksOpMode, String identifier, HardwareMap hardwareMap) {
        super(blocksOpMode, identifier, "");
        this.hardwareMap = hardwareMap;
    }

    @JavascriptInterface
    @Block(exclusiveToBlocks = true)
    public Object getNull() {
        try {
            startBlockExecution(BlockType.SPECIAL, "null");
            endBlockExecution();
            return null;
        } catch (Throwable e) {
            try {
                this.blocksOpMode.handleFatalException(e);
                throw new AssertionError("impossible", e);
            } catch (Throwable e2) {
                endBlockExecution();
                throw e2;
            }
        }
    }

    @JavascriptInterface
    @Block(exclusiveToBlocks = true)
    public boolean isNull(Object value) {
        try {
            startBlockExecution(BlockType.FUNCTION, "isNull");
            return value == null;
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
    @Block(exclusiveToBlocks = true)
    public boolean isNotNull(Object value) {
        try {
            startBlockExecution(BlockType.FUNCTION, "isNotNull");
            return value != null;
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
    @Block(exclusiveToBlocks = true)
    public String formatNumber(double number, int precision) {
        try {
            startBlockExecution(BlockType.FUNCTION, "formatNumber");
            return JavaUtil.formatNumber(number, precision);
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
    @Block(exclusiveToBlocks = true)
    public String formatNumber_withWidth(double number, int width, int precision) {
        try {
            startBlockExecution(BlockType.FUNCTION, "formatNumber");
            return JavaUtil.formatNumber(number, width, precision);
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
    @Block(exclusiveToBlocks = true)
    public double roundDecimal(double number, int precision) {
        try {
            startBlockExecution(BlockType.FUNCTION, "roundDecimal");
            return Double.parseDouble(JavaUtil.formatNumber(number, precision));
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
    @Block(exclusiveToBlocks = true)
    public Object callJava(String methodLookupString, String json, Object a0, Object a1, Object a2, Object a3, Object a4, Object a5, Object a6, Object a7, Object a8, Object a9, Object a10, Object a11, Object a12, Object a13, Object a14, Object a15, Object a16, Object a17, Object a18, Object a19, Object a20) {
        try {
            startBlockExecution(BlockType.FUNCTION, "Java method " + BlocksClassFilter.getUserVisibleName(methodLookupString));
            try {
                return callJavaVarArgs(methodLookupString, json, a0, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20);
            } catch (Throwable th) {
                e = th;
                try {
                    this.blocksOpMode.handleFatalException(e);
                    throw new AssertionError("impossible", e);
                } finally {
                    endBlockExecution();
                }
            }
        } catch (Throwable th2) {
            e = th2;
        }
    }

    @JavascriptInterface
    @Block(exclusiveToBlocks = true)
    public boolean callJava_boolean(String methodLookupString, String json, Object a0, Object a1, Object a2, Object a3, Object a4, Object a5, Object a6, Object a7, Object a8, Object a9, Object a10, Object a11, Object a12, Object a13, Object a14, Object a15, Object a16, Object a17, Object a18, Object a19, Object a20) {
        try {
            startBlockExecution(BlockType.FUNCTION, "Java method " + BlocksClassFilter.getUserVisibleName(methodLookupString));
            try {
                return ((Boolean) callJavaVarArgs(methodLookupString, json, a0, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20)).booleanValue();
            } catch (Throwable th) {
                e = th;
                try {
                    this.blocksOpMode.handleFatalException(e);
                    throw new AssertionError("impossible", e);
                } finally {
                    endBlockExecution();
                }
            }
        } catch (Throwable th2) {
            e = th2;
        }
    }

    @JavascriptInterface
    @Block(exclusiveToBlocks = true)
    public String callJava_String(String methodLookupString, String json, Object a0, Object a1, Object a2, Object a3, Object a4, Object a5, Object a6, Object a7, Object a8, Object a9, Object a10, Object a11, Object a12, Object a13, Object a14, Object a15, Object a16, Object a17, Object a18, Object a19, Object a20) {
        try {
            startBlockExecution(BlockType.FUNCTION, "Java method " + BlocksClassFilter.getUserVisibleName(methodLookupString));
            try {
                Object result = callJavaVarArgs(methodLookupString, json, a0, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20);
                return result == null ? null : result.toString();
            } catch (Throwable th) {
                e = th;
                try {
                    this.blocksOpMode.handleFatalException(e);
                    throw new AssertionError("impossible", e);
                } finally {
                    endBlockExecution();
                }
            }
        } catch (Throwable th2) {
            e = th2;
        }
    }

    private Object callJavaVarArgs(String methodLookupString, String json, Object... objectArgs) throws Throwable {
        Method method = BlocksClassFilter.getInstance().findStaticMethod(methodLookupString);
        if (method == null) {
            throw new RuntimeException("Could not find method " + methodLookupString + ".");
        }
        Object[] jsonArgs = (Object[]) SimpleGson.getInstance().fromJson(json, Object[].class);
        Class<?>[] parameterTypes = method.getParameterTypes();
        if (jsonArgs.length != parameterTypes.length || objectArgs.length < parameterTypes.length) {
            throw new RuntimeException("Number of arguments does not match required number of parameters.");
        }
        String[] parameterLabels = HardwareUtil.getParameterLabels(method);
        List<Gamepad> gamepads = new ArrayList<>();
        Object[] args = new Object[parameterTypes.length];
        for (int i = 0; i < args.length; i++) {
            args[i] = determineArgument(adjustParameterType(parameterTypes[i]), objectArgs[i], jsonArgs[i], parameterLabels[i], gamepads);
        }
        try {
            return method.invoke(null, args);
        } catch (InvocationTargetException e) {
            Throwable cause = e.getCause();
            if (cause != null) {
                throw cause;
            }
            throw e;
        } catch (Exception e2) {
            throw new RuntimeException("Unable to invoke method " + methodLookupString + ".", e2);
        }
    }

    @JavascriptInterface
    @Block(exclusiveToBlocks = true)
    public Object callHardware(String deviceName, String methodLookupString, String json, Object a0, Object a1, Object a2, Object a3, Object a4, Object a5, Object a6, Object a7, Object a8, Object a9, Object a10, Object a11, Object a12, Object a13, Object a14, Object a15, Object a16, Object a17, Object a18, Object a19, Object a20) {
        try {
            startBlockExecution(BlockType.FUNCTION, "Hardware method " + BlocksClassFilter.getUserVisibleName(methodLookupString));
            try {
                return callHardwareVarArgs(deviceName, methodLookupString, json, a0, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20);
            } catch (Throwable th) {
                e = th;
                try {
                    this.blocksOpMode.handleFatalException(e);
                    throw new AssertionError("impossible", e);
                } finally {
                    endBlockExecution();
                }
            }
        } catch (Throwable th2) {
            e = th2;
        }
    }

    @JavascriptInterface
    @Block(exclusiveToBlocks = true)
    public boolean callHardware_boolean(String deviceName, String methodLookupString, String json, Object a0, Object a1, Object a2, Object a3, Object a4, Object a5, Object a6, Object a7, Object a8, Object a9, Object a10, Object a11, Object a12, Object a13, Object a14, Object a15, Object a16, Object a17, Object a18, Object a19, Object a20) {
        try {
            startBlockExecution(BlockType.FUNCTION, "Hardware method " + BlocksClassFilter.getUserVisibleName(methodLookupString));
            try {
                return ((Boolean) callHardwareVarArgs(deviceName, methodLookupString, json, a0, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20)).booleanValue();
            } catch (Throwable th) {
                e = th;
                try {
                    this.blocksOpMode.handleFatalException(e);
                    throw new AssertionError("impossible", e);
                } finally {
                    endBlockExecution();
                }
            }
        } catch (Throwable th2) {
            e = th2;
        }
    }

    @JavascriptInterface
    @Block(exclusiveToBlocks = true)
    public String callHardware_String(String deviceName, String methodLookupString, String json, Object a0, Object a1, Object a2, Object a3, Object a4, Object a5, Object a6, Object a7, Object a8, Object a9, Object a10, Object a11, Object a12, Object a13, Object a14, Object a15, Object a16, Object a17, Object a18, Object a19, Object a20) {
        try {
            startBlockExecution(BlockType.FUNCTION, "Hardware method " + BlocksClassFilter.getUserVisibleName(methodLookupString));
            try {
                Object result = callHardwareVarArgs(deviceName, methodLookupString, json, a0, a1, a2, a3, a4, a5, a6, a7, a8, a9, a10, a11, a12, a13, a14, a15, a16, a17, a18, a19, a20);
                return result == null ? null : result.toString();
            } catch (Throwable th) {
                e = th;
                try {
                    this.blocksOpMode.handleFatalException(e);
                    throw new AssertionError("impossible", e);
                } finally {
                    endBlockExecution();
                }
            }
        } catch (Throwable th2) {
            e = th2;
        }
    }

    private Object callHardwareVarArgs(String deviceName, String methodLookupString, String json, Object... objectArgs) throws Throwable {
        MiscAccess miscAccess = this;
        Method method = BlocksClassFilter.getInstance().findHardwareMethod(methodLookupString);
        if (method == null) {
            throw new RuntimeException("Could not find method " + methodLookupString + ".");
        }
        Object hardwareDevice = miscAccess.hardwareMap.get((Class<? extends Object>) method.getDeclaringClass(), deviceName);
        Object[] jsonArgs = (Object[]) SimpleGson.getInstance().fromJson(json, Object[].class);
        Class<?>[] parameterTypes = method.getParameterTypes();
        if (jsonArgs.length != parameterTypes.length || objectArgs.length < parameterTypes.length) {
            throw new RuntimeException("Number of arguments does not match required number of parameters.");
        }
        String[] parameterLabels = HardwareUtil.getParameterLabels(method);
        List<Gamepad> gamepads = new ArrayList<>();
        Object[] args = new Object[parameterTypes.length];
        int i = 0;
        while (i < args.length) {
            Object[] args2 = args;
            args2[i] = determineArgument(miscAccess.adjustParameterType(parameterTypes[i]), objectArgs[i], jsonArgs[i], parameterLabels[i], gamepads);
            i++;
            args = args2;
            parameterTypes = parameterTypes;
            miscAccess = this;
        }
        try {
            return method.invoke(hardwareDevice, args);
        } catch (InvocationTargetException e) {
            Throwable cause = e.getCause();
            if (cause != null) {
                throw cause;
            }
            throw e;
        } catch (Exception e2) {
            throw new RuntimeException("Unable to invoke method " + methodLookupString + ".", e2);
        }
    }

    private Class adjustParameterType(Class parameterType) {
        if (parameterType.equals(Boolean.TYPE)) {
            return Boolean.class;
        }
        if (parameterType.equals(Character.TYPE)) {
            return Character.class;
        }
        if (parameterType.equals(Byte.TYPE)) {
            return Byte.class;
        }
        if (parameterType.equals(Short.TYPE)) {
            return Short.class;
        }
        if (parameterType.equals(Integer.TYPE)) {
            return Integer.class;
        }
        if (parameterType.equals(Long.TYPE)) {
            return Long.class;
        }
        if (parameterType.equals(Float.TYPE)) {
            return Float.class;
        }
        if (parameterType.equals(Double.TYPE)) {
            return Double.class;
        }
        return parameterType;
    }

    private Object determineArgument(Class parameterType, Object objectArg, Object jsonArg, String parameterLabel, List<Gamepad> gamepads) {
        if (parameterType.equals(LinearOpMode.class) || parameterType.equals(OpMode.class)) {
            return this.blocksOpMode;
        }
        if (parameterType.equals(HardwareMap.class)) {
            return this.blocksOpMode.hardwareMap;
        }
        if (parameterType.equals(Telemetry.class)) {
            return this.blocksOpMode.telemetry;
        }
        if (parameterType.equals(Gamepad.class)) {
            if (parameterLabel.equals("gamepad1")) {
                return this.blocksOpMode.gamepad1;
            }
            if (parameterLabel.equals("gamepad2")) {
                return this.blocksOpMode.gamepad2;
            }
            if (gamepads.isEmpty()) {
                gamepads.add(this.blocksOpMode.gamepad1);
                gamepads.add(this.blocksOpMode.gamepad2);
            }
            return gamepads.remove(0);
        }
        if (objectArg == null) {
            if (jsonArg == null) {
                return null;
            }
            if (parameterType.equals(jsonArg.getClass())) {
                return jsonArg;
            }
            if (jsonArg instanceof String) {
                try {
                    return coerceStringValue((String) jsonArg, parameterType);
                } catch (Exception e) {
                    try {
                        return parameterType.cast(jsonArg);
                    } catch (Exception e2) {
                    }
                }
            }
        } else {
            if (parameterType.equals(objectArg.getClass())) {
                return objectArg;
            }
            try {
                return parameterType.cast(objectArg);
            } catch (Exception e3) {
            }
        }
        throw new RuntimeException("Unable to convert " + objectArg + " and/or " + jsonArg + " to " + parameterType + ".");
    }

    private Object coerceStringValue(String value, Class parameterType) {
        if (parameterType.equals(Character.class)) {
            if (value.length() >= 1) {
                return new Character(value.charAt(0));
            }
        } else {
            if (parameterType.equals(Byte.class)) {
                return Byte.valueOf((byte) round(value));
            }
            if (parameterType.equals(Short.class)) {
                return Short.valueOf((short) round(value));
            }
            if (parameterType.equals(Integer.class)) {
                return Integer.valueOf((int) round(value));
            }
            if (parameterType.equals(Long.class)) {
                return Long.valueOf(round(value));
            }
            if (parameterType.equals(Float.class)) {
                return Float.valueOf(value);
            }
            if (parameterType.equals(Double.class)) {
                return Double.valueOf(value);
            }
            if (parameterType.isEnum()) {
                return coerceToEnum(value, parameterType);
            }
        }
        throw new RuntimeException("Unable to convert \"" + value + "\" to " + parameterType);
    }

    private Object coerceToEnum(String value, Class parameterType) {
        return checkArg(value, parameterType, parameterType.getSimpleName());
    }

    private static long round(String value) {
        return Math.round(Double.valueOf(value).doubleValue());
    }

    @JavascriptInterface
    @Block(exclusiveToBlocks = true)
    public int listLength(Object o) {
        try {
            startBlockExecution(BlockType.SPECIAL, "length of");
            return JavaUtil.listLength(o);
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
    @Block(exclusiveToBlocks = true)
    public boolean listIsEmpty(Object o) {
        try {
            startBlockExecution(BlockType.SPECIAL, "is empty");
            return JavaUtil.listIsEmpty(o);
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
    @Block(exclusiveToBlocks = true)
    public String huskyLensBlockToText(String json) {
        try {
            startBlockExecution(BlockType.FUNCTION, "HuskyLens.Block", "toText");
            return HuskyLensAccess.huskyLensBlockToText(json);
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
    @Block(exclusiveToBlocks = true)
    public String huskyLensArrowToText(String json) {
        try {
            startBlockExecution(BlockType.FUNCTION, "HuskyLens.Arrow", "toText");
            return HuskyLensAccess.huskyLensArrowToText(json);
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
    @Block(exclusiveToBlocks = true)
    public String objectToJson(Object o) {
        try {
            if (o instanceof LLResult) {
                return LLResultAccess.llResultToJson((LLResult) o);
            }
            return toJson(o);
        } catch (Throwable e) {
            this.blocksOpMode.handleFatalException(e);
            throw new AssertionError("impossible", e);
        }
    }
}
