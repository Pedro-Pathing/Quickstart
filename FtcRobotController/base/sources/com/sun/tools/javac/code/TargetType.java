package com.sun.tools.javac.code;

import com.sun.tools.javac.util.Assert;

/* JADX INFO: loaded from: classes.dex */
public enum TargetType {
    CLASS_TYPE_PARAMETER(0),
    METHOD_TYPE_PARAMETER(1),
    CLASS_EXTENDS(16),
    CLASS_TYPE_PARAMETER_BOUND(17),
    METHOD_TYPE_PARAMETER_BOUND(18),
    FIELD(19),
    METHOD_RETURN(20),
    METHOD_RECEIVER(21),
    METHOD_FORMAL_PARAMETER(22),
    THROWS(23),
    LOCAL_VARIABLE(64, true),
    RESOURCE_VARIABLE(65, true),
    EXCEPTION_PARAMETER(66, true),
    INSTANCEOF(67, true),
    NEW(68, true),
    CONSTRUCTOR_REFERENCE(69, true),
    METHOD_REFERENCE(70, true),
    CAST(71, true),
    CONSTRUCTOR_INVOCATION_TYPE_ARGUMENT(72, true),
    METHOD_INVOCATION_TYPE_ARGUMENT(73, true),
    CONSTRUCTOR_REFERENCE_TYPE_ARGUMENT(74, true),
    METHOD_REFERENCE_TYPE_ARGUMENT(75, true),
    UNKNOWN(255);

    private static final int MAXIMUM_TARGET_TYPE_VALUE = 75;
    private static final TargetType[] targets = new TargetType[76];
    private final boolean isLocal;
    private final int targetTypeValue;

    static {
        TargetType[] alltargets = values();
        for (TargetType target : alltargets) {
            if (target.targetTypeValue != UNKNOWN.targetTypeValue) {
                targets[target.targetTypeValue] = target;
            }
        }
        for (int i = 0; i <= 75; i++) {
            if (targets[i] == null) {
                targets[i] = UNKNOWN;
            }
        }
    }

    TargetType(int targetTypeValue) {
        this(targetTypeValue, false);
    }

    TargetType(int targetTypeValue, boolean isLocal) {
        if (targetTypeValue < 0 || targetTypeValue > 255) {
            Assert.error("Attribute type value needs to be an unsigned byte: " + String.format("0x%02X", Integer.valueOf(targetTypeValue)));
        }
        this.targetTypeValue = targetTypeValue;
        this.isLocal = isLocal;
    }

    public boolean isLocal() {
        return this.isLocal;
    }

    public int targetTypeValue() {
        return this.targetTypeValue;
    }

    public static boolean isValidTargetTypeValue(int tag) {
        if (tag == UNKNOWN.targetTypeValue) {
            return true;
        }
        return tag >= 0 && tag < targets.length;
    }

    public static TargetType fromTargetTypeValue(int tag) {
        if (tag == UNKNOWN.targetTypeValue) {
            return UNKNOWN;
        }
        if (tag < 0 || tag >= targets.length) {
            Assert.error("Unknown TargetType: " + tag);
        }
        return targets[tag];
    }
}
