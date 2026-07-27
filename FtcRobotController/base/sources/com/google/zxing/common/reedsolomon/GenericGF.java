package com.google.zxing.common.reedsolomon;

import org.firstinspires.ftc.robotcore.internal.usb.UsbConstants;
import org.opencv.videoio.Videoio;

/* JADX INFO: loaded from: classes8.dex */
public final class GenericGF {
    public static final GenericGF AZTEC_DATA_8;
    public static final GenericGF DATA_MATRIX_FIELD_256;
    public static final GenericGF MAXICODE_FIELD_64;
    private final int[] expTable;
    private final int generatorBase;
    private final int[] logTable;
    private final GenericGFPoly one;
    private final int primitive;
    private final int size;
    private final GenericGFPoly zero;
    public static final GenericGF AZTEC_DATA_12 = new GenericGF(4201, 4096, 1);
    public static final GenericGF AZTEC_DATA_10 = new GenericGF(UsbConstants.VENDOR_ID_NEC, 1024, 1);
    public static final GenericGF AZTEC_DATA_6 = new GenericGF(67, 64, 1);
    public static final GenericGF AZTEC_PARAM = new GenericGF(19, 16, 1);
    public static final GenericGF QR_CODE_FIELD_256 = new GenericGF(285, 256, 0);

    static {
        GenericGF genericGF = new GenericGF(Videoio.CAP_PROP_PVAPI_FRAMESTARTTRIGGERMODE, 256, 1);
        DATA_MATRIX_FIELD_256 = genericGF;
        AZTEC_DATA_8 = genericGF;
        MAXICODE_FIELD_64 = AZTEC_DATA_6;
    }

    public GenericGF(int primitive, int size, int b) {
        this.primitive = primitive;
        this.size = size;
        this.generatorBase = b;
        this.expTable = new int[size];
        this.logTable = new int[size];
        int x = 1;
        for (int i = 0; i < size; i++) {
            this.expTable[i] = x;
            int i2 = x << 1;
            x = i2;
            if (i2 >= size) {
                x = (x ^ primitive) & (size - 1);
            }
        }
        for (int i3 = 0; i3 < size - 1; i3++) {
            this.logTable[this.expTable[i3]] = i3;
        }
        this.zero = new GenericGFPoly(this, new int[]{0});
        this.one = new GenericGFPoly(this, new int[]{1});
    }

    GenericGFPoly getZero() {
        return this.zero;
    }

    GenericGFPoly getOne() {
        return this.one;
    }

    GenericGFPoly buildMonomial(int degree, int coefficient) {
        if (degree < 0) {
            throw new IllegalArgumentException();
        }
        if (coefficient == 0) {
            return this.zero;
        }
        int[] coefficients = new int[degree + 1];
        coefficients[0] = coefficient;
        return new GenericGFPoly(this, coefficients);
    }

    static int addOrSubtract(int a, int b) {
        return a ^ b;
    }

    int exp(int a) {
        return this.expTable[a];
    }

    int log(int a) {
        if (a == 0) {
            throw new IllegalArgumentException();
        }
        return this.logTable[a];
    }

    int inverse(int a) {
        if (a == 0) {
            throw new ArithmeticException();
        }
        return this.expTable[(this.size - this.logTable[a]) - 1];
    }

    int multiply(int a, int b) {
        if (a == 0 || b == 0) {
            return 0;
        }
        return this.expTable[(this.logTable[a] + this.logTable[b]) % (this.size - 1)];
    }

    public int getSize() {
        return this.size;
    }

    public int getGeneratorBase() {
        return this.generatorBase;
    }

    public String toString() {
        return "GF(0x" + Integer.toHexString(this.primitive) + ',' + this.size + ')';
    }
}
