package com.google.zxing.datamatrix.encoder;

import com.qualcomm.robotcore.eventloop.SyncdDevice;
import com.sun.tools.javac.jvm.ByteCodes;
import org.firstinspires.ftc.robotcore.internal.usb.UsbConstants;

/* JADX INFO: loaded from: classes8.dex */
public final class ErrorCorrection {
    private static final int MODULO_VALUE = 301;
    private static final int[] FACTOR_SETS = {5, 7, 10, 11, 12, 14, 18, 20, 24, 28, 36, 42, 48, 56, 62, 68};
    private static final int[][] FACTORS = {new int[]{228, 48, 15, 111, 62}, new int[]{23, 68, 144, 134, 240, 92, 254}, new int[]{28, 24, ByteCodes.invokeinterface, ByteCodes.if_acmpne, 223, 248, 116, 255, 110, 61}, new int[]{ByteCodes.dreturn, 138, 205, 12, ByteCodes.monitorenter, 168, 39, 245, 60, 97, 120}, new int[]{41, 153, ByteCodes.ifle, 91, 61, 42, 142, 213, 97, ByteCodes.getstatic, 100, 242}, new int[]{ByteCodes.ifge, 97, ByteCodes.checkcast, 252, 95, 9, ByteCodes.ifgt, 119, 138, 45, 18, ByteCodes.invokedynamic, 83, ByteCodes.invokeinterface}, new int[]{83, ByteCodes.monitorexit, 100, 39, ByteCodes.newarray, 75, 66, 61, 241, 213, 109, 129, 94, 254, 225, 48, 90, ByteCodes.newarray}, new int[]{15, ByteCodes.monitorexit, 244, 9, 233, 71, 168, 2, ByteCodes.newarray, ByteCodes.if_icmpne, 153, 145, 253, 79, 108, 82, 27, ByteCodes.freturn, ByteCodes.invokedynamic, ByteCodes.ireturn}, new int[]{52, ByteCodes.arraylength, 88, 205, 109, 39, ByteCodes.areturn, 21, 155, ByteCodes.multianewarray, 251, 223, 155, 21, 5, ByteCodes.ireturn, 254, 124, 12, ByteCodes.putfield, ByteCodes.invokestatic, 96, 50, ByteCodes.instanceof_}, new int[]{211, 231, 43, 97, 71, 96, 103, ByteCodes.freturn, 37, 151, ByteCodes.tableswitch, 53, 75, 34, 249, 121, 17, 138, 110, 213, 141, 136, 120, 151, 233, 168, 93, 255}, new int[]{245, 127, 242, 218, 130, SyncdDevice.msAbnormalReopenInterval, ByteCodes.if_icmpge, ByteCodes.putfield, 102, 120, 84, ByteCodes.putstatic, 220, 251, 80, ByteCodes.invokevirtual, 229, 18, 2, 4, 68, 33, 101, 137, 95, 119, 115, 44, ByteCodes.dreturn, ByteCodes.invokestatic, 59, 25, 225, 98, 81, 112}, new int[]{77, ByteCodes.instanceof_, 137, 31, 19, 38, 22, 153, 247, 105, 122, 2, 245, 133, 242, 8, ByteCodes.dreturn, 95, 100, 9, ByteCodes.goto_, 105, 214, 111, 57, 121, 21, 1, 253, 57, 54, 101, 248, ByteCodes.breakpoint, 69, 50, 150, ByteCodes.return_, 226, 5, 9, 5}, new int[]{245, 132, ByteCodes.ireturn, 223, 96, 32, 117, 22, 238, 133, 238, 231, 205, ByteCodes.newarray, 237, 87, ByteCodes.athrow, 106, 16, 147, 118, 23, 37, 90, ByteCodes.tableswitch, 205, 131, 88, 120, 100, 66, 138, ByteCodes.invokedynamic, 240, 82, 44, ByteCodes.areturn, 87, ByteCodes.new_, 147, ByteCodes.if_icmpne, ByteCodes.dreturn, 69, 213, 92, 253, 225, 19}, new int[]{ByteCodes.dreturn, 9, 223, 238, 12, 17, 220, 208, 100, 29, ByteCodes.dreturn, ByteCodes.tableswitch, 230, ByteCodes.checkcast, 215, 235, 150, ByteCodes.if_icmpeq, 36, 223, 38, 200, 132, 54, 228, 146, 218, 234, 117, ByteCodes.ByteCodeCount, 29, 232, 144, 238, 22, 150, ByteCodes.jsr_w, 117, 62, 207, ByteCodes.if_icmple, 13, 137, 245, 127, 67, 247, 28, 155, 43, ByteCodes.ByteCodeCount, 107, 233, 53, 143, 46}, new int[]{242, 93, ByteCodes.ret, 50, 144, 210, 39, 118, ByteCodes.breakpoint, ByteCodes.newarray, ByteCodes.jsr_w, ByteCodes.anewarray, 143, 108, ByteCodes.wide, 37, ByteCodes.invokeinterface, 112, 134, 230, 245, 63, ByteCodes.multianewarray, ByteCodes.arraylength, SyncdDevice.msAbnormalReopenInterval, 106, ByteCodes.invokeinterface, 221, ByteCodes.dreturn, 64, ByteCodes.fmod, 71, ByteCodes.if_icmplt, 44, 147, 6, 27, 218, 51, 63, 87, 10, 40, 130, ByteCodes.newarray, 17, ByteCodes.if_icmpgt, 31, ByteCodes.areturn, ByteCodes.tableswitch, 4, 107, 232, 7, 94, ByteCodes.if_acmpne, UsbConstants.USB_CLASS_WIRELESS_CONTROLLER, 124, 86, 47, 11, 204}, new int[]{220, 228, 173, 89, 251, 149, ByteCodes.if_icmpeq, 56, 89, 33, 147, 244, 154, 36, 73, 127, 213, 136, 248, ByteCodes.getfield, 234, ByteCodes.multianewarray, ByteCodes.ifle, ByteCodes.return_, 68, 122, 93, 213, 15, ByteCodes.if_icmpne, 227, 236, 66, 139, 153, ByteCodes.invokeinterface, ByteCodes.breakpoint, ByteCodes.goto_, ByteCodes.putstatic, 25, 220, 232, 96, 210, 231, 136, 223, UsbConstants.USB_CLASS_MISC, ByteCodes.putfield, 241, 59, 52, ByteCodes.ireturn, 25, 49, 232, 211, ByteCodes.anewarray, 64, 54, 108, 153, 132, 63, 96, 103, 82, ByteCodes.invokedynamic}};
    private static final int[] LOG = new int[256];
    private static final int[] ALOG = new int[255];

    static {
        int p = 1;
        for (int i = 0; i < 255; i++) {
            ALOG[i] = p;
            LOG[p] = i;
            int i2 = p << 1;
            p = i2;
            if (i2 >= 256) {
                p ^= 301;
            }
        }
    }

    private ErrorCorrection() {
    }

    public static String encodeECC200(String codewords, SymbolInfo symbolInfo) {
        if (codewords.length() != symbolInfo.getDataCapacity()) {
            throw new IllegalArgumentException("The number of codewords does not match the selected symbol");
        }
        StringBuilder sb = new StringBuilder(symbolInfo.getDataCapacity() + symbolInfo.getErrorCodewords());
        sb.append(codewords);
        int blockCount = symbolInfo.getInterleavedBlockCount();
        if (blockCount != 1) {
            sb.setLength(sb.capacity());
            int[] dataSizes = new int[blockCount];
            int[] errorSizes = new int[blockCount];
            for (int i = 0; i < blockCount; i++) {
                dataSizes[i] = symbolInfo.getDataLengthForInterleavedBlock(i + 1);
                errorSizes[i] = symbolInfo.getErrorLengthForInterleavedBlock(i + 1);
            }
            for (int block = 0; block < blockCount; block++) {
                StringBuilder temp = new StringBuilder(dataSizes[block]);
                for (int d = block; d < symbolInfo.getDataCapacity(); d += blockCount) {
                    temp.append(codewords.charAt(d));
                }
                String ecc = createECCBlock(temp.toString(), errorSizes[block]);
                int pos = 0;
                int e = block;
                while (e < errorSizes[block] * blockCount) {
                    sb.setCharAt(symbolInfo.getDataCapacity() + e, ecc.charAt(pos));
                    e += blockCount;
                    pos++;
                }
            }
        } else {
            String ecc2 = createECCBlock(codewords, symbolInfo.getErrorCodewords());
            sb.append(ecc2);
        }
        return sb.toString();
    }

    private static String createECCBlock(CharSequence codewords, int numECWords) {
        int table = -1;
        int i = 0;
        while (true) {
            if (i >= FACTOR_SETS.length) {
                break;
            }
            if (FACTOR_SETS[i] != numECWords) {
                i++;
            } else {
                table = i;
                break;
            }
        }
        if (table < 0) {
            throw new IllegalArgumentException("Illegal number of error correction codewords specified: ".concat(String.valueOf(numECWords)));
        }
        int[] poly = FACTORS[table];
        char[] ecc = new char[numECWords];
        for (int i2 = 0; i2 < numECWords; i2++) {
            ecc[i2] = 0;
        }
        for (int i3 = 0; i3 < codewords.length(); i3++) {
            int m = ecc[numECWords - 1] ^ codewords.charAt(i3);
            for (int k = numECWords - 1; k > 0; k--) {
                if (m != 0 && poly[k] != 0) {
                    ecc[k] = (char) (ecc[k - 1] ^ ALOG[(LOG[m] + LOG[poly[k]]) % 255]);
                } else {
                    ecc[k] = ecc[k - 1];
                }
            }
            if (m != 0 && poly[0] != 0) {
                ecc[0] = (char) ALOG[(LOG[m] + LOG[poly[0]]) % 255];
            } else {
                ecc[0] = 0;
            }
        }
        char[] eccReversed = new char[numECWords];
        for (int i4 = 0; i4 < numECWords; i4++) {
            eccReversed[i4] = ecc[(numECWords - i4) - 1];
        }
        return String.valueOf(eccReversed);
    }
}
