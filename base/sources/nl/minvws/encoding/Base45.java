package nl.minvws.encoding;

import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import java.nio.charset.StandardCharsets;
import java.util.Arrays;
import org.firstinspires.ftc.robotcore.internal.stellaris.FlashLoaderDatagram;
import org.firstinspires.ftc.robotcore.internal.stellaris.FlashLoaderGetStatusResponse;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;

/* JADX INFO: loaded from: classes.dex */
public class Base45 {
    private static final int BaseSize = 45;
    private static final int ByteSize = 256;
    private static final int ChunkSize = 2;
    private static final int EncodedChunkSize = 3;
    private static final int SmallEncodedChunkSize = 2;

    private Base45() {
    }

    public static Encoder getEncoder() {
        return Encoder.ENCODER;
    }

    public static Decoder getDecoder() {
        return Decoder.DECODER;
    }

    public static class Encoder {
        private static final byte[] toBase45 = {48, GoBildaPinpointDriver.DEFAULT_ADDRESS, 50, FlashLoaderDatagram.NAK, 52, 53, 54, 55, 56, 57, FlashLoaderGetStatusResponse.STATUS_UNKNOWN_CMD, FlashLoaderGetStatusResponse.STATUS_INVALID_CMD, FlashLoaderGetStatusResponse.STATUS_INVALID_ADDR, 68, 69, 70, 71, 72, 73, 74, 75, 76, 77, 78, 79, 80, OctoQuad.OCTOQUAD_CHIP_ID, 82, 83, 84, 85, 86, 87, 88, 89, 90, 32, 36, 37, 42, 43, 45, 46, 47, 58};
        static final Encoder ENCODER = new Encoder();

        public byte[] encode(byte[] src) {
            int wholeChunkCount = src.length / 2;
            byte[] result = new byte[(wholeChunkCount * 3) + (src.length % 2 == 1 ? 2 : 0)];
            int resultIndex = 0;
            int wholeChunkLength = wholeChunkCount * 2;
            int i = 0;
            while (i < wholeChunkLength) {
                int i2 = i + 1;
                int i3 = i2 + 1;
                int value = ((src[i] & 255) * 256) + (src[i2] & 255);
                int resultIndex2 = resultIndex + 1;
                result[resultIndex] = toBase45[value % 45];
                int resultIndex3 = resultIndex2 + 1;
                result[resultIndex2] = toBase45[(value / 45) % 45];
                result[resultIndex3] = toBase45[(value / 2025) % 45];
                resultIndex = resultIndex3 + 1;
                i = i3;
            }
            if (src.length % 2 != 0) {
                result[result.length - 2] = toBase45[(src[src.length - 1] & 255) % 45];
                result[result.length - 1] = (src[src.length - 1] & 255) < 45 ? toBase45[0] : toBase45[((src[src.length - 1] & 255) / 45) % 45];
            }
            return result;
        }

        public String encodeToString(byte[] src) {
            byte[] encoded = encode(src);
            return new String(encoded, 0, 0, encoded.length);
        }
    }

    public static class Decoder {
        static final Decoder DECODER;
        private static final int[] fromBase45 = new int[256];

        private Decoder() {
        }

        static {
            Arrays.fill(fromBase45, -1);
            for (int i = 0; i < Encoder.toBase45.length; i++) {
                fromBase45[Encoder.toBase45[i]] = i;
            }
            DECODER = new Decoder();
        }

        public byte[] decode(byte[] src) {
            int remainderSize = src.length % 3;
            int[] buffer = new int[src.length];
            for (int i = 0; i < src.length; i++) {
                buffer[i] = fromBase45[src[i]];
                if (buffer[i] == -1) {
                    throw new IllegalArgumentException();
                }
            }
            int i2 = buffer.length;
            int wholeChunkCount = i2 / 3;
            byte[] result = new byte[(wholeChunkCount * 2) + (remainderSize == 2 ? 1 : 0)];
            int resultIndex = 0;
            int wholeChunkLength = wholeChunkCount * 3;
            int val = 0;
            while (val < wholeChunkLength) {
                int i3 = val + 1;
                int i4 = buffer[val];
                int i5 = i3 + 1;
                int i6 = i4 + (buffer[i3] * 45);
                int i7 = i5 + 1;
                int val2 = i6 + (buffer[i5] * 2025);
                int resultIndex2 = resultIndex + 1;
                result[resultIndex] = (byte) (val2 / 256);
                resultIndex = resultIndex2 + 1;
                result[resultIndex2] = (byte) (val2 % 256);
                val = i7;
            }
            if (remainderSize != 0) {
                result[resultIndex] = (byte) (buffer[buffer.length - 2] + (buffer[buffer.length - 1] * 45));
            }
            return result;
        }

        public byte[] decode(String src) {
            return decode(src.getBytes(StandardCharsets.ISO_8859_1));
        }
    }
}
