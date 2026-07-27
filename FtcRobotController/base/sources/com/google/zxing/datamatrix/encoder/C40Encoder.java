package com.google.zxing.datamatrix.encoder;

/* JADX INFO: loaded from: classes8.dex */
class C40Encoder implements Encoder {
    C40Encoder() {
    }

    @Override // com.google.zxing.datamatrix.encoder.Encoder
    public int getEncodingMode() {
        return 1;
    }

    @Override // com.google.zxing.datamatrix.encoder.Encoder
    public void encode(EncoderContext encoderContext) {
        StringBuilder sb = new StringBuilder();
        while (true) {
            if (!encoderContext.hasMoreCharacters()) {
                break;
            }
            char currentChar = encoderContext.getCurrentChar();
            encoderContext.pos++;
            int iEncodeChar = encodeChar(currentChar, sb);
            int codewordCount = encoderContext.getCodewordCount() + ((sb.length() / 3) << 1);
            encoderContext.updateSymbolInfo(codewordCount);
            int dataCapacity = encoderContext.getSymbolInfo().getDataCapacity() - codewordCount;
            if (!encoderContext.hasMoreCharacters()) {
                StringBuilder sb2 = new StringBuilder();
                if (sb.length() % 3 == 2 && dataCapacity != 2) {
                    iEncodeChar = backtrackOneCharacter(encoderContext, sb, sb2, iEncodeChar);
                }
                while (sb.length() % 3 == 1 && (iEncodeChar > 3 || dataCapacity != 1)) {
                    iEncodeChar = backtrackOneCharacter(encoderContext, sb, sb2, iEncodeChar);
                }
            } else if (sb.length() % 3 == 0 && HighLevelEncoder.lookAheadTest(encoderContext.getMessage(), encoderContext.pos, getEncodingMode()) != getEncodingMode()) {
                encoderContext.signalEncoderChange(0);
                break;
            }
        }
        handleEOD(encoderContext, sb);
    }

    private int backtrackOneCharacter(EncoderContext context, StringBuilder buffer, StringBuilder removed, int lastCharSize) {
        int count = buffer.length();
        buffer.delete(count - lastCharSize, count);
        context.pos--;
        char c = context.getCurrentChar();
        int lastCharSize2 = encodeChar(c, removed);
        context.resetSymbolInfo();
        return lastCharSize2;
    }

    static void writeNextTriplet(EncoderContext context, StringBuilder buffer) {
        context.writeCodewords(encodeToCodewords(buffer));
        buffer.delete(0, 3);
    }

    void handleEOD(EncoderContext context, StringBuilder buffer) {
        int unwritten = (buffer.length() / 3) << 1;
        int rest = buffer.length() % 3;
        int curCodewordCount = context.getCodewordCount() + unwritten;
        context.updateSymbolInfo(curCodewordCount);
        int available = context.getSymbolInfo().getDataCapacity() - curCodewordCount;
        if (rest == 2) {
            buffer.append((char) 0);
            while (buffer.length() >= 3) {
                writeNextTriplet(context, buffer);
            }
            if (context.hasMoreCharacters()) {
                context.writeCodeword((char) 254);
            }
        } else if (available == 1 && rest == 1) {
            while (buffer.length() >= 3) {
                writeNextTriplet(context, buffer);
            }
            if (context.hasMoreCharacters()) {
                context.writeCodeword((char) 254);
            }
            context.pos--;
        } else if (rest == 0) {
            while (buffer.length() >= 3) {
                writeNextTriplet(context, buffer);
            }
            if (available > 0 || context.hasMoreCharacters()) {
                context.writeCodeword((char) 254);
            }
        } else {
            throw new IllegalStateException("Unexpected case. Please report!");
        }
        context.signalEncoderChange(0);
    }

    int encodeChar(char c, StringBuilder sb) {
        if (c == ' ') {
            sb.append((char) 3);
            return 1;
        }
        if (c >= '0' && c <= '9') {
            sb.append((char) ((c - '0') + 4));
            return 1;
        }
        if (c >= 'A' && c <= 'Z') {
            sb.append((char) ((c - 'A') + 14));
            return 1;
        }
        if (c < ' ') {
            sb.append((char) 0);
            sb.append(c);
            return 2;
        }
        if (c <= '/') {
            sb.append((char) 1);
            sb.append((char) (c - '!'));
            return 2;
        }
        if (c <= '@') {
            sb.append((char) 1);
            sb.append((char) ((c - ':') + 15));
            return 2;
        }
        if (c <= '_') {
            sb.append((char) 1);
            sb.append((char) ((c - '[') + 22));
            return 2;
        }
        if (c <= 127) {
            sb.append((char) 2);
            sb.append((char) (c - '`'));
            return 2;
        }
        sb.append("\u0001\u001e");
        return encodeChar((char) (c - 128), sb) + 2;
    }

    private static String encodeToCodewords(CharSequence sb) {
        int v = (sb.charAt(0) * 1600) + (sb.charAt(1) * '(') + sb.charAt(2) + 1;
        char cw1 = (char) (v / 256);
        char cw2 = (char) (v % 256);
        return new String(new char[]{cw1, cw2});
    }
}
