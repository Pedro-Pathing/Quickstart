package com.google.zxing.datamatrix.encoder;

/* JADX INFO: loaded from: classes8.dex */
final class EdifactEncoder implements Encoder {
    EdifactEncoder() {
    }

    @Override // com.google.zxing.datamatrix.encoder.Encoder
    public int getEncodingMode() {
        return 4;
    }

    @Override // com.google.zxing.datamatrix.encoder.Encoder
    public void encode(EncoderContext context) {
        StringBuilder buffer = new StringBuilder();
        while (true) {
            if (!context.hasMoreCharacters()) {
                break;
            }
            encodeChar(context.getCurrentChar(), buffer);
            context.pos++;
            if (buffer.length() >= 4) {
                context.writeCodewords(encodeToCodewords(buffer));
                buffer.delete(0, 4);
                if (HighLevelEncoder.lookAheadTest(context.getMessage(), context.pos, getEncodingMode()) != getEncodingMode()) {
                    context.signalEncoderChange(0);
                    break;
                }
            }
        }
        buffer.append((char) 31);
        handleEOD(context, buffer);
    }

    private static void handleEOD(EncoderContext context, CharSequence buffer) {
        try {
            int count = buffer.length();
            if (count == 0) {
                return;
            }
            boolean restInAscii = true;
            if (count == 1) {
                context.updateSymbolInfo();
                int available = context.getSymbolInfo().getDataCapacity() - context.getCodewordCount();
                int remaining = context.getRemainingCharacters();
                if (remaining > available) {
                    context.updateSymbolInfo(context.getCodewordCount() + 1);
                    available = context.getSymbolInfo().getDataCapacity() - context.getCodewordCount();
                }
                if (remaining <= available && available <= 2) {
                    return;
                }
            }
            if (count > 4) {
                throw new IllegalStateException("Count must not exceed 4");
            }
            int restChars = count - 1;
            String encoded = encodeToCodewords(buffer);
            if (context.hasMoreCharacters() || restChars > 2) {
                restInAscii = false;
            }
            if (restChars <= 2) {
                context.updateSymbolInfo(context.getCodewordCount() + restChars);
                if (context.getSymbolInfo().getDataCapacity() - context.getCodewordCount() >= 3) {
                    restInAscii = false;
                    context.updateSymbolInfo(context.getCodewordCount() + encoded.length());
                }
            }
            if (restInAscii) {
                context.resetSymbolInfo();
                context.pos -= restChars;
            } else {
                context.writeCodewords(encoded);
            }
        } finally {
            context.signalEncoderChange(0);
        }
    }

    private static void encodeChar(char c, StringBuilder sb) {
        if (c >= ' ' && c <= '?') {
            sb.append(c);
        } else if (c >= '@' && c <= '^') {
            sb.append((char) (c - '@'));
        } else {
            HighLevelEncoder.illegalCharacter(c);
        }
    }

    private static String encodeToCodewords(CharSequence sb) {
        char c3;
        int len = sb.length();
        if (len == 0) {
            throw new IllegalStateException("StringBuilder must not be empty");
        }
        char c4 = 0;
        char c1 = sb.charAt(0);
        char c2 = len >= 2 ? sb.charAt(1) : (char) 0;
        if (len < 3) {
            c3 = 0;
        } else {
            c3 = sb.charAt(2);
        }
        if (len >= 4) {
            c4 = sb.charAt(3);
        }
        int v = (c1 << 18) + (c2 << '\f') + (c3 << 6) + c4;
        char cw1 = (char) ((v >> 16) & 255);
        char cw2 = (char) ((v >> 8) & 255);
        char cw3 = (char) (v & 255);
        StringBuilder res = new StringBuilder(3);
        res.append(cw1);
        if (len >= 2) {
            res.append(cw2);
        }
        if (len >= 3) {
            res.append(cw3);
        }
        return res.toString();
    }
}
