package com.google.zxing.datamatrix.encoder;

/* JADX INFO: loaded from: classes8.dex */
final class X12Encoder extends C40Encoder {
    X12Encoder() {
    }

    @Override // com.google.zxing.datamatrix.encoder.C40Encoder, com.google.zxing.datamatrix.encoder.Encoder
    public int getEncodingMode() {
        return 3;
    }

    @Override // com.google.zxing.datamatrix.encoder.C40Encoder, com.google.zxing.datamatrix.encoder.Encoder
    public void encode(EncoderContext context) {
        StringBuilder buffer = new StringBuilder();
        while (true) {
            if (!context.hasMoreCharacters()) {
                break;
            }
            char c = context.getCurrentChar();
            context.pos++;
            encodeChar(c, buffer);
            if (buffer.length() % 3 == 0) {
                writeNextTriplet(context, buffer);
                if (HighLevelEncoder.lookAheadTest(context.getMessage(), context.pos, getEncodingMode()) != getEncodingMode()) {
                    context.signalEncoderChange(0);
                    break;
                }
            }
        }
        handleEOD(context, buffer);
    }

    @Override // com.google.zxing.datamatrix.encoder.C40Encoder
    int encodeChar(char c, StringBuilder sb) {
        switch (c) {
            case '\r':
                sb.append((char) 0);
                return 1;
            case ' ':
                sb.append((char) 3);
                return 1;
            case '*':
                sb.append((char) 1);
                return 1;
            case '>':
                sb.append((char) 2);
                return 1;
            default:
                if (c >= '0' && c <= '9') {
                    sb.append((char) ((c - '0') + 4));
                } else if (c >= 'A' && c <= 'Z') {
                    sb.append((char) ((c - 'A') + 14));
                } else {
                    HighLevelEncoder.illegalCharacter(c);
                }
                return 1;
        }
    }

    @Override // com.google.zxing.datamatrix.encoder.C40Encoder
    void handleEOD(EncoderContext context, StringBuilder buffer) {
        context.updateSymbolInfo();
        int available = context.getSymbolInfo().getDataCapacity() - context.getCodewordCount();
        int count = buffer.length();
        context.pos -= count;
        if (context.getRemainingCharacters() > 1 || available > 1 || context.getRemainingCharacters() != available) {
            context.writeCodeword((char) 254);
        }
        if (context.getNewEncoding() < 0) {
            context.signalEncoderChange(0);
        }
    }
}
