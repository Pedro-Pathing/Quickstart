package com.google.zxing.datamatrix.encoder;

import com.qualcomm.robotcore.eventloop.SyncdDevice;

/* JADX INFO: loaded from: classes8.dex */
final class Base256Encoder implements Encoder {
    Base256Encoder() {
    }

    @Override // com.google.zxing.datamatrix.encoder.Encoder
    public int getEncodingMode() {
        return 5;
    }

    @Override // com.google.zxing.datamatrix.encoder.Encoder
    public void encode(EncoderContext context) {
        StringBuilder buffer = new StringBuilder();
        buffer.append((char) 0);
        while (true) {
            if (!context.hasMoreCharacters()) {
                break;
            }
            char c = context.getCurrentChar();
            buffer.append(c);
            context.pos++;
            if (HighLevelEncoder.lookAheadTest(context.getMessage(), context.pos, getEncodingMode()) != getEncodingMode()) {
                context.signalEncoderChange(0);
                break;
            }
        }
        int dataCount = buffer.length() - 1;
        int currentSize = context.getCodewordCount() + dataCount + 1;
        context.updateSymbolInfo(currentSize);
        boolean mustPad = context.getSymbolInfo().getDataCapacity() - currentSize > 0;
        if (context.hasMoreCharacters() || mustPad) {
            if (dataCount <= 249) {
                buffer.setCharAt(0, (char) dataCount);
            } else if (dataCount <= 1555) {
                buffer.setCharAt(0, (char) ((dataCount / SyncdDevice.msAbnormalReopenInterval) + 249));
                buffer.insert(1, (char) (dataCount % SyncdDevice.msAbnormalReopenInterval));
            } else {
                throw new IllegalStateException("Message length not in valid ranges: ".concat(String.valueOf(dataCount)));
            }
        }
        int c2 = buffer.length();
        for (int i = 0; i < c2; i++) {
            context.writeCodeword(randomize255State(buffer.charAt(i), context.getCodewordCount() + 1));
        }
    }

    private static char randomize255State(char ch, int codewordPosition) {
        int pseudoRandom = ((codewordPosition * 149) % 255) + 1;
        int tempVariable = ch + pseudoRandom;
        if (tempVariable <= 255) {
            return (char) tempVariable;
        }
        return (char) (tempVariable - 256);
    }
}
