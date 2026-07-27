package com.google.zxing.datamatrix.encoder;

/* JADX INFO: loaded from: classes8.dex */
final class TextEncoder extends C40Encoder {
    TextEncoder() {
    }

    @Override // com.google.zxing.datamatrix.encoder.C40Encoder, com.google.zxing.datamatrix.encoder.Encoder
    public int getEncodingMode() {
        return 2;
    }

    @Override // com.google.zxing.datamatrix.encoder.C40Encoder
    int encodeChar(char c, StringBuilder sb) {
        if (c == ' ') {
            sb.append((char) 3);
            return 1;
        }
        if (c >= '0' && c <= '9') {
            sb.append((char) ((c - '0') + 4));
            return 1;
        }
        if (c >= 'a' && c <= 'z') {
            sb.append((char) ((c - 'a') + 14));
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
        if (c >= '[' && c <= '_') {
            sb.append((char) 1);
            sb.append((char) ((c - '[') + 22));
            return 2;
        }
        if (c == '`') {
            sb.append((char) 2);
            sb.append((char) (c - '`'));
            return 2;
        }
        if (c <= 'Z') {
            sb.append((char) 2);
            sb.append((char) ((c - 'A') + 1));
            return 2;
        }
        if (c <= 127) {
            sb.append((char) 2);
            sb.append((char) ((c - '{') + 27));
            return 2;
        }
        sb.append("\u0001\u001e");
        return encodeChar((char) (c - 128), sb) + 2;
    }
}
