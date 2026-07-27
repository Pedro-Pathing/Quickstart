package com.sun.tools.javac.parser;

import com.sun.tools.javac.file.JavacFileManager;
import com.sun.tools.javac.util.ArrayUtils;
import com.sun.tools.javac.util.Log;
import com.sun.tools.javac.util.Name;
import com.sun.tools.javac.util.Names;
import java.nio.CharBuffer;
import java.util.Arrays;

/* JADX INFO: loaded from: classes.dex */
public class UnicodeReader {
    static final boolean surrogatesSupported = surrogatesSupported();
    protected int bp;
    protected char[] buf;
    protected final int buflen;
    protected char ch;
    protected Log log;
    protected Names names;
    protected char[] sbuf;
    protected int sp;
    protected int unicodeConversionBp;

    protected UnicodeReader(ScannerFactory sf, CharBuffer buffer) {
        this(sf, JavacFileManager.toArray(buffer), buffer.limit());
    }

    protected UnicodeReader(ScannerFactory sf, char[] input, int inputLength) {
        this.unicodeConversionBp = -1;
        this.sbuf = new char[128];
        this.log = sf.log;
        this.names = sf.names;
        if (inputLength == input.length) {
            if (input.length > 0 && Character.isWhitespace(input[input.length - 1])) {
                inputLength--;
            } else {
                input = Arrays.copyOf(input, inputLength + 1);
            }
        }
        this.buf = input;
        this.buflen = inputLength;
        this.buf[this.buflen] = 26;
        this.bp = -1;
        scanChar();
    }

    protected void scanChar() {
        if (this.bp < this.buflen) {
            char[] cArr = this.buf;
            int i = this.bp + 1;
            this.bp = i;
            this.ch = cArr[i];
            if (this.ch == '\\') {
                convertUnicode();
            }
        }
    }

    protected void scanCommentChar() {
        scanChar();
        if (this.ch == '\\') {
            if (peekChar() == '\\' && !isUnicode()) {
                skipChar();
            } else {
                convertUnicode();
            }
        }
    }

    protected void putChar(char ch, boolean scan) {
        this.sbuf = ArrayUtils.ensureCapacity(this.sbuf, this.sp);
        char[] cArr = this.sbuf;
        int i = this.sp;
        this.sp = i + 1;
        cArr[i] = ch;
        if (scan) {
            scanChar();
        }
    }

    protected void putChar(char ch) {
        putChar(ch, false);
    }

    protected void putChar(boolean scan) {
        putChar(this.ch, scan);
    }

    Name name() {
        return this.names.fromChars(this.sbuf, 0, this.sp);
    }

    String chars() {
        return new String(this.sbuf, 0, this.sp);
    }

    protected void convertUnicode() {
        if (this.ch == '\\' && this.unicodeConversionBp != this.bp) {
            this.bp++;
            this.ch = this.buf[this.bp];
            if (this.ch == 'u') {
                do {
                    this.bp++;
                    this.ch = this.buf[this.bp];
                } while (this.ch == 'u');
                int limit = this.bp + 3;
                if (limit < this.buflen) {
                    int d = digit(this.bp, 16);
                    int code = d;
                    while (this.bp < limit && d >= 0) {
                        this.bp++;
                        this.ch = this.buf[this.bp];
                        d = digit(this.bp, 16);
                        code = (code << 4) + d;
                    }
                    if (d >= 0) {
                        this.ch = (char) code;
                        this.unicodeConversionBp = this.bp;
                        return;
                    }
                }
                this.log.error(this.bp, "illegal.unicode.esc", new Object[0]);
                return;
            }
            this.bp--;
            this.ch = '\\';
        }
    }

    private static boolean surrogatesSupported() {
        try {
            Character.isHighSurrogate('a');
            return true;
        } catch (NoSuchMethodError e) {
            return false;
        }
    }

    protected char scanSurrogates() {
        if (surrogatesSupported && Character.isHighSurrogate(this.ch)) {
            char high = this.ch;
            scanChar();
            if (Character.isLowSurrogate(this.ch)) {
                return high;
            }
            this.ch = high;
            return (char) 0;
        }
        return (char) 0;
    }

    protected int digit(int pos, int base) {
        char c = this.ch;
        int result = Character.digit(c, base);
        if (result >= 0 && c > 127) {
            this.log.error(pos + 1, "illegal.nonascii.digit", new Object[0]);
            this.ch = "0123456789abcdef".charAt(result);
        }
        return result;
    }

    protected boolean isUnicode() {
        return this.unicodeConversionBp == this.bp;
    }

    protected void skipChar() {
        this.bp++;
    }

    protected char peekChar() {
        return this.buf[this.bp + 1];
    }

    public char[] getRawCharacters() {
        char[] chars = new char[this.buflen];
        System.arraycopy(this.buf, 0, chars, 0, this.buflen);
        return chars;
    }

    public char[] getRawCharacters(int beginIndex, int endIndex) {
        int length = endIndex - beginIndex;
        char[] chars = new char[length];
        System.arraycopy(this.buf, beginIndex, chars, 0, length);
        return chars;
    }
}
