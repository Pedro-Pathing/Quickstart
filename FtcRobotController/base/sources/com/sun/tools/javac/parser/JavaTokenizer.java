package com.sun.tools.javac.parser;

import com.sun.tools.javac.code.Source;
import com.sun.tools.javac.jvm.ByteCodes;
import com.sun.tools.javac.parser.Tokens;
import com.sun.tools.javac.util.Assert;
import com.sun.tools.javac.util.List;
import com.sun.tools.javac.util.Log;
import com.sun.tools.javac.util.Name;
import com.sun.tools.javac.util.Position;
import java.nio.CharBuffer;

/* JADX INFO: loaded from: classes.dex */
public class JavaTokenizer {
    private static final boolean hexFloatsWork = hexFloatsWork();
    private static final boolean scannerDebug = false;
    private boolean allowBinaryLiterals;
    private boolean allowHexFloats;
    private boolean allowUnderscoresInLiterals;
    protected int errPos;
    protected ScannerFactory fac;
    private final Log log;
    protected Name name;
    protected int radix;
    protected UnicodeReader reader;
    private Source source;
    protected Tokens.TokenKind tk;
    private final Tokens tokens;

    private static boolean hexFloatsWork() {
        try {
            Float.valueOf("0x1.0p1");
            return true;
        } catch (NumberFormatException e) {
            return false;
        }
    }

    protected JavaTokenizer(ScannerFactory fac, CharBuffer buf) {
        this(fac, new UnicodeReader(fac, buf));
    }

    protected JavaTokenizer(ScannerFactory fac, char[] buf, int inputLength) {
        this(fac, new UnicodeReader(fac, buf, inputLength));
    }

    protected JavaTokenizer(ScannerFactory fac, UnicodeReader reader) {
        this.errPos = -1;
        this.fac = fac;
        this.log = fac.log;
        this.tokens = fac.tokens;
        this.source = fac.source;
        this.reader = reader;
        this.allowBinaryLiterals = this.source.allowBinaryLiterals();
        this.allowHexFloats = this.source.allowHexFloats();
        this.allowUnderscoresInLiterals = this.source.allowUnderscoresInLiterals();
    }

    protected void lexError(int pos, String key, Object... args) {
        this.log.error(pos, key, args);
        this.tk = Tokens.TokenKind.ERROR;
        this.errPos = pos;
    }

    private void scanLitChar(int pos) {
        if (this.reader.ch == '\\') {
            if (this.reader.peekChar() == '\\' && !this.reader.isUnicode()) {
                this.reader.skipChar();
                this.reader.putChar('\\', true);
                return;
            }
            this.reader.scanChar();
            switch (this.reader.ch) {
                case '\"':
                    this.reader.putChar('\"', true);
                    break;
                case '\'':
                    this.reader.putChar('\'', true);
                    break;
                case '0':
                case '1':
                case '2':
                case '3':
                case '4':
                case '5':
                case '6':
                case '7':
                    char leadch = this.reader.ch;
                    int oct = this.reader.digit(pos, 8);
                    this.reader.scanChar();
                    if ('0' <= this.reader.ch && this.reader.ch <= '7') {
                        oct = (oct * 8) + this.reader.digit(pos, 8);
                        this.reader.scanChar();
                        if (leadch <= '3' && '0' <= this.reader.ch && this.reader.ch <= '7') {
                            oct = (oct * 8) + this.reader.digit(pos, 8);
                            this.reader.scanChar();
                        }
                    }
                    this.reader.putChar((char) oct);
                    break;
                case '\\':
                    this.reader.putChar('\\', true);
                    break;
                case 'b':
                    this.reader.putChar('\b', true);
                    break;
                case 'f':
                    this.reader.putChar('\f', true);
                    break;
                case 'n':
                    this.reader.putChar('\n', true);
                    break;
                case ByteCodes.fmod /* 114 */:
                    this.reader.putChar('\r', true);
                    break;
                case 't':
                    this.reader.putChar('\t', true);
                    break;
                default:
                    lexError(this.reader.bp, "illegal.esc.char", new Object[0]);
                    break;
            }
        }
        if (this.reader.bp != this.reader.buflen) {
            this.reader.putChar(true);
        }
    }

    private void scanDigits(int pos, int digitRadix) {
        char saveCh;
        int savePos;
        while (true) {
            if (this.reader.ch != '_') {
                this.reader.putChar(false);
            } else if (!this.allowUnderscoresInLiterals) {
                lexError(pos, "unsupported.underscore.lit", this.source.name);
                this.allowUnderscoresInLiterals = true;
            }
            saveCh = this.reader.ch;
            savePos = this.reader.bp;
            this.reader.scanChar();
            if (this.reader.digit(pos, digitRadix) < 0 && this.reader.ch != '_') {
                break;
            }
        }
        if (saveCh == '_') {
            lexError(savePos, "illegal.underscore", new Object[0]);
        }
    }

    private void scanHexExponentAndSuffix(int pos) {
        if (this.reader.ch == 'p' || this.reader.ch == 'P') {
            this.reader.putChar(true);
            skipIllegalUnderscores();
            if (this.reader.ch == '+' || this.reader.ch == '-') {
                this.reader.putChar(true);
            }
            skipIllegalUnderscores();
            if ('0' <= this.reader.ch && this.reader.ch <= '9') {
                scanDigits(pos, 10);
                if (!this.allowHexFloats) {
                    lexError(pos, "unsupported.fp.lit", this.source.name);
                    this.allowHexFloats = true;
                } else if (!hexFloatsWork) {
                    lexError(pos, "unsupported.cross.fp.lit", new Object[0]);
                }
            } else {
                lexError(pos, "malformed.fp.lit", new Object[0]);
            }
        } else {
            lexError(pos, "malformed.fp.lit", new Object[0]);
        }
        if (this.reader.ch == 'f' || this.reader.ch == 'F') {
            this.reader.putChar(true);
            this.tk = Tokens.TokenKind.FLOATLITERAL;
            this.radix = 16;
        } else {
            if (this.reader.ch == 'd' || this.reader.ch == 'D') {
                this.reader.putChar(true);
            }
            this.tk = Tokens.TokenKind.DOUBLELITERAL;
            this.radix = 16;
        }
    }

    private void scanFraction(int pos) {
        skipIllegalUnderscores();
        if ('0' <= this.reader.ch && this.reader.ch <= '9') {
            scanDigits(pos, 10);
        }
        int sp1 = this.reader.sp;
        if (this.reader.ch == 'e' || this.reader.ch == 'E') {
            this.reader.putChar(true);
            skipIllegalUnderscores();
            if (this.reader.ch == '+' || this.reader.ch == '-') {
                this.reader.putChar(true);
            }
            skipIllegalUnderscores();
            if ('0' <= this.reader.ch && this.reader.ch <= '9') {
                scanDigits(pos, 10);
            } else {
                lexError(pos, "malformed.fp.lit", new Object[0]);
                this.reader.sp = sp1;
            }
        }
    }

    private void scanFractionAndSuffix(int pos) {
        this.radix = 10;
        scanFraction(pos);
        if (this.reader.ch == 'f' || this.reader.ch == 'F') {
            this.reader.putChar(true);
            this.tk = Tokens.TokenKind.FLOATLITERAL;
        } else {
            if (this.reader.ch == 'd' || this.reader.ch == 'D') {
                this.reader.putChar(true);
            }
            this.tk = Tokens.TokenKind.DOUBLELITERAL;
        }
    }

    private void scanHexFractionAndSuffix(int pos, boolean seendigit) {
        this.radix = 16;
        Assert.check(this.reader.ch == '.');
        this.reader.putChar(true);
        skipIllegalUnderscores();
        if (this.reader.digit(pos, 16) >= 0) {
            seendigit = true;
            scanDigits(pos, 16);
        }
        if (!seendigit) {
            lexError(pos, "invalid.hex.number", new Object[0]);
        } else {
            scanHexExponentAndSuffix(pos);
        }
    }

    private void skipIllegalUnderscores() {
        if (this.reader.ch == '_') {
            lexError(this.reader.bp, "illegal.underscore", new Object[0]);
            while (this.reader.ch == '_') {
                this.reader.scanChar();
            }
        }
    }

    private void scanNumber(int pos, int radix) {
        this.radix = radix;
        int digitRadix = radix == 8 ? 10 : radix;
        boolean seendigit = false;
        if (this.reader.digit(pos, digitRadix) >= 0) {
            seendigit = true;
            scanDigits(pos, digitRadix);
        }
        if (radix == 16 && this.reader.ch == '.') {
            scanHexFractionAndSuffix(pos, seendigit);
            return;
        }
        if (seendigit && radix == 16 && (this.reader.ch == 'p' || this.reader.ch == 'P')) {
            scanHexExponentAndSuffix(pos);
            return;
        }
        if (digitRadix == 10 && this.reader.ch == '.') {
            this.reader.putChar(true);
            scanFractionAndSuffix(pos);
            return;
        }
        if (digitRadix == 10 && (this.reader.ch == 'e' || this.reader.ch == 'E' || this.reader.ch == 'f' || this.reader.ch == 'F' || this.reader.ch == 'd' || this.reader.ch == 'D')) {
            scanFractionAndSuffix(pos);
        } else if (this.reader.ch == 'l' || this.reader.ch == 'L') {
            this.reader.scanChar();
            this.tk = Tokens.TokenKind.LONGLITERAL;
        } else {
            this.tk = Tokens.TokenKind.INTLITERAL;
        }
    }

    private void scanIdent() {
        boolean isJavaIdentifierPart;
        this.reader.putChar(true);
        while (true) {
            switch (this.reader.ch) {
                case 0:
                case 1:
                case 2:
                case 3:
                case 4:
                case 5:
                case 6:
                case 7:
                case '\b':
                case 14:
                case 15:
                case 16:
                case 17:
                case 18:
                case 19:
                case 20:
                case 21:
                case 22:
                case 23:
                case 24:
                case 25:
                case 27:
                case 127:
                    this.reader.scanChar();
                    continue;
                case '\t':
                case '\n':
                case 11:
                case '\f':
                case '\r':
                case 28:
                case 29:
                case 30:
                case 31:
                case ' ':
                case '!':
                case '\"':
                case '#':
                case '%':
                case '&':
                case '\'':
                case '(':
                case ')':
                case '*':
                case '+':
                case ',':
                case '-':
                case '.':
                case '/':
                case ':':
                case ';':
                case '<':
                case '=':
                case '>':
                case '?':
                case '@':
                case '[':
                case '\\':
                case ']':
                case '^':
                case '`':
                case '{':
                case '|':
                case '}':
                case '~':
                default:
                    if (this.reader.ch < 128) {
                        isJavaIdentifierPart = false;
                    } else if (Character.isIdentifierIgnorable(this.reader.ch)) {
                        this.reader.scanChar();
                        break;
                    } else {
                        char high = this.reader.scanSurrogates();
                        if (high != 0) {
                            this.reader.putChar(high);
                            isJavaIdentifierPart = Character.isJavaIdentifierPart(Character.toCodePoint(high, this.reader.ch));
                        } else {
                            isJavaIdentifierPart = Character.isJavaIdentifierPart(this.reader.ch);
                        }
                    }
                    if (!isJavaIdentifierPart) {
                        this.name = this.reader.name();
                        this.tk = this.tokens.lookupKind(this.name);
                        return;
                    }
                    break;
                case 26:
                    if (this.reader.bp >= this.reader.buflen) {
                        this.name = this.reader.name();
                        this.tk = this.tokens.lookupKind(this.name);
                        return;
                    } else {
                        this.reader.scanChar();
                        continue;
                    }
                    break;
                case '$':
                case '0':
                case '1':
                case '2':
                case '3':
                case '4':
                case '5':
                case '6':
                case '7':
                case '8':
                case '9':
                case 'A':
                case 'B':
                case 'C':
                case 'D':
                case 'E':
                case 'F':
                case 'G':
                case 'H':
                case 'I':
                case 'J':
                case 'K':
                case 'L':
                case 'M':
                case 'N':
                case 'O':
                case 'P':
                case 'Q':
                case 'R':
                case 'S':
                case 'T':
                case 'U':
                case 'V':
                case 'W':
                case 'X':
                case 'Y':
                case 'Z':
                case '_':
                case 'a':
                case 'b':
                case 'c':
                case 'd':
                case 'e':
                case 'f':
                case 'g':
                case 'h':
                case 'i':
                case 'j':
                case 'k':
                case 'l':
                case 'm':
                case 'n':
                case 'o':
                case 'p':
                case ByteCodes.lmod /* 113 */:
                case ByteCodes.fmod /* 114 */:
                case 's':
                case 't':
                case 'u':
                case 'v':
                case 'w':
                case 'x':
                case 'y':
                case 'z':
                    break;
            }
            this.reader.putChar(true);
        }
    }

    private boolean isSpecial(char ch) {
        switch (ch) {
            case '!':
            case '%':
            case '&':
            case '*':
            case '+':
            case '-':
            case ':':
            case '<':
            case '=':
            case '>':
            case '?':
            case '@':
            case '^':
            case '|':
            case '~':
                return true;
            default:
                return false;
        }
    }

    private void scanOperator() {
        do {
            this.reader.putChar(false);
            Name newname = this.reader.name();
            Tokens.TokenKind tk1 = this.tokens.lookupKind(newname);
            if (tk1 == Tokens.TokenKind.IDENTIFIER) {
                UnicodeReader unicodeReader = this.reader;
                unicodeReader.sp--;
                return;
            } else {
                this.tk = tk1;
                this.reader.scanChar();
            }
        } while (isSpecial(this.reader.ch));
    }

    /* JADX WARN: Removed duplicated region for block: B:200:0x014f A[SYNTHETIC] */
    /* JADX WARN: Removed duplicated region for block: B:201:0x01c3 A[SYNTHETIC] */
    /* JADX WARN: Removed duplicated region for block: B:210:0x000e A[SYNTHETIC] */
    /* JADX WARN: Removed duplicated region for block: B:75:0x01af A[Catch: all -> 0x0438, TryCatch #0 {, blocks: (B:3:0x000e, B:4:0x0022, B:5:0x0025, B:137:0x0335, B:139:0x033d, B:165:0x03e1, B:166:0x03f2, B:167:0x03f5, B:176:0x0434, B:177:0x0437, B:168:0x03f8, B:170:0x040c, B:172:0x041e, B:174:0x042c, B:140:0x0342, B:148:0x0372, B:149:0x0376, B:151:0x0380, B:153:0x0388, B:156:0x0394, B:158:0x039a, B:160:0x03a2, B:162:0x03c7, B:161:0x03b5, B:163:0x03d7, B:143:0x034c, B:145:0x0354, B:146:0x0367, B:6:0x0029, B:7:0x0034, B:8:0x003f, B:9:0x004a, B:10:0x0055, B:11:0x0060, B:12:0x0065, B:14:0x0072, B:17:0x007c, B:19:0x0084, B:22:0x008d, B:24:0x009a, B:25:0x009e, B:27:0x00a9, B:29:0x00b1, B:30:0x00b8, B:31:0x00bf, B:33:0x00c3, B:34:0x00d3, B:36:0x00e4, B:37:0x00ed, B:38:0x00f2, B:40:0x0100, B:41:0x0105, B:43:0x010f, B:44:0x0118, B:45:0x011d, B:47:0x012a, B:49:0x0135, B:51:0x013b, B:53:0x0145, B:55:0x014f, B:56:0x0160, B:58:0x0168, B:60:0x0174, B:65:0x0187, B:67:0x0191, B:69:0x0197, B:72:0x01a3, B:73:0x01a9, B:75:0x01af, B:76:0x01c3, B:63:0x0183, B:77:0x01cc, B:79:0x01d4, B:80:0x01df, B:81:0x01e5, B:83:0x01f0, B:85:0x01f8, B:86:0x0202, B:88:0x0208, B:90:0x021c, B:91:0x022b, B:93:0x0234, B:94:0x023a, B:95:0x0245, B:96:0x0250, B:97:0x025b, B:99:0x0268, B:100:0x0271, B:102:0x0277, B:105:0x0284, B:107:0x0291, B:108:0x029b, B:104:0x027d, B:110:0x02a4, B:111:0x02a9, B:112:0x02ae, B:114:0x02b6, B:116:0x02bc, B:118:0x02c2, B:120:0x02cc, B:121:0x02d0, B:123:0x02d6, B:124:0x02e1, B:125:0x02ea, B:127:0x02f5, B:128:0x02fa, B:129:0x0303, B:130:0x0311, B:132:0x031c, B:134:0x0324, B:136:0x032c), top: B:180:0x000e }] */
    /*
        Code decompiled incorrectly, please refer to instructions dump.
        To view partially-correct code enable 'Show inconsistent code' option in preferences
    */
    public com.sun.tools.javac.parser.Tokens.Token readToken() {
        /*
            Method dump skipped, instruction units count: 1332
            To view this dump change 'Code comments level' option to 'DEBUG'
        */
        throw new UnsupportedOperationException("Method not decompiled: com.sun.tools.javac.parser.JavaTokenizer.readToken():com.sun.tools.javac.parser.Tokens$Token");
    }

    List<Tokens.Comment> addComment(List<Tokens.Comment> comments, Tokens.Comment comment) {
        if (comments == null) {
            return List.of(comment);
        }
        return comments.prepend(comment);
    }

    public int errPos() {
        return this.errPos;
    }

    public void errPos(int pos) {
        this.errPos = pos;
    }

    protected Tokens.Comment processComment(int pos, int endPos, Tokens.Comment.CommentStyle style) {
        char[] buf = this.reader.getRawCharacters(pos, endPos);
        return new BasicComment(new UnicodeReader(this.fac, buf, buf.length), style);
    }

    protected void processWhiteSpace(int pos, int endPos) {
    }

    protected void processLineTerminator(int pos, int endPos) {
    }

    public Position.LineMap getLineMap() {
        return Position.makeLineMap(this.reader.getRawCharacters(), this.reader.buflen, false);
    }

    protected static class BasicComment<U extends UnicodeReader> implements Tokens.Comment {
        U comment_reader;
        Tokens.Comment.CommentStyle cs;
        protected boolean deprecatedFlag = false;
        protected boolean scanned = false;

        protected BasicComment(U comment_reader, Tokens.Comment.CommentStyle cs) {
            this.comment_reader = comment_reader;
            this.cs = cs;
        }

        @Override // com.sun.tools.javac.parser.Tokens.Comment
        public String getText() {
            return null;
        }

        @Override // com.sun.tools.javac.parser.Tokens.Comment
        public int getSourcePos(int pos) {
            return -1;
        }

        @Override // com.sun.tools.javac.parser.Tokens.Comment
        public Tokens.Comment.CommentStyle getStyle() {
            return this.cs;
        }

        @Override // com.sun.tools.javac.parser.Tokens.Comment
        public boolean isDeprecated() {
            if (!this.scanned && this.cs == Tokens.Comment.CommentStyle.JAVADOC) {
                scanDocComment();
            }
            return this.deprecatedFlag;
        }

        /* JADX WARN: Code restructure failed: missing block: B:67:0x0119, code lost:
        
            r9.comment_reader.scanCommentChar();
         */
        /*
            Code decompiled incorrectly, please refer to instructions dump.
            To view partially-correct code enable 'Show inconsistent code' option in preferences
        */
        protected void scanDocComment() {
            /*
                Method dump skipped, instruction units count: 314
                To view this dump change 'Code comments level' option to 'DEBUG'
            */
            throw new UnsupportedOperationException("Method not decompiled: com.sun.tools.javac.parser.JavaTokenizer.BasicComment.scanDocComment():void");
        }
    }
}
