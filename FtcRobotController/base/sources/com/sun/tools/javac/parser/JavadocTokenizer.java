package com.sun.tools.javac.parser;

import com.sun.tools.javac.parser.JavaTokenizer;
import com.sun.tools.javac.parser.Tokens;
import com.sun.tools.javac.util.Position;
import java.nio.CharBuffer;

/* JADX INFO: loaded from: classes.dex */
public class JavadocTokenizer extends JavaTokenizer {
    protected JavadocTokenizer(ScannerFactory fac, CharBuffer buffer) {
        super(fac, buffer);
    }

    protected JavadocTokenizer(ScannerFactory fac, char[] input, int inputLength) {
        super(fac, input, inputLength);
    }

    @Override // com.sun.tools.javac.parser.JavaTokenizer
    protected Tokens.Comment processComment(int pos, int endPos, Tokens.Comment.CommentStyle style) {
        char[] buf = this.reader.getRawCharacters(pos, endPos);
        return new JavadocComment(new DocReader(this.fac, buf, buf.length, pos), style);
    }

    static class DocReader extends UnicodeReader {
        int col;
        int[] pbuf;
        int pp;
        int startPos;

        DocReader(ScannerFactory fac, char[] input, int inputLength, int startPos) {
            super(fac, input, inputLength);
            this.pbuf = new int[128];
            this.pp = 0;
            this.startPos = startPos;
        }

        @Override // com.sun.tools.javac.parser.UnicodeReader
        protected void convertUnicode() {
            if (this.ch == '\\' && this.unicodeConversionBp != this.bp) {
                this.bp++;
                this.ch = this.buf[this.bp];
                this.col++;
                if (this.ch == 'u') {
                    do {
                        this.bp++;
                        this.ch = this.buf[this.bp];
                        this.col++;
                    } while (this.ch == 'u');
                    int limit = this.bp + 3;
                    if (limit < this.buflen) {
                        int d = digit(this.bp, 16);
                        int code = d;
                        while (this.bp < limit && d >= 0) {
                            this.bp++;
                            this.ch = this.buf[this.bp];
                            this.col++;
                            d = digit(this.bp, 16);
                            code = (code << 4) + d;
                        }
                        if (d >= 0) {
                            this.ch = (char) code;
                            this.unicodeConversionBp = this.bp;
                            return;
                        }
                        return;
                    }
                    return;
                }
                this.bp--;
                this.ch = '\\';
                this.col--;
            }
        }

        @Override // com.sun.tools.javac.parser.UnicodeReader
        protected void scanCommentChar() {
            scanChar();
            if (this.ch == '\\') {
                if (peekChar() == '\\' && !isUnicode()) {
                    putChar(this.ch, false);
                    this.bp++;
                    this.col++;
                    return;
                }
                convertUnicode();
            }
        }

        @Override // com.sun.tools.javac.parser.UnicodeReader
        protected void scanChar() {
            this.bp++;
            this.ch = this.buf[this.bp];
            switch (this.ch) {
                case '\t':
                    this.col = ((this.col / 8) * 8) + 8;
                    break;
                case '\n':
                    if (this.bp == 0 || this.buf[this.bp - 1] != '\r') {
                        this.col = 0;
                    }
                    break;
                case '\r':
                    this.col = 0;
                    break;
                case '\\':
                    this.col++;
                    convertUnicode();
                    break;
                default:
                    this.col++;
                    break;
            }
        }

        @Override // com.sun.tools.javac.parser.UnicodeReader
        public void putChar(char ch, boolean scan) {
            if (this.pp == 0 || this.sp - this.pbuf[this.pp - 2] != (this.startPos + this.bp) - this.pbuf[this.pp - 1]) {
                if (this.pp + 1 >= this.pbuf.length) {
                    int[] new_pbuf = new int[this.pbuf.length * 2];
                    System.arraycopy(this.pbuf, 0, new_pbuf, 0, this.pbuf.length);
                    this.pbuf = new_pbuf;
                }
                this.pbuf[this.pp] = this.sp;
                this.pbuf[this.pp + 1] = this.startPos + this.bp;
                this.pp += 2;
            }
            super.putChar(ch, scan);
        }
    }

    protected static class JavadocComment extends JavaTokenizer.BasicComment<DocReader> {
        private String docComment;
        private int[] docPosns;

        JavadocComment(DocReader reader, Tokens.Comment.CommentStyle cs) {
            super(reader, cs);
            this.docComment = null;
            this.docPosns = null;
        }

        @Override // com.sun.tools.javac.parser.JavaTokenizer.BasicComment, com.sun.tools.javac.parser.Tokens.Comment
        public String getText() {
            if (!this.scanned && this.cs == Tokens.Comment.CommentStyle.JAVADOC) {
                scanDocComment();
            }
            return this.docComment;
        }

        @Override // com.sun.tools.javac.parser.JavaTokenizer.BasicComment, com.sun.tools.javac.parser.Tokens.Comment
        public int getSourcePos(int pos) {
            if (pos == -1) {
                return -1;
            }
            if (pos < 0 || pos > this.docComment.length()) {
                throw new StringIndexOutOfBoundsException(String.valueOf(pos));
            }
            if (this.docPosns == null) {
                return -1;
            }
            int start = 0;
            int end = this.docPosns.length;
            while (start < end - 2) {
                int index = ((start + end) / 4) * 2;
                if (this.docPosns[index] < pos) {
                    start = index;
                } else {
                    if (this.docPosns[index] == pos) {
                        return this.docPosns[index + 1];
                    }
                    end = index;
                }
            }
            return this.docPosns[start + 1] + (pos - this.docPosns[start]);
        }

        /* JADX WARN: Code restructure failed: missing block: B:64:0x0188, code lost:
        
            ((com.sun.tools.javac.parser.JavadocTokenizer.DocReader) r13.comment_reader).putChar(((com.sun.tools.javac.parser.JavadocTokenizer.DocReader) r13.comment_reader).ch, false);
            ((com.sun.tools.javac.parser.JavadocTokenizer.DocReader) r13.comment_reader).scanCommentChar();
         */
        /* JADX WARN: Code restructure failed: missing block: B:67:0x01c6, code lost:
        
            r1 = false;
         */
        /* JADX WARN: Multi-variable type inference failed */
        /* JADX WARN: Removed duplicated region for block: B:129:? A[ADDED_TO_REGION, RETURN, SYNTHETIC] */
        /* JADX WARN: Removed duplicated region for block: B:44:0x0110 A[Catch: all -> 0x022a, LOOP:3: B:44:0x0110->B:113:?, LOOP_START, TryCatch #0 {all -> 0x022a, blocks: (B:3:0x0005, B:4:0x0013, B:6:0x0023, B:8:0x002b, B:9:0x0033, B:12:0x0045, B:14:0x004d, B:21:0x0062, B:23:0x0072, B:25:0x007a, B:26:0x0083, B:28:0x008d, B:30:0x009c, B:31:0x00a4, B:33:0x00b3, B:34:0x00bf, B:36:0x00cd, B:37:0x00d3, B:39:0x00d7, B:40:0x00df, B:41:0x00ed, B:42:0x0108, B:44:0x0110, B:46:0x011f, B:51:0x0137, B:53:0x0145, B:54:0x014b, B:55:0x014e, B:66:0x01b2, B:56:0x0151, B:59:0x0161, B:60:0x0169, B:62:0x0178, B:63:0x0180, B:64:0x0188, B:65:0x019d, B:50:0x012b, B:68:0x01c9, B:70:0x01d1, B:73:0x01db, B:74:0x01e3, B:76:0x01e7, B:77:0x01ea, B:78:0x0215), top: B:92:0x0005 }] */
        /* JADX WARN: Removed duplicated region for block: B:49:0x0129  */
        /* JADX WARN: Removed duplicated region for block: B:53:0x0145 A[Catch: all -> 0x022a, TryCatch #0 {all -> 0x022a, blocks: (B:3:0x0005, B:4:0x0013, B:6:0x0023, B:8:0x002b, B:9:0x0033, B:12:0x0045, B:14:0x004d, B:21:0x0062, B:23:0x0072, B:25:0x007a, B:26:0x0083, B:28:0x008d, B:30:0x009c, B:31:0x00a4, B:33:0x00b3, B:34:0x00bf, B:36:0x00cd, B:37:0x00d3, B:39:0x00d7, B:40:0x00df, B:41:0x00ed, B:42:0x0108, B:44:0x0110, B:46:0x011f, B:51:0x0137, B:53:0x0145, B:54:0x014b, B:55:0x014e, B:66:0x01b2, B:56:0x0151, B:59:0x0161, B:60:0x0169, B:62:0x0178, B:63:0x0180, B:64:0x0188, B:65:0x019d, B:50:0x012b, B:68:0x01c9, B:70:0x01d1, B:73:0x01db, B:74:0x01e3, B:76:0x01e7, B:77:0x01ea, B:78:0x0215), top: B:92:0x0005 }] */
        /* JADX WARN: Removed duplicated region for block: B:70:0x01d1 A[Catch: all -> 0x022a, TryCatch #0 {all -> 0x022a, blocks: (B:3:0x0005, B:4:0x0013, B:6:0x0023, B:8:0x002b, B:9:0x0033, B:12:0x0045, B:14:0x004d, B:21:0x0062, B:23:0x0072, B:25:0x007a, B:26:0x0083, B:28:0x008d, B:30:0x009c, B:31:0x00a4, B:33:0x00b3, B:34:0x00bf, B:36:0x00cd, B:37:0x00d3, B:39:0x00d7, B:40:0x00df, B:41:0x00ed, B:42:0x0108, B:44:0x0110, B:46:0x011f, B:51:0x0137, B:53:0x0145, B:54:0x014b, B:55:0x014e, B:66:0x01b2, B:56:0x0151, B:59:0x0161, B:60:0x0169, B:62:0x0178, B:63:0x0180, B:64:0x0188, B:65:0x019d, B:50:0x012b, B:68:0x01c9, B:70:0x01d1, B:73:0x01db, B:74:0x01e3, B:76:0x01e7, B:77:0x01ea, B:78:0x0215), top: B:92:0x0005 }] */
        /* JADX WARN: Removed duplicated region for block: B:78:0x0215 A[Catch: all -> 0x022a, TRY_LEAVE, TryCatch #0 {all -> 0x022a, blocks: (B:3:0x0005, B:4:0x0013, B:6:0x0023, B:8:0x002b, B:9:0x0033, B:12:0x0045, B:14:0x004d, B:21:0x0062, B:23:0x0072, B:25:0x007a, B:26:0x0083, B:28:0x008d, B:30:0x009c, B:31:0x00a4, B:33:0x00b3, B:34:0x00bf, B:36:0x00cd, B:37:0x00d3, B:39:0x00d7, B:40:0x00df, B:41:0x00ed, B:42:0x0108, B:44:0x0110, B:46:0x011f, B:51:0x0137, B:53:0x0145, B:54:0x014b, B:55:0x014e, B:66:0x01b2, B:56:0x0151, B:59:0x0161, B:60:0x0169, B:62:0x0178, B:63:0x0180, B:64:0x0188, B:65:0x019d, B:50:0x012b, B:68:0x01c9, B:70:0x01d1, B:73:0x01db, B:74:0x01e3, B:76:0x01e7, B:77:0x01ea, B:78:0x0215), top: B:92:0x0005 }] */
        /* JADX WARN: Removed duplicated region for block: B:81:0x021f  */
        @Override // com.sun.tools.javac.parser.JavaTokenizer.BasicComment
        /*
            Code decompiled incorrectly, please refer to instructions dump.
            To view partially-correct code enable 'Show inconsistent code' option in preferences
        */
        protected void scanDocComment() {
            /*
                Method dump skipped, instruction units count: 620
                To view this dump change 'Code comments level' option to 'DEBUG'
            */
            throw new UnsupportedOperationException("Method not decompiled: com.sun.tools.javac.parser.JavadocTokenizer.JavadocComment.scanDocComment():void");
        }
    }

    @Override // com.sun.tools.javac.parser.JavaTokenizer
    public Position.LineMap getLineMap() {
        char[] buf = this.reader.getRawCharacters();
        return Position.makeLineMap(buf, buf.length, true);
    }
}
