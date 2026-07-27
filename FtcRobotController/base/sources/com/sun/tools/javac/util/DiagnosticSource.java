package com.sun.tools.javac.util;

import com.sun.tools.javac.file.JavacFileManager;
import com.sun.tools.javac.tree.EndPosTable;
import java.io.IOException;
import java.lang.ref.SoftReference;
import java.nio.CharBuffer;
import javax.tools.JavaFileObject;

/* JADX INFO: loaded from: classes.dex */
public class DiagnosticSource {
    public static final DiagnosticSource NO_SOURCE = new DiagnosticSource() { // from class: com.sun.tools.javac.util.DiagnosticSource.1
        @Override // com.sun.tools.javac.util.DiagnosticSource
        protected boolean findLine(int pos) {
            return false;
        }
    };
    protected char[] buf;
    protected int bufLen;
    protected EndPosTable endPosTable;
    protected JavaFileObject fileObject;
    protected int line;
    protected int lineStart;
    protected AbstractLog log;
    protected SoftReference<char[]> refBuf;

    public DiagnosticSource(JavaFileObject fo, AbstractLog log) {
        this.fileObject = fo;
        this.log = log;
    }

    private DiagnosticSource() {
    }

    public JavaFileObject getFile() {
        return this.fileObject;
    }

    public int getLineNumber(int pos) {
        try {
            if (findLine(pos)) {
                return this.line;
            }
            this.buf = null;
            return 0;
        } finally {
            this.buf = null;
        }
    }

    public int getColumnNumber(int pos, boolean expandTabs) {
        try {
            if (!findLine(pos)) {
                return 0;
            }
            int column = 0;
            for (int bp = this.lineStart; bp < pos; bp++) {
                if (bp >= this.bufLen) {
                    return 0;
                }
                column = (this.buf[bp] == '\t' && expandTabs) ? ((column / 8) * 8) + 8 : column + 1;
            }
            return column + 1;
        } finally {
            this.buf = null;
        }
    }

    public String getLine(int pos) {
        try {
            if (!findLine(pos)) {
                return null;
            }
            int lineEnd = this.lineStart;
            while (lineEnd < this.bufLen && this.buf[lineEnd] != '\r' && this.buf[lineEnd] != '\n') {
                lineEnd++;
            }
            if (lineEnd - this.lineStart == 0) {
                return null;
            }
            return new String(this.buf, this.lineStart, lineEnd - this.lineStart);
        } finally {
            this.buf = null;
        }
    }

    public EndPosTable getEndPosTable() {
        return this.endPosTable;
    }

    public void setEndPosTable(EndPosTable t) {
        if (this.endPosTable != null && this.endPosTable != t) {
            throw new IllegalStateException("endPosTable already set");
        }
        this.endPosTable = t;
    }

    protected boolean findLine(int pos) {
        if (pos == -1) {
            return false;
        }
        try {
            if (this.buf == null && this.refBuf != null) {
                this.buf = this.refBuf.get();
            }
            if (this.buf == null) {
                this.buf = initBuf(this.fileObject);
                this.lineStart = 0;
                this.line = 1;
            } else if (this.lineStart > pos) {
                this.lineStart = 0;
                this.line = 1;
            }
            int bp = this.lineStart;
            while (bp < this.bufLen && bp < pos) {
                int bp2 = bp + 1;
                switch (this.buf[bp]) {
                    case '\n':
                        this.line++;
                        this.lineStart = bp2;
                        break;
                    case '\r':
                        if (bp2 < this.bufLen && this.buf[bp2] == '\n') {
                            bp2++;
                        }
                        this.line++;
                        this.lineStart = bp2;
                        bp = bp2;
                        continue;
                }
                bp = bp2;
            }
            return bp <= this.bufLen;
        } catch (IOException e) {
            this.log.directError("source.unavailable", new Object[0]);
            this.buf = new char[0];
            return false;
        }
    }

    protected char[] initBuf(JavaFileObject fileObject) throws IOException {
        char[] buf;
        CharSequence cs = fileObject.getCharContent(true);
        if (cs instanceof CharBuffer) {
            CharBuffer cb = (CharBuffer) cs;
            buf = JavacFileManager.toArray(cb);
            this.bufLen = cb.limit();
        } else {
            buf = cs.toString().toCharArray();
            this.bufLen = buf.length;
        }
        this.refBuf = new SoftReference<>(buf);
        return buf;
    }
}
