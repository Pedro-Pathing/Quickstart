package com.sun.tools.javac.model;

import com.sun.tools.javac.util.Position;
import javax.tools.JavaFileObject;

/* JADX INFO: loaded from: classes.dex */
class JavacSourcePosition {
    final Position.LineMap lineMap;
    final int pos;
    final JavaFileObject sourcefile;

    JavacSourcePosition(JavaFileObject sourcefile, int pos, Position.LineMap lineMap) {
        this.sourcefile = sourcefile;
        this.pos = pos;
        this.lineMap = pos != -1 ? lineMap : null;
    }

    public JavaFileObject getFile() {
        return this.sourcefile;
    }

    public int getOffset() {
        return this.pos;
    }

    public int getLine() {
        if (this.lineMap != null) {
            return this.lineMap.getLineNumber(this.pos);
        }
        return -1;
    }

    public int getColumn() {
        if (this.lineMap != null) {
            return this.lineMap.getColumnNumber(this.pos);
        }
        return -1;
    }

    public String toString() {
        int line = getLine();
        return line > 0 ? this.sourcefile + ":" + line : this.sourcefile.toString();
    }
}
