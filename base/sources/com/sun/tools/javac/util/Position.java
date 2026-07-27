package com.sun.tools.javac.util;

import java.util.BitSet;

/* JADX INFO: loaded from: classes.dex */
public class Position {
    public static final int FIRSTCOLUMN = 1;
    public static final int FIRSTLINE = 1;
    public static final int FIRSTPOS = 0;
    public static final int LINESHIFT = 10;
    public static final int MAXCOLUMN = 1023;
    public static final int MAXLINE = 4194303;
    public static final int MAXPOS = Integer.MAX_VALUE;
    public static final int NOPOS = -1;

    public interface LineMap extends com.sun.source.tree.LineMap {
        int getColumnNumber(int i);

        int getLineNumber(int i);

        int getPosition(int i, int i2);

        int getStartPosition(int i);
    }

    private Position() {
    }

    public static LineMap makeLineMap(char[] src, int max, boolean expandTabs) {
        LineMapImpl lineMap = expandTabs ? new LineTabMapImpl(max) : new LineMapImpl();
        lineMap.build(src, max);
        return lineMap;
    }

    public static int encodePosition(int line, int col) {
        if (line < 1) {
            throw new IllegalArgumentException("line must be greater than 0");
        }
        if (col < 1) {
            throw new IllegalArgumentException("column must be greater than 0");
        }
        if (line > 4194303 || col > 1023) {
            return -1;
        }
        return (line << 10) + col;
    }

    static class LineMapImpl implements LineMap {
        protected int[] startPosition;
        private int lastPosition = 0;
        private int lastLine = 1;

        protected LineMapImpl() {
        }

        protected void build(char[] src, int max) {
            int c = 0;
            int i = 0;
            int[] linebuf = new int[max];
            while (i < max) {
                int c2 = c + 1;
                linebuf[c] = i;
                do {
                    char ch = src[i];
                    if (ch == '\r' || ch == '\n') {
                        if (ch == '\r' && i + 1 < max && src[i + 1] == '\n') {
                            i += 2;
                            c = c2;
                        } else {
                            i++;
                            c = c2;
                        }
                    } else {
                        if (ch == '\t') {
                            setTabPosition(i);
                        }
                        i++;
                    }
                } while (i < max);
                c = c2;
            }
            this.startPosition = new int[c];
            System.arraycopy(linebuf, 0, this.startPosition, 0, c);
        }

        @Override // com.sun.tools.javac.util.Position.LineMap
        public int getStartPosition(int line) {
            return this.startPosition[line - 1];
        }

        @Override // com.sun.source.tree.LineMap
        public long getStartPosition(long line) {
            return getStartPosition(longToInt(line));
        }

        @Override // com.sun.tools.javac.util.Position.LineMap
        public int getPosition(int line, int column) {
            return (this.startPosition[line - 1] + column) - 1;
        }

        @Override // com.sun.source.tree.LineMap
        public long getPosition(long line, long column) {
            return getPosition(longToInt(line), longToInt(column));
        }

        @Override // com.sun.tools.javac.util.Position.LineMap
        public int getLineNumber(int pos) {
            if (pos == this.lastPosition) {
                return this.lastLine;
            }
            this.lastPosition = pos;
            int low = 0;
            int high = this.startPosition.length - 1;
            while (low <= high) {
                int mid = (low + high) >> 1;
                int midVal = this.startPosition[mid];
                if (midVal < pos) {
                    low = mid + 1;
                } else if (midVal > pos) {
                    high = mid - 1;
                } else {
                    this.lastLine = mid + 1;
                    return this.lastLine;
                }
            }
            this.lastLine = low;
            return this.lastLine;
        }

        @Override // com.sun.source.tree.LineMap
        public long getLineNumber(long pos) {
            return getLineNumber(longToInt(pos));
        }

        @Override // com.sun.tools.javac.util.Position.LineMap
        public int getColumnNumber(int pos) {
            return (pos - this.startPosition[getLineNumber(pos) - 1]) + 1;
        }

        @Override // com.sun.source.tree.LineMap
        public long getColumnNumber(long pos) {
            return getColumnNumber(longToInt(pos));
        }

        private static int longToInt(long longValue) {
            int intValue = (int) longValue;
            if (intValue != longValue) {
                throw new IndexOutOfBoundsException();
            }
            return intValue;
        }

        protected void setTabPosition(int offset) {
        }
    }

    public static class LineTabMapImpl extends LineMapImpl {
        private BitSet tabMap;

        @Override // com.sun.tools.javac.util.Position.LineMapImpl, com.sun.source.tree.LineMap
        public /* bridge */ /* synthetic */ long getColumnNumber(long j) {
            return super.getColumnNumber(j);
        }

        @Override // com.sun.tools.javac.util.Position.LineMapImpl, com.sun.tools.javac.util.Position.LineMap
        public /* bridge */ /* synthetic */ int getLineNumber(int i) {
            return super.getLineNumber(i);
        }

        @Override // com.sun.tools.javac.util.Position.LineMapImpl, com.sun.source.tree.LineMap
        public /* bridge */ /* synthetic */ long getLineNumber(long j) {
            return super.getLineNumber(j);
        }

        @Override // com.sun.tools.javac.util.Position.LineMapImpl, com.sun.source.tree.LineMap
        public /* bridge */ /* synthetic */ long getPosition(long j, long j2) {
            return super.getPosition(j, j2);
        }

        @Override // com.sun.tools.javac.util.Position.LineMapImpl, com.sun.tools.javac.util.Position.LineMap
        public /* bridge */ /* synthetic */ int getStartPosition(int i) {
            return super.getStartPosition(i);
        }

        @Override // com.sun.tools.javac.util.Position.LineMapImpl, com.sun.source.tree.LineMap
        public /* bridge */ /* synthetic */ long getStartPosition(long j) {
            return super.getStartPosition(j);
        }

        public LineTabMapImpl(int max) {
            this.tabMap = new BitSet(max);
        }

        @Override // com.sun.tools.javac.util.Position.LineMapImpl
        protected void setTabPosition(int offset) {
            this.tabMap.set(offset);
        }

        @Override // com.sun.tools.javac.util.Position.LineMapImpl, com.sun.tools.javac.util.Position.LineMap
        public int getColumnNumber(int pos) {
            int lineStart = this.startPosition[getLineNumber(pos) - 1];
            int column = 0;
            for (int bp = lineStart; bp < pos; bp++) {
                if (this.tabMap.get(bp)) {
                    column = ((column / 8) * 8) + 8;
                } else {
                    column++;
                }
            }
            int bp2 = column + 1;
            return bp2;
        }

        @Override // com.sun.tools.javac.util.Position.LineMapImpl, com.sun.tools.javac.util.Position.LineMap
        public int getPosition(int line, int column) {
            int pos = this.startPosition[line - 1];
            int column2 = column - 1;
            int col = 0;
            while (col < column2) {
                pos++;
                if (this.tabMap.get(pos)) {
                    col = ((col / 8) * 8) + 8;
                } else {
                    col++;
                }
            }
            return pos;
        }
    }
}
