package com.sun.tools.javac.parser;

import com.sun.tools.javac.parser.Tokens;
import com.sun.tools.javac.util.Position;

/* JADX INFO: loaded from: classes.dex */
public interface Lexer {
    int errPos();

    void errPos(int i);

    Position.LineMap getLineMap();

    void nextToken();

    Tokens.Token prevToken();

    Tokens.Token split();

    Tokens.Token token();

    Tokens.Token token(int i);
}
