package com.sun.tools.javac.parser;

import com.sun.tools.javac.code.Source;
import com.sun.tools.javac.tree.DocTreeMaker;
import com.sun.tools.javac.tree.TreeMaker;
import com.sun.tools.javac.util.Context;
import com.sun.tools.javac.util.Log;
import com.sun.tools.javac.util.Names;
import com.sun.tools.javac.util.Options;
import java.util.Locale;

/* JADX INFO: loaded from: classes.dex */
public class ParserFactory {
    protected static final Context.Key<ParserFactory> parserFactoryKey = new Context.Key<>();
    final TreeMaker F;
    final DocTreeMaker docTreeMaker;
    final Locale locale;
    final Log log;
    final Names names;
    final Options options;
    final ScannerFactory scannerFactory;
    final Source source;
    final Tokens tokens;

    public static ParserFactory instance(Context context) {
        ParserFactory instance = (ParserFactory) context.get(parserFactoryKey);
        if (instance == null) {
            return new ParserFactory(context);
        }
        return instance;
    }

    protected ParserFactory(Context context) {
        context.put(parserFactoryKey, this);
        this.F = TreeMaker.instance(context);
        this.docTreeMaker = DocTreeMaker.instance(context);
        this.log = Log.instance(context);
        this.names = Names.instance(context);
        this.tokens = Tokens.instance(context);
        this.source = Source.instance(context);
        this.options = Options.instance(context);
        this.scannerFactory = ScannerFactory.instance(context);
        this.locale = (Locale) context.get(Locale.class);
    }

    public JavacParser newParser(CharSequence input, boolean keepDocComments, boolean keepEndPos, boolean keepLineMap) {
        Lexer lexer = this.scannerFactory.newScanner(input, keepDocComments);
        return new JavacParser(this, lexer, keepDocComments, keepLineMap, keepEndPos);
    }
}
