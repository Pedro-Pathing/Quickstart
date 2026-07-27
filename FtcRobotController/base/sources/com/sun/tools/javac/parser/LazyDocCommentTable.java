package com.sun.tools.javac.parser;

import com.sun.tools.javac.parser.Tokens;
import com.sun.tools.javac.tree.DCTree;
import com.sun.tools.javac.tree.DocCommentTable;
import com.sun.tools.javac.tree.JCTree;
import com.sun.tools.javac.util.DiagnosticSource;
import java.util.HashMap;
import java.util.Map;

/* JADX INFO: loaded from: classes.dex */
public class LazyDocCommentTable implements DocCommentTable {
    DiagnosticSource diagSource;
    ParserFactory fac;
    Map<JCTree, Entry> table = new HashMap();

    private static class Entry {
        final Tokens.Comment comment;
        DCTree.DCDocComment tree;

        Entry(Tokens.Comment c) {
            this.comment = c;
        }
    }

    LazyDocCommentTable(ParserFactory fac) {
        this.fac = fac;
        this.diagSource = fac.log.currentSource();
    }

    @Override // com.sun.tools.javac.tree.DocCommentTable
    public boolean hasComment(JCTree tree) {
        return this.table.containsKey(tree);
    }

    @Override // com.sun.tools.javac.tree.DocCommentTable
    public Tokens.Comment getComment(JCTree tree) {
        Entry e = this.table.get(tree);
        if (e == null) {
            return null;
        }
        return e.comment;
    }

    @Override // com.sun.tools.javac.tree.DocCommentTable
    public String getCommentText(JCTree tree) {
        Tokens.Comment c = getComment(tree);
        if (c == null) {
            return null;
        }
        return c.getText();
    }

    @Override // com.sun.tools.javac.tree.DocCommentTable
    public DCTree.DCDocComment getCommentTree(JCTree tree) {
        Entry e = this.table.get(tree);
        if (e == null) {
            return null;
        }
        if (e.tree == null) {
            e.tree = new DocCommentParser(this.fac, this.diagSource, e.comment).parse();
        }
        return e.tree;
    }

    @Override // com.sun.tools.javac.tree.DocCommentTable
    public void putComment(JCTree tree, Tokens.Comment c) {
        this.table.put(tree, new Entry(c));
    }
}
