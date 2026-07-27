package com.sun.source.util;

import com.sun.source.doctree.DocCommentTree;
import com.sun.source.doctree.DocTree;
import com.sun.source.tree.CompilationUnitTree;
import javax.annotation.processing.ProcessingEnvironment;
import javax.lang.model.element.Element;
import javax.tools.Diagnostic;
import javax.tools.JavaCompiler;
import jdk.Exported;

/* JADX INFO: loaded from: classes.dex */
@Exported
public abstract class DocTrees extends Trees {
    public abstract DocCommentTree getDocCommentTree(TreePath treePath);

    public abstract Element getElement(DocTreePath docTreePath);

    @Override // com.sun.source.util.Trees
    public abstract DocSourcePositions getSourcePositions();

    public abstract void printMessage(Diagnostic.Kind kind, CharSequence charSequence, DocTree docTree, DocCommentTree docCommentTree, CompilationUnitTree compilationUnitTree);

    public static DocTrees instance(JavaCompiler.CompilationTask task) {
        return (DocTrees) Trees.instance(task);
    }

    public static DocTrees instance(ProcessingEnvironment env) {
        if (!env.getClass().getName().equals("com.sun.tools.javac.processing.JavacProcessingEnvironment")) {
            throw new IllegalArgumentException();
        }
        return (DocTrees) getJavacTrees(ProcessingEnvironment.class, env);
    }
}
