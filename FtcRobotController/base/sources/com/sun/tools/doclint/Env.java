package com.sun.tools.doclint;

import com.sun.source.doctree.DocCommentTree;
import com.sun.source.util.DocTrees;
import com.sun.source.util.JavacTask;
import com.sun.source.util.SourcePositions;
import com.sun.source.util.TreePath;
import com.sun.tools.javac.model.JavacTypes;
import com.sun.tools.javac.tree.JCTree;
import com.sun.tools.javac.util.StringUtils;
import java.util.LinkedHashSet;
import java.util.Set;
import javax.lang.model.element.Element;
import javax.lang.model.element.ElementKind;
import javax.lang.model.element.ExecutableElement;
import javax.lang.model.element.Modifier;
import javax.lang.model.type.TypeMirror;
import javax.lang.model.util.Elements;
import javax.lang.model.util.Types;
import javax.tools.JavaCompiler;

/* JADX INFO: loaded from: classes.dex */
public class Env {
    AccessKind currAccess;
    DocCommentTree currDocComment;
    Element currElement;
    Set<? extends ExecutableElement> currOverriddenMethods;
    TreePath currPath;
    Set<String> customTags;
    Elements elements;
    TypeMirror java_lang_Error;
    TypeMirror java_lang_RuntimeException;
    TypeMirror java_lang_Throwable;
    TypeMirror java_lang_Void;
    DocTrees trees;
    Types types;
    int implicitHeaderLevel = 0;
    final Messages messages = new Messages(this);

    public enum AccessKind {
        PRIVATE,
        PACKAGE,
        PROTECTED,
        PUBLIC;

        static boolean accepts(String opt) {
            for (AccessKind g : values()) {
                if (opt.equals(StringUtils.toLowerCase(g.name()))) {
                    return true;
                }
            }
            return false;
        }

        static AccessKind of(Set<Modifier> mods) {
            if (mods.contains(Modifier.PUBLIC)) {
                return PUBLIC;
            }
            if (mods.contains(Modifier.PROTECTED)) {
                return PROTECTED;
            }
            if (mods.contains(Modifier.PRIVATE)) {
                return PRIVATE;
            }
            return PACKAGE;
        }
    }

    Env() {
    }

    void init(JavacTask task) {
        init(DocTrees.instance((JavaCompiler.CompilationTask) task), task.getElements(), task.getTypes());
    }

    void init(DocTrees trees, Elements elements, Types types) {
        this.trees = trees;
        this.elements = elements;
        this.types = types;
        this.java_lang_Error = elements.getTypeElement("java.lang.Error").asType();
        this.java_lang_RuntimeException = elements.getTypeElement("java.lang.RuntimeException").asType();
        this.java_lang_Throwable = elements.getTypeElement("java.lang.Throwable").asType();
        this.java_lang_Void = elements.getTypeElement("java.lang.Void").asType();
    }

    void setImplicitHeaders(int n) {
        this.implicitHeaderLevel = n;
    }

    void setCustomTags(String cTags) {
        this.customTags = new LinkedHashSet();
        for (String s : cTags.split(DocLint.TAGS_SEPARATOR)) {
            if (!s.isEmpty()) {
                this.customTags.add(s);
            }
        }
    }

    void setCurrent(TreePath path, DocCommentTree comment) {
        this.currPath = path;
        this.currDocComment = comment;
        this.currElement = this.trees.getElement(this.currPath);
        this.currOverriddenMethods = ((JavacTypes) this.types).getOverriddenMethods(this.currElement);
        AccessKind ak = AccessKind.PUBLIC;
        for (TreePath p = path; p != null; p = p.getParentPath()) {
            Element e = this.trees.getElement(p);
            if (e != null && e.getKind() != ElementKind.PACKAGE) {
                ak = (AccessKind) min(ak, AccessKind.of(e.getModifiers()));
            }
        }
        this.currAccess = ak;
    }

    AccessKind getAccessKind() {
        return this.currAccess;
    }

    long getPos(TreePath p) {
        return ((JCTree) p.getLeaf()).pos;
    }

    long getStartPos(TreePath p) {
        SourcePositions sp = this.trees.getSourcePositions();
        return sp.getStartPosition(p.getCompilationUnit(), p.getLeaf());
    }

    private <T extends Comparable<T>> T min(T item1, T item2) {
        return (item1 != null && (item2 == null || item1.compareTo(item2) <= 0)) ? item1 : item2;
    }
}
