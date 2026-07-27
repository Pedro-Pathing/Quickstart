package com.sun.source.util;

import com.sun.source.tree.CatchTree;
import com.sun.source.tree.ClassTree;
import com.sun.source.tree.CompilationUnitTree;
import com.sun.source.tree.MethodTree;
import com.sun.source.tree.Scope;
import com.sun.source.tree.Tree;
import java.lang.reflect.Method;
import javax.annotation.processing.ProcessingEnvironment;
import javax.lang.model.element.AnnotationMirror;
import javax.lang.model.element.AnnotationValue;
import javax.lang.model.element.Element;
import javax.lang.model.element.ExecutableElement;
import javax.lang.model.element.TypeElement;
import javax.lang.model.type.DeclaredType;
import javax.lang.model.type.ErrorType;
import javax.lang.model.type.TypeMirror;
import javax.tools.Diagnostic;
import javax.tools.JavaCompiler;
import jdk.Exported;

/* JADX INFO: loaded from: classes.dex */
@Exported
public abstract class Trees {
    public abstract String getDocComment(TreePath treePath);

    public abstract Element getElement(TreePath treePath);

    public abstract TypeMirror getLub(CatchTree catchTree);

    public abstract TypeMirror getOriginalType(ErrorType errorType);

    public abstract TreePath getPath(CompilationUnitTree compilationUnitTree, Tree tree);

    public abstract TreePath getPath(Element element);

    public abstract TreePath getPath(Element element, AnnotationMirror annotationMirror);

    public abstract TreePath getPath(Element element, AnnotationMirror annotationMirror, AnnotationValue annotationValue);

    public abstract Scope getScope(TreePath treePath);

    public abstract SourcePositions getSourcePositions();

    public abstract ClassTree getTree(TypeElement typeElement);

    public abstract MethodTree getTree(ExecutableElement executableElement);

    public abstract Tree getTree(Element element);

    public abstract Tree getTree(Element element, AnnotationMirror annotationMirror);

    public abstract Tree getTree(Element element, AnnotationMirror annotationMirror, AnnotationValue annotationValue);

    public abstract TypeMirror getTypeMirror(TreePath treePath);

    public abstract boolean isAccessible(Scope scope, Element element, DeclaredType declaredType);

    public abstract boolean isAccessible(Scope scope, TypeElement typeElement);

    public abstract void printMessage(Diagnostic.Kind kind, CharSequence charSequence, Tree tree, CompilationUnitTree compilationUnitTree);

    public static Trees instance(JavaCompiler.CompilationTask task) {
        String taskClassName = task.getClass().getName();
        if (!taskClassName.equals("com.sun.tools.javac.api.JavacTaskImpl") && !taskClassName.equals("com.sun.tools.javac.api.BasicJavacTask")) {
            throw new IllegalArgumentException();
        }
        return getJavacTrees(JavaCompiler.CompilationTask.class, task);
    }

    public static Trees instance(ProcessingEnvironment env) {
        if (!env.getClass().getName().equals("com.sun.tools.javac.processing.JavacProcessingEnvironment")) {
            throw new IllegalArgumentException();
        }
        return getJavacTrees(ProcessingEnvironment.class, env);
    }

    static Trees getJavacTrees(Class<?> argType, Object arg) {
        try {
            ClassLoader cl = arg.getClass().getClassLoader();
            Class<?> c = Class.forName("com.sun.tools.javac.api.JavacTrees", false, cl);
            Method m = c.getMethod("instance", Class.forName(argType.getName(), false, cl));
            return (Trees) m.invoke(null, arg);
        } catch (Throwable e) {
            throw new AssertionError(e);
        }
    }
}
