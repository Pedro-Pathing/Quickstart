package com.sun.tools.javac.processing;

import com.sun.tools.javac.util.Context;
import com.sun.tools.javac.util.Log;
import javax.annotation.processing.Messager;
import javax.lang.model.element.AnnotationMirror;
import javax.lang.model.element.Element;
import javax.tools.Diagnostic;

/* JADX INFO: loaded from: classes.dex */
public class JavacMessager implements Messager {
    Log log;
    JavacProcessingEnvironment processingEnv;
    int errorCount = 0;
    int warningCount = 0;

    JavacMessager(Context context, JavacProcessingEnvironment processingEnv) {
        this.log = Log.instance(context);
        this.processingEnv = processingEnv;
    }

    @Override // javax.annotation.processing.Messager
    public void printMessage(Diagnostic.Kind kind, CharSequence msg) {
        printMessage(kind, msg, null, null, null);
    }

    @Override // javax.annotation.processing.Messager
    public void printMessage(Diagnostic.Kind kind, CharSequence msg, Element e) {
        printMessage(kind, msg, e, null, null);
    }

    @Override // javax.annotation.processing.Messager
    public void printMessage(Diagnostic.Kind kind, CharSequence msg, Element e, AnnotationMirror a) {
        printMessage(kind, msg, e, a, null);
    }

    /* JADX WARN: Removed duplicated region for block: B:21:0x008f A[DONT_GENERATE] */
    /* JADX WARN: Removed duplicated region for block: B:30:? A[RETURN, SYNTHETIC] */
    @Override // javax.annotation.processing.Messager
    /*
        Code decompiled incorrectly, please refer to instructions dump.
        To view partially-correct code enable 'Show inconsistent code' option in preferences
    */
    public void printMessage(javax.tools.Diagnostic.Kind r10, java.lang.CharSequence r11, javax.lang.model.element.Element r12, javax.lang.model.element.AnnotationMirror r13, javax.lang.model.element.AnnotationValue r14) {
        /*
            r9 = this;
            r0 = 0
            r1 = 0
            r2 = 0
            com.sun.tools.javac.processing.JavacProcessingEnvironment r3 = r9.processingEnv
            com.sun.tools.javac.model.JavacElements r3 = r3.getElementUtils()
            com.sun.tools.javac.util.Pair r4 = r3.getTreeAndTopLevel(r12, r13, r14)
            if (r4 == 0) goto L25
            B r5 = r4.snd
            com.sun.tools.javac.tree.JCTree$JCCompilationUnit r5 = (com.sun.tools.javac.tree.JCTree.JCCompilationUnit) r5
            javax.tools.JavaFileObject r1 = r5.sourcefile
            if (r1 == 0) goto L25
            com.sun.tools.javac.util.Log r5 = r9.log
            javax.tools.JavaFileObject r0 = r5.useSource(r1)
            A r5 = r4.fst
            com.sun.tools.javac.tree.JCTree r5 = (com.sun.tools.javac.tree.JCTree) r5
            com.sun.tools.javac.util.JCDiagnostic$DiagnosticPosition r2 = r5.pos()
        L25:
            int[] r5 = com.sun.tools.javac.processing.JavacMessager.AnonymousClass1.$SwitchMap$javax$tools$Diagnostic$Kind     // Catch: java.lang.Throwable -> L95
            int r6 = r10.ordinal()     // Catch: java.lang.Throwable -> L95
            r5 = r5[r6]     // Catch: java.lang.Throwable -> L95
            java.lang.String r6 = "proc.messager"
            r7 = 1
            switch(r5) {
                case 1: goto L5c;
                case 2: goto L49;
                case 3: goto L36;
                default: goto L33;
            }
        L33:
            com.sun.tools.javac.util.Log r5 = r9.log     // Catch: java.lang.Throwable -> L95
            goto L82
        L36:
            int r5 = r9.warningCount     // Catch: java.lang.Throwable -> L95
            int r5 = r5 + r7
            r9.warningCount = r5     // Catch: java.lang.Throwable -> L95
            com.sun.tools.javac.util.Log r5 = r9.log     // Catch: java.lang.Throwable -> L95
            java.lang.String r7 = r11.toString()     // Catch: java.lang.Throwable -> L95
            java.lang.Object[] r7 = new java.lang.Object[]{r7}     // Catch: java.lang.Throwable -> L95
            r5.mandatoryWarning(r2, r6, r7)     // Catch: java.lang.Throwable -> L95
            goto L8d
        L49:
            int r5 = r9.warningCount     // Catch: java.lang.Throwable -> L95
            int r5 = r5 + r7
            r9.warningCount = r5     // Catch: java.lang.Throwable -> L95
            com.sun.tools.javac.util.Log r5 = r9.log     // Catch: java.lang.Throwable -> L95
            java.lang.String r7 = r11.toString()     // Catch: java.lang.Throwable -> L95
            java.lang.Object[] r7 = new java.lang.Object[]{r7}     // Catch: java.lang.Throwable -> L95
            r5.warning(r2, r6, r7)     // Catch: java.lang.Throwable -> L95
            goto L8d
        L5c:
            int r5 = r9.errorCount     // Catch: java.lang.Throwable -> L95
            int r5 = r5 + r7
            r9.errorCount = r5     // Catch: java.lang.Throwable -> L95
            com.sun.tools.javac.util.Log r5 = r9.log     // Catch: java.lang.Throwable -> L95
            boolean r5 = r5.multipleErrors     // Catch: java.lang.Throwable -> L95
            com.sun.tools.javac.util.Log r8 = r9.log     // Catch: java.lang.Throwable -> L95
            r8.multipleErrors = r7     // Catch: java.lang.Throwable -> L95
            com.sun.tools.javac.util.Log r7 = r9.log     // Catch: java.lang.Throwable -> L7c
            java.lang.String r8 = r11.toString()     // Catch: java.lang.Throwable -> L7c
            java.lang.Object[] r8 = new java.lang.Object[]{r8}     // Catch: java.lang.Throwable -> L7c
            r7.error(r2, r6, r8)     // Catch: java.lang.Throwable -> L7c
            com.sun.tools.javac.util.Log r6 = r9.log     // Catch: java.lang.Throwable -> L95
            r6.multipleErrors = r5     // Catch: java.lang.Throwable -> L95
            goto L8d
        L7c:
            r6 = move-exception
            com.sun.tools.javac.util.Log r7 = r9.log     // Catch: java.lang.Throwable -> L95
            r7.multipleErrors = r5     // Catch: java.lang.Throwable -> L95
            throw r6     // Catch: java.lang.Throwable -> L95
        L82:
            java.lang.String r7 = r11.toString()     // Catch: java.lang.Throwable -> L95
            java.lang.Object[] r7 = new java.lang.Object[]{r7}     // Catch: java.lang.Throwable -> L95
            r5.note(r2, r6, r7)     // Catch: java.lang.Throwable -> L95
        L8d:
            if (r1 == 0) goto L94
            com.sun.tools.javac.util.Log r5 = r9.log
            r5.useSource(r0)
        L94:
            return
        L95:
            r5 = move-exception
            if (r1 == 0) goto L9d
            com.sun.tools.javac.util.Log r6 = r9.log
            r6.useSource(r0)
        L9d:
            throw r5
        */
        throw new UnsupportedOperationException("Method not decompiled: com.sun.tools.javac.processing.JavacMessager.printMessage(javax.tools.Diagnostic$Kind, java.lang.CharSequence, javax.lang.model.element.Element, javax.lang.model.element.AnnotationMirror, javax.lang.model.element.AnnotationValue):void");
    }

    public void printError(String msg) {
        printMessage(Diagnostic.Kind.ERROR, msg);
    }

    public void printWarning(String msg) {
        printMessage(Diagnostic.Kind.WARNING, msg);
    }

    public void printNotice(String msg) {
        printMessage(Diagnostic.Kind.NOTE, msg);
    }

    public boolean errorRaised() {
        return this.errorCount > 0;
    }

    public int errorCount() {
        return this.errorCount;
    }

    public int warningCount() {
        return this.warningCount;
    }

    public void newRound(Context context) {
        this.log = Log.instance(context);
        this.errorCount = 0;
    }

    public String toString() {
        return "javac Messager";
    }
}
