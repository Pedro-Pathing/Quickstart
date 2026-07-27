package com.sun.tools.javac.api;

import com.sun.source.tree.CompilationUnitTree;
import com.sun.source.tree.Tree;
import com.sun.source.util.JavacTask;
import com.sun.source.util.TaskListener;
import com.sun.tools.javac.model.JavacElements;
import com.sun.tools.javac.model.JavacTypes;
import com.sun.tools.javac.tree.JCTree;
import com.sun.tools.javac.util.Context;
import java.io.IOException;
import java.util.Collection;
import java.util.Locale;
import javax.annotation.processing.Processor;
import javax.lang.model.element.Element;
import javax.lang.model.type.TypeMirror;
import javax.lang.model.util.Elements;
import javax.lang.model.util.Types;
import javax.tools.JavaFileObject;

/* JADX INFO: loaded from: classes.dex */
public class BasicJavacTask extends JavacTask {
    protected Context context;
    private TaskListener taskListener;

    public static JavacTask instance(Context context) {
        JavacTask instance = (JavacTask) context.get(JavacTask.class);
        if (instance == null) {
            return new BasicJavacTask(context, true);
        }
        return instance;
    }

    public BasicJavacTask(Context c, boolean register) {
        this.context = c;
        if (register) {
            this.context.put((Class<BasicJavacTask>) JavacTask.class, this);
        }
    }

    @Override // com.sun.source.util.JavacTask
    public Iterable<? extends CompilationUnitTree> parse() throws IOException {
        throw new IllegalStateException();
    }

    @Override // com.sun.source.util.JavacTask
    public Iterable<? extends Element> analyze() throws IOException {
        throw new IllegalStateException();
    }

    @Override // com.sun.source.util.JavacTask
    public Iterable<? extends JavaFileObject> generate() throws IOException {
        throw new IllegalStateException();
    }

    @Override // com.sun.source.util.JavacTask
    public void setTaskListener(TaskListener tl) {
        MultiTaskListener mtl = MultiTaskListener.instance(this.context);
        if (this.taskListener != null) {
            mtl.remove(this.taskListener);
        }
        if (tl != null) {
            mtl.add(tl);
        }
        this.taskListener = tl;
    }

    @Override // com.sun.source.util.JavacTask
    public void addTaskListener(TaskListener taskListener) {
        MultiTaskListener mtl = MultiTaskListener.instance(this.context);
        mtl.add(taskListener);
    }

    @Override // com.sun.source.util.JavacTask
    public void removeTaskListener(TaskListener taskListener) {
        MultiTaskListener mtl = MultiTaskListener.instance(this.context);
        mtl.remove(taskListener);
    }

    public Collection<TaskListener> getTaskListeners() {
        MultiTaskListener mtl = MultiTaskListener.instance(this.context);
        return mtl.getTaskListeners();
    }

    @Override // com.sun.source.util.JavacTask
    public TypeMirror getTypeMirror(Iterable<? extends Tree> path) {
        Tree last = null;
        for (Tree node : path) {
            last = node;
        }
        return ((JCTree) last).type;
    }

    @Override // com.sun.source.util.JavacTask
    public Elements getElements() {
        return JavacElements.instance(this.context);
    }

    @Override // com.sun.source.util.JavacTask
    public Types getTypes() {
        return JavacTypes.instance(this.context);
    }

    @Override // javax.tools.JavaCompiler.CompilationTask
    public void setProcessors(Iterable<? extends Processor> processors) {
        throw new IllegalStateException();
    }

    @Override // javax.tools.JavaCompiler.CompilationTask
    public void setLocale(Locale locale) {
        throw new IllegalStateException();
    }

    /* JADX WARN: Can't rename method to resolve collision */
    @Override // java.util.concurrent.Callable
    public Boolean call() {
        throw new IllegalStateException();
    }

    public Context getContext() {
        return this.context;
    }

    public void updateContext(Context newContext) {
        this.context = newContext;
    }
}
