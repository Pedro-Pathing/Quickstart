package com.sun.tools.javac.comp;

import com.sun.tools.javac.util.Context;
import java.util.AbstractQueue;
import java.util.HashMap;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.Map;
import java.util.Queue;
import javax.tools.JavaFileObject;

/* JADX INFO: loaded from: classes.dex */
public class Todo extends AbstractQueue<Env<AttrContext>> {
    protected static final Context.Key<Todo> todoKey = new Context.Key<>();
    LinkedList<Env<AttrContext>> contents = new LinkedList<>();
    LinkedList<Queue<Env<AttrContext>>> contentsByFile;
    Map<JavaFileObject, FileQueue> fileMap;

    public static Todo instance(Context context) {
        Todo instance = (Todo) context.get(todoKey);
        if (instance == null) {
            return new Todo(context);
        }
        return instance;
    }

    protected Todo(Context context) {
        context.put(todoKey, this);
    }

    public void append(Env<AttrContext> env) {
        add(env);
    }

    @Override // java.util.AbstractCollection, java.util.Collection, java.lang.Iterable
    public Iterator<Env<AttrContext>> iterator() {
        return this.contents.iterator();
    }

    @Override // java.util.AbstractCollection, java.util.Collection
    public int size() {
        return this.contents.size();
    }

    @Override // java.util.Queue
    public boolean offer(Env<AttrContext> e) {
        if (this.contents.add(e)) {
            if (this.contentsByFile != null) {
                addByFile(e);
                return true;
            }
            return true;
        }
        return false;
    }

    @Override // java.util.Queue
    public Env<AttrContext> poll() {
        if (size() == 0) {
            return null;
        }
        Env<AttrContext> env = this.contents.remove(0);
        if (this.contentsByFile != null) {
            removeByFile(env);
        }
        return env;
    }

    @Override // java.util.Queue
    public Env<AttrContext> peek() {
        if (size() == 0) {
            return null;
        }
        return this.contents.get(0);
    }

    public Queue<Queue<Env<AttrContext>>> groupByFile() {
        if (this.contentsByFile == null) {
            this.contentsByFile = new LinkedList<>();
            for (Env<AttrContext> env : this.contents) {
                addByFile(env);
            }
        }
        return this.contentsByFile;
    }

    private void addByFile(Env<AttrContext> env) {
        JavaFileObject file = env.toplevel.sourcefile;
        if (this.fileMap == null) {
            this.fileMap = new HashMap();
        }
        FileQueue fq = this.fileMap.get(file);
        if (fq == null) {
            fq = new FileQueue();
            this.fileMap.put(file, fq);
            this.contentsByFile.add(fq);
        }
        fq.fileContents.add(env);
    }

    private void removeByFile(Env<AttrContext> env) {
        JavaFileObject file = env.toplevel.sourcefile;
        FileQueue fq = this.fileMap.get(file);
        if (fq != null && fq.fileContents.remove(env) && fq.isEmpty()) {
            this.fileMap.remove(file);
            this.contentsByFile.remove(fq);
        }
    }

    class FileQueue extends AbstractQueue<Env<AttrContext>> {
        LinkedList<Env<AttrContext>> fileContents = new LinkedList<>();

        FileQueue() {
        }

        @Override // java.util.AbstractCollection, java.util.Collection, java.lang.Iterable
        public Iterator<Env<AttrContext>> iterator() {
            return this.fileContents.iterator();
        }

        @Override // java.util.AbstractCollection, java.util.Collection
        public int size() {
            return this.fileContents.size();
        }

        @Override // java.util.Queue
        public boolean offer(Env<AttrContext> e) {
            if (this.fileContents.offer(e)) {
                Todo.this.contents.add(e);
                return true;
            }
            return false;
        }

        @Override // java.util.Queue
        public Env<AttrContext> poll() {
            if (this.fileContents.size() == 0) {
                return null;
            }
            Env<AttrContext> env = this.fileContents.remove(0);
            Todo.this.contents.remove(env);
            return env;
        }

        @Override // java.util.Queue
        public Env<AttrContext> peek() {
            if (this.fileContents.size() == 0) {
                return null;
            }
            return this.fileContents.get(0);
        }
    }
}
