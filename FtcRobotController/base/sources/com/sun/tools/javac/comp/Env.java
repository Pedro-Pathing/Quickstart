package com.sun.tools.javac.comp;

import com.sun.tools.javac.tree.JCTree;
import java.util.Iterator;
import java.util.NoSuchElementException;

/* JADX INFO: loaded from: classes.dex */
public class Env<A> implements Iterable<Env<A>> {
    public A info;
    public JCTree tree;
    public boolean baseClause = false;
    public Env<A> next = null;
    public Env<A> outer = null;
    public JCTree.JCCompilationUnit toplevel = null;
    public JCTree.JCClassDecl enclClass = null;
    public JCTree.JCMethodDecl enclMethod = null;

    public Env(JCTree tree, A info) {
        this.tree = tree;
        this.info = info;
    }

    public Env<A> dup(JCTree tree, A info) {
        return dupto(new Env<>(tree, info));
    }

    public Env<A> dupto(Env<A> that) {
        that.next = this;
        that.outer = this.outer;
        that.toplevel = this.toplevel;
        that.enclClass = this.enclClass;
        that.enclMethod = this.enclMethod;
        return that;
    }

    public Env<A> dup(JCTree tree) {
        return dup(tree, this.info);
    }

    public Env<A> enclosing(JCTree.Tag tag) {
        Env<A> env1 = this;
        while (env1 != null && !env1.tree.hasTag(tag)) {
            env1 = env1.next;
        }
        return env1;
    }

    public String toString() {
        StringBuilder sb = new StringBuilder();
        sb.append("Env[").append(this.info);
        if (this.outer != null) {
            sb.append(",outer=").append(this.outer);
        }
        sb.append("]");
        return sb.toString();
    }

    @Override // java.lang.Iterable
    public Iterator<Env<A>> iterator() {
        return new Iterator<Env<A>>() { // from class: com.sun.tools.javac.comp.Env.1
            Env<A> next;

            {
                this.next = Env.this;
            }

            @Override // java.util.Iterator
            public boolean hasNext() {
                return this.next.outer != null;
            }

            @Override // java.util.Iterator
            public Env<A> next() {
                if (hasNext()) {
                    Env<A> current = this.next;
                    this.next = current.outer;
                    return current;
                }
                throw new NoSuchElementException();
            }

            @Override // java.util.Iterator
            public void remove() {
                throw new UnsupportedOperationException();
            }
        };
    }
}
