package com.sun.source.util;

import com.sun.source.tree.CompilationUnitTree;
import com.sun.source.tree.Tree;
import java.util.Iterator;
import jdk.Exported;

/* JADX INFO: loaded from: classes.dex */
@Exported
public class TreePath implements Iterable<Tree> {
    private CompilationUnitTree compilationUnit;
    private Tree leaf;
    private TreePath parent;

    public static TreePath getPath(CompilationUnitTree unit, Tree target) {
        return getPath(new TreePath(unit), target);
    }

    public static TreePath getPath(TreePath path, Tree target) {
        path.getClass();
        target.getClass();
        if (path.getLeaf() == target) {
            return path;
        }
        try {
            new TreePathScanner<TreePath, Tree>() { // from class: com.sun.source.util.TreePath.1PathFinder
                @Override // com.sun.source.util.TreePathScanner, com.sun.source.util.TreeScanner
                public TreePath scan(Tree tree, Tree target2) {
                    if (tree == target2) {
                        throw new C1Result(new TreePath(getCurrentPath(), target2));
                    }
                    return (TreePath) super.scan(tree, target2);
                }
            }.scan(path, target);
            return null;
        } catch (C1Result result) {
            return result.path;
        }
    }

    /* JADX INFO: renamed from: com.sun.source.util.TreePath$1Result, reason: invalid class name */
    class C1Result extends Error {
        static final long serialVersionUID = -5942088234594905625L;
        TreePath path;

        C1Result(TreePath path) {
            this.path = path;
        }
    }

    public TreePath(CompilationUnitTree t) {
        this(null, t);
    }

    public TreePath(TreePath p, Tree t) {
        if (t.getKind() == Tree.Kind.COMPILATION_UNIT) {
            this.compilationUnit = (CompilationUnitTree) t;
            this.parent = null;
        } else {
            this.compilationUnit = p.compilationUnit;
            this.parent = p;
        }
        this.leaf = t;
    }

    public CompilationUnitTree getCompilationUnit() {
        return this.compilationUnit;
    }

    public Tree getLeaf() {
        return this.leaf;
    }

    public TreePath getParentPath() {
        return this.parent;
    }

    @Override // java.lang.Iterable
    public Iterator<Tree> iterator() {
        return new Iterator<Tree>() { // from class: com.sun.source.util.TreePath.1
            private TreePath next;

            {
                this.next = TreePath.this;
            }

            @Override // java.util.Iterator
            public boolean hasNext() {
                return this.next != null;
            }

            /* JADX WARN: Can't rename method to resolve collision */
            @Override // java.util.Iterator
            public Tree next() {
                Tree t = this.next.leaf;
                this.next = this.next.parent;
                return t;
            }

            @Override // java.util.Iterator
            public void remove() {
                throw new UnsupportedOperationException();
            }
        };
    }
}
