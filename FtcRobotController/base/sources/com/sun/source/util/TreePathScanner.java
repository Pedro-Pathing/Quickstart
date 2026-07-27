package com.sun.source.util;

import com.sun.source.tree.Tree;
import jdk.Exported;

/* JADX INFO: loaded from: classes.dex */
@Exported
public class TreePathScanner<R, P> extends TreeScanner<R, P> {
    private TreePath path;

    public R scan(TreePath treePath, P p) {
        this.path = treePath;
        try {
            return (R) treePath.getLeaf().accept(this, p);
        } finally {
            this.path = null;
        }
    }

    @Override // com.sun.source.util.TreeScanner
    public R scan(Tree tree, P p) {
        if (tree == null) {
            return null;
        }
        TreePath treePath = this.path;
        this.path = new TreePath(this.path, tree);
        try {
            return (R) tree.accept(this, p);
        } finally {
            this.path = treePath;
        }
    }

    public TreePath getCurrentPath() {
        return this.path;
    }
}
