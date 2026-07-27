package com.sun.source.util;

import com.sun.source.doctree.DocTree;
import jdk.Exported;

/* JADX INFO: loaded from: classes.dex */
@Exported
public class DocTreePathScanner<R, P> extends DocTreeScanner<R, P> {
    private DocTreePath path;

    public R scan(DocTreePath docTreePath, P p) {
        this.path = docTreePath;
        try {
            return (R) docTreePath.getLeaf().accept(this, p);
        } finally {
            this.path = null;
        }
    }

    @Override // com.sun.source.util.DocTreeScanner
    public R scan(DocTree docTree, P p) {
        if (docTree == null) {
            return null;
        }
        DocTreePath docTreePath = this.path;
        this.path = new DocTreePath(this.path, docTree);
        try {
            return (R) docTree.accept(this, p);
        } finally {
            this.path = docTreePath;
        }
    }

    public DocTreePath getCurrentPath() {
        return this.path;
    }
}
