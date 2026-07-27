package com.sun.source.util;

import com.sun.source.doctree.DocCommentTree;
import com.sun.source.doctree.DocTree;
import java.util.Iterator;
import jdk.Exported;

/* JADX INFO: loaded from: classes.dex */
@Exported
public class DocTreePath implements Iterable<DocTree> {
    private final DocCommentTree docComment;
    private final DocTree leaf;
    private final DocTreePath parent;
    private final TreePath treePath;

    public static DocTreePath getPath(TreePath treePath, DocCommentTree doc, DocTree target) {
        return getPath(new DocTreePath(treePath, doc), target);
    }

    public static DocTreePath getPath(DocTreePath path, DocTree target) {
        path.getClass();
        target.getClass();
        if (path.getLeaf() == target) {
            return path;
        }
        try {
            new DocTreePathScanner<DocTreePath, DocTree>() { // from class: com.sun.source.util.DocTreePath.1PathFinder
                @Override // com.sun.source.util.DocTreePathScanner, com.sun.source.util.DocTreeScanner
                public DocTreePath scan(DocTree tree, DocTree target2) {
                    if (tree == target2) {
                        throw new C1Result(new DocTreePath(getCurrentPath(), target2));
                    }
                    return (DocTreePath) super.scan(tree, target2);
                }
            }.scan(path, target);
            return null;
        } catch (C1Result result) {
            return result.path;
        }
    }

    /* JADX INFO: renamed from: com.sun.source.util.DocTreePath$1Result, reason: invalid class name */
    class C1Result extends Error {
        static final long serialVersionUID = -5942088234594905625L;
        DocTreePath path;

        C1Result(DocTreePath path) {
            this.path = path;
        }
    }

    public DocTreePath(TreePath treePath, DocCommentTree t) {
        treePath.getClass();
        t.getClass();
        this.treePath = treePath;
        this.docComment = t;
        this.parent = null;
        this.leaf = t;
    }

    public DocTreePath(DocTreePath p, DocTree t) {
        if (t.getKind() == DocTree.Kind.DOC_COMMENT) {
            throw new IllegalArgumentException("Use DocTreePath(TreePath, DocCommentTree) to construct DocTreePath for a DocCommentTree.");
        }
        this.treePath = p.treePath;
        this.docComment = p.docComment;
        this.parent = p;
        this.leaf = t;
    }

    public TreePath getTreePath() {
        return this.treePath;
    }

    public DocCommentTree getDocComment() {
        return this.docComment;
    }

    public DocTree getLeaf() {
        return this.leaf;
    }

    public DocTreePath getParentPath() {
        return this.parent;
    }

    @Override // java.lang.Iterable
    public Iterator<DocTree> iterator() {
        return new Iterator<DocTree>() { // from class: com.sun.source.util.DocTreePath.1
            private DocTreePath next;

            {
                this.next = DocTreePath.this;
            }

            @Override // java.util.Iterator
            public boolean hasNext() {
                return this.next != null;
            }

            /* JADX WARN: Can't rename method to resolve collision */
            @Override // java.util.Iterator
            public DocTree next() {
                DocTree t = this.next.leaf;
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
