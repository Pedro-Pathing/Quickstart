package com.sun.source.util;

import com.sun.source.doctree.AttributeTree;
import com.sun.source.doctree.AuthorTree;
import com.sun.source.doctree.CommentTree;
import com.sun.source.doctree.DeprecatedTree;
import com.sun.source.doctree.DocCommentTree;
import com.sun.source.doctree.DocRootTree;
import com.sun.source.doctree.DocTree;
import com.sun.source.doctree.DocTreeVisitor;
import com.sun.source.doctree.EndElementTree;
import com.sun.source.doctree.EntityTree;
import com.sun.source.doctree.ErroneousTree;
import com.sun.source.doctree.IdentifierTree;
import com.sun.source.doctree.InheritDocTree;
import com.sun.source.doctree.LinkTree;
import com.sun.source.doctree.LiteralTree;
import com.sun.source.doctree.ParamTree;
import com.sun.source.doctree.ReferenceTree;
import com.sun.source.doctree.ReturnTree;
import com.sun.source.doctree.SeeTree;
import com.sun.source.doctree.SerialDataTree;
import com.sun.source.doctree.SerialFieldTree;
import com.sun.source.doctree.SerialTree;
import com.sun.source.doctree.SinceTree;
import com.sun.source.doctree.StartElementTree;
import com.sun.source.doctree.TextTree;
import com.sun.source.doctree.ThrowsTree;
import com.sun.source.doctree.UnknownBlockTagTree;
import com.sun.source.doctree.UnknownInlineTagTree;
import com.sun.source.doctree.ValueTree;
import com.sun.source.doctree.VersionTree;
import jdk.Exported;

/* JADX INFO: loaded from: classes.dex */
@Exported
public class SimpleDocTreeVisitor<R, P> implements DocTreeVisitor<R, P> {
    protected final R DEFAULT_VALUE;

    protected SimpleDocTreeVisitor() {
        this.DEFAULT_VALUE = null;
    }

    protected SimpleDocTreeVisitor(R defaultValue) {
        this.DEFAULT_VALUE = defaultValue;
    }

    protected R defaultAction(DocTree node, P p) {
        return this.DEFAULT_VALUE;
    }

    public final R visit(DocTree docTree, P p) {
        if (docTree == null) {
            return null;
        }
        return (R) docTree.accept(this, p);
    }

    public final R visit(Iterable<? extends DocTree> nodes, P p) {
        R r = null;
        if (nodes != null) {
            for (DocTree node : nodes) {
                r = visit(node, p);
            }
        }
        return r;
    }

    @Override // com.sun.source.doctree.DocTreeVisitor
    public R visitAttribute(AttributeTree node, P p) {
        return defaultAction(node, p);
    }

    @Override // com.sun.source.doctree.DocTreeVisitor
    public R visitAuthor(AuthorTree node, P p) {
        return defaultAction(node, p);
    }

    @Override // com.sun.source.doctree.DocTreeVisitor
    public R visitComment(CommentTree node, P p) {
        return defaultAction(node, p);
    }

    @Override // com.sun.source.doctree.DocTreeVisitor
    public R visitDeprecated(DeprecatedTree node, P p) {
        return defaultAction(node, p);
    }

    @Override // com.sun.source.doctree.DocTreeVisitor
    public R visitDocComment(DocCommentTree node, P p) {
        return defaultAction(node, p);
    }

    @Override // com.sun.source.doctree.DocTreeVisitor
    public R visitDocRoot(DocRootTree node, P p) {
        return defaultAction(node, p);
    }

    @Override // com.sun.source.doctree.DocTreeVisitor
    public R visitEndElement(EndElementTree node, P p) {
        return defaultAction(node, p);
    }

    @Override // com.sun.source.doctree.DocTreeVisitor
    public R visitEntity(EntityTree node, P p) {
        return defaultAction(node, p);
    }

    @Override // com.sun.source.doctree.DocTreeVisitor
    public R visitErroneous(ErroneousTree node, P p) {
        return defaultAction(node, p);
    }

    @Override // com.sun.source.doctree.DocTreeVisitor
    public R visitIdentifier(IdentifierTree node, P p) {
        return defaultAction(node, p);
    }

    @Override // com.sun.source.doctree.DocTreeVisitor
    public R visitInheritDoc(InheritDocTree node, P p) {
        return defaultAction(node, p);
    }

    @Override // com.sun.source.doctree.DocTreeVisitor
    public R visitLink(LinkTree node, P p) {
        return defaultAction(node, p);
    }

    @Override // com.sun.source.doctree.DocTreeVisitor
    public R visitLiteral(LiteralTree node, P p) {
        return defaultAction(node, p);
    }

    @Override // com.sun.source.doctree.DocTreeVisitor
    public R visitParam(ParamTree node, P p) {
        return defaultAction(node, p);
    }

    @Override // com.sun.source.doctree.DocTreeVisitor
    public R visitReference(ReferenceTree node, P p) {
        return defaultAction(node, p);
    }

    @Override // com.sun.source.doctree.DocTreeVisitor
    public R visitReturn(ReturnTree node, P p) {
        return defaultAction(node, p);
    }

    @Override // com.sun.source.doctree.DocTreeVisitor
    public R visitSee(SeeTree node, P p) {
        return defaultAction(node, p);
    }

    @Override // com.sun.source.doctree.DocTreeVisitor
    public R visitSerial(SerialTree node, P p) {
        return defaultAction(node, p);
    }

    @Override // com.sun.source.doctree.DocTreeVisitor
    public R visitSerialData(SerialDataTree node, P p) {
        return defaultAction(node, p);
    }

    @Override // com.sun.source.doctree.DocTreeVisitor
    public R visitSerialField(SerialFieldTree node, P p) {
        return defaultAction(node, p);
    }

    @Override // com.sun.source.doctree.DocTreeVisitor
    public R visitSince(SinceTree node, P p) {
        return defaultAction(node, p);
    }

    @Override // com.sun.source.doctree.DocTreeVisitor
    public R visitStartElement(StartElementTree node, P p) {
        return defaultAction(node, p);
    }

    @Override // com.sun.source.doctree.DocTreeVisitor
    public R visitText(TextTree node, P p) {
        return defaultAction(node, p);
    }

    @Override // com.sun.source.doctree.DocTreeVisitor
    public R visitThrows(ThrowsTree node, P p) {
        return defaultAction(node, p);
    }

    @Override // com.sun.source.doctree.DocTreeVisitor
    public R visitUnknownBlockTag(UnknownBlockTagTree node, P p) {
        return defaultAction(node, p);
    }

    @Override // com.sun.source.doctree.DocTreeVisitor
    public R visitUnknownInlineTag(UnknownInlineTagTree node, P p) {
        return defaultAction(node, p);
    }

    @Override // com.sun.source.doctree.DocTreeVisitor
    public R visitValue(ValueTree node, P p) {
        return defaultAction(node, p);
    }

    @Override // com.sun.source.doctree.DocTreeVisitor
    public R visitVersion(VersionTree node, P p) {
        return defaultAction(node, p);
    }

    @Override // com.sun.source.doctree.DocTreeVisitor
    public R visitOther(DocTree node, P p) {
        return defaultAction(node, p);
    }
}
