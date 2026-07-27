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
public class DocTreeScanner<R, P> implements DocTreeVisitor<R, P> {
    public R scan(DocTree docTree, P p) {
        if (docTree == null) {
            return null;
        }
        return (R) docTree.accept(this, p);
    }

    private R scanAndReduce(DocTree node, P p, R r) {
        return reduce(scan(node, p), r);
    }

    public R scan(Iterable<? extends DocTree> iterable, P p) {
        R rScan = null;
        if (iterable != null) {
            boolean z = true;
            for (DocTree docTree : iterable) {
                rScan = z ? scan(docTree, p) : scanAndReduce(docTree, p, rScan);
                z = false;
            }
        }
        return rScan;
    }

    private R scanAndReduce(Iterable<? extends DocTree> nodes, P p, R r) {
        return reduce(scan(nodes, p), r);
    }

    public R reduce(R r1, R r2) {
        return r1;
    }

    @Override // com.sun.source.doctree.DocTreeVisitor
    public R visitAttribute(AttributeTree node, P p) {
        return null;
    }

    @Override // com.sun.source.doctree.DocTreeVisitor
    public R visitAuthor(AuthorTree node, P p) {
        return scan(node.getName(), p);
    }

    @Override // com.sun.source.doctree.DocTreeVisitor
    public R visitComment(CommentTree node, P p) {
        return null;
    }

    @Override // com.sun.source.doctree.DocTreeVisitor
    public R visitDeprecated(DeprecatedTree node, P p) {
        return scan(node.getBody(), p);
    }

    @Override // com.sun.source.doctree.DocTreeVisitor
    public R visitDocComment(DocCommentTree node, P p) {
        R r = scan(node.getFirstSentence(), p);
        return scanAndReduce(node.getBlockTags(), p, scanAndReduce(node.getBody(), p, r));
    }

    @Override // com.sun.source.doctree.DocTreeVisitor
    public R visitDocRoot(DocRootTree node, P p) {
        return null;
    }

    @Override // com.sun.source.doctree.DocTreeVisitor
    public R visitEndElement(EndElementTree node, P p) {
        return null;
    }

    @Override // com.sun.source.doctree.DocTreeVisitor
    public R visitEntity(EntityTree node, P p) {
        return null;
    }

    @Override // com.sun.source.doctree.DocTreeVisitor
    public R visitErroneous(ErroneousTree node, P p) {
        return null;
    }

    @Override // com.sun.source.doctree.DocTreeVisitor
    public R visitIdentifier(IdentifierTree node, P p) {
        return null;
    }

    @Override // com.sun.source.doctree.DocTreeVisitor
    public R visitInheritDoc(InheritDocTree node, P p) {
        return null;
    }

    @Override // com.sun.source.doctree.DocTreeVisitor
    public R visitLink(LinkTree node, P p) {
        R r = scan(node.getReference(), p);
        return scanAndReduce(node.getLabel(), p, r);
    }

    @Override // com.sun.source.doctree.DocTreeVisitor
    public R visitLiteral(LiteralTree node, P p) {
        return null;
    }

    @Override // com.sun.source.doctree.DocTreeVisitor
    public R visitParam(ParamTree node, P p) {
        R r = scan(node.getName(), p);
        return scanAndReduce(node.getDescription(), p, r);
    }

    @Override // com.sun.source.doctree.DocTreeVisitor
    public R visitReference(ReferenceTree node, P p) {
        return null;
    }

    @Override // com.sun.source.doctree.DocTreeVisitor
    public R visitReturn(ReturnTree node, P p) {
        return scan(node.getDescription(), p);
    }

    @Override // com.sun.source.doctree.DocTreeVisitor
    public R visitSee(SeeTree node, P p) {
        return scan(node.getReference(), p);
    }

    @Override // com.sun.source.doctree.DocTreeVisitor
    public R visitSerial(SerialTree node, P p) {
        return scan(node.getDescription(), p);
    }

    @Override // com.sun.source.doctree.DocTreeVisitor
    public R visitSerialData(SerialDataTree node, P p) {
        return scan(node.getDescription(), p);
    }

    @Override // com.sun.source.doctree.DocTreeVisitor
    public R visitSerialField(SerialFieldTree node, P p) {
        R r = scan(node.getName(), p);
        return scanAndReduce(node.getDescription(), p, scanAndReduce(node.getType(), p, r));
    }

    @Override // com.sun.source.doctree.DocTreeVisitor
    public R visitSince(SinceTree node, P p) {
        return scan(node.getBody(), p);
    }

    @Override // com.sun.source.doctree.DocTreeVisitor
    public R visitStartElement(StartElementTree node, P p) {
        return scan(node.getAttributes(), p);
    }

    @Override // com.sun.source.doctree.DocTreeVisitor
    public R visitText(TextTree node, P p) {
        return null;
    }

    @Override // com.sun.source.doctree.DocTreeVisitor
    public R visitThrows(ThrowsTree node, P p) {
        R r = scan(node.getExceptionName(), p);
        return scanAndReduce(node.getDescription(), p, r);
    }

    @Override // com.sun.source.doctree.DocTreeVisitor
    public R visitUnknownBlockTag(UnknownBlockTagTree node, P p) {
        return scan(node.getContent(), p);
    }

    @Override // com.sun.source.doctree.DocTreeVisitor
    public R visitUnknownInlineTag(UnknownInlineTagTree node, P p) {
        return scan(node.getContent(), p);
    }

    @Override // com.sun.source.doctree.DocTreeVisitor
    public R visitValue(ValueTree node, P p) {
        return scan(node.getReference(), p);
    }

    @Override // com.sun.source.doctree.DocTreeVisitor
    public R visitVersion(VersionTree node, P p) {
        return scan(node.getBody(), p);
    }

    @Override // com.sun.source.doctree.DocTreeVisitor
    public R visitOther(DocTree node, P p) {
        return null;
    }
}
