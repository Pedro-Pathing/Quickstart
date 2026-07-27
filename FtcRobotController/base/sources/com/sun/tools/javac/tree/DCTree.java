package com.sun.tools.javac.tree;

import com.sun.source.doctree.AttributeTree;
import com.sun.source.doctree.AuthorTree;
import com.sun.source.doctree.BlockTagTree;
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
import com.sun.source.doctree.InlineTagTree;
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
import com.sun.tools.javac.parser.Tokens;
import com.sun.tools.javac.util.Assert;
import com.sun.tools.javac.util.DiagnosticSource;
import com.sun.tools.javac.util.JCDiagnostic;
import com.sun.tools.javac.util.List;
import com.sun.tools.javac.util.Name;
import java.io.IOException;
import java.io.StringWriter;
import javax.tools.Diagnostic;
import javax.tools.JavaFileObject;

/* JADX INFO: loaded from: classes.dex */
public abstract class DCTree implements DocTree {
    public int pos;

    public long getSourcePosition(DCDocComment dc) {
        return dc.comment.getSourcePos(this.pos);
    }

    public JCDiagnostic.DiagnosticPosition pos(DCDocComment dc) {
        return new JCDiagnostic.SimpleDiagnosticPosition(dc.comment.getSourcePos(this.pos));
    }

    public String toString() {
        StringWriter s = new StringWriter();
        try {
            new DocPretty(s).print((DocTree) this);
            return s.toString();
        } catch (IOException e) {
            throw new AssertionError(e);
        }
    }

    public static abstract class DCEndPosTree<T extends DCEndPosTree<T>> extends DCTree {
        private int endPos = -1;

        public int getEndPos(DCDocComment dc) {
            return dc.comment.getSourcePos(this.endPos);
        }

        public T setEndPos(int endPos) {
            this.endPos = endPos;
            return this;
        }
    }

    public static class DCDocComment extends DCTree implements DocCommentTree {
        public final List<DCTree> body;
        public final Tokens.Comment comment;
        public final List<DCTree> firstSentence;
        public final List<DCTree> tags;

        public DCDocComment(Tokens.Comment comment, List<DCTree> firstSentence, List<DCTree> body, List<DCTree> tags) {
            this.comment = comment;
            this.firstSentence = firstSentence;
            this.body = body;
            this.tags = tags;
        }

        @Override // com.sun.source.doctree.DocTree
        public DocTree.Kind getKind() {
            return DocTree.Kind.DOC_COMMENT;
        }

        @Override // com.sun.source.doctree.DocTree
        public <R, D> R accept(DocTreeVisitor<R, D> v, D d) {
            return v.visitDocComment(this, d);
        }

        @Override // com.sun.source.doctree.DocCommentTree
        public List<? extends DocTree> getFirstSentence() {
            return this.firstSentence;
        }

        @Override // com.sun.source.doctree.DocCommentTree
        public List<? extends DocTree> getBody() {
            return this.body;
        }

        @Override // com.sun.source.doctree.DocCommentTree
        public List<? extends DocTree> getBlockTags() {
            return this.tags;
        }
    }

    public static abstract class DCBlockTag extends DCTree implements BlockTagTree {
        @Override // com.sun.source.doctree.BlockTagTree
        public String getTagName() {
            return getKind().tagName;
        }
    }

    public static abstract class DCInlineTag extends DCEndPosTree<DCInlineTag> implements InlineTagTree {
        @Override // com.sun.source.doctree.InlineTagTree
        public String getTagName() {
            return getKind().tagName;
        }
    }

    public static class DCAttribute extends DCTree implements AttributeTree {
        public final Name name;
        public final List<DCTree> value;
        public final AttributeTree.ValueKind vkind;

        DCAttribute(Name name, AttributeTree.ValueKind vkind, List<DCTree> value) {
            boolean z = true;
            if (vkind != AttributeTree.ValueKind.EMPTY ? value == null : value != null) {
                z = false;
            }
            Assert.check(z);
            this.name = name;
            this.vkind = vkind;
            this.value = value;
        }

        @Override // com.sun.source.doctree.DocTree
        public DocTree.Kind getKind() {
            return DocTree.Kind.ATTRIBUTE;
        }

        @Override // com.sun.source.doctree.DocTree
        public <R, D> R accept(DocTreeVisitor<R, D> v, D d) {
            return v.visitAttribute(this, d);
        }

        @Override // com.sun.source.doctree.AttributeTree
        public Name getName() {
            return this.name;
        }

        @Override // com.sun.source.doctree.AttributeTree
        public AttributeTree.ValueKind getValueKind() {
            return this.vkind;
        }

        @Override // com.sun.source.doctree.AttributeTree
        public List<DCTree> getValue() {
            return this.value;
        }
    }

    public static class DCAuthor extends DCBlockTag implements AuthorTree {
        public final List<DCTree> name;

        DCAuthor(List<DCTree> name) {
            this.name = name;
        }

        @Override // com.sun.source.doctree.DocTree
        public DocTree.Kind getKind() {
            return DocTree.Kind.AUTHOR;
        }

        @Override // com.sun.source.doctree.DocTree
        public <R, D> R accept(DocTreeVisitor<R, D> v, D d) {
            return v.visitAuthor(this, d);
        }

        @Override // com.sun.source.doctree.AuthorTree
        public List<? extends DocTree> getName() {
            return this.name;
        }
    }

    public static class DCComment extends DCTree implements CommentTree {
        public final String body;

        DCComment(String body) {
            this.body = body;
        }

        @Override // com.sun.source.doctree.DocTree
        public DocTree.Kind getKind() {
            return DocTree.Kind.COMMENT;
        }

        @Override // com.sun.source.doctree.DocTree
        public <R, D> R accept(DocTreeVisitor<R, D> v, D d) {
            return v.visitComment(this, d);
        }

        @Override // com.sun.source.doctree.CommentTree
        public String getBody() {
            return this.body;
        }
    }

    public static class DCDeprecated extends DCBlockTag implements DeprecatedTree {
        public final List<DCTree> body;

        DCDeprecated(List<DCTree> body) {
            this.body = body;
        }

        @Override // com.sun.source.doctree.DocTree
        public DocTree.Kind getKind() {
            return DocTree.Kind.DEPRECATED;
        }

        @Override // com.sun.source.doctree.DocTree
        public <R, D> R accept(DocTreeVisitor<R, D> v, D d) {
            return v.visitDeprecated(this, d);
        }

        @Override // com.sun.source.doctree.DeprecatedTree
        public List<? extends DocTree> getBody() {
            return this.body;
        }
    }

    public static class DCDocRoot extends DCInlineTag implements DocRootTree {
        @Override // com.sun.source.doctree.DocTree
        public DocTree.Kind getKind() {
            return DocTree.Kind.DOC_ROOT;
        }

        @Override // com.sun.source.doctree.DocTree
        public <R, D> R accept(DocTreeVisitor<R, D> v, D d) {
            return v.visitDocRoot(this, d);
        }
    }

    public static class DCEndElement extends DCTree implements EndElementTree {
        public final Name name;

        DCEndElement(Name name) {
            this.name = name;
        }

        @Override // com.sun.source.doctree.DocTree
        public DocTree.Kind getKind() {
            return DocTree.Kind.END_ELEMENT;
        }

        @Override // com.sun.source.doctree.DocTree
        public <R, D> R accept(DocTreeVisitor<R, D> v, D d) {
            return v.visitEndElement(this, d);
        }

        @Override // com.sun.source.doctree.EndElementTree
        public Name getName() {
            return this.name;
        }
    }

    public static class DCEntity extends DCTree implements EntityTree {
        public final Name name;

        DCEntity(Name name) {
            this.name = name;
        }

        @Override // com.sun.source.doctree.DocTree
        public DocTree.Kind getKind() {
            return DocTree.Kind.ENTITY;
        }

        @Override // com.sun.source.doctree.DocTree
        public <R, D> R accept(DocTreeVisitor<R, D> v, D d) {
            return v.visitEntity(this, d);
        }

        @Override // com.sun.source.doctree.EntityTree
        public Name getName() {
            return this.name;
        }
    }

    public static class DCErroneous extends DCTree implements ErroneousTree, JCDiagnostic.DiagnosticPosition {
        public final String body;
        public final JCDiagnostic diag;

        DCErroneous(String body, JCDiagnostic.Factory diags, DiagnosticSource diagSource, String code, Object... args) {
            this.body = body;
            this.diag = diags.error(diagSource, this, code, args);
        }

        @Override // com.sun.source.doctree.DocTree
        public DocTree.Kind getKind() {
            return DocTree.Kind.ERRONEOUS;
        }

        @Override // com.sun.source.doctree.DocTree
        public <R, D> R accept(DocTreeVisitor<R, D> v, D d) {
            return v.visitErroneous(this, d);
        }

        @Override // com.sun.source.doctree.TextTree
        public String getBody() {
            return this.body;
        }

        @Override // com.sun.source.doctree.ErroneousTree
        public Diagnostic<JavaFileObject> getDiagnostic() {
            return this.diag;
        }

        @Override // com.sun.tools.javac.util.JCDiagnostic.DiagnosticPosition
        public JCTree getTree() {
            return null;
        }

        @Override // com.sun.tools.javac.util.JCDiagnostic.DiagnosticPosition
        public int getStartPosition() {
            return this.pos;
        }

        @Override // com.sun.tools.javac.util.JCDiagnostic.DiagnosticPosition
        public int getPreferredPosition() {
            return (this.pos + this.body.length()) - 1;
        }

        @Override // com.sun.tools.javac.util.JCDiagnostic.DiagnosticPosition
        public int getEndPosition(EndPosTable endPosTable) {
            return this.pos + this.body.length();
        }
    }

    public static class DCIdentifier extends DCTree implements IdentifierTree {
        public final Name name;

        DCIdentifier(Name name) {
            this.name = name;
        }

        @Override // com.sun.source.doctree.DocTree
        public DocTree.Kind getKind() {
            return DocTree.Kind.IDENTIFIER;
        }

        @Override // com.sun.source.doctree.DocTree
        public <R, D> R accept(DocTreeVisitor<R, D> v, D d) {
            return v.visitIdentifier(this, d);
        }

        @Override // com.sun.source.doctree.IdentifierTree
        public Name getName() {
            return this.name;
        }
    }

    public static class DCInheritDoc extends DCInlineTag implements InheritDocTree {
        @Override // com.sun.source.doctree.DocTree
        public DocTree.Kind getKind() {
            return DocTree.Kind.INHERIT_DOC;
        }

        @Override // com.sun.source.doctree.DocTree
        public <R, D> R accept(DocTreeVisitor<R, D> v, D d) {
            return v.visitInheritDoc(this, d);
        }
    }

    public static class DCLink extends DCInlineTag implements LinkTree {
        public final DocTree.Kind kind;
        public final List<DCTree> label;
        public final DCReference ref;

        DCLink(DocTree.Kind kind, DCReference ref, List<DCTree> label) {
            Assert.check(kind == DocTree.Kind.LINK || kind == DocTree.Kind.LINK_PLAIN);
            this.kind = kind;
            this.ref = ref;
            this.label = label;
        }

        @Override // com.sun.source.doctree.DocTree
        public DocTree.Kind getKind() {
            return this.kind;
        }

        @Override // com.sun.source.doctree.DocTree
        public <R, D> R accept(DocTreeVisitor<R, D> v, D d) {
            return v.visitLink(this, d);
        }

        @Override // com.sun.source.doctree.LinkTree
        public ReferenceTree getReference() {
            return this.ref;
        }

        @Override // com.sun.source.doctree.LinkTree
        public List<? extends DocTree> getLabel() {
            return this.label;
        }
    }

    public static class DCLiteral extends DCInlineTag implements LiteralTree {
        public final DCText body;
        public final DocTree.Kind kind;

        DCLiteral(DocTree.Kind kind, DCText body) {
            Assert.check(kind == DocTree.Kind.CODE || kind == DocTree.Kind.LITERAL);
            this.kind = kind;
            this.body = body;
        }

        @Override // com.sun.source.doctree.DocTree
        public DocTree.Kind getKind() {
            return this.kind;
        }

        @Override // com.sun.source.doctree.DocTree
        public <R, D> R accept(DocTreeVisitor<R, D> v, D d) {
            return v.visitLiteral(this, d);
        }

        @Override // com.sun.source.doctree.LiteralTree
        public DCText getBody() {
            return this.body;
        }
    }

    public static class DCParam extends DCBlockTag implements ParamTree {
        public final List<DCTree> description;
        public final boolean isTypeParameter;
        public final DCIdentifier name;

        DCParam(boolean isTypeParameter, DCIdentifier name, List<DCTree> description) {
            this.isTypeParameter = isTypeParameter;
            this.name = name;
            this.description = description;
        }

        @Override // com.sun.source.doctree.DocTree
        public DocTree.Kind getKind() {
            return DocTree.Kind.PARAM;
        }

        @Override // com.sun.source.doctree.DocTree
        public <R, D> R accept(DocTreeVisitor<R, D> v, D d) {
            return v.visitParam(this, d);
        }

        @Override // com.sun.source.doctree.ParamTree
        public boolean isTypeParameter() {
            return this.isTypeParameter;
        }

        @Override // com.sun.source.doctree.ParamTree
        public IdentifierTree getName() {
            return this.name;
        }

        @Override // com.sun.source.doctree.ParamTree
        public List<? extends DocTree> getDescription() {
            return this.description;
        }
    }

    public static class DCReference extends DCEndPosTree<DCReference> implements ReferenceTree {
        public final Name memberName;
        public final List<JCTree> paramTypes;
        public final JCTree qualifierExpression;
        public final String signature;

        DCReference(String signature, JCTree qualExpr, Name member, List<JCTree> paramTypes) {
            this.signature = signature;
            this.qualifierExpression = qualExpr;
            this.memberName = member;
            this.paramTypes = paramTypes;
        }

        @Override // com.sun.source.doctree.DocTree
        public DocTree.Kind getKind() {
            return DocTree.Kind.REFERENCE;
        }

        @Override // com.sun.source.doctree.DocTree
        public <R, D> R accept(DocTreeVisitor<R, D> v, D d) {
            return v.visitReference(this, d);
        }

        @Override // com.sun.source.doctree.ReferenceTree
        public String getSignature() {
            return this.signature;
        }
    }

    public static class DCReturn extends DCBlockTag implements ReturnTree {
        public final List<DCTree> description;

        DCReturn(List<DCTree> description) {
            this.description = description;
        }

        @Override // com.sun.source.doctree.DocTree
        public DocTree.Kind getKind() {
            return DocTree.Kind.RETURN;
        }

        @Override // com.sun.source.doctree.DocTree
        public <R, D> R accept(DocTreeVisitor<R, D> v, D d) {
            return v.visitReturn(this, d);
        }

        @Override // com.sun.source.doctree.ReturnTree
        public List<? extends DocTree> getDescription() {
            return this.description;
        }
    }

    public static class DCSee extends DCBlockTag implements SeeTree {
        public final List<DCTree> reference;

        DCSee(List<DCTree> reference) {
            this.reference = reference;
        }

        @Override // com.sun.source.doctree.DocTree
        public DocTree.Kind getKind() {
            return DocTree.Kind.SEE;
        }

        @Override // com.sun.source.doctree.DocTree
        public <R, D> R accept(DocTreeVisitor<R, D> v, D d) {
            return v.visitSee(this, d);
        }

        @Override // com.sun.source.doctree.SeeTree
        public List<? extends DocTree> getReference() {
            return this.reference;
        }
    }

    public static class DCSerial extends DCBlockTag implements SerialTree {
        public final List<DCTree> description;

        DCSerial(List<DCTree> description) {
            this.description = description;
        }

        @Override // com.sun.source.doctree.DocTree
        public DocTree.Kind getKind() {
            return DocTree.Kind.SERIAL;
        }

        @Override // com.sun.source.doctree.DocTree
        public <R, D> R accept(DocTreeVisitor<R, D> v, D d) {
            return v.visitSerial(this, d);
        }

        @Override // com.sun.source.doctree.SerialTree
        public List<? extends DocTree> getDescription() {
            return this.description;
        }
    }

    public static class DCSerialData extends DCBlockTag implements SerialDataTree {
        public final List<DCTree> description;

        DCSerialData(List<DCTree> description) {
            this.description = description;
        }

        @Override // com.sun.source.doctree.DocTree
        public DocTree.Kind getKind() {
            return DocTree.Kind.SERIAL_DATA;
        }

        @Override // com.sun.source.doctree.DocTree
        public <R, D> R accept(DocTreeVisitor<R, D> v, D d) {
            return v.visitSerialData(this, d);
        }

        @Override // com.sun.source.doctree.SerialDataTree
        public List<? extends DocTree> getDescription() {
            return this.description;
        }
    }

    public static class DCSerialField extends DCBlockTag implements SerialFieldTree {
        public final List<DCTree> description;
        public final DCIdentifier name;
        public final DCReference type;

        DCSerialField(DCIdentifier name, DCReference type, List<DCTree> description) {
            this.description = description;
            this.name = name;
            this.type = type;
        }

        @Override // com.sun.source.doctree.DocTree
        public DocTree.Kind getKind() {
            return DocTree.Kind.SERIAL_FIELD;
        }

        @Override // com.sun.source.doctree.DocTree
        public <R, D> R accept(DocTreeVisitor<R, D> v, D d) {
            return v.visitSerialField(this, d);
        }

        @Override // com.sun.source.doctree.SerialFieldTree
        public List<? extends DocTree> getDescription() {
            return this.description;
        }

        @Override // com.sun.source.doctree.SerialFieldTree
        public IdentifierTree getName() {
            return this.name;
        }

        @Override // com.sun.source.doctree.SerialFieldTree
        public ReferenceTree getType() {
            return this.type;
        }
    }

    public static class DCSince extends DCBlockTag implements SinceTree {
        public final List<DCTree> body;

        DCSince(List<DCTree> body) {
            this.body = body;
        }

        @Override // com.sun.source.doctree.DocTree
        public DocTree.Kind getKind() {
            return DocTree.Kind.SINCE;
        }

        @Override // com.sun.source.doctree.DocTree
        public <R, D> R accept(DocTreeVisitor<R, D> v, D d) {
            return v.visitSince(this, d);
        }

        @Override // com.sun.source.doctree.SinceTree
        public List<? extends DocTree> getBody() {
            return this.body;
        }
    }

    public static class DCStartElement extends DCEndPosTree<DCStartElement> implements StartElementTree {
        public final List<DCTree> attrs;
        public final Name name;
        public final boolean selfClosing;

        DCStartElement(Name name, List<DCTree> attrs, boolean selfClosing) {
            this.name = name;
            this.attrs = attrs;
            this.selfClosing = selfClosing;
        }

        @Override // com.sun.source.doctree.DocTree
        public DocTree.Kind getKind() {
            return DocTree.Kind.START_ELEMENT;
        }

        @Override // com.sun.source.doctree.DocTree
        public <R, D> R accept(DocTreeVisitor<R, D> v, D d) {
            return v.visitStartElement(this, d);
        }

        @Override // com.sun.source.doctree.StartElementTree
        public Name getName() {
            return this.name;
        }

        @Override // com.sun.source.doctree.StartElementTree
        public List<? extends DocTree> getAttributes() {
            return this.attrs;
        }

        @Override // com.sun.source.doctree.StartElementTree
        public boolean isSelfClosing() {
            return this.selfClosing;
        }
    }

    public static class DCText extends DCTree implements TextTree {
        public final String text;

        DCText(String text) {
            this.text = text;
        }

        @Override // com.sun.source.doctree.DocTree
        public DocTree.Kind getKind() {
            return DocTree.Kind.TEXT;
        }

        @Override // com.sun.source.doctree.DocTree
        public <R, D> R accept(DocTreeVisitor<R, D> v, D d) {
            return v.visitText(this, d);
        }

        @Override // com.sun.source.doctree.TextTree
        public String getBody() {
            return this.text;
        }
    }

    public static class DCThrows extends DCBlockTag implements ThrowsTree {
        public final List<DCTree> description;
        public final DocTree.Kind kind;
        public final DCReference name;

        DCThrows(DocTree.Kind kind, DCReference name, List<DCTree> description) {
            Assert.check(kind == DocTree.Kind.EXCEPTION || kind == DocTree.Kind.THROWS);
            this.kind = kind;
            this.name = name;
            this.description = description;
        }

        @Override // com.sun.source.doctree.DocTree
        public DocTree.Kind getKind() {
            return this.kind;
        }

        @Override // com.sun.source.doctree.DocTree
        public <R, D> R accept(DocTreeVisitor<R, D> v, D d) {
            return v.visitThrows(this, d);
        }

        @Override // com.sun.source.doctree.ThrowsTree
        public ReferenceTree getExceptionName() {
            return this.name;
        }

        @Override // com.sun.source.doctree.ThrowsTree
        public List<? extends DocTree> getDescription() {
            return this.description;
        }
    }

    public static class DCUnknownBlockTag extends DCBlockTag implements UnknownBlockTagTree {
        public final List<DCTree> content;
        public final Name name;

        DCUnknownBlockTag(Name name, List<DCTree> content) {
            this.name = name;
            this.content = content;
        }

        @Override // com.sun.source.doctree.DocTree
        public DocTree.Kind getKind() {
            return DocTree.Kind.UNKNOWN_BLOCK_TAG;
        }

        @Override // com.sun.source.doctree.DocTree
        public <R, D> R accept(DocTreeVisitor<R, D> v, D d) {
            return v.visitUnknownBlockTag(this, d);
        }

        @Override // com.sun.tools.javac.tree.DCTree.DCBlockTag, com.sun.source.doctree.BlockTagTree
        public String getTagName() {
            return this.name.toString();
        }

        @Override // com.sun.source.doctree.UnknownBlockTagTree
        public List<? extends DocTree> getContent() {
            return this.content;
        }
    }

    public static class DCUnknownInlineTag extends DCInlineTag implements UnknownInlineTagTree {
        public final List<DCTree> content;
        public final Name name;

        DCUnknownInlineTag(Name name, List<DCTree> content) {
            this.name = name;
            this.content = content;
        }

        @Override // com.sun.source.doctree.DocTree
        public DocTree.Kind getKind() {
            return DocTree.Kind.UNKNOWN_INLINE_TAG;
        }

        @Override // com.sun.source.doctree.DocTree
        public <R, D> R accept(DocTreeVisitor<R, D> v, D d) {
            return v.visitUnknownInlineTag(this, d);
        }

        @Override // com.sun.tools.javac.tree.DCTree.DCInlineTag, com.sun.source.doctree.InlineTagTree
        public String getTagName() {
            return this.name.toString();
        }

        @Override // com.sun.source.doctree.UnknownInlineTagTree
        public List<? extends DocTree> getContent() {
            return this.content;
        }
    }

    public static class DCValue extends DCInlineTag implements ValueTree {
        public final DCReference ref;

        DCValue(DCReference ref) {
            this.ref = ref;
        }

        @Override // com.sun.source.doctree.DocTree
        public DocTree.Kind getKind() {
            return DocTree.Kind.VALUE;
        }

        @Override // com.sun.source.doctree.DocTree
        public <R, D> R accept(DocTreeVisitor<R, D> v, D d) {
            return v.visitValue(this, d);
        }

        @Override // com.sun.source.doctree.ValueTree
        public ReferenceTree getReference() {
            return this.ref;
        }
    }

    public static class DCVersion extends DCBlockTag implements VersionTree {
        public final List<DCTree> body;

        DCVersion(List<DCTree> body) {
            this.body = body;
        }

        @Override // com.sun.source.doctree.DocTree
        public DocTree.Kind getKind() {
            return DocTree.Kind.VERSION;
        }

        @Override // com.sun.source.doctree.DocTree
        public <R, D> R accept(DocTreeVisitor<R, D> v, D d) {
            return v.visitVersion(this, d);
        }

        @Override // com.sun.source.doctree.VersionTree
        public List<? extends DocTree> getBody() {
            return this.body;
        }
    }
}
