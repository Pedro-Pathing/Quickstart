package com.sun.tools.javac.tree;

import com.sun.source.doctree.AttributeTree;
import com.sun.source.doctree.DocTree;
import com.sun.tools.javac.parser.Tokens;
import com.sun.tools.javac.tree.DCTree;
import com.sun.tools.javac.util.Context;
import com.sun.tools.javac.util.DiagnosticSource;
import com.sun.tools.javac.util.JCDiagnostic;
import com.sun.tools.javac.util.List;
import com.sun.tools.javac.util.Name;

/* JADX INFO: loaded from: classes.dex */
public class DocTreeMaker {
    protected static final Context.Key<DocTreeMaker> treeMakerKey = new Context.Key<>();
    private final JCDiagnostic.Factory diags;
    public int pos;

    public static DocTreeMaker instance(Context context) {
        DocTreeMaker instance = (DocTreeMaker) context.get(treeMakerKey);
        if (instance == null) {
            return new DocTreeMaker(context);
        }
        return instance;
    }

    protected DocTreeMaker(Context context) {
        this.pos = -1;
        context.put(treeMakerKey, this);
        this.diags = JCDiagnostic.Factory.instance(context);
        this.pos = -1;
    }

    public DocTreeMaker at(int pos) {
        this.pos = pos;
        return this;
    }

    public DocTreeMaker at(JCDiagnostic.DiagnosticPosition pos) {
        this.pos = pos == null ? -1 : pos.getStartPosition();
        return this;
    }

    public DCTree.DCAttribute Attribute(Name name, AttributeTree.ValueKind vkind, List<DCTree> value) {
        DCTree.DCAttribute tree = new DCTree.DCAttribute(name, vkind, value);
        tree.pos = this.pos;
        return tree;
    }

    public DCTree.DCAuthor Author(List<DCTree> name) {
        DCTree.DCAuthor tree = new DCTree.DCAuthor(name);
        tree.pos = this.pos;
        return tree;
    }

    public DCTree.DCLiteral Code(DCTree.DCText text) {
        DCTree.DCLiteral tree = new DCTree.DCLiteral(DocTree.Kind.CODE, text);
        tree.pos = this.pos;
        return tree;
    }

    public DCTree.DCComment Comment(String text) {
        DCTree.DCComment tree = new DCTree.DCComment(text);
        tree.pos = this.pos;
        return tree;
    }

    public DCTree.DCDeprecated Deprecated(List<DCTree> text) {
        DCTree.DCDeprecated tree = new DCTree.DCDeprecated(text);
        tree.pos = this.pos;
        return tree;
    }

    public DCTree.DCDocComment DocComment(Tokens.Comment comment, List<DCTree> firstSentence, List<DCTree> body, List<DCTree> tags) {
        DCTree.DCDocComment tree = new DCTree.DCDocComment(comment, firstSentence, body, tags);
        tree.pos = this.pos;
        return tree;
    }

    public DCTree.DCDocRoot DocRoot() {
        DCTree.DCDocRoot tree = new DCTree.DCDocRoot();
        tree.pos = this.pos;
        return tree;
    }

    public DCTree.DCEndElement EndElement(Name name) {
        DCTree.DCEndElement tree = new DCTree.DCEndElement(name);
        tree.pos = this.pos;
        return tree;
    }

    public DCTree.DCEntity Entity(Name name) {
        DCTree.DCEntity tree = new DCTree.DCEntity(name);
        tree.pos = this.pos;
        return tree;
    }

    public DCTree.DCErroneous Erroneous(String text, DiagnosticSource diagSource, String code, Object... args) {
        DCTree.DCErroneous tree = new DCTree.DCErroneous(text, this.diags, diagSource, code, args);
        tree.pos = this.pos;
        return tree;
    }

    public DCTree.DCThrows Exception(DCTree.DCReference name, List<DCTree> description) {
        DCTree.DCThrows tree = new DCTree.DCThrows(DocTree.Kind.EXCEPTION, name, description);
        tree.pos = this.pos;
        return tree;
    }

    public DCTree.DCIdentifier Identifier(Name name) {
        DCTree.DCIdentifier tree = new DCTree.DCIdentifier(name);
        tree.pos = this.pos;
        return tree;
    }

    public DCTree.DCInheritDoc InheritDoc() {
        DCTree.DCInheritDoc tree = new DCTree.DCInheritDoc();
        tree.pos = this.pos;
        return tree;
    }

    public DCTree.DCLink Link(DCTree.DCReference ref, List<DCTree> label) {
        DCTree.DCLink tree = new DCTree.DCLink(DocTree.Kind.LINK, ref, label);
        tree.pos = this.pos;
        return tree;
    }

    public DCTree.DCLink LinkPlain(DCTree.DCReference ref, List<DCTree> label) {
        DCTree.DCLink tree = new DCTree.DCLink(DocTree.Kind.LINK_PLAIN, ref, label);
        tree.pos = this.pos;
        return tree;
    }

    public DCTree.DCLiteral Literal(DCTree.DCText text) {
        DCTree.DCLiteral tree = new DCTree.DCLiteral(DocTree.Kind.LITERAL, text);
        tree.pos = this.pos;
        return tree;
    }

    public DCTree.DCParam Param(boolean isTypeParameter, DCTree.DCIdentifier name, List<DCTree> description) {
        DCTree.DCParam tree = new DCTree.DCParam(isTypeParameter, name, description);
        tree.pos = this.pos;
        return tree;
    }

    public DCTree.DCReference Reference(String signature, JCTree qualExpr, Name member, List<JCTree> paramTypes) {
        DCTree.DCReference tree = new DCTree.DCReference(signature, qualExpr, member, paramTypes);
        tree.pos = this.pos;
        return tree;
    }

    public DCTree.DCReturn Return(List<DCTree> description) {
        DCTree.DCReturn tree = new DCTree.DCReturn(description);
        tree.pos = this.pos;
        return tree;
    }

    public DCTree.DCSee See(List<DCTree> reference) {
        DCTree.DCSee tree = new DCTree.DCSee(reference);
        tree.pos = this.pos;
        return tree;
    }

    public DCTree.DCSerial Serial(List<DCTree> description) {
        DCTree.DCSerial tree = new DCTree.DCSerial(description);
        tree.pos = this.pos;
        return tree;
    }

    public DCTree.DCSerialData SerialData(List<DCTree> description) {
        DCTree.DCSerialData tree = new DCTree.DCSerialData(description);
        tree.pos = this.pos;
        return tree;
    }

    public DCTree.DCSerialField SerialField(DCTree.DCIdentifier name, DCTree.DCReference type, List<DCTree> description) {
        DCTree.DCSerialField tree = new DCTree.DCSerialField(name, type, description);
        tree.pos = this.pos;
        return tree;
    }

    public DCTree.DCSince Since(List<DCTree> text) {
        DCTree.DCSince tree = new DCTree.DCSince(text);
        tree.pos = this.pos;
        return tree;
    }

    public DCTree.DCStartElement StartElement(Name name, List<DCTree> attrs, boolean selfClosing) {
        DCTree.DCStartElement tree = new DCTree.DCStartElement(name, attrs, selfClosing);
        tree.pos = this.pos;
        return tree;
    }

    public DCTree.DCText Text(String text) {
        DCTree.DCText tree = new DCTree.DCText(text);
        tree.pos = this.pos;
        return tree;
    }

    public DCTree.DCThrows Throws(DCTree.DCReference name, List<DCTree> description) {
        DCTree.DCThrows tree = new DCTree.DCThrows(DocTree.Kind.THROWS, name, description);
        tree.pos = this.pos;
        return tree;
    }

    public DCTree.DCUnknownBlockTag UnknownBlockTag(Name name, List<DCTree> content) {
        DCTree.DCUnknownBlockTag tree = new DCTree.DCUnknownBlockTag(name, content);
        tree.pos = this.pos;
        return tree;
    }

    public DCTree.DCUnknownInlineTag UnknownInlineTag(Name name, List<DCTree> content) {
        DCTree.DCUnknownInlineTag tree = new DCTree.DCUnknownInlineTag(name, content);
        tree.pos = this.pos;
        return tree;
    }

    public DCTree.DCValue Value(DCTree.DCReference ref) {
        DCTree.DCValue tree = new DCTree.DCValue(ref);
        tree.pos = this.pos;
        return tree;
    }

    public DCTree.DCVersion Version(List<DCTree> text) {
        DCTree.DCVersion tree = new DCTree.DCVersion(text);
        tree.pos = this.pos;
        return tree;
    }
}
