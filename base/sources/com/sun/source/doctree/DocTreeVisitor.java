package com.sun.source.doctree;

import jdk.Exported;

/* JADX INFO: loaded from: classes.dex */
@Exported
public interface DocTreeVisitor<R, P> {
    R visitAttribute(AttributeTree attributeTree, P p);

    R visitAuthor(AuthorTree authorTree, P p);

    R visitComment(CommentTree commentTree, P p);

    R visitDeprecated(DeprecatedTree deprecatedTree, P p);

    R visitDocComment(DocCommentTree docCommentTree, P p);

    R visitDocRoot(DocRootTree docRootTree, P p);

    R visitEndElement(EndElementTree endElementTree, P p);

    R visitEntity(EntityTree entityTree, P p);

    R visitErroneous(ErroneousTree erroneousTree, P p);

    R visitIdentifier(IdentifierTree identifierTree, P p);

    R visitInheritDoc(InheritDocTree inheritDocTree, P p);

    R visitLink(LinkTree linkTree, P p);

    R visitLiteral(LiteralTree literalTree, P p);

    R visitOther(DocTree docTree, P p);

    R visitParam(ParamTree paramTree, P p);

    R visitReference(ReferenceTree referenceTree, P p);

    R visitReturn(ReturnTree returnTree, P p);

    R visitSee(SeeTree seeTree, P p);

    R visitSerial(SerialTree serialTree, P p);

    R visitSerialData(SerialDataTree serialDataTree, P p);

    R visitSerialField(SerialFieldTree serialFieldTree, P p);

    R visitSince(SinceTree sinceTree, P p);

    R visitStartElement(StartElementTree startElementTree, P p);

    R visitText(TextTree textTree, P p);

    R visitThrows(ThrowsTree throwsTree, P p);

    R visitUnknownBlockTag(UnknownBlockTagTree unknownBlockTagTree, P p);

    R visitUnknownInlineTag(UnknownInlineTagTree unknownInlineTagTree, P p);

    R visitValue(ValueTree valueTree, P p);

    R visitVersion(VersionTree versionTree, P p);
}
