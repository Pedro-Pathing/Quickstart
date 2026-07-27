package com.sun.source.doctree;

import jdk.Exported;

/* JADX INFO: loaded from: classes.dex */
@Exported
public interface DocTree {
    <R, D> R accept(DocTreeVisitor<R, D> docTreeVisitor, D d);

    Kind getKind();

    @Exported
    public enum Kind {
        ATTRIBUTE,
        AUTHOR("author"),
        CODE("code"),
        COMMENT,
        DEPRECATED("deprecated"),
        DOC_COMMENT,
        DOC_ROOT("docRoot"),
        END_ELEMENT,
        ENTITY,
        ERRONEOUS,
        EXCEPTION("exception"),
        IDENTIFIER,
        INHERIT_DOC("inheritDoc"),
        LINK("link"),
        LINK_PLAIN("linkplain"),
        LITERAL("literal"),
        PARAM("param"),
        REFERENCE,
        RETURN("return"),
        SEE("see"),
        SERIAL("serial"),
        SERIAL_DATA("serialData"),
        SERIAL_FIELD("serialField"),
        SINCE("since"),
        START_ELEMENT,
        TEXT,
        THROWS("throws"),
        UNKNOWN_BLOCK_TAG,
        UNKNOWN_INLINE_TAG,
        VALUE("value"),
        VERSION("version"),
        OTHER;

        public final String tagName;

        Kind() {
            this.tagName = null;
        }

        Kind(String tagName) {
            this.tagName = tagName;
        }
    }
}
