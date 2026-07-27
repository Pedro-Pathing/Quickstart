package com.sun.tools.javac.jvm;

import com.sun.tools.javac.code.Type;
import com.sun.tools.javac.code.TypeTag;

/* JADX INFO: loaded from: classes.dex */
class UninitializedType extends Type.DelegatedType {
    public final int offset;

    public static UninitializedType uninitializedThis(Type qtype) {
        return new UninitializedType(TypeTag.UNINITIALIZED_THIS, qtype, -1);
    }

    public static UninitializedType uninitializedObject(Type qtype, int offset) {
        return new UninitializedType(TypeTag.UNINITIALIZED_OBJECT, qtype, offset);
    }

    private UninitializedType(TypeTag tag, Type qtype, int offset) {
        super(tag, qtype);
        this.offset = offset;
    }

    Type initializedType() {
        return this.qtype;
    }
}
