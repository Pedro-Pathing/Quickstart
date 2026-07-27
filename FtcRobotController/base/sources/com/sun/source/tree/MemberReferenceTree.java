package com.sun.source.tree;

import java.util.List;
import javax.lang.model.element.Name;
import jdk.Exported;

/* JADX INFO: loaded from: classes.dex */
@Exported
public interface MemberReferenceTree extends ExpressionTree {

    @Exported
    public enum ReferenceMode {
        INVOKE,
        NEW
    }

    ReferenceMode getMode();

    Name getName();

    ExpressionTree getQualifierExpression();

    List<? extends ExpressionTree> getTypeArguments();
}
