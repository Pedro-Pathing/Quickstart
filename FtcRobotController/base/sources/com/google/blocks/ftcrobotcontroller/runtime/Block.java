package com.google.blocks.ftcrobotcontroller.runtime;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

/* JADX INFO: loaded from: classes8.dex */
@Target({ElementType.METHOD})
@Retention(RetentionPolicy.RUNTIME)
public @interface Block {
    Class[] classes() default {};

    boolean constructor() default false;

    boolean exclusiveToBlocks() default false;

    String[] fieldName() default {};

    String[] methodName() default {};
}
