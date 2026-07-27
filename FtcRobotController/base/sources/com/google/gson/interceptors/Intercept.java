package com.google.gson.interceptors;

import java.lang.annotation.ElementType;
import java.lang.annotation.Retention;
import java.lang.annotation.RetentionPolicy;
import java.lang.annotation.Target;

/* JADX INFO: loaded from: classes8.dex */
@Target({ElementType.TYPE})
@Retention(RetentionPolicy.RUNTIME)
public @interface Intercept {
    Class<? extends JsonPostDeserializer> postDeserialize();
}
