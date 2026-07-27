package com.sun.tools.javac.util;

import com.sun.tools.doclint.DocLint;
import java.util.Objects;

/* JADX INFO: loaded from: classes.dex */
public class Pair<A, B> {
    public final A fst;
    public final B snd;

    public Pair(A fst, B snd) {
        this.fst = fst;
        this.snd = snd;
    }

    public String toString() {
        return "Pair[" + this.fst + DocLint.TAGS_SEPARATOR + this.snd + "]";
    }

    public boolean equals(Object other) {
        return (other instanceof Pair) && Objects.equals(this.fst, ((Pair) other).fst) && Objects.equals(this.snd, ((Pair) other).snd);
    }

    public int hashCode() {
        if (this.fst != null) {
            return this.snd == null ? this.fst.hashCode() + 2 : (this.fst.hashCode() * 17) + this.snd.hashCode();
        }
        if (this.snd == null) {
            return 0;
        }
        return this.snd.hashCode() + 1;
    }

    public static <A, B> Pair<A, B> of(A a, B b) {
        return new Pair<>(a, b);
    }
}
