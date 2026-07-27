package com.android.tools.r8.dex.code;

import java.nio.ShortBuffer;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes.dex */
public final class B1 implements InterfaceC0037a {
    public static final /* synthetic */ boolean g = true;
    public final int a;
    public final int b;
    public final ShortBuffer c;
    public int e;
    public int d = 0;
    public boolean f = false;

    public B1(int i, int i2, ShortBuffer shortBuffer) {
        this.b = i;
        this.a = i2;
        this.c = shortBuffer;
    }

    public final int a() {
        if (this.f) {
            this.f = false;
            return this.e;
        }
        int iB = b();
        this.e = iB & 255;
        this.f = true;
        return (iB >> 8) & 255;
    }

    public final int b() {
        boolean z = g;
        if (!z && this.f) {
            throw new AssertionError("Unread byte in cache.");
        }
        if (!z && this.d >= this.a) {
            throw new AssertionError();
        }
        short s = this.c.get(this.b + this.d);
        this.d++;
        return s;
    }
}
