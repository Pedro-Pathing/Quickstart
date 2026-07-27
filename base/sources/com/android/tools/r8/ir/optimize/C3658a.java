package com.android.tools.r8.ir.optimize;

import com.android.tools.r8.graph.C0421y;
import com.android.tools.r8.internal.AbstractC2990sv;
import com.android.tools.r8.internal.AbstractC3424xb0;
import com.android.tools.r8.internal.C0874Ot;
import com.android.tools.r8.internal.C0996Tg;
import com.android.tools.r8.internal.C2881rj0;
import com.android.tools.r8.internal.C3161ul0;
import com.android.tools.r8.internal.Z40;
import java.util.Collection;
import java.util.Iterator;
import java.util.Set;
import java.util.function.Consumer;
import java.util.function.Predicate;

/* JADX INFO: renamed from: com.android.tools.r8.ir.optimize.a, reason: case insensitive filesystem */
/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public class C3658a implements Set<C3161ul0> {
    public static final C3658a c;
    public final Set b;

    static {
        int i = AbstractC2990sv.c;
        c = new C3658a(Z40.j);
    }

    public C3658a() {
        this.b = AbstractC3424xb0.c();
    }

    public final void a(C0421y c0421y, C0874Ot c0874Ot) {
        a(c0421y, c0874Ot, C0996Tg.b());
    }

    @Override // java.util.Set, java.util.Collection
    public final boolean add(Object obj) {
        return this.b.add((C3161ul0) obj);
    }

    @Override // java.util.Set, java.util.Collection
    public final boolean addAll(Collection collection) {
        return this.b.addAll(collection);
    }

    public final void b(C0421y c0421y, C0874Ot c0874Ot, Consumer consumer) {
        if (this.b.isEmpty()) {
            return;
        }
        C2881rj0 c2881rj0 = new C2881rj0(c0421y, c0874Ot, false);
        consumer.accept(c2881rj0);
        Consumer consumerB = C0996Tg.b();
        c2881rj0.a(this, 4);
        c2881rj0.a(consumerB);
        this.b.clear();
    }

    @Override // java.util.Set, java.util.Collection
    public final void clear() {
        this.b.clear();
    }

    @Override // java.util.Set, java.util.Collection
    public final boolean contains(Object obj) {
        return this.b.contains(obj);
    }

    @Override // java.util.Set, java.util.Collection
    public final boolean containsAll(Collection collection) {
        return this.b.containsAll(collection);
    }

    @Override // java.util.Set, java.util.Collection
    public final boolean isEmpty() {
        return this.b.isEmpty();
    }

    @Override // java.util.Set, java.util.Collection, java.lang.Iterable
    public final Iterator iterator() {
        return this.b.iterator();
    }

    @Override // java.util.Set, java.util.Collection
    public final boolean remove(Object obj) {
        return this.b.remove(obj);
    }

    @Override // java.util.Set, java.util.Collection
    public final boolean removeAll(Collection collection) {
        return this.b.removeAll(collection);
    }

    @Override // java.util.Set, java.util.Collection
    public final boolean retainAll(Collection collection) {
        return this.b.retainAll(collection);
    }

    @Override // java.util.Set, java.util.Collection
    public final int size() {
        return this.b.size();
    }

    @Override // java.util.Set, java.util.Collection
    public final Object[] toArray() {
        return this.b.toArray();
    }

    public final boolean a(C3161ul0 c3161ul0) {
        return this.b.add(c3161ul0);
    }

    @Override // java.util.Set, java.util.Collection
    public final Object[] toArray(Object[] objArr) {
        return this.b.toArray(objArr);
    }

    public final void a(Predicate predicate, C3161ul0 c3161ul0) {
        for (C3161ul0 c3161ul02 : c3161ul0.a().b) {
            if (c3161ul02.w() && !predicate.test(c3161ul02.b())) {
                this.b.add(c3161ul02);
            }
        }
    }

    public C3658a(Set set) {
        this.b = set;
    }

    public final void a(C0421y c0421y, C0874Ot c0874Ot, Consumer consumer) {
        if (this.b.isEmpty()) {
            return;
        }
        C2881rj0 c2881rj0 = new C2881rj0(c0421y, c0874Ot, false);
        consumer.accept(c2881rj0);
        c2881rj0.a(this, C0996Tg.b());
        this.b.clear();
    }
}
