package com.android.tools.r8.ir.optimize;

import com.android.tools.r8.internal.AbstractC1076Vw;
import com.android.tools.r8.internal.C3161ul0;
import com.android.tools.r8.internal.InterfaceC2517ni0;
import com.android.tools.r8.internal.InterfaceC2604oi0;
import java.util.IdentityHashMap;
import java.util.Iterator;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.function.BiConsumer;
import java.util.function.Consumer;
import java.util.function.Predicate;

/* JADX INFO: renamed from: com.android.tools.r8.ir.optimize.k, reason: case insensitive filesystem */
/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public final class C3686k {
    public static final /* synthetic */ boolean b = true;
    public final Map a;

    public C3686k(LinkedHashMap linkedHashMap) {
        this.a = linkedHashMap;
    }

    public final void a(InterfaceC2517ni0 interfaceC2517ni0) {
        Iterator it = this.a.entrySet().iterator();
        while (it.hasNext()) {
            Map.Entry entry = (Map.Entry) it.next();
            AbstractC1076Vw abstractC1076Vw = (AbstractC1076Vw) entry.getKey();
            Map map = (Map) entry.getValue();
            Iterator it2 = map.entrySet().iterator();
            while (it2.hasNext()) {
                Map.Entry entry2 = (Map.Entry) it2.next();
                final C3161ul0 c3161ul0 = (C3161ul0) entry2.getKey();
                C3677i c3677i = (C3677i) entry2.getValue();
                AbstractC3675h abstractC3675h = c3677i.a;
                abstractC3675h.getClass();
                if (abstractC3675h instanceof C3687l) {
                    if (!b && !c3161ul0.c(new Predicate() { // from class: com.android.tools.r8.ir.optimize.k$$ExternalSyntheticLambda2
                        @Override // java.util.function.Predicate
                        public final boolean test(Object obj) {
                            return C3686k.a(c3161ul0, (AbstractC1076Vw) obj);
                        }
                    })) {
                        throw new AssertionError();
                    }
                } else {
                    if (!b && !(abstractC3675h instanceof C3691p)) {
                        throw new AssertionError();
                    }
                    AbstractC3675h abstractC3675h2 = (AbstractC3675h) interfaceC2517ni0.a(abstractC1076Vw, c3161ul0, c3677i);
                    abstractC3675h2.getClass();
                    if ((!(abstractC3675h2 instanceof C3689n) || c3161ul0.F()) && !(abstractC3675h2 instanceof C3691p)) {
                        c3677i.a = abstractC3675h2;
                    } else {
                        it2.remove();
                    }
                }
            }
            if (map.isEmpty()) {
                it.remove();
            }
        }
    }

    public static /* synthetic */ boolean a(C3161ul0 c3161ul0, AbstractC1076Vw abstractC1076Vw) {
        return abstractC1076Vw.d() == c3161ul0;
    }

    public final void a(IdentityHashMap identityHashMap) {
        identityHashMap.forEach(new BiConsumer() { // from class: com.android.tools.r8.ir.optimize.k$$ExternalSyntheticLambda1
            @Override // java.util.function.BiConsumer
            public final void accept(Object obj, Object obj2) {
                this.f$0.a((AbstractC1076Vw) obj, (Map) obj2);
            }
        });
    }

    public final /* synthetic */ void a(AbstractC1076Vw abstractC1076Vw, Map map) {
        final Map map2 = (Map) this.a.get(abstractC1076Vw);
        if (map2 != null) {
            map.keySet().forEach(new Consumer() { // from class: com.android.tools.r8.ir.optimize.k$$ExternalSyntheticLambda0
                @Override // java.util.function.Consumer
                public final void accept(Object obj) {
                    map2.remove((C3161ul0) obj);
                }
            });
            if (map2.isEmpty()) {
                this.a.remove(abstractC1076Vw);
            }
        }
    }

    public final void a(InterfaceC2604oi0 interfaceC2604oi0) {
        Iterator it = this.a.entrySet().iterator();
        while (it.hasNext()) {
            Map.Entry entry = (Map.Entry) it.next();
            AbstractC1076Vw abstractC1076Vw = (AbstractC1076Vw) entry.getKey();
            Map map = (Map) entry.getValue();
            Iterator it2 = map.entrySet().iterator();
            while (it2.hasNext()) {
                Map.Entry entry2 = (Map.Entry) it2.next();
                if (interfaceC2604oi0.a(abstractC1076Vw, (C3161ul0) entry2.getKey(), (C3677i) entry2.getValue())) {
                    it2.remove();
                }
            }
            if (map.isEmpty()) {
                it.remove();
            }
        }
    }
}
