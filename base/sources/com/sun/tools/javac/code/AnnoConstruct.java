package com.sun.tools.javac.code;

import com.sun.tools.javac.code.Attribute;
import com.sun.tools.javac.model.AnnotationProxyMaker;
import com.sun.tools.javac.util.List;
import com.sun.tools.javac.util.ListBuffer;
import java.lang.annotation.Annotation;
import java.lang.annotation.Inherited;
import java.lang.reflect.Array;
import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import javax.lang.model.AnnotatedConstruct;

/* JADX INFO: loaded from: classes.dex */
public abstract class AnnoConstruct implements AnnotatedConstruct {
    private static final Class<? extends Annotation> REPEATABLE_CLASS = initRepeatable();
    private static final Method VALUE_ELEMENT_METHOD = initValueElementMethod();

    @Override // javax.lang.model.AnnotatedConstruct
    public abstract List<? extends Attribute.Compound> getAnnotationMirrors();

    protected <A extends Annotation> Attribute.Compound getAttribute(Class<A> annoType) {
        String name = annoType.getName();
        for (Attribute.Compound anno : getAnnotationMirrors()) {
            if (name.equals(anno.type.tsym.flatName().toString())) {
                return anno;
            }
        }
        return null;
    }

    protected <A extends Annotation> A[] getInheritedAnnotations(Class<A> cls) {
        return (A[]) ((Annotation[]) Array.newInstance((Class<?>) cls, 0));
    }

    /* JADX WARN: Multi-variable type inference failed */
    @Override // javax.lang.model.AnnotatedConstruct
    public <A extends Annotation> A[] getAnnotationsByType(Class<A> cls) {
        int i;
        if (!cls.isAnnotation()) {
            throw new IllegalArgumentException("Not an annotation type: " + cls);
        }
        Class<? extends Annotation> container = getContainer(cls);
        if (container == null) {
            Annotation annotation = getAnnotation(cls);
            A[] aArr = (A[]) ((Annotation[]) Array.newInstance((Class<?>) cls, annotation == null ? 0 : 1));
            if (annotation != null) {
                aArr[0] = annotation;
            }
            return aArr;
        }
        String name = cls.getName();
        String name2 = container.getName();
        int i2 = -1;
        int i3 = -1;
        Attribute.Compound compound = null;
        Attribute.Compound compound2 = null;
        int i4 = -1;
        for (Attribute.Compound compound3 : getAnnotationMirrors()) {
            i4++;
            if (compound3.type.tsym.flatName().contentEquals(name)) {
                i2 = i4;
                compound = compound3;
            } else if (name2 != null && compound3.type.tsym.flatName().contentEquals(name2)) {
                i3 = i4;
                compound2 = compound3;
            }
        }
        if (compound == null && compound2 == null && cls.isAnnotationPresent(Inherited.class)) {
            return (A[]) getInheritedAnnotations(cls);
        }
        Attribute.Compound[] compoundArrUnpackContained = unpackContained(compound2);
        if (compound == null && compoundArrUnpackContained.length == 0 && cls.isAnnotationPresent(Inherited.class)) {
            return (A[]) getInheritedAnnotations(cls);
        }
        A[] aArr2 = (A[]) ((Annotation[]) Array.newInstance((Class<?>) cls, (compound == null ? 0 : 1) + compoundArrUnpackContained.length));
        int length = aArr2.length;
        if (i2 >= 0 && i3 >= 0) {
            if (i2 >= i3) {
                aArr2[aArr2.length - 1] = AnnotationProxyMaker.generateAnnotation(compound, cls);
                i = 0;
                length--;
            } else {
                aArr2[0] = AnnotationProxyMaker.generateAnnotation(compound, cls);
                i = 1;
            }
        } else {
            if (i2 >= 0) {
                aArr2[0] = AnnotationProxyMaker.generateAnnotation(compound, cls);
                return aArr2;
            }
            i = 0;
        }
        int i5 = 0;
        while (true) {
            Class<? extends Annotation> cls2 = container;
            if (i5 + i < length) {
                aArr2[i + i5] = AnnotationProxyMaker.generateAnnotation(compoundArrUnpackContained[i5], cls);
                i5++;
                container = cls2;
                i = i;
            } else {
                return aArr2;
            }
        }
    }

    private Attribute.Compound[] unpackContained(Attribute.Compound container) {
        Attribute[] contained0 = null;
        if (container != null) {
            contained0 = unpackAttributes(container);
        }
        ListBuffer<Attribute.Compound> compounds = new ListBuffer<>();
        if (contained0 != null) {
            for (Attribute a : contained0) {
                if (a instanceof Attribute.Compound) {
                    compounds = compounds.append((Attribute.Compound) a);
                }
            }
        }
        return (Attribute.Compound[]) compounds.toArray(new Attribute.Compound[compounds.size()]);
    }

    @Override // javax.lang.model.AnnotatedConstruct
    public <A extends Annotation> A getAnnotation(Class<A> cls) {
        if (!cls.isAnnotation()) {
            throw new IllegalArgumentException("Not an annotation type: " + cls);
        }
        Attribute.Compound attribute = getAttribute(cls);
        if (attribute == null) {
            return null;
        }
        return (A) AnnotationProxyMaker.generateAnnotation(attribute, cls);
    }

    private static Class<? extends Annotation> initRepeatable() {
        try {
            return Class.forName("java.lang.annotation.Repeatable").asSubclass(Annotation.class);
        } catch (ClassNotFoundException | SecurityException e) {
            return null;
        }
    }

    private static Method initValueElementMethod() {
        if (REPEATABLE_CLASS == null) {
            return null;
        }
        try {
            Method m = REPEATABLE_CLASS.getMethod("value", new Class[0]);
            if (m != null) {
                m.setAccessible(true);
            }
            return m;
        } catch (NoSuchMethodException e) {
            return null;
        }
    }

    private static Class<? extends Annotation> getContainer(Class<? extends Annotation> annoType) {
        Annotation repeatable;
        if (REPEATABLE_CLASS == null || VALUE_ELEMENT_METHOD == null || (repeatable = annoType.getAnnotation(REPEATABLE_CLASS)) == null) {
            return null;
        }
        try {
            Class<? extends Annotation> containerType = (Class) VALUE_ELEMENT_METHOD.invoke(repeatable, new Object[0]);
            if (containerType == null) {
                return null;
            }
            return containerType;
        } catch (ClassCastException | IllegalAccessException | InvocationTargetException e) {
            return null;
        }
    }

    private static Attribute[] unpackAttributes(Attribute.Compound container) {
        return ((Attribute.Array) container.member(container.type.tsym.name.table.names.value)).values;
    }
}
