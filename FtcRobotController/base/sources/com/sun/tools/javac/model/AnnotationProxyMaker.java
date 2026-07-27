package com.sun.tools.javac.model;

import com.sun.tools.javac.code.Attribute;
import com.sun.tools.javac.code.Scope;
import com.sun.tools.javac.code.Symbol;
import com.sun.tools.javac.code.Type;
import com.sun.tools.javac.util.List;
import com.sun.tools.javac.util.ListBuffer;
import com.sun.tools.javac.util.Name;
import com.sun.tools.javac.util.Pair;
import java.io.IOException;
import java.io.ObjectInputStream;
import java.lang.annotation.Annotation;
import java.lang.annotation.AnnotationTypeMismatchException;
import java.lang.reflect.Array;
import java.lang.reflect.Method;
import java.util.LinkedHashMap;
import java.util.Map;
import javax.lang.model.type.MirroredTypeException;
import javax.lang.model.type.MirroredTypesException;
import javax.lang.model.type.TypeMirror;
import sun.reflect.annotation.AnnotationParser;
import sun.reflect.annotation.AnnotationType;
import sun.reflect.annotation.EnumConstantNotPresentExceptionProxy;
import sun.reflect.annotation.ExceptionProxy;

/* JADX INFO: loaded from: classes.dex */
public class AnnotationProxyMaker {
    private final Attribute.Compound anno;
    private final Class<? extends Annotation> annoType;

    private AnnotationProxyMaker(Attribute.Compound anno, Class<? extends Annotation> annoType) {
        this.anno = anno;
        this.annoType = annoType;
    }

    public static <A extends Annotation> A generateAnnotation(Attribute.Compound anno, Class<A> annoType) {
        AnnotationProxyMaker apm = new AnnotationProxyMaker(anno, annoType);
        return annoType.cast(apm.generateAnnotation());
    }

    private Annotation generateAnnotation() {
        return AnnotationParser.annotationForMap(this.annoType, getAllReflectedValues());
    }

    private Map<String, Object> getAllReflectedValues() {
        Map<String, Object> res = new LinkedHashMap<>();
        for (Map.Entry<Symbol.MethodSymbol, Attribute> entry : getAllValues().entrySet()) {
            Symbol.MethodSymbol meth = entry.getKey();
            Object value = generateValue(meth, entry.getValue());
            if (value != null) {
                res.put(meth.name.toString(), value);
            }
        }
        return res;
    }

    private Map<Symbol.MethodSymbol, Attribute> getAllValues() {
        Symbol.MethodSymbol m;
        Attribute def;
        Map<Symbol.MethodSymbol, Attribute> res = new LinkedHashMap<>();
        Symbol.ClassSymbol sym = (Symbol.ClassSymbol) this.anno.type.tsym;
        for (Scope.Entry e = sym.members().elems; e != null; e = e.sibling) {
            if (e.sym.kind == 16 && (def = (m = (Symbol.MethodSymbol) e.sym).getDefaultValue()) != null) {
                res.put(m, def);
            }
        }
        for (Pair<Symbol.MethodSymbol, Attribute> p : this.anno.values) {
            res.put(p.fst, p.snd);
        }
        return res;
    }

    private Object generateValue(Symbol.MethodSymbol meth, Attribute attr) {
        ValueVisitor vv = new ValueVisitor(meth);
        return vv.getValue(attr);
    }

    private class ValueVisitor implements Attribute.Visitor {
        private Symbol.MethodSymbol meth;
        private Class<?> returnClass;
        private Object value;

        ValueVisitor(Symbol.MethodSymbol meth) {
            this.meth = meth;
        }

        Object getValue(Attribute attr) {
            try {
                Method method = AnnotationProxyMaker.this.annoType.getMethod(this.meth.name.toString(), new Class[0]);
                this.returnClass = method.getReturnType();
                attr.accept(this);
                if (!(this.value instanceof ExceptionProxy) && !AnnotationType.invocationHandlerReturnType(this.returnClass).isInstance(this.value)) {
                    typeMismatch(method, attr);
                }
                return this.value;
            } catch (NoSuchMethodException e) {
                return null;
            }
        }

        @Override // com.sun.tools.javac.code.Attribute.Visitor
        public void visitConstant(Attribute.Constant c) {
            this.value = c.getValue();
        }

        @Override // com.sun.tools.javac.code.Attribute.Visitor
        public void visitClass(Attribute.Class c) {
            this.value = new MirroredTypeExceptionProxy(c.classType);
        }

        @Override // com.sun.tools.javac.code.Attribute.Visitor
        public void visitArray(Attribute.Array a) {
            Name elemName = ((Type.ArrayType) a.type).elemtype.tsym.getQualifiedName();
            if (elemName.equals(elemName.table.names.java_lang_Class)) {
                ListBuffer<TypeMirror> elems = new ListBuffer<>();
                for (Attribute value : a.values) {
                    Type elem = ((Attribute.Class) value).classType;
                    elems.append(elem);
                }
                this.value = new MirroredTypesExceptionProxy(elems.toList());
                return;
            }
            int len = a.values.length;
            Class<?> returnClassSaved = this.returnClass;
            this.returnClass = this.returnClass.getComponentType();
            try {
                Object res = Array.newInstance(this.returnClass, len);
                for (int i = 0; i < len; i++) {
                    a.values[i].accept(this);
                    if (this.value == null || (this.value instanceof ExceptionProxy)) {
                        return;
                    }
                    Array.set(res, i, this.value);
                }
                this.value = res;
            } catch (IllegalArgumentException e) {
                this.value = null;
            } finally {
                this.returnClass = returnClassSaved;
            }
        }

        @Override // com.sun.tools.javac.code.Attribute.Visitor
        public void visitEnum(Attribute.Enum e) {
            if (this.returnClass.isEnum()) {
                String constName = e.value.toString();
                try {
                    this.value = Enum.valueOf(this.returnClass, constName);
                    return;
                } catch (IllegalArgumentException e2) {
                    this.value = new EnumConstantNotPresentExceptionProxy(this.returnClass, constName);
                    return;
                }
            }
            this.value = null;
        }

        @Override // com.sun.tools.javac.code.Attribute.Visitor
        public void visitCompound(Attribute.Compound c) {
            try {
                this.value = AnnotationProxyMaker.generateAnnotation(c, this.returnClass.asSubclass(Annotation.class));
            } catch (ClassCastException e) {
                this.value = null;
            }
        }

        @Override // com.sun.tools.javac.code.Attribute.Visitor
        public void visitError(Attribute.Error e) {
            if (e instanceof Attribute.UnresolvedClass) {
                this.value = new MirroredTypeExceptionProxy(((Attribute.UnresolvedClass) e).classType);
            } else {
                this.value = null;
            }
        }

        private void typeMismatch(Method method, Attribute attr) {
            this.value = new ExceptionProxy(method, attr) { // from class: com.sun.tools.javac.model.AnnotationProxyMaker.ValueVisitor.1AnnotationTypeMismatchExceptionProxy
                static final long serialVersionUID = 269;
                final transient Method method;
                final /* synthetic */ Attribute val$attr;

                {
                    this.val$attr = attr;
                    this.method = method;
                }

                public String toString() {
                    return "<error>";
                }

                protected RuntimeException generateException() {
                    return new AnnotationTypeMismatchException(this.method, this.val$attr.type.toString());
                }
            };
        }
    }

    private static final class MirroredTypeExceptionProxy extends ExceptionProxy {
        static final long serialVersionUID = 269;
        private transient TypeMirror type;
        private final String typeString;

        MirroredTypeExceptionProxy(TypeMirror t) {
            this.type = t;
            this.typeString = t.toString();
        }

        public String toString() {
            return this.typeString;
        }

        public int hashCode() {
            return (this.type != null ? this.type : this.typeString).hashCode();
        }

        public boolean equals(Object obj) {
            return this.type != null && (obj instanceof MirroredTypeExceptionProxy) && this.type.equals(((MirroredTypeExceptionProxy) obj).type);
        }

        protected RuntimeException generateException() {
            return new MirroredTypeException(this.type);
        }

        private void readObject(ObjectInputStream s) throws ClassNotFoundException, IOException {
            s.defaultReadObject();
            this.type = null;
        }
    }

    private static final class MirroredTypesExceptionProxy extends ExceptionProxy {
        static final long serialVersionUID = 269;
        private final String typeStrings;
        private transient List<TypeMirror> types;

        MirroredTypesExceptionProxy(List<TypeMirror> ts) {
            this.types = ts;
            this.typeStrings = ts.toString();
        }

        public String toString() {
            return this.typeStrings;
        }

        public int hashCode() {
            return (this.types != null ? this.types : this.typeStrings).hashCode();
        }

        public boolean equals(Object obj) {
            return this.types != null && (obj instanceof MirroredTypesExceptionProxy) && this.types.equals(((MirroredTypesExceptionProxy) obj).types);
        }

        protected RuntimeException generateException() {
            return new MirroredTypesException(this.types);
        }

        private void readObject(ObjectInputStream s) throws ClassNotFoundException, IOException {
            s.defaultReadObject();
            this.types = null;
        }
    }
}
