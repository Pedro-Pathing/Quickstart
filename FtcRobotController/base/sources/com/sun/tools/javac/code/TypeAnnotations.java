package com.sun.tools.javac.code;

import com.sun.source.tree.Tree;
import com.sun.tools.javac.code.Attribute;
import com.sun.tools.javac.code.Symbol;
import com.sun.tools.javac.code.Type;
import com.sun.tools.javac.code.TypeAnnotationPosition;
import com.sun.tools.javac.comp.Annotate;
import com.sun.tools.javac.comp.Attr;
import com.sun.tools.javac.comp.AttrContext;
import com.sun.tools.javac.comp.Env;
import com.sun.tools.javac.tree.JCTree;
import com.sun.tools.javac.tree.TreeInfo;
import com.sun.tools.javac.tree.TreeScanner;
import com.sun.tools.javac.util.Assert;
import com.sun.tools.javac.util.Context;
import com.sun.tools.javac.util.List;
import com.sun.tools.javac.util.ListBuffer;
import com.sun.tools.javac.util.Log;
import com.sun.tools.javac.util.Names;
import com.sun.tools.javac.util.Options;
import javax.lang.model.element.Element;
import javax.lang.model.element.ElementKind;
import javax.lang.model.type.TypeKind;
import javax.tools.JavaFileObject;

/* JADX INFO: loaded from: classes.dex */
public class TypeAnnotations {
    protected static final Context.Key<TypeAnnotations> typeAnnosKey = new Context.Key<>();
    final Annotate annotate;
    final Attr attr;
    final Log log;
    final Names names;
    final Symtab syms;

    public enum AnnotationType {
        DECLARATION,
        TYPE,
        BOTH
    }

    public static TypeAnnotations instance(Context context) {
        TypeAnnotations instance = (TypeAnnotations) context.get(typeAnnosKey);
        if (instance == null) {
            return new TypeAnnotations(context);
        }
        return instance;
    }

    protected TypeAnnotations(Context context) {
        context.put(typeAnnosKey, this);
        this.names = Names.instance(context);
        this.log = Log.instance(context);
        this.syms = Symtab.instance(context);
        this.annotate = Annotate.instance(context);
        this.attr = Attr.instance(context);
        Options.instance(context);
    }

    public void organizeTypeAnnotationsSignatures(final Env<AttrContext> env, final JCTree.JCClassDecl tree) {
        this.annotate.afterRepeated(new Annotate.Worker() { // from class: com.sun.tools.javac.code.TypeAnnotations.1
            @Override // com.sun.tools.javac.comp.Annotate.Worker
            public void run() {
                JavaFileObject oldSource = TypeAnnotations.this.log.useSource(env.toplevel.sourcefile);
                try {
                    TypeAnnotations.this.new TypeAnnotationPositions(true).scan(tree);
                } finally {
                    TypeAnnotations.this.log.useSource(oldSource);
                }
            }
        });
    }

    public void validateTypeAnnotationsSignatures(final Env<AttrContext> env, final JCTree.JCClassDecl tree) {
        this.annotate.validate(new Annotate.Worker() { // from class: com.sun.tools.javac.code.TypeAnnotations.2
            @Override // com.sun.tools.javac.comp.Annotate.Worker
            public void run() {
                JavaFileObject oldSource = TypeAnnotations.this.log.useSource(env.toplevel.sourcefile);
                try {
                    TypeAnnotations.this.attr.validateTypeAnnotations(tree, true);
                } finally {
                    TypeAnnotations.this.log.useSource(oldSource);
                }
            }
        });
    }

    public void organizeTypeAnnotationsBodies(JCTree.JCClassDecl tree) {
        new TypeAnnotationPositions(false).scan(tree);
    }

    public AnnotationType annotationType(Attribute.Compound a, Symbol s) {
        Attribute.Compound atTarget;
        Attribute.Compound atTarget2 = a.type.tsym.attribute(this.syms.annotationTargetType.tsym);
        if (atTarget2 == null) {
            return inferTargetMetaInfo(a, s);
        }
        Attribute atValue = atTarget2.member(this.names.value);
        if (!(atValue instanceof Attribute.Array)) {
            Assert.error("annotationType(): bad @Target argument " + atValue + " (" + atValue.getClass() + ")");
            return AnnotationType.DECLARATION;
        }
        Attribute.Array arr = (Attribute.Array) atValue;
        boolean isDecl = false;
        boolean isType = false;
        Attribute[] attributeArr = arr.values;
        int length = attributeArr.length;
        int i = 0;
        while (i < length) {
            Attribute app = attributeArr[i];
            if (!(app instanceof Attribute.Enum)) {
                Assert.error("annotationType(): unrecognized Attribute kind " + app + " (" + app.getClass() + ")");
                isDecl = true;
                atTarget = atTarget2;
            } else {
                Attribute.Enum e = (Attribute.Enum) app;
                atTarget = atTarget2;
                if (e.value.name != this.names.TYPE) {
                    if (e.value.name != this.names.FIELD) {
                        if (e.value.name != this.names.METHOD) {
                            if (e.value.name != this.names.PARAMETER) {
                                if (e.value.name != this.names.CONSTRUCTOR) {
                                    if (e.value.name != this.names.LOCAL_VARIABLE) {
                                        if (e.value.name != this.names.ANNOTATION_TYPE) {
                                            if (e.value.name != this.names.PACKAGE) {
                                                if (e.value.name != this.names.TYPE_USE) {
                                                    if (e.value.name != this.names.TYPE_PARAMETER) {
                                                        Assert.error("annotationType(): unrecognized Attribute name " + ((Object) e.value.name) + " (" + e.value.name.getClass() + ")");
                                                        isDecl = true;
                                                    }
                                                } else if (s.kind == 2 || s.kind == 4 || ((s.kind == 16 && !s.isConstructor() && !s.type.mo178getReturnType().hasTag(TypeTag.VOID)) || (s.kind == 16 && s.isConstructor()))) {
                                                    isType = true;
                                                }
                                            } else if (s.kind == 1) {
                                                isDecl = true;
                                            }
                                        } else if (s.kind == 2 && (s.flags() & 8192) != 0) {
                                            isDecl = true;
                                        }
                                    } else if (s.kind == 4 && s.owner.kind == 16 && (s.flags() & 8589934592L) == 0) {
                                        isDecl = true;
                                    }
                                } else if (s.kind == 16 && s.isConstructor()) {
                                    isDecl = true;
                                }
                            } else if (s.kind == 4 && s.owner.kind == 16 && (s.flags() & 8589934592L) != 0) {
                                isDecl = true;
                            }
                        } else if (s.kind == 16 && !s.isConstructor()) {
                            isDecl = true;
                        }
                    } else if (s.kind == 4 && s.owner.kind != 16) {
                        isDecl = true;
                    }
                } else if (s.kind == 2) {
                    isDecl = true;
                }
            }
            i++;
            atTarget2 = atTarget;
        }
        if (isDecl && isType) {
            return AnnotationType.BOTH;
        }
        if (isType) {
            return AnnotationType.TYPE;
        }
        return AnnotationType.DECLARATION;
    }

    private static AnnotationType inferTargetMetaInfo(Attribute.Compound a, Symbol s) {
        return AnnotationType.DECLARATION;
    }

    private class TypeAnnotationPositions extends TreeScanner {
        private final boolean sigOnly;
        private ListBuffer<JCTree> frames = new ListBuffer<>();
        private boolean isInClass = false;
        private JCTree.JCLambda currentLambda = null;

        TypeAnnotationPositions(boolean sigOnly) {
            this.sigOnly = sigOnly;
        }

        protected void push(JCTree t) {
            this.frames = this.frames.prepend(t);
        }

        protected JCTree pop() {
            return this.frames.next();
        }

        private JCTree peek2() {
            return this.frames.toList().tail.head;
        }

        @Override // com.sun.tools.javac.tree.TreeScanner
        public void scan(JCTree tree) {
            push(tree);
            super.scan(tree);
            pop();
        }

        private void separateAnnotationsKinds(JCTree typetree, Type type, Symbol sym, TypeAnnotationPosition pos) {
            List<Attribute.Compound> annotations = sym.getRawAttributes();
            ListBuffer<Attribute.Compound> declAnnos = new ListBuffer<>();
            ListBuffer<Attribute.TypeCompound> typeAnnos = new ListBuffer<>();
            ListBuffer<Attribute.TypeCompound> onlyTypeAnnos = new ListBuffer<>();
            for (Attribute.Compound a : annotations) {
                switch (TypeAnnotations.this.annotationType(a, sym)) {
                    case DECLARATION:
                        declAnnos.append(a);
                        break;
                    case BOTH:
                        declAnnos.append(a);
                        typeAnnos.append(toTypeCompound(a, pos));
                        break;
                    case TYPE:
                        Attribute.TypeCompound ta = toTypeCompound(a, pos);
                        typeAnnos.append(ta);
                        onlyTypeAnnos.append(ta);
                        break;
                }
            }
            sym.resetAnnotations();
            sym.setDeclarationAttributes(declAnnos.toList());
            if (typeAnnos.isEmpty()) {
                return;
            }
            List<Attribute.TypeCompound> typeAnnotations = typeAnnos.toList();
            if (type == null) {
                typeWithAnnotations(typetree, sym.getEnclosingElement().asType(), typeAnnotations, typeAnnotations);
                sym.appendUniqueTypeAttributes(typeAnnotations);
                return;
            }
            Type type2 = typeWithAnnotations(typetree, type, typeAnnotations, onlyTypeAnnos.toList());
            if (sym.getKind() == ElementKind.METHOD) {
                sym.type.asMethodType().restype = type2;
            } else if (sym.getKind() == ElementKind.PARAMETER) {
                sym.type = type2;
                if (sym.getQualifiedName().equals(TypeAnnotations.this.names._this)) {
                    sym.owner.type.asMethodType().recvtype = type2;
                } else {
                    Type.MethodType methType = sym.owner.type.asMethodType();
                    List list = methType.argtypes;
                    ListBuffer listBuffer = new ListBuffer();
                    for (List list2 = ((Symbol.MethodSymbol) sym.owner).params; list2.nonEmpty(); list2 = list2.tail) {
                        if (list2.head == sym) {
                            listBuffer.add(type2);
                        } else {
                            listBuffer.add(list.head);
                        }
                        list = list.tail;
                    }
                    methType.argtypes = listBuffer.toList();
                }
            } else {
                sym.type = type2;
            }
            sym.appendUniqueTypeAttributes(typeAnnotations);
            if (sym.getKind() == ElementKind.PARAMETER || sym.getKind() == ElementKind.LOCAL_VARIABLE || sym.getKind() == ElementKind.RESOURCE_VARIABLE || sym.getKind() == ElementKind.EXCEPTION_PARAMETER) {
                sym.owner.appendUniqueTypeAttributes(sym.getRawTypeAttributes());
            }
        }

        /* JADX WARN: Can't fix incorrect switch cases order, some code will duplicate */
        private Type typeWithAnnotations(JCTree typetree, Type type, List<Attribute.TypeCompound> annotations, List<Attribute.TypeCompound> onlyTypeAnnotations) {
            Type toreturn;
            if (annotations.isEmpty()) {
                return type;
            }
            if (type.hasTag(TypeTag.ARRAY)) {
                Type.ArrayType arType = (Type.ArrayType) type.unannotatedType();
                Type.ArrayType tomodify = new Type.ArrayType(null, arType.tsym);
                if (type.isAnnotated()) {
                    toreturn = tomodify.annotatedType(type.getAnnotationMirrors());
                } else {
                    toreturn = tomodify;
                }
                JCTree.JCArrayTypeTree arTree = arrayTypeTree(typetree);
                ListBuffer<TypeAnnotationPosition.TypePathEntry> depth = new ListBuffer<>();
                ListBuffer<TypeAnnotationPosition.TypePathEntry> depth2 = depth.append(TypeAnnotationPosition.TypePathEntry.ARRAY);
                while (arType.elemtype.hasTag(TypeTag.ARRAY)) {
                    if (arType.elemtype.isAnnotated()) {
                        Type aelemtype = arType.elemtype;
                        arType = (Type.ArrayType) aelemtype.unannotatedType();
                        Type.ArrayType prevToMod = tomodify;
                        tomodify = new Type.ArrayType(null, arType.tsym);
                        prevToMod.elemtype = tomodify.annotatedType(arType.elemtype.getAnnotationMirrors());
                    } else {
                        arType = (Type.ArrayType) arType.elemtype;
                        tomodify.elemtype = new Type.ArrayType(null, arType.tsym);
                        tomodify = (Type.ArrayType) tomodify.elemtype;
                    }
                    arTree = arrayTypeTree(arTree.elemtype);
                    depth2 = depth2.append(TypeAnnotationPosition.TypePathEntry.ARRAY);
                }
                Type arelemType = typeWithAnnotations(arTree.elemtype, arType.elemtype, annotations, onlyTypeAnnotations);
                tomodify.elemtype = arelemType;
                Attribute.TypeCompound a = annotations.get(0);
                TypeAnnotationPosition p = a.position;
                p.location = p.location.prependList(depth2.toList());
                typetree.type = toreturn;
                return toreturn;
            }
            if (type.hasTag(TypeTag.TYPEVAR)) {
                return type;
            }
            if (type.getKind() == TypeKind.UNION) {
                JCTree.JCTypeUnion tutree = (JCTree.JCTypeUnion) typetree;
                JCTree.JCExpression fst = tutree.alternatives.get(0);
                Type res = typeWithAnnotations(fst, fst.type, annotations, onlyTypeAnnotations);
                fst.type = res;
                return type;
            }
            Type enclTy = type;
            Element enclEl = type.asElement();
            JCTree enclTr = typetree;
            while (enclEl != null && enclEl.getKind() != ElementKind.PACKAGE && enclTy != null && enclTy.getKind() != TypeKind.NONE && enclTy.getKind() != TypeKind.ERROR && (enclTr.getKind() == Tree.Kind.MEMBER_SELECT || enclTr.getKind() == Tree.Kind.PARAMETERIZED_TYPE || enclTr.getKind() == Tree.Kind.ANNOTATED_TYPE)) {
                if (enclTr.getKind() == Tree.Kind.MEMBER_SELECT) {
                    enclTy = enclTy.getEnclosingType();
                    enclEl = enclEl.getEnclosingElement();
                    enclTr = ((JCTree.JCFieldAccess) enclTr).getExpression();
                } else if (enclTr.getKind() == Tree.Kind.PARAMETERIZED_TYPE) {
                    enclTr = ((JCTree.JCTypeApply) enclTr).getType();
                } else {
                    enclTr = ((JCTree.JCAnnotatedType) enclTr).getUnderlyingType();
                }
            }
            if (enclTy != null && enclTy.hasTag(TypeTag.NONE)) {
                switch (onlyTypeAnnotations.size()) {
                    case 0:
                        return type;
                    case 1:
                        TypeAnnotations.this.log.error(typetree.pos(), "cant.type.annotate.scoping.1", onlyTypeAnnotations);
                        return type;
                    default:
                        TypeAnnotations.this.log.error(typetree.pos(), "cant.type.annotate.scoping", onlyTypeAnnotations);
                        return type;
                }
            }
            ListBuffer<TypeAnnotationPosition.TypePathEntry> depth3 = new ListBuffer<>();
            Type topTy = enclTy;
            while (enclEl != null && enclEl.getKind() != ElementKind.PACKAGE && topTy != null && topTy.getKind() != TypeKind.NONE && topTy.getKind() != TypeKind.ERROR) {
                topTy = topTy.getEnclosingType();
                enclEl = enclEl.getEnclosingElement();
                if (topTy != null && topTy.getKind() != TypeKind.NONE) {
                    depth3 = depth3.append(TypeAnnotationPosition.TypePathEntry.INNER_TYPE);
                }
            }
            if (depth3.nonEmpty()) {
                Attribute.TypeCompound a2 = annotations.get(0);
                TypeAnnotationPosition p2 = a2.position;
                p2.location = p2.location.appendList(depth3.toList());
            }
            Type ret = typeWithAnnotations(type, enclTy, annotations);
            typetree.type = ret;
            return ret;
        }

        private JCTree.JCArrayTypeTree arrayTypeTree(JCTree typetree) {
            if (typetree.getKind() == Tree.Kind.ARRAY_TYPE) {
                return (JCTree.JCArrayTypeTree) typetree;
            }
            if (typetree.getKind() == Tree.Kind.ANNOTATED_TYPE) {
                return (JCTree.JCArrayTypeTree) ((JCTree.JCAnnotatedType) typetree).underlyingType;
            }
            Assert.error("Could not determine array type from type tree: " + typetree);
            return null;
        }

        private Type typeWithAnnotations(Type type, final Type stopAt, List<Attribute.TypeCompound> annotations) {
            Type.Visitor<Type, List<Attribute.TypeCompound>> visitor = new Type.Visitor<Type, List<Attribute.TypeCompound>>() { // from class: com.sun.tools.javac.code.TypeAnnotations.TypeAnnotationPositions.1
                @Override // com.sun.tools.javac.code.Type.Visitor
                public Type visitClassType(Type.ClassType t, List<Attribute.TypeCompound> s) {
                    if (t == stopAt || t.getEnclosingType() == Type.noType) {
                        return t.annotatedType(s);
                    }
                    Type.ClassType ret = new Type.ClassType((Type) t.getEnclosingType().accept(this, s), t.typarams_field, t.tsym);
                    ret.all_interfaces_field = t.all_interfaces_field;
                    ret.allparams_field = t.allparams_field;
                    ret.interfaces_field = t.interfaces_field;
                    ret.rank_field = t.rank_field;
                    ret.supertype_field = t.supertype_field;
                    return ret;
                }

                @Override // com.sun.tools.javac.code.Type.Visitor
                public Type visitAnnotatedType(Type.AnnotatedType t, List<Attribute.TypeCompound> s) {
                    return ((Type) t.unannotatedType().accept(this, s)).annotatedType(t.getAnnotationMirrors());
                }

                @Override // com.sun.tools.javac.code.Type.Visitor
                public Type visitWildcardType(Type.WildcardType t, List<Attribute.TypeCompound> s) {
                    return t.annotatedType(s);
                }

                @Override // com.sun.tools.javac.code.Type.Visitor
                public Type visitArrayType(Type.ArrayType t, List<Attribute.TypeCompound> s) {
                    Type.ArrayType ret = new Type.ArrayType((Type) t.elemtype.accept(this, s), t.tsym);
                    return ret;
                }

                @Override // com.sun.tools.javac.code.Type.Visitor
                public Type visitMethodType(Type.MethodType t, List<Attribute.TypeCompound> s) {
                    return t;
                }

                @Override // com.sun.tools.javac.code.Type.Visitor
                public Type visitPackageType(Type.PackageType t, List<Attribute.TypeCompound> s) {
                    return t;
                }

                @Override // com.sun.tools.javac.code.Type.Visitor
                public Type visitTypeVar(Type.TypeVar t, List<Attribute.TypeCompound> s) {
                    return t.annotatedType(s);
                }

                @Override // com.sun.tools.javac.code.Type.Visitor
                public Type visitCapturedType(Type.CapturedType t, List<Attribute.TypeCompound> s) {
                    return t.annotatedType(s);
                }

                @Override // com.sun.tools.javac.code.Type.Visitor
                public Type visitForAll(Type.ForAll t, List<Attribute.TypeCompound> s) {
                    return t;
                }

                @Override // com.sun.tools.javac.code.Type.Visitor
                public Type visitUndetVar(Type.UndetVar t, List<Attribute.TypeCompound> s) {
                    return t;
                }

                @Override // com.sun.tools.javac.code.Type.Visitor
                public Type visitErrorType(Type.ErrorType t, List<Attribute.TypeCompound> s) {
                    return t.annotatedType(s);
                }

                @Override // com.sun.tools.javac.code.Type.Visitor
                public Type visitType(Type t, List<Attribute.TypeCompound> s) {
                    return t.annotatedType(s);
                }
            };
            return (Type) type.accept((Type.Visitor<R, List<Attribute.TypeCompound>>) visitor, annotations);
        }

        private Attribute.TypeCompound toTypeCompound(Attribute.Compound a, TypeAnnotationPosition p) {
            return new Attribute.TypeCompound(a, p);
        }

        /* JADX WARN: Multi-variable type inference failed */
        private void resolveFrame(JCTree tree, JCTree frame, List<JCTree> path, TypeAnnotationPosition p) {
            Type typeToUse;
            switch (frame.getKind()) {
                case TYPE_CAST:
                    JCTree.JCTypeCast frameTC = (JCTree.JCTypeCast) frame;
                    p.type = TargetType.CAST;
                    if (!frameTC.clazz.hasTag(JCTree.Tag.TYPEINTERSECTION)) {
                        p.type_index = 0;
                    }
                    p.pos = frame.pos;
                    break;
                case INSTANCE_OF:
                    p.type = TargetType.INSTANCEOF;
                    p.pos = frame.pos;
                    break;
                case NEW_CLASS:
                    JCTree.JCNewClass frameNewClass = (JCTree.JCNewClass) frame;
                    if (frameNewClass.def != null) {
                        JCTree.JCClassDecl frameClassDecl = frameNewClass.def;
                        if (frameClassDecl.extending == tree) {
                            p.type = TargetType.CLASS_EXTENDS;
                            p.type_index = -1;
                        } else if (frameClassDecl.implementing.contains(tree)) {
                            p.type = TargetType.CLASS_EXTENDS;
                            p.type_index = frameClassDecl.implementing.indexOf(tree);
                        } else {
                            Assert.error("Could not determine position of tree " + tree + " within frame " + frame);
                        }
                    } else if (frameNewClass.typeargs.contains(tree)) {
                        p.type = TargetType.CONSTRUCTOR_INVOCATION_TYPE_ARGUMENT;
                        p.type_index = frameNewClass.typeargs.indexOf(tree);
                    } else {
                        p.type = TargetType.NEW;
                    }
                    p.pos = frame.pos;
                    break;
                case NEW_ARRAY:
                    p.type = TargetType.NEW;
                    p.pos = frame.pos;
                    break;
                case ANNOTATION_TYPE:
                case CLASS:
                case ENUM:
                case INTERFACE:
                    p.pos = frame.pos;
                    if (((JCTree.JCClassDecl) frame).extending == tree) {
                        p.type = TargetType.CLASS_EXTENDS;
                        p.type_index = -1;
                    } else if (((JCTree.JCClassDecl) frame).implementing.contains(tree)) {
                        p.type = TargetType.CLASS_EXTENDS;
                        p.type_index = ((JCTree.JCClassDecl) frame).implementing.indexOf(tree);
                    } else if (((JCTree.JCClassDecl) frame).typarams.contains(tree)) {
                        p.type = TargetType.CLASS_TYPE_PARAMETER;
                        p.parameter_index = ((JCTree.JCClassDecl) frame).typarams.indexOf(tree);
                    } else {
                        Assert.error("Could not determine position of tree " + tree + " within frame " + frame);
                    }
                    break;
                case METHOD:
                    JCTree.JCMethodDecl frameMethod = (JCTree.JCMethodDecl) frame;
                    p.pos = frame.pos;
                    if (frameMethod.thrown.contains(tree)) {
                        p.type = TargetType.THROWS;
                        p.type_index = frameMethod.thrown.indexOf(tree);
                    } else if (frameMethod.restype == tree) {
                        p.type = TargetType.METHOD_RETURN;
                    } else if (frameMethod.typarams.contains(tree)) {
                        p.type = TargetType.METHOD_TYPE_PARAMETER;
                        p.parameter_index = frameMethod.typarams.indexOf(tree);
                    } else {
                        Assert.error("Could not determine position of tree " + tree + " within frame " + frame);
                    }
                    break;
                case PARAMETERIZED_TYPE:
                    List<JCTree> newPath = path.tail;
                    if (((JCTree.JCTypeApply) frame).clazz != tree) {
                        if (((JCTree.JCTypeApply) frame).arguments.contains(tree)) {
                            JCTree.JCTypeApply taframe = (JCTree.JCTypeApply) frame;
                            int arg = taframe.arguments.indexOf(tree);
                            p.location = p.location.prepend(new TypeAnnotationPosition.TypePathEntry(TypeAnnotationPosition.TypePathEntryKind.TYPE_ARGUMENT, arg));
                            if (newPath.tail != null && newPath.tail.head.hasTag(JCTree.Tag.NEWCLASS)) {
                                typeToUse = newPath.tail.head.type;
                            } else {
                                typeToUse = taframe.type;
                            }
                            locateNestedTypes(typeToUse, p);
                        } else {
                            Assert.error("Could not determine type argument position of tree " + tree + " within frame " + frame);
                        }
                    }
                    resolveFrame(newPath.head, newPath.tail.head, newPath, p);
                    break;
                case MEMBER_REFERENCE:
                    JCTree.JCMemberReference mrframe = (JCTree.JCMemberReference) frame;
                    if (mrframe.expr == tree) {
                        switch (mrframe.mode) {
                            case INVOKE:
                                p.type = TargetType.METHOD_REFERENCE;
                                break;
                            case NEW:
                                p.type = TargetType.CONSTRUCTOR_REFERENCE;
                                break;
                            default:
                                Assert.error("Unknown method reference mode " + mrframe.mode + " for tree " + tree + " within frame " + frame);
                                break;
                        }
                        p.pos = frame.pos;
                    } else if (mrframe.typeargs != null && mrframe.typeargs.contains(tree)) {
                        int arg2 = mrframe.typeargs.indexOf(tree);
                        p.type_index = arg2;
                        switch (mrframe.mode) {
                            case INVOKE:
                                p.type = TargetType.METHOD_REFERENCE_TYPE_ARGUMENT;
                                break;
                            case NEW:
                                p.type = TargetType.CONSTRUCTOR_REFERENCE_TYPE_ARGUMENT;
                                break;
                            default:
                                Assert.error("Unknown method reference mode " + mrframe.mode + " for tree " + tree + " within frame " + frame);
                                break;
                        }
                        p.pos = frame.pos;
                    } else {
                        Assert.error("Could not determine type argument position of tree " + tree + " within frame " + frame);
                    }
                    break;
                case ARRAY_TYPE:
                    ListBuffer<TypeAnnotationPosition.TypePathEntry> index = new ListBuffer<>();
                    ListBuffer<TypeAnnotationPosition.TypePathEntry> index2 = index.append(TypeAnnotationPosition.TypePathEntry.ARRAY);
                    List list = path.tail;
                    while (true) {
                        JCTree npHead = (JCTree) list.tail.head;
                        if (npHead.hasTag(JCTree.Tag.TYPEARRAY)) {
                            list = list.tail;
                            index2 = index2.append(TypeAnnotationPosition.TypePathEntry.ARRAY);
                        } else if (npHead.hasTag(JCTree.Tag.ANNOTATED_TYPE)) {
                            list = list.tail;
                        } else {
                            p.location = p.location.prependList(index2.toList());
                            resolveFrame((JCTree) list.head, (JCTree) list.tail.head, list, p);
                        }
                        break;
                    }
                    break;
                case TYPE_PARAMETER:
                    if (path.tail.tail.head.hasTag(JCTree.Tag.CLASSDEF)) {
                        JCTree.JCClassDecl clazz = (JCTree.JCClassDecl) path.tail.tail.head;
                        p.type = TargetType.CLASS_TYPE_PARAMETER_BOUND;
                        p.parameter_index = clazz.typarams.indexOf(path.tail.head);
                        p.bound_index = ((JCTree.JCTypeParameter) frame).bounds.indexOf(tree);
                        if (((JCTree.JCTypeParameter) frame).bounds.get(0).type.isInterface()) {
                            p.bound_index++;
                        }
                    } else if (path.tail.tail.head.hasTag(JCTree.Tag.METHODDEF)) {
                        JCTree.JCMethodDecl method = (JCTree.JCMethodDecl) path.tail.tail.head;
                        p.type = TargetType.METHOD_TYPE_PARAMETER_BOUND;
                        p.parameter_index = method.typarams.indexOf(path.tail.head);
                        p.bound_index = ((JCTree.JCTypeParameter) frame).bounds.indexOf(tree);
                        if (((JCTree.JCTypeParameter) frame).bounds.get(0).type.isInterface()) {
                            p.bound_index++;
                        }
                    } else {
                        Assert.error("Could not determine position of tree " + tree + " within frame " + frame);
                    }
                    p.pos = frame.pos;
                    break;
                case VARIABLE:
                    Symbol.VarSymbol v = ((JCTree.JCVariableDecl) frame).sym;
                    p.pos = frame.pos;
                    switch (v.getKind()) {
                        case LOCAL_VARIABLE:
                            p.type = TargetType.LOCAL_VARIABLE;
                            break;
                        case FIELD:
                            p.type = TargetType.FIELD;
                            break;
                        case PARAMETER:
                            if (v.getQualifiedName().equals(TypeAnnotations.this.names._this)) {
                                p.type = TargetType.METHOD_RECEIVER;
                            } else {
                                p.type = TargetType.METHOD_FORMAL_PARAMETER;
                                p.parameter_index = methodParamIndex(path, frame);
                            }
                            break;
                        case EXCEPTION_PARAMETER:
                            p.type = TargetType.EXCEPTION_PARAMETER;
                            break;
                        case RESOURCE_VARIABLE:
                            p.type = TargetType.RESOURCE_VARIABLE;
                            break;
                        default:
                            Assert.error("Found unexpected type annotation for variable: " + v + " with kind: " + v.getKind());
                            break;
                    }
                    if (v.getKind() != ElementKind.FIELD) {
                        v.owner.appendUniqueTypeAttributes(v.getRawTypeAttributes());
                    }
                    break;
                case ANNOTATED_TYPE:
                    if (frame == tree) {
                        JCTree.JCAnnotatedType atypetree = (JCTree.JCAnnotatedType) frame;
                        Type utype = atypetree.underlyingType.type;
                        if (utype != null) {
                            Symbol tsym = utype.tsym;
                            if (!tsym.getKind().equals(ElementKind.TYPE_PARAMETER) && !utype.getKind().equals(TypeKind.WILDCARD) && !utype.getKind().equals(TypeKind.ARRAY)) {
                                locateNestedTypes(utype, p);
                            }
                        }
                    }
                    List<JCTree> newPath2 = path.tail;
                    resolveFrame(newPath2.head, newPath2.tail.head, newPath2, p);
                    break;
                case UNION_TYPE:
                    List<JCTree> newPath3 = path.tail;
                    resolveFrame(newPath3.head, newPath3.tail.head, newPath3, p);
                    break;
                case INTERSECTION_TYPE:
                    JCTree.JCTypeIntersection isect = (JCTree.JCTypeIntersection) frame;
                    p.type_index = isect.bounds.indexOf(tree);
                    List<JCTree> newPath4 = path.tail;
                    resolveFrame(newPath4.head, newPath4.tail.head, newPath4, p);
                    break;
                case METHOD_INVOCATION:
                    JCTree.JCMethodInvocation invocation = (JCTree.JCMethodInvocation) frame;
                    if (!invocation.typeargs.contains(tree)) {
                        Assert.error("{" + tree + "} is not an argument in the invocation: " + invocation);
                    }
                    Symbol.MethodSymbol exsym = (Symbol.MethodSymbol) TreeInfo.symbol(invocation.getMethodSelect());
                    if (exsym == null) {
                        Assert.error("could not determine symbol for {" + invocation + "}");
                    } else if (exsym.isConstructor()) {
                        p.type = TargetType.CONSTRUCTOR_INVOCATION_TYPE_ARGUMENT;
                    } else {
                        p.type = TargetType.METHOD_INVOCATION_TYPE_ARGUMENT;
                    }
                    p.pos = invocation.pos;
                    p.type_index = invocation.typeargs.indexOf(tree);
                    break;
                case EXTENDS_WILDCARD:
                case SUPER_WILDCARD:
                    p.location = p.location.prepend(TypeAnnotationPosition.TypePathEntry.WILDCARD);
                    List<JCTree> newPath5 = path.tail;
                    resolveFrame(newPath5.head, newPath5.tail.head, newPath5, p);
                    break;
                case MEMBER_SELECT:
                    List<JCTree> newPath6 = path.tail;
                    resolveFrame(newPath6.head, newPath6.tail.head, newPath6, p);
                    break;
                default:
                    Assert.error("Unresolved frame: " + frame + " of kind: " + frame.getKind() + "\n    Looking for tree: " + tree);
                    break;
            }
        }

        private void locateNestedTypes(Type type, TypeAnnotationPosition p) {
            ListBuffer<TypeAnnotationPosition.TypePathEntry> depth = new ListBuffer<>();
            for (Type encl = type.getEnclosingType(); encl != null && encl.getKind() != TypeKind.NONE && encl.getKind() != TypeKind.ERROR; encl = encl.getEnclosingType()) {
                depth = depth.append(TypeAnnotationPosition.TypePathEntry.INNER_TYPE);
            }
            if (depth.nonEmpty()) {
                p.location = p.location.prependList(depth.toList());
            }
        }

        /* JADX WARN: Multi-variable type inference failed */
        private int methodParamIndex(List<JCTree> path, JCTree param) {
            List list = path;
            while (((JCTree) list.head).getTag() != JCTree.Tag.METHODDEF && ((JCTree) list.head).getTag() != JCTree.Tag.LAMBDA) {
                list = list.tail;
            }
            if (((JCTree) list.head).getTag() == JCTree.Tag.METHODDEF) {
                JCTree.JCMethodDecl method = (JCTree.JCMethodDecl) list.head;
                return method.params.indexOf(param);
            }
            if (((JCTree) list.head).getTag() == JCTree.Tag.LAMBDA) {
                JCTree.JCLambda lambda = (JCTree.JCLambda) list.head;
                return lambda.params.indexOf(param);
            }
            Assert.error("methodParamIndex expected to find method or lambda for param: " + param);
            return -1;
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitClassDef(JCTree.JCClassDecl tree) {
            if (this.isInClass) {
                return;
            }
            this.isInClass = true;
            if (this.sigOnly) {
                scan(tree.mods);
                scan(tree.typarams);
                scan(tree.extending);
                scan(tree.implementing);
            }
            scan(tree.defs);
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitMethodDef(JCTree.JCMethodDecl tree) {
            if (tree.sym == null) {
                Assert.error("Visiting tree node before memberEnter");
            }
            if (this.sigOnly) {
                if (!tree.mods.annotations.isEmpty()) {
                    TypeAnnotationPosition pos = new TypeAnnotationPosition();
                    pos.type = TargetType.METHOD_RETURN;
                    if (tree.sym.isConstructor()) {
                        pos.pos = tree.pos;
                        separateAnnotationsKinds(tree, null, tree.sym, pos);
                    } else {
                        pos.pos = tree.restype.pos;
                        separateAnnotationsKinds(tree.restype, tree.sym.type.mo178getReturnType(), tree.sym, pos);
                    }
                }
                if (tree.recvparam != null && tree.recvparam.sym != null && !tree.recvparam.mods.annotations.isEmpty()) {
                    TypeAnnotationPosition pos2 = new TypeAnnotationPosition();
                    pos2.type = TargetType.METHOD_RECEIVER;
                    pos2.pos = tree.recvparam.vartype.pos;
                    separateAnnotationsKinds(tree.recvparam.vartype, tree.recvparam.sym.type, tree.recvparam.sym, pos2);
                }
                int i = 0;
                for (JCTree.JCVariableDecl param : tree.params) {
                    if (!param.mods.annotations.isEmpty()) {
                        TypeAnnotationPosition pos3 = new TypeAnnotationPosition();
                        pos3.type = TargetType.METHOD_FORMAL_PARAMETER;
                        pos3.parameter_index = i;
                        pos3.pos = param.vartype.pos;
                        separateAnnotationsKinds(param.vartype, param.sym.type, param.sym, pos3);
                    }
                    i++;
                }
            }
            push(tree);
            if (this.sigOnly) {
                scan(tree.mods);
                scan(tree.restype);
                scan(tree.typarams);
                scan(tree.recvparam);
                scan(tree.params);
                scan(tree.thrown);
            } else {
                scan(tree.defaultValue);
                scan(tree.body);
            }
            pop();
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitLambda(JCTree.JCLambda tree) {
            JCTree.JCLambda prevLambda = this.currentLambda;
            try {
                this.currentLambda = tree;
                int i = 0;
                for (JCTree.JCVariableDecl param : tree.params) {
                    if (!param.mods.annotations.isEmpty()) {
                        TypeAnnotationPosition pos = new TypeAnnotationPosition();
                        pos.type = TargetType.METHOD_FORMAL_PARAMETER;
                        pos.parameter_index = i;
                        pos.pos = param.vartype.pos;
                        pos.onLambda = tree;
                        separateAnnotationsKinds(param.vartype, param.sym.type, param.sym, pos);
                    }
                    i++;
                }
                push(tree);
                scan(tree.body);
                scan(tree.params);
                pop();
            } finally {
                this.currentLambda = prevLambda;
            }
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitVarDef(JCTree.JCVariableDecl tree) {
            if (!tree.mods.annotations.isEmpty()) {
                if (tree.sym == null) {
                    Assert.error("Visiting tree node before memberEnter");
                } else if (tree.sym.getKind() != ElementKind.PARAMETER) {
                    if (tree.sym.getKind() == ElementKind.FIELD) {
                        if (this.sigOnly) {
                            TypeAnnotationPosition pos = new TypeAnnotationPosition();
                            pos.type = TargetType.FIELD;
                            pos.pos = tree.pos;
                            separateAnnotationsKinds(tree.vartype, tree.sym.type, tree.sym, pos);
                        }
                    } else if (tree.sym.getKind() == ElementKind.LOCAL_VARIABLE) {
                        TypeAnnotationPosition pos2 = new TypeAnnotationPosition();
                        pos2.type = TargetType.LOCAL_VARIABLE;
                        pos2.pos = tree.pos;
                        pos2.onLambda = this.currentLambda;
                        separateAnnotationsKinds(tree.vartype, tree.sym.type, tree.sym, pos2);
                    } else if (tree.sym.getKind() == ElementKind.EXCEPTION_PARAMETER) {
                        TypeAnnotationPosition pos3 = new TypeAnnotationPosition();
                        pos3.type = TargetType.EXCEPTION_PARAMETER;
                        pos3.pos = tree.pos;
                        pos3.onLambda = this.currentLambda;
                        separateAnnotationsKinds(tree.vartype, tree.sym.type, tree.sym, pos3);
                    } else if (tree.sym.getKind() == ElementKind.RESOURCE_VARIABLE) {
                        TypeAnnotationPosition pos4 = new TypeAnnotationPosition();
                        pos4.type = TargetType.RESOURCE_VARIABLE;
                        pos4.pos = tree.pos;
                        pos4.onLambda = this.currentLambda;
                        separateAnnotationsKinds(tree.vartype, tree.sym.type, tree.sym, pos4);
                    } else if (tree.sym.getKind() != ElementKind.ENUM_CONSTANT) {
                        Assert.error("Unhandled variable kind: " + tree + " of kind: " + tree.sym.getKind());
                    }
                }
            }
            push(tree);
            scan(tree.mods);
            scan(tree.vartype);
            if (!this.sigOnly) {
                scan(tree.init);
            }
            pop();
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitBlock(JCTree.JCBlock tree) {
            if (!this.sigOnly) {
                scan(tree.stats);
            }
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitAnnotatedType(JCTree.JCAnnotatedType tree) {
            push(tree);
            findPosition(tree, tree, tree.annotations);
            pop();
            super.visitAnnotatedType(tree);
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitTypeParameter(JCTree.JCTypeParameter tree) {
            findPosition(tree, peek2(), tree.annotations);
            super.visitTypeParameter(tree);
        }

        private void copyNewClassAnnotationsToOwner(JCTree.JCNewClass tree) {
            Symbol sym = tree.def.sym;
            TypeAnnotationPosition pos = new TypeAnnotationPosition();
            ListBuffer<Attribute.TypeCompound> newattrs = new ListBuffer<>();
            for (Attribute.TypeCompound old : sym.getRawTypeAttributes()) {
                newattrs.append(new Attribute.TypeCompound(old.type, old.values, pos));
            }
            pos.type = TargetType.NEW;
            pos.pos = tree.pos;
            sym.owner.appendUniqueTypeAttributes(newattrs.toList());
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitNewClass(JCTree.JCNewClass tree) {
            if (tree.def != null && !tree.def.mods.annotations.isEmpty()) {
                JCTree.JCClassDecl classdecl = tree.def;
                TypeAnnotationPosition pos = new TypeAnnotationPosition();
                pos.type = TargetType.CLASS_EXTENDS;
                pos.pos = tree.pos;
                if (classdecl.extending == tree.clazz) {
                    pos.type_index = -1;
                } else if (classdecl.implementing.contains(tree.clazz)) {
                    pos.type_index = classdecl.implementing.indexOf(tree.clazz);
                } else {
                    Assert.error("Could not determine position of tree " + tree);
                }
                Type before = classdecl.sym.type;
                separateAnnotationsKinds(classdecl, tree.clazz.type, classdecl.sym, pos);
                copyNewClassAnnotationsToOwner(tree);
                classdecl.sym.type = before;
            }
            scan(tree.encl);
            scan(tree.typeargs);
            scan(tree.clazz);
            scan(tree.args);
        }

        @Override // com.sun.tools.javac.tree.TreeScanner, com.sun.tools.javac.tree.JCTree.Visitor
        public void visitNewArray(JCTree.JCNewArray tree) {
            findPosition(tree, tree, tree.annotations);
            int dimAnnosCount = tree.dimAnnotations.size();
            ListBuffer<TypeAnnotationPosition.TypePathEntry> depth = new ListBuffer<>();
            for (int i = 0; i < dimAnnosCount; i++) {
                TypeAnnotationPosition p = new TypeAnnotationPosition();
                p.pos = tree.pos;
                p.onLambda = this.currentLambda;
                p.type = TargetType.NEW;
                if (i != 0) {
                    depth = depth.append(TypeAnnotationPosition.TypePathEntry.ARRAY);
                    p.location = p.location.appendList(depth.toList());
                }
                setTypeAnnotationPos(tree.dimAnnotations.get(i), p);
            }
            JCTree.JCExpression elemType = tree.elemtype;
            ListBuffer<TypeAnnotationPosition.TypePathEntry> depth2 = depth.append(TypeAnnotationPosition.TypePathEntry.ARRAY);
            while (elemType != null) {
                if (elemType.hasTag(JCTree.Tag.ANNOTATED_TYPE)) {
                    JCTree.JCAnnotatedType at = (JCTree.JCAnnotatedType) elemType;
                    TypeAnnotationPosition p2 = new TypeAnnotationPosition();
                    p2.type = TargetType.NEW;
                    p2.pos = tree.pos;
                    p2.onLambda = this.currentLambda;
                    locateNestedTypes(elemType.type, p2);
                    p2.location = p2.location.prependList(depth2.toList());
                    setTypeAnnotationPos(at.annotations, p2);
                    elemType = at.underlyingType;
                } else if (elemType.hasTag(JCTree.Tag.TYPEARRAY)) {
                    depth2 = depth2.append(TypeAnnotationPosition.TypePathEntry.ARRAY);
                    elemType = ((JCTree.JCArrayTypeTree) elemType).elemtype;
                } else if (!elemType.hasTag(JCTree.Tag.SELECT)) {
                    break;
                } else {
                    elemType = ((JCTree.JCFieldAccess) elemType).selected;
                }
            }
            scan(tree.elems);
        }

        private void findPosition(JCTree tree, JCTree frame, List<JCTree.JCAnnotation> annotations) {
            if (!annotations.isEmpty()) {
                TypeAnnotationPosition p = new TypeAnnotationPosition();
                p.onLambda = this.currentLambda;
                resolveFrame(tree, frame, this.frames.toList(), p);
                setTypeAnnotationPos(annotations, p);
            }
        }

        private void setTypeAnnotationPos(List<JCTree.JCAnnotation> annotations, TypeAnnotationPosition position) {
            for (JCTree.JCAnnotation anno : annotations) {
                if (anno.attribute != null) {
                    ((Attribute.TypeCompound) anno.attribute).position = position;
                }
            }
        }

        public String toString() {
            return super.toString() + ": sigOnly: " + this.sigOnly;
        }
    }
}
