package com.sun.tools.javac.code;

import com.sun.tools.javac.tree.JCTree;
import com.sun.tools.javac.util.Assert;
import com.sun.tools.javac.util.List;
import com.sun.tools.javac.util.ListBuffer;
import java.util.Iterator;
import org.firstinspires.ftc.onbotjava.OnBotJavaFileSystemUtils;

/* JADX INFO: loaded from: classes.dex */
public class TypeAnnotationPosition {
    public TargetType type = TargetType.UNKNOWN;
    public List<TypePathEntry> location = List.nil();
    public int pos = -1;
    public boolean isValidOffset = false;
    public int offset = -1;
    public int[] lvarOffset = null;
    public int[] lvarLength = null;
    public int[] lvarIndex = null;
    public int bound_index = Integer.MIN_VALUE;
    public int parameter_index = Integer.MIN_VALUE;
    public int type_index = Integer.MIN_VALUE;
    public int exception_index = Integer.MIN_VALUE;
    public JCTree.JCLambda onLambda = null;

    public enum TypePathEntryKind {
        ARRAY(0),
        INNER_TYPE(1),
        WILDCARD(2),
        TYPE_ARGUMENT(3);

        public final int tag;

        TypePathEntryKind(int tag) {
            this.tag = tag;
        }
    }

    public static class TypePathEntry {
        public static final TypePathEntry ARRAY = new TypePathEntry(TypePathEntryKind.ARRAY);
        public static final TypePathEntry INNER_TYPE = new TypePathEntry(TypePathEntryKind.INNER_TYPE);
        public static final TypePathEntry WILDCARD = new TypePathEntry(TypePathEntryKind.WILDCARD);
        public static final int bytesPerEntry = 2;
        public final int arg;
        public final TypePathEntryKind tag;

        private TypePathEntry(TypePathEntryKind tag) {
            Assert.check(tag == TypePathEntryKind.ARRAY || tag == TypePathEntryKind.INNER_TYPE || tag == TypePathEntryKind.WILDCARD, "Invalid TypePathEntryKind: " + tag);
            this.tag = tag;
            this.arg = 0;
        }

        public TypePathEntry(TypePathEntryKind tag, int arg) {
            Assert.check(tag == TypePathEntryKind.TYPE_ARGUMENT, "Invalid TypePathEntryKind: " + tag);
            this.tag = tag;
            this.arg = arg;
        }

        public static TypePathEntry fromBinary(int tag, int arg) {
            Assert.check(arg == 0 || tag == TypePathEntryKind.TYPE_ARGUMENT.tag, "Invalid TypePathEntry tag/arg: " + tag + OnBotJavaFileSystemUtils.PATH_SEPARATOR + arg);
            switch (tag) {
                case 0:
                    return ARRAY;
                case 1:
                    return INNER_TYPE;
                case 2:
                    return WILDCARD;
                case 3:
                    return new TypePathEntry(TypePathEntryKind.TYPE_ARGUMENT, arg);
                default:
                    Assert.error("Invalid TypePathEntryKind tag: " + tag);
                    return null;
            }
        }

        public String toString() {
            return this.tag.toString() + (this.tag == TypePathEntryKind.TYPE_ARGUMENT ? "(" + this.arg + ")" : "");
        }

        public boolean equals(Object other) {
            if (!(other instanceof TypePathEntry)) {
                return false;
            }
            TypePathEntry tpe = (TypePathEntry) other;
            return this.tag == tpe.tag && this.arg == tpe.arg;
        }

        public int hashCode() {
            return (this.tag.hashCode() * 17) + this.arg;
        }
    }

    public String toString() {
        StringBuilder sb = new StringBuilder();
        sb.append('[');
        sb.append(this.type);
        switch (this.type) {
            case INSTANCEOF:
            case NEW:
            case CONSTRUCTOR_REFERENCE:
            case METHOD_REFERENCE:
                sb.append(", offset = ");
                sb.append(this.offset);
                break;
            case LOCAL_VARIABLE:
            case RESOURCE_VARIABLE:
                if (this.lvarOffset == null) {
                    sb.append(", lvarOffset is null!");
                } else {
                    sb.append(", {");
                    for (int i = 0; i < this.lvarOffset.length; i++) {
                        if (i != 0) {
                            sb.append("; ");
                        }
                        sb.append("start_pc = ");
                        sb.append(this.lvarOffset[i]);
                        sb.append(", length = ");
                        sb.append(this.lvarLength[i]);
                        sb.append(", index = ");
                        sb.append(this.lvarIndex[i]);
                    }
                    sb.append("}");
                }
                break;
            case METHOD_RECEIVER:
            case METHOD_RETURN:
            case FIELD:
                break;
            case CLASS_TYPE_PARAMETER:
            case METHOD_TYPE_PARAMETER:
                sb.append(", param_index = ");
                sb.append(this.parameter_index);
                break;
            case CLASS_TYPE_PARAMETER_BOUND:
            case METHOD_TYPE_PARAMETER_BOUND:
                sb.append(", param_index = ");
                sb.append(this.parameter_index);
                sb.append(", bound_index = ");
                sb.append(this.bound_index);
                break;
            case CLASS_EXTENDS:
                sb.append(", type_index = ");
                sb.append(this.type_index);
                break;
            case THROWS:
                sb.append(", type_index = ");
                sb.append(this.type_index);
                break;
            case EXCEPTION_PARAMETER:
                sb.append(", exception_index = ");
                sb.append(this.exception_index);
                break;
            case METHOD_FORMAL_PARAMETER:
                sb.append(", param_index = ");
                sb.append(this.parameter_index);
                break;
            case CAST:
            case CONSTRUCTOR_INVOCATION_TYPE_ARGUMENT:
            case METHOD_INVOCATION_TYPE_ARGUMENT:
            case CONSTRUCTOR_REFERENCE_TYPE_ARGUMENT:
            case METHOD_REFERENCE_TYPE_ARGUMENT:
                sb.append(", offset = ");
                sb.append(this.offset);
                sb.append(", type_index = ");
                sb.append(this.type_index);
                break;
            case UNKNOWN:
                sb.append(", position UNKNOWN!");
                break;
            default:
                Assert.error("Unknown target type: " + this.type);
                break;
        }
        if (!this.location.isEmpty()) {
            sb.append(", location = (");
            sb.append(this.location);
            sb.append(")");
        }
        sb.append(", pos = ");
        sb.append(this.pos);
        if (this.onLambda != null) {
            sb.append(", onLambda hash = ");
            sb.append(this.onLambda.hashCode());
        }
        sb.append(']');
        return sb.toString();
    }

    public boolean emitToClassfile() {
        return !this.type.isLocal() || this.isValidOffset;
    }

    public boolean matchesPos(int pos) {
        return this.pos == pos;
    }

    public void updatePosOffset(int to) {
        this.offset = to;
        this.lvarOffset = new int[]{to};
        this.isValidOffset = true;
    }

    public static List<TypePathEntry> getTypePathFromBinary(java.util.List<Integer> list) {
        ListBuffer<TypePathEntry> loc = new ListBuffer<>();
        Iterator<Integer> iter = list.iterator();
        while (iter.hasNext()) {
            Integer fst = iter.next();
            Assert.check(iter.hasNext(), "Could not decode type path: " + list);
            Integer snd = iter.next();
            loc = loc.append(TypePathEntry.fromBinary(fst.intValue(), snd.intValue()));
        }
        return loc.toList();
    }

    public static List<Integer> getBinaryFromTypePath(java.util.List<TypePathEntry> locs) {
        ListBuffer<Integer> loc = new ListBuffer<>();
        for (TypePathEntry tpe : locs) {
            loc = loc.append(Integer.valueOf(tpe.tag.tag)).append(Integer.valueOf(tpe.arg));
        }
        return loc.toList();
    }
}
