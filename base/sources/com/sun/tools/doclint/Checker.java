package com.sun.tools.doclint;

import com.sun.source.doctree.AttributeTree;
import com.sun.source.doctree.AuthorTree;
import com.sun.source.doctree.DocCommentTree;
import com.sun.source.doctree.DocRootTree;
import com.sun.source.doctree.DocTree;
import com.sun.source.doctree.EndElementTree;
import com.sun.source.doctree.EntityTree;
import com.sun.source.doctree.ErroneousTree;
import com.sun.source.doctree.IdentifierTree;
import com.sun.source.doctree.InheritDocTree;
import com.sun.source.doctree.LinkTree;
import com.sun.source.doctree.LiteralTree;
import com.sun.source.doctree.ParamTree;
import com.sun.source.doctree.ReferenceTree;
import com.sun.source.doctree.ReturnTree;
import com.sun.source.doctree.SerialDataTree;
import com.sun.source.doctree.SerialFieldTree;
import com.sun.source.doctree.SinceTree;
import com.sun.source.doctree.StartElementTree;
import com.sun.source.doctree.TextTree;
import com.sun.source.doctree.ThrowsTree;
import com.sun.source.doctree.UnknownBlockTagTree;
import com.sun.source.doctree.UnknownInlineTagTree;
import com.sun.source.doctree.ValueTree;
import com.sun.source.doctree.VersionTree;
import com.sun.source.util.DocTreePath;
import com.sun.source.util.DocTreePathScanner;
import com.sun.source.util.TreePath;
import com.sun.tools.doclint.HtmlTag;
import com.sun.tools.doclint.Messages;
import com.sun.tools.javac.tree.DocPretty;
import com.sun.tools.javac.util.StringUtils;
import java.io.IOException;
import java.io.StringWriter;
import java.net.URI;
import java.net.URISyntaxException;
import java.util.Deque;
import java.util.EnumSet;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.Set;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import javax.lang.model.element.Element;
import javax.lang.model.element.ElementKind;
import javax.lang.model.element.ExecutableElement;
import javax.lang.model.element.Name;
import javax.lang.model.element.VariableElement;
import javax.lang.model.type.TypeKind;
import javax.lang.model.type.TypeMirror;
import javax.tools.Diagnostic;
import javax.tools.JavaFileObject;

/* JADX INFO: loaded from: classes.dex */
public class Checker extends DocTreePathScanner<Void, Void> {
    private HtmlTag currHeaderTag;
    final Env env;
    private final int implicitHeaderLevel;
    private final Deque<TagStackItem> tagStack;
    private static final Pattern validName = Pattern.compile("[A-Za-z][A-Za-z0-9-_:.]*");
    private static final Pattern validNumber = Pattern.compile("-?[0-9]+");
    private static final Pattern docRoot = Pattern.compile("(?i)(\\{@docRoot *\\}/?)?(.*)");
    Set<Element> foundParams = new HashSet();
    Set<TypeMirror> foundThrows = new HashSet();
    Map<Element, Set<String>> foundAnchors = new HashMap();
    boolean foundInheritDoc = false;
    boolean foundReturn = false;

    public enum Flag {
        TABLE_HAS_CAPTION,
        HAS_ELEMENT,
        HAS_INLINE_TAG,
        HAS_TEXT,
        REPORTED_BAD_INLINE
    }

    static class TagStackItem {
        final Set<HtmlTag.Attr> attrs = EnumSet.noneOf(HtmlTag.Attr.class);
        final Set<Flag> flags = EnumSet.noneOf(Flag.class);
        final HtmlTag tag;
        final DocTree tree;

        TagStackItem(DocTree tree, HtmlTag tag) {
            this.tree = tree;
            this.tag = tag;
        }

        public String toString() {
            return String.valueOf(this.tag);
        }
    }

    Checker(Env env) {
        env.getClass();
        this.env = env;
        this.tagStack = new LinkedList();
        this.implicitHeaderLevel = env.implicitHeaderLevel;
    }

    public Void scan(DocCommentTree tree, TreePath p) {
        this.env.setCurrent(p, tree);
        boolean isOverridingMethod = !this.env.currOverriddenMethods.isEmpty();
        if (p.getLeaf() == p.getCompilationUnit()) {
            JavaFileObject fo = p.getCompilationUnit().getSourceFile();
            boolean isPkgInfo = fo.isNameCompatible("package-info", JavaFileObject.Kind.SOURCE);
            if (tree == null) {
                if (isPkgInfo) {
                    reportMissing("dc.missing.comment", new Object[0]);
                }
                return null;
            }
            if (!isPkgInfo) {
                reportReference("dc.unexpected.comment", new Object[0]);
            }
        } else if (tree == null) {
            if (!isSynthetic() && !isOverridingMethod) {
                reportMissing("dc.missing.comment", new Object[0]);
            }
            return null;
        }
        this.tagStack.clear();
        this.currHeaderTag = null;
        this.foundParams.clear();
        this.foundThrows.clear();
        this.foundInheritDoc = false;
        this.foundReturn = false;
        scan(new DocTreePath(p, tree), (Object) null);
        if (!isOverridingMethod) {
            switch (this.env.currElement.getKind()) {
                case METHOD:
                case CONSTRUCTOR:
                    ExecutableElement ee = (ExecutableElement) this.env.currElement;
                    checkParamsDocumented(ee.getTypeParameters());
                    checkParamsDocumented(ee.getParameters());
                    switch (ee.getReturnType().getKind()) {
                        case VOID:
                        case NONE:
                            break;
                        default:
                            if (!this.foundReturn && !this.foundInheritDoc && !this.env.types.isSameType(ee.getReturnType(), this.env.java_lang_Void)) {
                                reportMissing("dc.missing.return", new Object[0]);
                            }
                            break;
                    }
                    checkThrowsDocumented(ee.getThrownTypes());
                default:
                    return null;
            }
        }
        return null;
    }

    private void reportMissing(String code, Object... args) {
        this.env.messages.report(Messages.Group.MISSING, Diagnostic.Kind.WARNING, this.env.currPath.getLeaf(), code, args);
    }

    private void reportReference(String code, Object... args) {
        this.env.messages.report(Messages.Group.REFERENCE, Diagnostic.Kind.WARNING, this.env.currPath.getLeaf(), code, args);
    }

    @Override // com.sun.source.util.DocTreeScanner, com.sun.source.doctree.DocTreeVisitor
    public Void visitDocComment(DocCommentTree tree, Void ignore) {
        super.visitDocComment(tree, ignore);
        for (TagStackItem tsi : this.tagStack) {
            warnIfEmpty(tsi, (DocTree) null);
            if (tsi.tree.getKind() == DocTree.Kind.START_ELEMENT && tsi.tag.endKind == HtmlTag.EndKind.REQUIRED) {
                StartElementTree t = (StartElementTree) tsi.tree;
                this.env.messages.error(Messages.Group.HTML, t, "dc.tag.not.closed", t.getName());
            }
        }
        return null;
    }

    @Override // com.sun.source.util.DocTreeScanner, com.sun.source.doctree.DocTreeVisitor
    public Void visitText(TextTree tree, Void ignore) {
        if (hasNonWhitespace(tree)) {
            checkAllowsText(tree);
            markEnclosingTag(Flag.HAS_TEXT);
            return null;
        }
        return null;
    }

    @Override // com.sun.source.util.DocTreeScanner, com.sun.source.doctree.DocTreeVisitor
    public Void visitEntity(EntityTree tree, Void ignore) {
        int v;
        checkAllowsText(tree);
        markEnclosingTag(Flag.HAS_TEXT);
        String name = tree.getName().toString();
        if (name.startsWith("#")) {
            if (StringUtils.toLowerCase(name).startsWith("#x")) {
                v = Integer.parseInt(name.substring(2), 16);
            } else {
                v = Integer.parseInt(name.substring(1), 10);
            }
            if (!Entity.isValid(v)) {
                this.env.messages.error(Messages.Group.HTML, tree, "dc.entity.invalid", name);
                return null;
            }
            return null;
        }
        if (!Entity.isValid(name)) {
            this.env.messages.error(Messages.Group.HTML, tree, "dc.entity.invalid", name);
            return null;
        }
        return null;
    }

    void checkAllowsText(DocTree tree) {
        TagStackItem top = this.tagStack.peek();
        if (top != null && top.tree.getKind() == DocTree.Kind.START_ELEMENT && !top.tag.acceptsText() && top.flags.add(Flag.REPORTED_BAD_INLINE)) {
            this.env.messages.error(Messages.Group.HTML, tree, "dc.text.not.allowed", ((StartElementTree) top.tree).getName());
        }
    }

    /* JADX WARN: Removed duplicated region for block: B:56:0x0131 A[DONT_GENERATE] */
    @Override // com.sun.source.util.DocTreeScanner, com.sun.source.doctree.DocTreeVisitor
    /*
        Code decompiled incorrectly, please refer to instructions dump.
        To view partially-correct code enable 'Show inconsistent code' option in preferences
    */
    public java.lang.Void visitStartElement(com.sun.source.doctree.StartElementTree r10, java.lang.Void r11) {
        /*
            Method dump skipped, instruction units count: 350
            To view this dump change 'Code comments level' option to 'DEBUG'
        */
        throw new UnsupportedOperationException("Method not decompiled: com.sun.tools.doclint.Checker.visitStartElement(com.sun.source.doctree.StartElementTree, java.lang.Void):java.lang.Void");
    }

    private void checkStructure(StartElementTree tree, HtmlTag t) {
        Name treeName = tree.getName();
        TagStackItem top = this.tagStack.peek();
        switch (t.blockType) {
            case BLOCK:
                if (top == null || top.tag.accepts(t)) {
                    return;
                }
                switch (top.tree.getKind()) {
                    case START_ELEMENT:
                        if (top.tag.blockType == HtmlTag.BlockType.INLINE) {
                            Name name = ((StartElementTree) top.tree).getName();
                            this.env.messages.error(Messages.Group.HTML, tree, "dc.tag.not.allowed.inline.element", treeName, name);
                            return;
                        }
                        break;
                    case LINK:
                    case LINK_PLAIN:
                        String name2 = top.tree.getKind().tagName;
                        this.env.messages.error(Messages.Group.HTML, tree, "dc.tag.not.allowed.inline.tag", treeName, name2);
                        return;
                }
                break;
            case INLINE:
                if (top == null || top.tag.accepts(t)) {
                    return;
                }
                break;
            case LIST_ITEM:
            case TABLE_ITEM:
                if (top != null) {
                    top.flags.remove(Flag.REPORTED_BAD_INLINE);
                    if (top.tag.accepts(t)) {
                        return;
                    }
                }
                break;
            case OTHER:
                switch (t) {
                    case SCRIPT:
                        break;
                    default:
                        this.env.messages.error(Messages.Group.HTML, tree, "dc.tag.not.allowed", treeName);
                        break;
                }
                return;
        }
        this.env.messages.error(Messages.Group.HTML, tree, "dc.tag.not.allowed.here", treeName);
    }

    private void checkHeader(StartElementTree tree, HtmlTag tag) {
        if (getHeaderLevel(tag) > getHeaderLevel(this.currHeaderTag) + 1) {
            if (this.currHeaderTag == null) {
                this.env.messages.error(Messages.Group.ACCESSIBILITY, tree, "dc.tag.header.sequence.1", tag);
            } else {
                this.env.messages.error(Messages.Group.ACCESSIBILITY, tree, "dc.tag.header.sequence.2", tag, this.currHeaderTag);
            }
        }
        this.currHeaderTag = tag;
    }

    private int getHeaderLevel(HtmlTag tag) {
        if (tag == null) {
            return this.implicitHeaderLevel;
        }
        switch (tag) {
            case H1:
                return 1;
            case H2:
                return 2;
            case H3:
                return 3;
            case H4:
                return 4;
            case H5:
                return 5;
            case H6:
                return 6;
            default:
                throw new IllegalArgumentException();
        }
    }

    @Override // com.sun.source.util.DocTreeScanner, com.sun.source.doctree.DocTreeVisitor
    public Void visitEndElement(EndElementTree tree, Void ignore) {
        Name treeName = tree.getName();
        HtmlTag t = HtmlTag.get(treeName);
        if (t == null) {
            this.env.messages.error(Messages.Group.HTML, tree, "dc.tag.unknown", treeName);
        } else if (t.endKind == HtmlTag.EndKind.NONE) {
            this.env.messages.error(Messages.Group.HTML, tree, "dc.tag.end.not.permitted", treeName);
        } else {
            boolean done = false;
            while (true) {
                if (this.tagStack.isEmpty()) {
                    break;
                }
                TagStackItem top = this.tagStack.peek();
                if (t == top.tag) {
                    switch (t) {
                        case TABLE:
                            if (!top.attrs.contains(HtmlTag.Attr.SUMMARY) && !top.flags.contains(Flag.TABLE_HAS_CAPTION)) {
                                this.env.messages.error(Messages.Group.ACCESSIBILITY, tree, "dc.no.summary.or.caption.for.table", new Object[0]);
                            }
                            break;
                    }
                    warnIfEmpty(top, tree);
                    this.tagStack.pop();
                    done = true;
                } else if (top.tag == null || top.tag.endKind != HtmlTag.EndKind.REQUIRED) {
                    this.tagStack.pop();
                } else {
                    boolean found = false;
                    Iterator<TagStackItem> it = this.tagStack.iterator();
                    while (true) {
                        if (!it.hasNext()) {
                            break;
                        }
                        TagStackItem si = it.next();
                        if (si.tag == t) {
                            found = true;
                            break;
                        }
                    }
                    if (!found || top.tree.getKind() != DocTree.Kind.START_ELEMENT) {
                        break;
                    }
                    this.env.messages.error(Messages.Group.HTML, top.tree, "dc.tag.start.unmatched", ((StartElementTree) top.tree).getName());
                    this.tagStack.pop();
                }
            }
            this.env.messages.error(Messages.Group.HTML, tree, "dc.tag.end.unexpected", treeName);
            done = true;
            if (!done && this.tagStack.isEmpty()) {
                this.env.messages.error(Messages.Group.HTML, tree, "dc.tag.end.unexpected", treeName);
            }
        }
        return (Void) super.visitEndElement(tree, ignore);
    }

    void warnIfEmpty(TagStackItem tsi, DocTree endTree) {
        if (tsi.tag != null && (tsi.tree instanceof StartElementTree) && tsi.tag.flags.contains(HtmlTag.Flag.EXPECT_CONTENT) && !tsi.flags.contains(Flag.HAS_TEXT) && !tsi.flags.contains(Flag.HAS_ELEMENT) && !tsi.flags.contains(Flag.HAS_INLINE_TAG)) {
            DocTree tree = endTree != null ? endTree : tsi.tree;
            Name treeName = ((StartElementTree) tsi.tree).getName();
            this.env.messages.warning(Messages.Group.HTML, tree, "dc.tag.empty", treeName);
        }
    }

    @Override // com.sun.source.util.DocTreeScanner, com.sun.source.doctree.DocTreeVisitor
    public Void visitAttribute(AttributeTree tree, Void ignore) {
        HtmlTag currTag = this.tagStack.peek().tag;
        if (currTag != null) {
            Name name = tree.getName();
            HtmlTag.Attr attr = currTag.getAttr(name);
            if (attr != null) {
                boolean first = this.tagStack.peek().attrs.add(attr);
                if (!first) {
                    this.env.messages.error(Messages.Group.HTML, tree, "dc.attr.repeated", name);
                }
            }
            if (!name.toString().startsWith("on")) {
                HtmlTag.AttrKind k = currTag.getAttrKind(name);
                switch (k) {
                    case INVALID:
                        this.env.messages.error(Messages.Group.HTML, tree, "dc.attr.unknown", name);
                        break;
                    case OBSOLETE:
                        this.env.messages.warning(Messages.Group.ACCESSIBILITY, tree, "dc.attr.obsolete", name);
                        break;
                    case USE_CSS:
                        this.env.messages.warning(Messages.Group.ACCESSIBILITY, tree, "dc.attr.obsolete.use.css", name);
                        break;
                }
            }
            if (attr != null) {
                switch (attr) {
                    case NAME:
                        if (currTag == HtmlTag.A) {
                        }
                    case ID:
                        String value = getAttrValue(tree);
                        if (value == null) {
                            this.env.messages.error(Messages.Group.HTML, tree, "dc.anchor.value.missing", new Object[0]);
                        } else {
                            if (!validName.matcher(value).matches()) {
                                this.env.messages.error(Messages.Group.HTML, tree, "dc.invalid.anchor", value);
                            }
                            if (!checkAnchor(value)) {
                                this.env.messages.error(Messages.Group.HTML, tree, "dc.anchor.already.defined", value);
                            }
                        }
                        break;
                    case HREF:
                        if (currTag == HtmlTag.A) {
                            String v = getAttrValue(tree);
                            if (v == null || v.isEmpty()) {
                                this.env.messages.error(Messages.Group.HTML, tree, "dc.attr.lacks.value", new Object[0]);
                            } else {
                                Matcher m = docRoot.matcher(v);
                                if (m.matches()) {
                                    String rest = m.group(2);
                                    if (!rest.isEmpty()) {
                                        checkURI(tree, rest);
                                    }
                                } else {
                                    checkURI(tree, v);
                                }
                            }
                        }
                        break;
                    case VALUE:
                        if (currTag == HtmlTag.LI) {
                            String v2 = getAttrValue(tree);
                            if (v2 == null || v2.isEmpty()) {
                                this.env.messages.error(Messages.Group.HTML, tree, "dc.attr.lacks.value", new Object[0]);
                            } else if (!validNumber.matcher(v2).matches()) {
                                this.env.messages.error(Messages.Group.HTML, tree, "dc.attr.not.number", new Object[0]);
                            }
                        }
                        break;
                }
            }
        }
        return (Void) super.visitAttribute(tree, ignore);
    }

    private boolean checkAnchor(String name) {
        Element e = getEnclosingPackageOrClass(this.env.currElement);
        if (e == null) {
            return true;
        }
        Set<String> set = this.foundAnchors.get(e);
        if (set == null) {
            Map<Element, Set<String>> map = this.foundAnchors;
            HashSet hashSet = new HashSet();
            set = hashSet;
            map.put(e, hashSet);
        }
        return set.add(name);
    }

    private Element getEnclosingPackageOrClass(Element e) {
        while (e != null) {
            switch (e.getKind()) {
                case CLASS:
                case ENUM:
                case INTERFACE:
                case PACKAGE:
                    return e;
                default:
                    e = e.getEnclosingElement();
                    break;
            }
        }
        return e;
    }

    private String getAttrValue(AttributeTree tree) {
        if (tree.getValue() == null) {
            return null;
        }
        StringWriter sw = new StringWriter();
        try {
            new DocPretty(sw).print(tree.getValue());
        } catch (IOException e) {
        }
        return sw.toString();
    }

    private void checkURI(AttributeTree tree, String uri) {
        if (uri.startsWith("javascript:")) {
            return;
        }
        try {
            new URI(uri);
        } catch (URISyntaxException e) {
            this.env.messages.error(Messages.Group.HTML, tree, "dc.invalid.uri", uri);
        }
    }

    @Override // com.sun.source.util.DocTreeScanner, com.sun.source.doctree.DocTreeVisitor
    public Void visitAuthor(AuthorTree tree, Void ignore) {
        warnIfEmpty(tree, tree.getName());
        return (Void) super.visitAuthor(tree, ignore);
    }

    @Override // com.sun.source.util.DocTreeScanner, com.sun.source.doctree.DocTreeVisitor
    public Void visitDocRoot(DocRootTree tree, Void ignore) {
        markEnclosingTag(Flag.HAS_INLINE_TAG);
        return (Void) super.visitDocRoot(tree, ignore);
    }

    @Override // com.sun.source.util.DocTreeScanner, com.sun.source.doctree.DocTreeVisitor
    public Void visitInheritDoc(InheritDocTree tree, Void ignore) {
        markEnclosingTag(Flag.HAS_INLINE_TAG);
        this.foundInheritDoc = true;
        return (Void) super.visitInheritDoc(tree, ignore);
    }

    @Override // com.sun.source.util.DocTreeScanner, com.sun.source.doctree.DocTreeVisitor
    public Void visitLink(LinkTree tree, Void ignore) {
        markEnclosingTag(Flag.HAS_INLINE_TAG);
        HtmlTag t = tree.getKind() == DocTree.Kind.LINK ? HtmlTag.CODE : HtmlTag.SPAN;
        this.tagStack.push(new TagStackItem(tree, t));
        try {
            return (Void) super.visitLink(tree, ignore);
        } finally {
            this.tagStack.pop();
        }
    }

    @Override // com.sun.source.util.DocTreeScanner, com.sun.source.doctree.DocTreeVisitor
    public Void visitLiteral(LiteralTree tree, Void ignore) {
        markEnclosingTag(Flag.HAS_INLINE_TAG);
        if (tree.getKind() == DocTree.Kind.CODE) {
            Iterator<TagStackItem> it = this.tagStack.iterator();
            while (true) {
                if (!it.hasNext()) {
                    break;
                }
                TagStackItem tsi = it.next();
                if (tsi.tag == HtmlTag.CODE) {
                    this.env.messages.warning(Messages.Group.HTML, tree, "dc.tag.code.within.code", new Object[0]);
                    break;
                }
            }
        }
        return (Void) super.visitLiteral(tree, ignore);
    }

    @Override // com.sun.source.util.DocTreeScanner, com.sun.source.doctree.DocTreeVisitor
    public Void visitParam(ParamTree tree, Void ignore) {
        boolean typaram = tree.isTypeParameter();
        IdentifierTree nameTree = tree.getName();
        Element paramElement = nameTree != null ? this.env.trees.getElement(new DocTreePath(getCurrentPath(), nameTree)) : null;
        if (paramElement == null) {
            switch (this.env.currElement.getKind()) {
                case CLASS:
                case INTERFACE:
                    if (!typaram) {
                        this.env.messages.error(Messages.Group.REFERENCE, tree, "dc.invalid.param", new Object[0]);
                        break;
                    }
                case METHOD:
                case CONSTRUCTOR:
                    this.env.messages.error(Messages.Group.REFERENCE, nameTree, "dc.param.name.not.found", new Object[0]);
                    break;
                case ENUM:
                default:
                    this.env.messages.error(Messages.Group.REFERENCE, tree, "dc.invalid.param", new Object[0]);
                    break;
            }
        } else {
            this.foundParams.add(paramElement);
        }
        warnIfEmpty(tree, tree.getDescription());
        return (Void) super.visitParam(tree, ignore);
    }

    private void checkParamsDocumented(List<? extends Element> list) {
        CharSequence paramName;
        if (this.foundInheritDoc) {
            return;
        }
        for (Element e : list) {
            if (!this.foundParams.contains(e)) {
                if (e.getKind() == ElementKind.TYPE_PARAMETER) {
                    paramName = "<" + ((Object) e.getSimpleName()) + ">";
                } else {
                    paramName = e.getSimpleName();
                }
                reportMissing("dc.missing.param", paramName);
            }
        }
    }

    @Override // com.sun.source.util.DocTreeScanner, com.sun.source.doctree.DocTreeVisitor
    public Void visitReference(ReferenceTree tree, Void ignore) {
        String sig = tree.getSignature();
        if (sig.contains("<") || sig.contains(">")) {
            this.env.messages.error(Messages.Group.REFERENCE, tree, "dc.type.arg.not.allowed", new Object[0]);
        }
        Element e = this.env.trees.getElement(getCurrentPath());
        if (e == null) {
            this.env.messages.error(Messages.Group.REFERENCE, tree, "dc.ref.not.found", new Object[0]);
        }
        return (Void) super.visitReference(tree, ignore);
    }

    @Override // com.sun.source.util.DocTreeScanner, com.sun.source.doctree.DocTreeVisitor
    public Void visitReturn(ReturnTree tree, Void ignore) {
        Element e = this.env.trees.getElement(this.env.currPath);
        if (e.getKind() != ElementKind.METHOD || ((ExecutableElement) e).getReturnType().getKind() == TypeKind.VOID) {
            this.env.messages.error(Messages.Group.REFERENCE, tree, "dc.invalid.return", new Object[0]);
        }
        this.foundReturn = true;
        warnIfEmpty(tree, tree.getDescription());
        return (Void) super.visitReturn(tree, ignore);
    }

    @Override // com.sun.source.util.DocTreeScanner, com.sun.source.doctree.DocTreeVisitor
    public Void visitSerialData(SerialDataTree tree, Void ignore) {
        warnIfEmpty(tree, tree.getDescription());
        return (Void) super.visitSerialData(tree, ignore);
    }

    @Override // com.sun.source.util.DocTreeScanner, com.sun.source.doctree.DocTreeVisitor
    public Void visitSerialField(SerialFieldTree tree, Void ignore) {
        warnIfEmpty(tree, tree.getDescription());
        return (Void) super.visitSerialField(tree, ignore);
    }

    @Override // com.sun.source.util.DocTreeScanner, com.sun.source.doctree.DocTreeVisitor
    public Void visitSince(SinceTree tree, Void ignore) {
        warnIfEmpty(tree, tree.getBody());
        return (Void) super.visitSince(tree, ignore);
    }

    @Override // com.sun.source.util.DocTreeScanner, com.sun.source.doctree.DocTreeVisitor
    public Void visitThrows(ThrowsTree tree, Void ignore) {
        ReferenceTree exName = tree.getExceptionName();
        Element ex = this.env.trees.getElement(new DocTreePath(getCurrentPath(), exName));
        if (ex == null) {
            this.env.messages.error(Messages.Group.REFERENCE, tree, "dc.ref.not.found", new Object[0]);
        } else if (!isThrowable(ex.asType())) {
            this.env.messages.error(Messages.Group.REFERENCE, tree, "dc.invalid.throws", new Object[0]);
        } else {
            switch (this.env.currElement.getKind()) {
                case METHOD:
                case CONSTRUCTOR:
                    if (isCheckedException(ex.asType())) {
                        ExecutableElement ee = (ExecutableElement) this.env.currElement;
                        checkThrowsDeclared(exName, ex.asType(), ee.getThrownTypes());
                    }
                    break;
                default:
                    this.env.messages.error(Messages.Group.REFERENCE, tree, "dc.invalid.throws", new Object[0]);
                    break;
            }
        }
        warnIfEmpty(tree, tree.getDescription());
        return scan(tree.getDescription(), ignore);
    }

    private boolean isThrowable(TypeMirror tm) {
        switch (tm.getKind()) {
            case DECLARED:
            case TYPEVAR:
                return this.env.types.isAssignable(tm, this.env.java_lang_Throwable);
            default:
                return false;
        }
    }

    private void checkThrowsDeclared(ReferenceTree tree, TypeMirror t, List<? extends TypeMirror> list) {
        boolean found = false;
        for (TypeMirror tl : list) {
            if (this.env.types.isAssignable(t, tl)) {
                this.foundThrows.add(tl);
                found = true;
            }
        }
        if (!found) {
            this.env.messages.error(Messages.Group.REFERENCE, tree, "dc.exception.not.thrown", t);
        }
    }

    private void checkThrowsDocumented(List<? extends TypeMirror> list) {
        if (this.foundInheritDoc) {
            return;
        }
        for (TypeMirror tl : list) {
            if (isCheckedException(tl) && !this.foundThrows.contains(tl)) {
                reportMissing("dc.missing.throws", tl);
            }
        }
    }

    @Override // com.sun.source.util.DocTreeScanner, com.sun.source.doctree.DocTreeVisitor
    public Void visitUnknownBlockTag(UnknownBlockTagTree tree, Void ignore) {
        checkUnknownTag(tree, tree.getTagName());
        return (Void) super.visitUnknownBlockTag(tree, ignore);
    }

    @Override // com.sun.source.util.DocTreeScanner, com.sun.source.doctree.DocTreeVisitor
    public Void visitUnknownInlineTag(UnknownInlineTagTree tree, Void ignore) {
        checkUnknownTag(tree, tree.getTagName());
        return (Void) super.visitUnknownInlineTag(tree, ignore);
    }

    private void checkUnknownTag(DocTree tree, String tagName) {
        if (this.env.customTags != null && !this.env.customTags.contains(tagName)) {
            this.env.messages.error(Messages.Group.SYNTAX, tree, "dc.tag.unknown", tagName);
        }
    }

    @Override // com.sun.source.util.DocTreeScanner, com.sun.source.doctree.DocTreeVisitor
    public Void visitValue(ValueTree tree, Void ignore) {
        ReferenceTree ref = tree.getReference();
        if (ref == null || ref.getSignature().isEmpty()) {
            if (!isConstant(this.env.currElement)) {
                this.env.messages.error(Messages.Group.REFERENCE, tree, "dc.value.not.allowed.here", new Object[0]);
            }
        } else {
            Element e = this.env.trees.getElement(new DocTreePath(getCurrentPath(), ref));
            if (!isConstant(e)) {
                this.env.messages.error(Messages.Group.REFERENCE, tree, "dc.value.not.a.constant", new Object[0]);
            }
        }
        markEnclosingTag(Flag.HAS_INLINE_TAG);
        return (Void) super.visitValue(tree, ignore);
    }

    private boolean isConstant(Element e) {
        if (e == null) {
            return false;
        }
        switch (e.getKind()) {
            case FIELD:
                Object value = ((VariableElement) e).getConstantValue();
                if (value != null) {
                }
                break;
        }
        return false;
    }

    @Override // com.sun.source.util.DocTreeScanner, com.sun.source.doctree.DocTreeVisitor
    public Void visitVersion(VersionTree tree, Void ignore) {
        warnIfEmpty(tree, tree.getBody());
        return (Void) super.visitVersion(tree, ignore);
    }

    @Override // com.sun.source.util.DocTreeScanner, com.sun.source.doctree.DocTreeVisitor
    public Void visitErroneous(ErroneousTree tree, Void ignore) {
        this.env.messages.error(Messages.Group.SYNTAX, tree, null, tree.getDiagnostic().getMessage(null));
        return null;
    }

    private boolean isCheckedException(TypeMirror t) {
        return (this.env.types.isAssignable(t, this.env.java_lang_Error) || this.env.types.isAssignable(t, this.env.java_lang_RuntimeException)) ? false : true;
    }

    private boolean isSynthetic() {
        switch (this.env.currElement.getKind()) {
            case CONSTRUCTOR:
                TreePath p = this.env.currPath;
                if (this.env.getPos(p) == this.env.getPos(p.getParentPath())) {
                }
                break;
        }
        return false;
    }

    void markEnclosingTag(Flag flag) {
        TagStackItem top = this.tagStack.peek();
        if (top != null) {
            top.flags.add(flag);
        }
    }

    String toString(TreePath p) {
        StringBuilder sb = new StringBuilder("TreePath[");
        toString(p, sb);
        sb.append("]");
        return sb.toString();
    }

    void toString(TreePath p, StringBuilder sb) {
        TreePath parent = p.getParentPath();
        if (parent != null) {
            toString(parent, sb);
            sb.append(DocLint.TAGS_SEPARATOR);
        }
        sb.append(p.getLeaf().getKind()).append(":").append(this.env.getPos(p)).append(":S").append(this.env.getStartPos(p));
    }

    /* JADX WARN: Removed duplicated region for block: B:5:0x000a  */
    /*
        Code decompiled incorrectly, please refer to instructions dump.
        To view partially-correct code enable 'Show inconsistent code' option in preferences
    */
    void warnIfEmpty(com.sun.source.doctree.DocTree r5, java.util.List<? extends com.sun.source.doctree.DocTree> r6) {
        /*
            r4 = this;
            java.util.Iterator r0 = r6.iterator()
        L4:
            boolean r1 = r0.hasNext()
            if (r1 == 0) goto L2b
            java.lang.Object r1 = r0.next()
            com.sun.source.doctree.DocTree r1 = (com.sun.source.doctree.DocTree) r1
            int[] r2 = com.sun.tools.doclint.Checker.AnonymousClass1.$SwitchMap$com$sun$source$doctree$DocTree$Kind
            com.sun.source.doctree.DocTree$Kind r3 = r1.getKind()
            int r3 = r3.ordinal()
            r2 = r2[r3]
            switch(r2) {
                case 4: goto L20;
                default: goto L1f;
            }
        L1f:
            return
        L20:
            r2 = r1
            com.sun.source.doctree.TextTree r2 = (com.sun.source.doctree.TextTree) r2
            boolean r2 = r4.hasNonWhitespace(r2)
            if (r2 == 0) goto L2a
            return
        L2a:
            goto L4
        L2b:
            com.sun.tools.doclint.Env r0 = r4.env
            com.sun.tools.doclint.Messages r0 = r0.messages
            com.sun.tools.doclint.Messages$Group r1 = com.sun.tools.doclint.Messages.Group.SYNTAX
            com.sun.source.doctree.DocTree$Kind r2 = r5.getKind()
            java.lang.String r2 = r2.tagName
            java.lang.Object[] r2 = new java.lang.Object[]{r2}
            java.lang.String r3 = "dc.empty"
            r0.warning(r1, r5, r3, r2)
            return
        */
        throw new UnsupportedOperationException("Method not decompiled: com.sun.tools.doclint.Checker.warnIfEmpty(com.sun.source.doctree.DocTree, java.util.List):void");
    }

    boolean hasNonWhitespace(TextTree tree) {
        String s = tree.getBody();
        for (int i = 0; i < s.length(); i++) {
            if (!Character.isWhitespace(s.charAt(i))) {
                return true;
            }
        }
        return false;
    }
}
