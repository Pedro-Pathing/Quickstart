package com.sun.tools.javac.parser;

import com.sun.source.doctree.AttributeTree;
import com.sun.source.doctree.DocTree;
import com.sun.tools.javac.parser.Tokens;
import com.sun.tools.javac.tree.DCTree;
import com.sun.tools.javac.tree.DocTreeMaker;
import com.sun.tools.javac.tree.JCTree;
import com.sun.tools.javac.util.DiagnosticSource;
import com.sun.tools.javac.util.List;
import com.sun.tools.javac.util.ListBuffer;
import com.sun.tools.javac.util.Name;
import com.sun.tools.javac.util.Names;
import com.sun.tools.javac.util.Options;
import com.sun.tools.javac.util.StringUtils;
import java.text.BreakIterator;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Locale;
import java.util.Map;
import java.util.Set;

/* JADX INFO: loaded from: classes.dex */
public class DocCommentParser {
    protected int bp;
    protected char[] buf;
    protected int buflen;
    protected char ch;
    final Tokens.Comment comment;
    final DiagnosticSource diagSource;
    final ParserFactory fac;
    final DocTreeMaker m;
    final Names names;
    BreakIterator sentenceBreaker;
    Map<Name, TagParser> tagParsers;
    int textStart = -1;
    int lastNonWhite = -1;
    boolean newline = true;
    Set<String> htmlBlockTags = new HashSet(Arrays.asList("h1", "h2", "h3", "h4", "h5", "h6", "p", "pre"));

    static class ParseException extends Exception {
        private static final long serialVersionUID = 0;

        ParseException(String key) {
            super(key);
        }
    }

    DocCommentParser(ParserFactory fac, DiagnosticSource diagSource, Tokens.Comment comment) {
        this.fac = fac;
        this.diagSource = diagSource;
        this.comment = comment;
        this.names = fac.names;
        this.m = fac.docTreeMaker;
        Locale locale = fac.locale == null ? Locale.getDefault() : fac.locale;
        Options options = fac.options;
        boolean useBreakIterator = options.isSet("breakIterator");
        if (useBreakIterator || !locale.getLanguage().equals(Locale.ENGLISH.getLanguage())) {
            this.sentenceBreaker = BreakIterator.getSentenceInstance(locale);
        }
        initTagParsers();
    }

    /* JADX WARN: Multi-variable type inference failed */
    DCTree.DCDocComment parse() {
        String c = this.comment.getText();
        this.buf = new char[c.length() + 1];
        c.getChars(0, c.length(), this.buf, 0);
        this.buf[this.buf.length - 1] = 26;
        this.buflen = this.buf.length - 1;
        this.bp = -1;
        nextChar();
        List listBlockContent = blockContent();
        List<DCTree> tags = blockTags();
        ListBuffer<DCTree> fs = new ListBuffer<>();
        while (true) {
            if (listBlockContent.nonEmpty()) {
                DCTree t = (DCTree) listBlockContent.head;
                switch (t.getKind()) {
                    case TEXT:
                        String s = ((DCTree.DCText) t).getBody();
                        int i = getSentenceBreak(s);
                        if (i > 0) {
                            int i0 = i;
                            while (i0 > 0 && isWhitespace(s.charAt(i0 - 1))) {
                                i0--;
                            }
                            fs.add(this.m.at(t.pos).Text(s.substring(0, i0)));
                            int i1 = i;
                            while (i1 < s.length() && isWhitespace(s.charAt(i1))) {
                                i1++;
                            }
                            listBlockContent = listBlockContent.tail;
                            if (i1 < s.length()) {
                                listBlockContent = listBlockContent.prepend(this.m.at(t.pos + i1).Text(s.substring(i1)));
                            }
                        } else if (listBlockContent.tail.nonEmpty() && isSentenceBreak((DCTree) listBlockContent.tail.head)) {
                            int i02 = s.length() - 1;
                            while (i02 > 0 && isWhitespace(s.charAt(i02))) {
                                i02--;
                            }
                            fs.add(this.m.at(t.pos).Text(s.substring(0, i02 + 1)));
                            listBlockContent = listBlockContent.tail;
                        } else {
                            fs.add(t);
                            listBlockContent = listBlockContent.tail;
                        }
                        break;
                    case START_ELEMENT:
                    case END_ELEMENT:
                        if (!isSentenceBreak(t)) {
                            fs.add(t);
                            listBlockContent = listBlockContent.tail;
                        }
                        break;
                    default:
                        fs.add(t);
                        listBlockContent = listBlockContent.tail;
                        break;
                }
            }
        }
        DCTree first = (DCTree) getFirst(fs.toList(), listBlockContent, tags);
        int pos = first != null ? first.pos : -1;
        DCTree.DCDocComment dc = this.m.at(pos).DocComment(this.comment, fs.toList(), listBlockContent, tags);
        return dc;
    }

    void nextChar() {
        int i;
        char[] cArr = this.buf;
        if (this.bp < this.buflen) {
            i = this.bp + 1;
            this.bp = i;
        } else {
            i = this.buflen;
        }
        this.ch = cArr[i];
        switch (this.ch) {
            case '\n':
            case '\f':
            case '\r':
                this.newline = true;
                break;
        }
    }

    /* JADX WARN: Removed duplicated region for block: B:23:0x0082  */
    /*
        Code decompiled incorrectly, please refer to instructions dump.
        To view partially-correct code enable 'Show inconsistent code' option in preferences
    */
    protected com.sun.tools.javac.util.List<com.sun.tools.javac.tree.DCTree> blockContent() {
        /*
            r7 = this;
            com.sun.tools.javac.util.ListBuffer r0 = new com.sun.tools.javac.util.ListBuffer
            r0.<init>()
            r1 = -1
            r7.textStart = r1
        L8:
            int r2 = r7.bp
            int r3 = r7.buflen
            if (r2 >= r3) goto L8f
            char r2 = r7.ch
            r3 = 1
            r4 = 0
            switch(r2) {
                case 9: goto L78;
                case 10: goto L76;
                case 12: goto L76;
                case 13: goto L76;
                case 32: goto L78;
                case 38: goto L72;
                case 60: goto L58;
                case 62: goto L24;
                case 64: goto L1a;
                case 123: goto L16;
                default: goto L15;
            }
        L15:
            goto L7c
        L16:
            r7.inlineTag(r0)
            goto L8
        L1a:
            boolean r2 = r7.newline
            if (r2 == 0) goto L7c
            int r2 = r7.lastNonWhite
            r7.addPendingText(r0, r2)
            goto L8f
        L24:
            r7.newline = r4
            int r2 = r7.bp
            int r2 = r2 - r3
            r7.addPendingText(r0, r2)
            com.sun.tools.javac.tree.DocTreeMaker r2 = r7.m
            int r5 = r7.bp
            com.sun.tools.javac.tree.DocTreeMaker r2 = r2.at(r5)
            int r5 = r7.bp
            int r6 = r7.bp
            int r6 = r6 + r3
            java.lang.String r3 = r7.newString(r5, r6)
            com.sun.tools.javac.util.DiagnosticSource r5 = r7.diagSource
            java.lang.String r6 = "dc.bad.gt"
            java.lang.Object[] r4 = new java.lang.Object[r4]
            com.sun.tools.javac.tree.DCTree$DCErroneous r2 = r2.Erroneous(r3, r5, r6, r4)
            r0.add(r2)
            r7.nextChar()
            int r2 = r7.textStart
            if (r2 != r1) goto L8
            int r2 = r7.bp
            r7.textStart = r2
            r7.lastNonWhite = r1
            goto L8
        L58:
            r7.newline = r4
            int r2 = r7.bp
            int r2 = r2 - r3
            r7.addPendingText(r0, r2)
            com.sun.tools.javac.tree.DCTree r2 = r7.html()
            r0.add(r2)
            int r2 = r7.textStart
            if (r2 != r1) goto L8
            int r2 = r7.bp
            r7.textStart = r2
            r7.lastNonWhite = r1
            goto L8
        L72:
            r7.entity(r0)
            goto L8
        L76:
            r7.newline = r3
        L78:
            r7.nextChar()
            goto L8
        L7c:
            r7.newline = r4
            int r2 = r7.textStart
            if (r2 != r1) goto L86
            int r2 = r7.bp
            r7.textStart = r2
        L86:
            int r2 = r7.bp
            r7.lastNonWhite = r2
            r7.nextChar()
            goto L8
        L8f:
            int r2 = r7.lastNonWhite
            if (r2 == r1) goto L98
            int r1 = r7.lastNonWhite
            r7.addPendingText(r0, r1)
        L98:
            com.sun.tools.javac.util.List r1 = r0.toList()
            return r1
        */
        throw new UnsupportedOperationException("Method not decompiled: com.sun.tools.javac.parser.DocCommentParser.blockContent():com.sun.tools.javac.util.List");
    }

    protected List<DCTree> blockTags() {
        ListBuffer<DCTree> tags = new ListBuffer<>();
        while (this.ch == '@') {
            tags.add(blockTag());
        }
        return tags.toList();
    }

    protected DCTree blockTag() {
        int p = this.bp;
        try {
            nextChar();
            if (isIdentifierStart(this.ch)) {
                Name name = readTagName();
                TagParser tp = this.tagParsers.get(name);
                if (tp == null) {
                    List<DCTree> content = blockContent();
                    return this.m.at(p).UnknownBlockTag(name, content);
                }
                switch (tp.getKind()) {
                    case BLOCK:
                        return tp.parse(p);
                    case INLINE:
                        return erroneous("dc.bad.inline.tag", p);
                }
            }
            blockContent();
            return erroneous("dc.no.tag.name", p);
        } catch (ParseException e) {
            blockContent();
            return erroneous(e.getMessage(), p);
        }
    }

    protected void inlineTag(ListBuffer<DCTree> list) {
        this.newline = false;
        nextChar();
        if (this.ch == '@') {
            addPendingText(list, this.bp - 2);
            list.add(inlineTag());
            this.textStart = this.bp;
            this.lastNonWhite = -1;
            return;
        }
        if (this.textStart == -1) {
            this.textStart = this.bp - 1;
        }
        this.lastNonWhite = this.bp;
    }

    protected DCTree inlineTag() {
        int p = this.bp - 1;
        try {
            nextChar();
            if (isIdentifierStart(this.ch)) {
                Name name = readTagName();
                skipWhitespace();
                TagParser tp = this.tagParsers.get(name);
                if (tp == null) {
                    DCTree text = inlineText();
                    if (text != null) {
                        nextChar();
                        return this.m.at(p).UnknownInlineTag(name, List.of(text)).setEndPos(this.bp);
                    }
                } else if (tp.getKind() == TagParser.Kind.INLINE) {
                    DCTree.DCEndPosTree<?> tree = (DCTree.DCEndPosTree) tp.parse(p);
                    if (tree != null) {
                        return tree.setEndPos(this.bp);
                    }
                } else {
                    inlineText();
                    nextChar();
                }
            }
            return erroneous("dc.no.tag.name", p);
        } catch (ParseException e) {
            return erroneous(e.getMessage(), p);
        }
    }

    protected DCTree inlineText() throws ParseException {
        skipWhitespace();
        int pos = this.bp;
        int depth = 1;
        while (this.bp < this.buflen) {
            switch (this.ch) {
                case '\t':
                case ' ':
                    nextChar();
                    break;
                case '\n':
                case '\f':
                case '\r':
                    this.newline = true;
                    continue;
                    nextChar();
                    break;
                case '@':
                    if (!this.newline) {
                        this.newline = false;
                        this.lastNonWhite = this.bp;
                        nextChar();
                    }
                    break;
                case '{':
                    this.newline = false;
                    this.lastNonWhite = this.bp;
                    depth++;
                    continue;
                    nextChar();
                    break;
                case '}':
                    depth--;
                    if (depth == 0) {
                        return this.m.at(pos).Text(newString(pos, this.bp));
                    }
                    this.newline = false;
                    this.lastNonWhite = this.bp;
                    continue;
                    nextChar();
                    break;
                    break;
                default:
                    this.newline = false;
                    this.lastNonWhite = this.bp;
                    continue;
                    nextChar();
                    break;
            }
            throw new ParseException("dc.unterminated.inline.tag");
        }
        throw new ParseException("dc.unterminated.inline.tag");
    }

    /* JADX WARN: Can't fix incorrect switch cases order, some code will duplicate */
    /* JADX WARN: Removed duplicated region for block: B:54:0x0034 A[SYNTHETIC] */
    /* JADX WARN: Removed duplicated region for block: B:61:0x0030 A[SYNTHETIC] */
    /*
        Code decompiled incorrectly, please refer to instructions dump.
        To view partially-correct code enable 'Show inconsistent code' option in preferences
    */
    protected com.sun.tools.javac.tree.DCTree.DCReference reference(boolean r12) throws com.sun.tools.javac.parser.DocCommentParser.ParseException {
        /*
            Method dump skipped, instruction units count: 282
            To view this dump change 'Code comments level' option to 'DEBUG'
        */
        throw new UnsupportedOperationException("Method not decompiled: com.sun.tools.javac.parser.DocCommentParser.reference(boolean):com.sun.tools.javac.tree.DCTree$DCReference");
    }

    JCTree parseType(String s) throws ParseException {
        JavacParser p = this.fac.newParser(s, false, false, false);
        JCTree tree = p.parseType();
        if (p.token().kind != Tokens.TokenKind.EOF) {
            throw new ParseException("dc.ref.unexpected.input");
        }
        return tree;
    }

    Name parseMember(String s) throws ParseException {
        JavacParser p = this.fac.newParser(s, false, false, false);
        Name name = p.ident();
        if (p.token().kind != Tokens.TokenKind.EOF) {
            throw new ParseException("dc.ref.unexpected.input");
        }
        return name;
    }

    List<JCTree> parseParams(String s) throws ParseException {
        if (s.trim().isEmpty()) {
            return List.nil();
        }
        JavacParser p = this.fac.newParser(s.replace("...", "[]"), false, false, false);
        ListBuffer<JCTree> paramTypes = new ListBuffer<>();
        paramTypes.add(p.parseType());
        if (p.token().kind == Tokens.TokenKind.IDENTIFIER) {
            p.nextToken();
        }
        while (p.token().kind == Tokens.TokenKind.COMMA) {
            p.nextToken();
            paramTypes.add(p.parseType());
            if (p.token().kind == Tokens.TokenKind.IDENTIFIER) {
                p.nextToken();
            }
        }
        if (p.token().kind != Tokens.TokenKind.EOF) {
            throw new ParseException("dc.ref.unexpected.input");
        }
        return paramTypes.toList();
    }

    protected DCTree.DCIdentifier identifier() throws ParseException {
        skipWhitespace();
        int pos = this.bp;
        if (isJavaIdentifierStart(this.ch)) {
            Name name = readJavaIdentifier();
            return this.m.at(pos).Identifier(name);
        }
        throw new ParseException("dc.identifier.expected");
    }

    protected DCTree.DCText quotedString() {
        int pos = this.bp;
        nextChar();
        while (this.bp < this.buflen) {
            switch (this.ch) {
                case '\n':
                case '\f':
                case '\r':
                    this.newline = true;
                    break;
                case '\"':
                    nextChar();
                    return this.m.at(pos).Text(newString(pos, this.bp));
                case '@':
                    if (this.newline) {
                        return null;
                    }
                    break;
            }
            nextChar();
        }
        return null;
    }

    /* JADX WARN: Removed duplicated region for block: B:23:0x005f  */
    /*
        Code decompiled incorrectly, please refer to instructions dump.
        To view partially-correct code enable 'Show inconsistent code' option in preferences
    */
    protected com.sun.tools.javac.util.List<com.sun.tools.javac.tree.DCTree> inlineContent() {
        /*
            r7 = this;
            com.sun.tools.javac.util.ListBuffer r0 = new com.sun.tools.javac.util.ListBuffer
            r0.<init>()
            r7.skipWhitespace()
            int r1 = r7.bp
            r2 = 1
            r3 = -1
            r7.textStart = r3
        Le:
            int r4 = r7.bp
            int r5 = r7.buflen
            if (r4 >= r5) goto L67
            char r4 = r7.ch
            r5 = 0
            r6 = 1
            switch(r4) {
                case 9: goto L57;
                case 10: goto L55;
                case 12: goto L55;
                case 13: goto L55;
                case 32: goto L57;
                case 38: goto L51;
                case 60: goto L41;
                case 64: goto L3c;
                case 123: goto L34;
                case 125: goto L1c;
                default: goto L1b;
            }
        L1b:
            goto L5b
        L1c:
            r7.newline = r5
            int r2 = r2 + (-1)
            if (r2 != 0) goto L30
            int r3 = r7.bp
            int r3 = r3 - r6
            r7.addPendingText(r0, r3)
            r7.nextChar()
            com.sun.tools.javac.util.List r3 = r0.toList()
            return r3
        L30:
            r7.nextChar()
            goto Le
        L34:
            r7.newline = r5
            int r2 = r2 + 1
            r7.nextChar()
            goto Le
        L3c:
            boolean r4 = r7.newline
            if (r4 == 0) goto L5b
            goto L67
        L41:
            r7.newline = r5
            int r4 = r7.bp
            int r4 = r4 - r6
            r7.addPendingText(r0, r4)
            com.sun.tools.javac.tree.DCTree r4 = r7.html()
            r0.add(r4)
            goto Le
        L51:
            r7.entity(r0)
            goto Le
        L55:
            r7.newline = r6
        L57:
            r7.nextChar()
            goto Le
        L5b:
            int r4 = r7.textStart
            if (r4 != r3) goto L63
            int r4 = r7.bp
            r7.textStart = r4
        L63:
            r7.nextChar()
            goto Le
        L67:
            java.lang.String r3 = "dc.unterminated.inline.tag"
            com.sun.tools.javac.tree.DCTree$DCErroneous r3 = r7.erroneous(r3, r1)
            com.sun.tools.javac.util.List r3 = com.sun.tools.javac.util.List.of(r3)
            return r3
        */
        throw new UnsupportedOperationException("Method not decompiled: com.sun.tools.javac.parser.DocCommentParser.inlineContent():com.sun.tools.javac.util.List");
    }

    protected void entity(ListBuffer<DCTree> list) {
        this.newline = false;
        addPendingText(list, this.bp - 1);
        list.add(entity());
        if (this.textStart == -1) {
            this.textStart = this.bp;
            this.lastNonWhite = -1;
        }
    }

    protected DCTree entity() {
        int p = this.bp;
        nextChar();
        Name name = null;
        if (this.ch == '#') {
            int namep = this.bp;
            nextChar();
            if (isDecimalDigit(this.ch)) {
                nextChar();
                while (isDecimalDigit(this.ch)) {
                    nextChar();
                }
                name = this.names.fromChars(this.buf, namep, this.bp - namep);
            } else if (this.ch == 'x' || this.ch == 'X') {
                nextChar();
                if (isHexDigit(this.ch)) {
                    nextChar();
                    while (isHexDigit(this.ch)) {
                        nextChar();
                    }
                    name = this.names.fromChars(this.buf, namep, this.bp - namep);
                }
            }
        } else if (isIdentifierStart(this.ch)) {
            name = readIdentifier();
        }
        if (name == null) {
            return erroneous("dc.bad.entity", p);
        }
        if (this.ch != ';') {
            return erroneous("dc.missing.semicolon", p);
        }
        nextChar();
        return this.m.at(p).Entity(name);
    }

    protected DCTree html() {
        int p = this.bp;
        nextChar();
        if (isIdentifierStart(this.ch)) {
            Name name = readIdentifier();
            List<DCTree> attrs = htmlAttrs();
            if (attrs != null) {
                boolean selfClosing = false;
                if (this.ch == '/') {
                    nextChar();
                    selfClosing = true;
                }
                if (this.ch == '>') {
                    nextChar();
                    return this.m.at(p).StartElement(name, attrs, selfClosing).setEndPos(this.bp);
                }
            }
        } else if (this.ch == '/') {
            nextChar();
            if (isIdentifierStart(this.ch)) {
                Name name2 = readIdentifier();
                skipWhitespace();
                if (this.ch == '>') {
                    nextChar();
                    return this.m.at(p).EndElement(name2);
                }
            }
        } else if (this.ch == '!') {
            nextChar();
            if (this.ch == '-') {
                nextChar();
                if (this.ch == '-') {
                    nextChar();
                    while (this.bp < this.buflen) {
                        int dash = 0;
                        while (this.ch == '-') {
                            dash++;
                            nextChar();
                        }
                        if (dash >= 2 && this.ch == '>') {
                            nextChar();
                            return this.m.at(p).Comment(newString(p, this.bp));
                        }
                        nextChar();
                    }
                }
            }
        }
        this.bp = p + 1;
        this.ch = this.buf[this.bp];
        return erroneous("dc.malformed.html", p);
    }

    protected List<DCTree> htmlAttrs() {
        ListBuffer<DCTree> attrs = new ListBuffer<>();
        skipWhitespace();
        loop0: while (true) {
            if (!isIdentifierStart(this.ch)) {
                break;
            }
            int namePos = this.bp;
            Name name = readIdentifier();
            skipWhitespace();
            List<DCTree> value = null;
            AttributeTree.ValueKind vkind = AttributeTree.ValueKind.EMPTY;
            if (this.ch == '=') {
                ListBuffer<DCTree> v = new ListBuffer<>();
                nextChar();
                skipWhitespace();
                if (this.ch == '\'' || this.ch == '\"') {
                    vkind = this.ch == '\'' ? AttributeTree.ValueKind.SINGLE : AttributeTree.ValueKind.DOUBLE;
                    char quote = this.ch;
                    nextChar();
                    this.textStart = this.bp;
                    while (this.bp < this.buflen && this.ch != quote) {
                        if (this.newline && this.ch == '@') {
                            attrs.add(erroneous("dc.unterminated.string", namePos));
                            break loop0;
                        }
                        attrValueChar(v);
                    }
                    addPendingText(v, this.bp - 1);
                    nextChar();
                } else {
                    vkind = AttributeTree.ValueKind.UNQUOTED;
                    this.textStart = this.bp;
                    while (this.bp < this.buflen && !isUnquotedAttrValueTerminator(this.ch)) {
                        attrValueChar(v);
                    }
                    addPendingText(v, this.bp - 1);
                }
                skipWhitespace();
                value = v.toList();
            }
            DCTree.DCAttribute attr = this.m.at(namePos).Attribute(name, vkind, value);
            attrs.add(attr);
        }
        return attrs.toList();
    }

    protected void attrValueChar(ListBuffer<DCTree> list) {
        switch (this.ch) {
            case '&':
                entity(list);
                break;
            case '{':
                inlineTag(list);
                break;
            default:
                nextChar();
                break;
        }
    }

    protected void addPendingText(ListBuffer<DCTree> list, int textEnd) {
        if (this.textStart != -1) {
            if (this.textStart <= textEnd) {
                list.add(this.m.at(this.textStart).Text(newString(this.textStart, textEnd + 1)));
            }
            this.textStart = -1;
        }
    }

    protected DCTree.DCErroneous erroneous(String code, int pos) {
        for (int i = this.bp - 1; i > pos; i--) {
            switch (this.buf[i]) {
                case '\t':
                case ' ':
                    break;
                case '\n':
                case '\f':
                case '\r':
                    this.newline = true;
                    break;
                default:
                    this.textStart = -1;
                    return this.m.at(pos).Erroneous(newString(pos, i + 1), this.diagSource, code, new Object[0]);
            }
        }
        this.textStart = -1;
        return this.m.at(pos).Erroneous(newString(pos, i + 1), this.diagSource, code, new Object[0]);
    }

    <T> T getFirst(List<T>... lists) {
        for (List<T> list : lists) {
            if (list.nonEmpty()) {
                return list.head;
            }
        }
        return null;
    }

    protected boolean isIdentifierStart(char ch) {
        return Character.isUnicodeIdentifierStart(ch);
    }

    protected Name readIdentifier() {
        int start = this.bp;
        nextChar();
        while (this.bp < this.buflen && Character.isUnicodeIdentifierPart(this.ch)) {
            nextChar();
        }
        return this.names.fromChars(this.buf, start, this.bp - start);
    }

    protected Name readTagName() {
        int start = this.bp;
        nextChar();
        while (this.bp < this.buflen && (Character.isUnicodeIdentifierPart(this.ch) || this.ch == '.')) {
            nextChar();
        }
        return this.names.fromChars(this.buf, start, this.bp - start);
    }

    protected boolean isJavaIdentifierStart(char ch) {
        return Character.isJavaIdentifierStart(ch);
    }

    protected Name readJavaIdentifier() {
        int start = this.bp;
        nextChar();
        while (this.bp < this.buflen && Character.isJavaIdentifierPart(this.ch)) {
            nextChar();
        }
        return this.names.fromChars(this.buf, start, this.bp - start);
    }

    protected boolean isDecimalDigit(char ch) {
        return '0' <= ch && ch <= '9';
    }

    protected boolean isHexDigit(char ch) {
        return ('0' <= ch && ch <= '9') || ('a' <= ch && ch <= 'f') || ('A' <= ch && ch <= 'F');
    }

    protected boolean isUnquotedAttrValueTerminator(char ch) {
        switch (ch) {
            case '\t':
            case '\n':
            case '\f':
            case '\r':
            case ' ':
            case '\"':
            case '\'':
            case '<':
            case '=':
            case '>':
            case '`':
                return true;
            default:
                return false;
        }
    }

    protected boolean isWhitespace(char ch) {
        return Character.isWhitespace(ch);
    }

    protected void skipWhitespace() {
        while (isWhitespace(this.ch)) {
            nextChar();
        }
    }

    protected int getSentenceBreak(String s) {
        if (this.sentenceBreaker != null) {
            this.sentenceBreaker.setText(s);
            int i = this.sentenceBreaker.next();
            if (i == s.length()) {
                return -1;
            }
            return i;
        }
        int i2 = 0;
        for (int i3 = 0; i3 < s.length(); i3++) {
            switch (s.charAt(i3)) {
                case '\t':
                case '\n':
                case '\f':
                case '\r':
                case ' ':
                    if (i2 != 0) {
                        return i3;
                    }
                    break;
                    break;
                case '.':
                    i2 = 1;
                    break;
                default:
                    i2 = 0;
                    break;
            }
        }
        return -1;
    }

    protected boolean isSentenceBreak(Name n) {
        return this.htmlBlockTags.contains(StringUtils.toLowerCase(n.toString()));
    }

    protected boolean isSentenceBreak(DCTree t) {
        switch (t.getKind()) {
            case START_ELEMENT:
                return isSentenceBreak(((DCTree.DCStartElement) t).getName());
            case END_ELEMENT:
                return isSentenceBreak(((DCTree.DCEndElement) t).getName());
            default:
                return false;
        }
    }

    String newString(int start, int end) {
        return new String(this.buf, start, end - start);
    }

    static abstract class TagParser {
        Kind kind;
        DocTree.Kind treeKind;

        enum Kind {
            INLINE,
            BLOCK
        }

        abstract DCTree parse(int i) throws ParseException;

        TagParser(Kind k, DocTree.Kind tk) {
            this.kind = k;
            this.treeKind = tk;
        }

        Kind getKind() {
            return this.kind;
        }

        DocTree.Kind getTreeKind() {
            return this.treeKind;
        }
    }

    private void initTagParsers() {
        TagParser[] parsers = {new TagParser(TagParser.Kind.BLOCK, DocTree.Kind.AUTHOR) { // from class: com.sun.tools.javac.parser.DocCommentParser.1
            @Override // com.sun.tools.javac.parser.DocCommentParser.TagParser
            public DCTree parse(int pos) {
                List<DCTree> name = DocCommentParser.this.blockContent();
                return DocCommentParser.this.m.at(pos).Author(name);
            }
        }, new TagParser(TagParser.Kind.INLINE, DocTree.Kind.CODE) { // from class: com.sun.tools.javac.parser.DocCommentParser.2
            @Override // com.sun.tools.javac.parser.DocCommentParser.TagParser
            public DCTree parse(int pos) throws ParseException {
                DCTree text = DocCommentParser.this.inlineText();
                DocCommentParser.this.nextChar();
                return DocCommentParser.this.m.at(pos).Code((DCTree.DCText) text);
            }
        }, new TagParser(TagParser.Kind.BLOCK, DocTree.Kind.DEPRECATED) { // from class: com.sun.tools.javac.parser.DocCommentParser.3
            @Override // com.sun.tools.javac.parser.DocCommentParser.TagParser
            public DCTree parse(int pos) {
                List<DCTree> reason = DocCommentParser.this.blockContent();
                return DocCommentParser.this.m.at(pos).Deprecated(reason);
            }
        }, new TagParser(TagParser.Kind.INLINE, DocTree.Kind.DOC_ROOT) { // from class: com.sun.tools.javac.parser.DocCommentParser.4
            @Override // com.sun.tools.javac.parser.DocCommentParser.TagParser
            public DCTree parse(int pos) throws ParseException {
                if (DocCommentParser.this.ch == '}') {
                    DocCommentParser.this.nextChar();
                    return DocCommentParser.this.m.at(pos).DocRoot();
                }
                DocCommentParser.this.inlineText();
                DocCommentParser.this.nextChar();
                throw new ParseException("dc.unexpected.content");
            }
        }, new TagParser(TagParser.Kind.BLOCK, DocTree.Kind.EXCEPTION) { // from class: com.sun.tools.javac.parser.DocCommentParser.5
            @Override // com.sun.tools.javac.parser.DocCommentParser.TagParser
            public DCTree parse(int pos) throws ParseException {
                DocCommentParser.this.skipWhitespace();
                DCTree.DCReference ref = DocCommentParser.this.reference(false);
                List<DCTree> description = DocCommentParser.this.blockContent();
                return DocCommentParser.this.m.at(pos).Exception(ref, description);
            }
        }, new TagParser(TagParser.Kind.INLINE, DocTree.Kind.INHERIT_DOC) { // from class: com.sun.tools.javac.parser.DocCommentParser.6
            @Override // com.sun.tools.javac.parser.DocCommentParser.TagParser
            public DCTree parse(int pos) throws ParseException {
                if (DocCommentParser.this.ch == '}') {
                    DocCommentParser.this.nextChar();
                    return DocCommentParser.this.m.at(pos).InheritDoc();
                }
                DocCommentParser.this.inlineText();
                DocCommentParser.this.nextChar();
                throw new ParseException("dc.unexpected.content");
            }
        }, new TagParser(TagParser.Kind.INLINE, DocTree.Kind.LINK) { // from class: com.sun.tools.javac.parser.DocCommentParser.7
            @Override // com.sun.tools.javac.parser.DocCommentParser.TagParser
            public DCTree parse(int pos) throws ParseException {
                DCTree.DCReference ref = DocCommentParser.this.reference(true);
                List<DCTree> label = DocCommentParser.this.inlineContent();
                return DocCommentParser.this.m.at(pos).Link(ref, label);
            }
        }, new TagParser(TagParser.Kind.INLINE, DocTree.Kind.LINK_PLAIN) { // from class: com.sun.tools.javac.parser.DocCommentParser.8
            @Override // com.sun.tools.javac.parser.DocCommentParser.TagParser
            public DCTree parse(int pos) throws ParseException {
                DCTree.DCReference ref = DocCommentParser.this.reference(true);
                List<DCTree> label = DocCommentParser.this.inlineContent();
                return DocCommentParser.this.m.at(pos).LinkPlain(ref, label);
            }
        }, new TagParser(TagParser.Kind.INLINE, DocTree.Kind.LITERAL) { // from class: com.sun.tools.javac.parser.DocCommentParser.9
            @Override // com.sun.tools.javac.parser.DocCommentParser.TagParser
            public DCTree parse(int pos) throws ParseException {
                DCTree text = DocCommentParser.this.inlineText();
                DocCommentParser.this.nextChar();
                return DocCommentParser.this.m.at(pos).Literal((DCTree.DCText) text);
            }
        }, new TagParser(TagParser.Kind.BLOCK, DocTree.Kind.PARAM) { // from class: com.sun.tools.javac.parser.DocCommentParser.10
            @Override // com.sun.tools.javac.parser.DocCommentParser.TagParser
            public DCTree parse(int pos) throws ParseException {
                DocCommentParser.this.skipWhitespace();
                boolean typaram = false;
                if (DocCommentParser.this.ch == '<') {
                    typaram = true;
                    DocCommentParser.this.nextChar();
                }
                DCTree.DCIdentifier id = DocCommentParser.this.identifier();
                if (typaram) {
                    if (DocCommentParser.this.ch != '>') {
                        throw new ParseException("dc.gt.expected");
                    }
                    DocCommentParser.this.nextChar();
                }
                DocCommentParser.this.skipWhitespace();
                List<DCTree> desc = DocCommentParser.this.blockContent();
                return DocCommentParser.this.m.at(pos).Param(typaram, id, desc);
            }
        }, new TagParser(TagParser.Kind.BLOCK, DocTree.Kind.RETURN) { // from class: com.sun.tools.javac.parser.DocCommentParser.11
            @Override // com.sun.tools.javac.parser.DocCommentParser.TagParser
            public DCTree parse(int pos) {
                List<DCTree> description = DocCommentParser.this.blockContent();
                return DocCommentParser.this.m.at(pos).Return(description);
            }
        }, new TagParser(TagParser.Kind.BLOCK, DocTree.Kind.SEE) { // from class: com.sun.tools.javac.parser.DocCommentParser.12
            @Override // com.sun.tools.javac.parser.DocCommentParser.TagParser
            public DCTree parse(int pos) throws ParseException {
                DocCommentParser.this.skipWhitespace();
                switch (DocCommentParser.this.ch) {
                    case 26:
                        if (DocCommentParser.this.bp == DocCommentParser.this.buf.length - 1) {
                            throw new ParseException("dc.no.content");
                        }
                        break;
                    case '\"':
                        DCTree.DCText string = DocCommentParser.this.quotedString();
                        if (string != null) {
                            DocCommentParser.this.skipWhitespace();
                            if (DocCommentParser.this.ch == '@' || (DocCommentParser.this.ch == 26 && DocCommentParser.this.bp == DocCommentParser.this.buf.length - 1)) {
                                return DocCommentParser.this.m.at(pos).See(List.of(string));
                            }
                        }
                        break;
                    case '<':
                        List<DCTree> html = DocCommentParser.this.blockContent();
                        if (html != null) {
                            return DocCommentParser.this.m.at(pos).See(html);
                        }
                        break;
                    case '@':
                        if (DocCommentParser.this.newline) {
                            throw new ParseException("dc.no.content");
                        }
                        break;
                    default:
                        if (DocCommentParser.this.isJavaIdentifierStart(DocCommentParser.this.ch) || DocCommentParser.this.ch == '#') {
                            DCTree.DCReference ref = DocCommentParser.this.reference(true);
                            List<DCTree> description = DocCommentParser.this.blockContent();
                            return DocCommentParser.this.m.at(pos).See(description.prepend(ref));
                        }
                        break;
                }
                throw new ParseException("dc.unexpected.content");
            }
        }, new TagParser(TagParser.Kind.BLOCK, DocTree.Kind.SERIAL_DATA) { // from class: com.sun.tools.javac.parser.DocCommentParser.13
            @Override // com.sun.tools.javac.parser.DocCommentParser.TagParser
            public DCTree parse(int pos) {
                List<DCTree> description = DocCommentParser.this.blockContent();
                return DocCommentParser.this.m.at(pos).SerialData(description);
            }
        }, new TagParser(TagParser.Kind.BLOCK, DocTree.Kind.SERIAL_FIELD) { // from class: com.sun.tools.javac.parser.DocCommentParser.14
            @Override // com.sun.tools.javac.parser.DocCommentParser.TagParser
            public DCTree parse(int pos) throws ParseException {
                DocCommentParser.this.skipWhitespace();
                DCTree.DCIdentifier name = DocCommentParser.this.identifier();
                DocCommentParser.this.skipWhitespace();
                DCTree.DCReference type = DocCommentParser.this.reference(false);
                List<DCTree> description = null;
                if (DocCommentParser.this.isWhitespace(DocCommentParser.this.ch)) {
                    DocCommentParser.this.skipWhitespace();
                    description = DocCommentParser.this.blockContent();
                }
                return DocCommentParser.this.m.at(pos).SerialField(name, type, description);
            }
        }, new TagParser(TagParser.Kind.BLOCK, DocTree.Kind.SERIAL) { // from class: com.sun.tools.javac.parser.DocCommentParser.15
            @Override // com.sun.tools.javac.parser.DocCommentParser.TagParser
            public DCTree parse(int pos) {
                List<DCTree> description = DocCommentParser.this.blockContent();
                return DocCommentParser.this.m.at(pos).Serial(description);
            }
        }, new TagParser(TagParser.Kind.BLOCK, DocTree.Kind.SINCE) { // from class: com.sun.tools.javac.parser.DocCommentParser.16
            @Override // com.sun.tools.javac.parser.DocCommentParser.TagParser
            public DCTree parse(int pos) {
                List<DCTree> description = DocCommentParser.this.blockContent();
                return DocCommentParser.this.m.at(pos).Since(description);
            }
        }, new TagParser(TagParser.Kind.BLOCK, DocTree.Kind.THROWS) { // from class: com.sun.tools.javac.parser.DocCommentParser.17
            @Override // com.sun.tools.javac.parser.DocCommentParser.TagParser
            public DCTree parse(int pos) throws ParseException {
                DocCommentParser.this.skipWhitespace();
                DCTree.DCReference ref = DocCommentParser.this.reference(false);
                List<DCTree> description = DocCommentParser.this.blockContent();
                return DocCommentParser.this.m.at(pos).Throws(ref, description);
            }
        }, new TagParser(TagParser.Kind.INLINE, DocTree.Kind.VALUE) { // from class: com.sun.tools.javac.parser.DocCommentParser.18
            @Override // com.sun.tools.javac.parser.DocCommentParser.TagParser
            public DCTree parse(int pos) throws ParseException {
                DCTree.DCReference ref = DocCommentParser.this.reference(true);
                DocCommentParser.this.skipWhitespace();
                if (DocCommentParser.this.ch == '}') {
                    DocCommentParser.this.nextChar();
                    return DocCommentParser.this.m.at(pos).Value(ref);
                }
                DocCommentParser.this.nextChar();
                throw new ParseException("dc.unexpected.content");
            }
        }, new TagParser(TagParser.Kind.BLOCK, DocTree.Kind.VERSION) { // from class: com.sun.tools.javac.parser.DocCommentParser.19
            @Override // com.sun.tools.javac.parser.DocCommentParser.TagParser
            public DCTree parse(int pos) {
                List<DCTree> description = DocCommentParser.this.blockContent();
                return DocCommentParser.this.m.at(pos).Version(description);
            }
        }};
        this.tagParsers = new HashMap();
        for (TagParser p : parsers) {
            this.tagParsers.put(this.names.fromString(p.getTreeKind().tagName), p);
        }
    }
}
