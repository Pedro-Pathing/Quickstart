package com.sun.tools.javac.parser;

import com.qualcomm.ftccommon.configuration.RobotConfigFileManager;
import com.sun.tools.doclint.DocLint;
import com.sun.tools.javac.api.Formattable;
import com.sun.tools.javac.api.Messages;
import com.sun.tools.javac.util.Context;
import com.sun.tools.javac.util.Filter;
import com.sun.tools.javac.util.List;
import com.sun.tools.javac.util.ListBuffer;
import com.sun.tools.javac.util.Name;
import com.sun.tools.javac.util.Names;
import java.util.Locale;
import org.firstinspires.ftc.onbotjava.OnBotJavaFileSystemUtils;
import org.firstinspires.ftc.onbotjava.RequestConditions;
import org.slf4j.Marker;

/* JADX INFO: loaded from: classes.dex */
public class Tokens {
    private final TokenKind[] key;
    private final Names names;
    public static final Context.Key<Tokens> tokensKey = new Context.Key<>();
    public static final Token DUMMY = new Token(TokenKind.ERROR, 0, 0, null);
    private int maxKey = 0;
    private Name[] tokenName = new Name[TokenKind.values().length];

    public interface Comment {

        public enum CommentStyle {
            LINE,
            BLOCK,
            JAVADOC
        }

        int getSourcePos(int i);

        CommentStyle getStyle();

        String getText();

        boolean isDeprecated();
    }

    public static Tokens instance(Context context) {
        Tokens instance = (Tokens) context.get(tokensKey);
        if (instance == null) {
            return new Tokens(context);
        }
        return instance;
    }

    protected Tokens(Context context) {
        context.put(tokensKey, this);
        this.names = Names.instance(context);
        for (TokenKind t : TokenKind.values()) {
            if (t.name != null) {
                enterKeyword(t.name, t);
            } else {
                this.tokenName[t.ordinal()] = null;
            }
        }
        this.key = new TokenKind[this.maxKey + 1];
        for (int i = 0; i <= this.maxKey; i++) {
            this.key[i] = TokenKind.IDENTIFIER;
        }
        for (TokenKind t2 : TokenKind.values()) {
            if (t2.name != null) {
                this.key[this.tokenName[t2.ordinal()].getIndex()] = t2;
            }
        }
    }

    private void enterKeyword(String s, TokenKind token) {
        Name n = this.names.fromString(s);
        this.tokenName[token.ordinal()] = n;
        if (n.getIndex() > this.maxKey) {
            this.maxKey = n.getIndex();
        }
    }

    TokenKind lookupKind(Name name) {
        return name.getIndex() > this.maxKey ? TokenKind.IDENTIFIER : this.key[name.getIndex()];
    }

    TokenKind lookupKind(String name) {
        return lookupKind(this.names.fromString(name));
    }

    public enum TokenKind implements Formattable, Filter<TokenKind> {
        EOF,
        ERROR,
        IDENTIFIER(Token.Tag.NAMED),
        ABSTRACT("abstract"),
        ASSERT("assert", Token.Tag.NAMED),
        BOOLEAN("boolean", Token.Tag.NAMED),
        BREAK("break"),
        BYTE("byte", Token.Tag.NAMED),
        CASE("case"),
        CATCH("catch"),
        CHAR("char", Token.Tag.NAMED),
        CLASS("class"),
        CONST("const"),
        CONTINUE("continue"),
        DEFAULT("default"),
        DO("do"),
        DOUBLE("double", Token.Tag.NAMED),
        ELSE("else"),
        ENUM("enum", Token.Tag.NAMED),
        EXTENDS("extends"),
        FINAL("final"),
        FINALLY("finally"),
        FLOAT("float", Token.Tag.NAMED),
        FOR("for"),
        GOTO("goto"),
        IF("if"),
        IMPLEMENTS("implements"),
        IMPORT("import"),
        INSTANCEOF("instanceof"),
        INT("int", Token.Tag.NAMED),
        INTERFACE("interface"),
        LONG("long", Token.Tag.NAMED),
        NATIVE("native"),
        NEW(RequestConditions.REQUEST_KEY_NEW),
        PACKAGE("package"),
        PRIVATE("private"),
        PROTECTED("protected"),
        PUBLIC("public"),
        RETURN("return"),
        SHORT("short", Token.Tag.NAMED),
        STATIC("static"),
        STRICTFP("strictfp"),
        SUPER("super", Token.Tag.NAMED),
        SWITCH("switch"),
        SYNCHRONIZED("synchronized"),
        THIS("this", Token.Tag.NAMED),
        THROW("throw"),
        THROWS("throws"),
        TRANSIENT("transient"),
        TRY("try"),
        VOID("void", Token.Tag.NAMED),
        VOLATILE("volatile"),
        WHILE("while"),
        INTLITERAL(Token.Tag.NUMERIC),
        LONGLITERAL(Token.Tag.NUMERIC),
        FLOATLITERAL(Token.Tag.NUMERIC),
        DOUBLELITERAL(Token.Tag.NUMERIC),
        CHARLITERAL(Token.Tag.NUMERIC),
        STRINGLITERAL(Token.Tag.STRING),
        TRUE("true", Token.Tag.NAMED),
        FALSE("false", Token.Tag.NAMED),
        NULL("null", Token.Tag.NAMED),
        UNDERSCORE("_", Token.Tag.NAMED),
        ARROW("->"),
        COLCOL("::"),
        LPAREN("("),
        RPAREN(")"),
        LBRACE("{"),
        RBRACE("}"),
        LBRACKET("["),
        RBRACKET("]"),
        SEMI(RobotConfigFileManager.FILE_LIST_COMMAND_DELIMITER),
        COMMA(DocLint.TAGS_SEPARATOR),
        DOT("."),
        ELLIPSIS("..."),
        EQ("="),
        GT(">"),
        LT("<"),
        BANG("!"),
        TILDE("~"),
        QUES("?"),
        COLON(":"),
        EQEQ("=="),
        LTEQ("<="),
        GTEQ(">="),
        BANGEQ("!="),
        AMPAMP("&&"),
        BARBAR("||"),
        PLUSPLUS("++"),
        SUBSUB("--"),
        PLUS(Marker.ANY_NON_NULL_MARKER),
        SUB("-"),
        STAR(Marker.ANY_MARKER),
        SLASH(OnBotJavaFileSystemUtils.PATH_SEPARATOR),
        AMP("&"),
        BAR("|"),
        CARET("^"),
        PERCENT("%"),
        LTLT("<<"),
        GTGT(">>"),
        GTGTGT(">>>"),
        PLUSEQ("+="),
        SUBEQ("-="),
        STAREQ("*="),
        SLASHEQ("/="),
        AMPEQ("&="),
        BAREQ("|="),
        CARETEQ("^="),
        PERCENTEQ("%="),
        LTLTEQ("<<="),
        GTGTEQ(">>="),
        GTGTGTEQ(">>>="),
        MONKEYS_AT("@"),
        CUSTOM;

        public final String name;
        public final Token.Tag tag;

        TokenKind() {
            this(null, Token.Tag.DEFAULT);
        }

        TokenKind(String name) {
            this(name, Token.Tag.DEFAULT);
        }

        TokenKind(Token.Tag tag) {
            this(null, tag);
        }

        TokenKind(String name, Token.Tag tag) {
            this.name = name;
            this.tag = tag;
        }

        @Override // java.lang.Enum
        public String toString() {
            switch (this) {
                case IDENTIFIER:
                    return "token.identifier";
                case CHARLITERAL:
                    return "token.character";
                case STRINGLITERAL:
                    return "token.string";
                case INTLITERAL:
                    return "token.integer";
                case LONGLITERAL:
                    return "token.long-integer";
                case FLOATLITERAL:
                    return "token.float";
                case DOUBLELITERAL:
                    return "token.double";
                case ERROR:
                    return "token.bad-symbol";
                case EOF:
                    return "token.end-of-input";
                case DOT:
                case COMMA:
                case SEMI:
                case LPAREN:
                case RPAREN:
                case LBRACKET:
                case RBRACKET:
                case LBRACE:
                case RBRACE:
                    return "'" + this.name + "'";
                default:
                    return this.name;
            }
        }

        @Override // com.sun.tools.javac.api.Formattable
        public String getKind() {
            return "Token";
        }

        @Override // com.sun.tools.javac.api.Formattable
        public String toString(Locale locale, Messages messages) {
            return this.name != null ? toString() : messages.getLocalizedString(locale, "compiler.misc." + toString(), new Object[0]);
        }

        @Override // com.sun.tools.javac.util.Filter
        public boolean accepts(TokenKind that) {
            return this == that;
        }
    }

    public static class Token {
        public final List<Comment> comments;
        public final int endPos;
        public final TokenKind kind;
        public final int pos;

        enum Tag {
            DEFAULT,
            NAMED,
            STRING,
            NUMERIC
        }

        Token(TokenKind kind, int pos, int endPos, List<Comment> comments) {
            this.kind = kind;
            this.pos = pos;
            this.endPos = endPos;
            this.comments = comments;
            checkKind();
        }

        Token[] split(Tokens tokens) {
            if (this.kind.name.length() < 2 || this.kind.tag != Tag.DEFAULT) {
                throw new AssertionError("Cant split" + this.kind);
            }
            TokenKind t1 = tokens.lookupKind(this.kind.name.substring(0, 1));
            TokenKind t2 = tokens.lookupKind(this.kind.name.substring(1));
            if (t1 == null || t2 == null) {
                throw new AssertionError("Cant split - bad subtokens");
            }
            return new Token[]{new Token(t1, this.pos, this.pos + t1.name.length(), this.comments), new Token(t2, this.pos + t1.name.length(), this.endPos, null)};
        }

        protected void checkKind() {
            if (this.kind.tag != Tag.DEFAULT) {
                throw new AssertionError("Bad token kind - expected " + Tag.STRING);
            }
        }

        public Name name() {
            throw new UnsupportedOperationException();
        }

        public String stringVal() {
            throw new UnsupportedOperationException();
        }

        public int radix() {
            throw new UnsupportedOperationException();
        }

        public Comment comment(Comment.CommentStyle style) {
            List<Comment> comments = getComments(Comment.CommentStyle.JAVADOC);
            if (comments.isEmpty()) {
                return null;
            }
            return comments.head;
        }

        public boolean deprecatedFlag() {
            for (Comment c : getComments(Comment.CommentStyle.JAVADOC)) {
                if (c.isDeprecated()) {
                    return true;
                }
            }
            return false;
        }

        private List<Comment> getComments(Comment.CommentStyle style) {
            if (this.comments == null) {
                return List.nil();
            }
            ListBuffer<Comment> buf = new ListBuffer<>();
            for (Comment c : this.comments) {
                if (c.getStyle() == style) {
                    buf.add(c);
                }
            }
            return buf.toList();
        }
    }

    static final class NamedToken extends Token {
        public final Name name;

        public NamedToken(TokenKind kind, int pos, int endPos, Name name, List<Comment> comments) {
            super(kind, pos, endPos, comments);
            this.name = name;
        }

        @Override // com.sun.tools.javac.parser.Tokens.Token
        protected void checkKind() {
            if (this.kind.tag != Token.Tag.NAMED) {
                throw new AssertionError("Bad token kind - expected " + Token.Tag.NAMED);
            }
        }

        @Override // com.sun.tools.javac.parser.Tokens.Token
        public Name name() {
            return this.name;
        }
    }

    static class StringToken extends Token {
        public final String stringVal;

        public StringToken(TokenKind kind, int pos, int endPos, String stringVal, List<Comment> comments) {
            super(kind, pos, endPos, comments);
            this.stringVal = stringVal;
        }

        @Override // com.sun.tools.javac.parser.Tokens.Token
        protected void checkKind() {
            if (this.kind.tag != Token.Tag.STRING) {
                throw new AssertionError("Bad token kind - expected " + Token.Tag.STRING);
            }
        }

        @Override // com.sun.tools.javac.parser.Tokens.Token
        public String stringVal() {
            return this.stringVal;
        }
    }

    static final class NumericToken extends StringToken {
        public final int radix;

        public NumericToken(TokenKind kind, int pos, int endPos, String stringVal, int radix, List<Comment> comments) {
            super(kind, pos, endPos, stringVal, comments);
            this.radix = radix;
        }

        @Override // com.sun.tools.javac.parser.Tokens.StringToken, com.sun.tools.javac.parser.Tokens.Token
        protected void checkKind() {
            if (this.kind.tag != Token.Tag.NUMERIC) {
                throw new AssertionError("Bad token kind - expected " + Token.Tag.NUMERIC);
            }
        }

        @Override // com.sun.tools.javac.parser.Tokens.Token
        public int radix() {
            return this.radix;
        }
    }
}
