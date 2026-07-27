package com.sun.tools.javac.parser;

import com.qualcomm.hardware.lynx.LynxServoController;
import com.sun.source.tree.MemberReferenceTree;
import com.sun.tools.javac.code.BoundKind;
import com.sun.tools.javac.code.Flags;
import com.sun.tools.javac.code.Source;
import com.sun.tools.javac.code.TypeTag;
import com.sun.tools.javac.parser.Tokens;
import com.sun.tools.javac.tree.DocCommentTable;
import com.sun.tools.javac.tree.EndPosTable;
import com.sun.tools.javac.tree.JCTree;
import com.sun.tools.javac.tree.TreeInfo;
import com.sun.tools.javac.tree.TreeMaker;
import com.sun.tools.javac.util.Assert;
import com.sun.tools.javac.util.Convert;
import com.sun.tools.javac.util.Filter;
import com.sun.tools.javac.util.IntHashTable;
import com.sun.tools.javac.util.JCDiagnostic;
import com.sun.tools.javac.util.List;
import com.sun.tools.javac.util.ListBuffer;
import com.sun.tools.javac.util.Log;
import com.sun.tools.javac.util.Name;
import com.sun.tools.javac.util.Names;
import java.util.ArrayList;

/* JADX INFO: loaded from: classes.dex */
public class JavacParser implements Parser {
    static final /* synthetic */ boolean $assertionsDisabled = false;
    static final int DIAMOND = 16;
    static final int EXPR = 1;
    static final int NOPARAMS = 4;
    static final int TYPE = 2;
    static final int TYPEARG = 8;
    private static final int infixPrecedenceLevels = 10;
    protected TreeMaker F;
    protected Lexer S;
    boolean allowAnnotations;
    boolean allowAnnotationsAfterTypeParams;
    boolean allowAsserts;
    boolean allowDefaultMethods;
    boolean allowDiamond;
    boolean allowEnums;
    boolean allowForeach;
    boolean allowGenerics;
    boolean allowIntersectionTypesInCast;
    boolean allowLambda;
    boolean allowMethodReferences;
    boolean allowMulticatch;
    boolean allowStaticImport;
    boolean allowStaticInterfaceMethods;
    boolean allowStringFolding;
    boolean allowTWR;
    boolean allowThisIdent;
    boolean allowTypeAnnotations;
    boolean allowVarargs;
    private final DocCommentTable docComments;
    private final AbstractEndPosTable endPosTable;
    private JCTree.JCErroneous errorTree;
    boolean keepDocComments;
    boolean keepLineMap;
    private Log log;
    private Names names;
    JCTree.JCVariableDecl receiverParam;
    private Source source;
    protected Tokens.Token token;
    private List<JCTree.JCAnnotation> typeAnnotationsPushedBack = List.nil();
    private boolean permitTypeAnnotationsPushBack = false;
    private int mode = 0;
    private int lastmode = 0;
    private int errorPos = -1;
    ArrayList<JCTree.JCExpression[]> odStackSupply = new ArrayList<>();
    ArrayList<Tokens.Token[]> opStackSupply = new ArrayList<>();
    Filter<Tokens.TokenKind> LAX_IDENTIFIER = new Filter<Tokens.TokenKind>() { // from class: com.sun.tools.javac.parser.JavacParser.1
        @Override // com.sun.tools.javac.util.Filter
        public boolean accepts(Tokens.TokenKind t) {
            return t == Tokens.TokenKind.IDENTIFIER || t == Tokens.TokenKind.UNDERSCORE || t == Tokens.TokenKind.ASSERT || t == Tokens.TokenKind.ENUM;
        }
    };

    enum BasicErrorRecoveryAction implements ErrorRecoveryAction {
        BLOCK_STMT { // from class: com.sun.tools.javac.parser.JavacParser.BasicErrorRecoveryAction.1
            @Override // com.sun.tools.javac.parser.JavacParser.ErrorRecoveryAction
            public JCTree doRecover(JavacParser parser) {
                return parser.parseStatementAsBlock();
            }
        },
        CATCH_CLAUSE { // from class: com.sun.tools.javac.parser.JavacParser.BasicErrorRecoveryAction.2
            @Override // com.sun.tools.javac.parser.JavacParser.ErrorRecoveryAction
            public JCTree doRecover(JavacParser parser) {
                return parser.catchClause();
            }
        }
    }

    interface ErrorRecoveryAction {
        JCTree doRecover(JavacParser javacParser);
    }

    enum ParensResult {
        CAST,
        EXPLICIT_LAMBDA,
        IMPLICIT_LAMBDA,
        PARENS
    }

    protected JavacParser(ParserFactory fac, Lexer S, boolean keepDocComments, boolean keepLineMap, boolean keepEndPositions) {
        this.S = S;
        nextToken();
        this.F = fac.F;
        this.log = fac.log;
        this.names = fac.names;
        this.source = fac.source;
        this.allowGenerics = this.source.allowGenerics();
        this.allowVarargs = this.source.allowVarargs();
        this.allowAsserts = this.source.allowAsserts();
        this.allowEnums = this.source.allowEnums();
        this.allowForeach = this.source.allowForeach();
        this.allowStaticImport = this.source.allowStaticImport();
        this.allowAnnotations = this.source.allowAnnotations();
        this.allowTWR = this.source.allowTryWithResources();
        this.allowDiamond = this.source.allowDiamond();
        this.allowMulticatch = this.source.allowMulticatch();
        this.allowStringFolding = fac.options.getBoolean("allowStringFolding", true);
        this.allowLambda = this.source.allowLambda();
        this.allowMethodReferences = this.source.allowMethodReferences();
        this.allowDefaultMethods = this.source.allowDefaultMethods();
        this.allowStaticInterfaceMethods = this.source.allowStaticInterfaceMethods();
        this.allowIntersectionTypesInCast = this.source.allowIntersectionTypesInCast();
        this.allowTypeAnnotations = this.source.allowTypeAnnotations();
        this.allowAnnotationsAfterTypeParams = this.source.allowAnnotationsAfterTypeParams();
        this.keepDocComments = keepDocComments;
        this.docComments = newDocCommentTable(keepDocComments, fac);
        this.keepLineMap = keepLineMap;
        this.errorTree = this.F.Erroneous();
        this.endPosTable = newEndPosTable(keepEndPositions);
    }

    protected AbstractEndPosTable newEndPosTable(boolean keepEndPositions) {
        return keepEndPositions ? new SimpleEndPosTable(this) : new EmptyEndPosTable(this);
    }

    protected DocCommentTable newDocCommentTable(boolean keepDocComments, ParserFactory fac) {
        if (keepDocComments) {
            return new LazyDocCommentTable(fac);
        }
        return null;
    }

    public Tokens.Token token() {
        return this.token;
    }

    public void nextToken() {
        this.S.nextToken();
        this.token = this.S.token();
    }

    protected boolean peekToken(Filter<Tokens.TokenKind> tk) {
        return peekToken(0, tk);
    }

    protected boolean peekToken(int lookahead, Filter<Tokens.TokenKind> tk) {
        return tk.accepts(this.S.token(lookahead + 1).kind);
    }

    protected boolean peekToken(Filter<Tokens.TokenKind> tk1, Filter<Tokens.TokenKind> tk2) {
        return peekToken(0, tk1, tk2);
    }

    protected boolean peekToken(int lookahead, Filter<Tokens.TokenKind> tk1, Filter<Tokens.TokenKind> tk2) {
        return tk1.accepts(this.S.token(lookahead + 1).kind) && tk2.accepts(this.S.token(lookahead + 2).kind);
    }

    protected boolean peekToken(Filter<Tokens.TokenKind> tk1, Filter<Tokens.TokenKind> tk2, Filter<Tokens.TokenKind> tk3) {
        return peekToken(0, tk1, tk2, tk3);
    }

    protected boolean peekToken(int lookahead, Filter<Tokens.TokenKind> tk1, Filter<Tokens.TokenKind> tk2, Filter<Tokens.TokenKind> tk3) {
        return tk1.accepts(this.S.token(lookahead + 1).kind) && tk2.accepts(this.S.token(lookahead + 2).kind) && tk3.accepts(this.S.token(lookahead + 3).kind);
    }

    protected boolean peekToken(Filter<Tokens.TokenKind>... kinds) {
        return peekToken(0, kinds);
    }

    protected boolean peekToken(int lookahead, Filter<Tokens.TokenKind>... kinds) {
        while (lookahead < kinds.length) {
            if (kinds[lookahead].accepts(this.S.token(lookahead + 1).kind)) {
                lookahead++;
            } else {
                return false;
            }
        }
        return true;
    }

    private void skip(boolean stopAtImport, boolean stopAtMemberDecl, boolean stopAtIdentifier, boolean stopAtStatement) {
        while (true) {
            switch (this.token.kind) {
                case SEMI:
                    nextToken();
                    return;
                case PUBLIC:
                case FINAL:
                case ABSTRACT:
                case MONKEYS_AT:
                case EOF:
                case CLASS:
                case INTERFACE:
                case ENUM:
                    return;
                case IMPORT:
                    if (stopAtImport) {
                        return;
                    }
                    break;
                case LBRACE:
                case RBRACE:
                case PRIVATE:
                case PROTECTED:
                case STATIC:
                case TRANSIENT:
                case NATIVE:
                case VOLATILE:
                case SYNCHRONIZED:
                case STRICTFP:
                case LT:
                case BYTE:
                case SHORT:
                case CHAR:
                case INT:
                case LONG:
                case FLOAT:
                case DOUBLE:
                case BOOLEAN:
                case VOID:
                    if (stopAtMemberDecl) {
                        return;
                    }
                    break;
                case UNDERSCORE:
                case IDENTIFIER:
                    if (stopAtIdentifier) {
                        return;
                    }
                    break;
                case CASE:
                case DEFAULT:
                case IF:
                case FOR:
                case WHILE:
                case DO:
                case TRY:
                case SWITCH:
                case RETURN:
                case THROW:
                case BREAK:
                case CONTINUE:
                case ELSE:
                case FINALLY:
                case CATCH:
                    if (stopAtStatement) {
                        return;
                    }
                    break;
            }
            nextToken();
        }
    }

    private JCTree.JCErroneous syntaxError(int pos, String key, Tokens.TokenKind... args) {
        return syntaxError(pos, List.nil(), key, args);
    }

    private JCTree.JCErroneous syntaxError(int pos, List<JCTree> errs, String key, Tokens.TokenKind... args) {
        JCTree last;
        setErrorEndPos(pos);
        JCTree.JCErroneous err = this.F.at(pos).Erroneous(errs);
        reportSyntaxError(err, key, args);
        if (errs != null && (last = errs.last()) != null) {
            storeEnd(last, pos);
        }
        return (JCTree.JCErroneous) toP(err);
    }

    private void reportSyntaxError(int pos, String key, Object... args) {
        JCDiagnostic.DiagnosticPosition diag = new JCDiagnostic.SimpleDiagnosticPosition(pos);
        reportSyntaxError(diag, key, args);
    }

    private void reportSyntaxError(JCDiagnostic.DiagnosticPosition diagPos, String key, Object... args) {
        int pos = diagPos.getPreferredPosition();
        if (pos > this.S.errPos() || pos == -1) {
            if (this.token.kind == Tokens.TokenKind.EOF) {
                error(diagPos, "premature.eof", new Object[0]);
            } else {
                error(diagPos, key, args);
            }
        }
        this.S.errPos(pos);
        if (this.token.pos == this.errorPos) {
            nextToken();
        }
        this.errorPos = this.token.pos;
    }

    private JCTree.JCErroneous syntaxError(String key) {
        return syntaxError(this.token.pos, key, new Tokens.TokenKind[0]);
    }

    private JCTree.JCErroneous syntaxError(String key, Tokens.TokenKind arg) {
        return syntaxError(this.token.pos, key, arg);
    }

    public void accept(Tokens.TokenKind tk) {
        if (this.token.kind == tk) {
            nextToken();
        } else {
            setErrorEndPos(this.token.pos);
            reportSyntaxError(this.S.prevToken().endPos, "expected", tk);
        }
    }

    JCTree.JCExpression illegal(int pos) {
        setErrorEndPos(pos);
        if ((this.mode & 1) != 0) {
            return syntaxError(pos, "illegal.start.of.expr", new Tokens.TokenKind[0]);
        }
        return syntaxError(pos, "illegal.start.of.type", new Tokens.TokenKind[0]);
    }

    JCTree.JCExpression illegal() {
        return illegal(this.token.pos);
    }

    void checkNoMods(long mods) {
        if (mods != 0) {
            long lowestMod = (-mods) & mods;
            error(this.token.pos, "mod.not.allowed.here", Flags.asFlagSet(lowestMod));
        }
    }

    void attach(JCTree tree, Tokens.Comment dc) {
        if (this.keepDocComments && dc != null) {
            this.docComments.putComment(tree, dc);
        }
    }

    private void setErrorEndPos(int errPos) {
        this.endPosTable.setErrorEndPos(errPos);
    }

    private void storeEnd(JCTree tree, int endpos) {
        this.endPosTable.storeEnd(tree, endpos);
    }

    private <T extends JCTree> T to(T t) {
        return (T) this.endPosTable.to(t);
    }

    private <T extends JCTree> T toP(T t) {
        return (T) this.endPosTable.toP(t);
    }

    public int getStartPos(JCTree tree) {
        return TreeInfo.getStartPos(tree);
    }

    public int getEndPos(JCTree tree) {
        return this.endPosTable.getEndPos(tree);
    }

    public Name ident() {
        if (this.token.kind == Tokens.TokenKind.IDENTIFIER) {
            Name name = this.token.name();
            nextToken();
            return name;
        }
        if (this.token.kind == Tokens.TokenKind.ASSERT) {
            if (this.allowAsserts) {
                error(this.token.pos, "assert.as.identifier", new Object[0]);
                nextToken();
                return this.names.error;
            }
            warning(this.token.pos, "assert.as.identifier", new Object[0]);
            Name name2 = this.token.name();
            nextToken();
            return name2;
        }
        if (this.token.kind == Tokens.TokenKind.ENUM) {
            if (this.allowEnums) {
                error(this.token.pos, "enum.as.identifier", new Object[0]);
                nextToken();
                return this.names.error;
            }
            warning(this.token.pos, "enum.as.identifier", new Object[0]);
            Name name3 = this.token.name();
            nextToken();
            return name3;
        }
        if (this.token.kind == Tokens.TokenKind.THIS) {
            if (this.allowThisIdent) {
                checkTypeAnnotations();
                Name name4 = this.token.name();
                nextToken();
                return name4;
            }
            error(this.token.pos, "this.as.identifier", new Object[0]);
            nextToken();
            return this.names.error;
        }
        if (this.token.kind == Tokens.TokenKind.UNDERSCORE) {
            warning(this.token.pos, "underscore.as.identifier", new Object[0]);
            Name name5 = this.token.name();
            nextToken();
            return name5;
        }
        accept(Tokens.TokenKind.IDENTIFIER);
        return this.names.error;
    }

    public JCTree.JCExpression qualident(boolean allowAnnos) {
        JCTree.JCExpression t = (JCTree.JCExpression) toP(this.F.at(this.token.pos).Ident(ident()));
        while (this.token.kind == Tokens.TokenKind.DOT) {
            int pos = this.token.pos;
            nextToken();
            List<JCTree.JCAnnotation> tyannos = null;
            if (allowAnnos) {
                tyannos = typeAnnotationsOpt();
            }
            t = (JCTree.JCExpression) toP(this.F.at(pos).Select(t, ident()));
            if (tyannos != null && tyannos.nonEmpty()) {
                t = (JCTree.JCExpression) toP(this.F.at(tyannos.head.pos).AnnotatedType(tyannos, t));
            }
        }
        return t;
    }

    JCTree.JCExpression literal(Name prefix) {
        return literal(prefix, this.token.pos);
    }

    JCTree.JCExpression literal(Name prefix, int pos) {
        String proper;
        Float n;
        String proper2;
        Double n2;
        JCTree.JCExpression t = this.errorTree;
        switch (this.token.kind) {
            case INTLITERAL:
                try {
                    t = this.F.at(pos).Literal(TypeTag.INT, Integer.valueOf(Convert.string2int(strval(prefix), this.token.radix())));
                } catch (NumberFormatException e) {
                    error(this.token.pos, "int.number.too.large", strval(prefix));
                }
                break;
            case LONGLITERAL:
                try {
                    t = this.F.at(pos).Literal(TypeTag.LONG, new Long(Convert.string2long(strval(prefix), this.token.radix())));
                } catch (NumberFormatException e2) {
                    error(this.token.pos, "int.number.too.large", strval(prefix));
                }
                break;
            case FLOATLITERAL:
                if (this.token.radix() == 16) {
                    proper = "0x" + this.token.stringVal();
                } else {
                    proper = this.token.stringVal();
                }
                try {
                    n = Float.valueOf(proper);
                } catch (NumberFormatException e3) {
                    n = Float.valueOf(Float.NaN);
                }
                if (n.floatValue() == 0.0f && !isZero(proper)) {
                    error(this.token.pos, "fp.number.too.small", new Object[0]);
                } else if (n.floatValue() == Float.POSITIVE_INFINITY) {
                    error(this.token.pos, "fp.number.too.large", new Object[0]);
                } else {
                    t = this.F.at(pos).Literal(TypeTag.FLOAT, n);
                }
                break;
            case DOUBLELITERAL:
                if (this.token.radix() == 16) {
                    proper2 = "0x" + this.token.stringVal();
                } else {
                    proper2 = this.token.stringVal();
                }
                try {
                    n2 = Double.valueOf(proper2);
                } catch (NumberFormatException e4) {
                    n2 = Double.valueOf(Double.NaN);
                }
                if (n2.doubleValue() == LynxServoController.apiPositionFirst && !isZero(proper2)) {
                    error(this.token.pos, "fp.number.too.small", new Object[0]);
                } else if (n2.doubleValue() == Double.POSITIVE_INFINITY) {
                    error(this.token.pos, "fp.number.too.large", new Object[0]);
                } else {
                    t = this.F.at(pos).Literal(TypeTag.DOUBLE, n2);
                }
                break;
            case CHARLITERAL:
                t = this.F.at(pos).Literal(TypeTag.CHAR, Integer.valueOf(this.token.stringVal().charAt(0) + 0));
                break;
            case STRINGLITERAL:
                t = this.F.at(pos).Literal(TypeTag.CLASS, this.token.stringVal());
                break;
            case TRUE:
            case FALSE:
                t = this.F.at(pos).Literal(TypeTag.BOOLEAN, Integer.valueOf(this.token.kind == Tokens.TokenKind.TRUE ? 1 : 0));
                break;
            case NULL:
                t = this.F.at(pos).Literal(TypeTag.BOT, null);
                break;
            default:
                Assert.error();
                break;
        }
        if (t == this.errorTree) {
            t = this.F.at(pos).Erroneous();
        }
        storeEnd(t, this.token.endPos);
        nextToken();
        return t;
    }

    boolean isZero(String s) {
        char[] cs = s.toCharArray();
        int base = (cs.length <= 1 || Character.toLowerCase(cs[1]) != 'x') ? 10 : 16;
        int i = base == 16 ? 2 : 0;
        while (i < cs.length && (cs[i] == '0' || cs[i] == '.')) {
            i++;
        }
        return i >= cs.length || Character.digit(cs[i], base) <= 0;
    }

    String strval(Name prefix) {
        String s = this.token.stringVal();
        return prefix.isEmpty() ? s : ((Object) prefix) + s;
    }

    @Override // com.sun.tools.javac.parser.Parser
    public JCTree.JCExpression parseExpression() {
        return term(1);
    }

    @Override // com.sun.tools.javac.parser.Parser
    public JCTree.JCExpression parseType() {
        List<JCTree.JCAnnotation> annotations = typeAnnotationsOpt();
        return parseType(annotations);
    }

    public JCTree.JCExpression parseType(List<JCTree.JCAnnotation> annotations) {
        JCTree.JCExpression result = unannotatedType();
        if (annotations.nonEmpty()) {
            return insertAnnotationsToMostInner(result, annotations, false);
        }
        return result;
    }

    public JCTree.JCExpression unannotatedType() {
        return term(2);
    }

    JCTree.JCExpression term(int newmode) {
        int prevmode = this.mode;
        this.mode = newmode;
        JCTree.JCExpression t = term();
        this.lastmode = this.mode;
        this.mode = prevmode;
        return t;
    }

    JCTree.JCExpression term() {
        JCTree.JCExpression t = term1();
        if (((this.mode & 1) != 0 && this.token.kind == Tokens.TokenKind.EQ) || (Tokens.TokenKind.PLUSEQ.compareTo(this.token.kind) <= 0 && this.token.kind.compareTo(Tokens.TokenKind.GTGTGTEQ) <= 0)) {
            return termRest(t);
        }
        return t;
    }

    JCTree.JCExpression termRest(JCTree.JCExpression t) {
        switch (this.token.kind) {
            case EQ:
                int pos = this.token.pos;
                nextToken();
                this.mode = 1;
                JCTree.JCExpression t1 = term();
                return (JCTree.JCExpression) toP(this.F.at(pos).Assign(t, t1));
            case PLUSEQ:
            case SUBEQ:
            case STAREQ:
            case SLASHEQ:
            case PERCENTEQ:
            case AMPEQ:
            case BAREQ:
            case CARETEQ:
            case LTLTEQ:
            case GTGTEQ:
            case GTGTGTEQ:
                int pos2 = this.token.pos;
                Tokens.TokenKind tk = this.token.kind;
                nextToken();
                this.mode = 1;
                JCTree.JCExpression t12 = term();
                return this.F.at(pos2).Assignop(optag(tk), t, t12);
            default:
                return t;
        }
    }

    JCTree.JCExpression term1() {
        JCTree.JCExpression t = term2();
        if ((this.mode & 1) != 0 && this.token.kind == Tokens.TokenKind.QUES) {
            this.mode = 1;
            return term1Rest(t);
        }
        return t;
    }

    JCTree.JCExpression term1Rest(JCTree.JCExpression t) {
        if (this.token.kind == Tokens.TokenKind.QUES) {
            int pos = this.token.pos;
            nextToken();
            JCTree.JCExpression t1 = term();
            accept(Tokens.TokenKind.COLON);
            JCTree.JCExpression t2 = term1();
            return this.F.at(pos).Conditional(t, t1, t2);
        }
        return t;
    }

    JCTree.JCExpression term2() {
        JCTree.JCExpression t = term3();
        if ((this.mode & 1) != 0 && prec(this.token.kind) >= 4) {
            this.mode = 1;
            return term2Rest(t, 4);
        }
        return t;
    }

    JCTree.JCExpression term2Rest(JCTree.JCExpression t, int minprec) {
        JCTree.JCExpression[] odStack = newOdStack();
        Tokens.Token[] opStack = newOpStack();
        int top = 0;
        odStack[0] = t;
        int i = this.token.pos;
        Tokens.Token topOp = Tokens.DUMMY;
        while (prec(this.token.kind) >= minprec) {
            opStack[top] = topOp;
            top++;
            topOp = this.token;
            nextToken();
            odStack[top] = topOp.kind == Tokens.TokenKind.INSTANCEOF ? parseType() : term3();
            while (top > 0 && prec(topOp.kind) >= prec(this.token.kind)) {
                odStack[top - 1] = makeOp(topOp.pos, topOp.kind, odStack[top - 1], odStack[top]);
                top--;
                topOp = opStack[top];
            }
        }
        Assert.check(top == 0);
        JCTree.JCExpression t2 = odStack[0];
        if (t2.hasTag(JCTree.Tag.PLUS)) {
            t2 = foldStrings(t2);
        }
        this.odStackSupply.add(odStack);
        this.opStackSupply.add(opStack);
        return t2;
    }

    private JCTree.JCExpression makeOp(int pos, Tokens.TokenKind topOp, JCTree.JCExpression od1, JCTree.JCExpression od2) {
        if (topOp == Tokens.TokenKind.INSTANCEOF) {
            return this.F.at(pos).TypeTest(od1, od2);
        }
        return this.F.at(pos).Binary(optag(topOp), od1, od2);
    }

    protected JCTree.JCExpression foldStrings(JCTree.JCExpression tree) {
        if (!this.allowStringFolding) {
            return tree;
        }
        ListBuffer<JCTree.JCExpression> opStack = new ListBuffer<>();
        ListBuffer<JCTree.JCLiteral> litBuf = new ListBuffer<>();
        boolean needsFolding = false;
        JCTree.JCExpression curr = tree;
        while (curr.hasTag(JCTree.Tag.PLUS)) {
            JCTree.JCBinary op = (JCTree.JCBinary) curr;
            needsFolding |= foldIfNeeded(op.rhs, litBuf, opStack, false);
            curr = op.lhs;
        }
        if (needsFolding | foldIfNeeded(curr, litBuf, opStack, true)) {
            List<JCTree.JCExpression> ops = opStack.toList();
            JCTree.JCExpression res = ops.head;
            for (JCTree.JCExpression op2 : ops.tail) {
                res = this.F.at(op2.getStartPosition()).Binary(optag(Tokens.TokenKind.PLUS), res, op2);
                storeEnd(res, getEndPos(op2));
            }
            return res;
        }
        return tree;
    }

    private boolean foldIfNeeded(JCTree.JCExpression tree, ListBuffer<JCTree.JCLiteral> litBuf, ListBuffer<JCTree.JCExpression> opStack, boolean last) {
        JCTree.JCLiteral str = stringLiteral(tree);
        if (str != null) {
            litBuf.prepend(str);
            return last && merge(litBuf, opStack);
        }
        boolean res = merge(litBuf, opStack);
        litBuf.clear();
        opStack.prepend(tree);
        return res;
    }

    boolean merge(ListBuffer<JCTree.JCLiteral> litBuf, ListBuffer<JCTree.JCExpression> opStack) {
        if (litBuf.isEmpty()) {
            return false;
        }
        if (litBuf.size() == 1) {
            opStack.prepend(litBuf.first());
            return false;
        }
        StringBuilder sb = new StringBuilder();
        for (JCTree.JCLiteral lit : litBuf) {
            sb.append(lit.getValue());
        }
        JCTree.JCExpression t = this.F.at(litBuf.first().getStartPosition()).Literal(TypeTag.CLASS, sb.toString());
        storeEnd(t, litBuf.last().getEndPosition(this.endPosTable));
        opStack.prepend(t);
        return true;
    }

    private JCTree.JCLiteral stringLiteral(JCTree tree) {
        if (tree.hasTag(JCTree.Tag.LITERAL)) {
            JCTree.JCLiteral lit = (JCTree.JCLiteral) tree;
            if (lit.typetag == TypeTag.CLASS) {
                return lit;
            }
            return null;
        }
        return null;
    }

    private JCTree.JCExpression[] newOdStack() {
        if (this.odStackSupply.isEmpty()) {
            return new JCTree.JCExpression[11];
        }
        return this.odStackSupply.remove(this.odStackSupply.size() - 1);
    }

    private Tokens.Token[] newOpStack() {
        if (this.opStackSupply.isEmpty()) {
            return new Tokens.Token[11];
        }
        return this.opStackSupply.remove(this.opStackSupply.size() - 1);
    }

    /* JADX WARN: Code restructure failed: missing block: B:199:0x04e3, code lost:
    
        r1 = r3;
     */
    /*
        Code decompiled incorrectly, please refer to instructions dump.
        To view partially-correct code enable 'Show inconsistent code' option in preferences
    */
    protected com.sun.tools.javac.tree.JCTree.JCExpression term3() {
        /*
            Method dump skipped, instruction units count: 1576
            To view this dump change 'Code comments level' option to 'DEBUG'
        */
        throw new UnsupportedOperationException("Method not decompiled: com.sun.tools.javac.parser.JavacParser.term3():com.sun.tools.javac.tree.JCTree$JCExpression");
    }

    /* JADX WARN: Code restructure failed: missing block: B:62:0x016d, code lost:
    
        if (r1.isEmpty() != false) goto L111;
     */
    /* JADX WARN: Code restructure failed: missing block: B:64:0x0171, code lost:
    
        if (r6.permitTypeAnnotationsPushBack == false) goto L66;
     */
    /* JADX WARN: Code restructure failed: missing block: B:65:0x0173, code lost:
    
        r6.typeAnnotationsPushedBack = r1;
     */
    /* JADX WARN: Code restructure failed: missing block: B:67:0x0180, code lost:
    
        return illegal(r1.head.pos);
     */
    /* JADX WARN: Code restructure failed: missing block: B:69:0x0187, code lost:
    
        if (r6.token.kind == com.sun.tools.javac.parser.Tokens.TokenKind.PLUSPLUS) goto L72;
     */
    /* JADX WARN: Code restructure failed: missing block: B:71:0x018f, code lost:
    
        if (r6.token.kind != com.sun.tools.javac.parser.Tokens.TokenKind.SUBSUB) goto L107;
     */
    /* JADX WARN: Code restructure failed: missing block: B:73:0x0194, code lost:
    
        if ((r6.mode & 1) == 0) goto L108;
     */
    /* JADX WARN: Code restructure failed: missing block: B:74:0x0196, code lost:
    
        r6.mode = 1;
        r0 = r6.F.at(r6.token.pos);
     */
    /* JADX WARN: Code restructure failed: missing block: B:75:0x01a8, code lost:
    
        if (r6.token.kind != com.sun.tools.javac.parser.Tokens.TokenKind.PLUSPLUS) goto L77;
     */
    /* JADX WARN: Code restructure failed: missing block: B:76:0x01aa, code lost:
    
        r1 = com.sun.tools.javac.tree.JCTree.Tag.POSTINC;
     */
    /* JADX WARN: Code restructure failed: missing block: B:77:0x01ad, code lost:
    
        r1 = com.sun.tools.javac.tree.JCTree.Tag.POSTDEC;
     */
    /* JADX WARN: Code restructure failed: missing block: B:78:0x01af, code lost:
    
        r7 = (com.sun.tools.javac.tree.JCTree.JCExpression) to(r0.Unary(r1, r7));
        nextToken();
     */
    /* JADX WARN: Code restructure failed: missing block: B:80:0x01c4, code lost:
    
        return (com.sun.tools.javac.tree.JCTree.JCExpression) toP(r7);
     */
    /*
        Code decompiled incorrectly, please refer to instructions dump.
        To view partially-correct code enable 'Show inconsistent code' option in preferences
    */
    com.sun.tools.javac.tree.JCTree.JCExpression term3Rest(com.sun.tools.javac.tree.JCTree.JCExpression r7, com.sun.tools.javac.util.List<com.sun.tools.javac.tree.JCTree.JCExpression> r8) {
        /*
            Method dump skipped, instruction units count: 453
            To view this dump change 'Code comments level' option to 'DEBUG'
        */
        throw new UnsupportedOperationException("Method not decompiled: com.sun.tools.javac.parser.JavacParser.term3Rest(com.sun.tools.javac.tree.JCTree$JCExpression, com.sun.tools.javac.util.List):com.sun.tools.javac.tree.JCTree$JCExpression");
    }

    boolean isUnboundMemberRef() {
        int pos = 0;
        int depth = 0;
        Tokens.Token t = this.S.token(0);
        while (true) {
            switch (t.kind) {
                case MONKEYS_AT:
                case BYTE:
                case SHORT:
                case CHAR:
                case INT:
                case LONG:
                case FLOAT:
                case DOUBLE:
                case BOOLEAN:
                case UNDERSCORE:
                case IDENTIFIER:
                case SUPER:
                case LBRACKET:
                case DOT:
                case QUES:
                case EXTENDS:
                case RBRACKET:
                case COMMA:
                    pos++;
                    t = this.S.token(pos);
                    break;
                case LT:
                    depth++;
                    pos++;
                    t = this.S.token(pos);
                    break;
                case LPAREN:
                    int nesting = 0;
                    while (true) {
                        Tokens.TokenKind tk2 = this.S.token(pos).kind;
                        switch (tk2) {
                            case EOF:
                                return false;
                            case LPAREN:
                                nesting++;
                                pos++;
                                break;
                            case RPAREN:
                                nesting--;
                                if (nesting != 0) {
                                    pos++;
                                }
                                break;
                            default:
                                pos++;
                                break;
                        }
                    }
                    pos++;
                    t = this.S.token(pos);
                    break;
                case GTGTGT:
                    depth--;
                case GTGT:
                    depth--;
                case GT:
                    depth--;
                    if (depth != 0) {
                        pos++;
                        t = this.S.token(pos);
                    } else {
                        Tokens.TokenKind nextKind = this.S.token(pos + 1).kind;
                        return nextKind == Tokens.TokenKind.DOT || nextKind == Tokens.TokenKind.LBRACKET || nextKind == Tokens.TokenKind.COLCOL;
                    }
                    break;
                default:
                    return false;
            }
        }
    }

    /* JADX WARN: Code restructure failed: missing block: B:25:0x0056, code lost:
    
        return com.sun.tools.javac.parser.JavacParser.ParensResult.EXPLICIT_LAMBDA;
     */
    /* JADX WARN: Code restructure failed: missing block: B:27:0x0059, code lost:
    
        return com.sun.tools.javac.parser.JavacParser.ParensResult.CAST;
     */
    /* JADX WARN: Code restructure failed: missing block: B:68:0x00d9, code lost:
    
        return com.sun.tools.javac.parser.JavacParser.ParensResult.CAST;
     */
    /*
        Code decompiled incorrectly, please refer to instructions dump.
        To view partially-correct code enable 'Show inconsistent code' option in preferences
    */
    com.sun.tools.javac.parser.JavacParser.ParensResult analyzeParens() {
        /*
            Method dump skipped, instruction units count: 580
            To view this dump change 'Code comments level' option to 'DEBUG'
        */
        throw new UnsupportedOperationException("Method not decompiled: com.sun.tools.javac.parser.JavacParser.analyzeParens():com.sun.tools.javac.parser.JavacParser$ParensResult");
    }

    JCTree.JCExpression lambdaExpressionOrStatement(boolean hasParens, boolean explicitParams, int pos) {
        List<JCTree.JCVariableDecl> params;
        if (explicitParams) {
            params = formalParameters(true);
        } else {
            params = implicitParameters(hasParens);
        }
        return lambdaExpressionOrStatementRest(params, pos);
    }

    JCTree.JCExpression lambdaExpressionOrStatementRest(List<JCTree.JCVariableDecl> args, int pos) {
        checkLambda();
        accept(Tokens.TokenKind.ARROW);
        if (this.token.kind == Tokens.TokenKind.LBRACE) {
            return lambdaStatement(args, pos, pos);
        }
        return lambdaExpression(args, pos);
    }

    JCTree.JCExpression lambdaStatement(List<JCTree.JCVariableDecl> args, int pos, int pos2) {
        JCTree.JCBlock block = block(pos2, 0L);
        return (JCTree.JCExpression) toP(this.F.at(pos).Lambda(args, block));
    }

    JCTree.JCExpression lambdaExpression(List<JCTree.JCVariableDecl> args, int pos) {
        JCTree expr = parseExpression();
        return (JCTree.JCExpression) toP(this.F.at(pos).Lambda(args, expr));
    }

    JCTree.JCExpression superSuffix(List<JCTree.JCExpression> typeArgs, JCTree.JCExpression t) {
        nextToken();
        if (this.token.kind == Tokens.TokenKind.LPAREN || typeArgs != null) {
            return arguments(typeArgs, t);
        }
        if (this.token.kind == Tokens.TokenKind.COLCOL) {
            return typeArgs != null ? illegal() : memberReferenceSuffix(t);
        }
        int pos = this.token.pos;
        accept(Tokens.TokenKind.DOT);
        return argumentsOpt(this.token.kind == Tokens.TokenKind.LT ? typeArguments(false) : null, (JCTree.JCExpression) toP(this.F.at(pos).Select(t, ident())));
    }

    JCTree.JCPrimitiveTypeTree basicType() {
        JCTree.JCPrimitiveTypeTree t = (JCTree.JCPrimitiveTypeTree) to(this.F.at(this.token.pos).TypeIdent(typetag(this.token.kind)));
        nextToken();
        return t;
    }

    JCTree.JCExpression argumentsOpt(List<JCTree.JCExpression> typeArgs, JCTree.JCExpression t) {
        if (((this.mode & 1) != 0 && this.token.kind == Tokens.TokenKind.LPAREN) || typeArgs != null) {
            this.mode = 1;
            return arguments(typeArgs, t);
        }
        return t;
    }

    List<JCTree.JCExpression> arguments() {
        ListBuffer<JCTree.JCExpression> args = new ListBuffer<>();
        if (this.token.kind == Tokens.TokenKind.LPAREN) {
            nextToken();
            if (this.token.kind != Tokens.TokenKind.RPAREN) {
                args.append(parseExpression());
                while (this.token.kind == Tokens.TokenKind.COMMA) {
                    nextToken();
                    args.append(parseExpression());
                }
            }
            accept(Tokens.TokenKind.RPAREN);
        } else {
            syntaxError(this.token.pos, "expected", Tokens.TokenKind.LPAREN);
        }
        return args.toList();
    }

    JCTree.JCMethodInvocation arguments(List<JCTree.JCExpression> typeArgs, JCTree.JCExpression t) {
        int pos = this.token.pos;
        List<JCTree.JCExpression> args = arguments();
        return (JCTree.JCMethodInvocation) toP(this.F.at(pos).Apply(typeArgs, t, args));
    }

    JCTree.JCExpression typeArgumentsOpt(JCTree.JCExpression t) {
        if (this.token.kind == Tokens.TokenKind.LT && (this.mode & 2) != 0 && (this.mode & 4) == 0) {
            this.mode = 2;
            checkGenerics();
            return typeArguments(t, false);
        }
        return t;
    }

    List<JCTree.JCExpression> typeArgumentsOpt() {
        return typeArgumentsOpt(2);
    }

    List<JCTree.JCExpression> typeArgumentsOpt(int useMode) {
        if (this.token.kind == Tokens.TokenKind.LT) {
            checkGenerics();
            if ((this.mode & useMode) == 0 || (this.mode & 4) != 0) {
                illegal();
            }
            this.mode = useMode;
            return typeArguments(false);
        }
        return null;
    }

    List<JCTree.JCExpression> typeArguments(boolean diamondAllowed) {
        if (this.token.kind != Tokens.TokenKind.LT) {
            return List.of(syntaxError(this.token.pos, "expected", Tokens.TokenKind.LT));
        }
        nextToken();
        if (this.token.kind == Tokens.TokenKind.GT && diamondAllowed) {
            checkDiamond();
            this.mode |= 16;
            nextToken();
            return List.nil();
        }
        ListBuffer<JCTree.JCExpression> args = new ListBuffer<>();
        args.append((this.mode & 1) == 0 ? typeArgument() : parseType());
        while (this.token.kind == Tokens.TokenKind.COMMA) {
            nextToken();
            args.append((this.mode & 1) == 0 ? typeArgument() : parseType());
        }
        switch (this.token.kind) {
            case GTGTEQ:
            case GTGTGTEQ:
            case GTGTGT:
            case GTGT:
            case GTEQ:
                this.token = this.S.split();
                break;
            case GT:
                nextToken();
                break;
            default:
                args.append(syntaxError(this.token.pos, "expected", Tokens.TokenKind.GT));
                break;
        }
        return args.toList();
    }

    JCTree.JCExpression typeArgument() {
        JCTree.JCExpression result;
        List<JCTree.JCAnnotation> annotations = typeAnnotationsOpt();
        if (this.token.kind != Tokens.TokenKind.QUES) {
            return parseType(annotations);
        }
        int pos = this.token.pos;
        nextToken();
        if (this.token.kind == Tokens.TokenKind.EXTENDS) {
            JCTree.TypeBoundKind t = (JCTree.TypeBoundKind) to(this.F.at(pos).TypeBoundKind(BoundKind.EXTENDS));
            nextToken();
            JCTree.JCExpression bound = parseType();
            result = this.F.at(pos).Wildcard(t, bound);
        } else if (this.token.kind == Tokens.TokenKind.SUPER) {
            JCTree.TypeBoundKind t2 = (JCTree.TypeBoundKind) to(this.F.at(pos).TypeBoundKind(BoundKind.SUPER));
            nextToken();
            JCTree.JCExpression bound2 = parseType();
            result = this.F.at(pos).Wildcard(t2, bound2);
        } else if (this.LAX_IDENTIFIER.accepts(this.token.kind)) {
            JCTree.TypeBoundKind t3 = this.F.at(-1).TypeBoundKind(BoundKind.UNBOUND);
            JCTree.JCExpression wc = (JCTree.JCExpression) toP(this.F.at(pos).Wildcard(t3, null));
            JCTree.JCIdent id = (JCTree.JCIdent) toP(this.F.at(this.token.pos).Ident(ident()));
            JCTree.JCErroneous err = this.F.at(pos).Erroneous(List.of((JCTree.JCIdent) wc, id));
            reportSyntaxError(err, "expected3", Tokens.TokenKind.GT, Tokens.TokenKind.EXTENDS, Tokens.TokenKind.SUPER);
            result = err;
        } else {
            JCTree.TypeBoundKind t4 = (JCTree.TypeBoundKind) toP(this.F.at(pos).TypeBoundKind(BoundKind.UNBOUND));
            result = (JCTree.JCExpression) toP(this.F.at(pos).Wildcard(t4, null));
        }
        if (!annotations.isEmpty()) {
            return (JCTree.JCExpression) toP(this.F.at(annotations.head.pos).AnnotatedType(annotations, result));
        }
        return result;
    }

    JCTree.JCTypeApply typeArguments(JCTree.JCExpression t, boolean diamondAllowed) {
        int pos = this.token.pos;
        List<JCTree.JCExpression> args = typeArguments(diamondAllowed);
        return (JCTree.JCTypeApply) toP(this.F.at(pos).TypeApply(t, args));
    }

    private JCTree.JCExpression bracketsOpt(JCTree.JCExpression t, List<JCTree.JCAnnotation> annotations) {
        List<JCTree.JCAnnotation> nextLevelAnnotations = typeAnnotationsOpt();
        if (this.token.kind == Tokens.TokenKind.LBRACKET) {
            int pos = this.token.pos;
            nextToken();
            t = bracketsOptCont(t, pos, nextLevelAnnotations);
        } else if (!nextLevelAnnotations.isEmpty()) {
            if (this.permitTypeAnnotationsPushBack) {
                this.typeAnnotationsPushedBack = nextLevelAnnotations;
            } else {
                return illegal(nextLevelAnnotations.head.pos);
            }
        }
        if (!annotations.isEmpty()) {
            return (JCTree.JCExpression) toP(this.F.at(this.token.pos).AnnotatedType(annotations, t));
        }
        return t;
    }

    private JCTree.JCExpression bracketsOpt(JCTree.JCExpression t) {
        return bracketsOpt(t, List.nil());
    }

    private JCTree.JCExpression bracketsOptCont(JCTree.JCExpression t, int pos, List<JCTree.JCAnnotation> annotations) {
        accept(Tokens.TokenKind.RBRACKET);
        JCTree.JCExpression t2 = (JCTree.JCExpression) toP(this.F.at(pos).TypeArray(bracketsOpt(t)));
        if (annotations.nonEmpty()) {
            return (JCTree.JCExpression) toP(this.F.at(pos).AnnotatedType(annotations, t2));
        }
        return t2;
    }

    JCTree.JCExpression bracketsSuffix(JCTree.JCExpression t) {
        Name name;
        if ((this.mode & 1) == 0 || this.token.kind != Tokens.TokenKind.DOT) {
            if ((this.mode & 2) != 0) {
                if (this.token.kind != Tokens.TokenKind.COLCOL) {
                    this.mode = 2;
                    return t;
                }
                return t;
            }
            if (this.token.kind != Tokens.TokenKind.COLCOL) {
                syntaxError(this.token.pos, "dot.class.expected", new Tokens.TokenKind[0]);
                return t;
            }
            return t;
        }
        this.mode = 1;
        int pos = this.token.pos;
        nextToken();
        accept(Tokens.TokenKind.CLASS);
        if (this.token.pos == this.endPosTable.errorEndPos) {
            if (this.LAX_IDENTIFIER.accepts(this.token.kind)) {
                name = this.token.name();
                nextToken();
            } else {
                name = this.names.error;
            }
            return this.F.at(pos).Erroneous(List.of(toP(this.F.at(pos).Select(t, name))));
        }
        return (JCTree.JCExpression) toP(this.F.at(pos).Select(t, this.names._class));
    }

    JCTree.JCExpression memberReferenceSuffix(JCTree.JCExpression t) {
        int pos1 = this.token.pos;
        accept(Tokens.TokenKind.COLCOL);
        return memberReferenceSuffix(pos1, t);
    }

    JCTree.JCExpression memberReferenceSuffix(int pos1, JCTree.JCExpression t) {
        MemberReferenceTree.ReferenceMode refMode;
        Name refName;
        checkMethodReferences();
        this.mode = 1;
        List<JCTree.JCExpression> typeArgs = null;
        if (this.token.kind == Tokens.TokenKind.LT) {
            typeArgs = typeArguments(false);
        }
        if (this.token.kind == Tokens.TokenKind.NEW) {
            refMode = MemberReferenceTree.ReferenceMode.NEW;
            refName = this.names.init;
            nextToken();
        } else {
            refMode = MemberReferenceTree.ReferenceMode.INVOKE;
            refName = ident();
        }
        return (JCTree.JCExpression) toP(this.F.at(t.getStartPosition()).Reference(refMode, refName, t, typeArgs));
    }

    JCTree.JCExpression creator(int newpos, List<JCTree.JCExpression> typeArgs) {
        JCTree.JCExpression t;
        boolean diamondFound;
        int lastTypeargsPos;
        List<JCTree.JCAnnotation> newAnnotations = typeAnnotationsOpt();
        switch (this.token.kind) {
            case BYTE:
            case SHORT:
            case CHAR:
            case INT:
            case LONG:
            case FLOAT:
            case DOUBLE:
            case BOOLEAN:
                if (typeArgs == null) {
                    if (newAnnotations.isEmpty()) {
                        return arrayCreatorRest(newpos, basicType());
                    }
                    return arrayCreatorRest(newpos, (JCTree.JCExpression) toP(this.F.at(newAnnotations.head.pos).AnnotatedType(newAnnotations, basicType())));
                }
                break;
        }
        JCTree.JCExpression t2 = qualident(true);
        int oldmode = this.mode;
        this.mode = 2;
        if (this.token.kind != Tokens.TokenKind.LT) {
            t = t2;
            diamondFound = false;
            lastTypeargsPos = -1;
        } else {
            checkGenerics();
            int lastTypeargsPos2 = this.token.pos;
            JCTree.JCExpression t3 = typeArguments(t2, true);
            boolean diamondFound2 = (this.mode & 16) != 0;
            t = t3;
            diamondFound = diamondFound2;
            lastTypeargsPos = lastTypeargsPos2;
        }
        while (this.token.kind == Tokens.TokenKind.DOT) {
            if (diamondFound) {
                illegal();
            }
            int pos = this.token.pos;
            nextToken();
            List<JCTree.JCAnnotation> tyannos = typeAnnotationsOpt();
            JCTree.JCExpression t4 = (JCTree.JCExpression) toP(this.F.at(pos).Select(t, ident()));
            if (tyannos != null && tyannos.nonEmpty()) {
                t4 = (JCTree.JCExpression) toP(this.F.at(tyannos.head.pos).AnnotatedType(tyannos, t4));
            }
            if (this.token.kind != Tokens.TokenKind.LT) {
                t = t4;
            } else {
                int lastTypeargsPos3 = this.token.pos;
                checkGenerics();
                t = typeArguments(t4, true);
                lastTypeargsPos = lastTypeargsPos3;
                diamondFound = (this.mode & 16) != 0;
            }
        }
        this.mode = oldmode;
        if (this.token.kind == Tokens.TokenKind.LBRACKET || this.token.kind == Tokens.TokenKind.MONKEYS_AT) {
            if (newAnnotations.nonEmpty()) {
                t = insertAnnotationsToMostInner(t, newAnnotations, false);
            }
            JCTree.JCExpression e = arrayCreatorRest(newpos, t);
            if (diamondFound) {
                reportSyntaxError(lastTypeargsPos, "cannot.create.array.with.diamond", new Object[0]);
                return (JCTree.JCExpression) toP(this.F.at(newpos).Erroneous(List.of(e)));
            }
            if (typeArgs != null) {
                int pos2 = newpos;
                if (!typeArgs.isEmpty() && typeArgs.head.pos != -1) {
                    pos2 = typeArgs.head.pos;
                }
                setErrorEndPos(this.S.prevToken().endPos);
                JCTree.JCErroneous err = this.F.at(pos2).Erroneous(typeArgs.prepend(e));
                reportSyntaxError(err, "cannot.create.array.with.type.arguments", new Object[0]);
                return (JCTree.JCExpression) toP(err);
            }
            return e;
        }
        if (this.token.kind == Tokens.TokenKind.LPAREN) {
            JCTree.JCNewClass newClass = classCreatorRest(newpos, null, typeArgs, t);
            if (newClass.def != null) {
                if (!newClass.def.mods.annotations.isEmpty()) {
                    throw new AssertionError();
                }
                if (newAnnotations.nonEmpty()) {
                    newClass.def.mods.pos = earlier(newClass.def.mods.pos, newAnnotations.head.pos);
                    newClass.def.mods.annotations = newAnnotations;
                }
            } else if (newAnnotations.nonEmpty()) {
                newClass.clazz = insertAnnotationsToMostInner(t, newAnnotations, false);
            }
            return newClass;
        }
        setErrorEndPos(this.token.pos);
        reportSyntaxError(this.token.pos, "expected2", Tokens.TokenKind.LPAREN, Tokens.TokenKind.LBRACKET);
        return (JCTree.JCExpression) toP(this.F.at(newpos).Erroneous(List.of((JCTree.JCExpression) toP(this.F.at(newpos).NewClass(null, typeArgs, t, List.nil(), null)))));
    }

    JCTree.JCExpression innerCreator(int newpos, List<JCTree.JCExpression> typeArgs, JCTree.JCExpression encl) {
        List<JCTree.JCAnnotation> newAnnotations = typeAnnotationsOpt();
        JCTree.JCExpression t = (JCTree.JCExpression) toP(this.F.at(this.token.pos).Ident(ident()));
        if (newAnnotations.nonEmpty()) {
            t = (JCTree.JCExpression) toP(this.F.at(newAnnotations.head.pos).AnnotatedType(newAnnotations, t));
        }
        if (this.token.kind == Tokens.TokenKind.LT) {
            int oldmode = this.mode;
            checkGenerics();
            t = typeArguments(t, true);
            this.mode = oldmode;
        }
        return classCreatorRest(newpos, encl, typeArgs, t);
    }

    JCTree.JCExpression arrayCreatorRest(int newpos, JCTree.JCExpression elemtype) {
        List<JCTree.JCAnnotation> annos = typeAnnotationsOpt();
        accept(Tokens.TokenKind.LBRACKET);
        if (this.token.kind == Tokens.TokenKind.RBRACKET) {
            accept(Tokens.TokenKind.RBRACKET);
            JCTree.JCExpression elemtype2 = bracketsOpt(elemtype, annos);
            if (this.token.kind == Tokens.TokenKind.LBRACE) {
                JCTree.JCNewArray na = (JCTree.JCNewArray) arrayInitializer(newpos, elemtype2);
                if (annos.nonEmpty()) {
                    JCTree.JCAnnotatedType annotated = (JCTree.JCAnnotatedType) elemtype2;
                    if (annotated.annotations != annos) {
                        throw new AssertionError();
                    }
                    na.annotations = annotated.annotations;
                    na.elemtype = annotated.underlyingType;
                }
                return na;
            }
            JCTree.JCExpression t = (JCTree.JCExpression) toP(this.F.at(newpos).NewArray(elemtype2, List.nil(), null));
            return syntaxError(this.token.pos, List.of(t), "array.dimension.missing", new Tokens.TokenKind[0]);
        }
        ListBuffer<JCTree.JCExpression> dims = new ListBuffer<>();
        ListBuffer<List<JCTree.JCAnnotation>> dimAnnotations = new ListBuffer<>();
        dimAnnotations.append(annos);
        dims.append(parseExpression());
        accept(Tokens.TokenKind.RBRACKET);
        while (true) {
            if (this.token.kind == Tokens.TokenKind.LBRACKET || this.token.kind == Tokens.TokenKind.MONKEYS_AT) {
                List<JCTree.JCAnnotation> maybeDimAnnos = typeAnnotationsOpt();
                int pos = this.token.pos;
                nextToken();
                if (this.token.kind == Tokens.TokenKind.RBRACKET) {
                    elemtype = bracketsOptCont(elemtype, pos, maybeDimAnnos);
                } else if (this.token.kind == Tokens.TokenKind.RBRACKET) {
                    elemtype = bracketsOptCont(elemtype, pos, maybeDimAnnos);
                } else {
                    dimAnnotations.append(maybeDimAnnos);
                    dims.append(parseExpression());
                    accept(Tokens.TokenKind.RBRACKET);
                }
            } else {
                JCTree.JCNewArray na2 = (JCTree.JCNewArray) toP(this.F.at(newpos).NewArray(elemtype, dims.toList(), null));
                na2.dimAnnotations = dimAnnotations.toList();
                return na2;
            }
        }
    }

    JCTree.JCNewClass classCreatorRest(int newpos, JCTree.JCExpression encl, List<JCTree.JCExpression> typeArgs, JCTree.JCExpression t) {
        JCTree.JCClassDecl body;
        List<JCTree.JCExpression> args = arguments();
        if (this.token.kind != Tokens.TokenKind.LBRACE) {
            body = null;
        } else {
            int pos = this.token.pos;
            List<JCTree> defs = classOrInterfaceBody(this.names.empty, false);
            JCTree.JCModifiers mods = this.F.at(-1).Modifiers(0L);
            JCTree.JCClassDecl body2 = (JCTree.JCClassDecl) toP(this.F.at(pos).AnonymousClassDef(mods, defs));
            body = body2;
        }
        return (JCTree.JCNewClass) toP(this.F.at(newpos).NewClass(encl, typeArgs, t, args, body));
    }

    JCTree.JCExpression arrayInitializer(int newpos, JCTree.JCExpression t) {
        accept(Tokens.TokenKind.LBRACE);
        ListBuffer<JCTree.JCExpression> elems = new ListBuffer<>();
        if (this.token.kind == Tokens.TokenKind.COMMA) {
            nextToken();
        } else if (this.token.kind != Tokens.TokenKind.RBRACE) {
            elems.append(variableInitializer());
            while (this.token.kind == Tokens.TokenKind.COMMA) {
                nextToken();
                if (this.token.kind == Tokens.TokenKind.RBRACE) {
                    break;
                }
                elems.append(variableInitializer());
            }
        }
        accept(Tokens.TokenKind.RBRACE);
        return (JCTree.JCExpression) toP(this.F.at(newpos).NewArray(t, List.nil(), elems.toList()));
    }

    public JCTree.JCExpression variableInitializer() {
        return this.token.kind == Tokens.TokenKind.LBRACE ? arrayInitializer(this.token.pos, null) : parseExpression();
    }

    JCTree.JCExpression parExpression() {
        int pos = this.token.pos;
        accept(Tokens.TokenKind.LPAREN);
        JCTree.JCExpression t = parseExpression();
        accept(Tokens.TokenKind.RPAREN);
        return (JCTree.JCExpression) toP(this.F.at(pos).Parens(t));
    }

    JCTree.JCBlock block(int pos, long flags) {
        accept(Tokens.TokenKind.LBRACE);
        List<JCTree.JCStatement> stats = blockStatements();
        JCTree.JCBlock t = this.F.at(pos).Block(flags, stats);
        while (true) {
            if (this.token.kind == Tokens.TokenKind.CASE || this.token.kind == Tokens.TokenKind.DEFAULT) {
                syntaxError("orphaned", this.token.kind);
                switchBlockStatementGroups();
            } else {
                t.endpos = this.token.pos;
                accept(Tokens.TokenKind.RBRACE);
                return (JCTree.JCBlock) toP(t);
            }
        }
    }

    public JCTree.JCBlock block() {
        return block(this.token.pos, 0L);
    }

    List<JCTree.JCStatement> blockStatements() {
        ListBuffer<JCTree.JCStatement> stats = new ListBuffer<>();
        while (true) {
            List<JCTree.JCStatement> stat = blockStatement();
            if (stat.isEmpty()) {
                return stats.toList();
            }
            if (this.token.pos <= this.endPosTable.errorEndPos) {
                skip(false, true, true, true);
            }
            stats.addAll(stat);
        }
    }

    JCTree.JCStatement parseStatementAsBlock() {
        int pos = this.token.pos;
        List<JCTree.JCStatement> stats = blockStatement();
        if (stats.isEmpty()) {
            JCTree.JCErroneous e = this.F.at(pos).Erroneous();
            error(e, "illegal.start.of.stmt", new Object[0]);
            return this.F.at(pos).Exec(e);
        }
        JCTree.JCStatement first = stats.head;
        String error = null;
        switch (first.getTag()) {
            case CLASSDEF:
                error = "class.not.allowed";
                break;
            case VARDEF:
                error = "variable.not.allowed";
                break;
        }
        if (error != null) {
            error(first, error, new Object[0]);
            List<JCTree.JCBlock> blist = List.of(this.F.at(first.pos).Block(0L, stats));
            return (JCTree.JCStatement) toP(this.F.at(pos).Exec(this.F.at(first.pos).Erroneous(blist)));
        }
        return first;
    }

    List<JCTree.JCStatement> blockStatement() {
        int pos = this.token.pos;
        switch (this.token.kind) {
            case SEMI:
            case LBRACE:
            case SYNCHRONIZED:
            case IF:
            case FOR:
            case WHILE:
            case DO:
            case TRY:
            case SWITCH:
            case RETURN:
            case THROW:
            case BREAK:
            case CONTINUE:
            case ELSE:
            case FINALLY:
            case CATCH:
                return List.of(parseStatement());
            case FINAL:
            case MONKEYS_AT:
                Tokens.Comment dc = this.token.comment(Tokens.Comment.CommentStyle.JAVADOC);
                JCTree.JCModifiers mods = modifiersOpt();
                if (this.token.kind == Tokens.TokenKind.INTERFACE || this.token.kind == Tokens.TokenKind.CLASS || (this.allowEnums && this.token.kind == Tokens.TokenKind.ENUM)) {
                    return List.of(classOrInterfaceOrEnumDeclaration(mods, dc));
                }
                ListBuffer<JCTree.JCStatement> stats = variableDeclarators(mods, parseType(), new ListBuffer());
                storeEnd(stats.last(), this.token.endPos);
                accept(Tokens.TokenKind.SEMI);
                return stats.toList();
            case ABSTRACT:
            case STRICTFP:
                Tokens.Comment dc2 = this.token.comment(Tokens.Comment.CommentStyle.JAVADOC);
                JCTree.JCModifiers mods2 = modifiersOpt();
                return List.of(classOrInterfaceOrEnumDeclaration(mods2, dc2));
            case EOF:
            case RBRACE:
            case CASE:
            case DEFAULT:
                return List.nil();
            case CLASS:
            case INTERFACE:
                Tokens.Comment dc3 = this.token.comment(Tokens.Comment.CommentStyle.JAVADOC);
                return List.of(classOrInterfaceOrEnumDeclaration(modifiersOpt(), dc3));
            case ENUM:
            case ASSERT:
                if (this.allowEnums && this.token.kind == Tokens.TokenKind.ENUM) {
                    error(this.token.pos, "local.enum", new Object[0]);
                    Tokens.Comment dc4 = this.token.comment(Tokens.Comment.CommentStyle.JAVADOC);
                    return List.of(classOrInterfaceOrEnumDeclaration(modifiersOpt(), dc4));
                }
                if (this.allowAsserts && this.token.kind == Tokens.TokenKind.ASSERT) {
                    return List.of(parseStatement());
                }
                break;
        }
        Tokens.Token prevToken = this.token;
        JCTree.JCExpression t = term(3);
        if (this.token.kind == Tokens.TokenKind.COLON && t.hasTag(JCTree.Tag.IDENT)) {
            nextToken();
            JCTree.JCStatement stat = parseStatement();
            return List.of(this.F.at(pos).Labelled(prevToken.name(), stat));
        }
        if ((this.lastmode & 2) != 0 && this.LAX_IDENTIFIER.accepts(this.token.kind)) {
            int pos2 = this.token.pos;
            JCTree.JCModifiers mods3 = this.F.at(-1).Modifiers(0L);
            this.F.at(pos2);
            ListBuffer<JCTree.JCStatement> stats2 = variableDeclarators(mods3, t, new ListBuffer());
            storeEnd(stats2.last(), this.token.endPos);
            accept(Tokens.TokenKind.SEMI);
            return stats2.toList();
        }
        JCTree.JCExpressionStatement expr = (JCTree.JCExpressionStatement) to(this.F.at(pos).Exec(checkExprStat(t)));
        accept(Tokens.TokenKind.SEMI);
        return List.of(expr);
    }

    @Override // com.sun.tools.javac.parser.Parser
    public JCTree.JCStatement parseStatement() {
        int i = this.token.pos;
        switch (this.token.kind) {
            case SEMI:
                nextToken();
                return (JCTree.JCStatement) toP(this.F.at(i).Skip());
            case LBRACE:
                return block();
            case SYNCHRONIZED:
                nextToken();
                return this.F.at(i).Synchronized(parExpression(), block());
            case IF:
                nextToken();
                JCTree.JCExpression jCExpressionParExpression = parExpression();
                JCTree.JCStatement statementAsBlock = parseStatementAsBlock();
                JCTree.JCStatement statementAsBlock2 = null;
                if (this.token.kind == Tokens.TokenKind.ELSE) {
                    nextToken();
                    statementAsBlock2 = parseStatementAsBlock();
                }
                return this.F.at(i).If(jCExpressionParExpression, statementAsBlock, statementAsBlock2);
            case FOR:
                nextToken();
                accept(Tokens.TokenKind.LPAREN);
                List<JCTree.JCStatement> listNil = this.token.kind == Tokens.TokenKind.SEMI ? List.nil() : forInit();
                if (listNil.length() == 1 && listNil.head.hasTag(JCTree.Tag.VARDEF) && ((JCTree.JCVariableDecl) listNil.head).init == null && this.token.kind == Tokens.TokenKind.COLON) {
                    checkForeach();
                    JCTree.JCVariableDecl jCVariableDecl = (JCTree.JCVariableDecl) listNil.head;
                    accept(Tokens.TokenKind.COLON);
                    JCTree.JCExpression expression = parseExpression();
                    accept(Tokens.TokenKind.RPAREN);
                    return this.F.at(i).ForeachLoop(jCVariableDecl, expression, parseStatementAsBlock());
                }
                accept(Tokens.TokenKind.SEMI);
                JCTree.JCExpression expression2 = this.token.kind != Tokens.TokenKind.SEMI ? parseExpression() : null;
                accept(Tokens.TokenKind.SEMI);
                List<JCTree.JCExpressionStatement> listNil2 = this.token.kind == Tokens.TokenKind.RPAREN ? List.nil() : forUpdate();
                accept(Tokens.TokenKind.RPAREN);
                return this.F.at(i).ForLoop(listNil, expression2, listNil2, parseStatementAsBlock());
            case WHILE:
                nextToken();
                return this.F.at(i).WhileLoop(parExpression(), parseStatementAsBlock());
            case DO:
                nextToken();
                JCTree.JCStatement statementAsBlock3 = parseStatementAsBlock();
                accept(Tokens.TokenKind.WHILE);
                JCTree.JCDoWhileLoop jCDoWhileLoop = (JCTree.JCDoWhileLoop) to(this.F.at(i).DoLoop(statementAsBlock3, parExpression()));
                accept(Tokens.TokenKind.SEMI);
                return jCDoWhileLoop;
            case TRY:
                nextToken();
                List<JCTree> listNil3 = List.nil();
                if (this.token.kind == Tokens.TokenKind.LPAREN) {
                    checkTryWithResources();
                    nextToken();
                    listNil3 = resources();
                    accept(Tokens.TokenKind.RPAREN);
                }
                JCTree.JCBlock jCBlockBlock = block();
                ListBuffer listBuffer = new ListBuffer();
                JCTree.JCBlock jCBlockBlock2 = null;
                if (this.token.kind == Tokens.TokenKind.CATCH || this.token.kind == Tokens.TokenKind.FINALLY) {
                    while (this.token.kind == Tokens.TokenKind.CATCH) {
                        listBuffer.append(catchClause());
                    }
                    if (this.token.kind == Tokens.TokenKind.FINALLY) {
                        nextToken();
                        jCBlockBlock2 = block();
                    }
                } else if (this.allowTWR) {
                    if (listNil3.isEmpty()) {
                        error(i, "try.without.catch.finally.or.resource.decls", new Object[0]);
                    }
                } else {
                    error(i, "try.without.catch.or.finally", new Object[0]);
                }
                return this.F.at(i).Try(listNil3, jCBlockBlock, listBuffer.toList(), jCBlockBlock2);
            case SWITCH:
                nextToken();
                JCTree.JCExpression jCExpressionParExpression2 = parExpression();
                accept(Tokens.TokenKind.LBRACE);
                JCTree.JCSwitch jCSwitch = (JCTree.JCSwitch) to(this.F.at(i).Switch(jCExpressionParExpression2, switchBlockStatementGroups()));
                accept(Tokens.TokenKind.RBRACE);
                return jCSwitch;
            case RETURN:
                nextToken();
                JCTree.JCReturn jCReturn = (JCTree.JCReturn) to(this.F.at(i).Return(this.token.kind != Tokens.TokenKind.SEMI ? parseExpression() : null));
                accept(Tokens.TokenKind.SEMI);
                return jCReturn;
            case THROW:
                nextToken();
                JCTree.JCThrow jCThrow = (JCTree.JCThrow) to(this.F.at(i).Throw(parseExpression()));
                accept(Tokens.TokenKind.SEMI);
                return jCThrow;
            case BREAK:
                nextToken();
                JCTree.JCBreak jCBreak = (JCTree.JCBreak) to(this.F.at(i).Break(this.LAX_IDENTIFIER.accepts(this.token.kind) ? ident() : null));
                accept(Tokens.TokenKind.SEMI);
                return jCBreak;
            case CONTINUE:
                nextToken();
                JCTree.JCContinue jCContinue = (JCTree.JCContinue) to(this.F.at(i).Continue(this.LAX_IDENTIFIER.accepts(this.token.kind) ? ident() : null));
                accept(Tokens.TokenKind.SEMI);
                return jCContinue;
            case ELSE:
                int i2 = this.token.pos;
                nextToken();
                return doRecover(i2, BasicErrorRecoveryAction.BLOCK_STMT, "else.without.if");
            case FINALLY:
                int i3 = this.token.pos;
                nextToken();
                return doRecover(i3, BasicErrorRecoveryAction.BLOCK_STMT, "finally.without.try");
            case CATCH:
                return doRecover(this.token.pos, BasicErrorRecoveryAction.CATCH_CLAUSE, "catch.without.try");
            case ASSERT:
                if (this.allowAsserts && this.token.kind == Tokens.TokenKind.ASSERT) {
                    nextToken();
                    JCTree.JCExpression expression3 = parseExpression();
                    JCTree.JCExpression expression4 = null;
                    if (this.token.kind == Tokens.TokenKind.COLON) {
                        nextToken();
                        expression4 = parseExpression();
                    }
                    JCTree.JCAssert jCAssert = (JCTree.JCAssert) to(this.F.at(i).Assert(expression3, expression4));
                    accept(Tokens.TokenKind.SEMI);
                    return jCAssert;
                }
                break;
        }
        Tokens.Token token = this.token;
        JCTree.JCExpression expression5 = parseExpression();
        if (this.token.kind == Tokens.TokenKind.COLON && expression5.hasTag(JCTree.Tag.IDENT)) {
            nextToken();
            return this.F.at(i).Labelled(token.name(), parseStatement());
        }
        JCTree.JCExpressionStatement jCExpressionStatement = (JCTree.JCExpressionStatement) to(this.F.at(i).Exec(checkExprStat(expression5)));
        accept(Tokens.TokenKind.SEMI);
        return jCExpressionStatement;
    }

    private JCTree.JCStatement doRecover(int startPos, ErrorRecoveryAction action, String key) {
        int errPos = this.S.errPos();
        JCTree stm = action.doRecover(this);
        this.S.errPos(errPos);
        return (JCTree.JCStatement) toP(this.F.Exec(syntaxError(startPos, List.of(stm), key, new Tokens.TokenKind[0])));
    }

    protected JCTree.JCCatch catchClause() {
        int pos = this.token.pos;
        accept(Tokens.TokenKind.CATCH);
        accept(Tokens.TokenKind.LPAREN);
        JCTree.JCModifiers mods = optFinal(8589934592L);
        List<JCTree.JCExpression> catchTypes = catchTypes();
        JCTree.JCExpression paramType = (JCTree.JCExpression) (catchTypes.size() > 1 ? toP(this.F.at(catchTypes.head.getStartPosition()).TypeUnion(catchTypes)) : catchTypes.head);
        JCTree.JCVariableDecl formal = variableDeclaratorId(mods, paramType);
        accept(Tokens.TokenKind.RPAREN);
        JCTree.JCBlock body = block();
        return this.F.at(pos).Catch(formal, body);
    }

    List<JCTree.JCExpression> catchTypes() {
        ListBuffer<JCTree.JCExpression> catchTypes = new ListBuffer<>();
        catchTypes.add(parseType());
        while (this.token.kind == Tokens.TokenKind.BAR) {
            checkMulticatch();
            nextToken();
            catchTypes.add(parseType());
        }
        return catchTypes.toList();
    }

    List<JCTree.JCCase> switchBlockStatementGroups() {
        ListBuffer<JCTree.JCCase> cases = new ListBuffer<>();
        while (true) {
            int pos = this.token.pos;
            switch (this.token.kind) {
                case EOF:
                case RBRACE:
                    return cases.toList();
                case CASE:
                case DEFAULT:
                    cases.append(switchBlockStatementGroup());
                    break;
                default:
                    nextToken();
                    syntaxError(pos, "expected3", Tokens.TokenKind.CASE, Tokens.TokenKind.DEFAULT, Tokens.TokenKind.RBRACE);
                    break;
            }
        }
    }

    protected JCTree.JCCase switchBlockStatementGroup() {
        int pos = this.token.pos;
        switch (this.token.kind) {
            case CASE:
                nextToken();
                JCTree.JCExpression pat = parseExpression();
                accept(Tokens.TokenKind.COLON);
                List<JCTree.JCStatement> stats = blockStatements();
                JCTree.JCCase c = this.F.at(pos).Case(pat, stats);
                if (stats.isEmpty()) {
                    storeEnd(c, this.S.prevToken().endPos);
                }
                return c;
            case DEFAULT:
                nextToken();
                accept(Tokens.TokenKind.COLON);
                List<JCTree.JCStatement> stats2 = blockStatements();
                JCTree.JCCase c2 = this.F.at(pos).Case(null, stats2);
                if (stats2.isEmpty()) {
                    storeEnd(c2, this.S.prevToken().endPos);
                }
                return c2;
            default:
                throw new AssertionError("should not reach here");
        }
    }

    <T extends ListBuffer<? super JCTree.JCExpressionStatement>> T moreStatementExpressions(int pos, JCTree.JCExpression first, T stats) {
        stats.append(toP(this.F.at(pos).Exec(checkExprStat(first))));
        while (this.token.kind == Tokens.TokenKind.COMMA) {
            nextToken();
            int pos2 = this.token.pos;
            JCTree.JCExpression t = parseExpression();
            stats.append(toP(this.F.at(pos2).Exec(checkExprStat(t))));
        }
        return stats;
    }

    List<JCTree.JCStatement> forInit() {
        ListBuffer<JCTree.JCStatement> stats = new ListBuffer<>();
        int pos = this.token.pos;
        if (this.token.kind == Tokens.TokenKind.FINAL || this.token.kind == Tokens.TokenKind.MONKEYS_AT) {
            return variableDeclarators(optFinal(0L), parseType(), stats).toList();
        }
        JCTree.JCExpression t = term(3);
        if ((this.lastmode & 2) != 0 && this.LAX_IDENTIFIER.accepts(this.token.kind)) {
            return variableDeclarators(mods(pos, 0L, List.nil()), t, stats).toList();
        }
        if ((this.lastmode & 2) != 0 && this.token.kind == Tokens.TokenKind.COLON) {
            error(pos, "bad.initializer", "for-loop");
            return List.of(this.F.at(pos).VarDef(null, null, t, null));
        }
        return moreStatementExpressions(pos, t, stats).toList();
    }

    List<JCTree.JCExpressionStatement> forUpdate() {
        return moreStatementExpressions(this.token.pos, parseExpression(), new ListBuffer()).toList();
    }

    List<JCTree.JCAnnotation> annotationsOpt(JCTree.Tag kind) {
        if (this.token.kind != Tokens.TokenKind.MONKEYS_AT) {
            return List.nil();
        }
        ListBuffer<JCTree.JCAnnotation> buf = new ListBuffer<>();
        int prevmode = this.mode;
        while (this.token.kind == Tokens.TokenKind.MONKEYS_AT) {
            int pos = this.token.pos;
            nextToken();
            buf.append(annotation(pos, kind));
        }
        this.lastmode = this.mode;
        this.mode = prevmode;
        List<JCTree.JCAnnotation> annotations = buf.toList();
        return annotations;
    }

    List<JCTree.JCAnnotation> typeAnnotationsOpt() {
        List<JCTree.JCAnnotation> annotations = annotationsOpt(JCTree.Tag.TYPE_ANNOTATION);
        return annotations;
    }

    JCTree.JCModifiers modifiersOpt() {
        return modifiersOpt(null);
    }

    protected JCTree.JCModifiers modifiersOpt(JCTree.JCModifiers partial) {
        long flags;
        int pos;
        long flag;
        ListBuffer<JCTree.JCAnnotation> annotations = new ListBuffer<>();
        if (partial == null) {
            flags = 0;
            pos = this.token.pos;
        } else {
            flags = partial.flags;
            annotations.appendList(partial.annotations);
            pos = partial.pos;
        }
        if (this.token.deprecatedFlag()) {
            flags |= 131072;
        }
        while (true) {
            switch (this.token.kind) {
                case PUBLIC:
                    flag = 1;
                    break;
                case FINAL:
                    flag = 16;
                    break;
                case ABSTRACT:
                    flag = 1024;
                    break;
                case MONKEYS_AT:
                    flag = 8192;
                    break;
                case PRIVATE:
                    flag = 2;
                    break;
                case PROTECTED:
                    flag = 4;
                    break;
                case STATIC:
                    flag = 8;
                    break;
                case TRANSIENT:
                    flag = 128;
                    break;
                case NATIVE:
                    flag = 256;
                    break;
                case VOLATILE:
                    flag = 64;
                    break;
                case SYNCHRONIZED:
                    flag = 32;
                    break;
                case STRICTFP:
                    flag = 2048;
                    break;
                case DEFAULT:
                    checkDefaultMethods();
                    flag = Flags.DEFAULT;
                    break;
                case ERROR:
                    flag = 0;
                    nextToken();
                    break;
                default:
                    switch (this.token.kind) {
                        case INTERFACE:
                            flags |= 512;
                            break;
                        case ENUM:
                            flags |= 16384;
                            break;
                    }
                    return mods(pos, flags, annotations.toList());
            }
            if ((flags & flag) != 0) {
                error(this.token.pos, "repeated.modifier", new Object[0]);
            }
            int lastPos = this.token.pos;
            nextToken();
            if (flag == 8192) {
                checkAnnotations();
                if (this.token.kind != Tokens.TokenKind.INTERFACE) {
                    JCTree.JCAnnotation ann = annotation(lastPos, JCTree.Tag.ANNOTATION);
                    if (flags == 0 && annotations.isEmpty()) {
                        pos = ann.pos;
                    }
                    annotations.append(ann);
                    flag = 0;
                }
            }
            flags |= flag;
        }
    }

    JCTree.JCModifiers mods(int pos, long flags, List<JCTree.JCAnnotation> annotations) {
        if ((8796093033983L & flags) == 0 && annotations.isEmpty()) {
            pos = -1;
        }
        JCTree.JCModifiers mods = this.F.at(pos).Modifiers(flags, annotations);
        if (pos != -1) {
            storeEnd(mods, this.S.prevToken().endPos);
        }
        return mods;
    }

    JCTree.JCAnnotation annotation(int pos, JCTree.Tag kind) {
        JCTree.JCAnnotation ann;
        checkAnnotations();
        if (kind == JCTree.Tag.TYPE_ANNOTATION) {
            checkTypeAnnotations();
        }
        JCTree.JCExpression ident = qualident(false);
        List<JCTree.JCExpression> fieldValues = annotationFieldValuesOpt();
        if (kind == JCTree.Tag.ANNOTATION) {
            ann = this.F.at(pos).Annotation(ident, fieldValues);
        } else if (kind == JCTree.Tag.TYPE_ANNOTATION) {
            ann = this.F.at(pos).TypeAnnotation(ident, fieldValues);
        } else {
            throw new AssertionError("Unhandled annotation kind: " + kind);
        }
        storeEnd(ann, this.S.prevToken().endPos);
        return ann;
    }

    List<JCTree.JCExpression> annotationFieldValuesOpt() {
        return this.token.kind == Tokens.TokenKind.LPAREN ? annotationFieldValues() : List.nil();
    }

    List<JCTree.JCExpression> annotationFieldValues() {
        accept(Tokens.TokenKind.LPAREN);
        ListBuffer<JCTree.JCExpression> buf = new ListBuffer<>();
        if (this.token.kind != Tokens.TokenKind.RPAREN) {
            buf.append(annotationFieldValue());
            while (this.token.kind == Tokens.TokenKind.COMMA) {
                nextToken();
                buf.append(annotationFieldValue());
            }
        }
        accept(Tokens.TokenKind.RPAREN);
        return buf.toList();
    }

    JCTree.JCExpression annotationFieldValue() {
        if (this.LAX_IDENTIFIER.accepts(this.token.kind)) {
            this.mode = 1;
            JCTree.JCExpression t1 = term1();
            if (t1.hasTag(JCTree.Tag.IDENT) && this.token.kind == Tokens.TokenKind.EQ) {
                int pos = this.token.pos;
                accept(Tokens.TokenKind.EQ);
                JCTree.JCExpression v = annotationValue();
                return (JCTree.JCExpression) toP(this.F.at(pos).Assign(t1, v));
            }
            return t1;
        }
        return annotationValue();
    }

    JCTree.JCExpression annotationValue() {
        switch (this.token.kind) {
            case MONKEYS_AT:
                int pos = this.token.pos;
                nextToken();
                return annotation(pos, JCTree.Tag.ANNOTATION);
            case LBRACE:
                int pos2 = this.token.pos;
                accept(Tokens.TokenKind.LBRACE);
                ListBuffer<JCTree.JCExpression> buf = new ListBuffer<>();
                if (this.token.kind == Tokens.TokenKind.COMMA) {
                    nextToken();
                } else if (this.token.kind != Tokens.TokenKind.RBRACE) {
                    buf.append(annotationValue());
                    while (this.token.kind == Tokens.TokenKind.COMMA) {
                        nextToken();
                        if (this.token.kind != Tokens.TokenKind.RBRACE) {
                            buf.append(annotationValue());
                        }
                    }
                }
                accept(Tokens.TokenKind.RBRACE);
                return (JCTree.JCExpression) toP(this.F.at(pos2).NewArray(null, List.nil(), buf.toList()));
            default:
                this.mode = 1;
                return term1();
        }
    }

    public <T extends ListBuffer<? super JCTree.JCVariableDecl>> T variableDeclarators(JCTree.JCModifiers jCModifiers, JCTree.JCExpression jCExpression, T t) {
        return (T) variableDeclaratorsRest(this.token.pos, jCModifiers, jCExpression, ident(), false, null, t);
    }

    <T extends ListBuffer<? super JCTree.JCVariableDecl>> T variableDeclaratorsRest(int pos, JCTree.JCModifiers mods, JCTree.JCExpression type, Name name, boolean reqInit, Tokens.Comment dc, T vdefs) {
        vdefs.append(variableDeclaratorRest(pos, mods, type, name, reqInit, dc));
        while (this.token.kind == Tokens.TokenKind.COMMA) {
            storeEnd((JCTree) vdefs.last(), this.token.endPos);
            nextToken();
            vdefs.append(variableDeclarator(mods, type, reqInit, dc));
        }
        return vdefs;
    }

    JCTree.JCVariableDecl variableDeclarator(JCTree.JCModifiers mods, JCTree.JCExpression type, boolean reqInit, Tokens.Comment dc) {
        return variableDeclaratorRest(this.token.pos, mods, type, ident(), reqInit, dc);
    }

    JCTree.JCVariableDecl variableDeclaratorRest(int pos, JCTree.JCModifiers mods, JCTree.JCExpression type, Name name, boolean reqInit, Tokens.Comment dc) {
        JCTree.JCExpression type2 = bracketsOpt(type);
        JCTree.JCExpression init = null;
        if (this.token.kind == Tokens.TokenKind.EQ) {
            nextToken();
            init = variableInitializer();
        } else if (reqInit) {
            syntaxError(this.token.pos, "expected", Tokens.TokenKind.EQ);
        }
        JCTree.JCVariableDecl result = (JCTree.JCVariableDecl) toP(this.F.at(pos).VarDef(mods, name, type2, init));
        attach(result, dc);
        return result;
    }

    JCTree.JCVariableDecl variableDeclaratorId(JCTree.JCModifiers mods, JCTree.JCExpression type) {
        return variableDeclaratorId(mods, type, false);
    }

    JCTree.JCVariableDecl variableDeclaratorId(JCTree.JCModifiers mods, JCTree.JCExpression type, boolean lambdaParameter) {
        Name name;
        int pos = this.token.pos;
        if (lambdaParameter && this.token.kind == Tokens.TokenKind.UNDERSCORE) {
            this.log.error(pos, "underscore.as.identifier.in.lambda", new Object[0]);
            name = this.token.name();
            nextToken();
        } else if (this.allowThisIdent) {
            JCTree.JCExpression pn = qualident(false);
            if (pn.hasTag(JCTree.Tag.IDENT) && ((JCTree.JCIdent) pn).name != this.names._this) {
                Name name2 = ((JCTree.JCIdent) pn).name;
                name = name2;
            } else {
                if ((Flags.VARARGS & mods.flags) != 0) {
                    this.log.error(this.token.pos, "varargs.and.receiver", new Object[0]);
                }
                if (this.token.kind == Tokens.TokenKind.LBRACKET) {
                    this.log.error(this.token.pos, "array.and.receiver", new Object[0]);
                }
                return (JCTree.JCVariableDecl) toP(this.F.at(pos).ReceiverVarDef(mods, pn, type));
            }
        } else {
            name = ident();
        }
        if ((Flags.VARARGS & mods.flags) != 0 && this.token.kind == Tokens.TokenKind.LBRACKET) {
            this.log.error(this.token.pos, "varargs.and.old.array.syntax", new Object[0]);
        }
        return (JCTree.JCVariableDecl) toP(this.F.at(pos).VarDef(mods, name, bracketsOpt(type), null));
    }

    List<JCTree> resources() {
        ListBuffer<JCTree> defs = new ListBuffer<>();
        defs.append(resource());
        while (this.token.kind == Tokens.TokenKind.SEMI) {
            storeEnd(defs.last(), this.token.endPos);
            int i = this.token.pos;
            nextToken();
            if (this.token.kind == Tokens.TokenKind.RPAREN) {
                break;
            }
            defs.append(resource());
        }
        return defs.toList();
    }

    protected JCTree resource() {
        JCTree.JCModifiers optFinal = optFinal(16L);
        JCTree.JCExpression type = parseType();
        int pos = this.token.pos;
        Name ident = ident();
        return variableDeclaratorRest(pos, optFinal, type, ident, true, null);
    }

    @Override // com.sun.tools.javac.parser.Parser
    public JCTree.JCCompilationUnit parseCompilationUnit() {
        Tokens.Token firstToken = this.token;
        JCTree.JCExpression pid = null;
        JCTree.JCModifiers mods = null;
        boolean consumedToplevelDoc = false;
        boolean seenImport = false;
        boolean seenPackage = false;
        List<JCTree.JCAnnotation> packageAnnotations = List.nil();
        if (this.token.kind == Tokens.TokenKind.MONKEYS_AT) {
            mods = modifiersOpt();
        }
        if (this.token.kind == Tokens.TokenKind.PACKAGE) {
            seenPackage = true;
            if (mods != null) {
                checkNoMods(mods.flags);
                packageAnnotations = mods.annotations;
                mods = null;
            }
            nextToken();
            pid = qualident(false);
            accept(Tokens.TokenKind.SEMI);
        }
        ListBuffer<JCTree> defs = new ListBuffer<>();
        boolean checkForImports = true;
        boolean firstTypeDecl = true;
        while (this.token.kind != Tokens.TokenKind.EOF) {
            if (this.token.pos > 0 && this.token.pos <= this.endPosTable.errorEndPos) {
                skip(checkForImports, false, false, false);
                if (this.token.kind == Tokens.TokenKind.EOF) {
                    break;
                }
            }
            if (checkForImports && mods == null && this.token.kind == Tokens.TokenKind.IMPORT) {
                seenImport = true;
                defs.append(importDeclaration());
            } else {
                Tokens.Comment docComment = this.token.comment(Tokens.Comment.CommentStyle.JAVADOC);
                if (firstTypeDecl && !seenImport && !seenPackage) {
                    docComment = firstToken.comment(Tokens.Comment.CommentStyle.JAVADOC);
                    consumedToplevelDoc = true;
                }
                JCTree def = typeDeclaration(mods, docComment);
                if (def instanceof JCTree.JCExpressionStatement) {
                    def = ((JCTree.JCExpressionStatement) def).expr;
                }
                defs.append(def);
                if (def instanceof JCTree.JCClassDecl) {
                    checkForImports = false;
                }
                mods = null;
                firstTypeDecl = false;
            }
        }
        JCTree.JCCompilationUnit toplevel = this.F.at(firstToken.pos).TopLevel(packageAnnotations, pid, defs.toList());
        if (!consumedToplevelDoc) {
            attach(toplevel, firstToken.comment(Tokens.Comment.CommentStyle.JAVADOC));
        }
        if (defs.isEmpty()) {
            storeEnd(toplevel, this.S.prevToken().endPos);
        }
        if (this.keepDocComments) {
            toplevel.docComments = this.docComments;
        }
        if (this.keepLineMap) {
            toplevel.lineMap = this.S.getLineMap();
        }
        this.endPosTable.setParser(null);
        toplevel.endPositions = this.endPosTable;
        return toplevel;
    }

    JCTree importDeclaration() {
        int pos = this.token.pos;
        nextToken();
        boolean importStatic = false;
        if (this.token.kind == Tokens.TokenKind.STATIC) {
            checkStaticImports();
            importStatic = true;
            nextToken();
        }
        JCTree.JCExpression pid = (JCTree.JCExpression) toP(this.F.at(this.token.pos).Ident(ident()));
        while (true) {
            int pos1 = this.token.pos;
            accept(Tokens.TokenKind.DOT);
            if (this.token.kind == Tokens.TokenKind.STAR) {
                pid = (JCTree.JCExpression) to(this.F.at(pos1).Select(pid, this.names.asterisk));
                nextToken();
                break;
            }
            pid = (JCTree.JCExpression) toP(this.F.at(pos1).Select(pid, ident()));
            if (this.token.kind != Tokens.TokenKind.DOT) {
                break;
            }
        }
        accept(Tokens.TokenKind.SEMI);
        return toP(this.F.at(pos).Import(pid, importStatic));
    }

    JCTree typeDeclaration(JCTree.JCModifiers mods, Tokens.Comment docComment) {
        int pos = this.token.pos;
        if (mods == null && this.token.kind == Tokens.TokenKind.SEMI) {
            nextToken();
            return toP(this.F.at(pos).Skip());
        }
        return classOrInterfaceOrEnumDeclaration(modifiersOpt(mods), docComment);
    }

    JCTree.JCStatement classOrInterfaceOrEnumDeclaration(JCTree.JCModifiers mods, Tokens.Comment dc) {
        List<JCTree> errs;
        List<JCTree> errs2;
        if (this.token.kind == Tokens.TokenKind.CLASS) {
            return classDeclaration(mods, dc);
        }
        if (this.token.kind == Tokens.TokenKind.INTERFACE) {
            return interfaceDeclaration(mods, dc);
        }
        if (this.allowEnums) {
            if (this.token.kind == Tokens.TokenKind.ENUM) {
                return enumDeclaration(mods, dc);
            }
            int pos = this.token.pos;
            if (this.LAX_IDENTIFIER.accepts(this.token.kind)) {
                errs2 = List.of((JCTree) mods, toP(this.F.at(pos).Ident(ident())));
                setErrorEndPos(this.token.pos);
            } else {
                errs2 = List.of(mods);
            }
            return (JCTree.JCStatement) toP(this.F.Exec(syntaxError(pos, errs2, "expected3", Tokens.TokenKind.CLASS, Tokens.TokenKind.INTERFACE, Tokens.TokenKind.ENUM)));
        }
        if (this.token.kind == Tokens.TokenKind.ENUM) {
            error(this.token.pos, "enums.not.supported.in.source", this.source.name);
            this.allowEnums = true;
            return enumDeclaration(mods, dc);
        }
        int pos2 = this.token.pos;
        if (this.LAX_IDENTIFIER.accepts(this.token.kind)) {
            errs = List.of((JCTree) mods, toP(this.F.at(pos2).Ident(ident())));
            setErrorEndPos(this.token.pos);
        } else {
            errs = List.of(mods);
        }
        return (JCTree.JCStatement) toP(this.F.Exec(syntaxError(pos2, errs, "expected2", Tokens.TokenKind.CLASS, Tokens.TokenKind.INTERFACE)));
    }

    protected JCTree.JCClassDecl classDeclaration(JCTree.JCModifiers mods, Tokens.Comment dc) {
        JCTree.JCExpression extending;
        List<JCTree.JCExpression> implementing;
        int pos = this.token.pos;
        accept(Tokens.TokenKind.CLASS);
        Name name = ident();
        List<JCTree.JCTypeParameter> typarams = typeParametersOpt();
        if (this.token.kind != Tokens.TokenKind.EXTENDS) {
            extending = null;
        } else {
            nextToken();
            JCTree.JCExpression extending2 = parseType();
            extending = extending2;
        }
        List<JCTree.JCExpression> implementing2 = List.nil();
        if (this.token.kind != Tokens.TokenKind.IMPLEMENTS) {
            implementing = implementing2;
        } else {
            nextToken();
            List<JCTree.JCExpression> implementing3 = typeList();
            implementing = implementing3;
        }
        List<JCTree> defs = classOrInterfaceBody(name, false);
        JCTree.JCClassDecl result = (JCTree.JCClassDecl) toP(this.F.at(pos).ClassDef(mods, name, typarams, extending, implementing, defs));
        attach(result, dc);
        return result;
    }

    protected JCTree.JCClassDecl interfaceDeclaration(JCTree.JCModifiers mods, Tokens.Comment dc) {
        List<JCTree.JCExpression> extending;
        int pos = this.token.pos;
        accept(Tokens.TokenKind.INTERFACE);
        Name name = ident();
        List<JCTree.JCTypeParameter> typarams = typeParametersOpt();
        List<JCTree.JCExpression> extending2 = List.nil();
        if (this.token.kind != Tokens.TokenKind.EXTENDS) {
            extending = extending2;
        } else {
            nextToken();
            List<JCTree.JCExpression> extending3 = typeList();
            extending = extending3;
        }
        List<JCTree> defs = classOrInterfaceBody(name, true);
        JCTree.JCClassDecl result = (JCTree.JCClassDecl) toP(this.F.at(pos).ClassDef(mods, name, typarams, null, extending, defs));
        attach(result, dc);
        return result;
    }

    protected JCTree.JCClassDecl enumDeclaration(JCTree.JCModifiers mods, Tokens.Comment dc) {
        List<JCTree.JCExpression> implementing;
        int pos = this.token.pos;
        accept(Tokens.TokenKind.ENUM);
        Name name = ident();
        List<JCTree.JCExpression> implementing2 = List.nil();
        if (this.token.kind != Tokens.TokenKind.IMPLEMENTS) {
            implementing = implementing2;
        } else {
            nextToken();
            List<JCTree.JCExpression> implementing3 = typeList();
            implementing = implementing3;
        }
        List<JCTree> defs = enumBody(name);
        mods.flags |= 16384;
        JCTree.JCClassDecl result = (JCTree.JCClassDecl) toP(this.F.at(pos).ClassDef(mods, name, List.nil(), null, implementing, defs));
        attach(result, dc);
        return result;
    }

    List<JCTree> enumBody(Name enumName) {
        accept(Tokens.TokenKind.LBRACE);
        ListBuffer<JCTree> defs = new ListBuffer<>();
        if (this.token.kind == Tokens.TokenKind.COMMA) {
            nextToken();
        } else if (this.token.kind != Tokens.TokenKind.RBRACE && this.token.kind != Tokens.TokenKind.SEMI) {
            defs.append(enumeratorDeclaration(enumName));
            while (this.token.kind == Tokens.TokenKind.COMMA) {
                nextToken();
                if (this.token.kind == Tokens.TokenKind.RBRACE || this.token.kind == Tokens.TokenKind.SEMI) {
                    break;
                }
                defs.append(enumeratorDeclaration(enumName));
            }
            if (this.token.kind != Tokens.TokenKind.SEMI && this.token.kind != Tokens.TokenKind.RBRACE) {
                defs.append(syntaxError(this.token.pos, "expected3", Tokens.TokenKind.COMMA, Tokens.TokenKind.RBRACE, Tokens.TokenKind.SEMI));
                nextToken();
            }
        }
        if (this.token.kind == Tokens.TokenKind.SEMI) {
            nextToken();
            while (this.token.kind != Tokens.TokenKind.RBRACE && this.token.kind != Tokens.TokenKind.EOF) {
                defs.appendList(classOrInterfaceBodyDeclaration(enumName, false));
                if (this.token.pos <= this.endPosTable.errorEndPos) {
                    skip(false, true, true, false);
                }
            }
        }
        accept(Tokens.TokenKind.RBRACE);
        return defs.toList();
    }

    JCTree enumeratorDeclaration(Name enumName) {
        JCTree.JCClassDecl body;
        int createPos;
        Tokens.Comment dc = this.token.comment(Tokens.Comment.CommentStyle.JAVADOC);
        int flags = this.token.deprecatedFlag() ? 16409 | 131072 : 16409;
        int pos = this.token.pos;
        List<JCTree.JCAnnotation> annotations = annotationsOpt(JCTree.Tag.ANNOTATION);
        JCTree.JCModifiers mods = this.F.at(annotations.isEmpty() ? -1 : pos).Modifiers(flags, annotations);
        List<JCTree.JCExpression> typeArgs = typeArgumentsOpt();
        int identPos = this.token.pos;
        Name name = ident();
        int createPos2 = this.token.pos;
        List<JCTree.JCExpression> args = this.token.kind == Tokens.TokenKind.LPAREN ? arguments() : List.nil();
        if (this.token.kind != Tokens.TokenKind.LBRACE) {
            body = null;
        } else {
            JCTree.JCModifiers mods1 = this.F.at(-1).Modifiers(16392L);
            List<JCTree> defs = classOrInterfaceBody(this.names.empty, false);
            JCTree.JCClassDecl body2 = (JCTree.JCClassDecl) toP(this.F.at(identPos).AnonymousClassDef(mods1, defs));
            body = body2;
        }
        if (args.isEmpty() && body == null) {
            createPos = identPos;
        } else {
            createPos = createPos2;
        }
        JCTree.JCIdent ident = this.F.at(identPos).Ident(enumName);
        int createPos3 = createPos;
        JCTree.JCNewClass create = this.F.at(createPos).NewClass(null, typeArgs, ident, args, body);
        if (createPos3 != identPos) {
            storeEnd(create, this.S.prevToken().endPos);
        }
        JCTree.JCIdent ident2 = this.F.at(identPos).Ident(enumName);
        JCTree result = toP(this.F.at(pos).VarDef(mods, name, ident2, create));
        attach(result, dc);
        return result;
    }

    List<JCTree.JCExpression> typeList() {
        ListBuffer<JCTree.JCExpression> ts = new ListBuffer<>();
        ts.append(parseType());
        while (this.token.kind == Tokens.TokenKind.COMMA) {
            nextToken();
            ts.append(parseType());
        }
        return ts.toList();
    }

    List<JCTree> classOrInterfaceBody(Name className, boolean isInterface) {
        accept(Tokens.TokenKind.LBRACE);
        if (this.token.pos <= this.endPosTable.errorEndPos) {
            skip(false, true, false, false);
            if (this.token.kind == Tokens.TokenKind.LBRACE) {
                nextToken();
            }
        }
        ListBuffer<JCTree> defs = new ListBuffer<>();
        while (this.token.kind != Tokens.TokenKind.RBRACE && this.token.kind != Tokens.TokenKind.EOF) {
            defs.appendList(classOrInterfaceBodyDeclaration(className, isInterface));
            if (this.token.pos <= this.endPosTable.errorEndPos) {
                skip(false, true, true, false);
            }
        }
        accept(Tokens.TokenKind.RBRACE);
        return defs.toList();
    }

    protected List<JCTree> classOrInterfaceBodyDeclaration(Name className, boolean isInterface) {
        JCTree.JCModifiers mods;
        JCTree.JCExpression type;
        if (this.token.kind == Tokens.TokenKind.SEMI) {
            nextToken();
            return List.nil();
        }
        Tokens.Comment dc = this.token.comment(Tokens.Comment.CommentStyle.JAVADOC);
        int pos = this.token.pos;
        JCTree.JCModifiers mods2 = modifiersOpt();
        if (this.token.kind != Tokens.TokenKind.CLASS && this.token.kind != Tokens.TokenKind.INTERFACE) {
            if (!this.allowEnums || this.token.kind != Tokens.TokenKind.ENUM) {
                if (this.token.kind == Tokens.TokenKind.LBRACE && !isInterface && (mods2.flags & 4095 & (-9)) == 0 && mods2.annotations.isEmpty()) {
                    return List.of(block(pos, mods2.flags));
                }
                int pos2 = this.token.pos;
                List<JCTree.JCTypeParameter> typarams = typeParametersOpt();
                if (typarams.nonEmpty() && mods2.pos == -1) {
                    mods2.pos = pos2;
                    storeEnd(mods2, pos2);
                }
                List<JCTree.JCAnnotation> annosAfterParams = annotationsOpt(JCTree.Tag.ANNOTATION);
                if (annosAfterParams.nonEmpty()) {
                    checkAnnotationsAfterTypeParams(annosAfterParams.head.pos);
                    mods2.annotations = mods2.annotations.appendList(annosAfterParams);
                    if (mods2.pos == -1) {
                        mods2.pos = mods2.annotations.head.pos;
                    }
                }
                Tokens.Token tk = this.token;
                int pos3 = this.token.pos;
                boolean isVoid = this.token.kind == Tokens.TokenKind.VOID;
                if (isVoid) {
                    JCTree.JCExpression type2 = (JCTree.JCExpression) to(this.F.at(pos3).TypeIdent(TypeTag.VOID));
                    nextToken();
                    type = type2;
                } else {
                    JCTree.JCExpression type3 = unannotatedType();
                    type = type3;
                }
                if (this.token.kind == Tokens.TokenKind.LPAREN && !isInterface && type.hasTag(JCTree.Tag.IDENT)) {
                    if (isInterface || tk.name() != className) {
                        error(pos3, "invalid.meth.decl.ret.type.req", new Object[0]);
                    } else if (annosAfterParams.nonEmpty()) {
                        illegal(annosAfterParams.head.pos);
                    }
                    return List.of(methodDeclaratorRest(pos3, mods2, null, this.names.init, typarams, isInterface, true, dc));
                }
                int pos4 = this.token.pos;
                Name name = ident();
                if (this.token.kind == Tokens.TokenKind.LPAREN) {
                    return List.of(methodDeclaratorRest(pos4, mods2, type, name, typarams, isInterface, isVoid, dc));
                }
                if (!isVoid && typarams.isEmpty()) {
                    List<JCTree> defs = variableDeclaratorsRest(pos4, mods2, type, name, isInterface, dc, new ListBuffer()).toList();
                    storeEnd(defs.last(), this.token.endPos);
                    accept(Tokens.TokenKind.SEMI);
                    return defs;
                }
                int pos5 = this.token.pos;
                List<JCTree> err = isVoid ? List.of(toP(this.F.at(pos5).MethodDef(mods2, name, type, typarams, List.nil(), List.nil(), null, null))) : null;
                return List.of(syntaxError(this.token.pos, err, "expected", Tokens.TokenKind.LPAREN));
            }
            mods = mods2;
        } else {
            mods = mods2;
        }
        return List.of(classOrInterfaceOrEnumDeclaration(mods, dc));
    }

    /* JADX WARN: Removed duplicated region for block: B:32:0x008f  */
    /*
        Code decompiled incorrectly, please refer to instructions dump.
        To view partially-correct code enable 'Show inconsistent code' option in preferences
    */
    protected com.sun.tools.javac.tree.JCTree methodDeclaratorRest(int r18, com.sun.tools.javac.tree.JCTree.JCModifiers r19, com.sun.tools.javac.tree.JCTree.JCExpression r20, com.sun.tools.javac.util.Name r21, com.sun.tools.javac.util.List<com.sun.tools.javac.tree.JCTree.JCTypeParameter> r22, boolean r23, boolean r24, com.sun.tools.javac.parser.Tokens.Comment r25) throws java.lang.Throwable {
        /*
            Method dump skipped, instruction units count: 201
            To view this dump change 'Code comments level' option to 'DEBUG'
        */
        throw new UnsupportedOperationException("Method not decompiled: com.sun.tools.javac.parser.JavacParser.methodDeclaratorRest(int, com.sun.tools.javac.tree.JCTree$JCModifiers, com.sun.tools.javac.tree.JCTree$JCExpression, com.sun.tools.javac.util.Name, com.sun.tools.javac.util.List, boolean, boolean, com.sun.tools.javac.parser.Tokens$Comment):com.sun.tools.javac.tree.JCTree");
    }

    List<JCTree.JCExpression> qualidentList() {
        ListBuffer<JCTree.JCExpression> ts = new ListBuffer<>();
        List<JCTree.JCAnnotation> typeAnnos = typeAnnotationsOpt();
        JCTree.JCExpression qi = qualident(true);
        if (!typeAnnos.isEmpty()) {
            JCTree.JCExpression at = insertAnnotationsToMostInner(qi, typeAnnos, false);
            ts.append(at);
        } else {
            ts.append(qi);
        }
        while (this.token.kind == Tokens.TokenKind.COMMA) {
            nextToken();
            List<JCTree.JCAnnotation> typeAnnos2 = typeAnnotationsOpt();
            JCTree.JCExpression qi2 = qualident(true);
            if (!typeAnnos2.isEmpty()) {
                JCTree.JCExpression at2 = insertAnnotationsToMostInner(qi2, typeAnnos2, false);
                ts.append(at2);
            } else {
                ts.append(qi2);
            }
        }
        return ts.toList();
    }

    List<JCTree.JCTypeParameter> typeParametersOpt() {
        if (this.token.kind == Tokens.TokenKind.LT) {
            checkGenerics();
            ListBuffer<JCTree.JCTypeParameter> typarams = new ListBuffer<>();
            nextToken();
            typarams.append(typeParameter());
            while (this.token.kind == Tokens.TokenKind.COMMA) {
                nextToken();
                typarams.append(typeParameter());
            }
            accept(Tokens.TokenKind.GT);
            return typarams.toList();
        }
        return List.nil();
    }

    JCTree.JCTypeParameter typeParameter() {
        int pos = this.token.pos;
        List<JCTree.JCAnnotation> annos = typeAnnotationsOpt();
        Name name = ident();
        ListBuffer<JCTree.JCExpression> bounds = new ListBuffer<>();
        if (this.token.kind == Tokens.TokenKind.EXTENDS) {
            nextToken();
            bounds.append(parseType());
            while (this.token.kind == Tokens.TokenKind.AMP) {
                nextToken();
                bounds.append(parseType());
            }
        }
        return (JCTree.JCTypeParameter) toP(this.F.at(pos).TypeParameter(name, bounds.toList(), annos));
    }

    List<JCTree.JCVariableDecl> formalParameters() {
        return formalParameters(false);
    }

    List<JCTree.JCVariableDecl> formalParameters(boolean lambdaParameters) {
        ListBuffer<JCTree.JCVariableDecl> params = new ListBuffer<>();
        accept(Tokens.TokenKind.LPAREN);
        if (this.token.kind != Tokens.TokenKind.RPAREN) {
            this.allowThisIdent = true;
            JCTree.JCVariableDecl lastParam = formalParameter(lambdaParameters);
            if (lastParam.nameexpr != null) {
                this.receiverParam = lastParam;
            } else {
                params.append(lastParam);
            }
            this.allowThisIdent = false;
            while ((lastParam.mods.flags & Flags.VARARGS) == 0 && this.token.kind == Tokens.TokenKind.COMMA) {
                nextToken();
                JCTree.JCVariableDecl jCVariableDeclFormalParameter = formalParameter(lambdaParameters);
                lastParam = jCVariableDeclFormalParameter;
                params.append(jCVariableDeclFormalParameter);
            }
        }
        accept(Tokens.TokenKind.RPAREN);
        return params.toList();
    }

    List<JCTree.JCVariableDecl> implicitParameters(boolean hasParens) {
        if (hasParens) {
            accept(Tokens.TokenKind.LPAREN);
        }
        ListBuffer<JCTree.JCVariableDecl> params = new ListBuffer<>();
        if (this.token.kind != Tokens.TokenKind.RPAREN && this.token.kind != Tokens.TokenKind.ARROW) {
            params.append(implicitParameter());
            while (this.token.kind == Tokens.TokenKind.COMMA) {
                nextToken();
                params.append(implicitParameter());
            }
        }
        if (hasParens) {
            accept(Tokens.TokenKind.RPAREN);
        }
        return params.toList();
    }

    JCTree.JCModifiers optFinal(long flags) {
        JCTree.JCModifiers mods = modifiersOpt();
        checkNoMods(mods.flags & (-131089));
        mods.flags |= flags;
        return mods;
    }

    private JCTree.JCExpression insertAnnotationsToMostInner(JCTree.JCExpression type, List<JCTree.JCAnnotation> annos, boolean createNewLevel) {
        int origEndPos = getEndPos(type);
        JCTree.JCExpression mostInnerType = type;
        JCTree.JCArrayTypeTree mostInnerArrayType = null;
        while (TreeInfo.typeIn(mostInnerType).hasTag(JCTree.Tag.TYPEARRAY)) {
            mostInnerArrayType = (JCTree.JCArrayTypeTree) TreeInfo.typeIn(mostInnerType);
            mostInnerType = mostInnerArrayType.elemtype;
        }
        if (createNewLevel) {
            mostInnerType = (JCTree.JCExpression) to(this.F.at(this.token.pos).TypeArray(mostInnerType));
        }
        JCTree.JCExpression mostInnerTypeToReturn = mostInnerType;
        if (annos.nonEmpty()) {
            JCTree.JCExpression lastToModify = mostInnerType;
            while (true) {
                if (!TreeInfo.typeIn(mostInnerType).hasTag(JCTree.Tag.SELECT) && !TreeInfo.typeIn(mostInnerType).hasTag(JCTree.Tag.TYPEAPPLY)) {
                    break;
                }
                while (TreeInfo.typeIn(mostInnerType).hasTag(JCTree.Tag.SELECT)) {
                    lastToModify = mostInnerType;
                    mostInnerType = ((JCTree.JCFieldAccess) TreeInfo.typeIn(mostInnerType)).getExpression();
                }
                while (TreeInfo.typeIn(mostInnerType).hasTag(JCTree.Tag.TYPEAPPLY)) {
                    lastToModify = mostInnerType;
                    mostInnerType = ((JCTree.JCTypeApply) TreeInfo.typeIn(mostInnerType)).clazz;
                }
            }
            JCTree.JCExpression mostInnerType2 = this.F.at(annos.head.pos).AnnotatedType(annos, mostInnerType);
            if (TreeInfo.typeIn(lastToModify).hasTag(JCTree.Tag.TYPEAPPLY)) {
                ((JCTree.JCTypeApply) TreeInfo.typeIn(lastToModify)).clazz = mostInnerType2;
            } else if (TreeInfo.typeIn(lastToModify).hasTag(JCTree.Tag.SELECT)) {
                ((JCTree.JCFieldAccess) TreeInfo.typeIn(lastToModify)).selected = mostInnerType2;
            } else {
                mostInnerTypeToReturn = mostInnerType2;
            }
        }
        if (mostInnerArrayType == null) {
            return mostInnerTypeToReturn;
        }
        mostInnerArrayType.elemtype = mostInnerTypeToReturn;
        storeEnd(type, origEndPos);
        return type;
    }

    protected JCTree.JCVariableDecl formalParameter() {
        return formalParameter(false);
    }

    protected JCTree.JCVariableDecl formalParameter(boolean lambdaParameter) {
        JCTree.JCModifiers mods = optFinal(8589934592L);
        this.permitTypeAnnotationsPushBack = true;
        JCTree.JCExpression type = parseType();
        this.permitTypeAnnotationsPushBack = false;
        if (this.token.kind == Tokens.TokenKind.ELLIPSIS) {
            List<JCTree.JCAnnotation> varargsAnnos = this.typeAnnotationsPushedBack;
            this.typeAnnotationsPushedBack = List.nil();
            checkVarargs();
            mods.flags |= Flags.VARARGS;
            type = insertAnnotationsToMostInner(type, varargsAnnos, true);
            nextToken();
        } else {
            if (this.typeAnnotationsPushedBack.nonEmpty()) {
                reportSyntaxError(this.typeAnnotationsPushedBack.head.pos, "illegal.start.of.type", new Object[0]);
            }
            this.typeAnnotationsPushedBack = List.nil();
        }
        return variableDeclaratorId(mods, type, lambdaParameter);
    }

    protected JCTree.JCVariableDecl implicitParameter() {
        JCTree.JCModifiers mods = this.F.at(this.token.pos).Modifiers(8589934592L);
        return variableDeclaratorId(mods, null, true);
    }

    void error(int pos, String key, Object... args) {
        this.log.error(JCDiagnostic.DiagnosticFlag.SYNTAX, pos, key, args);
    }

    void error(JCDiagnostic.DiagnosticPosition pos, String key, Object... args) {
        this.log.error(JCDiagnostic.DiagnosticFlag.SYNTAX, pos, key, args);
    }

    void warning(int pos, String key, Object... args) {
        this.log.warning(pos, key, args);
    }

    protected JCTree.JCExpression checkExprStat(JCTree.JCExpression t) {
        if (!TreeInfo.isExpressionStatement(t)) {
            JCTree.JCExpression ret = this.F.at(t.pos).Erroneous(List.of(t));
            error(ret, "not.stmt", new Object[0]);
            return ret;
        }
        return t;
    }

    static int prec(Tokens.TokenKind token) {
        JCTree.Tag oc = optag(token);
        if (oc != JCTree.Tag.NO_TAG) {
            return TreeInfo.opPrec(oc);
        }
        return -1;
    }

    static int earlier(int pos1, int pos2) {
        if (pos1 == -1) {
            return pos2;
        }
        if (pos2 == -1) {
            return pos1;
        }
        return pos1 < pos2 ? pos1 : pos2;
    }

    static JCTree.Tag optag(Tokens.TokenKind token) {
        switch (token) {
            case LT:
                return JCTree.Tag.LT;
            case PLUSEQ:
                return JCTree.Tag.PLUS_ASG;
            case SUBEQ:
                return JCTree.Tag.MINUS_ASG;
            case STAREQ:
                return JCTree.Tag.MUL_ASG;
            case SLASHEQ:
                return JCTree.Tag.DIV_ASG;
            case PERCENTEQ:
                return JCTree.Tag.MOD_ASG;
            case AMPEQ:
                return JCTree.Tag.BITAND_ASG;
            case BAREQ:
                return JCTree.Tag.BITOR_ASG;
            case CARETEQ:
                return JCTree.Tag.BITXOR_ASG;
            case LTLTEQ:
                return JCTree.Tag.SL_ASG;
            case GTGTEQ:
                return JCTree.Tag.SR_ASG;
            case GTGTGTEQ:
                return JCTree.Tag.USR_ASG;
            case PLUS:
                return JCTree.Tag.PLUS;
            case SUB:
                return JCTree.Tag.MINUS;
            case GTGTGT:
                return JCTree.Tag.USR;
            case GTGT:
                return JCTree.Tag.SR;
            case GT:
                return JCTree.Tag.GT;
            case AMP:
                return JCTree.Tag.BITAND;
            case GTEQ:
                return JCTree.Tag.GE;
            case BARBAR:
                return JCTree.Tag.OR;
            case AMPAMP:
                return JCTree.Tag.AND;
            case BAR:
                return JCTree.Tag.BITOR;
            case CARET:
                return JCTree.Tag.BITXOR;
            case EQEQ:
                return JCTree.Tag.EQ;
            case BANGEQ:
                return JCTree.Tag.NE;
            case LTEQ:
                return JCTree.Tag.LE;
            case LTLT:
                return JCTree.Tag.SL;
            case STAR:
                return JCTree.Tag.MUL;
            case SLASH:
                return JCTree.Tag.DIV;
            case PERCENT:
                return JCTree.Tag.MOD;
            case INSTANCEOF:
                return JCTree.Tag.TYPETEST;
            default:
                return JCTree.Tag.NO_TAG;
        }
    }

    static JCTree.Tag unoptag(Tokens.TokenKind token) {
        switch (token) {
            case PLUSPLUS:
                return JCTree.Tag.PREINC;
            case SUBSUB:
                return JCTree.Tag.PREDEC;
            case BANG:
                return JCTree.Tag.NOT;
            case TILDE:
                return JCTree.Tag.COMPL;
            case PLUS:
                return JCTree.Tag.POS;
            case SUB:
                return JCTree.Tag.NEG;
            default:
                return JCTree.Tag.NO_TAG;
        }
    }

    static TypeTag typetag(Tokens.TokenKind token) {
        switch (token) {
            case BYTE:
                return TypeTag.BYTE;
            case SHORT:
                return TypeTag.SHORT;
            case CHAR:
                return TypeTag.CHAR;
            case INT:
                return TypeTag.INT;
            case LONG:
                return TypeTag.LONG;
            case FLOAT:
                return TypeTag.FLOAT;
            case DOUBLE:
                return TypeTag.DOUBLE;
            case BOOLEAN:
                return TypeTag.BOOLEAN;
            default:
                return TypeTag.NONE;
        }
    }

    void checkGenerics() {
        if (!this.allowGenerics) {
            error(this.token.pos, "generics.not.supported.in.source", this.source.name);
            this.allowGenerics = true;
        }
    }

    void checkVarargs() {
        if (!this.allowVarargs) {
            error(this.token.pos, "varargs.not.supported.in.source", this.source.name);
            this.allowVarargs = true;
        }
    }

    void checkForeach() {
        if (!this.allowForeach) {
            error(this.token.pos, "foreach.not.supported.in.source", this.source.name);
            this.allowForeach = true;
        }
    }

    void checkStaticImports() {
        if (!this.allowStaticImport) {
            error(this.token.pos, "static.import.not.supported.in.source", this.source.name);
            this.allowStaticImport = true;
        }
    }

    void checkAnnotations() {
        if (!this.allowAnnotations) {
            error(this.token.pos, "annotations.not.supported.in.source", this.source.name);
            this.allowAnnotations = true;
        }
    }

    void checkDiamond() {
        if (!this.allowDiamond) {
            error(this.token.pos, "diamond.not.supported.in.source", this.source.name);
            this.allowDiamond = true;
        }
    }

    void checkMulticatch() {
        if (!this.allowMulticatch) {
            error(this.token.pos, "multicatch.not.supported.in.source", this.source.name);
            this.allowMulticatch = true;
        }
    }

    void checkTryWithResources() {
        if (!this.allowTWR) {
            error(this.token.pos, "try.with.resources.not.supported.in.source", this.source.name);
            this.allowTWR = true;
        }
    }

    void checkLambda() {
        if (!this.allowLambda) {
            this.log.error(this.token.pos, "lambda.not.supported.in.source", this.source.name);
            this.allowLambda = true;
        }
    }

    void checkMethodReferences() {
        if (!this.allowMethodReferences) {
            this.log.error(this.token.pos, "method.references.not.supported.in.source", this.source.name);
            this.allowMethodReferences = true;
        }
    }

    void checkDefaultMethods() {
        if (!this.allowDefaultMethods) {
            this.log.error(this.token.pos, "default.methods.not.supported.in.source", this.source.name);
            this.allowDefaultMethods = true;
        }
    }

    void checkIntersectionTypesInCast() {
        if (!this.allowIntersectionTypesInCast) {
            this.log.error(this.token.pos, "intersection.types.in.cast.not.supported.in.source", this.source.name);
            this.allowIntersectionTypesInCast = true;
        }
    }

    void checkStaticInterfaceMethods() {
        if (!this.allowStaticInterfaceMethods) {
            this.log.error(this.token.pos, "static.intf.methods.not.supported.in.source", this.source.name);
            this.allowStaticInterfaceMethods = true;
        }
    }

    void checkTypeAnnotations() {
        if (!this.allowTypeAnnotations) {
            this.log.error(this.token.pos, "type.annotations.not.supported.in.source", this.source.name);
            this.allowTypeAnnotations = true;
        }
    }

    void checkAnnotationsAfterTypeParams(int pos) {
        if (!this.allowAnnotationsAfterTypeParams) {
            this.log.error(pos, "annotations.after.type.params.not.supported.in.source", this.source.name);
            this.allowAnnotationsAfterTypeParams = true;
        }
    }

    protected static class SimpleEndPosTable extends AbstractEndPosTable {
        private final IntHashTable endPosMap;

        SimpleEndPosTable(JavacParser parser) {
            super(parser);
            this.endPosMap = new IntHashTable();
        }

        @Override // com.sun.tools.javac.tree.EndPosTable
        public void storeEnd(JCTree tree, int endpos) {
            this.endPosMap.putAtIndex(tree, this.errorEndPos > endpos ? this.errorEndPos : endpos, this.endPosMap.lookup(tree));
        }

        @Override // com.sun.tools.javac.parser.JavacParser.AbstractEndPosTable
        protected <T extends JCTree> T to(T t) {
            storeEnd(t, this.parser.token.endPos);
            return t;
        }

        @Override // com.sun.tools.javac.parser.JavacParser.AbstractEndPosTable
        protected <T extends JCTree> T toP(T t) {
            storeEnd(t, this.parser.S.prevToken().endPos);
            return t;
        }

        @Override // com.sun.tools.javac.tree.EndPosTable
        public int getEndPos(JCTree tree) {
            int value = this.endPosMap.getFromIndex(this.endPosMap.lookup(tree));
            if (value == -1) {
                return -1;
            }
            return value;
        }

        @Override // com.sun.tools.javac.tree.EndPosTable
        public int replaceTree(JCTree oldTree, JCTree newTree) {
            int pos = this.endPosMap.remove(oldTree);
            if (pos == -1) {
                return -1;
            }
            storeEnd(newTree, pos);
            return pos;
        }
    }

    protected static class EmptyEndPosTable extends AbstractEndPosTable {
        EmptyEndPosTable(JavacParser parser) {
            super(parser);
        }

        @Override // com.sun.tools.javac.tree.EndPosTable
        public void storeEnd(JCTree tree, int endpos) {
        }

        @Override // com.sun.tools.javac.parser.JavacParser.AbstractEndPosTable
        protected <T extends JCTree> T to(T t) {
            return t;
        }

        @Override // com.sun.tools.javac.parser.JavacParser.AbstractEndPosTable
        protected <T extends JCTree> T toP(T t) {
            return t;
        }

        @Override // com.sun.tools.javac.tree.EndPosTable
        public int getEndPos(JCTree tree) {
            return -1;
        }

        @Override // com.sun.tools.javac.tree.EndPosTable
        public int replaceTree(JCTree oldTree, JCTree newTree) {
            return -1;
        }
    }

    protected static abstract class AbstractEndPosTable implements EndPosTable {
        protected int errorEndPos;
        protected JavacParser parser;

        protected abstract <T extends JCTree> T to(T t);

        protected abstract <T extends JCTree> T toP(T t);

        public AbstractEndPosTable(JavacParser parser) {
            this.parser = parser;
        }

        protected void setErrorEndPos(int errPos) {
            if (errPos > this.errorEndPos) {
                this.errorEndPos = errPos;
            }
        }

        protected void setParser(JavacParser parser) {
            this.parser = parser;
        }
    }
}
