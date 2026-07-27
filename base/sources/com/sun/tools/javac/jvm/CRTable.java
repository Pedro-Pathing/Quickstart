package com.sun.tools.javac.jvm;

import com.sun.tools.javac.tree.EndPosTable;
import com.sun.tools.javac.tree.JCTree;
import com.sun.tools.javac.tree.TreeInfo;
import com.sun.tools.javac.util.Assert;
import com.sun.tools.javac.util.ByteBuffer;
import com.sun.tools.javac.util.List;
import com.sun.tools.javac.util.ListBuffer;
import com.sun.tools.javac.util.Log;
import com.sun.tools.javac.util.Position;
import java.util.HashMap;
import java.util.Map;

/* JADX INFO: loaded from: classes.dex */
public class CRTable implements CRTFlags {
    private EndPosTable endPosTable;
    JCTree.JCMethodDecl methodTree;
    private final boolean crtDebug = false;
    private ListBuffer<CRTEntry> entries = new ListBuffer<>();
    private Map<Object, SourceRange> positions = new HashMap();

    public CRTable(JCTree.JCMethodDecl tree, EndPosTable endPosTable) {
        this.methodTree = tree;
        this.endPosTable = endPosTable;
    }

    public void put(Object tree, int flags, int startPc, int endPc) {
        this.entries.append(new CRTEntry(tree, flags, startPc, endPc));
    }

    /* JADX WARN: Multi-variable type inference failed */
    public int writeCRT(ByteBuffer databuf, Position.LineMap lineMap, Log log) {
        int startPos;
        int endPos;
        int crtEntries = 0;
        new SourceComputer().csp(this.methodTree);
        for (List list = this.entries.toList(); list.nonEmpty(); list = list.tail) {
            CRTEntry entry = (CRTEntry) list.head;
            if (entry.startPc != entry.endPc) {
                SourceRange pos = this.positions.get(entry.tree);
                Assert.checkNonNull(pos, "CRT: tree source positions are undefined");
                if (pos.startPos != -1 && pos.endPos != -1 && (startPos = encodePosition(pos.startPos, lineMap, log)) != -1 && (endPos = encodePosition(pos.endPos, lineMap, log)) != -1) {
                    databuf.appendChar(entry.startPc);
                    databuf.appendChar(entry.endPc - 1);
                    databuf.appendInt(startPos);
                    databuf.appendInt(endPos);
                    databuf.appendChar(entry.flags);
                    crtEntries++;
                }
            }
        }
        return crtEntries;
    }

    public int length() {
        return this.entries.length();
    }

    private String getTypes(int flags) {
        String types = (flags & 1) != 0 ? " CRT_STATEMENT" : "";
        if ((flags & 2) != 0) {
            types = types + " CRT_BLOCK";
        }
        if ((flags & 4) != 0) {
            types = types + " CRT_ASSIGNMENT";
        }
        if ((flags & 8) != 0) {
            types = types + " CRT_FLOW_CONTROLLER";
        }
        if ((flags & 16) != 0) {
            types = types + " CRT_FLOW_TARGET";
        }
        if ((flags & 32) != 0) {
            types = types + " CRT_INVOKE";
        }
        if ((flags & 64) != 0) {
            types = types + " CRT_CREATE";
        }
        if ((flags & 128) != 0) {
            types = types + " CRT_BRANCH_TRUE";
        }
        return (flags & 256) != 0 ? types + " CRT_BRANCH_FALSE" : types;
    }

    private int encodePosition(int pos, Position.LineMap lineMap, Log log) {
        int line = lineMap.getLineNumber(pos);
        int col = lineMap.getColumnNumber(pos);
        int new_pos = Position.encodePosition(line, col);
        if (new_pos == -1) {
            log.warning(pos, "position.overflow", Integer.valueOf(line));
        }
        return new_pos;
    }

    class SourceComputer extends JCTree.Visitor {
        SourceRange result;

        SourceComputer() {
        }

        public SourceRange csp(JCTree tree) {
            if (tree == null) {
                return null;
            }
            tree.accept(this);
            if (this.result != null) {
                CRTable.this.positions.put(tree, this.result);
            }
            return this.result;
        }

        /* JADX WARN: Multi-variable type inference failed */
        public SourceRange csp(List<? extends JCTree> trees) {
            if (trees == null || !trees.nonEmpty()) {
                return null;
            }
            SourceRange list_sr = new SourceRange();
            for (List list = trees; list.nonEmpty(); list = list.tail) {
                list_sr.mergeWith(csp((JCTree) list.head));
            }
            CRTable.this.positions.put(trees, list_sr);
            return list_sr;
        }

        /* JADX WARN: Multi-variable type inference failed */
        public SourceRange cspCases(List<JCTree.JCCase> trees) {
            if (trees == null || !trees.nonEmpty()) {
                return null;
            }
            SourceRange list_sr = new SourceRange();
            for (List list = trees; list.nonEmpty(); list = list.tail) {
                list_sr.mergeWith(csp((JCTree) list.head));
            }
            CRTable.this.positions.put(trees, list_sr);
            return list_sr;
        }

        /* JADX WARN: Multi-variable type inference failed */
        public SourceRange cspCatchers(List<JCTree.JCCatch> trees) {
            if (trees == null || !trees.nonEmpty()) {
                return null;
            }
            SourceRange list_sr = new SourceRange();
            for (List list = trees; list.nonEmpty(); list = list.tail) {
                list_sr.mergeWith(csp((JCTree) list.head));
            }
            CRTable.this.positions.put(trees, list_sr);
            return list_sr;
        }

        @Override // com.sun.tools.javac.tree.JCTree.Visitor
        public void visitMethodDef(JCTree.JCMethodDecl tree) {
            SourceRange sr = new SourceRange(startPos(tree), endPos(tree));
            sr.mergeWith(csp(tree.body));
            this.result = sr;
        }

        @Override // com.sun.tools.javac.tree.JCTree.Visitor
        public void visitVarDef(JCTree.JCVariableDecl tree) {
            SourceRange sr = new SourceRange(startPos(tree), endPos(tree));
            csp(tree.vartype);
            sr.mergeWith(csp(tree.init));
            this.result = sr;
        }

        @Override // com.sun.tools.javac.tree.JCTree.Visitor
        public void visitSkip(JCTree.JCSkip tree) {
            SourceRange sr = new SourceRange(startPos(tree), startPos(tree));
            this.result = sr;
        }

        @Override // com.sun.tools.javac.tree.JCTree.Visitor
        public void visitBlock(JCTree.JCBlock tree) {
            SourceRange sr = new SourceRange(startPos(tree), endPos(tree));
            csp(tree.stats);
            this.result = sr;
        }

        @Override // com.sun.tools.javac.tree.JCTree.Visitor
        public void visitDoLoop(JCTree.JCDoWhileLoop tree) {
            SourceRange sr = new SourceRange(startPos(tree), endPos(tree));
            sr.mergeWith(csp(tree.body));
            sr.mergeWith(csp(tree.cond));
            this.result = sr;
        }

        @Override // com.sun.tools.javac.tree.JCTree.Visitor
        public void visitWhileLoop(JCTree.JCWhileLoop tree) {
            SourceRange sr = new SourceRange(startPos(tree), endPos(tree));
            sr.mergeWith(csp(tree.cond));
            sr.mergeWith(csp(tree.body));
            this.result = sr;
        }

        @Override // com.sun.tools.javac.tree.JCTree.Visitor
        public void visitForLoop(JCTree.JCForLoop tree) {
            SourceRange sr = new SourceRange(startPos(tree), endPos(tree));
            sr.mergeWith(csp(tree.init));
            sr.mergeWith(csp(tree.cond));
            sr.mergeWith(csp(tree.step));
            sr.mergeWith(csp(tree.body));
            this.result = sr;
        }

        @Override // com.sun.tools.javac.tree.JCTree.Visitor
        public void visitForeachLoop(JCTree.JCEnhancedForLoop tree) {
            SourceRange sr = new SourceRange(startPos(tree), endPos(tree));
            sr.mergeWith(csp(tree.var));
            sr.mergeWith(csp(tree.expr));
            sr.mergeWith(csp(tree.body));
            this.result = sr;
        }

        @Override // com.sun.tools.javac.tree.JCTree.Visitor
        public void visitLabelled(JCTree.JCLabeledStatement tree) {
            SourceRange sr = new SourceRange(startPos(tree), endPos(tree));
            sr.mergeWith(csp(tree.body));
            this.result = sr;
        }

        @Override // com.sun.tools.javac.tree.JCTree.Visitor
        public void visitSwitch(JCTree.JCSwitch tree) {
            SourceRange sr = new SourceRange(startPos(tree), endPos(tree));
            sr.mergeWith(csp(tree.selector));
            sr.mergeWith(cspCases(tree.cases));
            this.result = sr;
        }

        @Override // com.sun.tools.javac.tree.JCTree.Visitor
        public void visitCase(JCTree.JCCase tree) {
            SourceRange sr = new SourceRange(startPos(tree), endPos(tree));
            sr.mergeWith(csp(tree.pat));
            sr.mergeWith(csp(tree.stats));
            this.result = sr;
        }

        @Override // com.sun.tools.javac.tree.JCTree.Visitor
        public void visitSynchronized(JCTree.JCSynchronized tree) {
            SourceRange sr = new SourceRange(startPos(tree), endPos(tree));
            sr.mergeWith(csp(tree.lock));
            sr.mergeWith(csp(tree.body));
            this.result = sr;
        }

        @Override // com.sun.tools.javac.tree.JCTree.Visitor
        public void visitTry(JCTree.JCTry tree) {
            SourceRange sr = new SourceRange(startPos(tree), endPos(tree));
            sr.mergeWith(csp(tree.resources));
            sr.mergeWith(csp(tree.body));
            sr.mergeWith(cspCatchers(tree.catchers));
            sr.mergeWith(csp(tree.finalizer));
            this.result = sr;
        }

        @Override // com.sun.tools.javac.tree.JCTree.Visitor
        public void visitCatch(JCTree.JCCatch tree) {
            SourceRange sr = new SourceRange(startPos(tree), endPos(tree));
            sr.mergeWith(csp(tree.param));
            sr.mergeWith(csp(tree.body));
            this.result = sr;
        }

        @Override // com.sun.tools.javac.tree.JCTree.Visitor
        public void visitConditional(JCTree.JCConditional tree) {
            SourceRange sr = new SourceRange(startPos(tree), endPos(tree));
            sr.mergeWith(csp(tree.cond));
            sr.mergeWith(csp(tree.truepart));
            sr.mergeWith(csp(tree.falsepart));
            this.result = sr;
        }

        @Override // com.sun.tools.javac.tree.JCTree.Visitor
        public void visitIf(JCTree.JCIf tree) {
            SourceRange sr = new SourceRange(startPos(tree), endPos(tree));
            sr.mergeWith(csp(tree.cond));
            sr.mergeWith(csp(tree.thenpart));
            sr.mergeWith(csp(tree.elsepart));
            this.result = sr;
        }

        @Override // com.sun.tools.javac.tree.JCTree.Visitor
        public void visitExec(JCTree.JCExpressionStatement tree) {
            SourceRange sr = new SourceRange(startPos(tree), endPos(tree));
            sr.mergeWith(csp(tree.expr));
            this.result = sr;
        }

        @Override // com.sun.tools.javac.tree.JCTree.Visitor
        public void visitBreak(JCTree.JCBreak tree) {
            SourceRange sr = new SourceRange(startPos(tree), endPos(tree));
            this.result = sr;
        }

        @Override // com.sun.tools.javac.tree.JCTree.Visitor
        public void visitContinue(JCTree.JCContinue tree) {
            SourceRange sr = new SourceRange(startPos(tree), endPos(tree));
            this.result = sr;
        }

        @Override // com.sun.tools.javac.tree.JCTree.Visitor
        public void visitReturn(JCTree.JCReturn tree) {
            SourceRange sr = new SourceRange(startPos(tree), endPos(tree));
            sr.mergeWith(csp(tree.expr));
            this.result = sr;
        }

        @Override // com.sun.tools.javac.tree.JCTree.Visitor
        public void visitThrow(JCTree.JCThrow tree) {
            SourceRange sr = new SourceRange(startPos(tree), endPos(tree));
            sr.mergeWith(csp(tree.expr));
            this.result = sr;
        }

        @Override // com.sun.tools.javac.tree.JCTree.Visitor
        public void visitAssert(JCTree.JCAssert tree) {
            SourceRange sr = new SourceRange(startPos(tree), endPos(tree));
            sr.mergeWith(csp(tree.cond));
            sr.mergeWith(csp(tree.detail));
            this.result = sr;
        }

        @Override // com.sun.tools.javac.tree.JCTree.Visitor
        public void visitApply(JCTree.JCMethodInvocation tree) {
            SourceRange sr = new SourceRange(startPos(tree), endPos(tree));
            sr.mergeWith(csp(tree.meth));
            sr.mergeWith(csp(tree.args));
            this.result = sr;
        }

        @Override // com.sun.tools.javac.tree.JCTree.Visitor
        public void visitNewClass(JCTree.JCNewClass tree) {
            SourceRange sr = new SourceRange(startPos(tree), endPos(tree));
            sr.mergeWith(csp(tree.encl));
            sr.mergeWith(csp(tree.clazz));
            sr.mergeWith(csp(tree.args));
            sr.mergeWith(csp(tree.def));
            this.result = sr;
        }

        @Override // com.sun.tools.javac.tree.JCTree.Visitor
        public void visitNewArray(JCTree.JCNewArray tree) {
            SourceRange sr = new SourceRange(startPos(tree), endPos(tree));
            sr.mergeWith(csp(tree.elemtype));
            sr.mergeWith(csp(tree.dims));
            sr.mergeWith(csp(tree.elems));
            this.result = sr;
        }

        @Override // com.sun.tools.javac.tree.JCTree.Visitor
        public void visitParens(JCTree.JCParens tree) {
            SourceRange sr = new SourceRange(startPos(tree), endPos(tree));
            sr.mergeWith(csp(tree.expr));
            this.result = sr;
        }

        @Override // com.sun.tools.javac.tree.JCTree.Visitor
        public void visitAssign(JCTree.JCAssign tree) {
            SourceRange sr = new SourceRange(startPos(tree), endPos(tree));
            sr.mergeWith(csp(tree.lhs));
            sr.mergeWith(csp(tree.rhs));
            this.result = sr;
        }

        @Override // com.sun.tools.javac.tree.JCTree.Visitor
        public void visitAssignop(JCTree.JCAssignOp tree) {
            SourceRange sr = new SourceRange(startPos(tree), endPos(tree));
            sr.mergeWith(csp(tree.lhs));
            sr.mergeWith(csp(tree.rhs));
            this.result = sr;
        }

        @Override // com.sun.tools.javac.tree.JCTree.Visitor
        public void visitUnary(JCTree.JCUnary tree) {
            SourceRange sr = new SourceRange(startPos(tree), endPos(tree));
            sr.mergeWith(csp(tree.arg));
            this.result = sr;
        }

        @Override // com.sun.tools.javac.tree.JCTree.Visitor
        public void visitBinary(JCTree.JCBinary tree) {
            SourceRange sr = new SourceRange(startPos(tree), endPos(tree));
            sr.mergeWith(csp(tree.lhs));
            sr.mergeWith(csp(tree.rhs));
            this.result = sr;
        }

        @Override // com.sun.tools.javac.tree.JCTree.Visitor
        public void visitTypeCast(JCTree.JCTypeCast tree) {
            SourceRange sr = new SourceRange(startPos(tree), endPos(tree));
            sr.mergeWith(csp(tree.clazz));
            sr.mergeWith(csp(tree.expr));
            this.result = sr;
        }

        @Override // com.sun.tools.javac.tree.JCTree.Visitor
        public void visitTypeTest(JCTree.JCInstanceOf tree) {
            SourceRange sr = new SourceRange(startPos(tree), endPos(tree));
            sr.mergeWith(csp(tree.expr));
            sr.mergeWith(csp(tree.clazz));
            this.result = sr;
        }

        @Override // com.sun.tools.javac.tree.JCTree.Visitor
        public void visitIndexed(JCTree.JCArrayAccess tree) {
            SourceRange sr = new SourceRange(startPos(tree), endPos(tree));
            sr.mergeWith(csp(tree.indexed));
            sr.mergeWith(csp(tree.index));
            this.result = sr;
        }

        @Override // com.sun.tools.javac.tree.JCTree.Visitor
        public void visitSelect(JCTree.JCFieldAccess tree) {
            SourceRange sr = new SourceRange(startPos(tree), endPos(tree));
            sr.mergeWith(csp(tree.selected));
            this.result = sr;
        }

        @Override // com.sun.tools.javac.tree.JCTree.Visitor
        public void visitIdent(JCTree.JCIdent tree) {
            SourceRange sr = new SourceRange(startPos(tree), endPos(tree));
            this.result = sr;
        }

        @Override // com.sun.tools.javac.tree.JCTree.Visitor
        public void visitLiteral(JCTree.JCLiteral tree) {
            SourceRange sr = new SourceRange(startPos(tree), endPos(tree));
            this.result = sr;
        }

        @Override // com.sun.tools.javac.tree.JCTree.Visitor
        public void visitTypeIdent(JCTree.JCPrimitiveTypeTree tree) {
            SourceRange sr = new SourceRange(startPos(tree), endPos(tree));
            this.result = sr;
        }

        @Override // com.sun.tools.javac.tree.JCTree.Visitor
        public void visitTypeArray(JCTree.JCArrayTypeTree tree) {
            SourceRange sr = new SourceRange(startPos(tree), endPos(tree));
            sr.mergeWith(csp(tree.elemtype));
            this.result = sr;
        }

        @Override // com.sun.tools.javac.tree.JCTree.Visitor
        public void visitTypeApply(JCTree.JCTypeApply tree) {
            SourceRange sr = new SourceRange(startPos(tree), endPos(tree));
            sr.mergeWith(csp(tree.clazz));
            sr.mergeWith(csp(tree.arguments));
            this.result = sr;
        }

        @Override // com.sun.tools.javac.tree.JCTree.Visitor
        public void visitLetExpr(JCTree.LetExpr tree) {
            SourceRange sr = new SourceRange(startPos(tree), endPos(tree));
            sr.mergeWith(csp(tree.defs));
            sr.mergeWith(csp(tree.expr));
            this.result = sr;
        }

        @Override // com.sun.tools.javac.tree.JCTree.Visitor
        public void visitTypeParameter(JCTree.JCTypeParameter tree) {
            SourceRange sr = new SourceRange(startPos(tree), endPos(tree));
            sr.mergeWith(csp(tree.bounds));
            this.result = sr;
        }

        @Override // com.sun.tools.javac.tree.JCTree.Visitor
        public void visitWildcard(JCTree.JCWildcard tree) {
            this.result = null;
        }

        @Override // com.sun.tools.javac.tree.JCTree.Visitor
        public void visitErroneous(JCTree.JCErroneous tree) {
            this.result = null;
        }

        @Override // com.sun.tools.javac.tree.JCTree.Visitor
        public void visitTree(JCTree tree) {
            Assert.error();
        }

        public int startPos(JCTree tree) {
            if (tree == null) {
                return -1;
            }
            return TreeInfo.getStartPos(tree);
        }

        public int endPos(JCTree tree) {
            if (tree == null) {
                return -1;
            }
            return TreeInfo.getEndPos(tree, CRTable.this.endPosTable);
        }
    }

    static class CRTEntry {
        int endPc;
        int flags;
        int startPc;
        Object tree;

        CRTEntry(Object tree, int flags, int startPc, int endPc) {
            this.tree = tree;
            this.flags = flags;
            this.startPc = startPc;
            this.endPc = endPc;
        }
    }

    static class SourceRange {
        int endPos;
        int startPos;

        SourceRange() {
            this.startPos = -1;
            this.endPos = -1;
        }

        SourceRange(int startPos, int endPos) {
            this.startPos = startPos;
            this.endPos = endPos;
        }

        SourceRange mergeWith(SourceRange sr) {
            if (sr == null) {
                return this;
            }
            if (this.startPos == -1) {
                this.startPos = sr.startPos;
            } else if (sr.startPos != -1) {
                this.startPos = this.startPos < sr.startPos ? this.startPos : sr.startPos;
            }
            if (this.endPos == -1) {
                this.endPos = sr.endPos;
            } else if (sr.endPos != -1) {
                this.endPos = this.endPos > sr.endPos ? this.endPos : sr.endPos;
            }
            return this;
        }
    }
}
