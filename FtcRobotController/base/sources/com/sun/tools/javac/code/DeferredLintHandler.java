package com.sun.tools.javac.code;

import com.sun.tools.javac.tree.EndPosTable;
import com.sun.tools.javac.tree.JCTree;
import com.sun.tools.javac.util.Assert;
import com.sun.tools.javac.util.Context;
import com.sun.tools.javac.util.JCDiagnostic;
import com.sun.tools.javac.util.ListBuffer;
import java.util.HashMap;
import java.util.Map;

/* JADX INFO: loaded from: classes.dex */
public class DeferredLintHandler {
    private JCDiagnostic.DiagnosticPosition currentPos;
    private Map<JCDiagnostic.DiagnosticPosition, ListBuffer<LintLogger>> loggersQueue = new HashMap();
    protected static final Context.Key<DeferredLintHandler> deferredLintHandlerKey = new Context.Key<>();
    private static final JCDiagnostic.DiagnosticPosition IMMEDIATE_POSITION = new JCDiagnostic.DiagnosticPosition() { // from class: com.sun.tools.javac.code.DeferredLintHandler.1
        @Override // com.sun.tools.javac.util.JCDiagnostic.DiagnosticPosition
        public JCTree getTree() {
            Assert.error();
            return null;
        }

        @Override // com.sun.tools.javac.util.JCDiagnostic.DiagnosticPosition
        public int getStartPosition() {
            Assert.error();
            return -1;
        }

        @Override // com.sun.tools.javac.util.JCDiagnostic.DiagnosticPosition
        public int getPreferredPosition() {
            Assert.error();
            return -1;
        }

        @Override // com.sun.tools.javac.util.JCDiagnostic.DiagnosticPosition
        public int getEndPosition(EndPosTable endPosTable) {
            Assert.error();
            return -1;
        }
    };

    public interface LintLogger {
        void report();
    }

    public static DeferredLintHandler instance(Context context) {
        DeferredLintHandler instance = (DeferredLintHandler) context.get(deferredLintHandlerKey);
        if (instance == null) {
            return new DeferredLintHandler(context);
        }
        return instance;
    }

    protected DeferredLintHandler(Context context) {
        context.put(deferredLintHandlerKey, this);
        this.currentPos = IMMEDIATE_POSITION;
    }

    public void report(LintLogger logger) {
        if (this.currentPos == IMMEDIATE_POSITION) {
            logger.report();
            return;
        }
        ListBuffer<LintLogger> loggers = this.loggersQueue.get(this.currentPos);
        if (loggers == null) {
            Map<JCDiagnostic.DiagnosticPosition, ListBuffer<LintLogger>> map = this.loggersQueue;
            JCDiagnostic.DiagnosticPosition diagnosticPosition = this.currentPos;
            ListBuffer<LintLogger> listBuffer = new ListBuffer<>();
            loggers = listBuffer;
            map.put(diagnosticPosition, listBuffer);
        }
        loggers.append(logger);
    }

    public void flush(JCDiagnostic.DiagnosticPosition pos) {
        ListBuffer<LintLogger> loggers = this.loggersQueue.get(pos);
        if (loggers != null) {
            for (LintLogger lintLogger : loggers) {
                lintLogger.report();
            }
            this.loggersQueue.remove(pos);
        }
    }

    public JCDiagnostic.DiagnosticPosition setPos(JCDiagnostic.DiagnosticPosition currentPos) {
        JCDiagnostic.DiagnosticPosition prevPosition = this.currentPos;
        this.currentPos = currentPos;
        return prevPosition;
    }

    public JCDiagnostic.DiagnosticPosition immediate() {
        return setPos(IMMEDIATE_POSITION);
    }
}
