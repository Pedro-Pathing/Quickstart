package com.sun.tools.doclint;

import com.sun.source.doctree.DocTree;
import com.sun.source.tree.Tree;
import com.sun.tools.doclint.Env;
import com.sun.tools.javac.util.StringUtils;
import java.io.PrintWriter;
import java.text.MessageFormat;
import java.util.Comparator;
import java.util.HashMap;
import java.util.Locale;
import java.util.Map;
import java.util.MissingResourceException;
import java.util.ResourceBundle;
import java.util.Set;
import java.util.TreeMap;
import java.util.TreeSet;
import javax.tools.Diagnostic;
import org.firstinspires.ftc.onbotjava.OnBotJavaFileSystemUtils;

/* JADX INFO: loaded from: classes.dex */
public class Messages {
    ResourceBundle bundle;
    Env env;
    private final Options options;
    private final Stats stats;

    public enum Group {
        ACCESSIBILITY,
        HTML,
        MISSING,
        SYNTAX,
        REFERENCE;

        String optName() {
            return StringUtils.toLowerCase(name());
        }

        String notOptName() {
            return "-" + optName();
        }

        static boolean accepts(String opt) {
            for (Group g : values()) {
                if (opt.equals(g.optName())) {
                    return true;
                }
            }
            return false;
        }
    }

    Messages(Env env) {
        this.env = env;
        String name = getClass().getPackage().getName() + ".resources.doclint";
        this.bundle = ResourceBundle.getBundle(name, Locale.ENGLISH);
        this.stats = new Stats(this.bundle);
        this.options = new Options(this.stats);
    }

    void error(Group group, DocTree tree, String code, Object... args) {
        report(group, Diagnostic.Kind.ERROR, tree, code, args);
    }

    void warning(Group group, DocTree tree, String code, Object... args) {
        report(group, Diagnostic.Kind.WARNING, tree, code, args);
    }

    void setOptions(String opts) {
        this.options.setOptions(opts);
    }

    void setStatsEnabled(boolean b) {
        this.stats.setEnabled(b);
    }

    void reportStats(PrintWriter out) {
        this.stats.report(out);
    }

    protected void report(Group group, Diagnostic.Kind dkind, DocTree tree, String code, Object... args) {
        if (this.options.isEnabled(group, this.env.currAccess)) {
            String msg = code == null ? (String) args[0] : localize(code, args);
            this.env.trees.printMessage(dkind, msg, tree, this.env.currDocComment, this.env.currPath.getCompilationUnit());
            this.stats.record(group, dkind, code);
        }
    }

    protected void report(Group group, Diagnostic.Kind dkind, Tree tree, String code, Object... args) {
        if (this.options.isEnabled(group, this.env.currAccess)) {
            String msg = localize(code, args);
            this.env.trees.printMessage(dkind, msg, tree, this.env.currPath.getCompilationUnit());
            this.stats.record(group, dkind, code);
        }
    }

    String localize(String code, Object... args) {
        String msg = this.bundle.getString(code);
        if (msg == null) {
            StringBuilder sb = new StringBuilder();
            sb.append("message file broken: code=").append(code);
            if (args.length > 0) {
                sb.append(" arguments={0}");
                for (int i = 1; i < args.length; i++) {
                    sb.append(", {").append(i).append("}");
                }
            }
            msg = sb.toString();
        }
        return MessageFormat.format(msg, args);
    }

    static class Options {
        private static final String ALL = "all";
        Map<String, Env.AccessKind> map = new HashMap();
        private final Stats stats;

        static boolean isValidOptions(String opts) {
            for (String opt : opts.split(DocLint.TAGS_SEPARATOR)) {
                if (!isValidOption(StringUtils.toLowerCase(opt.trim()))) {
                    return false;
                }
            }
            return true;
        }

        private static boolean isValidOption(String str) {
            if (str.equals("none") || str.equals(Stats.OPT)) {
                return true;
            }
            boolean zStartsWith = str.startsWith("-");
            int iIndexOf = str.indexOf(OnBotJavaFileSystemUtils.PATH_SEPARATOR);
            String strSubstring = str.substring(zStartsWith ? 1 : 0, iIndexOf != -1 ? iIndexOf : str.length());
            return ((!zStartsWith && strSubstring.equals(ALL)) || Group.accepts(strSubstring)) && (iIndexOf == -1 || Env.AccessKind.accepts(str.substring(iIndexOf + 1)));
        }

        Options(Stats stats) {
            this.stats = stats;
        }

        boolean isEnabled(Group g, Env.AccessKind access) {
            if (this.map.isEmpty()) {
                this.map.put(ALL, Env.AccessKind.PROTECTED);
            }
            Env.AccessKind ak = this.map.get(g.optName());
            if (ak != null && access.compareTo(ak) >= 0) {
                return true;
            }
            Env.AccessKind ak2 = this.map.get(ALL);
            if (ak2 != null && access.compareTo(ak2) >= 0) {
                Env.AccessKind ak3 = this.map.get(g.notOptName());
                if (ak3 == null || access.compareTo(ak3) > 0) {
                    return true;
                }
                return false;
            }
            return false;
        }

        void setOptions(String opts) {
            if (opts == null) {
                setOption(ALL, Env.AccessKind.PRIVATE);
                return;
            }
            for (String opt : opts.split(DocLint.TAGS_SEPARATOR)) {
                setOption(StringUtils.toLowerCase(opt.trim()));
            }
        }

        private void setOption(String arg) throws IllegalArgumentException {
            if (arg.equals(Stats.OPT)) {
                this.stats.setEnabled(true);
                return;
            }
            int sep = arg.indexOf(OnBotJavaFileSystemUtils.PATH_SEPARATOR);
            if (sep > 0) {
                Env.AccessKind ak = Env.AccessKind.valueOf(StringUtils.toUpperCase(arg.substring(sep + 1)));
                setOption(arg.substring(0, sep), ak);
            } else {
                setOption(arg, null);
            }
        }

        private void setOption(String opt, Env.AccessKind ak) {
            Env.AccessKind accessKind;
            Map<String, Env.AccessKind> map = this.map;
            if (ak != null) {
                accessKind = ak;
            } else {
                accessKind = opt.startsWith("-") ? Env.AccessKind.PUBLIC : Env.AccessKind.PRIVATE;
            }
            map.put(opt, accessKind);
        }
    }

    static class Stats {
        public static final String NO_CODE = "";
        public static final String OPT = "stats";
        final ResourceBundle bundle;
        Map<String, Integer> codeCounts;
        int[] dkindCounts;
        int[] groupCounts;

        Stats(ResourceBundle bundle) {
            this.bundle = bundle;
        }

        void setEnabled(boolean b) {
            if (b) {
                this.groupCounts = new int[Group.values().length];
                this.dkindCounts = new int[Diagnostic.Kind.values().length];
                this.codeCounts = new HashMap();
            } else {
                this.groupCounts = null;
                this.dkindCounts = null;
                this.codeCounts = null;
            }
        }

        void record(Group g, Diagnostic.Kind dkind, String code) {
            if (this.codeCounts == null) {
                return;
            }
            int[] iArr = this.groupCounts;
            int iOrdinal = g.ordinal();
            iArr[iOrdinal] = iArr[iOrdinal] + 1;
            int[] iArr2 = this.dkindCounts;
            int iOrdinal2 = dkind.ordinal();
            iArr2[iOrdinal2] = iArr2[iOrdinal2] + 1;
            if (code == null) {
                code = "";
            }
            Integer i = this.codeCounts.get(code);
            this.codeCounts.put(code, Integer.valueOf(i != null ? 1 + i.intValue() : 1));
        }

        void report(PrintWriter out) {
            String msg;
            if (this.codeCounts == null) {
                return;
            }
            out.println("By group...");
            Table groupTable = new Table();
            for (Group g : Group.values()) {
                groupTable.put(g.optName(), this.groupCounts[g.ordinal()]);
            }
            groupTable.print(out);
            out.println();
            out.println("By diagnostic kind...");
            Table dkindTable = new Table();
            for (Diagnostic.Kind k : Diagnostic.Kind.values()) {
                dkindTable.put(StringUtils.toLowerCase(k.toString()), this.dkindCounts[k.ordinal()]);
            }
            dkindTable.print(out);
            out.println();
            out.println("By message kind...");
            Table codeTable = new Table();
            for (Map.Entry<String, Integer> e : this.codeCounts.entrySet()) {
                String code = e.getKey();
                try {
                    msg = code.equals("") ? "OTHER" : this.bundle.getString(code);
                } catch (MissingResourceException e2) {
                    msg = code;
                }
                codeTable.put(msg, e.getValue().intValue());
            }
            codeTable.print(out);
        }

        private static class Table {
            private static final Comparator<Integer> DECREASING = new Comparator<Integer>() { // from class: com.sun.tools.doclint.Messages.Stats.Table.1
                @Override // java.util.Comparator
                public int compare(Integer o1, Integer o2) {
                    return o2.compareTo(o1);
                }
            };
            private final TreeMap<Integer, Set<String>> map;

            private Table() {
                this.map = new TreeMap<>(DECREASING);
            }

            void put(String label, int n) {
                if (n == 0) {
                    return;
                }
                Set<String> labels = this.map.get(Integer.valueOf(n));
                if (labels == null) {
                    TreeMap<Integer, Set<String>> treeMap = this.map;
                    Integer numValueOf = Integer.valueOf(n);
                    TreeSet treeSet = new TreeSet();
                    labels = treeSet;
                    treeMap.put(numValueOf, treeSet);
                }
                labels.add(label);
            }

            void print(PrintWriter out) {
                for (Map.Entry<Integer, Set<String>> e : this.map.entrySet()) {
                    int count = e.getKey().intValue();
                    Set<String> labels = e.getValue();
                    for (String label : labels) {
                        out.println(String.format("%6d: %s", Integer.valueOf(count), label));
                    }
                }
            }
        }
    }
}
