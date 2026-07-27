package com.sun.tools.javac.main;

import com.sun.tools.doclint.DocLint;
import com.sun.tools.javac.code.Lint;
import com.sun.tools.javac.code.Source;
import com.sun.tools.javac.code.Type;
import com.sun.tools.javac.jvm.Profile;
import com.sun.tools.javac.jvm.Target;
import com.sun.tools.javac.processing.JavacProcessingEnvironment;
import com.sun.tools.javac.util.Log;
import com.sun.tools.javac.util.Options;
import com.sun.tools.javac.util.StringUtils;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.io.Writer;
import java.util.Collections;
import java.util.EnumSet;
import java.util.LinkedHashMap;
import java.util.Map;
import java.util.Set;
import javax.lang.model.SourceVersion;
import org.firstinspires.ftc.onbotjava.OnBotJavaFileSystemUtils;

/* JADX INFO: loaded from: classes.dex */
public enum Option {
    G("-g", "opt.g", OptionKind.STANDARD, OptionGroup.BASIC),
    G_NONE("-g:none", "opt.g.none", OptionKind.STANDARD, OptionGroup.BASIC) { // from class: com.sun.tools.javac.main.Option.1
        @Override // com.sun.tools.javac.main.Option
        public boolean process(OptionHelper helper, String option) {
            helper.put("-g:", "none");
            return false;
        }
    },
    G_CUSTOM("-g:", "opt.g.lines.vars.source", OptionKind.STANDARD, OptionGroup.BASIC, ChoiceKind.ANYOF, "lines", "vars", "source"),
    XLINT("-Xlint", "opt.Xlint", OptionKind.EXTENDED, OptionGroup.BASIC),
    XLINT_CUSTOM("-Xlint:", "opt.Xlint.suboptlist", OptionKind.EXTENDED, OptionGroup.BASIC, ChoiceKind.ANYOF, getXLintChoices()),
    XDOCLINT("-Xdoclint", "opt.Xdoclint", OptionKind.EXTENDED, OptionGroup.BASIC),
    XDOCLINT_CUSTOM("-Xdoclint:", "opt.Xdoclint.subopts", "opt.Xdoclint.custom", OptionKind.EXTENDED, OptionGroup.BASIC) { // from class: com.sun.tools.javac.main.Option.2
        @Override // com.sun.tools.javac.main.Option
        public boolean matches(String option) {
            return DocLint.isValidOption(option.replace(XDOCLINT_CUSTOM.text, DocLint.XMSGS_CUSTOM_PREFIX));
        }

        @Override // com.sun.tools.javac.main.Option
        public boolean process(OptionHelper helper, String option) {
            String prev = helper.get(XDOCLINT_CUSTOM);
            String next = prev == null ? option : prev + " " + option;
            helper.put(XDOCLINT_CUSTOM.text, next);
            return false;
        }
    },
    NOWARN("-nowarn", "opt.nowarn", OptionKind.STANDARD, OptionGroup.BASIC) { // from class: com.sun.tools.javac.main.Option.3
        @Override // com.sun.tools.javac.main.Option
        public boolean process(OptionHelper helper, String option) {
            helper.put("-Xlint:none", option);
            return false;
        }
    },
    VERBOSE("-verbose", "opt.verbose", OptionKind.STANDARD, OptionGroup.BASIC),
    DEPRECATION("-deprecation", "opt.deprecation", OptionKind.STANDARD, OptionGroup.BASIC) { // from class: com.sun.tools.javac.main.Option.4
        @Override // com.sun.tools.javac.main.Option
        public boolean process(OptionHelper helper, String option) {
            helper.put("-Xlint:deprecation", option);
            return false;
        }
    },
    CLASSPATH("-classpath", "opt.arg.path", "opt.classpath", OptionKind.STANDARD, OptionGroup.FILEMANAGER),
    CP("-cp", "opt.arg.path", "opt.classpath", OptionKind.STANDARD, OptionGroup.FILEMANAGER) { // from class: com.sun.tools.javac.main.Option.5
        @Override // com.sun.tools.javac.main.Option
        public boolean process(OptionHelper helper, String option, String arg) {
            return super.process(helper, "-classpath", arg);
        }
    },
    SOURCEPATH("-sourcepath", "opt.arg.path", "opt.sourcepath", OptionKind.STANDARD, OptionGroup.FILEMANAGER),
    BOOTCLASSPATH("-bootclasspath", "opt.arg.path", "opt.bootclasspath", OptionKind.STANDARD, OptionGroup.FILEMANAGER) { // from class: com.sun.tools.javac.main.Option.6
        @Override // com.sun.tools.javac.main.Option
        public boolean process(OptionHelper helper, String option, String arg) {
            helper.remove("-Xbootclasspath/p:");
            helper.remove("-Xbootclasspath/a:");
            return super.process(helper, option, arg);
        }
    },
    XBOOTCLASSPATH_PREPEND("-Xbootclasspath/p:", "opt.arg.path", "opt.Xbootclasspath.p", OptionKind.EXTENDED, OptionGroup.FILEMANAGER),
    XBOOTCLASSPATH_APPEND("-Xbootclasspath/a:", "opt.arg.path", "opt.Xbootclasspath.a", OptionKind.EXTENDED, OptionGroup.FILEMANAGER),
    XBOOTCLASSPATH("-Xbootclasspath:", "opt.arg.path", "opt.bootclasspath", OptionKind.EXTENDED, OptionGroup.FILEMANAGER) { // from class: com.sun.tools.javac.main.Option.7
        @Override // com.sun.tools.javac.main.Option
        public boolean process(OptionHelper helper, String option, String arg) {
            helper.remove("-Xbootclasspath/p:");
            helper.remove("-Xbootclasspath/a:");
            return super.process(helper, "-bootclasspath", arg);
        }
    },
    EXTDIRS("-extdirs", "opt.arg.dirs", "opt.extdirs", OptionKind.STANDARD, OptionGroup.FILEMANAGER),
    DJAVA_EXT_DIRS("-Djava.ext.dirs=", "opt.arg.dirs", "opt.extdirs", OptionKind.EXTENDED, OptionGroup.FILEMANAGER) { // from class: com.sun.tools.javac.main.Option.8
        @Override // com.sun.tools.javac.main.Option
        public boolean process(OptionHelper helper, String option, String arg) {
            return super.process(helper, "-extdirs", arg);
        }
    },
    ENDORSEDDIRS("-endorseddirs", "opt.arg.dirs", "opt.endorseddirs", OptionKind.STANDARD, OptionGroup.FILEMANAGER),
    DJAVA_ENDORSED_DIRS("-Djava.endorsed.dirs=", "opt.arg.dirs", "opt.endorseddirs", OptionKind.EXTENDED, OptionGroup.FILEMANAGER) { // from class: com.sun.tools.javac.main.Option.9
        @Override // com.sun.tools.javac.main.Option
        public boolean process(OptionHelper helper, String option, String arg) {
            return super.process(helper, "-endorseddirs", arg);
        }
    },
    PROC("-proc:", "opt.proc.none.only", OptionKind.STANDARD, OptionGroup.BASIC, ChoiceKind.ONEOF, "none", "only"),
    PROCESSOR("-processor", "opt.arg.class.list", "opt.processor", OptionKind.STANDARD, OptionGroup.BASIC),
    PROCESSORPATH("-processorpath", "opt.arg.path", "opt.processorpath", OptionKind.STANDARD, OptionGroup.FILEMANAGER),
    PARAMETERS("-parameters", "opt.parameters", OptionKind.STANDARD, OptionGroup.BASIC),
    D("-d", "opt.arg.directory", "opt.d", OptionKind.STANDARD, OptionGroup.FILEMANAGER),
    S("-s", "opt.arg.directory", "opt.sourceDest", OptionKind.STANDARD, OptionGroup.FILEMANAGER),
    H("-h", "opt.arg.directory", "opt.headerDest", OptionKind.STANDARD, OptionGroup.FILEMANAGER),
    IMPLICIT("-implicit:", "opt.implicit", OptionKind.STANDARD, OptionGroup.BASIC, ChoiceKind.ONEOF, "none", "class"),
    ENCODING("-encoding", "opt.arg.encoding", "opt.encoding", OptionKind.STANDARD, OptionGroup.FILEMANAGER) { // from class: com.sun.tools.javac.main.Option.10
        @Override // com.sun.tools.javac.main.Option
        public boolean process(OptionHelper helper, String option, String operand) {
            return super.process(helper, option, operand);
        }
    },
    SOURCE("-source", "opt.arg.release", "opt.source", OptionKind.STANDARD, OptionGroup.BASIC) { // from class: com.sun.tools.javac.main.Option.11
        @Override // com.sun.tools.javac.main.Option
        public boolean process(OptionHelper helper, String option, String operand) {
            Source source = Source.lookup(operand);
            if (source == null) {
                helper.error("err.invalid.source", operand);
                return true;
            }
            return super.process(helper, option, operand);
        }
    },
    TARGET("-target", "opt.arg.release", "opt.target", OptionKind.STANDARD, OptionGroup.BASIC) { // from class: com.sun.tools.javac.main.Option.12
        @Override // com.sun.tools.javac.main.Option
        public boolean process(OptionHelper helper, String option, String operand) {
            Target target = Target.lookup(operand);
            if (target == null) {
                helper.error("err.invalid.target", operand);
                return true;
            }
            return super.process(helper, option, operand);
        }
    },
    PROFILE("-profile", "opt.arg.profile", "opt.profile", OptionKind.STANDARD, OptionGroup.BASIC) { // from class: com.sun.tools.javac.main.Option.13
        @Override // com.sun.tools.javac.main.Option
        public boolean process(OptionHelper helper, String option, String operand) {
            Profile profile = Profile.lookup(operand);
            if (profile == null) {
                helper.error("err.invalid.profile", operand);
                return true;
            }
            return super.process(helper, option, operand);
        }
    },
    VERSION("-version", "opt.version", OptionKind.STANDARD, OptionGroup.INFO) { // from class: com.sun.tools.javac.main.Option.14
        @Override // com.sun.tools.javac.main.Option
        public boolean process(OptionHelper helper, String option) {
            Log log = helper.getLog();
            String ownName = helper.getOwnName();
            log.printLines(Log.PrefixKind.JAVAC, "version", ownName, JavaCompiler.version());
            return super.process(helper, option);
        }
    },
    FULLVERSION("-fullversion", null, OptionKind.HIDDEN, OptionGroup.INFO) { // from class: com.sun.tools.javac.main.Option.15
        @Override // com.sun.tools.javac.main.Option
        public boolean process(OptionHelper helper, String option) {
            Log log = helper.getLog();
            String ownName = helper.getOwnName();
            log.printLines(Log.PrefixKind.JAVAC, "fullVersion", ownName, JavaCompiler.fullVersion());
            return super.process(helper, option);
        }
    },
    DIAGS("-XDdiags=", null, OptionKind.HIDDEN, OptionGroup.INFO) { // from class: com.sun.tools.javac.main.Option.16
        @Override // com.sun.tools.javac.main.Option
        public boolean process(OptionHelper helper, String option) {
            String option2 = option.substring(option.indexOf(61) + 1);
            String diagsOption = (option2.contains("%") ? "-XDdiagsFormat=" : "-XDdiags=") + option2;
            if (XD.matches(diagsOption)) {
                return XD.process(helper, diagsOption);
            }
            return false;
        }
    },
    HELP("-help", "opt.help", OptionKind.STANDARD, OptionGroup.INFO) { // from class: com.sun.tools.javac.main.Option.17
        @Override // com.sun.tools.javac.main.Option
        public boolean process(OptionHelper helper, String option) {
            Log log = helper.getLog();
            String ownName = helper.getOwnName();
            log.printLines(Log.PrefixKind.JAVAC, "msg.usage.header", ownName);
            for (Option o : getJavaCompilerOptions()) {
                o.help(log, OptionKind.STANDARD);
            }
            log.printNewline();
            return super.process(helper, option);
        }
    },
    A("-A", "opt.arg.key.equals.value", "opt.A", OptionKind.STANDARD, OptionGroup.BASIC, true) { // from class: com.sun.tools.javac.main.Option.18
        @Override // com.sun.tools.javac.main.Option
        public boolean matches(String arg) {
            return arg.startsWith("-A");
        }

        @Override // com.sun.tools.javac.main.Option
        public boolean hasArg() {
            return false;
        }

        @Override // com.sun.tools.javac.main.Option
        public boolean process(OptionHelper helper, String option) {
            int argLength = option.length();
            if (argLength == 2) {
                helper.error("err.empty.A.argument", new Object[0]);
                return true;
            }
            int sepIndex = option.indexOf(61);
            String key = option.substring(2, sepIndex != -1 ? sepIndex : argLength);
            if (!JavacProcessingEnvironment.isValidOptionName(key)) {
                helper.error("err.invalid.A.key", option);
                return true;
            }
            return process(helper, option, option);
        }
    },
    X("-X", "opt.X", OptionKind.STANDARD, OptionGroup.INFO) { // from class: com.sun.tools.javac.main.Option.19
        @Override // com.sun.tools.javac.main.Option
        public boolean process(OptionHelper helper, String option) {
            Log log = helper.getLog();
            for (Option o : getJavaCompilerOptions()) {
                o.help(log, OptionKind.EXTENDED);
            }
            log.printNewline();
            log.printLines(Log.PrefixKind.JAVAC, "msg.usage.nonstandard.footer", new Object[0]);
            return super.process(helper, option);
        }
    },
    J("-J", "opt.arg.flag", "opt.J", OptionKind.STANDARD, OptionGroup.INFO, true) { // from class: com.sun.tools.javac.main.Option.20
        @Override // com.sun.tools.javac.main.Option
        public boolean process(OptionHelper helper, String option) {
            throw new AssertionError("the -J flag should be caught by the launcher.");
        }
    },
    MOREINFO("-moreinfo", null, OptionKind.HIDDEN, OptionGroup.BASIC) { // from class: com.sun.tools.javac.main.Option.21
        @Override // com.sun.tools.javac.main.Option
        public boolean process(OptionHelper helper, String option) {
            Type.moreInfo = true;
            return super.process(helper, option);
        }
    },
    WERROR("-Werror", "opt.Werror", OptionKind.STANDARD, OptionGroup.BASIC),
    PROMPT("-prompt", null, OptionKind.HIDDEN, OptionGroup.BASIC),
    DOE("-doe", null, OptionKind.HIDDEN, OptionGroup.BASIC),
    PRINTSOURCE("-printsource", null, OptionKind.HIDDEN, OptionGroup.BASIC),
    WARNUNCHECKED("-warnunchecked", null, OptionKind.HIDDEN, OptionGroup.BASIC) { // from class: com.sun.tools.javac.main.Option.22
        @Override // com.sun.tools.javac.main.Option
        public boolean process(OptionHelper helper, String option) {
            helper.put("-Xlint:unchecked", option);
            return false;
        }
    },
    XMAXERRS("-Xmaxerrs", "opt.arg.number", "opt.maxerrs", OptionKind.EXTENDED, OptionGroup.BASIC),
    XMAXWARNS("-Xmaxwarns", "opt.arg.number", "opt.maxwarns", OptionKind.EXTENDED, OptionGroup.BASIC),
    XSTDOUT("-Xstdout", "opt.arg.file", "opt.Xstdout", OptionKind.EXTENDED, OptionGroup.INFO) { // from class: com.sun.tools.javac.main.Option.23
        @Override // com.sun.tools.javac.main.Option
        public boolean process(OptionHelper helper, String option, String arg) {
            try {
                Log log = helper.getLog();
                log.setWriters(new PrintWriter((Writer) new FileWriter(arg), true));
                return super.process(helper, option, arg);
            } catch (IOException e) {
                helper.error("err.error.writing.file", arg, e);
                return true;
            }
        }
    },
    XPRINT("-Xprint", "opt.print", OptionKind.EXTENDED, OptionGroup.BASIC),
    XPRINTROUNDS("-XprintRounds", "opt.printRounds", OptionKind.EXTENDED, OptionGroup.BASIC),
    XPRINTPROCESSORINFO("-XprintProcessorInfo", "opt.printProcessorInfo", OptionKind.EXTENDED, OptionGroup.BASIC),
    XPREFER("-Xprefer:", "opt.prefer", OptionKind.EXTENDED, OptionGroup.BASIC, ChoiceKind.ONEOF, "source", "newer"),
    XPKGINFO("-Xpkginfo:", "opt.pkginfo", OptionKind.EXTENDED, OptionGroup.BASIC, ChoiceKind.ONEOF, "always", "legacy", "nonempty"),
    O("-O", null, OptionKind.HIDDEN, OptionGroup.BASIC),
    XJCOV("-Xjcov", null, OptionKind.HIDDEN, OptionGroup.BASIC),
    PLUGIN("-Xplugin:", "opt.arg.plugin", "opt.plugin", OptionKind.EXTENDED, OptionGroup.BASIC) { // from class: com.sun.tools.javac.main.Option.24
        @Override // com.sun.tools.javac.main.Option
        public boolean process(OptionHelper helper, String option) {
            String p = option.substring(option.indexOf(58) + 1);
            String prev = helper.get(PLUGIN);
            helper.put(PLUGIN.text, prev == null ? p : prev + (char) 0 + p.trim());
            return false;
        }
    },
    XDIAGS("-Xdiags:", "opt.diags", OptionKind.EXTENDED, OptionGroup.BASIC, ChoiceKind.ONEOF, "compact", "verbose"),
    XD("-XD", null, OptionKind.HIDDEN, OptionGroup.BASIC) { // from class: com.sun.tools.javac.main.Option.25
        @Override // com.sun.tools.javac.main.Option
        public boolean matches(String s) {
            return s.startsWith(this.text);
        }

        @Override // com.sun.tools.javac.main.Option
        public boolean process(OptionHelper helper, String option) {
            String option2 = option.substring(this.text.length());
            int eq = option2.indexOf(61);
            String key = eq < 0 ? option2 : option2.substring(0, eq);
            String value = eq < 0 ? option2 : option2.substring(eq + 1);
            helper.put(key, value);
            return false;
        }
    },
    AT("@", "opt.arg.file", "opt.AT", OptionKind.STANDARD, OptionGroup.INFO, true) { // from class: com.sun.tools.javac.main.Option.26
        @Override // com.sun.tools.javac.main.Option
        public boolean process(OptionHelper helper, String option) {
            throw new AssertionError("the @ flag should be caught by CommandLine.");
        }
    },
    SOURCEFILE("sourcefile", null, OptionKind.HIDDEN, OptionGroup.INFO) { // from class: com.sun.tools.javac.main.Option.27
        @Override // com.sun.tools.javac.main.Option
        public boolean matches(String s) {
            return s.endsWith(OnBotJavaFileSystemUtils.EXT_JAVA_FILE) || SourceVersion.isName(s);
        }

        @Override // com.sun.tools.javac.main.Option
        public boolean process(OptionHelper helper, String option) {
            if (option.endsWith(OnBotJavaFileSystemUtils.EXT_JAVA_FILE)) {
                File f = new File(option);
                if (!f.exists()) {
                    helper.error("err.file.not.found", f);
                    return true;
                }
                if (!f.isFile()) {
                    helper.error("err.file.not.file", f);
                    return true;
                }
                helper.addFile(f);
                return false;
            }
            helper.addClassName(option);
            return false;
        }
    };

    final String argsNameKey;
    final ChoiceKind choiceKind;
    final Map<String, Boolean> choices;
    final String descrKey;
    final OptionGroup group;
    final boolean hasSuffix;
    final OptionKind kind;
    public final String text;

    enum ChoiceKind {
        ONEOF,
        ANYOF
    }

    enum OptionGroup {
        BASIC,
        FILEMANAGER,
        INFO,
        OPERAND
    }

    public enum OptionKind {
        STANDARD,
        EXTENDED,
        HIDDEN
    }

    Option(String text, String descrKey, OptionKind kind, OptionGroup group) {
        this(text, null, descrKey, kind, group, null, null, false);
    }

    Option(String text, String argsNameKey, String descrKey, OptionKind kind, OptionGroup group) {
        this(text, argsNameKey, descrKey, kind, group, null, null, false);
    }

    Option(String text, String argsNameKey, String descrKey, OptionKind kind, OptionGroup group, boolean doHasSuffix) {
        this(text, argsNameKey, descrKey, kind, group, null, null, doHasSuffix);
    }

    Option(String text, String descrKey, OptionKind kind, OptionGroup group, ChoiceKind choiceKind, Map map) {
        this(text, null, descrKey, kind, group, choiceKind, map, false);
    }

    Option(String text, String descrKey, OptionKind kind, OptionGroup group, ChoiceKind choiceKind, String... choices) {
        this(text, null, descrKey, kind, group, choiceKind, createChoices(choices), false);
    }

    private static Map<String, Boolean> createChoices(String... choices) {
        Map<String, Boolean> map = new LinkedHashMap<>();
        for (String c : choices) {
            map.put(c, false);
        }
        return map;
    }

    Option(String text, String argsNameKey, String descrKey, OptionKind kind, OptionGroup group, ChoiceKind choiceKind, Map map, boolean doHasSuffix) {
        this.text = text;
        this.argsNameKey = argsNameKey;
        this.descrKey = descrKey;
        this.kind = kind;
        this.group = group;
        this.choiceKind = choiceKind;
        this.choices = map;
        boolean z = true;
        char lastChar = text.charAt(text.length() - 1);
        if (!doHasSuffix && lastChar != ':' && lastChar != '=') {
            z = false;
        }
        this.hasSuffix = z;
    }

    public String getText() {
        return this.text;
    }

    public OptionKind getKind() {
        return this.kind;
    }

    public boolean hasArg() {
        return (this.argsNameKey == null || this.hasSuffix) ? false : true;
    }

    public boolean matches(String option) {
        if (!this.hasSuffix) {
            return option.equals(this.text);
        }
        if (!option.startsWith(this.text)) {
            return false;
        }
        if (this.choices != null) {
            String arg = option.substring(this.text.length());
            if (this.choiceKind == ChoiceKind.ONEOF) {
                return this.choices.keySet().contains(arg);
            }
            for (String a : arg.split(",+")) {
                if (!this.choices.keySet().contains(a)) {
                    return false;
                }
            }
            return true;
        }
        return true;
    }

    public boolean process(OptionHelper helper, String option, String arg) {
        if (this.choices != null) {
            if (this.choiceKind == ChoiceKind.ONEOF) {
                for (String s : this.choices.keySet()) {
                    helper.remove(option + s);
                }
                String opt = option + arg;
                helper.put(opt, opt);
                String nm = option.substring(0, option.length() - 1);
                helper.put(nm, arg);
            } else {
                for (String a : arg.split(",+")) {
                    String opt2 = option + a;
                    helper.put(opt2, opt2);
                }
            }
        }
        helper.put(option, arg);
        return false;
    }

    public boolean process(OptionHelper helper, String option) {
        if (this.hasSuffix) {
            return process(helper, this.text, option.substring(this.text.length()));
        }
        return process(helper, option, option);
    }

    void help(Log log, OptionKind kind) {
        if (this.kind != kind) {
            return;
        }
        log.printRawLines(Log.WriterKind.NOTICE, String.format("  %-26s %s", helpSynopsis(log), log.localize(Log.PrefixKind.JAVAC, this.descrKey, new Object[0])));
    }

    private String helpSynopsis(Log log) {
        StringBuilder sb = new StringBuilder();
        sb.append(this.text);
        if (this.argsNameKey == null) {
            if (this.choices != null) {
                String sep = "{";
                for (Map.Entry<String, Boolean> e : this.choices.entrySet()) {
                    if (!e.getValue().booleanValue()) {
                        sb.append(sep);
                        sb.append(e.getKey());
                        sep = DocLint.TAGS_SEPARATOR;
                    }
                }
                sb.append("}");
            }
        } else {
            if (!this.hasSuffix) {
                sb.append(" ");
            }
            sb.append(log.localize(Log.PrefixKind.JAVAC, this.argsNameKey, new Object[0]));
        }
        return sb.toString();
    }

    public enum PkgInfo {
        ALWAYS,
        LEGACY,
        NONEMPTY;

        public static PkgInfo get(Options options) {
            String v = options.get(Option.XPKGINFO);
            return v == null ? LEGACY : valueOf(StringUtils.toUpperCase(v));
        }
    }

    private static Map<String, Boolean> getXLintChoices() {
        Map<String, Boolean> choices = new LinkedHashMap<>();
        choices.put("all", false);
        for (Lint.LintCategory c : Lint.LintCategory.values()) {
            choices.put(c.option, Boolean.valueOf(c.hidden));
        }
        for (Lint.LintCategory c2 : Lint.LintCategory.values()) {
            choices.put("-" + c2.option, Boolean.valueOf(c2.hidden));
        }
        choices.put("none", false);
        return choices;
    }

    static Set<Option> getJavaCompilerOptions() {
        return EnumSet.allOf(Option.class);
    }

    public static Set<Option> getJavacFileManagerOptions() {
        return getOptions(EnumSet.of(OptionGroup.FILEMANAGER));
    }

    public static Set<Option> getJavacToolOptions() {
        return getOptions(EnumSet.of(OptionGroup.BASIC));
    }

    static Set<Option> getOptions(Set<OptionGroup> desired) {
        Set<Option> options = EnumSet.noneOf(Option.class);
        for (Option option : values()) {
            if (desired.contains(option.group)) {
                options.add(option);
            }
        }
        return Collections.unmodifiableSet(options);
    }
}
