package com.sun.tools.javac.main;

import com.sun.tools.javac.util.ListBuffer;
import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.io.Reader;
import java.io.StreamTokenizer;

/* JADX INFO: loaded from: classes.dex */
public class CommandLine {
    public static String[] parse(String[] args) throws IOException {
        ListBuffer<String> newArgs = new ListBuffer<>();
        for (String arg : args) {
            if (arg.length() > 1 && arg.charAt(0) == '@') {
                String arg2 = arg.substring(1);
                if (arg2.charAt(0) == '@') {
                    newArgs.append(arg2);
                } else {
                    loadCmdFile(arg2, newArgs);
                }
            } else {
                newArgs.append(arg);
            }
        }
        return (String[]) newArgs.toList().toArray(new String[newArgs.length()]);
    }

    private static void loadCmdFile(String name, ListBuffer<String> args) throws IOException {
        Reader r = new BufferedReader(new FileReader(name));
        StreamTokenizer st = new StreamTokenizer(r);
        st.resetSyntax();
        st.wordChars(32, 255);
        st.whitespaceChars(0, 32);
        st.commentChar(35);
        st.quoteChar(34);
        st.quoteChar(39);
        while (st.nextToken() != -1) {
            args.append(st.sval);
        }
        r.close();
    }
}
