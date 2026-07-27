package com.sun.tools.javac.jvm;

import com.sun.tools.javac.main.Option;
import com.sun.tools.javac.util.Context;
import com.sun.tools.javac.util.Options;
import java.util.EnumSet;
import java.util.Set;

/* JADX INFO: loaded from: classes.dex */
public enum Profile {
    COMPACT1("compact1", 1, Target.JDK1_8, new Target[0]),
    COMPACT2("compact2", 2, Target.JDK1_8, new Target[0]),
    COMPACT3("compact3", 3, Target.JDK1_8, new Target[0]),
    DEFAULT { // from class: com.sun.tools.javac.jvm.Profile.1
        @Override // com.sun.tools.javac.jvm.Profile
        public boolean isValid(Target t) {
            return true;
        }
    };

    private static final Context.Key<Profile> profileKey = new Context.Key<>();
    public final String name;
    final Set<Target> targets;
    public final int value;

    public static Profile instance(Context context) {
        Profile instance = (Profile) context.get(profileKey);
        if (instance == null) {
            Options options = Options.instance(context);
            String profileString = options.get(Option.PROFILE);
            if (profileString != null) {
                instance = lookup(profileString);
            }
            if (instance == null) {
                instance = DEFAULT;
            }
            context.put(profileKey, instance);
        }
        return instance;
    }

    Profile() {
        this.name = null;
        this.value = Integer.MAX_VALUE;
        this.targets = null;
    }

    Profile(String name, int value, Target t, Target... targets) {
        this.name = name;
        this.value = value;
        this.targets = EnumSet.of(t, targets);
    }

    public static Profile lookup(String name) {
        for (Profile p : values()) {
            if (name.equals(p.name)) {
                return p;
            }
        }
        return null;
    }

    public static Profile lookup(int value) {
        for (Profile p : values()) {
            if (value == p.value) {
                return p;
            }
        }
        return null;
    }

    public boolean isValid(Target t) {
        return this.targets.contains(t);
    }
}
