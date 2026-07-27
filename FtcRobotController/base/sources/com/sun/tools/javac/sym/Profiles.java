package com.sun.tools.javac.sym;

import com.sun.tools.javac.util.Assert;
import dk.sgjesse.r8api.FileUtils;
import java.io.BufferedInputStream;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileWriter;
import java.io.IOException;
import java.nio.charset.Charset;
import java.nio.file.Files;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;
import java.util.Properties;
import java.util.Set;
import java.util.TreeMap;
import java.util.TreeSet;
import org.firstinspires.ftc.onbotjava.OnBotJavaFileSystemUtils;
import org.slf4j.Marker;

/* JADX INFO: loaded from: classes.dex */
public abstract class Profiles {
    public abstract Set<String> getPackages(int i);

    public abstract int getProfile(String str);

    public abstract int getProfileCount();

    public static void main(String[] args) throws IOException {
        Profiles p = read(new File(args[0]));
        if (args.length >= 2) {
            Map<Integer, Set<String>> lists = new TreeMap<>();
            for (int i = 1; i <= 4; i++) {
                lists.put(Integer.valueOf(i), new TreeSet<>());
            }
            File rt_jar_lst = new File(args[1]);
            for (String line : Files.readAllLines(rt_jar_lst.toPath(), Charset.defaultCharset())) {
                if (line.endsWith(FileUtils.CLASS_EXTENSION)) {
                    String type = line.substring(0, line.length() - 6);
                    int profile = p.getProfile(type);
                    for (int i2 = profile; i2 <= 4; i2++) {
                        lists.get(Integer.valueOf(i2)).add(type);
                    }
                }
            }
            for (int i3 = 1; i3 <= 4; i3++) {
                BufferedWriter out = new BufferedWriter(new FileWriter(i3 + ".txt"));
                try {
                    Iterator<String> it = lists.get(Integer.valueOf(i3)).iterator();
                    while (it.hasNext()) {
                        out.write(it.next());
                        out.newLine();
                    }
                    out.close();
                } catch (Throwable th) {
                    out.close();
                    throw th;
                }
            }
        }
    }

    public static Profiles read(File file) throws IOException {
        BufferedInputStream in = new BufferedInputStream(new FileInputStream(file));
        try {
            Properties p = new Properties();
            p.load(in);
            if (p.containsKey("java/lang/Object")) {
                return new SimpleProfiles(p);
            }
            return new MakefileProfiles(p);
        } finally {
            in.close();
        }
    }

    private static class MakefileProfiles extends Profiles {
        final Map<String, Package> packages = new TreeMap();
        final int maxProfile = 4;

        static class Package {
            final String name;
            final Package parent;
            int profile;
            Map<String, Package> subpackages = new TreeMap();
            Map<String, Integer> includedTypes = new TreeMap();
            Map<String, Integer> excludedTypes = new TreeMap();

            Package(Package parent, String name) {
                this.parent = parent;
                this.name = name;
            }

            int getProfile() {
                return this.parent == null ? this.profile : Math.max(this.parent.getProfile(), this.profile);
            }

            int getProfile(String simpleTypeName) {
                Integer i = this.includedTypes.get(simpleTypeName);
                if (i != null) {
                    return i.intValue();
                }
                Integer i2 = this.includedTypes.get(Marker.ANY_MARKER);
                if (i2 != null) {
                    return i2.intValue();
                }
                Integer i3 = this.excludedTypes.get(simpleTypeName);
                if (i3 != null) {
                    return i3.intValue() + 1;
                }
                Integer i4 = this.excludedTypes.get(Marker.ANY_MARKER);
                if (i4 != null) {
                    return i4.intValue() + 1;
                }
                return getProfile();
            }

            String getName() {
                return this.parent == null ? this.name : this.parent.getName() + OnBotJavaFileSystemUtils.PATH_SEPARATOR + this.name;
            }

            void getPackages(int profile, Set<String> results) {
                int prf = getProfile();
                if (prf != 0 && profile >= prf) {
                    results.add(getName());
                }
                for (Package pkg : this.subpackages.values()) {
                    pkg.getPackages(profile, results);
                }
            }
        }

        MakefileProfiles(Properties p) {
            boolean foundJavaLang;
            boolean foundJavaLang2;
            int i = 4;
            boolean foundJavaLang3 = false;
            int profile = 1;
            while (profile <= i) {
                String prefix = profile < i ? "PROFILE_" + profile : "FULL_JRE";
                String inclPackages = p.getProperty(prefix + "_RTJAR_INCLUDE_PACKAGES");
                if (inclPackages == null) {
                    break;
                }
                for (String pkg : inclPackages.substring(1).trim().split("\\s+")) {
                    pkg = pkg.endsWith(OnBotJavaFileSystemUtils.PATH_SEPARATOR) ? pkg.substring(0, pkg.length() - 1) : pkg;
                    if (!foundJavaLang3 && pkg.equals("java/lang")) {
                        foundJavaLang3 = true;
                    }
                    includePackage(profile, pkg);
                }
                String inclTypes = p.getProperty(prefix + "_RTJAR_INCLUDE_TYPES");
                if (inclTypes == null) {
                    foundJavaLang = foundJavaLang3;
                } else {
                    String[] strArrSplit = inclTypes.replace("$$", "$").split("\\s+");
                    int length = strArrSplit.length;
                    int i2 = 0;
                    while (i2 < length) {
                        String type = strArrSplit[i2];
                        if (type.endsWith(FileUtils.CLASS_EXTENSION)) {
                            foundJavaLang2 = foundJavaLang3;
                            includeType(profile, type.substring(0, type.length() - 6));
                        } else {
                            foundJavaLang2 = foundJavaLang3;
                        }
                        i2++;
                        foundJavaLang3 = foundJavaLang2;
                    }
                    foundJavaLang = foundJavaLang3;
                }
                String exclTypes = p.getProperty(prefix + "_RTJAR_EXCLUDE_TYPES");
                if (exclTypes != null) {
                    for (String type2 : exclTypes.replace("$$", "$").split("\\s+")) {
                        if (type2.endsWith(FileUtils.CLASS_EXTENSION)) {
                            excludeType(profile, type2.substring(0, type2.length() - 6));
                        }
                    }
                }
                profile++;
                foundJavaLang3 = foundJavaLang;
                i = 4;
            }
            if (foundJavaLang3) {
                includePackage(1, "javax/crypto");
            }
        }

        @Override // com.sun.tools.javac.sym.Profiles
        public int getProfileCount() {
            return 4;
        }

        @Override // com.sun.tools.javac.sym.Profiles
        public int getProfile(String typeName) {
            int sep = typeName.lastIndexOf(OnBotJavaFileSystemUtils.PATH_SEPARATOR);
            String packageName = typeName.substring(0, sep);
            String simpleName = typeName.substring(sep + 1);
            Package p = getPackage(packageName);
            return p.getProfile(simpleName);
        }

        @Override // com.sun.tools.javac.sym.Profiles
        public Set<String> getPackages(int profile) {
            Set<String> results = new TreeSet<>();
            for (Package p : this.packages.values()) {
                p.getPackages(profile, results);
            }
            return results;
        }

        private void includePackage(int profile, String packageName) {
            Package p = getPackage(packageName);
            Assert.check(p.profile == 0);
            p.profile = profile;
        }

        private void includeType(int profile, String typeName) {
            int sep = typeName.lastIndexOf(OnBotJavaFileSystemUtils.PATH_SEPARATOR);
            String packageName = typeName.substring(0, sep);
            String simpleName = typeName.substring(sep + 1);
            Package p = getPackage(packageName);
            Assert.check(!p.includedTypes.containsKey(simpleName));
            p.includedTypes.put(simpleName, Integer.valueOf(profile));
        }

        private void excludeType(int profile, String typeName) {
            int sep = typeName.lastIndexOf(OnBotJavaFileSystemUtils.PATH_SEPARATOR);
            String packageName = typeName.substring(0, sep);
            String simpleName = typeName.substring(sep + 1);
            Package p = getPackage(packageName);
            Assert.check(!p.excludedTypes.containsKey(simpleName));
            p.excludedTypes.put(simpleName, Integer.valueOf(profile));
        }

        private Package getPackage(String packageName) {
            Package parent;
            Map<String, Package> parentSubpackages;
            String simpleName;
            int sep = packageName.lastIndexOf(OnBotJavaFileSystemUtils.PATH_SEPARATOR);
            if (sep == -1) {
                parent = null;
                parentSubpackages = this.packages;
                simpleName = packageName;
            } else {
                parent = getPackage(packageName.substring(0, sep));
                parentSubpackages = parent.subpackages;
                simpleName = packageName.substring(sep + 1);
            }
            Package p = parentSubpackages.get(simpleName);
            if (p == null) {
                Package p2 = new Package(parent, simpleName);
                parentSubpackages.put(simpleName, p2);
                return p2;
            }
            return p;
        }
    }

    private static class SimpleProfiles extends Profiles {
        private final Map<String, Integer> map = new HashMap();
        private final int profileCount;

        SimpleProfiles(Properties p) {
            int max = 0;
            for (Map.Entry<Object, Object> e : p.entrySet()) {
                String typeName = (String) e.getKey();
                int profile = Integer.valueOf((String) e.getValue()).intValue();
                this.map.put(typeName, Integer.valueOf(profile));
                max = Math.max(max, profile);
            }
            this.profileCount = max;
        }

        @Override // com.sun.tools.javac.sym.Profiles
        public int getProfileCount() {
            return this.profileCount;
        }

        @Override // com.sun.tools.javac.sym.Profiles
        public int getProfile(String typeName) {
            return this.map.get(typeName).intValue();
        }

        @Override // com.sun.tools.javac.sym.Profiles
        public Set<String> getPackages(int profile) {
            Set<String> results = new TreeSet<>();
            for (Map.Entry<String, Integer> e : this.map.entrySet()) {
                String tn = e.getKey();
                int prf = e.getValue().intValue();
                int sep = tn.lastIndexOf(OnBotJavaFileSystemUtils.PATH_SEPARATOR);
                if (sep > 0 && profile >= prf) {
                    results.add(tn);
                }
            }
            return results;
        }
    }
}
