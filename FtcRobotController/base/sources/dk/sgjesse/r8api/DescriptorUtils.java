package dk.sgjesse.r8api;

import com.qualcomm.ftccommon.configuration.RobotConfigFileManager;

/* JADX INFO: loaded from: classes.dex */
class DescriptorUtils {
    static final /* synthetic */ boolean $assertionsDisabled = false;
    public static final char JAVA_PACKAGE_SEPARATOR = '.';

    DescriptorUtils() {
    }

    public static String guessTypeDescriptor(String name) {
        if (name == null) {
            throw new AssertionError();
        }
        if (name.endsWith(FileUtils.CLASS_EXTENSION)) {
            String descriptor = name.substring(0, name.length() - FileUtils.CLASS_EXTENSION.length());
            if (descriptor.indexOf(46) != -1) {
                throw new RuntimeException("Unexpected class file name: " + name);
            }
            return 'L' + descriptor + ';';
        }
        throw new AssertionError("Name " + name + " must have " + FileUtils.CLASS_EXTENSION + " suffix");
    }

    public static String getPathFromDescriptor(String descriptor) {
        if (!descriptor.startsWith("L")) {
            throw new AssertionError();
        }
        if (!descriptor.endsWith(RobotConfigFileManager.FILE_LIST_COMMAND_DELIMITER)) {
            throw new AssertionError();
        }
        return descriptor.substring(1, descriptor.length() - 1) + FileUtils.CLASS_EXTENSION;
    }
}
