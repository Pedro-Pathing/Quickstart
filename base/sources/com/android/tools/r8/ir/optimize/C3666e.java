package com.android.tools.r8.ir.optimize;

import com.android.tools.r8.AssertionsConfiguration;
import com.android.tools.r8.DataResource;
import com.android.tools.r8.graph.B1;
import com.android.tools.r8.graph.H2;
import com.android.tools.r8.internal.Nk0;
import com.qualcomm.ftccommon.configuration.RobotConfigFileManager;
import dk.sgjesse.r8api.DescriptorUtils;
import org.firstinspires.ftc.onbotjava.OnBotJavaFileSystemUtils;

/* JADX INFO: renamed from: com.android.tools.r8.ir.optimize.e, reason: case insensitive filesystem */
/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes8.dex */
public final class C3666e {
    public static final /* synthetic */ boolean c = true;
    public final AssertionsConfiguration a;
    public final H2 b;

    public C3666e(AssertionsConfiguration assertionsConfiguration, B1 b1) {
        this.a = assertionsConfiguration;
        int i = AbstractC3664d.a[assertionsConfiguration.getScope().ordinal()];
        if (i == 1) {
            if (assertionsConfiguration.getValue().length() == 0) {
                this.b = b1.c("");
                return;
            } else {
                this.b = b1.c("L" + assertionsConfiguration.getValue().replace(DescriptorUtils.JAVA_PACKAGE_SEPARATOR, DataResource.SEPARATOR) + OnBotJavaFileSystemUtils.PATH_SEPARATOR);
                return;
            }
        }
        if (i == 2) {
            this.b = b1.c("L" + assertionsConfiguration.getValue().replace(DescriptorUtils.JAVA_PACKAGE_SEPARATOR, DataResource.SEPARATOR) + RobotConfigFileManager.FILE_LIST_COMMAND_DELIMITER);
        } else {
            if (i != 3) {
                throw new Nk0();
            }
            this.b = null;
        }
    }
}
