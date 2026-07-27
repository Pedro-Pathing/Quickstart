package com.google.blocks.ftcrobotcontroller;

import com.google.blocks.ftcrobotcontroller.hardware.HardwareUtil;
import com.google.blocks.ftcrobotcontroller.util.BlocksArchive;
import com.google.blocks.ftcrobotcontroller.util.ClipboardUtil;
import com.google.blocks.ftcrobotcontroller.util.FileManager;
import com.google.blocks.ftcrobotcontroller.util.OfflineBlocksUtil;
import com.google.blocks.ftcrobotcontroller.util.ProjectsUtil;
import com.qualcomm.ftccommon.CommandList;
import com.qualcomm.robotcore.robocol.Command;
import fi.iki.elonen.NanoHTTPD;
import java.io.IOException;
import java.util.List;
import java.util.Map;
import org.firstinspires.ftc.onbotjava.OnBotJavaFileSystemUtils;
import org.firstinspires.ftc.onbotjava.OnBotJavaProgrammingMode;
import org.firstinspires.ftc.robotcore.internal.network.NetworkConnectionHandler;
import org.firstinspires.ftc.robotcore.internal.webserver.WebHandler;
import org.firstinspires.ftc.robotserver.internal.programmingmode.ProgrammingMode;
import org.firstinspires.ftc.robotserver.internal.programmingmode.ProgrammingModeManager;
import org.firstinspires.ftc.robotserver.internal.webserver.MimeTypesUtil;
import org.firstinspires.ftc.robotserver.internal.webserver.NoCachingWebHandler;
import org.firstinspires.ftc.robotserver.internal.webserver.RobotControllerWebHandlers;
import org.firstinspires.ftc.robotserver.internal.webserver.RobotWebHandlerManager;
import org.firstinspires.ftc.robotserver.internal.webserver.SessionParametersGenerator;

/* JADX INFO: loaded from: classes8.dex */
public class ProgrammingWebHandlers implements ProgrammingMode {
    private static final String PARAM_BLK = "blk";
    private static final String PARAM_CLIPBOARD = "cb";
    private static final String PARAM_CONTENT = "content";
    private static final String PARAM_ENABLE = "enable";
    private static final String PARAM_FM_NAME = "fmname";
    private static final String PARAM_JAVA = "java";
    private static final String PARAM_JS = "js";
    private static final String PARAM_NAME = "name";
    private static final String PARAM_NEW_NAME = "new_name";
    private static final String PARAM_SAMPLE_NAME = "sample";
    private static final String URI_COLORS = "/css/colors.less";
    private static final String URI_COPY_FILE = "/copy_file";
    private static final String URI_COPY_PROJECT = "/copy";
    private static final String URI_DELETE_FILES = "/delete_files";
    private static final String URI_DELETE_PROJECTS = "/delete";
    private static final String URI_ENABLE_PROJECT = "/enable";
    private static final String URI_FETCH_BLK = "/fetch_blk";
    private static final String URI_FETCH_BLOCKS_ARCHIVE = "/archive_blocks";
    private static final String URI_FETCH_CLIPBOARD = "/fetch_cb";
    private static final String URI_FETCH_FILE = "/fetch_file";
    private static final String URI_FETCH_FILE_TYPE = "/fetch_file_type";
    private static final String URI_FETCH_OFFLINE_BLOCKS_EDITOR = "/offline_blocks_editor";
    private static final String URI_FILE_MANAGER_JS = "/file_manager_js";
    private static final String URI_GET_BLOCKS_JAVA_CLASS_NAME = "/get_blocks_java_class_name";
    private static final String URI_GET_CONFIGURATION_NAME = "/get_config_name";
    private static final String URI_HARDWARE = "/hardware";
    private static final String URI_LIST_FILES = "/list_files";
    private static final String URI_LIST_PROJECTS = "/list";
    private static final String URI_LIST_SAMPLES = "/samples";
    private static final String URI_NEW_PROJECT = "/new";
    private static final String URI_RENAME_FILE = "/rename_file";
    private static final String URI_RENAME_PROJECT = "/rename";
    private static final String URI_RESTART_ROBOT = "/restart_robot";
    private static final String URI_SAVE_BLOCKS_JAVA = "/save_blocks_java";
    private static final String URI_SAVE_CLIPBOARD = "/savecb";
    private static final String URI_SAVE_FILE = "/save_file";
    private static final String URI_SAVE_PROJECT = "/save";
    private static final String URI_SERVER = "/server";
    private volatile ProgrammingModeManager programmingModeManager;
    public static final String TAG = ProgrammingWebHandlers.class.getSimpleName();
    private static String URI_NAV_BLOCKS_OLD = "/FtcProjects.html";
    private static String URI_NAV_BLOCKS = "/FtcBlocksProjects.html";
    private static String URI_NAV_ONBOTJAVA = OnBotJavaProgrammingMode.URI_JAVA_EDITOR;

    private static class Server implements WebHandler {
        private Server() {
        }

        @Override // org.firstinspires.ftc.robotcore.internal.webserver.WebHandler
        public NanoHTTPD.Response getResponse(NanoHTTPD.IHTTPSession session) throws NanoHTTPD.ResponseException, IOException {
            return fetchJavaScriptForServer(session);
        }

        private NanoHTTPD.Response fetchJavaScriptForServer(NanoHTTPD.IHTTPSession session) throws IOException {
            StringBuilder js = new StringBuilder();
            js.append("var URI_HARDWARE = '").append(ProgrammingWebHandlers.URI_HARDWARE).append("';\n");
            js.append("var URI_FILE_MANAGER_JS = '").append(ProgrammingWebHandlers.URI_FILE_MANAGER_JS).append("';\n");
            js.append("var URI_GET_CONFIGURATION_NAME = '").append(ProgrammingWebHandlers.URI_GET_CONFIGURATION_NAME).append("';\n");
            js.append("var URI_FETCH_OFFLINE_BLOCKS_EDITOR = '").append(ProgrammingWebHandlers.URI_FETCH_OFFLINE_BLOCKS_EDITOR).append("';\n");
            js.append("var URI_FETCH_BLOCKS_ARCHIVE = '").append(ProgrammingWebHandlers.URI_FETCH_BLOCKS_ARCHIVE).append("';\n");
            js.append("var URI_LIST_PROJECTS = '").append(ProgrammingWebHandlers.URI_LIST_PROJECTS).append("';\n");
            js.append("var URI_LIST_SAMPLES = '").append(ProgrammingWebHandlers.URI_LIST_SAMPLES).append("';\n");
            js.append("var URI_FETCH_BLK = '").append(ProgrammingWebHandlers.URI_FETCH_BLK).append("';\n");
            js.append("var URI_NEW_PROJECT = '").append(ProgrammingWebHandlers.URI_NEW_PROJECT).append("';\n");
            js.append("var URI_SAVE_PROJECT = '").append(ProgrammingWebHandlers.URI_SAVE_PROJECT).append("';\n");
            js.append("var URI_RENAME_PROJECT = '").append(ProgrammingWebHandlers.URI_RENAME_PROJECT).append("';\n");
            js.append("var URI_COPY_PROJECT = '").append(ProgrammingWebHandlers.URI_COPY_PROJECT).append("';\n");
            js.append("var URI_ENABLE_PROJECT = '").append(ProgrammingWebHandlers.URI_ENABLE_PROJECT).append("';\n");
            js.append("var URI_DELETE_PROJECTS = '").append(ProgrammingWebHandlers.URI_DELETE_PROJECTS).append("';\n");
            js.append("var URI_GET_BLOCKS_JAVA_CLASS_NAME = '").append(ProgrammingWebHandlers.URI_GET_BLOCKS_JAVA_CLASS_NAME).append("';\n");
            js.append("var URI_SAVE_BLOCKS_JAVA = '").append(ProgrammingWebHandlers.URI_SAVE_BLOCKS_JAVA).append("';\n");
            js.append("var URI_SAVE_CLIPBOARD = '").append(ProgrammingWebHandlers.URI_SAVE_CLIPBOARD).append("';\n");
            js.append("var URI_FETCH_CLIPBOARD = '").append(ProgrammingWebHandlers.URI_FETCH_CLIPBOARD).append("';\n");
            js.append("var URI_LIST_FILES = '").append(ProgrammingWebHandlers.URI_LIST_FILES).append("';\n");
            js.append("var URI_SAVE_FILE = '").append(ProgrammingWebHandlers.URI_SAVE_FILE).append("';\n");
            js.append("var URI_FETCH_FILE = '").append(ProgrammingWebHandlers.URI_FETCH_FILE).append("';\n");
            js.append("var URI_FETCH_FILE_TYPE = '").append(ProgrammingWebHandlers.URI_FETCH_FILE_TYPE).append("';\n");
            js.append("var URI_RENAME_FILE = '").append(ProgrammingWebHandlers.URI_RENAME_FILE).append("';\n");
            js.append("var URI_COPY_FILE = '").append(ProgrammingWebHandlers.URI_COPY_FILE).append("';\n");
            js.append("var URI_DELETE_FILES = '").append(ProgrammingWebHandlers.URI_DELETE_FILES).append("';\n");
            js.append("var URI_RESTART_ROBOT = '").append(ProgrammingWebHandlers.URI_RESTART_ROBOT).append("';\n");
            js.append("var PARAM_FM_NAME = '").append(ProgrammingWebHandlers.PARAM_FM_NAME).append("';\n");
            js.append("var PARAM_NAME = '").append("name").append("';\n");
            js.append("var PARAM_NEW_NAME = '").append("new_name").append("';\n");
            js.append("var PARAM_SAMPLE_NAME = '").append(ProgrammingWebHandlers.PARAM_SAMPLE_NAME).append("';\n");
            js.append("var PARAM_BLK = '").append(ProgrammingWebHandlers.PARAM_BLK).append("';\n");
            js.append("var PARAM_JS = '").append(ProgrammingWebHandlers.PARAM_JS).append("';\n");
            js.append("var PARAM_JAVA = '").append(ProgrammingWebHandlers.PARAM_JAVA).append("';\n");
            js.append("var PARAM_ENABLE = '").append(ProgrammingWebHandlers.PARAM_ENABLE).append("';\n");
            js.append("var PARAM_CLIPBOARD = '").append(ProgrammingWebHandlers.PARAM_CLIPBOARD).append("';\n");
            js.append("var PARAM_CONTENT = '").append(ProgrammingWebHandlers.PARAM_CONTENT).append("';\n");
            return NoCachingWebHandler.setNoCache(session, NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.OK, MimeTypesUtil.MIME_JAVASCRIPT, js.toString()));
        }
    }

    private static class RobotControllerConfiguration extends RobotControllerWebHandlers.RobotControllerConfiguration {
        private RobotControllerConfiguration() {
        }

        @Override // org.firstinspires.ftc.robotserver.internal.webserver.RobotControllerWebHandlers.RobotControllerConfiguration
        protected void appendVariables(StringBuilder js) {
            super.appendVariables(js);
            appendVariable(js, "URI_NAV_BLOCKS", ProgrammingWebHandlers.URI_NAV_BLOCKS);
            appendVariable(js, "URI_NAV_ONBOTJAVA", ProgrammingWebHandlers.URI_NAV_ONBOTJAVA);
        }
    }

    private static class ListProjects implements WebHandler {
        private ListProjects() {
        }

        @Override // org.firstinspires.ftc.robotcore.internal.webserver.WebHandler
        public NanoHTTPD.Response getResponse(NanoHTTPD.IHTTPSession session) throws NanoHTTPD.ResponseException, IOException {
            return fetchProjects(session);
        }

        private NanoHTTPD.Response fetchProjects(NanoHTTPD.IHTTPSession session) throws IOException {
            String jsonProjects = ProjectsUtil.fetchProjectsWithBlocks();
            return NoCachingWebHandler.setNoCache(session, NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.OK, "text/plain", jsonProjects));
        }
    }

    private static class ListSamples implements WebHandler {
        private ListSamples() {
        }

        @Override // org.firstinspires.ftc.robotcore.internal.webserver.WebHandler
        public NanoHTTPD.Response getResponse(NanoHTTPD.IHTTPSession session) throws NanoHTTPD.ResponseException, IOException {
            return fetchSamples(session);
        }

        private NanoHTTPD.Response fetchSamples(NanoHTTPD.IHTTPSession session) throws IOException {
            String jsonSamples = ProjectsUtil.fetchSampleNames();
            return NoCachingWebHandler.setNoCache(session, NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.OK, "text/plain", jsonSamples));
        }
    }

    private static class Hardware implements WebHandler {
        private Hardware() {
        }

        @Override // org.firstinspires.ftc.robotcore.internal.webserver.WebHandler
        public NanoHTTPD.Response getResponse(NanoHTTPD.IHTTPSession session) throws NanoHTTPD.ResponseException, IOException {
            return fetchJavaScriptForHardware(session);
        }

        private NanoHTTPD.Response fetchJavaScriptForHardware(NanoHTTPD.IHTTPSession session) throws IOException {
            String jsHardware = HardwareUtil.fetchJavaScriptForHardware();
            return NoCachingWebHandler.setNoCache(session, NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.OK, MimeTypesUtil.MIME_JAVASCRIPT, jsHardware));
        }
    }

    private static class GetConfigurationName implements WebHandler {
        private GetConfigurationName() {
        }

        @Override // org.firstinspires.ftc.robotcore.internal.webserver.WebHandler
        public NanoHTTPD.Response getResponse(NanoHTTPD.IHTTPSession session) throws NanoHTTPD.ResponseException, IOException {
            return getConfigurationName(session);
        }

        private NanoHTTPD.Response getConfigurationName(NanoHTTPD.IHTTPSession session) throws IOException {
            String configName = HardwareUtil.getConfigurationName();
            return NoCachingWebHandler.setNoCache(session, NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.OK, "text/plain", configName));
        }
    }

    private static class FetchOfflineBlocksEditor implements WebHandler {
        private FetchOfflineBlocksEditor() {
        }

        @Override // org.firstinspires.ftc.robotcore.internal.webserver.WebHandler
        public NanoHTTPD.Response getResponse(NanoHTTPD.IHTTPSession session) throws NanoHTTPD.ResponseException, IOException {
            return fetchOfflineBlocksEditor(session);
        }

        private NanoHTTPD.Response fetchOfflineBlocksEditor(NanoHTTPD.IHTTPSession session) throws IOException {
            return NoCachingWebHandler.setNoCache(session, NanoHTTPD.newChunkedResponse(NanoHTTPD.Response.Status.OK, "application/zip", OfflineBlocksUtil.fetchOfflineBlocksEditor()));
        }
    }

    private static class FetchBlocksArchive implements WebHandler {
        private FetchBlocksArchive() {
        }

        @Override // org.firstinspires.ftc.robotcore.internal.webserver.WebHandler
        public NanoHTTPD.Response getResponse(NanoHTTPD.IHTTPSession session) throws NanoHTTPD.ResponseException, IOException {
            return fetchBlocksArchive(session);
        }

        private NanoHTTPD.Response fetchBlocksArchive(NanoHTTPD.IHTTPSession session) throws IOException {
            return NoCachingWebHandler.setNoCache(session, NanoHTTPD.newChunkedResponse(NanoHTTPD.Response.Status.OK, "application/zip", BlocksArchive.fetchBlocksArchive()));
        }
    }

    private static class FetchBlockFile implements WebHandler {
        private FetchBlockFile() {
        }

        @Override // org.firstinspires.ftc.robotcore.internal.webserver.WebHandler
        public NanoHTTPD.Response getResponse(NanoHTTPD.IHTTPSession session) throws NanoHTTPD.ResponseException, IOException {
            String name = ProgrammingWebHandlers.getFirstNamedParameter(session, "name");
            if (name != null) {
                return fetchBlkFileContent(session, name);
            }
            return NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.BAD_REQUEST, "text/plain", "Bad Request: name parameter is required");
        }

        private NanoHTTPD.Response fetchBlkFileContent(NanoHTTPD.IHTTPSession session, String projectName) throws IOException {
            String blkFileContent = ProjectsUtil.fetchBlkFileContent(projectName);
            return NoCachingWebHandler.setNoCache(session, NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.OK, "text/plain", blkFileContent));
        }
    }

    private static class NewProject implements WebHandler {
        private NewProject() {
        }

        @Override // org.firstinspires.ftc.robotcore.internal.webserver.WebHandler
        public NanoHTTPD.Response getResponse(NanoHTTPD.IHTTPSession session) throws NanoHTTPD.ResponseException, IOException {
            String name = ProgrammingWebHandlers.getFirstNamedParameter(session, "name");
            if (name != null) {
                String sampleName = ProgrammingWebHandlers.getFirstNamedParameter(session, ProgrammingWebHandlers.PARAM_SAMPLE_NAME);
                return newProject(session, name, sampleName);
            }
            return NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.BAD_REQUEST, "text/plain", "Bad Request: name parameter is required");
        }

        private NanoHTTPD.Response newProject(NanoHTTPD.IHTTPSession session, String projectName, String sampleName) throws IOException {
            String blkContent = ProjectsUtil.newProject(projectName, sampleName);
            return NoCachingWebHandler.setNoCache(session, NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.OK, "text/plain", blkContent));
        }
    }

    private static class SaveProject implements WebHandler {
        private SaveProject() {
        }

        @Override // org.firstinspires.ftc.robotcore.internal.webserver.WebHandler
        public NanoHTTPD.Response getResponse(NanoHTTPD.IHTTPSession session) throws NanoHTTPD.ResponseException, IOException {
            String name = ProgrammingWebHandlers.getFirstNamedParameter(session, "name");
            String blk = ProgrammingWebHandlers.getFirstNamedParameter(session, ProgrammingWebHandlers.PARAM_BLK);
            String js = ProgrammingWebHandlers.getFirstNamedParameter(session, ProgrammingWebHandlers.PARAM_JS);
            if (name != null && blk != null && js != null) {
                return saveProject(name, blk, js);
            }
            return NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.BAD_REQUEST, "text/plain", "Bad Request: name, blk, and js parameters are required");
        }

        private NanoHTTPD.Response saveProject(String projectName, String blkFileContent, String jsFileContent) throws Throwable {
            ProjectsUtil.saveProject(projectName, blkFileContent, jsFileContent);
            return NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.OK, "text/plain", "");
        }
    }

    private static class RenameProject implements WebHandler {
        private RenameProject() {
        }

        @Override // org.firstinspires.ftc.robotcore.internal.webserver.WebHandler
        public NanoHTTPD.Response getResponse(NanoHTTPD.IHTTPSession session) throws NanoHTTPD.ResponseException, IOException {
            String oldName = ProgrammingWebHandlers.getFirstNamedParameter(session, "name");
            String newName = ProgrammingWebHandlers.getFirstNamedParameter(session, "new_name");
            if (oldName != null && newName != null) {
                return renameProject(oldName, newName);
            }
            return NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.BAD_REQUEST, "text/plain", "Bad Request: name and new_name parameters are required");
        }

        private NanoHTTPD.Response renameProject(String oldProjectName, String newProjectName) throws Throwable {
            ProjectsUtil.renameProject(oldProjectName, newProjectName);
            return NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.OK, "text/plain", "");
        }
    }

    private static class CopyProject implements WebHandler {
        private CopyProject() {
        }

        @Override // org.firstinspires.ftc.robotcore.internal.webserver.WebHandler
        public NanoHTTPD.Response getResponse(NanoHTTPD.IHTTPSession session) throws NanoHTTPD.ResponseException, IOException {
            String oldName = ProgrammingWebHandlers.getFirstNamedParameter(session, "name");
            String newName = ProgrammingWebHandlers.getFirstNamedParameter(session, "new_name");
            if (oldName != null && newName != null) {
                return copyProject(oldName, newName);
            }
            return NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.BAD_REQUEST, "text/plain", "Bad Request: name and new_name parameters are required");
        }

        private NanoHTTPD.Response copyProject(String oldProjectName, String newProjectName) throws Throwable {
            try {
                ProjectsUtil.copyProject(oldProjectName, newProjectName);
                return NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.OK, "text/plain", "");
            } catch (IOExceptionWithUserVisibleMessage e) {
                return NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.OK, "text/plain", e.getMessage());
            }
        }
    }

    private static class EnableProject implements WebHandler {
        private EnableProject() {
        }

        @Override // org.firstinspires.ftc.robotcore.internal.webserver.WebHandler
        public NanoHTTPD.Response getResponse(NanoHTTPD.IHTTPSession session) throws NanoHTTPD.ResponseException, IOException {
            String name = ProgrammingWebHandlers.getFirstNamedParameter(session, "name");
            String enable = ProgrammingWebHandlers.getFirstNamedParameter(session, ProgrammingWebHandlers.PARAM_ENABLE);
            if (name != null && enable != null) {
                return enableProject(name, Boolean.parseBoolean(enable));
            }
            return NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.BAD_REQUEST, "text/plain", "Bad Request: name and new_name parameters are required");
        }

        private NanoHTTPD.Response enableProject(String projectName, boolean enable) throws Throwable {
            ProjectsUtil.enableProject(projectName, enable);
            return NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.OK, "text/plain", "");
        }
    }

    private static class DeleteProjects implements WebHandler {
        private DeleteProjects() {
        }

        @Override // org.firstinspires.ftc.robotcore.internal.webserver.WebHandler
        public NanoHTTPD.Response getResponse(NanoHTTPD.IHTTPSession session) throws NanoHTTPD.ResponseException, IOException {
            String names = ProgrammingWebHandlers.getFirstNamedParameter(session, "name");
            if (names != null) {
                return deleteProjects(names);
            }
            return NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.BAD_REQUEST, "text/plain", "Bad Request: name parameter is required");
        }

        private NanoHTTPD.Response deleteProjects(String starDelimitedProjectNames) throws IOException {
            String[] projectNames = starDelimitedProjectNames.split("\\*");
            ProjectsUtil.deleteProjects(projectNames);
            return NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.OK, "text/plain", "");
        }
    }

    private static class GetBlocksJavaClassName implements WebHandler {
        private GetBlocksJavaClassName() {
        }

        @Override // org.firstinspires.ftc.robotcore.internal.webserver.WebHandler
        public NanoHTTPD.Response getResponse(NanoHTTPD.IHTTPSession session) throws NanoHTTPD.ResponseException, IOException {
            String name = ProgrammingWebHandlers.getFirstNamedParameter(session, "name");
            if (name != null) {
                return getBlocksJavaClassName(session, name);
            }
            return NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.BAD_REQUEST, "text/plain", "Bad Request: name parameter is required");
        }

        private NanoHTTPD.Response getBlocksJavaClassName(NanoHTTPD.IHTTPSession session, String projectName) throws IOException {
            String className = ProjectsUtil.getBlocksJavaClassName(projectName);
            return NoCachingWebHandler.setNoCache(session, NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.OK, "text/plain", className));
        }
    }

    private static class SaveBlocksJava implements WebHandler {
        private SaveBlocksJava() {
        }

        @Override // org.firstinspires.ftc.robotcore.internal.webserver.WebHandler
        public NanoHTTPD.Response getResponse(NanoHTTPD.IHTTPSession session) throws NanoHTTPD.ResponseException, IOException {
            String relativeFileName = ProgrammingWebHandlers.getFirstNamedParameter(session, "name");
            String javaContent = ProgrammingWebHandlers.getFirstNamedParameter(session, ProgrammingWebHandlers.PARAM_JAVA);
            if (relativeFileName != null && javaContent != null) {
                return saveBlocksJava(relativeFileName, javaContent);
            }
            return NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.BAD_REQUEST, "text/plain", "Bad Request: name and java parameters are required");
        }

        private NanoHTTPD.Response saveBlocksJava(String relativeFileName, String javaContent) throws Throwable {
            ProjectsUtil.saveBlocksJava(relativeFileName, javaContent);
            return NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.OK, "text/plain", "");
        }
    }

    private static class SaveClipboard implements WebHandler {
        private SaveClipboard() {
        }

        @Override // org.firstinspires.ftc.robotcore.internal.webserver.WebHandler
        public NanoHTTPD.Response getResponse(NanoHTTPD.IHTTPSession session) throws NanoHTTPD.ResponseException, IOException {
            String clipboardContent = ProgrammingWebHandlers.getFirstNamedParameter(session, ProgrammingWebHandlers.PARAM_CLIPBOARD);
            if (clipboardContent != null) {
                return saveClipboardContent(clipboardContent);
            }
            return NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.BAD_REQUEST, "text/plain", "Bad Request: cb parameter is required");
        }

        private NanoHTTPD.Response saveClipboardContent(String clipboardContent) throws IOException {
            ClipboardUtil.saveClipboardContent(clipboardContent);
            return NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.OK, "text/plain", "");
        }
    }

    private static class FetchClipboard implements WebHandler {
        private FetchClipboard() {
        }

        @Override // org.firstinspires.ftc.robotcore.internal.webserver.WebHandler
        public NanoHTTPD.Response getResponse(NanoHTTPD.IHTTPSession session) throws NanoHTTPD.ResponseException, IOException {
            return fetchClipboardContent();
        }

        private NanoHTTPD.Response fetchClipboardContent() throws IOException {
            String clipboardContent = ClipboardUtil.fetchClipboardContent();
            return NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.OK, "text/plain", clipboardContent);
        }
    }

    private static class FileManagerJS implements WebHandler {
        private FileManagerJS() {
        }

        @Override // org.firstinspires.ftc.robotcore.internal.webserver.WebHandler
        public NanoHTTPD.Response getResponse(NanoHTTPD.IHTTPSession session) throws NanoHTTPD.ResponseException, IOException {
            String fmName = ProgrammingWebHandlers.getFirstNamedParameter(session, ProgrammingWebHandlers.PARAM_FM_NAME);
            if (fmName == null) {
                return NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.BAD_REQUEST, "text/plain", "Bad Request: fmname parameter is required");
            }
            try {
                String js = FileManager.valueOf(fmName).fetchJavaScript();
                return NoCachingWebHandler.setNoCache(session, NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.OK, MimeTypesUtil.MIME_JAVASCRIPT, js));
            } catch (Exception e) {
                e.printStackTrace();
                return NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.INTERNAL_ERROR, "text/plain", "Internal Error");
            }
        }
    }

    private static class ListFiles implements WebHandler {
        private ListFiles() {
        }

        @Override // org.firstinspires.ftc.robotcore.internal.webserver.WebHandler
        public NanoHTTPD.Response getResponse(NanoHTTPD.IHTTPSession session) throws NanoHTTPD.ResponseException, IOException {
            String fmName = ProgrammingWebHandlers.getFirstNamedParameter(session, ProgrammingWebHandlers.PARAM_FM_NAME);
            if (fmName == null) {
                return NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.BAD_REQUEST, "text/plain", "Bad Request: fmname parameter is required");
            }
            try {
                String json = FileManager.valueOf(fmName).fetchFiles();
                return NoCachingWebHandler.setNoCache(session, NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.OK, "text/plain", json));
            } catch (Exception e) {
                e.printStackTrace();
                return NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.INTERNAL_ERROR, "text/plain", "Internal Error");
            }
        }
    }

    private static class SaveFile implements WebHandler {
        private SaveFile() {
        }

        @Override // org.firstinspires.ftc.robotcore.internal.webserver.WebHandler
        public NanoHTTPD.Response getResponse(NanoHTTPD.IHTTPSession session) throws NanoHTTPD.ResponseException, IOException {
            String fmName = ProgrammingWebHandlers.getFirstNamedParameter(session, ProgrammingWebHandlers.PARAM_FM_NAME);
            String name = ProgrammingWebHandlers.getFirstNamedParameter(session, "name");
            String base64Content = ProgrammingWebHandlers.getFirstNamedParameter(session, ProgrammingWebHandlers.PARAM_CONTENT);
            if (fmName == null || name == null || base64Content == null) {
                return NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.BAD_REQUEST, "text/plain", "Bad Request: fmname, name, and content parameters are required");
            }
            try {
                FileManager.valueOf(fmName).saveFile(name, base64Content);
                return NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.OK, "text/plain", "");
            } catch (Exception e) {
                e.printStackTrace();
                return NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.INTERNAL_ERROR, "text/plain", "Internal Error");
            }
        }
    }

    private static class FetchFile implements WebHandler {
        private FetchFile() {
        }

        @Override // org.firstinspires.ftc.robotcore.internal.webserver.WebHandler
        public NanoHTTPD.Response getResponse(NanoHTTPD.IHTTPSession session) throws NanoHTTPD.ResponseException, IOException {
            String name = ProgrammingWebHandlers.getFirstNamedParameter(session, "name");
            String fmName = ProgrammingWebHandlers.getFirstNamedParameter(session, ProgrammingWebHandlers.PARAM_FM_NAME);
            if (fmName == null || name == null) {
                return NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.BAD_REQUEST, "text/plain", "Bad Request: fmname and name parameters are required");
            }
            try {
                String base64Content = FileManager.valueOf(fmName).fetchFileContent(name);
                return NoCachingWebHandler.setNoCache(session, NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.OK, "text/plain", base64Content));
            } catch (Exception e) {
                e.printStackTrace();
                return NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.INTERNAL_ERROR, "text/plain", "Internal Error");
            }
        }
    }

    private static class FetchFileType implements WebHandler {
        private FetchFileType() {
        }

        @Override // org.firstinspires.ftc.robotcore.internal.webserver.WebHandler
        public NanoHTTPD.Response getResponse(NanoHTTPD.IHTTPSession session) throws NanoHTTPD.ResponseException, IOException {
            String name = ProgrammingWebHandlers.getFirstNamedParameter(session, "name");
            if (name == null) {
                return NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.BAD_REQUEST, "text/plain", "Bad Request: name parameter is required");
            }
            String mimeType = MimeTypesUtil.determineMimeType(name);
            if (mimeType == null) {
                mimeType = "";
            }
            return NoCachingWebHandler.setNoCache(session, NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.OK, "text/plain", mimeType));
        }
    }

    private static class RenameFile implements WebHandler {
        private RenameFile() {
        }

        @Override // org.firstinspires.ftc.robotcore.internal.webserver.WebHandler
        public NanoHTTPD.Response getResponse(NanoHTTPD.IHTTPSession session) throws NanoHTTPD.ResponseException, IOException {
            String fmName = ProgrammingWebHandlers.getFirstNamedParameter(session, ProgrammingWebHandlers.PARAM_FM_NAME);
            String oldName = ProgrammingWebHandlers.getFirstNamedParameter(session, "name");
            String newName = ProgrammingWebHandlers.getFirstNamedParameter(session, "new_name");
            if (fmName == null || oldName == null || newName == null) {
                return NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.BAD_REQUEST, "text/plain", "Bad Request: fmname, name, and new_name parameters are required");
            }
            try {
                FileManager.valueOf(fmName).renameFile(oldName, newName);
                return NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.OK, "text/plain", "");
            } catch (Exception e) {
                e.printStackTrace();
                return NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.INTERNAL_ERROR, "text/plain", "Internal Error");
            }
        }
    }

    private static class CopyFile implements WebHandler {
        private CopyFile() {
        }

        @Override // org.firstinspires.ftc.robotcore.internal.webserver.WebHandler
        public NanoHTTPD.Response getResponse(NanoHTTPD.IHTTPSession session) throws NanoHTTPD.ResponseException, IOException {
            String fmName = ProgrammingWebHandlers.getFirstNamedParameter(session, ProgrammingWebHandlers.PARAM_FM_NAME);
            String oldName = ProgrammingWebHandlers.getFirstNamedParameter(session, "name");
            String newName = ProgrammingWebHandlers.getFirstNamedParameter(session, "new_name");
            if (fmName == null || oldName == null || newName == null) {
                return NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.BAD_REQUEST, "text/plain", "Bad Request: fmname, name, and new_name parameters are required");
            }
            try {
                FileManager.valueOf(fmName).copyFile(oldName, newName);
                return NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.OK, "text/plain", "");
            } catch (Exception e) {
                e.printStackTrace();
                return NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.INTERNAL_ERROR, "text/plain", "Internal Error");
            }
        }
    }

    private static class DeleteFiles implements WebHandler {
        private DeleteFiles() {
        }

        @Override // org.firstinspires.ftc.robotcore.internal.webserver.WebHandler
        public NanoHTTPD.Response getResponse(NanoHTTPD.IHTTPSession session) throws NanoHTTPD.ResponseException, IOException {
            String fmName = ProgrammingWebHandlers.getFirstNamedParameter(session, ProgrammingWebHandlers.PARAM_FM_NAME);
            String starDelimitedNames = ProgrammingWebHandlers.getFirstNamedParameter(session, "name");
            if (fmName == null || starDelimitedNames == null) {
                return NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.BAD_REQUEST, "text/plain", "Bad Request: fmname and name parameters are required");
            }
            try {
                String[] names = starDelimitedNames.split("\\*");
                FileManager.valueOf(fmName).deleteFiles(names);
                return NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.OK, "text/plain", "");
            } catch (Exception e) {
                e.printStackTrace();
                return NanoHTTPD.newFixedLengthResponse(NanoHTTPD.Response.Status.INTERNAL_ERROR, "text/plain", "Internal Error");
            }
        }
    }

    private static class RestartRobot implements WebHandler {
        private RestartRobot() {
        }

        @Override // org.firstinspires.ftc.robotcore.internal.webserver.WebHandler
        public NanoHTTPD.Response getResponse(NanoHTTPD.IHTTPSession session) throws NanoHTTPD.ResponseException, IOException {
            NetworkConnectionHandler connectionHandler = NetworkConnectionHandler.getInstance();
            connectionHandler.injectReceivedCommand(new Command(CommandList.CMD_RESTART_ROBOT));
            return RobotWebHandlerManager.OK_RESPONSE;
        }
    }

    private WebHandler decorateWithLogging(WebHandler handler) {
        return this.programmingModeManager.decorate(false, handler);
    }

    @Override // org.firstinspires.ftc.robotserver.internal.programmingmode.ProgrammingMode
    public void register(ProgrammingModeManager manager) {
        this.programmingModeManager = manager;
        manager.register(URI_NAV_BLOCKS_OLD, new RobotControllerWebHandlers.Redirection(OnBotJavaFileSystemUtils.PATH_SEPARATOR));
        manager.register(URI_SERVER, decorateWithLogging(new Server()));
        manager.register(URI_HARDWARE, decorateWithLogging(new Hardware()));
        manager.register(URI_FILE_MANAGER_JS, decorateWithLogging(decorateWithParms(new FileManagerJS())));
        manager.register(URI_GET_CONFIGURATION_NAME, decorateWithLogging(decorateWithParms(new GetConfigurationName())));
        manager.register(URI_FETCH_OFFLINE_BLOCKS_EDITOR, decorateWithLogging(new FetchOfflineBlocksEditor()));
        manager.register(URI_FETCH_BLOCKS_ARCHIVE, decorateWithLogging(new FetchBlocksArchive()));
        manager.register(URI_LIST_PROJECTS, decorateWithLogging(new ListProjects()));
        manager.register(URI_LIST_SAMPLES, decorateWithLogging(new ListSamples()));
        manager.register(URI_FETCH_BLK, decorateWithLogging(decorateWithParms(new FetchBlockFile())));
        manager.register(URI_NEW_PROJECT, decorateWithLogging(decorateWithParms(new NewProject())));
        manager.register(URI_SAVE_PROJECT, decorateWithLogging(decorateWithParms(new SaveProject())));
        manager.register(URI_RENAME_PROJECT, decorateWithLogging(decorateWithParms(new RenameProject())));
        manager.register(URI_COPY_PROJECT, decorateWithLogging(decorateWithParms(new CopyProject())));
        manager.register(URI_ENABLE_PROJECT, decorateWithLogging(decorateWithParms(new EnableProject())));
        manager.register(URI_DELETE_PROJECTS, decorateWithLogging(decorateWithParms(new DeleteProjects())));
        manager.register(URI_GET_BLOCKS_JAVA_CLASS_NAME, decorateWithLogging(decorateWithParms(new GetBlocksJavaClassName())));
        manager.register(URI_SAVE_BLOCKS_JAVA, decorateWithLogging(decorateWithParms(new SaveBlocksJava())));
        manager.register(URI_SAVE_CLIPBOARD, decorateWithLogging(decorateWithParms(new SaveClipboard())));
        manager.register(URI_FETCH_CLIPBOARD, decorateWithLogging(new FetchClipboard()));
        manager.register(URI_LIST_FILES, decorateWithLogging(decorateWithParms(new ListFiles())));
        manager.register(URI_SAVE_FILE, decorateWithLogging(decorateWithParms(new SaveFile())));
        manager.register(URI_FETCH_FILE, decorateWithLogging(decorateWithParms(new FetchFile())));
        manager.register(URI_FETCH_FILE_TYPE, decorateWithLogging(decorateWithParms(new FetchFileType())));
        manager.register(URI_RENAME_FILE, decorateWithLogging(decorateWithParms(new RenameFile())));
        manager.register(URI_COPY_FILE, decorateWithLogging(decorateWithParms(new CopyFile())));
        manager.register(URI_DELETE_FILES, decorateWithLogging(decorateWithParms(new DeleteFiles())));
        manager.register(URI_RESTART_ROBOT, decorateWithLogging(new RestartRobot()));
        manager.register("/css/colors.less", decorateWithLogging(manager.getRegisteredHandler("/css/colors.less")));
        manager.register(RobotControllerWebHandlers.URI_RC_CONFIG, new RobotControllerConfiguration());
    }

    static String getFirstNamedParameter(NanoHTTPD.IHTTPSession session, String name) {
        Map<String, List<String>> parameters = session.getParameters();
        if (parameters.containsKey(name)) {
            return parameters.get(name).get(0);
        }
        return null;
    }

    private WebHandler decorateWithParms(WebHandler delegate) {
        return new SessionParametersGenerator(delegate);
    }
}
