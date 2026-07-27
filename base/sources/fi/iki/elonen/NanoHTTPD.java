package fi.iki.elonen;

import com.qualcomm.ftccommon.configuration.RobotConfigFileManager;
import com.sun.tools.javac.jvm.ByteCodes;
import java.io.BufferedInputStream;
import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.Closeable;
import java.io.DataOutput;
import java.io.DataOutputStream;
import java.io.File;
import java.io.FileOutputStream;
import java.io.FilterOutputStream;
import java.io.IOException;
import java.io.InputStream;
import java.io.InputStreamReader;
import java.io.OutputStream;
import java.io.OutputStreamWriter;
import java.io.PrintWriter;
import java.io.RandomAccessFile;
import java.io.UnsupportedEncodingException;
import java.io.Writer;
import java.net.InetAddress;
import java.net.InetSocketAddress;
import java.net.ServerSocket;
import java.net.Socket;
import java.net.SocketException;
import java.net.SocketTimeoutException;
import java.net.URL;
import java.net.URLDecoder;
import java.nio.ByteBuffer;
import java.nio.channels.FileChannel;
import java.nio.charset.Charset;
import java.security.KeyStore;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Calendar;
import java.util.Collections;
import java.util.Date;
import java.util.Enumeration;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.Properties;
import java.util.StringTokenizer;
import java.util.TimeZone;
import java.util.logging.Level;
import java.util.logging.Logger;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.util.zip.GZIPOutputStream;
import javax.net.ssl.KeyManager;
import javax.net.ssl.KeyManagerFactory;
import javax.net.ssl.SSLContext;
import javax.net.ssl.SSLException;
import javax.net.ssl.SSLServerSocket;
import javax.net.ssl.SSLServerSocketFactory;
import javax.net.ssl.TrustManagerFactory;
import org.firstinspires.ftc.robotserver.internal.webserver.RobotControllerWebHandlers;
import org.opencv.videoio.Videoio;

/* JADX INFO: loaded from: classes.dex */
public abstract class NanoHTTPD {
    public static final String MIME_HTML = "text/html";
    public static final String MIME_PLAINTEXT = "text/plain";
    protected static Map<String, String> MIME_TYPES = null;
    private static final String QUERY_STRING_PARAMETER = "NanoHttpd.QUERY_STRING";
    public static final int SOCKET_READ_TIMEOUT = 5000;
    protected AsyncRunner asyncRunner;
    private final String hostname;
    private final int myPort;
    private volatile ServerSocket myServerSocket;
    private Thread myThread;
    private ServerSocketFactory serverSocketFactory;
    private TempFileManagerFactory tempFileManagerFactory;
    private static final String CONTENT_DISPOSITION_REGEX = "([ |\t]*Content-Disposition[ |\t]*:)(.*)";
    private static final Pattern CONTENT_DISPOSITION_PATTERN = Pattern.compile(CONTENT_DISPOSITION_REGEX, 2);
    private static final String CONTENT_TYPE_REGEX = "([ |\t]*content-type[ |\t]*:)(.*)";
    private static final Pattern CONTENT_TYPE_PATTERN = Pattern.compile(CONTENT_TYPE_REGEX, 2);
    private static final String CONTENT_DISPOSITION_ATTRIBUTE_REGEX = "[ |\t]*([a-zA-Z]*)[ |\t]*=[ |\t]*['|\"]([^\"^']*)['|\"]";
    private static final Pattern CONTENT_DISPOSITION_ATTRIBUTE_PATTERN = Pattern.compile(CONTENT_DISPOSITION_ATTRIBUTE_REGEX);
    private static final Logger LOG = Logger.getLogger(NanoHTTPD.class.getName());

    public interface AsyncRunner {
        void closeAll();

        void closed(ClientHandler clientHandler);

        void exec(ClientHandler clientHandler);
    }

    public interface IHTTPSession {
        void execute() throws IOException;

        CookieHandler getCookies();

        Map<String, String> getHeaders();

        InputStream getInputStream();

        Method getMethod();

        Map<String, List<String>> getParameters();

        @Deprecated
        Map<String, String> getParms();

        String getQueryParameterString();

        String getRemoteHostName();

        String getRemoteIpAddress();

        String getUri();

        void parseBody(Map<String, String> map) throws ResponseException, IOException;
    }

    public interface ServerSocketFactory {
        ServerSocket create() throws IOException;
    }

    public interface TempFile {
        void delete() throws Exception;

        String getName();

        OutputStream open() throws Exception;
    }

    public interface TempFileManager {
        void clear();

        TempFile createTempFile(String str) throws Exception;
    }

    public interface TempFileManagerFactory {
        TempFileManager create();
    }

    public class ClientHandler implements Runnable {
        private final Socket acceptSocket;
        private final InputStream inputStream;

        public ClientHandler(InputStream inputStream, Socket acceptSocket) {
            this.inputStream = inputStream;
            this.acceptSocket = acceptSocket;
        }

        public void close() {
            NanoHTTPD.safeClose(this.inputStream);
            NanoHTTPD.safeClose(this.acceptSocket);
        }

        @Override // java.lang.Runnable
        public void run() throws Throwable {
            OutputStream outputStream;
            Throwable th;
            Exception e;
            try {
                outputStream = this.acceptSocket.getOutputStream();
                try {
                    try {
                        TempFileManager tempFileManager = NanoHTTPD.this.tempFileManagerFactory.create();
                        HTTPSession session = NanoHTTPD.this.new HTTPSession(tempFileManager, this.inputStream, outputStream, this.acceptSocket.getInetAddress());
                        while (!this.acceptSocket.isClosed()) {
                            session.execute();
                        }
                    } catch (Exception e2) {
                        e = e2;
                        if ((!(e instanceof SocketException) || !"NanoHttpd Shutdown".equals(e.getMessage())) && !(e instanceof SocketTimeoutException)) {
                            NanoHTTPD.LOG.log(Level.SEVERE, "Communication with the client broken, or an bug in the handler code", (Throwable) e);
                        }
                    }
                } catch (Throwable th2) {
                    th = th2;
                    NanoHTTPD.safeClose(outputStream);
                    NanoHTTPD.safeClose(this.inputStream);
                    NanoHTTPD.safeClose(this.acceptSocket);
                    NanoHTTPD.this.asyncRunner.closed(this);
                    throw th;
                }
            } catch (Exception e3) {
                outputStream = null;
                e = e3;
            } catch (Throwable th3) {
                outputStream = null;
                th = th3;
                NanoHTTPD.safeClose(outputStream);
                NanoHTTPD.safeClose(this.inputStream);
                NanoHTTPD.safeClose(this.acceptSocket);
                NanoHTTPD.this.asyncRunner.closed(this);
                throw th;
            }
            NanoHTTPD.safeClose(outputStream);
            NanoHTTPD.safeClose(this.inputStream);
            NanoHTTPD.safeClose(this.acceptSocket);
            NanoHTTPD.this.asyncRunner.closed(this);
        }
    }

    public static class Cookie {
        protected final String e;
        protected final String n;
        protected final String v;

        public static String getHTTPTime(int days) {
            Calendar calendar = Calendar.getInstance();
            SimpleDateFormat dateFormat = new SimpleDateFormat("EEE, dd MMM yyyy HH:mm:ss z", Locale.US);
            dateFormat.setTimeZone(TimeZone.getTimeZone("GMT"));
            calendar.add(5, days);
            return dateFormat.format(calendar.getTime());
        }

        public Cookie(String name, String value) {
            this(name, value, 30);
        }

        public Cookie(String name, String value, int numDays) {
            this.n = name;
            this.v = value;
            this.e = getHTTPTime(numDays);
        }

        public Cookie(String name, String value, String expires) {
            this.n = name;
            this.v = value;
            this.e = expires;
        }

        public String getHTTPHeader() {
            return String.format("%s=%s; expires=%s", this.n, this.v, this.e);
        }
    }

    public class CookieHandler implements Iterable<String> {
        private final HashMap<String, String> cookies = new HashMap<>();
        private final ArrayList<Cookie> queue = new ArrayList<>();

        public CookieHandler(Map<String, String> httpHeaders) {
            String raw = httpHeaders.get("cookie");
            if (raw != null) {
                String[] tokens = raw.split(RobotConfigFileManager.FILE_LIST_COMMAND_DELIMITER);
                for (String token : tokens) {
                    String[] data = token.trim().split("=");
                    if (data.length == 2) {
                        this.cookies.put(data[0], data[1]);
                    }
                }
            }
        }

        public void delete(String name) {
            set(name, "-delete-", -30);
        }

        @Override // java.lang.Iterable
        public Iterator<String> iterator() {
            return this.cookies.keySet().iterator();
        }

        public String read(String name) {
            return this.cookies.get(name);
        }

        public void set(Cookie cookie) {
            this.queue.add(cookie);
        }

        public void set(String name, String value, int expires) {
            this.queue.add(new Cookie(name, value, Cookie.getHTTPTime(expires)));
        }

        public void unloadQueue(Response response) {
            for (Cookie cookie : this.queue) {
                response.addHeader("Set-Cookie", cookie.getHTTPHeader());
            }
        }
    }

    public static class DefaultAsyncRunner implements AsyncRunner {
        private long requestCount;
        private final List<ClientHandler> running = Collections.synchronizedList(new ArrayList());

        public List<ClientHandler> getRunning() {
            return this.running;
        }

        @Override // fi.iki.elonen.NanoHTTPD.AsyncRunner
        public void closeAll() {
            for (ClientHandler clientHandler : new ArrayList(this.running)) {
                clientHandler.close();
            }
        }

        @Override // fi.iki.elonen.NanoHTTPD.AsyncRunner
        public void closed(ClientHandler clientHandler) {
            this.running.remove(clientHandler);
        }

        @Override // fi.iki.elonen.NanoHTTPD.AsyncRunner
        public void exec(ClientHandler clientHandler) {
            this.requestCount++;
            Thread t = new Thread(clientHandler);
            t.setDaemon(true);
            t.setName("NanoHttpd Request Processor (#" + this.requestCount + ")");
            this.running.add(clientHandler);
            t.start();
        }
    }

    public static class DefaultTempFile implements TempFile {
        private final File file;
        private final OutputStream fstream;

        public DefaultTempFile(File tempdir) throws IOException {
            this.file = File.createTempFile("NanoHTTPD-", "", tempdir);
            this.fstream = new FileOutputStream(this.file);
        }

        @Override // fi.iki.elonen.NanoHTTPD.TempFile
        public void delete() throws Exception {
            NanoHTTPD.safeClose(this.fstream);
            if (!this.file.delete()) {
                throw new Exception("could not delete temporary file: " + this.file.getAbsolutePath());
            }
        }

        @Override // fi.iki.elonen.NanoHTTPD.TempFile
        public String getName() {
            return this.file.getAbsolutePath();
        }

        @Override // fi.iki.elonen.NanoHTTPD.TempFile
        public OutputStream open() throws Exception {
            return this.fstream;
        }
    }

    public static class DefaultTempFileManager implements TempFileManager {
        private final List<TempFile> tempFiles;
        private final File tmpdir = new File(System.getProperty("java.io.tmpdir"));

        public DefaultTempFileManager() {
            if (!this.tmpdir.exists()) {
                this.tmpdir.mkdirs();
            }
            this.tempFiles = new ArrayList();
        }

        @Override // fi.iki.elonen.NanoHTTPD.TempFileManager
        public void clear() {
            for (TempFile file : this.tempFiles) {
                try {
                    file.delete();
                } catch (Exception ignored) {
                    NanoHTTPD.LOG.log(Level.WARNING, "could not delete file ", (Throwable) ignored);
                }
            }
            this.tempFiles.clear();
        }

        @Override // fi.iki.elonen.NanoHTTPD.TempFileManager
        public TempFile createTempFile(String filename_hint) throws Exception {
            DefaultTempFile tempFile = new DefaultTempFile(this.tmpdir);
            this.tempFiles.add(tempFile);
            return tempFile;
        }
    }

    private class DefaultTempFileManagerFactory implements TempFileManagerFactory {
        private DefaultTempFileManagerFactory() {
        }

        @Override // fi.iki.elonen.NanoHTTPD.TempFileManagerFactory
        public TempFileManager create() {
            return new DefaultTempFileManager();
        }
    }

    public static class DefaultServerSocketFactory implements ServerSocketFactory {
        @Override // fi.iki.elonen.NanoHTTPD.ServerSocketFactory
        public ServerSocket create() throws IOException {
            return new ServerSocket();
        }
    }

    public static class SecureServerSocketFactory implements ServerSocketFactory {
        private String[] sslProtocols;
        private SSLServerSocketFactory sslServerSocketFactory;

        public SecureServerSocketFactory(SSLServerSocketFactory sslServerSocketFactory, String[] sslProtocols) {
            this.sslServerSocketFactory = sslServerSocketFactory;
            this.sslProtocols = sslProtocols;
        }

        @Override // fi.iki.elonen.NanoHTTPD.ServerSocketFactory
        public ServerSocket create() throws IOException {
            SSLServerSocket ss = (SSLServerSocket) this.sslServerSocketFactory.createServerSocket();
            if (this.sslProtocols != null) {
                ss.setEnabledProtocols(this.sslProtocols);
            } else {
                ss.setEnabledProtocols(ss.getSupportedProtocols());
            }
            ss.setUseClientMode(false);
            ss.setWantClientAuth(false);
            ss.setNeedClientAuth(false);
            return ss;
        }
    }

    protected static class ContentType {
        private static final String ASCII_ENCODING = "US-ASCII";
        private static final String MULTIPART_FORM_DATA_HEADER = "multipart/form-data";
        private static final String UTF8_ENCODING = "UTF-8";
        private final String boundary;
        private final String contentType;
        private final String contentTypeHeader;
        private final String encoding;
        private static final String CONTENT_REGEX = "[ |\t]*([^/^ ^;^,]+/[^ ^;^,]+)";
        private static final Pattern MIME_PATTERN = Pattern.compile(CONTENT_REGEX, 2);
        private static final String CHARSET_REGEX = "[ |\t]*(charset)[ |\t]*=[ |\t]*['|\"]?([^\"^'^;^,]*)['|\"]?";
        private static final Pattern CHARSET_PATTERN = Pattern.compile(CHARSET_REGEX, 2);
        private static final String BOUNDARY_REGEX = "[ |\t]*(boundary)[ |\t]*=[ |\t]*['|\"]?([^\"^'^;^,]*)['|\"]?";
        private static final Pattern BOUNDARY_PATTERN = Pattern.compile(BOUNDARY_REGEX, 2);

        public ContentType(String contentTypeHeader) {
            this.contentTypeHeader = contentTypeHeader;
            if (contentTypeHeader != null) {
                this.contentType = getDetailFromContentHeader(contentTypeHeader, MIME_PATTERN, "", 1);
                String charsetFromHeader = getDetailFromContentHeader(contentTypeHeader, CHARSET_PATTERN, null, 2);
                if (charsetFromHeader != null && !charsetFromHeader.equals(UTF8_ENCODING)) {
                    NanoHTTPD.LOG.log(Level.WARNING, "A charset other than UTF-8 was specified in the server response, however the server will respond with UTF-8");
                }
                this.encoding = UTF8_ENCODING;
            } else {
                this.contentType = "";
                this.encoding = UTF8_ENCODING;
            }
            if (MULTIPART_FORM_DATA_HEADER.equalsIgnoreCase(this.contentType)) {
                this.boundary = getDetailFromContentHeader(contentTypeHeader, BOUNDARY_PATTERN, null, 2);
            } else {
                this.boundary = null;
            }
        }

        private String getDetailFromContentHeader(String contentTypeHeader, Pattern pattern, String defaultValue, int group) {
            Matcher matcher = pattern.matcher(contentTypeHeader);
            return matcher.find() ? matcher.group(group) : defaultValue;
        }

        public String getContentTypeHeader() {
            return this.contentTypeHeader;
        }

        public String getContentType() {
            return this.contentType;
        }

        public String getEncoding() {
            return this.encoding == null ? UTF8_ENCODING : this.encoding;
        }

        public String getBoundary() {
            return this.boundary;
        }

        public boolean isMultipart() {
            return MULTIPART_FORM_DATA_HEADER.equalsIgnoreCase(this.contentType);
        }

        public ContentType tryUTF8() {
            if (this.encoding == null) {
                return new ContentType(this.contentTypeHeader + "; charset=UTF-8");
            }
            return this;
        }
    }

    protected class HTTPSession implements IHTTPSession {
        public static final int BUFSIZE = 8192;
        public static final int MAX_HEADER_SIZE = 1024;
        private static final int MEMORY_STORE_LIMIT = 1024;
        private static final int REQUEST_BUFFER_LEN = 512;
        private CookieHandler cookies;
        private Map<String, String> headers;
        private final BufferedInputStream inputStream;
        private Method method;
        private final OutputStream outputStream;
        private Map<String, List<String>> parms;
        private String protocolVersion;
        private String queryParameterString;
        private String remoteHostname;
        private String remoteIp;
        private int rlen;
        private int splitbyte;
        private final TempFileManager tempFileManager;
        private String uri;

        public HTTPSession(TempFileManager tempFileManager, InputStream inputStream, OutputStream outputStream) {
            this.tempFileManager = tempFileManager;
            this.inputStream = new BufferedInputStream(inputStream, 8192);
            this.outputStream = outputStream;
        }

        public HTTPSession(TempFileManager tempFileManager, InputStream inputStream, OutputStream outputStream, InetAddress inetAddress) {
            this.tempFileManager = tempFileManager;
            this.inputStream = new BufferedInputStream(inputStream, 8192);
            this.outputStream = outputStream;
            this.remoteIp = (inetAddress.isLoopbackAddress() || inetAddress.isAnyLocalAddress()) ? "127.0.0.1" : inetAddress.getHostAddress().toString();
            this.remoteHostname = (inetAddress.isLoopbackAddress() || inetAddress.isAnyLocalAddress()) ? "localhost" : inetAddress.getHostName().toString();
            this.headers = new HashMap();
        }

        private void decodeHeader(BufferedReader in, Map<String, String> pre, Map<String, List<String>> parms, Map<String, String> headers) throws ResponseException {
            String uri;
            try {
                String inLine = in.readLine();
                if (inLine == null) {
                    return;
                }
                StringTokenizer st = new StringTokenizer(inLine);
                if (!st.hasMoreTokens()) {
                    throw new ResponseException(Response.Status.BAD_REQUEST, "BAD REQUEST: Syntax error. Usage: GET /example/file.html");
                }
                pre.put("method", st.nextToken());
                if (!st.hasMoreTokens()) {
                    throw new ResponseException(Response.Status.BAD_REQUEST, "BAD REQUEST: Missing URI. Usage: GET /example/file.html");
                }
                String uri2 = st.nextToken();
                int qmi = uri2.indexOf(63);
                if (qmi >= 0) {
                    decodeParms(uri2.substring(qmi + 1), parms);
                    uri = NanoHTTPD.decodePercent(uri2.substring(0, qmi));
                } else {
                    uri = NanoHTTPD.decodePercent(uri2);
                }
                if (st.hasMoreTokens()) {
                    this.protocolVersion = st.nextToken();
                } else {
                    this.protocolVersion = "HTTP/1.1";
                    NanoHTTPD.LOG.log(Level.FINE, "no protocol version specified, strange. Assuming HTTP/1.1.");
                }
                String line = in.readLine();
                while (line != null && !line.trim().isEmpty()) {
                    int p = line.indexOf(58);
                    if (p >= 0) {
                        headers.put(line.substring(0, p).trim().toLowerCase(Locale.US), line.substring(p + 1).trim());
                    }
                    line = in.readLine();
                }
                pre.put("uri", uri);
            } catch (IOException ioe) {
                throw new ResponseException(Response.Status.INTERNAL_ERROR, "SERVER INTERNAL ERROR: IOException: " + ioe.getMessage(), ioe);
            }
        }

        private void decodeMultipartFormData(ContentType contentType, ByteBuffer fbuf, Map<String, List<String>> parms, Map<String, String> files) throws ResponseException, IOException {
            int pcount;
            List<String> values;
            int[] boundaryIdxs;
            int i;
            Map<String, List<String>> map = parms;
            int pcount2 = 0;
            try {
                int[] boundaryIdxs2 = getBoundaryPositions(fbuf, contentType.getBoundary().getBytes());
                int i2 = 2;
                if (boundaryIdxs2.length < 2) {
                    throw new ResponseException(Response.Status.BAD_REQUEST, "BAD REQUEST: Content type is multipart/form-data but contains less than two boundary strings.");
                }
                int i3 = 1024;
                byte[] partHeaderBuff = new byte[1024];
                int boundaryIdx = 0;
                while (boundaryIdx < boundaryIdxs2.length - 1) {
                    fbuf.position(boundaryIdxs2[boundaryIdx]);
                    int len = fbuf.remaining() < i3 ? fbuf.remaining() : i3;
                    fbuf.get(partHeaderBuff, 0, len);
                    BufferedReader in = new BufferedReader(new InputStreamReader(new ByteArrayInputStream(partHeaderBuff, 0, len), Charset.forName(contentType.getEncoding())), len);
                    String mpline = in.readLine();
                    int headerLines = 0 + 1;
                    if (mpline == null || !mpline.contains(contentType.getBoundary())) {
                        throw new ResponseException(Response.Status.BAD_REQUEST, "BAD REQUEST: Content type is multipart/form-data but chunk does not start with boundary.");
                    }
                    String partName = null;
                    String partContentType = null;
                    String mpline2 = in.readLine();
                    int headerLines2 = headerLines + 1;
                    String fileName = null;
                    while (mpline2 != null) {
                        try {
                            if (mpline2.trim().length() <= 0) {
                                break;
                            }
                            Matcher matcher = NanoHTTPD.CONTENT_DISPOSITION_PATTERN.matcher(mpline2);
                            if (matcher.matches()) {
                                String attributeString = matcher.group(i2);
                                pcount = pcount2;
                                String key = attributeString;
                                try {
                                    Matcher matcher2 = NanoHTTPD.CONTENT_DISPOSITION_ATTRIBUTE_PATTERN.matcher(key);
                                    while (matcher2.find()) {
                                        String key2 = matcher2.group(1);
                                        String attributeString2 = key;
                                        if ("name".equalsIgnoreCase(key2)) {
                                            partName = matcher2.group(2);
                                        } else if (RobotControllerWebHandlers.PARAM_FILENAME.equalsIgnoreCase(key2)) {
                                            fileName = matcher2.group(2);
                                            if (!fileName.isEmpty()) {
                                                if (pcount > 0) {
                                                    int pcount3 = pcount + 1;
                                                    try {
                                                        partName = partName + String.valueOf(pcount);
                                                        pcount = pcount3;
                                                    } catch (ResponseException re) {
                                                        throw re;
                                                    } catch (Exception e) {
                                                        e = e;
                                                        throw new ResponseException(Response.Status.INTERNAL_ERROR, e.toString());
                                                    }
                                                } else {
                                                    pcount++;
                                                }
                                            }
                                        }
                                        key = attributeString2;
                                    }
                                    pcount2 = pcount;
                                } catch (ResponseException re2) {
                                    throw re2;
                                } catch (Exception e2) {
                                    e = e2;
                                }
                            }
                            Matcher matcher3 = NanoHTTPD.CONTENT_TYPE_PATTERN.matcher(mpline2);
                            if (matcher3.matches()) {
                                i = 2;
                                partContentType = matcher3.group(2).trim();
                            } else {
                                i = 2;
                            }
                            mpline2 = in.readLine();
                            headerLines2++;
                            i2 = i;
                        } catch (ResponseException re3) {
                            throw re3;
                        } catch (Exception e3) {
                            e = e3;
                        }
                    }
                    pcount = pcount2;
                    int partHeaderLength = 0;
                    while (true) {
                        int headerLines3 = headerLines2 - 1;
                        if (headerLines2 <= 0) {
                            break;
                        }
                        partHeaderLength = scipOverNewLine(partHeaderBuff, partHeaderLength);
                        headerLines2 = headerLines3;
                    }
                    if (partHeaderLength >= len - 4) {
                        throw new ResponseException(Response.Status.INTERNAL_ERROR, "Multipart header size exceeds MAX_HEADER_SIZE.");
                    }
                    int partDataStart = boundaryIdxs2[boundaryIdx] + partHeaderLength;
                    int partDataEnd = boundaryIdxs2[boundaryIdx + 1] - 4;
                    fbuf.position(partDataStart);
                    List<String> values2 = map.get(partName);
                    if (values2 == null) {
                        values = new ArrayList<>();
                        map.put(partName, values);
                    } else {
                        values = values2;
                    }
                    if (partContentType == null) {
                        boundaryIdxs = boundaryIdxs2;
                        byte[] data_bytes = new byte[partDataEnd - partDataStart];
                        fbuf.get(data_bytes);
                        values.add(new String(data_bytes, contentType.getEncoding()));
                    } else {
                        boundaryIdxs = boundaryIdxs2;
                        String path = saveTmpFile(fbuf, partDataStart, partDataEnd - partDataStart, fileName);
                        if (files.containsKey(partName)) {
                            int count = 2;
                            while (files.containsKey(partName + count)) {
                                count++;
                            }
                            files.put(partName + count, path);
                        } else {
                            files.put(partName, path);
                        }
                        values.add(fileName);
                    }
                    boundaryIdx++;
                    map = parms;
                    boundaryIdxs2 = boundaryIdxs;
                    pcount2 = pcount;
                    i3 = 1024;
                    i2 = 2;
                }
            } catch (ResponseException re4) {
                throw re4;
            } catch (Exception e4) {
                e = e4;
            }
        }

        private int scipOverNewLine(byte[] partHeaderBuff, int index) {
            while (partHeaderBuff[index] != 10) {
                index++;
            }
            return index + 1;
        }

        private void decodeParms(String parms, Map<String, List<String>> p) {
            String key;
            String value;
            if (parms == null) {
                this.queryParameterString = "";
                return;
            }
            this.queryParameterString = parms;
            StringTokenizer st = new StringTokenizer(parms, "&");
            while (st.hasMoreTokens()) {
                String e = st.nextToken();
                int sep = e.indexOf(61);
                if (sep >= 0) {
                    key = NanoHTTPD.decodePercent(e.substring(0, sep)).trim();
                    value = NanoHTTPD.decodePercent(e.substring(sep + 1));
                } else {
                    key = NanoHTTPD.decodePercent(e).trim();
                    value = "";
                }
                List<String> values = p.get(key);
                if (values == null) {
                    values = new ArrayList();
                    p.put(key, values);
                }
                values.add(value);
            }
        }

        @Override // fi.iki.elonen.NanoHTTPD.IHTTPSession
        public void execute() throws IOException {
            byte[] buf;
            boolean z;
            Response r = null;
            try {
                try {
                    try {
                        try {
                            buf = new byte[8192];
                            z = false;
                            this.splitbyte = 0;
                            this.rlen = 0;
                            this.inputStream.mark(8192);
                        } catch (SocketException e) {
                            throw e;
                        }
                    } catch (ResponseException re) {
                        Response resp = NanoHTTPD.newFixedLengthResponse(re.getStatus(), "text/plain", re.getMessage());
                        resp.send(this.outputStream);
                        NanoHTTPD.safeClose(this.outputStream);
                    } catch (SocketTimeoutException ste) {
                        throw ste;
                    }
                } catch (SSLException ssle) {
                    Response resp2 = NanoHTTPD.newFixedLengthResponse(Response.Status.INTERNAL_ERROR, "text/plain", "SSL PROTOCOL FAILURE: " + ssle.getMessage());
                    resp2.send(this.outputStream);
                    NanoHTTPD.safeClose(this.outputStream);
                } catch (IOException ioe) {
                    Response resp3 = NanoHTTPD.newFixedLengthResponse(Response.Status.INTERNAL_ERROR, "text/plain", "SERVER INTERNAL ERROR: IOException: " + ioe.getMessage());
                    resp3.send(this.outputStream);
                    NanoHTTPD.safeClose(this.outputStream);
                }
                try {
                    int read = this.inputStream.read(buf, 0, 8192);
                    if (read == -1) {
                        NanoHTTPD.safeClose(this.inputStream);
                        NanoHTTPD.safeClose(this.outputStream);
                        throw new SocketException("NanoHttpd Shutdown");
                    }
                    while (read > 0) {
                        this.rlen += read;
                        this.splitbyte = findHeaderEnd(buf, this.rlen);
                        if (this.splitbyte > 0) {
                            break;
                        } else {
                            read = this.inputStream.read(buf, this.rlen, 8192 - this.rlen);
                        }
                    }
                    if (this.splitbyte < this.rlen) {
                        this.inputStream.reset();
                        this.inputStream.skip(this.splitbyte);
                    }
                    this.parms = new HashMap();
                    if (this.headers == null) {
                        this.headers = new HashMap();
                    } else {
                        this.headers.clear();
                    }
                    this.queryParameterString = null;
                    BufferedReader hin = new BufferedReader(new InputStreamReader(new ByteArrayInputStream(buf, 0, this.rlen)));
                    Map<String, String> pre = new HashMap<>();
                    decodeHeader(hin, pre, this.parms, this.headers);
                    if (this.remoteIp != null) {
                        this.headers.put("remote-addr", this.remoteIp);
                        this.headers.put("http-client-ip", this.remoteIp);
                    }
                    this.method = Method.lookup(pre.get("method"));
                    if (this.method == null) {
                        throw new ResponseException(Response.Status.BAD_REQUEST, "BAD REQUEST: Syntax error. HTTP verb " + pre.get("method") + " unhandled.");
                    }
                    this.uri = pre.get("uri");
                    this.cookies = NanoHTTPD.this.new CookieHandler(this.headers);
                    String connection = this.headers.get(NanoWSD.HEADER_CONNECTION);
                    boolean keepAlive = "HTTP/1.1".equals(this.protocolVersion) && (connection == null || !connection.matches("(?i).*close.*"));
                    r = NanoHTTPD.this.serve(this);
                    if (r == null) {
                        throw new ResponseException(Response.Status.INTERNAL_ERROR, "SERVER INTERNAL ERROR: Serve() returned a null response.");
                    }
                    String acceptEncoding = this.headers.get("accept-encoding");
                    this.cookies.unloadQueue(r);
                    r.setRequestMethod(this.method);
                    if (NanoHTTPD.this.useGzipWhenAccepted(r) && acceptEncoding != null && acceptEncoding.contains("gzip")) {
                        z = true;
                    }
                    r.setGzipEncoding(z);
                    r.setKeepAlive(keepAlive);
                    r.send(this.outputStream);
                    if (!keepAlive || r.isCloseConnection()) {
                        throw new SocketException("NanoHttpd Shutdown");
                    }
                } catch (SSLException e2) {
                    throw e2;
                } catch (IOException e3) {
                    NanoHTTPD.safeClose(this.inputStream);
                    NanoHTTPD.safeClose(this.outputStream);
                    throw new SocketException("NanoHttpd Shutdown");
                }
            } finally {
                NanoHTTPD.safeClose(null);
                this.tempFileManager.clear();
            }
        }

        private int findHeaderEnd(byte[] buf, int rlen) {
            for (int splitbyte = 0; splitbyte + 1 < rlen; splitbyte++) {
                if (buf[splitbyte] == 13 && buf[splitbyte + 1] == 10 && splitbyte + 3 < rlen && buf[splitbyte + 2] == 13 && buf[splitbyte + 3] == 10) {
                    return splitbyte + 4;
                }
                if (buf[splitbyte] == 10 && buf[splitbyte + 1] == 10) {
                    return splitbyte + 2;
                }
            }
            return 0;
        }

        private int[] getBoundaryPositions(ByteBuffer b, byte[] boundary) {
            int[] res = new int[0];
            if (b.remaining() < boundary.length) {
                return res;
            }
            int search_window_pos = 0;
            byte[] search_window = new byte[boundary.length + 4096];
            int first_fill = b.remaining() < search_window.length ? b.remaining() : search_window.length;
            b.get(search_window, 0, first_fill);
            int new_bytes = first_fill - boundary.length;
            do {
                for (int j = 0; j < new_bytes; j++) {
                    for (int i = 0; i < boundary.length && search_window[j + i] == boundary[i]; i++) {
                        if (i == boundary.length - 1) {
                            int[] new_res = new int[res.length + 1];
                            System.arraycopy(res, 0, new_res, 0, res.length);
                            new_res[res.length] = search_window_pos + j;
                            res = new_res;
                        }
                    }
                }
                search_window_pos += new_bytes;
                System.arraycopy(search_window, search_window.length - boundary.length, search_window, 0, boundary.length);
                int new_bytes2 = search_window.length - boundary.length;
                int new_bytes3 = b.remaining();
                new_bytes = new_bytes3 < new_bytes2 ? b.remaining() : new_bytes2;
                int new_bytes4 = boundary.length;
                b.get(search_window, new_bytes4, new_bytes);
            } while (new_bytes > 0);
            return res;
        }

        @Override // fi.iki.elonen.NanoHTTPD.IHTTPSession
        public CookieHandler getCookies() {
            return this.cookies;
        }

        @Override // fi.iki.elonen.NanoHTTPD.IHTTPSession
        public final Map<String, String> getHeaders() {
            return this.headers;
        }

        @Override // fi.iki.elonen.NanoHTTPD.IHTTPSession
        public final InputStream getInputStream() {
            return this.inputStream;
        }

        @Override // fi.iki.elonen.NanoHTTPD.IHTTPSession
        public final Method getMethod() {
            return this.method;
        }

        @Override // fi.iki.elonen.NanoHTTPD.IHTTPSession
        @Deprecated
        public final Map<String, String> getParms() {
            Map<String, String> result = new HashMap<>();
            for (String key : this.parms.keySet()) {
                result.put(key, this.parms.get(key).get(0));
            }
            return result;
        }

        @Override // fi.iki.elonen.NanoHTTPD.IHTTPSession
        public final Map<String, List<String>> getParameters() {
            return this.parms;
        }

        @Override // fi.iki.elonen.NanoHTTPD.IHTTPSession
        public String getQueryParameterString() {
            return this.queryParameterString;
        }

        private RandomAccessFile getTmpBucket() {
            try {
                TempFile tempFile = this.tempFileManager.createTempFile(null);
                return new RandomAccessFile(tempFile.getName(), "rw");
            } catch (Exception e) {
                throw new Error(e);
            }
        }

        @Override // fi.iki.elonen.NanoHTTPD.IHTTPSession
        public final String getUri() {
            return this.uri;
        }

        public long getBodySize() {
            if (this.headers.containsKey("content-length")) {
                return Long.parseLong(this.headers.get("content-length"));
            }
            if (this.splitbyte < this.rlen) {
                return this.rlen - this.splitbyte;
            }
            return 0L;
        }

        @Override // fi.iki.elonen.NanoHTTPD.IHTTPSession
        public void parseBody(Map<String, String> files) throws ResponseException, IOException {
            DataOutput requestDataOutput;
            ByteBuffer fbuf;
            RandomAccessFile randomAccessFile = null;
            try {
                long size = getBodySize();
                ByteArrayOutputStream baos = null;
                if (size < 1024) {
                    baos = new ByteArrayOutputStream();
                    requestDataOutput = new DataOutputStream(baos);
                } else {
                    randomAccessFile = getTmpBucket();
                    requestDataOutput = randomAccessFile;
                }
                byte[] buf = new byte[512];
                while (this.rlen >= 0 && size > 0) {
                    this.rlen = this.inputStream.read(buf, 0, (int) Math.min(size, 512L));
                    size -= (long) this.rlen;
                    if (this.rlen > 0) {
                        requestDataOutput.write(buf, 0, this.rlen);
                    }
                }
                if (baos != null) {
                    fbuf = ByteBuffer.wrap(baos.toByteArray(), 0, baos.size());
                } else {
                    fbuf = randomAccessFile.getChannel().map(FileChannel.MapMode.READ_ONLY, 0L, randomAccessFile.length());
                    randomAccessFile.seek(0L);
                }
                if (!Method.POST.equals(this.method)) {
                    if (Method.PUT.equals(this.method)) {
                        files.put("content", saveTmpFile(fbuf, 0, fbuf.limit(), null));
                    }
                } else {
                    ContentType contentType = new ContentType(this.headers.get("content-type"));
                    if (contentType.isMultipart()) {
                        String boundary = contentType.getBoundary();
                        if (boundary != null) {
                            decodeMultipartFormData(contentType, fbuf, this.parms, files);
                        } else {
                            throw new ResponseException(Response.Status.BAD_REQUEST, "BAD REQUEST: Content type is multipart/form-data but boundary missing. Usage: GET /example/file.html");
                        }
                    } else {
                        byte[] postBytes = new byte[fbuf.remaining()];
                        fbuf.get(postBytes);
                        String postLine = new String(postBytes, contentType.getEncoding()).trim();
                        if ("application/x-www-form-urlencoded".equalsIgnoreCase(contentType.getContentType())) {
                            decodeParms(postLine, this.parms);
                        } else if (postLine.length() != 0) {
                            files.put("postData", postLine);
                        }
                    }
                }
            } finally {
                NanoHTTPD.safeClose(null);
            }
        }

        private String saveTmpFile(ByteBuffer b, int offset, int len, String filename_hint) {
            if (len <= 0) {
                return "";
            }
            FileOutputStream fileOutputStream = null;
            try {
                try {
                    TempFile tempFile = this.tempFileManager.createTempFile(filename_hint);
                    ByteBuffer src = b.duplicate();
                    fileOutputStream = new FileOutputStream(tempFile.getName());
                    FileChannel dest = fileOutputStream.getChannel();
                    src.position(offset).limit(offset + len);
                    dest.write(src.slice());
                    String path = tempFile.getName();
                    return path;
                } catch (Exception e) {
                    throw new Error(e);
                }
            } finally {
                NanoHTTPD.safeClose(fileOutputStream);
            }
        }

        @Override // fi.iki.elonen.NanoHTTPD.IHTTPSession
        public String getRemoteIpAddress() {
            return this.remoteIp;
        }

        @Override // fi.iki.elonen.NanoHTTPD.IHTTPSession
        public String getRemoteHostName() {
            return this.remoteHostname;
        }
    }

    public enum Method {
        GET,
        PUT,
        POST,
        DELETE,
        HEAD,
        OPTIONS,
        TRACE,
        CONNECT,
        PATCH,
        PROPFIND,
        PROPPATCH,
        MKCOL,
        MOVE,
        COPY,
        LOCK,
        UNLOCK;

        static Method lookup(String method) {
            if (method == null) {
                return null;
            }
            try {
                return valueOf(method);
            } catch (IllegalArgumentException e) {
                return null;
            }
        }
    }

    public static class Response implements Closeable {
        private boolean chunkedTransfer;
        private long contentLength;
        private InputStream data;
        private boolean encodeAsGzip;
        private boolean keepAlive;
        private String mimeType;
        private Method requestMethod;
        private IStatus status;
        private final Map<String, String> header = new HashMap<String, String>() { // from class: fi.iki.elonen.NanoHTTPD.Response.1
            @Override // java.util.HashMap, java.util.AbstractMap, java.util.Map
            public String put(String key, String value) {
                Response.this.lowerCaseHeader.put(key == null ? key : key.toLowerCase(), value);
                return (String) super.put(key, value);
            }
        };
        private final Map<String, String> lowerCaseHeader = new HashMap();

        public interface IStatus {
            String getDescription();

            int getRequestStatus();
        }

        public enum Status implements IStatus {
            SWITCH_PROTOCOL(101, "Switching Protocols"),
            OK(200, "OK"),
            CREATED(ByteCodes.jsr_w, "Created"),
            ACCEPTED(ByteCodes.breakpoint, "Accepted"),
            NO_CONTENT(204, "No Content"),
            PARTIAL_CONTENT(206, "Partial Content"),
            MULTI_STATUS(207, "Multi-Status"),
            REDIRECT(Videoio.CAP_PROP_PVAPI_FRAMESTARTTRIGGERMODE, "Moved Permanently"),
            FOUND(Videoio.CAP_PROP_PVAPI_DECIMATIONHORIZONTAL, "Found"),
            REDIRECT_SEE_OTHER(Videoio.CAP_PROP_PVAPI_DECIMATIONVERTICAL, "See Other"),
            NOT_MODIFIED(Videoio.CAP_PROP_PVAPI_BINNINGX, "Not Modified"),
            TEMPORARY_REDIRECT(307, "Temporary Redirect"),
            BAD_REQUEST(400, "Bad Request"),
            UNAUTHORIZED(Videoio.CAP_PROP_XI_DATA_FORMAT, "Unauthorized"),
            FORBIDDEN(Videoio.CAP_PROP_XI_OFFSET_Y, "Forbidden"),
            NOT_FOUND(Videoio.CAP_PROP_XI_TRG_SOURCE, "Not Found"),
            METHOD_NOT_ALLOWED(Videoio.CAP_PROP_XI_TRG_SOFTWARE, "Method Not Allowed"),
            NOT_ACCEPTABLE(Videoio.CAP_PROP_XI_GPI_SELECTOR, "Not Acceptable"),
            REQUEST_TIMEOUT(Videoio.CAP_PROP_XI_GPI_LEVEL, "Request Timeout"),
            CONFLICT(Videoio.CAP_PROP_XI_GPO_SELECTOR, "Conflict"),
            GONE(Videoio.CAP_PROP_XI_GPO_MODE, "Gone"),
            LENGTH_REQUIRED(Videoio.CAP_PROP_XI_LED_SELECTOR, "Length Required"),
            PRECONDITION_FAILED(Videoio.CAP_PROP_XI_LED_MODE, "Precondition Failed"),
            PAYLOAD_TOO_LARGE(Videoio.CAP_PROP_XI_MANUAL_WB, "Payload Too Large"),
            UNSUPPORTED_MEDIA_TYPE(Videoio.CAP_PROP_XI_AEAG, "Unsupported Media Type"),
            RANGE_NOT_SATISFIABLE(Videoio.CAP_PROP_XI_EXP_PRIORITY, "Requested Range Not Satisfiable"),
            EXPECTATION_FAILED(Videoio.CAP_PROP_XI_AE_MAX_LIMIT, "Expectation Failed"),
            TOO_MANY_REQUESTS(Videoio.CAP_PROP_XI_BINNING_HORIZONTAL, "Too Many Requests"),
            INTERNAL_ERROR(Videoio.CAP_QT, "Internal Server Error"),
            NOT_IMPLEMENTED(501, "Not Implemented"),
            SERVICE_UNAVAILABLE(503, "Service Unavailable"),
            UNSUPPORTED_HTTP_VERSION(505, "HTTP Version Not Supported");

            private final String description;
            private final int requestStatus;

            Status(int requestStatus, String description) {
                this.requestStatus = requestStatus;
                this.description = description;
            }

            public static Status lookup(int requestStatus) {
                for (Status status : values()) {
                    if (status.getRequestStatus() == requestStatus) {
                        return status;
                    }
                }
                return null;
            }

            @Override // fi.iki.elonen.NanoHTTPD.Response.IStatus
            public String getDescription() {
                return "" + this.requestStatus + " " + this.description;
            }

            @Override // fi.iki.elonen.NanoHTTPD.Response.IStatus
            public int getRequestStatus() {
                return this.requestStatus;
            }
        }

        private static class ChunkedOutputStream extends FilterOutputStream {
            public ChunkedOutputStream(OutputStream out) {
                super(out);
            }

            @Override // java.io.FilterOutputStream, java.io.OutputStream
            public void write(int b) throws IOException {
                byte[] data = {(byte) b};
                write(data, 0, 1);
            }

            @Override // java.io.FilterOutputStream, java.io.OutputStream
            public void write(byte[] b) throws IOException {
                write(b, 0, b.length);
            }

            @Override // java.io.FilterOutputStream, java.io.OutputStream
            public void write(byte[] b, int off, int len) throws IOException {
                if (len == 0) {
                    return;
                }
                this.out.write(String.format("%x\r\n", Integer.valueOf(len)).getBytes());
                this.out.write(b, off, len);
                this.out.write("\r\n".getBytes());
            }

            public void finish() throws IOException {
                this.out.write("0\r\n\r\n".getBytes());
            }
        }

        protected Response(IStatus status, String mimeType, InputStream data, long totalBytes) {
            this.status = status;
            this.mimeType = mimeType;
            if (data == null) {
                this.data = new ByteArrayInputStream(new byte[0]);
                this.contentLength = 0L;
            } else {
                this.data = data;
                this.contentLength = totalBytes;
            }
            this.chunkedTransfer = this.contentLength < 0;
            this.keepAlive = true;
        }

        @Override // java.io.Closeable, java.lang.AutoCloseable
        public void close() throws IOException {
            if (this.data != null) {
                this.data.close();
            }
        }

        public void addHeader(String name, String value) {
            this.header.put(name, value);
        }

        public void closeConnection(boolean close) {
            if (close) {
                this.header.put(NanoWSD.HEADER_CONNECTION, "close");
            } else {
                this.header.remove(NanoWSD.HEADER_CONNECTION);
            }
        }

        public boolean isCloseConnection() {
            return "close".equals(getHeader(NanoWSD.HEADER_CONNECTION));
        }

        public InputStream getData() {
            return this.data;
        }

        public String getHeader(String name) {
            return this.lowerCaseHeader.get(name.toLowerCase());
        }

        public String getMimeType() {
            return this.mimeType;
        }

        public Method getRequestMethod() {
            return this.requestMethod;
        }

        public IStatus getStatus() {
            return this.status;
        }

        public void setGzipEncoding(boolean encodeAsGzip) {
            this.encodeAsGzip = encodeAsGzip;
        }

        public void setKeepAlive(boolean useKeepAlive) {
            this.keepAlive = useKeepAlive;
        }

        protected void send(OutputStream outputStream) {
            SimpleDateFormat gmtFrmt = new SimpleDateFormat("E, d MMM yyyy HH:mm:ss 'GMT'", Locale.US);
            gmtFrmt.setTimeZone(TimeZone.getTimeZone("GMT"));
            try {
                if (this.status == null) {
                    throw new Error("sendResponse(): Status can't be null.");
                }
                PrintWriter pw = new PrintWriter((Writer) new BufferedWriter(new OutputStreamWriter(outputStream, new ContentType(this.mimeType).getEncoding())), false);
                pw.append("HTTP/1.1 ").append((CharSequence) this.status.getDescription()).append(" \r\n");
                if (this.mimeType != null) {
                    printHeader(pw, "Content-Type", this.mimeType);
                }
                if (getHeader("date") == null) {
                    printHeader(pw, "Date", gmtFrmt.format(new Date()));
                }
                for (Map.Entry<String, String> entry : this.header.entrySet()) {
                    printHeader(pw, entry.getKey(), entry.getValue());
                }
                if (getHeader(NanoWSD.HEADER_CONNECTION) == null) {
                    printHeader(pw, "Connection", this.keepAlive ? "keep-alive" : "close");
                }
                if (getHeader("content-length") != null) {
                    this.encodeAsGzip = false;
                }
                if (this.encodeAsGzip) {
                    printHeader(pw, "Content-Encoding", "gzip");
                    setChunkedTransfer(true);
                }
                long pending = this.data != null ? this.contentLength : 0L;
                if (this.requestMethod != Method.HEAD && this.chunkedTransfer) {
                    printHeader(pw, "Transfer-Encoding", "chunked");
                } else if (!this.encodeAsGzip) {
                    pending = sendContentLengthHeaderIfNotAlreadyPresent(pw, pending);
                }
                pw.append("\r\n");
                pw.flush();
                sendBodyWithCorrectTransferAndEncoding(outputStream, pending);
                outputStream.flush();
                NanoHTTPD.safeClose(this.data);
            } catch (IOException ioe) {
                NanoHTTPD.LOG.log(Level.SEVERE, "Could not send response to the client", (Throwable) ioe);
            }
        }

        protected void printHeader(PrintWriter pw, String key, String value) {
            pw.append((CharSequence) key).append(": ").append((CharSequence) value).append("\r\n");
        }

        protected long sendContentLengthHeaderIfNotAlreadyPresent(PrintWriter pw, long defaultSize) {
            String contentLengthString = getHeader("content-length");
            long size = defaultSize;
            if (contentLengthString != null) {
                try {
                    size = Long.parseLong(contentLengthString);
                } catch (NumberFormatException e) {
                    NanoHTTPD.LOG.severe("content-length was no number " + contentLengthString);
                }
            }
            pw.print("Content-Length: " + size + "\r\n");
            return size;
        }

        private void sendBodyWithCorrectTransferAndEncoding(OutputStream outputStream, long pending) throws IOException {
            if (this.requestMethod != Method.HEAD && this.chunkedTransfer) {
                ChunkedOutputStream chunkedOutputStream = new ChunkedOutputStream(outputStream);
                sendBodyWithCorrectEncoding(chunkedOutputStream, -1L);
                chunkedOutputStream.finish();
                return;
            }
            sendBodyWithCorrectEncoding(outputStream, pending);
        }

        private void sendBodyWithCorrectEncoding(OutputStream outputStream, long pending) throws IOException {
            if (this.encodeAsGzip) {
                GZIPOutputStream gzipOutputStream = new GZIPOutputStream(outputStream);
                sendBody(gzipOutputStream, -1L);
                gzipOutputStream.finish();
                return;
            }
            sendBody(outputStream, pending);
        }

        private void sendBody(OutputStream outputStream, long pending) throws IOException {
            byte[] buff = new byte[(int) 16384];
            boolean sendEverything = pending == -1;
            while (true) {
                if (pending > 0 || sendEverything) {
                    long bytesToRead = sendEverything ? 16384L : Math.min(pending, 16384L);
                    int read = this.data.read(buff, 0, (int) bytesToRead);
                    if (read > 0) {
                        outputStream.write(buff, 0, read);
                        if (!sendEverything) {
                            pending -= (long) read;
                        }
                    } else {
                        return;
                    }
                } else {
                    return;
                }
            }
        }

        public void setChunkedTransfer(boolean chunkedTransfer) {
            this.chunkedTransfer = chunkedTransfer;
        }

        public void setData(InputStream data) {
            this.data = data;
        }

        public void setMimeType(String mimeType) {
            this.mimeType = mimeType;
        }

        public void setRequestMethod(Method requestMethod) {
            this.requestMethod = requestMethod;
        }

        public void setStatus(IStatus status) {
            this.status = status;
        }
    }

    public static final class ResponseException extends Exception {
        private static final long serialVersionUID = 6569838532917408380L;
        private final Response.Status status;

        public ResponseException(Response.Status status, String message) {
            super(message);
            this.status = status;
        }

        public ResponseException(Response.Status status, String message, Exception e) {
            super(message, e);
            this.status = status;
        }

        public Response.Status getStatus() {
            return this.status;
        }
    }

    public class ServerRunnable implements Runnable {
        private IOException bindException;
        private boolean hasBinded = false;
        private final int timeout;

        public ServerRunnable(int timeout) {
            this.timeout = timeout;
        }

        @Override // java.lang.Runnable
        public void run() {
            try {
                NanoHTTPD.this.myServerSocket.bind(NanoHTTPD.this.hostname != null ? new InetSocketAddress(NanoHTTPD.this.hostname, NanoHTTPD.this.myPort) : new InetSocketAddress(NanoHTTPD.this.myPort));
                this.hasBinded = true;
                do {
                    try {
                        Socket finalAccept = NanoHTTPD.this.myServerSocket.accept();
                        if (this.timeout > 0) {
                            finalAccept.setSoTimeout(this.timeout);
                        }
                        InputStream inputStream = finalAccept.getInputStream();
                        NanoHTTPD.this.asyncRunner.exec(NanoHTTPD.this.createClientHandler(finalAccept, inputStream));
                    } catch (IOException e) {
                        NanoHTTPD.LOG.log(Level.FINE, "Communication with the client broken", (Throwable) e);
                    }
                } while (!NanoHTTPD.this.myServerSocket.isClosed());
            } catch (IOException e2) {
                this.bindException = e2;
            }
        }
    }

    public static Map<String, String> mimeTypes() {
        if (MIME_TYPES == null) {
            MIME_TYPES = new HashMap();
            loadMimeTypes(MIME_TYPES, "META-INF/nanohttpd/default-mimetypes.properties");
            loadMimeTypes(MIME_TYPES, "META-INF/nanohttpd/mimetypes.properties");
            if (MIME_TYPES.isEmpty()) {
                LOG.log(Level.WARNING, "no mime types found in the classpath! please provide mimetypes.properties");
            }
        }
        return MIME_TYPES;
    }

    private static void loadMimeTypes(Map<String, String> result, String resourceName) {
        try {
            Enumeration<URL> resources = NanoHTTPD.class.getClassLoader().getResources(resourceName);
            while (resources.hasMoreElements()) {
                URL url = resources.nextElement();
                Properties properties = new Properties();
                InputStream stream = null;
                try {
                    try {
                        stream = url.openStream();
                        properties.load(stream);
                        safeClose(stream);
                    } catch (IOException e) {
                        LOG.log(Level.SEVERE, "could not load mimetypes from " + url, (Throwable) e);
                        safeClose(stream);
                    }
                    result.putAll(properties);
                } catch (Throwable th) {
                    safeClose(stream);
                    throw th;
                }
            }
        } catch (IOException e2) {
            LOG.log(Level.INFO, "no mime types available at " + resourceName);
        }
    }

    public static SSLServerSocketFactory makeSSLSocketFactory(KeyStore loadedKeyStore, KeyManager[] keyManagers) throws IOException {
        try {
            TrustManagerFactory trustManagerFactory = TrustManagerFactory.getInstance(TrustManagerFactory.getDefaultAlgorithm());
            trustManagerFactory.init(loadedKeyStore);
            SSLContext ctx = SSLContext.getInstance("TLS");
            ctx.init(keyManagers, trustManagerFactory.getTrustManagers(), null);
            SSLServerSocketFactory res = ctx.getServerSocketFactory();
            return res;
        } catch (Exception e) {
            throw new IOException(e.getMessage());
        }
    }

    public static SSLServerSocketFactory makeSSLSocketFactory(KeyStore loadedKeyStore, KeyManagerFactory loadedKeyFactory) throws IOException {
        try {
            return makeSSLSocketFactory(loadedKeyStore, loadedKeyFactory.getKeyManagers());
        } catch (Exception e) {
            throw new IOException(e.getMessage());
        }
    }

    public static SSLServerSocketFactory makeSSLSocketFactory(String keyAndTrustStoreClasspathPath, char[] passphrase) throws IOException {
        try {
            KeyStore keystore = KeyStore.getInstance(KeyStore.getDefaultType());
            InputStream keystoreStream = NanoHTTPD.class.getResourceAsStream(keyAndTrustStoreClasspathPath);
            if (keystoreStream == null) {
                throw new IOException("Unable to load keystore from classpath: " + keyAndTrustStoreClasspathPath);
            }
            keystore.load(keystoreStream, passphrase);
            KeyManagerFactory keyManagerFactory = KeyManagerFactory.getInstance(KeyManagerFactory.getDefaultAlgorithm());
            keyManagerFactory.init(keystore, passphrase);
            return makeSSLSocketFactory(keystore, keyManagerFactory);
        } catch (Exception e) {
            throw new IOException(e.getMessage());
        }
    }

    public static String getMimeTypeForFile(String uri) {
        int dot = uri.lastIndexOf(46);
        String mime = null;
        if (dot >= 0) {
            String mime2 = mimeTypes().get(uri.substring(dot + 1).toLowerCase());
            mime = mime2;
        }
        return mime == null ? "application/octet-stream" : mime;
    }

    /* JADX INFO: Access modifiers changed from: private */
    public static final void safeClose(Object closeable) {
        if (closeable != null) {
            try {
                if (closeable instanceof Closeable) {
                    ((Closeable) closeable).close();
                } else if (closeable instanceof Socket) {
                    ((Socket) closeable).close();
                } else {
                    if (closeable instanceof ServerSocket) {
                        ((ServerSocket) closeable).close();
                        return;
                    }
                    throw new IllegalArgumentException("Unknown object to close");
                }
            } catch (IOException e) {
                LOG.log(Level.SEVERE, "Could not close", (Throwable) e);
            }
        }
    }

    public NanoHTTPD(int port) {
        this(null, port);
    }

    public NanoHTTPD(String hostname, int port) {
        this.serverSocketFactory = new DefaultServerSocketFactory();
        this.hostname = hostname;
        this.myPort = port;
        setTempFileManagerFactory(new DefaultTempFileManagerFactory());
        setAsyncRunner(new DefaultAsyncRunner());
    }

    public synchronized void closeAllConnections() {
        stop();
    }

    protected ClientHandler createClientHandler(Socket finalAccept, InputStream inputStream) {
        return new ClientHandler(inputStream, finalAccept);
    }

    protected ServerRunnable createServerRunnable(int timeout) {
        return new ServerRunnable(timeout);
    }

    protected static Map<String, List<String>> decodeParameters(Map<String, String> parms) {
        return decodeParameters(parms.get(QUERY_STRING_PARAMETER));
    }

    protected static Map<String, List<String>> decodeParameters(String queryString) {
        Map<String, List<String>> parms = new HashMap<>();
        if (queryString != null) {
            StringTokenizer st = new StringTokenizer(queryString, "&");
            while (st.hasMoreTokens()) {
                String e = st.nextToken();
                int sep = e.indexOf(61);
                String propertyName = (sep >= 0 ? decodePercent(e.substring(0, sep)) : decodePercent(e)).trim();
                if (!parms.containsKey(propertyName)) {
                    parms.put(propertyName, new ArrayList<>());
                }
                String propertyValue = sep >= 0 ? decodePercent(e.substring(sep + 1)) : null;
                if (propertyValue != null) {
                    parms.get(propertyName).add(propertyValue);
                }
            }
        }
        return parms;
    }

    protected static String decodePercent(String str) {
        try {
            String decoded = URLDecoder.decode(str, "UTF8");
            return decoded;
        } catch (UnsupportedEncodingException ignored) {
            LOG.log(Level.WARNING, "Encoding not supported, ignored", (Throwable) ignored);
            return null;
        }
    }

    protected boolean useGzipWhenAccepted(Response r) {
        return r.getMimeType() != null && (r.getMimeType().toLowerCase().contains("text/") || r.getMimeType().toLowerCase().contains("/json"));
    }

    public final int getListeningPort() {
        if (this.myServerSocket == null) {
            return -1;
        }
        return this.myServerSocket.getLocalPort();
    }

    public final boolean isAlive() {
        return wasStarted() && !this.myServerSocket.isClosed() && this.myThread.isAlive();
    }

    public ServerSocketFactory getServerSocketFactory() {
        return this.serverSocketFactory;
    }

    public void setServerSocketFactory(ServerSocketFactory serverSocketFactory) {
        this.serverSocketFactory = serverSocketFactory;
    }

    public String getHostname() {
        return this.hostname;
    }

    public TempFileManagerFactory getTempFileManagerFactory() {
        return this.tempFileManagerFactory;
    }

    public void makeSecure(SSLServerSocketFactory sslServerSocketFactory, String[] sslProtocols) {
        this.serverSocketFactory = new SecureServerSocketFactory(sslServerSocketFactory, sslProtocols);
    }

    public static Response newChunkedResponse(Response.IStatus status, String mimeType, InputStream data) {
        return new Response(status, mimeType, data, -1L);
    }

    public static Response newFixedLengthResponse(Response.IStatus status, String mimeType, InputStream data, long totalBytes) {
        return new Response(status, mimeType, data, totalBytes);
    }

    public static Response newFixedLengthResponse(Response.IStatus status, String mimeType, String txt) {
        byte[] bytes;
        ContentType contentType = new ContentType(mimeType);
        if (txt == null) {
            return newFixedLengthResponse(status, mimeType, new ByteArrayInputStream(new byte[0]), 0L);
        }
        try {
            bytes = txt.getBytes(contentType.getEncoding());
        } catch (UnsupportedEncodingException e) {
            LOG.log(Level.SEVERE, "UTF-8 encoding problem, responding nothing", (Throwable) e);
            bytes = new byte[0];
        }
        return newFixedLengthResponse(status, contentType.getContentTypeHeader(), new ByteArrayInputStream(bytes), bytes.length);
    }

    public static Response newFixedLengthResponse(String msg) {
        return newFixedLengthResponse(Response.Status.OK, MIME_HTML, msg);
    }

    public Response serve(IHTTPSession session) {
        Map<String, String> files = new HashMap<>();
        Method method = session.getMethod();
        if (Method.PUT.equals(method) || Method.POST.equals(method)) {
            try {
                session.parseBody(files);
            } catch (ResponseException re) {
                return newFixedLengthResponse(re.getStatus(), "text/plain", re.getMessage());
            } catch (IOException ioe) {
                return newFixedLengthResponse(Response.Status.INTERNAL_ERROR, "text/plain", "SERVER INTERNAL ERROR: IOException: " + ioe.getMessage());
            }
        }
        Map<String, String> parms = session.getParms();
        parms.put(QUERY_STRING_PARAMETER, session.getQueryParameterString());
        return serve(session.getUri(), method, session.getHeaders(), parms, files);
    }

    @Deprecated
    public Response serve(String uri, Method method, Map<String, String> headers, Map<String, String> parms, Map<String, String> files) {
        return newFixedLengthResponse(Response.Status.NOT_FOUND, "text/plain", "Not Found");
    }

    public void setAsyncRunner(AsyncRunner asyncRunner) {
        this.asyncRunner = asyncRunner;
    }

    public void setTempFileManagerFactory(TempFileManagerFactory tempFileManagerFactory) {
        this.tempFileManagerFactory = tempFileManagerFactory;
    }

    public void start() throws IOException {
        start(SOCKET_READ_TIMEOUT);
    }

    public void start(int timeout) throws IOException {
        start(timeout, true);
    }

    public void start(int timeout, boolean daemon) throws IOException {
        this.myServerSocket = getServerSocketFactory().create();
        this.myServerSocket.setReuseAddress(true);
        ServerRunnable serverRunnable = createServerRunnable(timeout);
        this.myThread = new Thread(serverRunnable);
        this.myThread.setDaemon(daemon);
        this.myThread.setName("NanoHttpd Main Listener");
        this.myThread.start();
        while (!serverRunnable.hasBinded && serverRunnable.bindException == null) {
            try {
                Thread.sleep(10L);
            } catch (Throwable th) {
            }
        }
        if (serverRunnable.bindException != null) {
            throw serverRunnable.bindException;
        }
    }

    public void stop() {
        try {
            safeClose(this.myServerSocket);
            this.asyncRunner.closeAll();
            if (this.myThread != null) {
                this.myThread.join();
            }
        } catch (Exception e) {
            LOG.log(Level.SEVERE, "Could not stop all connections", (Throwable) e);
        }
    }

    public final boolean wasStarted() {
        return (this.myServerSocket == null || this.myThread == null) ? false : true;
    }
}
