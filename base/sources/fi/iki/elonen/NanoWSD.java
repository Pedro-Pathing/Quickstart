package fi.iki.elonen;

import com.sun.tools.doclint.DocLint;
import fi.iki.elonen.NanoHTTPD;
import java.io.EOFException;
import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.nio.charset.CharacterCodingException;
import java.nio.charset.Charset;
import java.security.MessageDigest;
import java.security.NoSuchAlgorithmException;
import java.util.Arrays;
import java.util.Iterator;
import java.util.LinkedList;
import java.util.List;
import java.util.Map;
import java.util.logging.Level;
import java.util.logging.Logger;
import org.firstinspires.ftc.robotcore.internal.ftdi.FtConstants;

/* JADX INFO: loaded from: classes.dex */
public abstract class NanoWSD extends NanoHTTPD {
    public static final String HEADER_CONNECTION = "connection";
    public static final String HEADER_CONNECTION_VALUE = "Upgrade";
    public static final String HEADER_UPGRADE = "upgrade";
    public static final String HEADER_UPGRADE_VALUE = "websocket";
    public static final String HEADER_WEBSOCKET_ACCEPT = "sec-websocket-accept";
    public static final String HEADER_WEBSOCKET_KEY = "sec-websocket-key";
    public static final String HEADER_WEBSOCKET_PROTOCOL = "sec-websocket-protocol";
    public static final String HEADER_WEBSOCKET_VERSION = "sec-websocket-version";
    public static final String HEADER_WEBSOCKET_VERSION_VALUE = "13";
    private static final String WEBSOCKET_KEY_MAGIC = "258EAFA5-E914-47DA-95CA-C5AB0DC85B11";
    private static final Logger LOG = Logger.getLogger(NanoWSD.class.getName());
    private static final char[] ALPHABET = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/".toCharArray();

    public enum State {
        UNCONNECTED,
        CONNECTING,
        OPEN,
        CLOSING,
        CLOSED
    }

    protected abstract WebSocket openWebSocket(NanoHTTPD.IHTTPSession iHTTPSession);

    public static abstract class WebSocket {
        private final NanoHTTPD.IHTTPSession handshakeRequest;
        private final InputStream in;
        private OutputStream out;
        private WebSocketFrame.OpCode continuousOpCode = null;
        private final List<WebSocketFrame> continuousFrames = new LinkedList();
        private State state = State.UNCONNECTED;
        private final NanoHTTPD.Response handshakeResponse = new NanoHTTPD.Response(NanoHTTPD.Response.Status.SWITCH_PROTOCOL, null, null, 0) { // from class: fi.iki.elonen.NanoWSD.WebSocket.1
            @Override // fi.iki.elonen.NanoHTTPD.Response
            protected void send(OutputStream out) {
                WebSocket.this.out = out;
                WebSocket.this.state = State.CONNECTING;
                super.send(out);
                WebSocket.this.state = State.OPEN;
                WebSocket.this.onOpen();
                WebSocket.this.readWebsocket();
            }
        };

        protected abstract void onClose(WebSocketFrame.CloseCode closeCode, String str, boolean z);

        protected abstract void onException(IOException iOException);

        protected abstract void onMessage(WebSocketFrame webSocketFrame);

        protected abstract void onOpen();

        protected abstract void onPong(WebSocketFrame webSocketFrame);

        public WebSocket(NanoHTTPD.IHTTPSession handshakeRequest) {
            this.handshakeRequest = handshakeRequest;
            this.in = handshakeRequest.getInputStream();
            this.handshakeResponse.addHeader(NanoWSD.HEADER_UPGRADE, NanoWSD.HEADER_UPGRADE_VALUE);
            this.handshakeResponse.addHeader(NanoWSD.HEADER_CONNECTION, NanoWSD.HEADER_CONNECTION_VALUE);
        }

        public boolean isOpen() {
            return this.state == State.OPEN;
        }

        protected void debugFrameReceived(WebSocketFrame frame) {
        }

        protected void debugFrameSent(WebSocketFrame frame) {
        }

        public void close(WebSocketFrame.CloseCode code, String reason, boolean initiatedByRemote) throws IOException {
            State oldState = this.state;
            this.state = State.CLOSING;
            if (oldState == State.OPEN) {
                sendFrame(new WebSocketFrame.CloseFrame(code, reason));
            } else {
                doClose(code, reason, initiatedByRemote);
            }
        }

        private void doClose(WebSocketFrame.CloseCode code, String reason, boolean initiatedByRemote) {
            if (this.state == State.CLOSED) {
                return;
            }
            if (this.in != null) {
                try {
                    this.in.close();
                } catch (IOException e) {
                    NanoWSD.LOG.log(Level.FINE, "close failed", (Throwable) e);
                }
            }
            if (this.out != null) {
                try {
                    this.out.close();
                } catch (IOException e2) {
                    NanoWSD.LOG.log(Level.FINE, "close failed", (Throwable) e2);
                }
            }
            this.state = State.CLOSED;
            onClose(code, reason, initiatedByRemote);
        }

        public NanoHTTPD.IHTTPSession getHandshakeRequest() {
            return this.handshakeRequest;
        }

        public NanoHTTPD.Response getHandshakeResponse() {
            return this.handshakeResponse;
        }

        private void handleCloseFrame(WebSocketFrame frame) throws IOException {
            WebSocketFrame.CloseCode code = WebSocketFrame.CloseCode.NormalClosure;
            String reason = "";
            if (frame instanceof WebSocketFrame.CloseFrame) {
                code = ((WebSocketFrame.CloseFrame) frame).getCloseCode();
                reason = ((WebSocketFrame.CloseFrame) frame).getCloseReason();
            }
            if (this.state == State.CLOSING) {
                doClose(code, reason, false);
            } else {
                close(code, reason, true);
            }
        }

        private void handleFrameFragment(WebSocketFrame frame) throws IOException {
            if (frame.getOpCode() != WebSocketFrame.OpCode.Continuation) {
                if (this.continuousOpCode != null) {
                    throw new WebSocketException(WebSocketFrame.CloseCode.ProtocolError, "Previous continuous frame sequence not completed.");
                }
                this.continuousOpCode = frame.getOpCode();
                this.continuousFrames.clear();
                this.continuousFrames.add(frame);
                return;
            }
            if (frame.isFin()) {
                if (this.continuousOpCode == null) {
                    throw new WebSocketException(WebSocketFrame.CloseCode.ProtocolError, "Continuous frame sequence was not started.");
                }
                this.continuousFrames.add(frame);
                onMessage(new WebSocketFrame(this.continuousOpCode, this.continuousFrames));
                this.continuousOpCode = null;
                this.continuousFrames.clear();
                return;
            }
            if (this.continuousOpCode == null) {
                throw new WebSocketException(WebSocketFrame.CloseCode.ProtocolError, "Continuous frame sequence was not started.");
            }
            this.continuousFrames.add(frame);
        }

        private void handleWebsocketFrame(WebSocketFrame frame) throws IOException {
            debugFrameReceived(frame);
            if (frame.getOpCode() == WebSocketFrame.OpCode.Close) {
                handleCloseFrame(frame);
                return;
            }
            if (frame.getOpCode() == WebSocketFrame.OpCode.Ping) {
                sendFrame(new WebSocketFrame(WebSocketFrame.OpCode.Pong, true, frame.getBinaryPayload()));
                return;
            }
            if (frame.getOpCode() == WebSocketFrame.OpCode.Pong) {
                onPong(frame);
                return;
            }
            if (!frame.isFin() || frame.getOpCode() == WebSocketFrame.OpCode.Continuation) {
                handleFrameFragment(frame);
            } else {
                if (this.continuousOpCode != null) {
                    throw new WebSocketException(WebSocketFrame.CloseCode.ProtocolError, "Continuous frame sequence not completed.");
                }
                if (frame.getOpCode() == WebSocketFrame.OpCode.Text || frame.getOpCode() == WebSocketFrame.OpCode.Binary) {
                    onMessage(frame);
                    return;
                }
                throw new WebSocketException(WebSocketFrame.CloseCode.ProtocolError, "Non control or continuous frame expected.");
            }
        }

        public void ping(byte[] payload) throws IOException {
            sendFrame(new WebSocketFrame(WebSocketFrame.OpCode.Ping, true, payload));
        }

        /* JADX INFO: Access modifiers changed from: private */
        public void readWebsocket() {
            while (true) {
                try {
                    try {
                        if (this.state != State.OPEN) {
                            break;
                        } else {
                            handleWebsocketFrame(WebSocketFrame.read(this.in));
                        }
                    } catch (CharacterCodingException e) {
                        onException(e);
                        doClose(WebSocketFrame.CloseCode.InvalidFramePayloadData, e.toString(), false);
                    } catch (IOException e2) {
                        onException(e2);
                        if (e2 instanceof WebSocketException) {
                            doClose(((WebSocketException) e2).getCode(), ((WebSocketException) e2).getReason(), false);
                        }
                    }
                } finally {
                    doClose(WebSocketFrame.CloseCode.InternalServerError, "Handler terminated without closing the connection.", false);
                }
            }
        }

        public void send(byte[] payload) throws IOException {
            sendFrame(new WebSocketFrame(WebSocketFrame.OpCode.Binary, true, payload));
        }

        public void send(String payload) throws IOException {
            sendFrame(new WebSocketFrame(WebSocketFrame.OpCode.Text, true, payload));
        }

        public synchronized void sendFrame(WebSocketFrame frame) throws IOException {
            debugFrameSent(frame);
            frame.write(this.out);
        }
    }

    public static class WebSocketException extends IOException {
        private static final long serialVersionUID = 1;
        private final WebSocketFrame.CloseCode code;
        private final String reason;

        public WebSocketException(WebSocketFrame.CloseCode code, String reason) {
            this(code, reason, null);
        }

        public WebSocketException(WebSocketFrame.CloseCode code, String reason, Exception cause) {
            super(code + ": " + reason, cause);
            this.code = code;
            this.reason = reason;
        }

        public WebSocketException(Exception cause) {
            this(WebSocketFrame.CloseCode.InternalServerError, cause.toString(), cause);
        }

        public WebSocketFrame.CloseCode getCode() {
            return this.code;
        }

        public String getReason() {
            return this.reason;
        }
    }

    public static class WebSocketFrame {
        public static final Charset TEXT_CHARSET = Charset.forName("UTF-8");
        private transient int _payloadLength;
        private transient String _payloadString;
        private boolean fin;
        private byte[] maskingKey;
        private OpCode opCode;
        private byte[] payload;

        public enum CloseCode {
            NormalClosure(1000),
            GoingAway(1001),
            ProtocolError(1002),
            UnsupportedData(1003),
            NoStatusRcvd(org.java_websocket.framing.CloseFrame.NOCODE),
            AbnormalClosure(1006),
            InvalidFramePayloadData(1007),
            PolicyViolation(1008),
            MessageTooBig(1009),
            MandatoryExt(1010),
            InternalServerError(1011),
            TLSHandshake(1015);

            private final int code;

            public static CloseCode find(int value) {
                CloseCode[] arr$ = values();
                for (CloseCode code : arr$) {
                    if (code.getValue() == value) {
                        return code;
                    }
                }
                return null;
            }

            CloseCode(int code) {
                this.code = code;
            }

            public int getValue() {
                return this.code;
            }
        }

        public static class CloseFrame extends WebSocketFrame {
            static final /* synthetic */ boolean $assertionsDisabled = false;
            private CloseCode _closeCode;
            private String _closeReason;

            private static byte[] generatePayload(CloseCode code, String closeReason) throws CharacterCodingException {
                if (code != null) {
                    byte[] reasonBytes = text2Binary(closeReason);
                    byte[] payload = new byte[reasonBytes.length + 2];
                    payload[0] = (byte) ((code.getValue() >> 8) & 255);
                    payload[1] = (byte) (code.getValue() & 255);
                    System.arraycopy(reasonBytes, 0, payload, 2, reasonBytes.length);
                    return payload;
                }
                return new byte[0];
            }

            public CloseFrame(CloseCode code, String closeReason) throws CharacterCodingException {
                super(OpCode.Close, true, generatePayload(code, closeReason));
            }

            private CloseFrame(WebSocketFrame wrap) throws CharacterCodingException {
                super(wrap);
                if (wrap.getOpCode() != OpCode.Close) {
                    throw new AssertionError();
                }
                if (wrap.getBinaryPayload().length >= 2) {
                    this._closeCode = CloseCode.find(((wrap.getBinaryPayload()[0] & 255) << 8) | (wrap.getBinaryPayload()[1] & 255));
                    this._closeReason = binary2Text(getBinaryPayload(), 2, getBinaryPayload().length - 2);
                }
            }

            public CloseCode getCloseCode() {
                return this._closeCode;
            }

            public String getCloseReason() {
                return this._closeReason;
            }
        }

        public enum OpCode {
            Continuation(0),
            Text(1),
            Binary(2),
            Close(8),
            Ping(9),
            Pong(10);

            private final byte code;

            public static OpCode find(byte value) {
                OpCode[] arr$ = values();
                for (OpCode opcode : arr$) {
                    if (opcode.getValue() == value) {
                        return opcode;
                    }
                }
                return null;
            }

            OpCode(int code) {
                this.code = (byte) code;
            }

            public byte getValue() {
                return this.code;
            }

            public boolean isControlFrame() {
                return this == Close || this == Ping || this == Pong;
            }
        }

        public static String binary2Text(byte[] payload) throws CharacterCodingException {
            return new String(payload, TEXT_CHARSET);
        }

        public static String binary2Text(byte[] payload, int offset, int length) throws CharacterCodingException {
            return new String(payload, offset, length, TEXT_CHARSET);
        }

        private static int checkedRead(int read) throws IOException {
            if (read < 0) {
                throw new EOFException();
            }
            return read;
        }

        public static WebSocketFrame read(InputStream in) throws IOException {
            byte head = (byte) checkedRead(in.read());
            boolean fin = (head & FtConstants.DCD) != 0;
            OpCode opCode = OpCode.find((byte) (head & 15));
            if ((head & 112) != 0) {
                throw new WebSocketException(CloseCode.ProtocolError, "The reserved bits (" + Integer.toBinaryString(head & 112) + ") must be 0.");
            }
            if (opCode == null) {
                throw new WebSocketException(CloseCode.ProtocolError, "Received frame with reserved/unknown opcode " + (head & 15) + ".");
            }
            if (opCode.isControlFrame() && !fin) {
                throw new WebSocketException(CloseCode.ProtocolError, "Fragmented control frame.");
            }
            WebSocketFrame frame = new WebSocketFrame(opCode, fin);
            frame.readPayloadInfo(in);
            frame.readPayload(in);
            if (frame.getOpCode() == OpCode.Close) {
                return new CloseFrame();
            }
            return frame;
        }

        public static byte[] text2Binary(String payload) throws CharacterCodingException {
            return payload.getBytes(TEXT_CHARSET);
        }

        private WebSocketFrame(OpCode opCode, boolean fin) {
            setOpCode(opCode);
            setFin(fin);
        }

        public WebSocketFrame(OpCode opCode, boolean fin, byte[] payload) {
            this(opCode, fin, payload, (byte[]) null);
        }

        public WebSocketFrame(OpCode opCode, boolean fin, byte[] payload, byte[] maskingKey) {
            this(opCode, fin);
            setMaskingKey(maskingKey);
            setBinaryPayload(payload);
        }

        public WebSocketFrame(OpCode opCode, boolean fin, String payload) throws CharacterCodingException {
            this(opCode, fin, payload, (byte[]) null);
        }

        public WebSocketFrame(OpCode opCode, boolean fin, String payload, byte[] maskingKey) throws CharacterCodingException {
            this(opCode, fin);
            setMaskingKey(maskingKey);
            setTextPayload(payload);
        }

        public WebSocketFrame(OpCode opCode, List<WebSocketFrame> fragments) throws WebSocketException {
            setOpCode(opCode);
            setFin(true);
            long _payloadLength = 0;
            Iterator<WebSocketFrame> it = fragments.iterator();
            while (it.hasNext()) {
                _payloadLength += (long) it.next().getBinaryPayload().length;
            }
            if (_payloadLength < 0 || _payloadLength > 2147483647L) {
                throw new WebSocketException(CloseCode.MessageTooBig, "Max frame length has been exceeded.");
            }
            this._payloadLength = (int) _payloadLength;
            byte[] payload = new byte[this._payloadLength];
            int offset = 0;
            for (WebSocketFrame inter : fragments) {
                System.arraycopy(inter.getBinaryPayload(), 0, payload, offset, inter.getBinaryPayload().length);
                offset += inter.getBinaryPayload().length;
            }
            setBinaryPayload(payload);
        }

        public WebSocketFrame(WebSocketFrame clone) {
            setOpCode(clone.getOpCode());
            setFin(clone.isFin());
            setBinaryPayload(clone.getBinaryPayload());
            setMaskingKey(clone.getMaskingKey());
        }

        public byte[] getBinaryPayload() {
            return this.payload;
        }

        public byte[] getMaskingKey() {
            return this.maskingKey;
        }

        public OpCode getOpCode() {
            return this.opCode;
        }

        public String getTextPayload() {
            if (this._payloadString == null) {
                try {
                    this._payloadString = binary2Text(getBinaryPayload());
                } catch (CharacterCodingException e) {
                    throw new RuntimeException("Undetected CharacterCodingException", e);
                }
            }
            return this._payloadString;
        }

        public boolean isFin() {
            return this.fin;
        }

        public boolean isMasked() {
            return this.maskingKey != null && this.maskingKey.length == 4;
        }

        private String payloadToString() {
            if (this.payload == null) {
                return "null";
            }
            StringBuilder sb = new StringBuilder();
            sb.append('[').append(this.payload.length).append("b] ");
            if (getOpCode() == OpCode.Text) {
                String text = getTextPayload();
                if (text.length() > 100) {
                    sb.append(text.substring(0, 100)).append("...");
                } else {
                    sb.append(text);
                }
            } else {
                sb.append("0x");
                for (int i = 0; i < Math.min(this.payload.length, 50); i++) {
                    sb.append(Integer.toHexString(this.payload[i] & 255));
                }
                if (this.payload.length > 50) {
                    sb.append("...");
                }
            }
            return sb.toString();
        }

        private void readPayload(InputStream in) throws IOException {
            this.payload = new byte[this._payloadLength];
            int read = 0;
            while (read < this._payloadLength) {
                read += checkedRead(in.read(this.payload, read, this._payloadLength - read));
            }
            if (isMasked()) {
                for (int i = 0; i < this.payload.length; i++) {
                    byte[] bArr = this.payload;
                    bArr[i] = (byte) (bArr[i] ^ this.maskingKey[i % 4]);
                }
            }
            if (getOpCode() == OpCode.Text) {
                this._payloadString = binary2Text(getBinaryPayload());
            }
        }

        private void readPayloadInfo(InputStream in) throws IOException {
            byte b = (byte) checkedRead(in.read());
            boolean masked = (b & FtConstants.DCD) != 0;
            this._payloadLength = (byte) (b & 127);
            if (this._payloadLength == 126) {
                this._payloadLength = ((checkedRead(in.read()) << 8) | checkedRead(in.read())) & 65535;
                if (this._payloadLength < 126) {
                    throw new WebSocketException(CloseCode.ProtocolError, "Invalid data frame 2byte length. (not using minimal length encoding)");
                }
            } else if (this._payloadLength == 127) {
                long _payloadLength = (((long) checkedRead(in.read())) << 56) | (((long) checkedRead(in.read())) << 48) | (((long) checkedRead(in.read())) << 40) | (((long) checkedRead(in.read())) << 32) | ((long) (checkedRead(in.read()) << 24)) | ((long) (checkedRead(in.read()) << 16)) | ((long) (checkedRead(in.read()) << 8)) | ((long) checkedRead(in.read()));
                if (_payloadLength < 65536) {
                    throw new WebSocketException(CloseCode.ProtocolError, "Invalid data frame 4byte length. (not using minimal length encoding)");
                }
                if (_payloadLength < 0 || _payloadLength > 2147483647L) {
                    throw new WebSocketException(CloseCode.MessageTooBig, "Max frame length has been exceeded.");
                }
                this._payloadLength = (int) _payloadLength;
            }
            if (this.opCode.isControlFrame()) {
                if (this._payloadLength > 125) {
                    throw new WebSocketException(CloseCode.ProtocolError, "Control frame with payload length > 125 bytes.");
                }
                if (this.opCode == OpCode.Close && this._payloadLength == 1) {
                    throw new WebSocketException(CloseCode.ProtocolError, "Received close frame with payload len 1.");
                }
            }
            if (masked) {
                this.maskingKey = new byte[4];
                int read = 0;
                while (read < this.maskingKey.length) {
                    read += checkedRead(in.read(this.maskingKey, read, this.maskingKey.length - read));
                }
            }
        }

        public void setBinaryPayload(byte[] payload) {
            this.payload = payload;
            this._payloadLength = payload.length;
            this._payloadString = null;
        }

        public void setFin(boolean fin) {
            this.fin = fin;
        }

        public void setMaskingKey(byte[] maskingKey) {
            if (maskingKey != null && maskingKey.length != 4) {
                throw new IllegalArgumentException("MaskingKey " + Arrays.toString(maskingKey) + " hasn't length 4");
            }
            this.maskingKey = maskingKey;
        }

        public void setOpCode(OpCode opcode) {
            this.opCode = opcode;
        }

        public void setTextPayload(String payload) throws CharacterCodingException {
            this.payload = text2Binary(payload);
            this._payloadLength = payload.length();
            this._payloadString = payload;
        }

        public void setUnmasked() {
            setMaskingKey(null);
        }

        public String toString() {
            StringBuilder sb = new StringBuilder("WS[");
            sb.append(getOpCode());
            sb.append(", ").append(isFin() ? "fin" : "inter");
            sb.append(", ").append(isMasked() ? "masked" : "unmasked");
            sb.append(", ").append(payloadToString());
            sb.append(']');
            return sb.toString();
        }

        public void write(OutputStream out) throws IOException {
            byte header = 0;
            if (this.fin) {
                header = (byte) (0 | 128);
            }
            out.write((byte) ((this.opCode.getValue() & 15) | header));
            this._payloadLength = getBinaryPayload().length;
            if (this._payloadLength <= 125) {
                out.write(isMasked() ? ((byte) this._payloadLength) | FtConstants.DCD : (byte) this._payloadLength);
            } else if (this._payloadLength <= 65535) {
                out.write(isMasked() ? 254 : 126);
                out.write(this._payloadLength >>> 8);
                out.write(this._payloadLength);
            } else {
                out.write(isMasked() ? 255 : 127);
                out.write(0);
                out.write(0);
                out.write(0);
                out.write(0);
                out.write(this._payloadLength >>> 24);
                out.write(this._payloadLength >>> 16);
                out.write(this._payloadLength >>> 8);
                out.write(this._payloadLength);
            }
            if (isMasked()) {
                out.write(this.maskingKey);
                for (int i = 0; i < this._payloadLength; i++) {
                    out.write(getBinaryPayload()[i] ^ this.maskingKey[i % 4]);
                }
            } else {
                out.write(getBinaryPayload());
            }
            out.flush();
        }
    }

    private static String encodeBase64(byte[] buf) {
        int i;
        int i2;
        int i3;
        int i4;
        int size = buf.length;
        char[] ar = new char[((size + 2) / 3) * 4];
        int a = 0;
        int i5 = 0;
        while (i5 < size) {
            int i6 = i5 + 1;
            byte b0 = buf[i5];
            if (i6 < size) {
                i = i6 + 1;
                i2 = buf[i6];
            } else {
                i = i6;
                i2 = 0;
            }
            if (i < size) {
                i4 = i + 1;
                i3 = buf[i];
            } else {
                int i7 = i;
                i3 = 0;
                i4 = i7;
            }
            int a2 = a + 1;
            ar[a] = ALPHABET[(b0 >> 2) & 63];
            int a3 = a2 + 1;
            ar[a2] = ALPHABET[((b0 << 4) | ((i2 & 255) >> 4)) & 63];
            int a4 = a3 + 1;
            ar[a3] = ALPHABET[((i2 << 2) | ((i3 & 255) >> 6)) & 63];
            a = a4 + 1;
            ar[a4] = ALPHABET[i3 & 63];
            i5 = i4;
        }
        switch (size % 3) {
            case 1:
                a--;
                ar[a] = '=';
            case 2:
                ar[a - 1] = '=';
                break;
        }
        return new String(ar);
    }

    public static String makeAcceptKey(String key) throws NoSuchAlgorithmException {
        MessageDigest md = MessageDigest.getInstance("SHA-1");
        String text = key + WEBSOCKET_KEY_MAGIC;
        md.update(text.getBytes(), 0, text.length());
        byte[] sha1hash = md.digest();
        return encodeBase64(sha1hash);
    }

    public NanoWSD(int port) {
        super(port);
    }

    public NanoWSD(String hostname, int port) {
        super(hostname, port);
    }

    private boolean isWebSocketConnectionHeader(Map<String, String> headers) {
        String connection = headers.get(HEADER_CONNECTION);
        return connection != null && connection.toLowerCase().contains(HEADER_CONNECTION_VALUE.toLowerCase());
    }

    protected boolean isWebsocketRequested(NanoHTTPD.IHTTPSession session) {
        Map<String, String> headers = session.getHeaders();
        String upgrade = headers.get(HEADER_UPGRADE);
        boolean isCorrectConnection = isWebSocketConnectionHeader(headers);
        boolean isUpgrade = HEADER_UPGRADE_VALUE.equalsIgnoreCase(upgrade);
        return isUpgrade && isCorrectConnection;
    }

    @Override // fi.iki.elonen.NanoHTTPD
    public NanoHTTPD.Response serve(NanoHTTPD.IHTTPSession session) {
        Map<String, String> headers = session.getHeaders();
        if (isWebsocketRequested(session)) {
            if (!HEADER_WEBSOCKET_VERSION_VALUE.equalsIgnoreCase(headers.get(HEADER_WEBSOCKET_VERSION))) {
                return newFixedLengthResponse(NanoHTTPD.Response.Status.BAD_REQUEST, "text/plain", "Invalid Websocket-Version " + headers.get(HEADER_WEBSOCKET_VERSION));
            }
            if (!headers.containsKey(HEADER_WEBSOCKET_KEY)) {
                return newFixedLengthResponse(NanoHTTPD.Response.Status.BAD_REQUEST, "text/plain", "Missing Websocket-Key");
            }
            WebSocket webSocket = openWebSocket(session);
            NanoHTTPD.Response handshakeResponse = webSocket.getHandshakeResponse();
            try {
                handshakeResponse.addHeader(HEADER_WEBSOCKET_ACCEPT, makeAcceptKey(headers.get(HEADER_WEBSOCKET_KEY)));
                if (headers.containsKey(HEADER_WEBSOCKET_PROTOCOL)) {
                    handshakeResponse.addHeader(HEADER_WEBSOCKET_PROTOCOL, headers.get(HEADER_WEBSOCKET_PROTOCOL).split(DocLint.TAGS_SEPARATOR)[0]);
                }
                return handshakeResponse;
            } catch (NoSuchAlgorithmException e) {
                return newFixedLengthResponse(NanoHTTPD.Response.Status.INTERNAL_ERROR, "text/plain", "The SHA-1 Algorithm required for websockets is not available on the server.");
            }
        }
        return serveHttp(session);
    }

    protected NanoHTTPD.Response serveHttp(NanoHTTPD.IHTTPSession session) {
        return super.serve(session);
    }

    @Override // fi.iki.elonen.NanoHTTPD
    protected boolean useGzipWhenAccepted(NanoHTTPD.Response r) {
        return false;
    }
}
