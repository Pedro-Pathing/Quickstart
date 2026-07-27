package com.android.tools.r8.dex.code;

import com.android.tools.r8.graph.C0365q5;
import com.qualcomm.robotcore.eventloop.SyncdDevice;
import com.sun.tools.javac.jvm.ByteCodes;
import org.firstinspires.ftc.robotcore.internal.usb.UsbConstants;

/* JADX INFO: compiled from: R8_8.5.35_9c55004e7c41a17b1ed47c4e1952cb6778b3dac6afb6afc113a2737c3cde13e0 */
/* JADX INFO: loaded from: classes.dex */
public abstract class O {
    public static A1 a(int i, int i2, B1 b1, C0365q5 c0365q5) {
        switch (i2) {
            case 0:
                return Q2.a(i, b1);
            case 1:
                return new C0105n2(i, b1);
            case 2:
                return new C0115p2(i, b1);
            case 3:
                return new C0100m2(i, b1);
            case 4:
                return new C0154x2(i, b1);
            case 5:
                return new C0161y2(i, b1);
            case 6:
                return new C0149w2(i, b1);
            case 7:
                return new C0124r2(i, b1);
            case 8:
                return new C0129s2(i, b1);
            case 9:
                return new C0120q2(i, b1);
            case 10:
                return new C0134t2(i, b1);
            case 11:
                return new C0144v2(i, b1);
            case 12:
                return new C0139u2(i, b1);
            case 13:
                return new C0110o2(i, b1);
            case 14:
                return new C0111o3(i, b1);
            case 15:
                return new C0101m3(i, b1);
            case 16:
                return new C0116p3(i, b1);
            case 17:
                return new C0106n3(i, b1);
            case 18:
                return new Y(i, b1);
            case 19:
                return new X(i, b1);
            case 20:
                return new Z(i, b1);
            case 21:
                return new C0043b0(i, b1);
            case 22:
                return new C0068g0(i, b1);
            case 23:
                return new C0073h0(i, b1);
            case 24:
                return new C0078i0(i, b1);
            case 25:
                return new C0083j0(i, b1);
            case 26:
                return new C0058e0(i, b1, c0365q5);
            case 27:
                return new C0063f0(i, b1, c0365q5);
            case 28:
                return new C0038a0(i, b1, c0365q5);
            case 29:
                return new C0090k2(i, b1);
            case 30:
                return new C0095l2(i, b1);
            case 31:
                return new P(i, b1, c0365q5);
            case 32:
                return new C0165z1(i, b1, c0365q5);
            case 33:
                return new I(i, b1);
            case 34:
                return new O2(i, b1, c0365q5);
            case 35:
                return new N2(i, b1, c0365q5);
            case 36:
                return new C0164z0(i, b1, c0365q5);
            case 37:
                return new A0(i, b1, c0365q5);
            case 38:
                return new C0152x0(i, b1);
            case 39:
                return new d4(i, b1);
            case 40:
                return new C0059e1(i, b1);
            case 41:
                return new C0049c1(i, b1);
            case 42:
                return new C0054d1(i, b1);
            case 43:
                return new Z2(i, b1);
            case 44:
                return new L3(i, b1);
            case 45:
                return new V(i, b1);
            case 46:
                return new T(i, b1);
            case 47:
                return new U(i, b1);
            case 48:
                return new S(i, b1);
            case 49:
                return new Q(i, b1);
            case 50:
                return new C0064f1(i, b1);
            case 51:
                return new C0114p1(i, b1);
            case 52:
                return new C0104n1(i, b1);
            case 53:
                return new C0074h1(i, b1);
            case 54:
                return new C0084j1(i, b1);
            case 55:
                return new C0094l1(i, b1);
            case 56:
                return new C0069g1(i, b1);
            case 57:
                return new C0119q1(i, b1);
            case 58:
                return new C0109o1(i, b1);
            case 59:
                return new C0079i1(i, b1);
            case 60:
                return new C0089k1(i, b1);
            case 61:
                return new C0099m1(i, b1);
            default:
                switch (i2) {
                    case 68:
                        return new C0107o(i, b1);
                    case 69:
                        return new C0136u(i, b1);
                    case 70:
                        return new C0126s(i, b1);
                    case 71:
                        return new C0112p(i, b1);
                    case 72:
                        return new C0117q(i, b1);
                    case 73:
                        return new r(i, b1);
                    case 74:
                        return new C0131t(i, b1);
                    case 75:
                        return new B(i, b1);
                    case 76:
                        return new H(i, b1);
                    case 77:
                        return new F(i, b1);
                    case 78:
                        return new C(i, b1);
                    case 79:
                        return new D(i, b1);
                    case 80:
                        return new E(i, b1);
                    case 81:
                        return new G(i, b1);
                    case 82:
                        return new C0123r1(i, b1, c0365q5);
                    case 83:
                        return new C0160y1(i, b1, c0365q5);
                    case 84:
                        return new C0143v1(i, b1, c0365q5);
                    case 85:
                        return new C0128s1(i, b1, c0365q5);
                    case 86:
                        return new C0133t1(i, b1, c0365q5);
                    case 87:
                        return new C0138u1(i, b1, c0365q5);
                    case 88:
                        return new C0153x1(i, b1, c0365q5);
                    case 89:
                        return new Z1(i, b1, c0365q5);
                    case 90:
                        return new C0065f2(i, b1, c0365q5);
                    case 91:
                        return new C0055d2(i, b1, c0365q5);
                    case 92:
                        return new C0040a2(i, b1, c0365q5);
                    case 93:
                        return new C0045b2(i, b1, c0365q5);
                    case 94:
                        return new C0050c2(i, b1, c0365q5);
                    case 95:
                        return new C0060e2(i, b1, c0365q5);
                    case 96:
                        return new C0135t3(i, b1, c0365q5);
                    case 97:
                        return new A3(i, b1, c0365q5);
                    case 98:
                        return new C0155x3(i, b1, c0365q5);
                    case 99:
                        return new C0140u3(i, b1, c0365q5);
                    case 100:
                        return new C0145v3(i, b1, c0365q5);
                    case 101:
                        return new C0150w3(i, b1, c0365q5);
                    case 102:
                        return new C0167z3(i, b1, c0365q5);
                    case 103:
                        return new N3(i, b1, c0365q5);
                    case 104:
                        return new T3(i, b1, c0365q5);
                    case 105:
                        return new R3(i, b1, c0365q5);
                    case 106:
                        return new O3(i, b1, c0365q5);
                    case 107:
                        return new P3(i, b1, c0365q5);
                    case 108:
                        return new Q3(i, b1, c0365q5);
                    case 109:
                        return new S3(i, b1, c0365q5);
                    case 110:
                        return new X1(i, b1, c0365q5);
                    case 111:
                        return new V1(i, b1, c0365q5);
                    case 112:
                        return new L1(i, b1, c0365q5);
                    case ByteCodes.lmod /* 113 */:
                        return new T1(i, b1, c0365q5);
                    case ByteCodes.fmod /* 114 */:
                        return new N1(i, b1, c0365q5);
                    default:
                        switch (i2) {
                            case 116:
                                return new Y1(i, b1, c0365q5);
                            case 117:
                                return new W1(i, b1, c0365q5);
                            case 118:
                                return new M1(i, b1, c0365q5);
                            case 119:
                                return new U1(i, b1, c0365q5);
                            case 120:
                                return new O1(i, b1, c0365q5);
                            default:
                                switch (i2) {
                                    case 123:
                                        return new L2(i, b1);
                                    case 124:
                                        return new R2(i, b1);
                                    case 125:
                                        return new M2(i, b1);
                                    case 126:
                                        return new S2(i, b1);
                                    case 127:
                                        return new K2(i, b1);
                                    case 128:
                                        return new J2(i, b1);
                                    case 129:
                                        return new H1(i, b1);
                                    case 130:
                                        return new G1(i, b1);
                                    case 131:
                                        return new F1(i, b1);
                                    case 132:
                                        return new C0085j2(i, b1);
                                    case 133:
                                        return new C0080i2(i, b1);
                                    case 134:
                                        return new C0075h2(i, b1);
                                    case 135:
                                        return new C0(i, b1);
                                    case 136:
                                        return new D0(i, b1);
                                    case 137:
                                        return new B0(i, b1);
                                    case 138:
                                        return new C0142v0(i, b1);
                                    case 139:
                                        return new C0147w0(i, b1);
                                    case 140:
                                        return new C0137u0(i, b1);
                                    case 141:
                                        return new D1(i, b1);
                                    case 142:
                                        return new E1(i, b1);
                                    case 143:
                                        return new I1(i, b1);
                                    case 144:
                                        return new C0082j(i, b1);
                                    case 145:
                                        return new Z3(i, b1);
                                    case 146:
                                        return new E2(i, b1);
                                    case 147:
                                        return new C0113p0(i, b1);
                                    case 148:
                                        return new C0076h3(i, b1);
                                    case 149:
                                        return new C0146w(i, b1);
                                    case 150:
                                        return new U2(i, b1);
                                    case 151:
                                        return new k4(i, b1);
                                    case 152:
                                        return new C3(i, b1);
                                    case 153:
                                        return new H3(i, b1);
                                    case 154:
                                        return new f4(i, b1);
                                    case 155:
                                        return new C0102n(i, b1);
                                    case ByteCodes.ifge /* 156 */:
                                        return new b4(i, b1);
                                    case ByteCodes.ifgt /* 157 */:
                                        return new I2(i, b1);
                                    case ByteCodes.ifle /* 158 */:
                                        return new C0132t0(i, b1);
                                    case ByteCodes.if_icmpeq /* 159 */:
                                        return new C0096l3(i, b1);
                                    case ByteCodes.if_icmpne /* 160 */:
                                        return new A(i, b1);
                                    case ByteCodes.if_icmplt /* 161 */:
                                        return new Y2(i, b1);
                                    case ByteCodes.if_icmpge /* 162 */:
                                        return new o4(i, b1);
                                    case ByteCodes.if_icmpgt /* 163 */:
                                        return new F3(i, b1);
                                    case ByteCodes.if_icmple /* 164 */:
                                        return new K3(i, b1);
                                    case ByteCodes.if_acmpeq /* 165 */:
                                        return new i4(i, b1);
                                    case ByteCodes.if_acmpne /* 166 */:
                                        return new C0072h(i, b1);
                                    case ByteCodes.goto_ /* 167 */:
                                        return new X3(i, b1);
                                    case 168:
                                        return new C2(i, b1);
                                    case ByteCodes.ret /* 169 */:
                                        return new C0103n0(i, b1);
                                    case ByteCodes.tableswitch /* 170 */:
                                        return new C0066f3(i, b1);
                                    case ByteCodes.lookupswitch /* 171 */:
                                        return new C0062f(i, b1);
                                    case ByteCodes.ireturn /* 172 */:
                                        return new V3(i, b1);
                                    case 173:
                                        return new A2(i, b1);
                                    case ByteCodes.freturn /* 174 */:
                                        return new C0093l0(i, b1);
                                    case ByteCodes.dreturn /* 175 */:
                                        return new C0056d3(i, b1);
                                    case ByteCodes.areturn /* 176 */:
                                        return new C0077i(i, b1);
                                    case ByteCodes.return_ /* 177 */:
                                        return new Y3(i, b1);
                                    case ByteCodes.getstatic /* 178 */:
                                        return new D2(i, b1);
                                    case ByteCodes.putstatic /* 179 */:
                                        return new C0108o0(i, b1);
                                    case ByteCodes.getfield /* 180 */:
                                        return new C0071g3(i, b1);
                                    case ByteCodes.putfield /* 181 */:
                                        return new C0141v(i, b1);
                                    case ByteCodes.invokevirtual /* 182 */:
                                        return new T2(i, b1);
                                    case ByteCodes.invokespecial /* 183 */:
                                        return new j4(i, b1);
                                    case ByteCodes.invokestatic /* 184 */:
                                        return new B3(i, b1);
                                    case ByteCodes.invokeinterface /* 185 */:
                                        return new G3(i, b1);
                                    case ByteCodes.invokedynamic /* 186 */:
                                        return new e4(i, b1);
                                    case ByteCodes.new_ /* 187 */:
                                        return new C0097m(i, b1);
                                    case ByteCodes.newarray /* 188 */:
                                        return new a4(i, b1);
                                    case ByteCodes.anewarray /* 189 */:
                                        return new H2(i, b1);
                                    case ByteCodes.arraylength /* 190 */:
                                        return new C0127s0(i, b1);
                                    case ByteCodes.athrow /* 191 */:
                                        return new C0091k3(i, b1);
                                    case ByteCodes.checkcast /* 192 */:
                                        return new C0163z(i, b1);
                                    case ByteCodes.instanceof_ /* 193 */:
                                        return new X2(i, b1);
                                    case ByteCodes.monitorenter /* 194 */:
                                        return new n4(i, b1);
                                    case ByteCodes.monitorexit /* 195 */:
                                        return new E3(i, b1);
                                    case ByteCodes.wide /* 196 */:
                                        return new J3(i, b1);
                                    case ByteCodes.multianewarray /* 197 */:
                                        return new h4(i, b1);
                                    case ByteCodes.if_acmp_null /* 198 */:
                                        return new C0067g(i, b1);
                                    case ByteCodes.if_acmp_nonnull /* 199 */:
                                        return new W3(i, b1);
                                    case 200:
                                        return new B2(i, b1);
                                    case ByteCodes.jsr_w /* 201 */:
                                        return new C0098m0(i, b1);
                                    case ByteCodes.breakpoint /* 202 */:
                                        return new C0061e3(i, b1);
                                    case ByteCodes.ByteCodeCount /* 203 */:
                                        return new C0057e(i, b1);
                                    case 204:
                                        return new U3(i, b1);
                                    case 205:
                                        return new C0166z2(i, b1);
                                    case 206:
                                        return new C0088k0(i, b1);
                                    case 207:
                                        return new C0051c3(i, b1);
                                    case 208:
                                        return new C0087k(i, b1);
                                    case 209:
                                        return new C0121q3(i, b1);
                                    case 210:
                                        return new F2(i, b1);
                                    case 211:
                                        return new C0118q0(i, b1);
                                    case 212:
                                        return new C0081i3(i, b1);
                                    case 213:
                                        return new C0151x(i, b1);
                                    case 214:
                                        return new V2(i, b1);
                                    case 215:
                                        return new l4(i, b1);
                                    case 216:
                                        return new C0092l(i, b1);
                                    case 217:
                                        return new C0125r3(i, b1);
                                    case 218:
                                        return new G2(i, b1);
                                    case 219:
                                        return new C0122r0(i, b1);
                                    case 220:
                                        return new C0086j3(i, b1);
                                    case 221:
                                        return new C0156y(i, b1);
                                    case 222:
                                        return new W2(i, b1);
                                    case 223:
                                        return new m4(i, b1);
                                    case UsbConstants.USB_CLASS_WIRELESS_CONTROLLER /* 224 */:
                                        return new D3(i, b1);
                                    case 225:
                                        return new I3(i, b1);
                                    case 226:
                                        return new g4(i, b1);
                                    default:
                                        switch (i2) {
                                            case SyncdDevice.msAbnormalReopenInterval /* 250 */:
                                                return new R1(i, b1, c0365q5);
                                            case 251:
                                                return new S1(i, b1, c0365q5);
                                            case 252:
                                                return new J1(i, b1, c0365q5);
                                            case 253:
                                                return new K1(i, b1, c0365q5);
                                            case 254:
                                                return new C0048c0(i, b1, c0365q5);
                                            case 255:
                                                return new C0053d0(i, b1, c0365q5);
                                            default:
                                                throw new IllegalArgumentException("Illegal Opcode: 0x" + Integer.toString(i2, 16));
                                        }
                                }
                        }
                }
        }
    }
}
