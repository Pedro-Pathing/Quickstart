#!/usr/bin/env python3
"""Generates TurretProof/Step.lean from the contracts of TurretStep.java.

Each method of TurretStep has a contract `ensures \\result == <formula>` that OpenJML proves
against the real turret code. This script turns exactly those formulas into Lean definitions, so the
Lean convergence proof is about the same formulas the code is proven to compute; nothing is copied
by hand. It accepts only the small formula language TurretStep documents and fails on anything else.

Usage: gen_step.py <TurretStep.java> <Step.lean>
"""
import re
import sys

# Java parameter names that are Lean functions: renamed consistently.
RENAME = {"min": "travelMin", "max": "travelMax"}


def tokenize(src):
    spec = r"""\s*(?:(\d+\.\d+|\d+)|(Math\.min|Math\.max|Math\.abs|[A-Za-z_]\w*)|(&&|\|\||==|!=|<=|>=|[-+*/()<>?:!,]))"""
    pos, out = 0, []
    while pos < len(src):
        m = re.compile(spec).match(src, pos)
        if not m or m.end() == pos:
            if src[pos:].strip() == "":
                break
            raise SystemExit(f"gen_step: cannot read formula at: {src[pos:pos + 30]!r}")
        num, ident, op = m.groups()
        out.append(("num", num) if num else ("id", ident) if ident else ("op", op))
        pos = m.end()
    return out


class Parser:
    def __init__(self, tokens, params, methods):
        self.t, self.i, self.params, self.methods = tokens, 0, params, methods

    def peek(self):
        return self.t[self.i] if self.i < len(self.t) else ("eof", "")

    def take(self, value=None):
        tok = self.peek()
        if value is not None and tok[1] != value:
            raise SystemExit(f"gen_step: expected {value!r}, found {tok[1]!r}")
        self.i += 1
        return tok

    # Each parse returns (lean_text, kind) with kind "real" or "prop".
    def expr(self):
        cond = self.or_()
        if self.peek()[1] == "?":
            self.take("?")
            a = self.expr()
            self.take(":")
            b = self.expr()
            if a[1] != b[1]:
                raise SystemExit("gen_step: ?: branches of different kinds")
            return (f"(if {self.prop(cond)} then {a[0]} else {b[0]})", a[1])
        return cond

    def prop(self, e):
        if e[1] != "prop":
            raise SystemExit(f"gen_step: expected a condition, found {e[0]}")
        return e[0]

    def binary(self, sub, ops, lean, kind_in, kind_out):
        left = sub()
        while self.peek()[1] in ops:
            op = self.take()[1]
            right = sub()
            for side in (left, right):
                if side[1] != kind_in:
                    raise SystemExit(f"gen_step: {op} on a {side[1]}: {side[0]}")
            left = (f"({left[0]} {lean[op]} {right[0]})", kind_out)
        return left

    def or_(self):
        return self.binary(self.and_, {"||"}, {"||": "∨"}, "prop", "prop")

    def and_(self):
        return self.binary(self.rel, {"&&"}, {"&&": "∧"}, "prop", "prop")

    def rel(self):
        ops = {"==": "=", "!=": "≠", "<=": "≤", ">=": "≥", "<": "<", ">": ">"}
        left = self.add()
        if self.peek()[1] in ops:
            op = self.take()[1]
            right = self.add()
            if left[1] != "real" or right[1] != "real":
                raise SystemExit(f"gen_step: comparison of non-numbers: {left[0]} {op} {right[0]}")
            return (f"({left[0]} {ops[op]} {right[0]})", "prop")
        return left

    def add(self):
        return self.binary(self.mul, {"+", "-"}, {"+": "+", "-": "-"}, "real", "real")

    def mul(self):
        return self.binary(self.unary, {"*", "/"}, {"*": "*", "/": "/"}, "real", "real")

    def unary(self):
        tok = self.peek()
        if tok[1] == "!":
            self.take()
            e = self.unary()
            return (f"(¬{self.prop(e)})", "prop")
        if tok[1] == "-":
            self.take()
            e = self.unary()
            return (f"(-{e[0]})", "real")
        return self.primary()

    def args(self):
        self.take("(")
        out = [self.expr()]
        while self.peek()[1] == ",":
            self.take(",")
            out.append(self.expr())
        self.take(")")
        return out

    def primary(self):
        kind, val = self.take()
        if kind == "num":
            return (val[:-2] if val.endswith(".0") else val, "real")
        if val == "(":
            e = self.expr()
            self.take(")")
            return e
        if val in ("Math.min", "Math.max"):
            a, b = self.args()
            return (f"({val[5:]} {a[0]} {b[0]})", "real")
        if val == "Math.abs":
            (a,) = self.args()
            return (f"|{a[0]}|", "real")
        if kind == "id" and self.peek()[1] == "(":
            if val not in self.methods:
                raise SystemExit(f"gen_step: call to {val}, which is not a TurretStep formula")
            args = self.args()
            ptypes = self.methods[val]["types"]
            if len(args) != len(ptypes):
                raise SystemExit(f"gen_step: {val} called with {len(args)} arguments")
            parts = []
            for (text, k), ty in zip(args, ptypes):
                if ty == "boolean" and k != "bool":
                    raise SystemExit(f"gen_step: {val} needs a boolean variable, given {text}")
                parts.append(text)
            return (f"({RENAME.get(val, val)} {' '.join(parts)})", self.methods[val]["kind"])
        if kind == "id" and val in self.params:
            name = RENAME.get(val, val)
            if self.params[val] == "boolean":
                # A boolean variable: a Bool, used as a condition or passed on as is.
                if self.peek()[1] in (",", ")") and self.i > 1 and self.t[self.i - 2][1] in ("(", ","):
                    return (name, "bool")
                return (f"({name} = true)", "prop")
            return (name, "real")
        raise SystemExit(f"gen_step: unknown name {val!r}")


def methods_of(java):
    out = {}
    pat = re.compile(
        r"/\*@(?P<spec>.*?)@\*/\s*static\s+(?P<ret>double|boolean)\s+(?P<name>\w+)\s*\((?P<params>[^)]*)\)",
        re.S,
    )
    for m in pat.finditer(java):
        spec = re.sub(r"(?m)^\s*@", " ", m.group("spec"))
        ens = re.search(r"ensures\s+\\result\s*==\s*(.*?);", spec, re.S)
        if not ens:
            raise SystemExit(f"gen_step: {m.group('name')} has no `ensures \\result == ...`")
        params = [p.split() for p in m.group("params").split(",") if p.strip()]
        out[m.group("name")] = {
            "ret": m.group("ret"),
            "kind": "real" if m.group("ret") == "double" else "prop",
            "params": [(n, t) for t, n in params],
            "types": [t for t, _ in params],
            "formula": " ".join(ens.group(1).split()),
        }
    return out


def main():
    java = open(sys.argv[1]).read()
    methods = methods_of(java)
    lines = [
        "-- GENERATED by gen_step.py from TurretStep.java's contracts. Do not edit: change the",
        "-- Java contracts (which OpenJML proves against the turret code) and regenerate.",
        "import Mathlib.Basic.Real.Basic",
        "import Mathlib.Order.MinMax",
        "import Mathlib.Algebra.Order.AbsoluteValue.Basic",
        "",
        "noncomputable section",
        "open Classical",
        "namespace TurretStep",
        "",
    ]
    for name, m in methods.items():
        params = {n: t for n, t in m["params"]}
        p = Parser(tokenize(m["formula"]), params, methods)
        text, kind = p.expr()
        if p.peek()[0] != "eof":
            raise SystemExit(f"gen_step: trailing text in {name}'s formula")
        if kind == "bool":
            kind = "prop"
            text = f"({text} = true)"
        if kind != m["kind"]:
            raise SystemExit(f"gen_step: {name} returns {m['ret']} but its formula is a {kind}")
        binders = " ".join(
            f"({RENAME.get(n, n)} : {'Bool' if t == 'boolean' else 'ℝ'})" for n, t in m["params"]
        )
        result = "ℝ" if kind == "real" else "Prop"
        lines.append(f"/-- `TurretStep.{name}`: `{m['formula']}` -/")
        lines.append(f"def {RENAME.get(name, name)} {binders} : {result} :=")
        lines.append(f"  {text}")
        lines.append("")
    lines += ["end TurretStep", "end", ""]
    open(sys.argv[2], "w").write("\n".join(lines))


if __name__ == "__main__":
    main()
