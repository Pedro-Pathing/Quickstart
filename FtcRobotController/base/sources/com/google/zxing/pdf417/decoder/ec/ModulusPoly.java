package com.google.zxing.pdf417.decoder.ec;

/* JADX INFO: loaded from: classes8.dex */
final class ModulusPoly {
    private final int[] coefficients;
    private final ModulusGF field;

    ModulusPoly(ModulusGF field, int[] coefficients) {
        if (coefficients.length == 0) {
            throw new IllegalArgumentException();
        }
        this.field = field;
        int coefficientsLength = coefficients.length;
        if (coefficientsLength > 1 && coefficients[0] == 0) {
            int firstNonZero = 1;
            while (firstNonZero < coefficientsLength && coefficients[firstNonZero] == 0) {
                firstNonZero++;
            }
            if (firstNonZero == coefficientsLength) {
                this.coefficients = new int[]{0};
                return;
            } else {
                this.coefficients = new int[coefficientsLength - firstNonZero];
                System.arraycopy(coefficients, firstNonZero, this.coefficients, 0, this.coefficients.length);
                return;
            }
        }
        this.coefficients = coefficients;
    }

    int[] getCoefficients() {
        return this.coefficients;
    }

    int getDegree() {
        return this.coefficients.length - 1;
    }

    boolean isZero() {
        return this.coefficients[0] == 0;
    }

    int getCoefficient(int degree) {
        return this.coefficients[(this.coefficients.length - 1) - degree];
    }

    int evaluateAt(int a) {
        if (a == 0) {
            return getCoefficient(0);
        }
        if (a != 1) {
            int result = this.coefficients[0];
            int size = this.coefficients.length;
            for (int i = 1; i < size; i++) {
                result = this.field.add(this.field.multiply(a, result), this.coefficients[i]);
            }
            return result;
        }
        int result2 = 0;
        for (int coefficient : this.coefficients) {
            result2 = this.field.add(result2, coefficient);
        }
        return result2;
    }

    ModulusPoly add(ModulusPoly other) {
        if (!this.field.equals(other.field)) {
            throw new IllegalArgumentException("ModulusPolys do not have same ModulusGF field");
        }
        if (isZero()) {
            return other;
        }
        if (other.isZero()) {
            return this;
        }
        int[] smallerCoefficients = this.coefficients;
        int[] largerCoefficients = other.coefficients;
        if (smallerCoefficients.length > largerCoefficients.length) {
            smallerCoefficients = largerCoefficients;
            largerCoefficients = smallerCoefficients;
        }
        int[] sumDiff = new int[largerCoefficients.length];
        int lengthDiff = largerCoefficients.length - smallerCoefficients.length;
        System.arraycopy(largerCoefficients, 0, sumDiff, 0, lengthDiff);
        for (int i = lengthDiff; i < largerCoefficients.length; i++) {
            sumDiff[i] = this.field.add(smallerCoefficients[i - lengthDiff], largerCoefficients[i]);
        }
        return new ModulusPoly(this.field, sumDiff);
    }

    ModulusPoly subtract(ModulusPoly other) {
        if (!this.field.equals(other.field)) {
            throw new IllegalArgumentException("ModulusPolys do not have same ModulusGF field");
        }
        if (other.isZero()) {
            return this;
        }
        return add(other.negative());
    }

    ModulusPoly multiply(ModulusPoly other) {
        if (!this.field.equals(other.field)) {
            throw new IllegalArgumentException("ModulusPolys do not have same ModulusGF field");
        }
        if (isZero() || other.isZero()) {
            return this.field.getZero();
        }
        int[] aCoefficients = this.coefficients;
        int aLength = aCoefficients.length;
        int[] bCoefficients = other.coefficients;
        int bLength = bCoefficients.length;
        int[] product = new int[(aLength + bLength) - 1];
        for (int i = 0; i < aLength; i++) {
            int aCoeff = aCoefficients[i];
            for (int j = 0; j < bLength; j++) {
                product[i + j] = this.field.add(product[i + j], this.field.multiply(aCoeff, bCoefficients[j]));
            }
        }
        return new ModulusPoly(this.field, product);
    }

    ModulusPoly negative() {
        int size = this.coefficients.length;
        int[] negativeCoefficients = new int[size];
        for (int i = 0; i < size; i++) {
            negativeCoefficients[i] = this.field.subtract(0, this.coefficients[i]);
        }
        return new ModulusPoly(this.field, negativeCoefficients);
    }

    ModulusPoly multiply(int scalar) {
        if (scalar == 0) {
            return this.field.getZero();
        }
        if (scalar == 1) {
            return this;
        }
        int size = this.coefficients.length;
        int[] product = new int[size];
        for (int i = 0; i < size; i++) {
            product[i] = this.field.multiply(this.coefficients[i], scalar);
        }
        return new ModulusPoly(this.field, product);
    }

    ModulusPoly multiplyByMonomial(int degree, int coefficient) {
        if (degree < 0) {
            throw new IllegalArgumentException();
        }
        if (coefficient == 0) {
            return this.field.getZero();
        }
        int size = this.coefficients.length;
        int[] product = new int[size + degree];
        for (int i = 0; i < size; i++) {
            product[i] = this.field.multiply(this.coefficients[i], coefficient);
        }
        return new ModulusPoly(this.field, product);
    }

    public String toString() {
        StringBuilder result = new StringBuilder(getDegree() * 8);
        for (int degree = getDegree(); degree >= 0; degree--) {
            int coefficient = getCoefficient(degree);
            int coefficient2 = coefficient;
            if (coefficient != 0) {
                if (coefficient2 < 0) {
                    result.append(" - ");
                    coefficient2 = -coefficient2;
                } else if (result.length() > 0) {
                    result.append(" + ");
                }
                if (degree == 0 || coefficient2 != 1) {
                    result.append(coefficient2);
                }
                if (degree != 0) {
                    if (degree == 1) {
                        result.append('x');
                    } else {
                        result.append("x^");
                        result.append(degree);
                    }
                }
            }
        }
        return result.toString();
    }
}
