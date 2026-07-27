package com.google.zxing.pdf417.decoder.ec;

import com.google.zxing.ChecksumException;

/* JADX INFO: loaded from: classes8.dex */
public final class ErrorCorrection {
    private final ModulusGF field = ModulusGF.PDF417_GF;

    public int decode(int[] iArr, int i, int[] iArr2) throws ChecksumException {
        ModulusPoly modulusPoly = new ModulusPoly(this.field, iArr);
        int[] iArr3 = new int[i];
        boolean z = false;
        for (int i2 = i; i2 > 0; i2--) {
            int iEvaluateAt = modulusPoly.evaluateAt(this.field.exp(i2));
            iArr3[i - i2] = iEvaluateAt;
            if (iEvaluateAt != 0) {
                z = true;
            }
        }
        if (!z) {
            return 0;
        }
        ModulusPoly one = this.field.getOne();
        if (iArr2 != null) {
            for (int i3 : iArr2) {
                one = one.multiply(new ModulusPoly(this.field, new int[]{this.field.subtract(0, this.field.exp((iArr.length - 1) - i3)), 1}));
            }
        }
        ModulusPoly[] modulusPolyArrRunEuclideanAlgorithm = runEuclideanAlgorithm(this.field.buildMonomial(i, 1), new ModulusPoly(this.field, iArr3), i);
        ModulusPoly modulusPoly2 = modulusPolyArrRunEuclideanAlgorithm[0];
        ModulusPoly modulusPoly3 = modulusPolyArrRunEuclideanAlgorithm[1];
        int[] iArrFindErrorLocations = findErrorLocations(modulusPoly2);
        int[] iArrFindErrorMagnitudes = findErrorMagnitudes(modulusPoly3, modulusPoly2, iArrFindErrorLocations);
        for (int i4 = 0; i4 < iArrFindErrorLocations.length; i4++) {
            int length = (iArr.length - 1) - this.field.log(iArrFindErrorLocations[i4]);
            if (length < 0) {
                throw ChecksumException.getChecksumInstance();
            }
            iArr[length] = this.field.subtract(iArr[length], iArrFindErrorMagnitudes[i4]);
        }
        return iArrFindErrorLocations.length;
    }

    private ModulusPoly[] runEuclideanAlgorithm(ModulusPoly modulusPoly, ModulusPoly modulusPoly2, int i) throws ChecksumException {
        if (modulusPoly.getDegree() < modulusPoly2.getDegree()) {
            modulusPoly2 = modulusPoly;
            modulusPoly = modulusPoly2;
        }
        ModulusPoly zero = this.field.getZero();
        ModulusPoly one = this.field.getOne();
        while (true) {
            ModulusPoly modulusPoly3 = modulusPoly2;
            modulusPoly2 = modulusPoly;
            modulusPoly = modulusPoly3;
            ModulusPoly modulusPoly4 = one;
            ModulusPoly modulusPoly5 = zero;
            zero = modulusPoly4;
            if (modulusPoly.getDegree() >= i / 2) {
                if (modulusPoly.isZero()) {
                    throw ChecksumException.getChecksumInstance();
                }
                ModulusPoly zero2 = this.field.getZero();
                int iInverse = this.field.inverse(modulusPoly.getCoefficient(modulusPoly.getDegree()));
                while (modulusPoly2.getDegree() >= modulusPoly.getDegree() && !modulusPoly2.isZero()) {
                    int degree = modulusPoly2.getDegree() - modulusPoly.getDegree();
                    int iMultiply = this.field.multiply(modulusPoly2.getCoefficient(modulusPoly2.getDegree()), iInverse);
                    zero2 = zero2.add(this.field.buildMonomial(degree, iMultiply));
                    modulusPoly2 = modulusPoly2.subtract(modulusPoly.multiplyByMonomial(degree, iMultiply));
                }
                one = zero2.multiply(zero).subtract(modulusPoly5).negative();
            } else {
                int coefficient = zero.getCoefficient(0);
                if (coefficient == 0) {
                    throw ChecksumException.getChecksumInstance();
                }
                int iInverse2 = this.field.inverse(coefficient);
                return new ModulusPoly[]{zero.multiply(iInverse2), modulusPoly.multiply(iInverse2)};
            }
        }
    }

    private int[] findErrorLocations(ModulusPoly errorLocator) throws ChecksumException {
        int numErrors = errorLocator.getDegree();
        int[] result = new int[numErrors];
        int e = 0;
        for (int i = 1; i < this.field.getSize() && e < numErrors; i++) {
            if (errorLocator.evaluateAt(i) == 0) {
                result[e] = this.field.inverse(i);
                e++;
            }
        }
        if (e != numErrors) {
            throw ChecksumException.getChecksumInstance();
        }
        return result;
    }

    private int[] findErrorMagnitudes(ModulusPoly errorEvaluator, ModulusPoly errorLocator, int[] errorLocations) {
        int errorLocatorDegree = errorLocator.getDegree();
        int[] formalDerivativeCoefficients = new int[errorLocatorDegree];
        for (int i = 1; i <= errorLocatorDegree; i++) {
            formalDerivativeCoefficients[errorLocatorDegree - i] = this.field.multiply(i, errorLocator.getCoefficient(i));
        }
        ModulusPoly formalDerivative = new ModulusPoly(this.field, formalDerivativeCoefficients);
        int s = errorLocations.length;
        int[] result = new int[s];
        for (int i2 = 0; i2 < s; i2++) {
            int xiInverse = this.field.inverse(errorLocations[i2]);
            int numerator = this.field.subtract(0, errorEvaluator.evaluateAt(xiInverse));
            int denominator = this.field.inverse(formalDerivative.evaluateAt(xiInverse));
            result[i2] = this.field.multiply(numerator, denominator);
        }
        return result;
    }
}
