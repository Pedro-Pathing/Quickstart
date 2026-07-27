package com.google.zxing.common.reedsolomon;

/* JADX INFO: loaded from: classes8.dex */
public final class ReedSolomonDecoder {
    private final GenericGF field;

    public ReedSolomonDecoder(GenericGF field) {
        this.field = field;
    }

    public void decode(int[] iArr, int i) throws ReedSolomonException {
        GenericGFPoly genericGFPoly = new GenericGFPoly(this.field, iArr);
        int[] iArr2 = new int[i];
        boolean z = true;
        for (int i2 = 0; i2 < i; i2++) {
            GenericGF genericGF = this.field;
            int iEvaluateAt = genericGFPoly.evaluateAt(genericGF.exp(genericGF.getGeneratorBase() + i2));
            iArr2[(i - 1) - i2] = iEvaluateAt;
            if (iEvaluateAt != 0) {
                z = false;
            }
        }
        if (z) {
            return;
        }
        GenericGFPoly[] genericGFPolyArrRunEuclideanAlgorithm = runEuclideanAlgorithm(this.field.buildMonomial(i, 1), new GenericGFPoly(this.field, iArr2), i);
        GenericGFPoly genericGFPoly2 = genericGFPolyArrRunEuclideanAlgorithm[0];
        GenericGFPoly genericGFPoly3 = genericGFPolyArrRunEuclideanAlgorithm[1];
        int[] iArrFindErrorLocations = findErrorLocations(genericGFPoly2);
        int[] iArrFindErrorMagnitudes = findErrorMagnitudes(genericGFPoly3, iArrFindErrorLocations);
        for (int i3 = 0; i3 < iArrFindErrorLocations.length; i3++) {
            int length = (iArr.length - 1) - this.field.log(iArrFindErrorLocations[i3]);
            if (length < 0) {
                throw new ReedSolomonException("Bad error location");
            }
            iArr[length] = GenericGF.addOrSubtract(iArr[length], iArrFindErrorMagnitudes[i3]);
        }
    }

    private GenericGFPoly[] runEuclideanAlgorithm(GenericGFPoly genericGFPoly, GenericGFPoly genericGFPoly2, int i) throws ReedSolomonException {
        if (genericGFPoly.getDegree() < genericGFPoly2.getDegree()) {
            genericGFPoly2 = genericGFPoly;
            genericGFPoly = genericGFPoly2;
        }
        GenericGFPoly zero = this.field.getZero();
        GenericGFPoly one = this.field.getOne();
        do {
            GenericGFPoly genericGFPoly3 = genericGFPoly2;
            genericGFPoly2 = genericGFPoly;
            genericGFPoly = genericGFPoly3;
            GenericGFPoly genericGFPoly4 = one;
            GenericGFPoly genericGFPoly5 = zero;
            zero = genericGFPoly4;
            if (genericGFPoly.getDegree() >= i / 2) {
                if (genericGFPoly.isZero()) {
                    throw new ReedSolomonException("r_{i-1} was zero");
                }
                GenericGFPoly zero2 = this.field.getZero();
                int iInverse = this.field.inverse(genericGFPoly.getCoefficient(genericGFPoly.getDegree()));
                while (genericGFPoly2.getDegree() >= genericGFPoly.getDegree() && !genericGFPoly2.isZero()) {
                    int degree = genericGFPoly2.getDegree() - genericGFPoly.getDegree();
                    int iMultiply = this.field.multiply(genericGFPoly2.getCoefficient(genericGFPoly2.getDegree()), iInverse);
                    zero2 = zero2.addOrSubtract(this.field.buildMonomial(degree, iMultiply));
                    genericGFPoly2 = genericGFPoly2.addOrSubtract(genericGFPoly.multiplyByMonomial(degree, iMultiply));
                }
                one = zero2.multiply(zero).addOrSubtract(genericGFPoly5);
            } else {
                int coefficient = zero.getCoefficient(0);
                if (coefficient == 0) {
                    throw new ReedSolomonException("sigmaTilde(0) was zero");
                }
                int iInverse2 = this.field.inverse(coefficient);
                return new GenericGFPoly[]{zero.multiply(iInverse2), genericGFPoly.multiply(iInverse2)};
            }
        } while (genericGFPoly2.getDegree() < genericGFPoly.getDegree());
        throw new IllegalStateException("Division algorithm failed to reduce polynomial?");
    }

    private int[] findErrorLocations(GenericGFPoly errorLocator) throws ReedSolomonException {
        int numErrors = errorLocator.getDegree();
        if (numErrors == 1) {
            return new int[]{errorLocator.getCoefficient(1)};
        }
        int[] result = new int[numErrors];
        int e = 0;
        for (int i = 1; i < this.field.getSize() && e < numErrors; i++) {
            if (errorLocator.evaluateAt(i) == 0) {
                result[e] = this.field.inverse(i);
                e++;
            }
        }
        if (e != numErrors) {
            throw new ReedSolomonException("Error locator degree does not match number of roots");
        }
        return result;
    }

    private int[] findErrorMagnitudes(GenericGFPoly errorEvaluator, int[] errorLocations) {
        int s = errorLocations.length;
        int[] result = new int[s];
        for (int i = 0; i < s; i++) {
            int xiInverse = this.field.inverse(errorLocations[i]);
            int denominator = 1;
            for (int j = 0; j < s; j++) {
                if (i != j) {
                    int term = this.field.multiply(errorLocations[j], xiInverse);
                    int termPlus1 = (term & 1) == 0 ? term | 1 : term & (-2);
                    denominator = this.field.multiply(denominator, termPlus1);
                }
            }
            result[i] = this.field.multiply(errorEvaluator.evaluateAt(xiInverse), this.field.inverse(denominator));
            if (this.field.getGeneratorBase() != 0) {
                result[i] = this.field.multiply(result[i], xiInverse);
            }
        }
        return result;
    }
}
