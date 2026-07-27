package com.google.zxing.client.result;

/* JADX INFO: loaded from: classes8.dex */
public final class ProductParsedResult extends ParsedResult {
    private final String normalizedProductID;
    private final String productID;

    ProductParsedResult(String productID) {
        this(productID, productID);
    }

    ProductParsedResult(String productID, String normalizedProductID) {
        super(ParsedResultType.PRODUCT);
        this.productID = productID;
        this.normalizedProductID = normalizedProductID;
    }

    public String getProductID() {
        return this.productID;
    }

    public String getNormalizedProductID() {
        return this.normalizedProductID;
    }

    @Override // com.google.zxing.client.result.ParsedResult
    public String getDisplayResult() {
        return this.productID;
    }
}
