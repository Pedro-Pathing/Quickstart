package com.google.zxing.client.result;

import java.util.Map;
import java.util.Objects;

/* JADX INFO: loaded from: classes8.dex */
public final class ExpandedProductParsedResult extends ParsedResult {
    public static final String KILOGRAM = "KG";
    public static final String POUND = "LB";
    private final String bestBeforeDate;
    private final String expirationDate;
    private final String lotNumber;
    private final String packagingDate;
    private final String price;
    private final String priceCurrency;
    private final String priceIncrement;
    private final String productID;
    private final String productionDate;
    private final String rawText;
    private final String sscc;
    private final Map<String, String> uncommonAIs;
    private final String weight;
    private final String weightIncrement;
    private final String weightType;

    public ExpandedProductParsedResult(String rawText, String productID, String sscc, String lotNumber, String productionDate, String packagingDate, String bestBeforeDate, String expirationDate, String weight, String weightType, String weightIncrement, String price, String priceIncrement, String priceCurrency, Map<String, String> uncommonAIs) {
        super(ParsedResultType.PRODUCT);
        this.rawText = rawText;
        this.productID = productID;
        this.sscc = sscc;
        this.lotNumber = lotNumber;
        this.productionDate = productionDate;
        this.packagingDate = packagingDate;
        this.bestBeforeDate = bestBeforeDate;
        this.expirationDate = expirationDate;
        this.weight = weight;
        this.weightType = weightType;
        this.weightIncrement = weightIncrement;
        this.price = price;
        this.priceIncrement = priceIncrement;
        this.priceCurrency = priceCurrency;
        this.uncommonAIs = uncommonAIs;
    }

    public boolean equals(Object o) {
        if (!(o instanceof ExpandedProductParsedResult)) {
            return false;
        }
        ExpandedProductParsedResult other = (ExpandedProductParsedResult) o;
        return Objects.equals(this.productID, other.productID) && Objects.equals(this.sscc, other.sscc) && Objects.equals(this.lotNumber, other.lotNumber) && Objects.equals(this.productionDate, other.productionDate) && Objects.equals(this.bestBeforeDate, other.bestBeforeDate) && Objects.equals(this.expirationDate, other.expirationDate) && Objects.equals(this.weight, other.weight) && Objects.equals(this.weightType, other.weightType) && Objects.equals(this.weightIncrement, other.weightIncrement) && Objects.equals(this.price, other.price) && Objects.equals(this.priceIncrement, other.priceIncrement) && Objects.equals(this.priceCurrency, other.priceCurrency) && Objects.equals(this.uncommonAIs, other.uncommonAIs);
    }

    public int hashCode() {
        return (((((((((((Objects.hashCode(this.productID) ^ Objects.hashCode(this.sscc)) ^ Objects.hashCode(this.lotNumber)) ^ Objects.hashCode(this.productionDate)) ^ Objects.hashCode(this.bestBeforeDate)) ^ Objects.hashCode(this.expirationDate)) ^ Objects.hashCode(this.weight)) ^ Objects.hashCode(this.weightType)) ^ Objects.hashCode(this.weightIncrement)) ^ Objects.hashCode(this.price)) ^ Objects.hashCode(this.priceIncrement)) ^ Objects.hashCode(this.priceCurrency)) ^ Objects.hashCode(this.uncommonAIs);
    }

    public String getRawText() {
        return this.rawText;
    }

    public String getProductID() {
        return this.productID;
    }

    public String getSscc() {
        return this.sscc;
    }

    public String getLotNumber() {
        return this.lotNumber;
    }

    public String getProductionDate() {
        return this.productionDate;
    }

    public String getPackagingDate() {
        return this.packagingDate;
    }

    public String getBestBeforeDate() {
        return this.bestBeforeDate;
    }

    public String getExpirationDate() {
        return this.expirationDate;
    }

    public String getWeight() {
        return this.weight;
    }

    public String getWeightType() {
        return this.weightType;
    }

    public String getWeightIncrement() {
        return this.weightIncrement;
    }

    public String getPrice() {
        return this.price;
    }

    public String getPriceIncrement() {
        return this.priceIncrement;
    }

    public String getPriceCurrency() {
        return this.priceCurrency;
    }

    public Map<String, String> getUncommonAIs() {
        return this.uncommonAIs;
    }

    @Override // com.google.zxing.client.result.ParsedResult
    public String getDisplayResult() {
        return String.valueOf(this.rawText);
    }
}
