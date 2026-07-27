package com.google.gson.internal.bind;

import com.google.gson.Gson;
import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonIOException;
import com.google.gson.JsonNull;
import com.google.gson.JsonObject;
import com.google.gson.JsonPrimitive;
import com.google.gson.JsonSyntaxException;
import com.google.gson.TypeAdapter;
import com.google.gson.TypeAdapterFactory;
import com.google.gson.annotations.SerializedName;
import com.google.gson.internal.LazilyParsedNumber;
import com.google.gson.reflect.TypeToken;
import com.google.gson.stream.JsonReader;
import com.google.gson.stream.JsonToken;
import com.google.gson.stream.JsonWriter;
import java.io.IOException;
import java.math.BigDecimal;
import java.math.BigInteger;
import java.net.InetAddress;
import java.net.URI;
import java.net.URISyntaxException;
import java.net.URL;
import java.sql.Timestamp;
import java.util.ArrayList;
import java.util.BitSet;
import java.util.Calendar;
import java.util.Currency;
import java.util.Date;
import java.util.GregorianCalendar;
import java.util.HashMap;
import java.util.Iterator;
import java.util.List;
import java.util.Locale;
import java.util.Map;
import java.util.StringTokenizer;
import java.util.UUID;
import java.util.concurrent.atomic.AtomicBoolean;
import java.util.concurrent.atomic.AtomicInteger;
import java.util.concurrent.atomic.AtomicIntegerArray;
import org.slf4j.Marker;

/* JADX INFO: loaded from: classes8.dex */
public final class TypeAdapters {
    public static final TypeAdapter<Class> CLASS = new TypeAdapter<Class>() { // from class: com.google.gson.internal.bind.TypeAdapters.1
        @Override // com.google.gson.TypeAdapter
        public void write(JsonWriter out, Class value) throws IOException {
            if (value == null) {
                out.nullValue();
                return;
            }
            throw new UnsupportedOperationException("Attempted to serialize java.lang.Class: " + value.getName() + ". Forgot to register a type adapter?");
        }

        /* JADX WARN: Can't rename method to resolve collision */
        @Override // com.google.gson.TypeAdapter
        /* JADX INFO: renamed from: read */
        public Class read2(JsonReader in) throws IOException {
            if (in.peek() == JsonToken.NULL) {
                in.nextNull();
                return null;
            }
            throw new UnsupportedOperationException("Attempted to deserialize a java.lang.Class. Forgot to register a type adapter?");
        }
    };
    public static final TypeAdapterFactory CLASS_FACTORY = newFactory(Class.class, CLASS);
    public static final TypeAdapter<BitSet> BIT_SET = new TypeAdapter<BitSet>() { // from class: com.google.gson.internal.bind.TypeAdapters.2
        @Override // com.google.gson.TypeAdapter
        /* JADX INFO: renamed from: read, reason: avoid collision after fix types in other method */
        public BitSet read2(JsonReader in) throws IOException {
            if (in.peek() == JsonToken.NULL) {
                in.nextNull();
                return null;
            }
            BitSet bitset = new BitSet();
            in.beginArray();
            int i = 0;
            JsonToken tokenType = in.peek();
            while (tokenType != JsonToken.END_ARRAY) {
                boolean set = false;
                switch (AnonymousClass36.$SwitchMap$com$google$gson$stream$JsonToken[tokenType.ordinal()]) {
                    case 1:
                        if (in.nextInt() != 0) {
                            set = true;
                        }
                        break;
                    case 2:
                        set = in.nextBoolean();
                        break;
                    case 3:
                        String stringValue = in.nextString();
                        try {
                            if (Integer.parseInt(stringValue) != 0) {
                                set = true;
                            }
                        } catch (NumberFormatException e) {
                            throw new JsonSyntaxException("Error: Expecting: bitset number value (1, 0), Found: " + stringValue);
                        }
                        break;
                    default:
                        throw new JsonSyntaxException("Invalid bitset value type: " + tokenType);
                }
                if (set) {
                    bitset.set(i);
                }
                i++;
                tokenType = in.peek();
            }
            in.endArray();
            return bitset;
        }

        @Override // com.google.gson.TypeAdapter
        public void write(JsonWriter jsonWriter, BitSet bitSet) throws IOException {
            if (bitSet == null) {
                jsonWriter.nullValue();
                return;
            }
            jsonWriter.beginArray();
            for (int i = 0; i < bitSet.length(); i++) {
                jsonWriter.value(bitSet.get(i) ? 1L : 0L);
            }
            jsonWriter.endArray();
        }
    };
    public static final TypeAdapterFactory BIT_SET_FACTORY = newFactory(BitSet.class, BIT_SET);
    public static final TypeAdapter<Boolean> BOOLEAN = new TypeAdapter<Boolean>() { // from class: com.google.gson.internal.bind.TypeAdapters.3
        /* JADX WARN: Can't rename method to resolve collision */
        @Override // com.google.gson.TypeAdapter
        /* JADX INFO: renamed from: read */
        public Boolean read2(JsonReader in) throws IOException {
            if (in.peek() == JsonToken.NULL) {
                in.nextNull();
                return null;
            }
            if (in.peek() == JsonToken.STRING) {
                return Boolean.valueOf(Boolean.parseBoolean(in.nextString()));
            }
            return Boolean.valueOf(in.nextBoolean());
        }

        @Override // com.google.gson.TypeAdapter
        public void write(JsonWriter out, Boolean value) throws IOException {
            out.value(value);
        }
    };
    public static final TypeAdapter<Boolean> BOOLEAN_AS_STRING = new TypeAdapter<Boolean>() { // from class: com.google.gson.internal.bind.TypeAdapters.4
        /* JADX WARN: Can't rename method to resolve collision */
        @Override // com.google.gson.TypeAdapter
        /* JADX INFO: renamed from: read */
        public Boolean read2(JsonReader in) throws IOException {
            if (in.peek() == JsonToken.NULL) {
                in.nextNull();
                return null;
            }
            return Boolean.valueOf(in.nextString());
        }

        @Override // com.google.gson.TypeAdapter
        public void write(JsonWriter out, Boolean value) throws IOException {
            out.value(value == null ? "null" : value.toString());
        }
    };
    public static final TypeAdapterFactory BOOLEAN_FACTORY = newFactory(Boolean.TYPE, Boolean.class, BOOLEAN);
    public static final TypeAdapter<Number> BYTE = new TypeAdapter<Number>() { // from class: com.google.gson.internal.bind.TypeAdapters.5
        /* JADX WARN: Can't rename method to resolve collision */
        @Override // com.google.gson.TypeAdapter
        /* JADX INFO: renamed from: read */
        public Number read2(JsonReader in) throws IOException {
            if (in.peek() == JsonToken.NULL) {
                in.nextNull();
                return null;
            }
            try {
                int intValue = in.nextInt();
                return Byte.valueOf((byte) intValue);
            } catch (NumberFormatException e) {
                throw new JsonSyntaxException(e);
            }
        }

        @Override // com.google.gson.TypeAdapter
        public void write(JsonWriter out, Number value) throws IOException {
            out.value(value);
        }
    };
    public static final TypeAdapterFactory BYTE_FACTORY = newFactory(Byte.TYPE, Byte.class, BYTE);
    public static final TypeAdapter<Number> SHORT = new TypeAdapter<Number>() { // from class: com.google.gson.internal.bind.TypeAdapters.6
        /* JADX WARN: Can't rename method to resolve collision */
        @Override // com.google.gson.TypeAdapter
        /* JADX INFO: renamed from: read */
        public Number read2(JsonReader in) throws IOException {
            if (in.peek() == JsonToken.NULL) {
                in.nextNull();
                return null;
            }
            try {
                return Short.valueOf((short) in.nextInt());
            } catch (NumberFormatException e) {
                throw new JsonSyntaxException(e);
            }
        }

        @Override // com.google.gson.TypeAdapter
        public void write(JsonWriter out, Number value) throws IOException {
            out.value(value);
        }
    };
    public static final TypeAdapterFactory SHORT_FACTORY = newFactory(Short.TYPE, Short.class, SHORT);
    public static final TypeAdapter<Number> INTEGER = new TypeAdapter<Number>() { // from class: com.google.gson.internal.bind.TypeAdapters.7
        /* JADX WARN: Can't rename method to resolve collision */
        @Override // com.google.gson.TypeAdapter
        /* JADX INFO: renamed from: read */
        public Number read2(JsonReader in) throws IOException {
            if (in.peek() == JsonToken.NULL) {
                in.nextNull();
                return null;
            }
            try {
                return Integer.valueOf(in.nextInt());
            } catch (NumberFormatException e) {
                throw new JsonSyntaxException(e);
            }
        }

        @Override // com.google.gson.TypeAdapter
        public void write(JsonWriter out, Number value) throws IOException {
            out.value(value);
        }
    };
    public static final TypeAdapterFactory INTEGER_FACTORY = newFactory(Integer.TYPE, Integer.class, INTEGER);
    public static final TypeAdapter<AtomicInteger> ATOMIC_INTEGER = new TypeAdapter<AtomicInteger>() { // from class: com.google.gson.internal.bind.TypeAdapters.8
        @Override // com.google.gson.TypeAdapter
        /* JADX INFO: renamed from: read, reason: avoid collision after fix types in other method */
        public AtomicInteger read2(JsonReader in) throws IOException {
            try {
                return new AtomicInteger(in.nextInt());
            } catch (NumberFormatException e) {
                throw new JsonSyntaxException(e);
            }
        }

        @Override // com.google.gson.TypeAdapter
        public void write(JsonWriter out, AtomicInteger value) throws IOException {
            out.value(value.get());
        }
    }.nullSafe();
    public static final TypeAdapterFactory ATOMIC_INTEGER_FACTORY = newFactory(AtomicInteger.class, ATOMIC_INTEGER);
    public static final TypeAdapter<AtomicBoolean> ATOMIC_BOOLEAN = new TypeAdapter<AtomicBoolean>() { // from class: com.google.gson.internal.bind.TypeAdapters.9
        @Override // com.google.gson.TypeAdapter
        /* JADX INFO: renamed from: read, reason: avoid collision after fix types in other method */
        public AtomicBoolean read2(JsonReader in) throws IOException {
            return new AtomicBoolean(in.nextBoolean());
        }

        @Override // com.google.gson.TypeAdapter
        public void write(JsonWriter out, AtomicBoolean value) throws IOException {
            out.value(value.get());
        }
    }.nullSafe();
    public static final TypeAdapterFactory ATOMIC_BOOLEAN_FACTORY = newFactory(AtomicBoolean.class, ATOMIC_BOOLEAN);
    public static final TypeAdapter<AtomicIntegerArray> ATOMIC_INTEGER_ARRAY = new TypeAdapter<AtomicIntegerArray>() { // from class: com.google.gson.internal.bind.TypeAdapters.10
        @Override // com.google.gson.TypeAdapter
        /* JADX INFO: renamed from: read, reason: avoid collision after fix types in other method */
        public AtomicIntegerArray read2(JsonReader in) throws IOException {
            List<Integer> list = new ArrayList<>();
            in.beginArray();
            while (in.hasNext()) {
                try {
                    int integer = in.nextInt();
                    list.add(Integer.valueOf(integer));
                } catch (NumberFormatException e) {
                    throw new JsonSyntaxException(e);
                }
            }
            in.endArray();
            int length = list.size();
            AtomicIntegerArray array = new AtomicIntegerArray(length);
            for (int i = 0; i < length; i++) {
                array.set(i, list.get(i).intValue());
            }
            return array;
        }

        @Override // com.google.gson.TypeAdapter
        public void write(JsonWriter out, AtomicIntegerArray value) throws IOException {
            out.beginArray();
            int length = value.length();
            for (int i = 0; i < length; i++) {
                out.value(value.get(i));
            }
            out.endArray();
        }
    }.nullSafe();
    public static final TypeAdapterFactory ATOMIC_INTEGER_ARRAY_FACTORY = newFactory(AtomicIntegerArray.class, ATOMIC_INTEGER_ARRAY);
    public static final TypeAdapter<Number> LONG = new TypeAdapter<Number>() { // from class: com.google.gson.internal.bind.TypeAdapters.11
        /* JADX WARN: Can't rename method to resolve collision */
        @Override // com.google.gson.TypeAdapter
        /* JADX INFO: renamed from: read */
        public Number read2(JsonReader in) throws IOException {
            if (in.peek() == JsonToken.NULL) {
                in.nextNull();
                return null;
            }
            try {
                return Long.valueOf(in.nextLong());
            } catch (NumberFormatException e) {
                throw new JsonSyntaxException(e);
            }
        }

        @Override // com.google.gson.TypeAdapter
        public void write(JsonWriter out, Number value) throws IOException {
            out.value(value);
        }
    };
    public static final TypeAdapter<Number> FLOAT = new TypeAdapter<Number>() { // from class: com.google.gson.internal.bind.TypeAdapters.12
        /* JADX WARN: Can't rename method to resolve collision */
        @Override // com.google.gson.TypeAdapter
        /* JADX INFO: renamed from: read */
        public Number read2(JsonReader in) throws IOException {
            if (in.peek() == JsonToken.NULL) {
                in.nextNull();
                return null;
            }
            return Float.valueOf((float) in.nextDouble());
        }

        @Override // com.google.gson.TypeAdapter
        public void write(JsonWriter out, Number value) throws IOException {
            out.value(value);
        }
    };
    public static final TypeAdapter<Number> DOUBLE = new TypeAdapter<Number>() { // from class: com.google.gson.internal.bind.TypeAdapters.13
        /* JADX WARN: Can't rename method to resolve collision */
        @Override // com.google.gson.TypeAdapter
        /* JADX INFO: renamed from: read */
        public Number read2(JsonReader in) throws IOException {
            if (in.peek() == JsonToken.NULL) {
                in.nextNull();
                return null;
            }
            return Double.valueOf(in.nextDouble());
        }

        @Override // com.google.gson.TypeAdapter
        public void write(JsonWriter out, Number value) throws IOException {
            out.value(value);
        }
    };
    public static final TypeAdapter<Number> NUMBER = new TypeAdapter<Number>() { // from class: com.google.gson.internal.bind.TypeAdapters.14
        /* JADX WARN: Can't rename method to resolve collision */
        @Override // com.google.gson.TypeAdapter
        /* JADX INFO: renamed from: read */
        public Number read2(JsonReader in) throws IOException {
            JsonToken jsonToken = in.peek();
            switch (AnonymousClass36.$SwitchMap$com$google$gson$stream$JsonToken[jsonToken.ordinal()]) {
                case 1:
                    return new LazilyParsedNumber(in.nextString());
                case 4:
                    in.nextNull();
                    return null;
                default:
                    throw new JsonSyntaxException("Expecting number, got: " + jsonToken);
            }
        }

        @Override // com.google.gson.TypeAdapter
        public void write(JsonWriter out, Number value) throws IOException {
            out.value(value);
        }
    };
    public static final TypeAdapterFactory NUMBER_FACTORY = newFactory(Number.class, NUMBER);
    public static final TypeAdapter<Character> CHARACTER = new TypeAdapter<Character>() { // from class: com.google.gson.internal.bind.TypeAdapters.15
        /* JADX WARN: Can't rename method to resolve collision */
        @Override // com.google.gson.TypeAdapter
        /* JADX INFO: renamed from: read */
        public Character read2(JsonReader in) throws IOException {
            if (in.peek() == JsonToken.NULL) {
                in.nextNull();
                return null;
            }
            String str = in.nextString();
            if (str.length() != 1) {
                throw new JsonSyntaxException("Expecting character, got: " + str);
            }
            return Character.valueOf(str.charAt(0));
        }

        @Override // com.google.gson.TypeAdapter
        public void write(JsonWriter out, Character value) throws IOException {
            out.value(value == null ? null : String.valueOf(value));
        }
    };
    public static final TypeAdapterFactory CHARACTER_FACTORY = newFactory(Character.TYPE, Character.class, CHARACTER);
    public static final TypeAdapter<String> STRING = new TypeAdapter<String>() { // from class: com.google.gson.internal.bind.TypeAdapters.16
        @Override // com.google.gson.TypeAdapter
        /* JADX INFO: renamed from: read, reason: avoid collision after fix types in other method */
        public String read2(JsonReader in) throws IOException {
            JsonToken peek = in.peek();
            if (peek == JsonToken.NULL) {
                in.nextNull();
                return null;
            }
            if (peek == JsonToken.BOOLEAN) {
                return Boolean.toString(in.nextBoolean());
            }
            return in.nextString();
        }

        @Override // com.google.gson.TypeAdapter
        public void write(JsonWriter out, String value) throws IOException {
            out.value(value);
        }
    };
    public static final TypeAdapter<BigDecimal> BIG_DECIMAL = new TypeAdapter<BigDecimal>() { // from class: com.google.gson.internal.bind.TypeAdapters.17
        @Override // com.google.gson.TypeAdapter
        /* JADX INFO: renamed from: read, reason: avoid collision after fix types in other method */
        public BigDecimal read2(JsonReader in) throws IOException {
            if (in.peek() == JsonToken.NULL) {
                in.nextNull();
                return null;
            }
            try {
                return new BigDecimal(in.nextString());
            } catch (NumberFormatException e) {
                throw new JsonSyntaxException(e);
            }
        }

        @Override // com.google.gson.TypeAdapter
        public void write(JsonWriter out, BigDecimal value) throws IOException {
            out.value(value);
        }
    };
    public static final TypeAdapter<BigInteger> BIG_INTEGER = new TypeAdapter<BigInteger>() { // from class: com.google.gson.internal.bind.TypeAdapters.18
        @Override // com.google.gson.TypeAdapter
        /* JADX INFO: renamed from: read, reason: avoid collision after fix types in other method */
        public BigInteger read2(JsonReader in) throws IOException {
            if (in.peek() == JsonToken.NULL) {
                in.nextNull();
                return null;
            }
            try {
                return new BigInteger(in.nextString());
            } catch (NumberFormatException e) {
                throw new JsonSyntaxException(e);
            }
        }

        @Override // com.google.gson.TypeAdapter
        public void write(JsonWriter out, BigInteger value) throws IOException {
            out.value(value);
        }
    };
    public static final TypeAdapterFactory STRING_FACTORY = newFactory(String.class, STRING);
    public static final TypeAdapter<StringBuilder> STRING_BUILDER = new TypeAdapter<StringBuilder>() { // from class: com.google.gson.internal.bind.TypeAdapters.19
        @Override // com.google.gson.TypeAdapter
        /* JADX INFO: renamed from: read, reason: avoid collision after fix types in other method */
        public StringBuilder read2(JsonReader in) throws IOException {
            if (in.peek() == JsonToken.NULL) {
                in.nextNull();
                return null;
            }
            return new StringBuilder(in.nextString());
        }

        @Override // com.google.gson.TypeAdapter
        public void write(JsonWriter out, StringBuilder value) throws IOException {
            out.value(value == null ? null : value.toString());
        }
    };
    public static final TypeAdapterFactory STRING_BUILDER_FACTORY = newFactory(StringBuilder.class, STRING_BUILDER);
    public static final TypeAdapter<StringBuffer> STRING_BUFFER = new TypeAdapter<StringBuffer>() { // from class: com.google.gson.internal.bind.TypeAdapters.20
        @Override // com.google.gson.TypeAdapter
        /* JADX INFO: renamed from: read, reason: avoid collision after fix types in other method */
        public StringBuffer read2(JsonReader in) throws IOException {
            if (in.peek() == JsonToken.NULL) {
                in.nextNull();
                return null;
            }
            return new StringBuffer(in.nextString());
        }

        @Override // com.google.gson.TypeAdapter
        public void write(JsonWriter out, StringBuffer value) throws IOException {
            out.value(value == null ? null : value.toString());
        }
    };
    public static final TypeAdapterFactory STRING_BUFFER_FACTORY = newFactory(StringBuffer.class, STRING_BUFFER);
    public static final TypeAdapter<URL> URL = new TypeAdapter<URL>() { // from class: com.google.gson.internal.bind.TypeAdapters.21
        @Override // com.google.gson.TypeAdapter
        /* JADX INFO: renamed from: read, reason: avoid collision after fix types in other method */
        public URL read2(JsonReader in) throws IOException {
            if (in.peek() == JsonToken.NULL) {
                in.nextNull();
                return null;
            }
            String nextString = in.nextString();
            if ("null".equals(nextString)) {
                return null;
            }
            return new URL(nextString);
        }

        @Override // com.google.gson.TypeAdapter
        public void write(JsonWriter out, URL value) throws IOException {
            out.value(value == null ? null : value.toExternalForm());
        }
    };
    public static final TypeAdapterFactory URL_FACTORY = newFactory(URL.class, URL);
    public static final TypeAdapter<URI> URI = new TypeAdapter<URI>() { // from class: com.google.gson.internal.bind.TypeAdapters.22
        @Override // com.google.gson.TypeAdapter
        /* JADX INFO: renamed from: read, reason: avoid collision after fix types in other method */
        public URI read2(JsonReader in) throws IOException {
            if (in.peek() == JsonToken.NULL) {
                in.nextNull();
                return null;
            }
            try {
                String nextString = in.nextString();
                return "null".equals(nextString) ? null : new URI(nextString);
            } catch (URISyntaxException e) {
                throw new JsonIOException(e);
            }
        }

        @Override // com.google.gson.TypeAdapter
        public void write(JsonWriter out, URI value) throws IOException {
            out.value(value == null ? null : value.toASCIIString());
        }
    };
    public static final TypeAdapterFactory URI_FACTORY = newFactory(URI.class, URI);
    public static final TypeAdapter<InetAddress> INET_ADDRESS = new TypeAdapter<InetAddress>() { // from class: com.google.gson.internal.bind.TypeAdapters.23
        @Override // com.google.gson.TypeAdapter
        /* JADX INFO: renamed from: read, reason: avoid collision after fix types in other method */
        public InetAddress read2(JsonReader in) throws IOException {
            if (in.peek() == JsonToken.NULL) {
                in.nextNull();
                return null;
            }
            return InetAddress.getByName(in.nextString());
        }

        @Override // com.google.gson.TypeAdapter
        public void write(JsonWriter out, InetAddress value) throws IOException {
            out.value(value == null ? null : value.getHostAddress());
        }
    };
    public static final TypeAdapterFactory INET_ADDRESS_FACTORY = newTypeHierarchyFactory(InetAddress.class, INET_ADDRESS);
    public static final TypeAdapter<UUID> UUID = new TypeAdapter<UUID>() { // from class: com.google.gson.internal.bind.TypeAdapters.24
        @Override // com.google.gson.TypeAdapter
        /* JADX INFO: renamed from: read, reason: avoid collision after fix types in other method */
        public UUID read2(JsonReader in) throws IOException {
            if (in.peek() == JsonToken.NULL) {
                in.nextNull();
                return null;
            }
            return UUID.fromString(in.nextString());
        }

        @Override // com.google.gson.TypeAdapter
        public void write(JsonWriter out, UUID value) throws IOException {
            out.value(value == null ? null : value.toString());
        }
    };
    public static final TypeAdapterFactory UUID_FACTORY = newFactory(UUID.class, UUID);
    public static final TypeAdapter<Currency> CURRENCY = new TypeAdapter<Currency>() { // from class: com.google.gson.internal.bind.TypeAdapters.25
        @Override // com.google.gson.TypeAdapter
        /* JADX INFO: renamed from: read, reason: avoid collision after fix types in other method */
        public Currency read2(JsonReader in) throws IOException {
            return Currency.getInstance(in.nextString());
        }

        @Override // com.google.gson.TypeAdapter
        public void write(JsonWriter out, Currency value) throws IOException {
            out.value(value.getCurrencyCode());
        }
    }.nullSafe();
    public static final TypeAdapterFactory CURRENCY_FACTORY = newFactory(Currency.class, CURRENCY);
    public static final TypeAdapterFactory TIMESTAMP_FACTORY = new TypeAdapterFactory() { // from class: com.google.gson.internal.bind.TypeAdapters.26
        @Override // com.google.gson.TypeAdapterFactory
        public <T> TypeAdapter<T> create(Gson gson, TypeToken<T> typeToken) {
            if (typeToken.getRawType() != Timestamp.class) {
                return null;
            }
            final TypeAdapter<T> adapter = gson.getAdapter(Date.class);
            return (TypeAdapter<T>) new TypeAdapter<Timestamp>() { // from class: com.google.gson.internal.bind.TypeAdapters.26.1
                @Override // com.google.gson.TypeAdapter
                /* JADX INFO: renamed from: read, reason: avoid collision after fix types in other method */
                public Timestamp read2(JsonReader in) throws IOException {
                    Date date = (Date) adapter.read2(in);
                    if (date != null) {
                        return new Timestamp(date.getTime());
                    }
                    return null;
                }

                @Override // com.google.gson.TypeAdapter
                public void write(JsonWriter out, Timestamp value) throws IOException {
                    adapter.write(out, value);
                }
            };
        }
    };
    public static final TypeAdapter<Calendar> CALENDAR = new TypeAdapter<Calendar>() { // from class: com.google.gson.internal.bind.TypeAdapters.27
        private static final String DAY_OF_MONTH = "dayOfMonth";
        private static final String HOUR_OF_DAY = "hourOfDay";
        private static final String MINUTE = "minute";
        private static final String MONTH = "month";
        private static final String SECOND = "second";
        private static final String YEAR = "year";

        @Override // com.google.gson.TypeAdapter
        /* JADX INFO: renamed from: read, reason: avoid collision after fix types in other method */
        public Calendar read2(JsonReader in) throws IOException {
            if (in.peek() == JsonToken.NULL) {
                in.nextNull();
                return null;
            }
            in.beginObject();
            int year = 0;
            int month = 0;
            int dayOfMonth = 0;
            int hourOfDay = 0;
            int minute = 0;
            int second = 0;
            while (in.peek() != JsonToken.END_OBJECT) {
                String name = in.nextName();
                int value = in.nextInt();
                if (YEAR.equals(name)) {
                    year = value;
                } else if (MONTH.equals(name)) {
                    month = value;
                } else if (DAY_OF_MONTH.equals(name)) {
                    dayOfMonth = value;
                } else if (HOUR_OF_DAY.equals(name)) {
                    hourOfDay = value;
                } else if (MINUTE.equals(name)) {
                    minute = value;
                } else if (SECOND.equals(name)) {
                    second = value;
                }
            }
            in.endObject();
            return new GregorianCalendar(year, month, dayOfMonth, hourOfDay, minute, second);
        }

        @Override // com.google.gson.TypeAdapter
        public void write(JsonWriter out, Calendar value) throws IOException {
            if (value == null) {
                out.nullValue();
                return;
            }
            out.beginObject();
            out.name(YEAR);
            out.value(value.get(1));
            out.name(MONTH);
            out.value(value.get(2));
            out.name(DAY_OF_MONTH);
            out.value(value.get(5));
            out.name(HOUR_OF_DAY);
            out.value(value.get(11));
            out.name(MINUTE);
            out.value(value.get(12));
            out.name(SECOND);
            out.value(value.get(13));
            out.endObject();
        }
    };
    public static final TypeAdapterFactory CALENDAR_FACTORY = newFactoryForMultipleTypes(Calendar.class, GregorianCalendar.class, CALENDAR);
    public static final TypeAdapter<Locale> LOCALE = new TypeAdapter<Locale>() { // from class: com.google.gson.internal.bind.TypeAdapters.28
        @Override // com.google.gson.TypeAdapter
        /* JADX INFO: renamed from: read, reason: avoid collision after fix types in other method */
        public Locale read2(JsonReader in) throws IOException {
            if (in.peek() == JsonToken.NULL) {
                in.nextNull();
                return null;
            }
            String locale = in.nextString();
            StringTokenizer tokenizer = new StringTokenizer(locale, "_");
            String language = null;
            String country = null;
            String variant = null;
            if (tokenizer.hasMoreElements()) {
                language = tokenizer.nextToken();
            }
            if (tokenizer.hasMoreElements()) {
                country = tokenizer.nextToken();
            }
            if (tokenizer.hasMoreElements()) {
                variant = tokenizer.nextToken();
            }
            if (country == null && variant == null) {
                return new Locale(language);
            }
            if (variant == null) {
                return new Locale(language, country);
            }
            return new Locale(language, country, variant);
        }

        @Override // com.google.gson.TypeAdapter
        public void write(JsonWriter out, Locale value) throws IOException {
            out.value(value == null ? null : value.toString());
        }
    };
    public static final TypeAdapterFactory LOCALE_FACTORY = newFactory(Locale.class, LOCALE);
    public static final TypeAdapter<JsonElement> JSON_ELEMENT = new TypeAdapter<JsonElement>() { // from class: com.google.gson.internal.bind.TypeAdapters.29
        /* JADX WARN: Can't rename method to resolve collision */
        @Override // com.google.gson.TypeAdapter
        /* JADX INFO: renamed from: read */
        public JsonElement read2(JsonReader in) throws IOException {
            switch (AnonymousClass36.$SwitchMap$com$google$gson$stream$JsonToken[in.peek().ordinal()]) {
                case 1:
                    String number = in.nextString();
                    return new JsonPrimitive((Number) new LazilyParsedNumber(number));
                case 2:
                    return new JsonPrimitive(Boolean.valueOf(in.nextBoolean()));
                case 3:
                    return new JsonPrimitive(in.nextString());
                case 4:
                    in.nextNull();
                    return JsonNull.INSTANCE;
                case 5:
                    JsonArray array = new JsonArray();
                    in.beginArray();
                    while (in.hasNext()) {
                        array.add(read2(in));
                    }
                    in.endArray();
                    return array;
                case 6:
                    JsonObject object = new JsonObject();
                    in.beginObject();
                    while (in.hasNext()) {
                        object.add(in.nextName(), read2(in));
                    }
                    in.endObject();
                    return object;
                default:
                    throw new IllegalArgumentException();
            }
        }

        @Override // com.google.gson.TypeAdapter
        public void write(JsonWriter out, JsonElement value) throws IOException {
            if (value == null || value.isJsonNull()) {
                out.nullValue();
                return;
            }
            if (value.isJsonPrimitive()) {
                JsonPrimitive primitive = value.getAsJsonPrimitive();
                if (primitive.isNumber()) {
                    out.value(primitive.getAsNumber());
                    return;
                } else if (primitive.isBoolean()) {
                    out.value(primitive.getAsBoolean());
                    return;
                } else {
                    out.value(primitive.getAsString());
                    return;
                }
            }
            if (value.isJsonArray()) {
                out.beginArray();
                Iterator<JsonElement> it = value.getAsJsonArray().iterator();
                while (it.hasNext()) {
                    write(out, it.next());
                }
                out.endArray();
                return;
            }
            if (value.isJsonObject()) {
                out.beginObject();
                for (Map.Entry<String, JsonElement> e : value.getAsJsonObject().entrySet()) {
                    out.name(e.getKey());
                    write(out, e.getValue());
                }
                out.endObject();
                return;
            }
            throw new IllegalArgumentException("Couldn't write " + value.getClass());
        }
    };
    public static final TypeAdapterFactory JSON_ELEMENT_FACTORY = newTypeHierarchyFactory(JsonElement.class, JSON_ELEMENT);
    public static final TypeAdapterFactory ENUM_FACTORY = new TypeAdapterFactory() { // from class: com.google.gson.internal.bind.TypeAdapters.30
        @Override // com.google.gson.TypeAdapterFactory
        public <T> TypeAdapter<T> create(Gson gson, TypeToken<T> typeToken) {
            Class<? super T> rawType = typeToken.getRawType();
            if (!Enum.class.isAssignableFrom(rawType) || rawType == Enum.class) {
                return null;
            }
            if (!rawType.isEnum()) {
                rawType = rawType.getSuperclass();
            }
            return new EnumTypeAdapter(rawType);
        }
    };

    private TypeAdapters() {
        throw new UnsupportedOperationException();
    }

    /* JADX INFO: renamed from: com.google.gson.internal.bind.TypeAdapters$36, reason: invalid class name */
    static /* synthetic */ class AnonymousClass36 {
        static final /* synthetic */ int[] $SwitchMap$com$google$gson$stream$JsonToken = new int[JsonToken.values().length];

        static {
            try {
                $SwitchMap$com$google$gson$stream$JsonToken[JsonToken.NUMBER.ordinal()] = 1;
            } catch (NoSuchFieldError e) {
            }
            try {
                $SwitchMap$com$google$gson$stream$JsonToken[JsonToken.BOOLEAN.ordinal()] = 2;
            } catch (NoSuchFieldError e2) {
            }
            try {
                $SwitchMap$com$google$gson$stream$JsonToken[JsonToken.STRING.ordinal()] = 3;
            } catch (NoSuchFieldError e3) {
            }
            try {
                $SwitchMap$com$google$gson$stream$JsonToken[JsonToken.NULL.ordinal()] = 4;
            } catch (NoSuchFieldError e4) {
            }
            try {
                $SwitchMap$com$google$gson$stream$JsonToken[JsonToken.BEGIN_ARRAY.ordinal()] = 5;
            } catch (NoSuchFieldError e5) {
            }
            try {
                $SwitchMap$com$google$gson$stream$JsonToken[JsonToken.BEGIN_OBJECT.ordinal()] = 6;
            } catch (NoSuchFieldError e6) {
            }
            try {
                $SwitchMap$com$google$gson$stream$JsonToken[JsonToken.END_DOCUMENT.ordinal()] = 7;
            } catch (NoSuchFieldError e7) {
            }
            try {
                $SwitchMap$com$google$gson$stream$JsonToken[JsonToken.NAME.ordinal()] = 8;
            } catch (NoSuchFieldError e8) {
            }
            try {
                $SwitchMap$com$google$gson$stream$JsonToken[JsonToken.END_OBJECT.ordinal()] = 9;
            } catch (NoSuchFieldError e9) {
            }
            try {
                $SwitchMap$com$google$gson$stream$JsonToken[JsonToken.END_ARRAY.ordinal()] = 10;
            } catch (NoSuchFieldError e10) {
            }
        }
    }

    private static final class EnumTypeAdapter<T extends Enum<T>> extends TypeAdapter<T> {
        private final Map<String, T> nameToConstant = new HashMap();
        private final Map<T, String> constantToName = new HashMap();

        public EnumTypeAdapter(Class<T> classOfT) {
            try {
                for (T constant : classOfT.getEnumConstants()) {
                    String name = constant.name();
                    SerializedName annotation = (SerializedName) classOfT.getField(name).getAnnotation(SerializedName.class);
                    if (annotation != null) {
                        name = annotation.value();
                        for (String alternate : annotation.alternate()) {
                            this.nameToConstant.put(alternate, constant);
                        }
                    }
                    this.nameToConstant.put(name, constant);
                    this.constantToName.put(constant, name);
                }
            } catch (NoSuchFieldException e) {
                throw new AssertionError(e);
            }
        }

        @Override // com.google.gson.TypeAdapter
        /* JADX INFO: renamed from: read */
        public T read2(JsonReader in) throws IOException {
            if (in.peek() == JsonToken.NULL) {
                in.nextNull();
                return null;
            }
            return this.nameToConstant.get(in.nextString());
        }

        @Override // com.google.gson.TypeAdapter
        public void write(JsonWriter out, T value) throws IOException {
            out.value(value == null ? null : this.constantToName.get(value));
        }
    }

    public static <TT> TypeAdapterFactory newFactory(final TypeToken<TT> type, final TypeAdapter<TT> typeAdapter) {
        return new TypeAdapterFactory() { // from class: com.google.gson.internal.bind.TypeAdapters.31
            @Override // com.google.gson.TypeAdapterFactory
            public <T> TypeAdapter<T> create(Gson gson, TypeToken<T> typeToken) {
                if (typeToken.equals(type)) {
                    return typeAdapter;
                }
                return null;
            }
        };
    }

    public static <TT> TypeAdapterFactory newFactory(final Class<TT> type, final TypeAdapter<TT> typeAdapter) {
        return new TypeAdapterFactory() { // from class: com.google.gson.internal.bind.TypeAdapters.32
            @Override // com.google.gson.TypeAdapterFactory
            public <T> TypeAdapter<T> create(Gson gson, TypeToken<T> typeToken) {
                if (typeToken.getRawType() == type) {
                    return typeAdapter;
                }
                return null;
            }

            public String toString() {
                return "Factory[type=" + type.getName() + ",adapter=" + typeAdapter + "]";
            }
        };
    }

    public static <TT> TypeAdapterFactory newFactory(final Class<TT> unboxed, final Class<TT> boxed, final TypeAdapter<? super TT> typeAdapter) {
        return new TypeAdapterFactory() { // from class: com.google.gson.internal.bind.TypeAdapters.33
            @Override // com.google.gson.TypeAdapterFactory
            public <T> TypeAdapter<T> create(Gson gson, TypeToken<T> typeToken) {
                Class<? super T> rawType = typeToken.getRawType();
                if (rawType == unboxed || rawType == boxed) {
                    return typeAdapter;
                }
                return null;
            }

            public String toString() {
                return "Factory[type=" + boxed.getName() + Marker.ANY_NON_NULL_MARKER + unboxed.getName() + ",adapter=" + typeAdapter + "]";
            }
        };
    }

    public static <TT> TypeAdapterFactory newFactoryForMultipleTypes(final Class<TT> base, final Class<? extends TT> sub, final TypeAdapter<? super TT> typeAdapter) {
        return new TypeAdapterFactory() { // from class: com.google.gson.internal.bind.TypeAdapters.34
            @Override // com.google.gson.TypeAdapterFactory
            public <T> TypeAdapter<T> create(Gson gson, TypeToken<T> typeToken) {
                Class<? super T> rawType = typeToken.getRawType();
                if (rawType == base || rawType == sub) {
                    return typeAdapter;
                }
                return null;
            }

            public String toString() {
                return "Factory[type=" + base.getName() + Marker.ANY_NON_NULL_MARKER + sub.getName() + ",adapter=" + typeAdapter + "]";
            }
        };
    }

    public static <T1> TypeAdapterFactory newTypeHierarchyFactory(final Class<T1> clazz, final TypeAdapter<T1> typeAdapter) {
        return new TypeAdapterFactory() { // from class: com.google.gson.internal.bind.TypeAdapters.35
            @Override // com.google.gson.TypeAdapterFactory
            public <T2> TypeAdapter<T2> create(Gson gson, TypeToken<T2> typeToken) {
                final Class<? super T2> rawType = typeToken.getRawType();
                if (!clazz.isAssignableFrom(rawType)) {
                    return null;
                }
                return (TypeAdapter<T2>) new TypeAdapter<T1>() { // from class: com.google.gson.internal.bind.TypeAdapters.35.1
                    @Override // com.google.gson.TypeAdapter
                    public void write(JsonWriter out, T1 value) throws IOException {
                        typeAdapter.write(out, value);
                    }

                    @Override // com.google.gson.TypeAdapter
                    /* JADX INFO: renamed from: read */
                    public T1 read2(JsonReader jsonReader) throws IOException {
                        T1 t1 = (T1) typeAdapter.read2(jsonReader);
                        if (t1 != null && !rawType.isInstance(t1)) {
                            throw new JsonSyntaxException("Expected a " + rawType.getName() + " but was " + t1.getClass().getName());
                        }
                        return t1;
                    }
                };
            }

            public String toString() {
                return "Factory[typeHierarchy=" + clazz.getName() + ",adapter=" + typeAdapter + "]";
            }
        };
    }
}
