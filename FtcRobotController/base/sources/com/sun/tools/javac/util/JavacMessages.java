package com.sun.tools.javac.util;

import com.sun.tools.javac.api.Messages;
import com.sun.tools.javac.util.Context;
import java.lang.ref.SoftReference;
import java.text.MessageFormat;
import java.util.HashMap;
import java.util.Locale;
import java.util.Map;
import java.util.MissingResourceException;
import java.util.ResourceBundle;

/* JADX INFO: loaded from: classes.dex */
public class JavacMessages implements Messages {
    private static ResourceBundle defaultBundle = null;
    private static final String defaultBundleName = "com.sun.tools.javac.resources.compiler";
    private static JavacMessages defaultMessages;
    public static final Context.Key<JavacMessages> messagesKey = new Context.Key<>();
    private Map<Locale, SoftReference<List<ResourceBundle>>> bundleCache;
    private List<String> bundleNames;
    private List<ResourceBundle> currentBundles;
    private Locale currentLocale;

    public static JavacMessages instance(Context context) {
        JavacMessages instance = (JavacMessages) context.get(messagesKey);
        if (instance == null) {
            return new JavacMessages(context);
        }
        return instance;
    }

    public Locale getCurrentLocale() {
        return this.currentLocale;
    }

    public void setCurrentLocale(Locale locale) {
        if (locale == null) {
            locale = Locale.getDefault();
        }
        this.currentBundles = getBundles(locale);
        this.currentLocale = locale;
    }

    public JavacMessages(Context context) {
        this(defaultBundleName, (Locale) context.get(Locale.class));
        context.put(messagesKey, this);
    }

    public JavacMessages(String bundleName) throws MissingResourceException {
        this(bundleName, null);
    }

    public JavacMessages(String bundleName, Locale locale) throws MissingResourceException {
        this.bundleNames = List.nil();
        this.bundleCache = new HashMap();
        add(bundleName);
        setCurrentLocale(locale);
    }

    public JavacMessages() throws MissingResourceException {
        this(defaultBundleName);
    }

    @Override // com.sun.tools.javac.api.Messages
    public void add(String bundleName) throws MissingResourceException {
        this.bundleNames = this.bundleNames.prepend(bundleName);
        if (!this.bundleCache.isEmpty()) {
            this.bundleCache.clear();
        }
        this.currentBundles = null;
    }

    public List<ResourceBundle> getBundles(Locale locale) {
        if (locale == this.currentLocale && this.currentBundles != null) {
            return this.currentBundles;
        }
        SoftReference<List<ResourceBundle>> bundles = this.bundleCache.get(locale);
        List<ResourceBundle> bundleList = bundles == null ? null : bundles.get();
        if (bundleList == null) {
            bundleList = List.nil();
            for (String bundleName : this.bundleNames) {
                try {
                    ResourceBundle rb = ResourceBundle.getBundle(bundleName, locale);
                    bundleList = bundleList.prepend(rb);
                } catch (MissingResourceException e) {
                    throw new InternalError("Cannot find javac resource bundle for locale " + locale);
                }
            }
            this.bundleCache.put(locale, new SoftReference<>(bundleList));
        }
        return bundleList;
    }

    public String getLocalizedString(String key, Object... args) {
        return getLocalizedString(this.currentLocale, key, args);
    }

    @Override // com.sun.tools.javac.api.Messages
    public String getLocalizedString(Locale l, String key, Object... args) {
        if (l == null) {
            l = getCurrentLocale();
        }
        return getLocalizedString(getBundles(l), key, args);
    }

    static String getDefaultLocalizedString(String key, Object... args) {
        return getLocalizedString((List<ResourceBundle>) List.of(getDefaultBundle()), key, args);
    }

    @Deprecated
    static JavacMessages getDefaultMessages() {
        if (defaultMessages == null) {
            defaultMessages = new JavacMessages(defaultBundleName);
        }
        return defaultMessages;
    }

    public static ResourceBundle getDefaultBundle() {
        try {
            if (defaultBundle == null) {
                defaultBundle = ResourceBundle.getBundle(defaultBundleName);
            }
            return defaultBundle;
        } catch (MissingResourceException e) {
            throw new Error("Fatal: Resource for compiler is missing", e);
        }
    }

    /* JADX WARN: Multi-variable type inference failed */
    private static String getLocalizedString(List<ResourceBundle> bundles, String key, Object... args) {
        String msg = null;
        for (List list = bundles; list.nonEmpty() && msg == null; list = list.tail) {
            ResourceBundle rb = (ResourceBundle) list.head;
            try {
                msg = rb.getString(key);
            } catch (MissingResourceException e) {
            }
        }
        if (msg == null) {
            msg = "compiler message file broken: key=" + key + " arguments={0}, {1}, {2}, {3}, {4}, {5}, {6}, {7}";
        }
        return MessageFormat.format(msg, args);
    }
}
