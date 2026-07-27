package com.google.blocks.ftcrobotcontroller.runtime;

import android.webkit.JavascriptInterface;
import org.firstinspires.ftc.robotcore.external.android.AndroidTextToSpeech;

/* JADX INFO: loaded from: classes8.dex */
class AndroidTextToSpeechAccess extends Access {
    private final AndroidTextToSpeech androidTextToSpeech;

    AndroidTextToSpeechAccess(BlocksOpMode blocksOpMode, String identifier) {
        super(blocksOpMode, identifier, "AndroidTextToSpeech");
        this.androidTextToSpeech = new AndroidTextToSpeech();
    }

    @Override // com.google.blocks.ftcrobotcontroller.runtime.Access
    void close() {
        this.androidTextToSpeech.close();
    }

    @JavascriptInterface
    public void initialize() {
        try {
            startBlockExecution(BlockType.FUNCTION, ".initialize");
            this.androidTextToSpeech.initialize();
        } catch (Throwable e) {
            try {
                this.blocksOpMode.handleFatalException(e);
                throw new AssertionError("impossible", e);
            } finally {
                endBlockExecution();
            }
        }
    }

    @JavascriptInterface
    public String getStatus() {
        try {
            startBlockExecution(BlockType.GETTER, ".Status");
            return this.androidTextToSpeech.getStatus();
        } catch (Throwable e) {
            try {
                this.blocksOpMode.handleFatalException(e);
                throw new AssertionError("impossible", e);
            } finally {
                endBlockExecution();
            }
        }
    }

    @JavascriptInterface
    public String getLanguageCode() {
        try {
            startBlockExecution(BlockType.GETTER, ".LanguageCode");
            try {
                return this.androidTextToSpeech.getLanguageCode();
            } catch (IllegalStateException e) {
                reportWarning(e.getMessage());
                return "";
            }
        } finally {
        }
    }

    @JavascriptInterface
    public String getCountryCode() {
        try {
            startBlockExecution(BlockType.GETTER, ".CountryCode");
            try {
                return this.androidTextToSpeech.getCountryCode();
            } catch (IllegalStateException e) {
                reportWarning(e.getMessage());
                return "";
            }
        } finally {
        }
    }

    @JavascriptInterface
    public boolean getIsSpeaking() {
        try {
            startBlockExecution(BlockType.GETTER, ".IsSpeaking");
            try {
                return this.androidTextToSpeech.isSpeaking();
            } catch (IllegalStateException e) {
                reportWarning(e.getMessage());
                endBlockExecution();
                return false;
            }
        } finally {
        }
    }

    @JavascriptInterface
    public void setPitch(float pitch) {
        try {
            startBlockExecution(BlockType.SETTER, ".Pitch");
            try {
                this.androidTextToSpeech.setPitch(pitch);
            } catch (IllegalStateException e) {
                reportWarning(e.getMessage());
            }
        } finally {
        }
    }

    @JavascriptInterface
    public void setSpeechRate(float speechRate) {
        try {
            startBlockExecution(BlockType.SETTER, ".SpeechRate");
            try {
                this.androidTextToSpeech.setSpeechRate(speechRate);
            } catch (IllegalStateException e) {
                reportWarning(e.getMessage());
            }
        } finally {
        }
    }

    @JavascriptInterface
    public boolean isLanguageAvailable(String languageCode) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".isLanguageAvailable");
            try {
                return this.androidTextToSpeech.isLanguageAvailable(languageCode);
            } catch (IllegalStateException e) {
                reportWarning(e.getMessage());
                endBlockExecution();
                return false;
            }
        } finally {
        }
    }

    @JavascriptInterface
    public boolean isLanguageAndCountryAvailable(String languageCode, String countryCode) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".isLanguageAndCountryAvailable");
            try {
                return this.androidTextToSpeech.isLanguageAndCountryAvailable(languageCode, countryCode);
            } catch (IllegalStateException e) {
                reportWarning(e.getMessage());
                endBlockExecution();
                return false;
            }
        } finally {
        }
    }

    @JavascriptInterface
    public void setLanguage(String languageCode) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".setLanguage");
            try {
                this.androidTextToSpeech.setLanguage(languageCode);
            } catch (IllegalStateException e) {
                reportWarning(e.getMessage());
            }
        } finally {
        }
    }

    @JavascriptInterface
    public void setLanguageAndCountry(String languageCode, String countryCode) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".setLanguageAndCountry");
            try {
                this.androidTextToSpeech.setLanguageAndCountry(languageCode, countryCode);
            } catch (IllegalStateException e) {
                reportWarning(e.getMessage());
            }
        } finally {
        }
    }

    @JavascriptInterface
    public void speak(String text) {
        try {
            startBlockExecution(BlockType.FUNCTION, ".speak");
            try {
                this.androidTextToSpeech.speak(text);
            } catch (IllegalStateException e) {
                reportWarning(e.getMessage());
            }
        } finally {
        }
    }
}
