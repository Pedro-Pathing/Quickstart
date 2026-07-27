package com.google.blocks.ftcrobotcontroller.util;

import com.google.blocks.ftcrobotcontroller.hardware.HardwareType;
import java.util.Iterator;
import java.util.Map;

/* JADX INFO: loaded from: classes8.dex */
public class ToolboxUtil {
    private ToolboxUtil() {
    }

    public static String makeNumberShadow(int n) {
        return "<shadow type=\"math_number\"><field name=\"NUM\">" + n + "</field></shadow>\n";
    }

    public static String makeNumberShadow(double n) {
        return "<shadow type=\"math_number\"><field name=\"NUM\">" + n + "</field></shadow>\n";
    }

    public static String makeBooleanShadow(boolean value) {
        String b = value ? "TRUE" : "FALSE";
        return "<shadow type=\"logic_boolean\"><field name=\"BOOL\">" + b + "</field></shadow>\n";
    }

    public static String makeTextShadow(String text) {
        return "<shadow type=\"text\"><field name=\"TEXT\">" + escapeForXml(text) + "</field></shadow>\n";
    }

    public static String makeTypedEnumBlock(HardwareType hardwareType, String enumType) {
        return "<block type=\"" + hardwareType.blockTypePrefix + "_typedEnum_" + enumType + "\">\n</block>\n";
    }

    public static String makeTypedEnumBlock(HardwareType hardwareType, String enumType, String fieldName, String fieldValue) {
        return "<block type=\"" + hardwareType.blockTypePrefix + "_typedEnum_" + enumType + "\">\n<field name=\"" + fieldName + "\">" + fieldValue + "</field></block>\n";
    }

    public static String makeTypedEnumShadow(HardwareType hardwareType, String enumType) {
        return makeTypedEnumShadow(hardwareType.blockTypePrefix, enumType);
    }

    public static String makeTypedEnumShadow(HardwareType hardwareType, String enumType, String fieldName, String fieldValue) {
        return makeTypedEnumShadow(hardwareType.blockTypePrefix, enumType, fieldName, fieldValue);
    }

    public static String makeTypedEnumShadow(String blockTypePrefix, String enumType) {
        return "<shadow type=\"" + blockTypePrefix + "_typedEnum_" + enumType + "\">\n</shadow>\n";
    }

    public static String makeTypedEnumShadow(String blockTypePrefix, String enumType, String fieldName, String fieldValue) {
        return "<shadow type=\"" + blockTypePrefix + "_typedEnum_" + enumType + "\">\n<field name=\"" + fieldName + "\">" + fieldValue + "</field></shadow>\n";
    }

    public static String makeVariableGetBlock(String t) {
        return "<block type=\"variables_get\"><field name=\"VAR\">" + t + "</field></block>\n";
    }

    private static void addPropertySetter(StringBuilder xmlToolbox, HardwareType hardwareType, String identifier, String propertyName, String propertyType, String setterValue) {
        xmlToolbox.append("<block type=\"").append(hardwareType.blockTypePrefix).append("_setProperty_").append(propertyType).append("\">\n").append("<field name=\"IDENTIFIER\">").append(identifier).append("</field>\n").append("<field name=\"PROP\">").append(propertyName).append("</field>\n").append("<value name=\"VALUE\">\n").append(setterValue).append("</value>\n").append("</block>\n");
    }

    public static void addDualPropertySetters(StringBuilder xmlToolbox, HardwareType hardwareType, String propertyName, String propertyType, String identifier1, String setterValue1, String identifier2, String setterValue2) {
        xmlToolbox.append("<block type=\"").append(hardwareType.blockTypePrefix).append("_setDualProperty_").append(propertyType).append("\">\n").append("<field name=\"PROP\">").append(propertyName).append("</field>\n").append("<field name=\"IDENTIFIER1\">").append(identifier1).append("</field>\n").append("<field name=\"IDENTIFIER2\">").append(identifier2).append("</field>\n").append("<value name=\"VALUE1\">\n").append(setterValue1).append("</value>\n").append("<value name=\"VALUE2\">\n").append(setterValue2).append("</value>\n").append("</block>\n");
    }

    private static void addPropertyGetter(StringBuilder xmlToolbox, HardwareType hardwareType, String identifier, String propertyName, String propertyType) {
        xmlToolbox.append("<block type=\"").append(hardwareType.blockTypePrefix).append("_getProperty_").append(propertyType).append("\">\n").append("<field name=\"IDENTIFIER\">").append(identifier).append("</field>\n").append("<field name=\"PROP\">").append(propertyName).append("</field>\n").append("</block>\n");
    }

    public static void addProperties(StringBuilder xmlToolbox, HardwareType hardwareType, String identifier, Map<String, String> properties, Map<String, String[]> setterValues, Map<String, String> enumBlocks) {
        for (Map.Entry<String, String> property : properties.entrySet()) {
            String propertyName = property.getKey();
            String propertyType = property.getValue();
            if (setterValues != null && setterValues.containsKey(propertyName)) {
                String[] strArr = setterValues.get(propertyName);
                int i = 0;
                for (int length = strArr.length; i < length; length = length) {
                    String setterValue = strArr[i];
                    addPropertySetter(xmlToolbox, hardwareType, identifier, propertyName, propertyType, setterValue);
                    i++;
                }
            }
            addPropertyGetter(xmlToolbox, hardwareType, identifier, propertyName, propertyType);
            if (enumBlocks != null && enumBlocks.containsKey(propertyName)) {
                xmlToolbox.append("<block type=\"logic_compare\"><field name=\"OP\">EQ</field><value name=\"A\">");
                addPropertyGetter(xmlToolbox, hardwareType, identifier, propertyName, propertyType);
                xmlToolbox.append("</value><value name=\"B\">").append(enumBlocks.get(propertyName)).append("</value></block>");
            }
        }
    }

    public static void addFunctions(StringBuilder xmlToolbox, HardwareType hardwareType, String identifier, Map<String, Map<String, String>> functions) {
        addFunctions(xmlToolbox, hardwareType, identifier, functions, null, null, null);
    }

    public static void addFunctions(StringBuilder xmlToolbox, HardwareType hardwareType, String identifier, Map<String, Map<String, String>> functions, Map<String, String> functionComments, Map<String, String> variableSetters, Map<String, String> enumBlocks) {
        Iterator<Map.Entry<String, Map<String, String>>> it;
        Map<String, String> map = functionComments;
        Map<String, String> map2 = variableSetters;
        Map<String, String> map3 = enumBlocks;
        Iterator<Map.Entry<String, Map<String, String>>> it2 = functions.entrySet().iterator();
        while (it2.hasNext()) {
            Map.Entry<String, Map<String, String>> functionEntry = it2.next();
            String functionName = functionEntry.getKey();
            Map<String, String> args = functionEntry.getValue();
            String enumBlock = map3 != null ? map3.get(functionName) : null;
            String variableSetter = map2 != null ? map2.get(functionName) : null;
            if (enumBlock != null) {
                xmlToolbox.append("<block type=\"logic_compare\"><field name=\"OP\">EQ</field><value name=\"A\">");
            } else if (variableSetter != null) {
                xmlToolbox.append("<block type=\"variables_set\">").append("<field name=\"VAR\">").append(variableSetter).append("</field>").append("<value name=\"VALUE\">\n");
            }
            xmlToolbox.append("<block type=\"").append(hardwareType.blockTypePrefix).append("_").append(functionName).append("\">\n");
            String comment = map != null ? map.get(functionName) : null;
            if (comment != null && comment.startsWith("<comment") && comment.endsWith("</comment>")) {
                xmlToolbox.append(comment).append("\n");
            }
            xmlToolbox.append("<field name=\"IDENTIFIER\">").append(identifier).append("</field>\n");
            if (args == null) {
                it = it2;
            } else {
                for (Map.Entry<String, String> argEntry : args.entrySet()) {
                    String argName = argEntry.getKey();
                    String value = argEntry.getValue();
                    xmlToolbox.append("<value name=\"" + argName + "\">\n").append(value).append("</value>\n");
                    it2 = it2;
                }
                it = it2;
            }
            xmlToolbox.append("</block>\n");
            if (enumBlock != null) {
                xmlToolbox.append("</value><value name=\"B\">").append(enumBlock).append("</value></block>");
            } else if (variableSetter != null) {
                xmlToolbox.append("</value></block>");
            }
            map = functionComments;
            map2 = variableSetters;
            map3 = enumBlocks;
            it2 = it;
        }
    }

    public static String escapeForXml(String s) {
        return s.replace("&", "&amp;").replace("\n", "&#x0D;").replace("\"", "&quot;").replace("'", "&apos;").replace("<", "&lt;").replace(">", "&gt;");
    }
}
