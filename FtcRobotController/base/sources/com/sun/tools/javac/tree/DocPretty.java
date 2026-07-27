package com.sun.tools.javac.tree;

import com.qualcomm.ftccommon.configuration.RobotConfigFileManager;
import com.sun.source.doctree.AttributeTree;
import com.sun.source.doctree.AuthorTree;
import com.sun.source.doctree.CommentTree;
import com.sun.source.doctree.DeprecatedTree;
import com.sun.source.doctree.DocCommentTree;
import com.sun.source.doctree.DocRootTree;
import com.sun.source.doctree.DocTree;
import com.sun.source.doctree.DocTreeVisitor;
import com.sun.source.doctree.EndElementTree;
import com.sun.source.doctree.EntityTree;
import com.sun.source.doctree.ErroneousTree;
import com.sun.source.doctree.IdentifierTree;
import com.sun.source.doctree.InheritDocTree;
import com.sun.source.doctree.LinkTree;
import com.sun.source.doctree.LiteralTree;
import com.sun.source.doctree.ParamTree;
import com.sun.source.doctree.ReferenceTree;
import com.sun.source.doctree.ReturnTree;
import com.sun.source.doctree.SeeTree;
import com.sun.source.doctree.SerialDataTree;
import com.sun.source.doctree.SerialFieldTree;
import com.sun.source.doctree.SerialTree;
import com.sun.source.doctree.SinceTree;
import com.sun.source.doctree.StartElementTree;
import com.sun.source.doctree.TextTree;
import com.sun.source.doctree.ThrowsTree;
import com.sun.source.doctree.UnknownBlockTagTree;
import com.sun.source.doctree.UnknownInlineTagTree;
import com.sun.source.doctree.ValueTree;
import com.sun.source.doctree.VersionTree;
import com.sun.tools.javac.util.Convert;
import java.io.IOException;
import java.io.Writer;
import java.util.List;
import org.firstinspires.ftc.onbotjava.OnBotJavaFileSystemUtils;

/* JADX INFO: loaded from: classes.dex */
public class DocPretty implements DocTreeVisitor<Void, Void> {
    final Writer out;
    int lmargin = 0;
    final String lineSep = System.getProperty("line.separator");

    public DocPretty(Writer out) {
        this.out = out;
    }

    public void print(DocTree tree) throws IOException {
        try {
            if (tree == null) {
                print("/*missing*/");
            } else {
                tree.accept(this, null);
            }
        } catch (UncheckedIOException ex) {
            throw new IOException(ex.getMessage(), ex);
        }
    }

    protected void print(Object s) throws IOException {
        this.out.write(Convert.escapeUnicode(s.toString()));
    }

    public void print(List<? extends DocTree> list) throws IOException {
        for (DocTree t : list) {
            print(t);
        }
    }

    protected void print(List<? extends DocTree> list, String sep) throws IOException {
        if (list.isEmpty()) {
            return;
        }
        boolean first = true;
        for (DocTree t : list) {
            if (!first) {
                print(sep);
            }
            print(t);
            first = false;
        }
    }

    protected void println() throws IOException {
        this.out.write(this.lineSep);
    }

    protected void printTagName(DocTree node) throws IOException {
        this.out.write("@");
        this.out.write(node.getKind().tagName);
    }

    private static class UncheckedIOException extends Error {
        static final long serialVersionUID = -4032692679158424751L;

        UncheckedIOException(IOException e) {
            super(e.getMessage(), e);
        }
    }

    @Override // com.sun.source.doctree.DocTreeVisitor
    public Void visitAttribute(AttributeTree node, Void p) {
        String quote;
        try {
            print(node.getName());
            switch (node.getValueKind()) {
                case EMPTY:
                    quote = null;
                    break;
                case UNQUOTED:
                    quote = "";
                    break;
                case SINGLE:
                    quote = "'";
                    break;
                case DOUBLE:
                    quote = "\"";
                    break;
                default:
                    throw new AssertionError();
            }
            if (quote != null) {
                print("=" + quote);
                print(node.getValue());
                print(quote);
                return null;
            }
            return null;
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    @Override // com.sun.source.doctree.DocTreeVisitor
    public Void visitAuthor(AuthorTree node, Void p) {
        try {
            printTagName(node);
            print(" ");
            print(node.getName());
            return null;
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    @Override // com.sun.source.doctree.DocTreeVisitor
    public Void visitComment(CommentTree node, Void p) {
        try {
            print(node.getBody());
            return null;
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    @Override // com.sun.source.doctree.DocTreeVisitor
    public Void visitDeprecated(DeprecatedTree node, Void p) {
        try {
            printTagName(node);
            if (!node.getBody().isEmpty()) {
                print(" ");
                print(node.getBody());
                return null;
            }
            return null;
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    @Override // com.sun.source.doctree.DocTreeVisitor
    public Void visitDocComment(DocCommentTree node, Void p) {
        try {
            List<? extends DocTree> fs = node.getFirstSentence();
            List<? extends DocTree> b = node.getBody();
            List<? extends DocTree> t = node.getBlockTags();
            print(fs);
            if (!fs.isEmpty() && !b.isEmpty()) {
                print(" ");
            }
            print(b);
            if ((!fs.isEmpty() || !b.isEmpty()) && !t.isEmpty()) {
                print("\n");
            }
            print(t, "\n");
            return null;
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    @Override // com.sun.source.doctree.DocTreeVisitor
    public Void visitDocRoot(DocRootTree node, Void p) {
        try {
            print("{");
            printTagName(node);
            print("}");
            return null;
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    @Override // com.sun.source.doctree.DocTreeVisitor
    public Void visitEndElement(EndElementTree node, Void p) {
        try {
            print("</");
            print(node.getName());
            print(">");
            return null;
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    @Override // com.sun.source.doctree.DocTreeVisitor
    public Void visitEntity(EntityTree node, Void p) {
        try {
            print("&");
            print(node.getName());
            print(RobotConfigFileManager.FILE_LIST_COMMAND_DELIMITER);
            return null;
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    @Override // com.sun.source.doctree.DocTreeVisitor
    public Void visitErroneous(ErroneousTree node, Void p) {
        try {
            print(node.getBody());
            return null;
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    @Override // com.sun.source.doctree.DocTreeVisitor
    public Void visitIdentifier(IdentifierTree node, Void p) {
        try {
            print(node.getName());
            return null;
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    @Override // com.sun.source.doctree.DocTreeVisitor
    public Void visitInheritDoc(InheritDocTree node, Void p) {
        try {
            print("{");
            printTagName(node);
            print("}");
            return null;
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    @Override // com.sun.source.doctree.DocTreeVisitor
    public Void visitLink(LinkTree node, Void p) {
        try {
            print("{");
            printTagName(node);
            print(" ");
            print((DocTree) node.getReference());
            if (!node.getLabel().isEmpty()) {
                print(" ");
                print(node.getLabel());
            }
            print("}");
            return null;
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    @Override // com.sun.source.doctree.DocTreeVisitor
    public Void visitLiteral(LiteralTree node, Void p) {
        try {
            print("{");
            printTagName(node);
            print(" ");
            print((DocTree) node.getBody());
            print("}");
            return null;
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    @Override // com.sun.source.doctree.DocTreeVisitor
    public Void visitParam(ParamTree node, Void p) {
        try {
            printTagName(node);
            print(" ");
            if (node.isTypeParameter()) {
                print("<");
            }
            print((DocTree) node.getName());
            if (node.isTypeParameter()) {
                print(">");
            }
            if (!node.getDescription().isEmpty()) {
                print(" ");
                print(node.getDescription());
                return null;
            }
            return null;
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    @Override // com.sun.source.doctree.DocTreeVisitor
    public Void visitReference(ReferenceTree node, Void p) {
        try {
            print(node.getSignature());
            return null;
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    @Override // com.sun.source.doctree.DocTreeVisitor
    public Void visitReturn(ReturnTree node, Void p) {
        try {
            printTagName(node);
            print(" ");
            print(node.getDescription());
            return null;
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    @Override // com.sun.source.doctree.DocTreeVisitor
    public Void visitSee(SeeTree node, Void p) {
        try {
            printTagName(node);
            boolean first = true;
            boolean needSep = true;
            for (DocTree t : node.getReference()) {
                if (needSep) {
                    print(" ");
                }
                needSep = first && (t instanceof ReferenceTree);
                first = false;
                print(t);
            }
            return null;
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    @Override // com.sun.source.doctree.DocTreeVisitor
    public Void visitSerial(SerialTree node, Void p) {
        try {
            printTagName(node);
            if (!node.getDescription().isEmpty()) {
                print(" ");
                print(node.getDescription());
                return null;
            }
            return null;
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    @Override // com.sun.source.doctree.DocTreeVisitor
    public Void visitSerialData(SerialDataTree node, Void p) {
        try {
            printTagName(node);
            if (!node.getDescription().isEmpty()) {
                print(" ");
                print(node.getDescription());
                return null;
            }
            return null;
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    @Override // com.sun.source.doctree.DocTreeVisitor
    public Void visitSerialField(SerialFieldTree node, Void p) {
        try {
            printTagName(node);
            print(" ");
            print((DocTree) node.getName());
            print(" ");
            print((DocTree) node.getType());
            if (!node.getDescription().isEmpty()) {
                print(" ");
                print(node.getDescription());
                return null;
            }
            return null;
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    @Override // com.sun.source.doctree.DocTreeVisitor
    public Void visitSince(SinceTree node, Void p) {
        try {
            printTagName(node);
            print(" ");
            print(node.getBody());
            return null;
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    @Override // com.sun.source.doctree.DocTreeVisitor
    public Void visitStartElement(StartElementTree node, Void p) {
        try {
            print("<");
            print(node.getName());
            List<? extends DocTree> attrs = node.getAttributes();
            if (!attrs.isEmpty()) {
                print(" ");
                print(attrs);
                DocTree last = node.getAttributes().get(attrs.size() - 1);
                if (node.isSelfClosing() && (last instanceof AttributeTree) && ((AttributeTree) last).getValueKind() == AttributeTree.ValueKind.UNQUOTED) {
                    print(" ");
                }
            }
            if (node.isSelfClosing()) {
                print(OnBotJavaFileSystemUtils.PATH_SEPARATOR);
            }
            print(">");
            return null;
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    @Override // com.sun.source.doctree.DocTreeVisitor
    public Void visitText(TextTree node, Void p) {
        try {
            print(node.getBody());
            return null;
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    @Override // com.sun.source.doctree.DocTreeVisitor
    public Void visitThrows(ThrowsTree node, Void p) {
        try {
            printTagName(node);
            print(" ");
            print((DocTree) node.getExceptionName());
            if (!node.getDescription().isEmpty()) {
                print(" ");
                print(node.getDescription());
                return null;
            }
            return null;
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    @Override // com.sun.source.doctree.DocTreeVisitor
    public Void visitUnknownBlockTag(UnknownBlockTagTree node, Void p) {
        try {
            print("@");
            print(node.getTagName());
            print(" ");
            print(node.getContent());
            return null;
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    @Override // com.sun.source.doctree.DocTreeVisitor
    public Void visitUnknownInlineTag(UnknownInlineTagTree node, Void p) {
        try {
            print("{");
            print("@");
            print(node.getTagName());
            print(" ");
            print(node.getContent());
            print("}");
            return null;
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    @Override // com.sun.source.doctree.DocTreeVisitor
    public Void visitValue(ValueTree node, Void p) {
        try {
            print("{");
            printTagName(node);
            if (node.getReference() != null) {
                print(" ");
                print((DocTree) node.getReference());
            }
            print("}");
            return null;
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    @Override // com.sun.source.doctree.DocTreeVisitor
    public Void visitVersion(VersionTree node, Void p) {
        try {
            printTagName(node);
            print(" ");
            print(node.getBody());
            return null;
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }

    @Override // com.sun.source.doctree.DocTreeVisitor
    public Void visitOther(DocTree node, Void p) {
        try {
            print("(UNKNOWN: " + node + ")");
            println();
            return null;
        } catch (IOException e) {
            throw new UncheckedIOException(e);
        }
    }
}
