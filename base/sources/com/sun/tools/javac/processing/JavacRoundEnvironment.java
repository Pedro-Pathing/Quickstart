package com.sun.tools.javac.processing;

import java.lang.annotation.Annotation;
import java.util.Collections;
import java.util.LinkedHashSet;
import java.util.List;
import java.util.Set;
import javax.annotation.processing.ProcessingEnvironment;
import javax.annotation.processing.RoundEnvironment;
import javax.lang.model.element.AnnotationMirror;
import javax.lang.model.element.Element;
import javax.lang.model.element.ElementKind;
import javax.lang.model.element.ExecutableElement;
import javax.lang.model.element.TypeElement;
import javax.lang.model.util.ElementScanner8;

/* JADX INFO: loaded from: classes.dex */
public class JavacRoundEnvironment implements RoundEnvironment {
    private static final String NOT_AN_ANNOTATION_TYPE = "The argument does not represent an annotation type: ";
    private final boolean errorRaised;
    private final ProcessingEnvironment processingEnv;
    private final boolean processingOver;
    private final Set<? extends Element> rootElements;

    JavacRoundEnvironment(boolean processingOver, boolean errorRaised, Set<? extends Element> rootElements, ProcessingEnvironment processingEnv) {
        this.processingOver = processingOver;
        this.errorRaised = errorRaised;
        this.rootElements = rootElements;
        this.processingEnv = processingEnv;
    }

    public String toString() {
        return String.format("[errorRaised=%b, rootElements=%s, processingOver=%b]", Boolean.valueOf(this.errorRaised), this.rootElements, Boolean.valueOf(this.processingOver));
    }

    @Override // javax.annotation.processing.RoundEnvironment
    public boolean processingOver() {
        return this.processingOver;
    }

    @Override // javax.annotation.processing.RoundEnvironment
    public boolean errorRaised() {
        return this.errorRaised;
    }

    @Override // javax.annotation.processing.RoundEnvironment
    public Set<? extends Element> getRootElements() {
        return this.rootElements;
    }

    @Override // javax.annotation.processing.RoundEnvironment
    public Set<? extends Element> getElementsAnnotatedWith(TypeElement a) {
        Set<Element> result = Collections.emptySet();
        if (a.getKind() != ElementKind.ANNOTATION_TYPE) {
            throw new IllegalArgumentException(NOT_AN_ANNOTATION_TYPE + a);
        }
        ElementScanner8<Set<Element>, TypeElement> scanner = new AnnotationSetScanner(result);
        for (Element element : this.rootElements) {
            result = scanner.scan(element, a);
        }
        return result;
    }

    private class AnnotationSetScanner extends ElementScanner8<Set<Element>, TypeElement> {
        Set<Element> annotatedElements;

        AnnotationSetScanner(Set<Element> defaultSet) {
            super(defaultSet);
            this.annotatedElements = new LinkedHashSet();
        }

        @Override // javax.lang.model.util.ElementScanner6, javax.lang.model.element.ElementVisitor
        public Set<Element> visitType(TypeElement e, TypeElement p) {
            scan(e.getTypeParameters(), p);
            return (Set) super.visitType(e, p);
        }

        @Override // javax.lang.model.util.ElementScanner6, javax.lang.model.element.ElementVisitor
        public Set<Element> visitExecutable(ExecutableElement e, TypeElement p) {
            scan(e.getTypeParameters(), p);
            return (Set) super.visitExecutable(e, p);
        }

        @Override // javax.lang.model.util.ElementScanner6
        public Set<Element> scan(Element e, TypeElement p) {
            List<? extends AnnotationMirror> annotationMirrors = JavacRoundEnvironment.this.processingEnv.getElementUtils().getAllAnnotationMirrors(e);
            for (AnnotationMirror annotationMirror : annotationMirrors) {
                if (p.equals(annotationMirror.getAnnotationType().asElement())) {
                    this.annotatedElements.add(e);
                }
            }
            e.accept(this, p);
            return this.annotatedElements;
        }
    }

    @Override // javax.annotation.processing.RoundEnvironment
    public Set<? extends Element> getElementsAnnotatedWith(Class<? extends Annotation> a) {
        if (!a.isAnnotation()) {
            throw new IllegalArgumentException(NOT_AN_ANNOTATION_TYPE + a);
        }
        String name = a.getCanonicalName();
        if (name == null) {
            return Collections.emptySet();
        }
        TypeElement annotationType = this.processingEnv.getElementUtils().getTypeElement(name);
        if (annotationType == null) {
            return Collections.emptySet();
        }
        return getElementsAnnotatedWith(annotationType);
    }
}
