package com.google.blocks.ftcrobotcontroller.util;

import org.firstinspires.ftc.robotcore.external.Supplier;
import org.firstinspires.ftc.robotcore.external.ThrowingCallable;
import org.firstinspires.ftc.robotcore.internal.system.LockingRunner;

/* JADX INFO: loaded from: classes8.dex */
public final class ProjectsLockManager {
    private static final LockingRunner PROJECTS_LOCK = new LockingRunner();

    public static <T> T lockProjectsWhile(Supplier<T> supplier) {
        try {
            return (T) PROJECTS_LOCK.lockWhile(supplier);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            return null;
        }
    }

    public static <T, E extends Throwable> T lockProjectsWhile(ThrowingCallable<T, E> throwingCallable) throws Throwable {
        try {
            return (T) PROJECTS_LOCK.lockWhile(throwingCallable);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
            return null;
        }
    }
}
