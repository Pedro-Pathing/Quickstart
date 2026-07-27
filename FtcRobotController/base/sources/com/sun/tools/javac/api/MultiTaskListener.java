package com.sun.tools.javac.api;

import com.sun.source.util.TaskEvent;
import com.sun.source.util.TaskListener;
import com.sun.tools.javac.util.Context;
import java.util.Arrays;
import java.util.Collection;

/* JADX INFO: loaded from: classes.dex */
public class MultiTaskListener implements TaskListener {
    public static final Context.Key<MultiTaskListener> taskListenerKey = new Context.Key<>();
    ClientCodeWrapper ccw;
    TaskListener[] listeners = new TaskListener[0];

    public static MultiTaskListener instance(Context context) {
        MultiTaskListener instance = (MultiTaskListener) context.get(taskListenerKey);
        if (instance == null) {
            return new MultiTaskListener(context);
        }
        return instance;
    }

    protected MultiTaskListener(Context context) {
        context.put(taskListenerKey, this);
        this.ccw = ClientCodeWrapper.instance(context);
    }

    public Collection<TaskListener> getTaskListeners() {
        return Arrays.asList(this.listeners);
    }

    public boolean isEmpty() {
        return this.listeners.length == 0;
    }

    public void add(TaskListener listener) {
        for (TaskListener l : this.listeners) {
            if (this.ccw.unwrap(l) == listener) {
                throw new IllegalStateException();
            }
        }
        this.listeners = (TaskListener[]) Arrays.copyOf(this.listeners, this.listeners.length + 1);
        this.listeners[this.listeners.length - 1] = this.ccw.wrap(listener);
    }

    public void remove(TaskListener listener) {
        for (int i = 0; i < this.listeners.length; i++) {
            if (this.ccw.unwrap(this.listeners[i]) == listener) {
                TaskListener[] newListeners = new TaskListener[this.listeners.length - 1];
                System.arraycopy(this.listeners, 0, newListeners, 0, i);
                System.arraycopy(this.listeners, i + 1, newListeners, i, newListeners.length - i);
                this.listeners = newListeners;
                return;
            }
        }
    }

    @Override // com.sun.source.util.TaskListener
    public void started(TaskEvent e) {
        TaskListener[] ll = this.listeners;
        for (TaskListener l : ll) {
            l.started(e);
        }
    }

    @Override // com.sun.source.util.TaskListener
    public void finished(TaskEvent e) {
        TaskListener[] ll = this.listeners;
        for (TaskListener l : ll) {
            l.finished(e);
        }
    }

    public String toString() {
        return Arrays.toString(this.listeners);
    }
}
