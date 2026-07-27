package com.sun.tools.javac.util;

/* JADX INFO: loaded from: classes.dex */
public class IntHashTable {
    private static final int DEFAULT_INITIAL_SIZE = 64;
    private static final Object DELETED = new Object();
    protected int[] ints;
    protected int mask;
    protected int num_bindings;
    protected Object[] objs;

    public IntHashTable() {
        this.objs = new Object[64];
        this.ints = new int[64];
        this.mask = 63;
    }

    public IntHashTable(int capacity) {
        int log2Size = 4;
        while (capacity > (1 << log2Size)) {
            log2Size++;
        }
        int capacity2 = 1 << log2Size;
        this.objs = new Object[capacity2];
        this.ints = new int[capacity2];
        this.mask = capacity2 - 1;
    }

    public int hash(Object key) {
        return System.identityHashCode(key);
    }

    public int lookup(Object key, int hash) {
        int hash1 = (hash >>> 15) ^ hash;
        int hash2 = ((hash << 6) ^ hash) | 1;
        int deleted = -1;
        int i = this.mask & hash1;
        while (true) {
            Object node = this.objs[i];
            if (node == key) {
                return i;
            }
            if (node == null) {
                return deleted >= 0 ? deleted : i;
            }
            if (node == DELETED && deleted < 0) {
                deleted = i;
            }
            i = (i + hash2) & this.mask;
        }
    }

    public int lookup(Object key) {
        return lookup(key, hash(key));
    }

    public int getFromIndex(int index) {
        Object node = this.objs[index];
        if (node == null || node == DELETED) {
            return -1;
        }
        return this.ints[index];
    }

    public int putAtIndex(Object key, int value, int index) {
        Object old = this.objs[index];
        if (old == null || old == DELETED) {
            this.objs[index] = key;
            this.ints[index] = value;
            if (old != DELETED) {
                this.num_bindings++;
            }
            if (this.num_bindings * 3 >= this.objs.length * 2) {
                rehash();
                return -1;
            }
            return -1;
        }
        int oldValue = this.ints[index];
        this.ints[index] = value;
        return oldValue;
    }

    public int remove(Object key) {
        int index = lookup(key);
        Object old = this.objs[index];
        if (old == null || old == DELETED) {
            return -1;
        }
        this.objs[index] = DELETED;
        return this.ints[index];
    }

    protected void rehash() {
        Object[] oldObjsTable = this.objs;
        int[] oldIntsTable = this.ints;
        int oldCapacity = oldObjsTable.length;
        int newCapacity = oldCapacity << 1;
        Object[] newObjTable = new Object[newCapacity];
        int[] newIntTable = new int[newCapacity];
        int newMask = newCapacity - 1;
        this.objs = newObjTable;
        this.ints = newIntTable;
        this.mask = newMask;
        this.num_bindings = 0;
        int i = oldIntsTable.length;
        while (true) {
            i--;
            if (i >= 0) {
                Object key = oldObjsTable[i];
                if (key != null && key != DELETED) {
                    putAtIndex(key, oldIntsTable[i], lookup(key, hash(key)));
                }
            } else {
                return;
            }
        }
    }

    public void clear() {
        int i = this.objs.length;
        while (true) {
            i--;
            if (i >= 0) {
                this.objs[i] = null;
            } else {
                this.num_bindings = 0;
                return;
            }
        }
    }
}
