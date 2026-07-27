package com.sun.tools.javac.util;

/* JADX INFO: loaded from: classes.dex */
public class GraphUtils {

    public interface DependencyKind {
        String getDotStyle();
    }

    public static abstract class Node<D> {
        public final D data;

        public abstract Iterable<? extends Node<D>> getAllDependencies();

        public abstract String getDependencyName(Node<D> node, DependencyKind dependencyKind);

        public abstract DependencyKind[] getSupportedDependencyKinds();

        public Node(D data) {
            this.data = data;
        }

        public String toString() {
            return this.data.toString();
        }
    }

    public static abstract class TarjanNode<D> extends Node<D> implements Comparable<TarjanNode<D>> {
        boolean active;
        int index;
        int lowlink;

        @Override // com.sun.tools.javac.util.GraphUtils.Node
        public abstract Iterable<? extends TarjanNode<D>> getAllDependencies();

        public abstract Iterable<? extends TarjanNode<D>> getDependenciesByKind(DependencyKind dependencyKind);

        public TarjanNode(D data) {
            super(data);
            this.index = -1;
        }

        @Override // java.lang.Comparable
        public int compareTo(TarjanNode<D> o) {
            if (this.index < o.index) {
                return -1;
            }
            return this.index == o.index ? 0 : 1;
        }
    }

    public static <D, N extends TarjanNode<D>> List<? extends List<? extends N>> tarjan(Iterable<? extends N> nodes) {
        Tarjan<D, N> tarjan = new Tarjan<>();
        return tarjan.findSCC(nodes);
    }

    private static class Tarjan<D, N extends TarjanNode<D>> {
        int index;
        ListBuffer<List<N>> sccs;
        ListBuffer<N> stack;

        private Tarjan() {
            this.index = 0;
            this.sccs = new ListBuffer<>();
            this.stack = new ListBuffer<>();
        }

        /* JADX INFO: Access modifiers changed from: private */
        public List<? extends List<? extends N>> findSCC(Iterable<? extends N> nodes) {
            for (N node : nodes) {
                if (node.index == -1) {
                    findSCC(node);
                }
            }
            return this.sccs.toList();
        }

        private void findSCC(N v) {
            visitNode(v);
            for (TarjanNode<D> tn : v.getAllDependencies()) {
                if (tn.index != -1) {
                    if (this.stack.contains(tn)) {
                        v.lowlink = Math.min(v.lowlink, tn.index);
                    }
                } else {
                    findSCC(tn);
                    v.lowlink = Math.min(v.lowlink, tn.lowlink);
                }
            }
            if (v.lowlink == v.index) {
                addSCC(v);
            }
        }

        private void visitNode(N n) {
            n.index = this.index;
            n.lowlink = this.index;
            this.index++;
            this.stack.prepend(n);
            n.active = true;
        }

        private void addSCC(N v) {
            N n;
            ListBuffer<N> cycle = new ListBuffer<>();
            do {
                n = this.stack.remove();
                n.active = false;
                cycle.add(n);
            } while (n != v);
            this.sccs.add(cycle.toList());
        }
    }

    public static <D> String toDot(Iterable<? extends TarjanNode<D>> nodes, String name, String header) {
        StringBuilder buf = new StringBuilder();
        buf.append(String.format("digraph %s {\n", name));
        buf.append(String.format("label = \"%s\";\n", header));
        for (TarjanNode<D> n : nodes) {
            buf.append(String.format("%s [label = \"%s\"];\n", Integer.valueOf(n.hashCode()), n.toString()));
        }
        for (TarjanNode<D> from : nodes) {
            for (DependencyKind dk2 : from.getSupportedDependencyKinds()) {
                for (TarjanNode<D> to : from.getDependenciesByKind(dk2)) {
                    buf.append(String.format("%s -> %s [label = \" %s \" style = %s ];\n", Integer.valueOf(from.hashCode()), Integer.valueOf(to.hashCode()), from.getDependencyName(to, dk2), dk2.getDotStyle()));
                }
            }
        }
        buf.append("}\n");
        return buf.toString();
    }
}
