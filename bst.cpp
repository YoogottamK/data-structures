#include<iostream>

using namespace std;

class BST {
    struct node {
        int n, lc, rc, ls, rs;
        node *l, *r;
    };

    node * rt;

    node * mkNode(int v) {
        node * n = new node;
        n->lc = n->rc = n->ls = n->rs = 0;
        n->l = n->r = 0;
        n->n = v;

        return n;
    }

    node * __insert(node * rt, int v) {
        if(!rt)
            return mkNode(v);

        if(v < rt->n) {
            rt->l = __insert(rt->l, v);
            rt->lc++;
            rt->ls += v;
        } else {
            rt->r = __insert(rt->r, v);
            rt->rc++;
            rt->rs += v;
        }

        return rt;
    }

    node * __delHelper(node * rt) {
        if(!rt || !rt->r)
            return 0;

        rt = rt->r;

        while(rt->l)
            rt = rt->l;

        return rt;
    }

    node * __del(node * rt, int v) {
        if(!rt)
            return 0;

        if(v < rt->n) {
            rt->l = __del(rt->l, v);
            rt->lc--;
            rt->ls -= v;
        } else if(v > rt->n) {
            rt->r = __del(rt->r, v);
            rt->rc--;
            rt->rs -= v;
        } else {
            if(!rt->l && !rt->r) {
                delete rt;
                return 0;
            }

            if(!rt->r) {
                node * lc = rt->l;
                delete rt;
                return lc;
            }

            if(!rt->l) {
                node * rc = rt->r;
                delete rt;
                return rc;
            }

            rt->rs -= rt->n;
            node * succ = __delHelper(rt);
            rt->n = succ->n;
            rt->rc--;
            rt->r = __del(rt->r, rt->n);
        }

        return rt;
    }

    bool __search(node * rt, int n) {
        if(!rt)
            return false;

        if(n < rt->n)
            return __search(rt->l, n);
        else
            return __search(rt->r, n);
    }

    void __inOrder(node * rt) {
        if(!rt)
            return;

        if(rt->l)
            __inOrder(rt->l);

        cout << rt->n << ' ';

        if(rt->r)
            __inOrder(rt->r);
    }

    void __postOrder(node * rt) {
        if(!rt)
            return;

        if(rt->l)
            __postOrder(rt->l);

        if(rt->r)
            __postOrder(rt->r);

        cout << rt->n << ' ';
    }

    void __preOrder(node * rt) {
        if(!rt)
            return;

        cout << rt->n << ' ';

        if(rt->l)
            __preOrder(rt->l);

        if(rt->r)
            __preOrder(rt->r);
    }

    void __kMinSum(node * rt, int k, int & s) {
        if(!rt)
            return;

        if(k <= rt->lc)
            __kMinSum(rt->l, k, s);
        else {
            k -= rt->lc;

            if(k == 1)
                s = s + rt->n + rt->ls;
            else {
                s = s + rt->n + rt->ls;
                __kMinSum(rt->r, k - 1, s);
            }
        }

    }

    public:
        BST() { rt = 0; }

        void insert(int n) { rt = __insert(rt, n); }

        void del(int n) { rt = __del(rt, n); }

        bool search(int n) { return __search(rt, n); }

        void inOrder() { __inOrder(rt); cout << endl; }

        void postOrder() { __postOrder(rt); cout << endl; }

        void preOrder() { __preOrder(rt); cout << endl; }

        int sumAll() {
            if(!rt)
                return 0;

            return rt->ls + rt->rs + rt->n;
        }

        int kMinSum(int k) {
            int s = 0;
            __kMinSum(rt, k, s);
            return s;
        }

};

