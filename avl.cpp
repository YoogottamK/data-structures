#include<bits/stdc++.h>

using namespace std;

template <class T>
class AVLTree {

    /*
     * Basic struct representing a node
     * h: height
     * c: count
     * n: the key
     * l, r: left and right children
     */
    struct Node {
        int h, c;
        T n;
        Node *l, *r;

        Node(T n) {
            this->l = this->r = NULL;
            this->h = 1;
            this->c = 1;
            this->n = n;
        }
    };

    //  root ptr
    Node * rt;

    //  function to compare two keys
    function<bool(T, T)> comp;

    /*
     * function to return the height of a node
     * handles null ptr case
     * ht(NULL) = 0
     * ht(_leaf_) = 1
     */
    int ht(Node * n) {
        if(!n)
            return 0;

        return n->h;
    }

    /*
     * Performs a left rotation
     * the node passed as a parameter gets left rotated
     * returns the new root
     */
    Node * __rotL(Node * z) {
        Node *y = z->r, *B = y->l;

        y->l = z;
        z->r = B;

        y->h = 1 + max(ht(y->l), ht(y->r));
        z->h = 1 + max(ht(z->l), ht(z->r));

        return y;
    }

    /*
     * Performs a right rotation
     * the node passed as a parameter gets right rotated
     * returns the new root
     */
    Node * __rotR(Node * z) {
        Node *y = z->l, *B = y->r;

        y->r = z;
        z->l = B;

        z->h = 1 + max(ht(z->l), ht(z->r));
        y->h = 1 + max(ht(y->l), ht(y->r));

        return y;
    }

    // TODO: add the count for multiple values feature

    /*
     * Inserts a new node in the avl tree
     * keeps it balanced
     */
    Node * __insert(Node * rt, T val) {
        if(!rt)
            return new Node(val);

        if(comp(val, rt->n))
            rt->l = __insert(rt->l, val);
        else
            rt->r = __insert(rt->r, val);

        rt->h = 1 + max(ht(rt->l), ht(rt->r));

        int bal = ht(rt->l) - ht(rt->r);

        if(bal > 1) {
            if(comp(val, rt->l->n))
                return __rotR(rt);
            else {
                rt->l = __rotL(rt->l);
                return __rotR(rt);
            }
        } else if(bal < -1) {
            if(comp(val, rt->r->n)) {
                rt->r = __rotR(rt->r);
                return __rotL(rt);
            } else {
                return __rotL(rt);
            }
        }

        return rt;
    }

    /*
     * Used for finding the node to delete in 2 chilren case
     */
    Node * __delHelper(Node * rt) {
        if(!rt || !rt->r)
            return rt;

        rt = rt->r;

        while(rt->l)
            rt = rt->l;

        return rt;
    }

    /*
     * Deletes node from the avl tree
     * keeps it balanced
     */
    Node * __del(Node * rt, T val) {
        if(!rt)
            return 0;

        if(val == rt->n) {
            Node * tmp;

            if(!rt->l && !rt->r) {
                tmp = rt;
                rt = 0;
                delete tmp;
            } else if(!rt->r) {
                tmp = rt->l;
                *rt = *tmp;
                delete tmp;
            } else if(!rt->l) {
                tmp = rt->r;
                *rt = *tmp;
                delete tmp;
            } else {
                Node * succ = __delHelper(rt);

                rt->n = succ->n;

                rt->r = __del(rt->r, rt->n);
            }
        } else if(comp(val, rt->n)) {
            rt->l = __del(rt->l, val);
        } else {
            rt->r = __del(rt->r, val);
        }

        if(!rt)
            return 0;

        rt->h = 1 + max(ht(rt->l), ht(rt->r));

        int bal = ht(rt->l) - ht(rt->r);

        if(bal > 1) {
            if(ht(rt->l->l) >= ht(rt->l->r))
                return __rotR(rt);
            else {
                rt->l = __rotL(rt->l);
                return __rotR(rt);
            }
        } else if(bal < -1) {
            if(ht(rt->r->l) <= ht(rt->r->r))
                return __rotL(rt);
            else {
                rt->r = __rotR(rt->r);
                return __rotL(rt);
            }
        }

        return rt;
    }

    /*
     * Found / not?
     */
    bool __search(Node * rt, T val) {
        if(!rt)
            return false;

        if(rt->n == val)
            return true;
        else if(comp(val, rt->n))
            return __search(rt->l, val);
        else
            return __search(rt->r, val);
    }

    /*
     * Performs an inorder walk on the tree
     */
    void __inOrder(Node * rt, vector<T> & ord) {
        if(!rt)
            return;

        if(rt->l)
            __inOrder(rt->l, ord);

        ord.push_back(rt->n);

        if(rt->r)
            __inOrder(rt->r, ord);
    }

    /*
     * Performs an pre-order walk on the tree
     */
    void __preOrder(Node * rt, vector<T> & ord) {
        if(!rt)
            return;

        ord.push_back(rt->n);

        if(rt->l)
            __preOrder(rt->l, ord);

        if(rt->r)
            __preOrder(rt->r, ord);
    }

    /*
     * Performs an post-order walk on the tree
     */
    void __postOrder(Node * rt, vector<T> & ord) {
        if(!rt)
            return;

        if(rt->l)
            __postOrder(rt->l, ord);

        if(rt->r)
            __postOrder(rt->r, ord);

        ord.push_back(rt->n);
    }

    /*
     * Frees the memory used by the tree currently
     */
    void __freeTree(Node * rt) {
        if(!rt)
            return;

        if(rt->l)
            __freeTree(rt->l);
        if(rt->r)
            __freeTree(rt->r);

        delete rt;
    }

    public:

    AVLTree(function<bool(T, T)> cmp) {
        this->comp = cmp;
        rt = 0;
    }

    void insert(T n) { rt = __insert(rt, n); }

    void del(T n) { rt = __del(rt, n); }

    vector<T> inOrder() {
        vector<T> ord;
        __inOrder(rt, ord);
        return ord;
    }

    vector<T> preOrder() {
        vector<T> ord;
        __preOrder(rt, ord);
        return ord;
    }

    vector<T> postOrder() {
        vector<T> ord;
        __postOrder(rt, ord);
        return ord;
    }

    bool search(T n) { return __search(rt, n); }

    ~AVLTree() { __freeTree(rt); }
};

int main() {

    AVLTree<int> avl([](int a, int b) { return a < b; });

    avl.insert(3);
    avl.insert(10);
    avl.insert(4);
    avl.insert(9);
    avl.insert(5);
    avl.insert(6);
    avl.insert(1);
    avl.insert(2);
    avl.insert(8);
    avl.insert(7);

    auto x = avl.inOrder();

    for(int i = 0; i < (int)x.size(); i++)
        cout << x[i] << " ";
    cout << endl;

    avl.del(8);
    avl.del(5);

    x = avl.inOrder();
    for(int i = 0; i < (int)x.size(); i++)
        cout << x[i] << " ";
    cout << endl;

    x = avl.preOrder();
    for(int i = 0; i < (int)x.size(); i++)
        cout << x[i] << " ";
    cout << endl;

    if(avl.search(6))
        cout << "6 found" << endl;
    else
        cout << "6 not found" << endl;

    return 0;
}

