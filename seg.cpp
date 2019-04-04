#include<bits/stdc++.h>

using namespace std;

template <class T>
class SegTree {
    vector<T> segtree;
    vector<T> arr;

    function<T(T, T)> merge;
    T identity;

    void __build(int s, int e, int c) {
        if(s == e) {
            segtree[c] = arr[s];
            return;
        }

        int m = (s + e) >> 1, lc = (c << 1) + 1, rc = lc + 1;
        __build(s, m, lc);
        __build(m + 1, e, rc);
        segtree[c] = merge(segtree[lc], segtree[rc]);
    }

    T __query(int l, int r, int s, int e, int c) {
        if(e < l || s > r)
            return identity;

        if(s >= l && e <= r)
            return segtree[c];

        int m = (s + e) >> 1, lc = (c << 1) + 1, rc = lc +1;

        return merge(__query(l, r, s, m, lc), __query(l, r, m + 1, e, rc));
    }

    void __update(int s, int e, int i, T val, int c) {
        if(s == e) {
            arr[i] = val;
            segtree[c] = val;
            return;
        }

        int m = (s + e) >> 1, lc = (c << 1) + 1, rc = lc + 1;

        if(i <= m)
            __update(s, m, i, val, lc);
        else
            __update(m + 1, e, i, val, rc);

        segtree[c] = merge(segtree[lc], segtree[rc]);
    }

    public:

    SegTree(function<T(T, T)> merge, T identity) {
        this->merge = merge;
        this->identity = identity;
    }

    void build(vector<T> & arr) {
        int n = arr.size();
        this->arr = arr;
        segtree.resize(4 * n);

        __build(0, n - 1, 0);
    }

    T query(int l, int r) {
        return __query(l, r, 0, arr.size() - 1, 0);
    }

    void update(int i, T val) {
        __update(0, arr.size() - 1, i, val, 0);
    }
};

int main() {
    SegTree<int> seg([](int a, int b) { return a + b; }, 0);
    vector<int> v = { 1, 2, 3, 4, 5, 6, 7, 8, 9, 0 };

    seg.build(v);
    cout << seg.query(0, 9) << endl;

    seg.update(9, 10);
    cout << seg.query(0, 2) << endl;
}

