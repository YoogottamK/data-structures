#include<bits/stdc++.h>

using namespace std;

class Trie {
    struct Node {
        bool $;
        map<char, int> next;

        Node() {
            $ = 0;
        }
    };

    vector<Node> trie;

    /*
     * inserts a string into trie
     *
     * @param s: the string
     * @param si: index of current character in string
     * @param l: length of string left to process
     * @param i: index for trie
     */
    void __insert(string s, int l, int si = 0, int i = 0) {
        if(!l) {
            trie[i].$ = 1;
            return;
        }

        char curr = s[si];

        if(trie[i].next.find(curr) != trie[i].next.end()) {
            // this means that the link exists
            __insert(s, l - 1, si + 1, trie[i].next[curr]);
        } else {
            // this means that the link doesn't exist
            trie.push_back(*(new Node));
            int sz = trie[i].next[curr] = trie.size() - 1;
            __insert(s, l - 1, si + 1, sz);
        }
    }

    /*
     * searchs for a string in the trie
     *
     * @param s: the string
     * @param si: index of current character in string
     * @param l: length of string left to process
     * @param i: index for trie
     *
     * @return boolean value whether found or not
     */
    bool __search(string s, int l, int si = 0, int i = 0) {
        if(!l)
            return trie[i].$;

        if(trie[i].next.find(s[si]) != trie[i].next.end()) {
            //  the link exists
            int nextIndex = trie[i].next[s[si]];
            return __search(s, l - 1, si + 1, nextIndex);
        } else
            return false;
    }

public:
    Trie() {
        //  init root of trie
        trie.resize(1);
    }

    void insert(string s) {
        int l = s.size();
        __insert(s, l);
    }

    bool search(string s) {
        int l = s.size();
        return __search(s, l);
    }
};

int main() {
    ios::sync_with_stdio(0);
    cin.tie(NULL);

    Trie t;

    t.insert("hello");
    t.insert("world");

    cout << t.search("h") << endl;
    cout << t.search("hello") << endl;
    cout << t.search("worldwefjkdh") << endl;
    cout << t.search("world") << endl;

    return 0;
}

