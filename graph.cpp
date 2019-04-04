#include<bits/stdc++.h>

using namespace std;

class Graph {
    //  The adjacency list
    vector<vector<int>> adjList;

    //  Adjacany list with weights.
    //  Format: adjListWeighted[x] = { y, w }
    //  This denotes an edge from x to y with weight w
    vector<vector<pair<int, int>>> adjListWeighted;

    //  Is the graph directed?
    bool isDirected;

    //  Is the graph weighted?
    bool isWeighted;

    //  Number of vertices
    int n;

    /*
     * Does a DFS traversal on the current graph
     *
     * @param vis:          visited array [Which vertices have been visited?]
     * @param s:            source for starting DFS
     * @param timeStamp:    starting and finish times for every node
     * @param t:            initial time
     */
    void __DFS(vector<bool> & vis, int s, vector<pair<int, int>> & timeStamp,
            int & t) {
        if(!vis[s]) {
            vis[s] = 1;
            timeStamp[s].first = t++;
        }

        for(int next : adjList[s]) {
            if(!vis[next])
                __DFS(vis, next, timeStamp, t);
        }

        timeStamp[s].second = t++;
    }

    /*
     * Does a BFS traversal on the current graph
     *
     * @param level:    stores the level from source for every vertex
     * @param vis:      visited array [Which vertices have been visited?]
     * @param s:        source for starting BFS
     */
    void __BFS(vector<int> & level, vector<bool> & vis, int s) {
        queue<pair<int, int>> q;

        if(!vis[s]) {
            q.push({ s, 0 });
            level[s] = 0;
            vis[s] = 1;
        }

        while(!q.empty()) {
            auto curr = q.front();
            q.pop();

            if(!vis[curr.first]) {
                vis[curr.first] = 1;
                level[curr.first] = curr.second;
            }

            for(int i : adjList[curr.first])
                if(!vis[i])
                    q.push({ i, curr.second + 1 });
        }
    }

    /*
     * Does a DFS traversal on the current graph, iteratively
     *
     * @param vis:          visited array [Which vertices have been visited?]
     * @param s:            source for starting DFS
     * @param timeStamp:    starting and finish times for every node
     * @param t:            initial time
     */
    void __DFSIterative(vector<bool> & vis, int s,
            vector<pair<int, int>> & timeStamp, int & t) {
        stack<pair<int, char>> st;

        if(!vis[s]) {
            st.push({ s, 0 });
            timeStamp[s].first = t++;
            vis[s] = 1;
        }

        while(!st.empty()) {
            auto curr = st.top();
            st.pop();

            if(!curr.second) {
                st.push({ curr.first, curr.second + 1 });

                if(!vis[curr.first]) {
                    vis[curr.first] = 1;
                    timeStamp[curr.first].first = t++;
                }

                for(int i : adjList[curr.first])
                    if(!vis[i])
                        st.push({ i, 0 });

            } else {
                timeStamp[curr.first].second = t++;
            }
        }
    }

    /*
     * Performs a topological sort on the graph
     *
     * @param vis:          visited array [Which vertices have been visited?]
     *                      vis[v] = 0 => not visited [white]
     *                      vis[v] = 1 => visited, DFS call not finished [grey]
     *                      vis[v] = 2 => DFS call finished [black]
     * @param s:            source for starting DFS
     * @param timeStamp:    starting and finish times for every node
     * @param t:            initial time
     * @param cycle:        was a cycle found in this graph?
     * @param ord:          stack to store reverse order of topoSort
     */
    void __topoSortDFS(vector<char> & vis, int s,
            vector<pair<int, int>> & timeStamp, int & t, bool & cycle,
            stack<int> & ord) {
        if(!vis[s]) {
            vis[s] = 1;
            timeStamp[s].first = t++;
        }

        for(int next : adjList[s]) {
            if(!vis[next])
                __topoSortDFS(vis, next, timeStamp, t, cycle, ord);
            if(vis[next] == 1)
                cycle = true;
        }

        timeStamp[s].second = t++;
        vis[s] = 2;
        ord.push(s);
    }

    /*
     * Performs a topological sort on the graph
     *
     * @param cycle:    was a cycle found in this graph?
     */
    vector<int> __topoSortBFS(bool & cycle) {
        vector<int> topo(n);
        vector<int> indeg(n + 1);
        queue<int> q;

        for(int i = 1; i < n + 1; i++)
            for(int j : adjList[i])
                indeg[j]++;

        int cnt = 0;

        for(int i = 1; i < n + 1; i++) {
            if(!indeg[i]) {
                q.push(i);
                cnt++;
            }
        }

        if(!cnt) {
            cycle = true;
            return topo;
        }

        cnt = 0;

        while(!q.empty()) {
            int curr = q.front();
            q.pop();

            topo[cnt++] = curr;

            for(int j : adjList[curr]) {
                indeg[j]--;

                if(!indeg[j])
                    q.push(j);
            }
        }

        cycle = cnt != n;

        return topo;
    }

    /*
     * Performs dfs, pushing the vectors accessible in this call.
     * Helper for calculating strongly connected components [SCC]
     *
     * @param vis:      visited array [Which vertices have been visited?]
     * @param s:        source to start DFS from
     * @param reach:    list of vertices accessible from current starting point
     */
    void __sccDFS(vector<bool> & vis,int s, vector<int> & reach) {
        if(!vis[s]) {
            vis[s] = 1;
            reach.push_back(s);
        }

        for(int j : adjList[s]) {
            if(!vis[j])
                __sccDFS(vis, j, reach);
        }
    }

    /*
     * Calculates the shortest path from source to all vertices
     * Bellman-Ford algorithm
     *
     * @param s:                source from which shortest path is calculated
     * @param negativeCycle:    was a negative cycle found in the graph?
     *
     * @return vector with shortest path from given source to other vertices
     */
    vector<int> __bellmanFord(int s, bool & negativeCycle) {
        const int INF = 1e9;
        vector<int> sp(n + 1, INF);
        sp[s] = 0;
        int delta;
        negativeCycle = false;

        for(int k = 1; k < n; k++) {
            delta = 0;

            for(int i = 1; i <= n; i++) {
                for(size_t j = 0; j < adjListWeighted[i].size(); j++) {
                    int edgeFrom = i,
                        edgeTo = adjListWeighted[i][j].first,
                        weight = adjListWeighted[i][j].second;

                    if(sp[edgeFrom] + weight < sp[edgeTo]) {
                        sp[edgeTo] = sp[edgeFrom] + weight;
                        delta++;
                    }
                }
            }

            if(!delta)
                return sp;
        }

        delta = 0;
        for(int i = 1; i <= n; i++) {
            for(size_t j = 0; j < adjListWeighted[i].size(); j++) {
                int edgeFrom = i,
                    edgeTo = adjListWeighted[i][j].first,
                    weight = adjListWeighted[i][j].second;

                if(sp[edgeFrom] + weight < sp[edgeTo])
                    delta++;
            }
        }
        if(delta)
            negativeCycle = true;

        return sp;
    }

    /*
     * Finds the cut vertices [articulation points] in the given graph
     *
     * @param t:    time
     * @param s:    source for dfs
     * @param p:    vector storing the parent
     * @param d:    vector storing discovery time
     * @param l:    vector storing min time for any node within it's reach
     * @param vis:  visited array
     * @param cv:   vector storing whether given vertex is cut vertex or not
     *
     */
    void __getCutVertices(int & t, int s, vector<int> & p, vector<int> & d,
            vector<int> & l, vector<bool> & vis, vector<bool> & cv) {
        l[s] = d[s] = t++;
        vis[s] = 1;
        int c = 0;

        for(int i : adjList[s]) {
            if(!vis[i]) {
                p[i] = s;
                __getCutVertices(t, i, p, d, l, vis, cv);
                c++;

                l[s] = min(l[s], l[i]);

                if(p[s] == -1 && c > 1)
                    cv[s] = 1;

                if(p[s] != -1 && l[i] >= d[s])
                    cv[s] = 1;
            } else if(i != p[s])
                l[s] = min(l[s], d[i]);
        }
    }

    /*
     * Finds the cut edges [bridges] in the given graph
     *
     * @param t:    time
     * @param s:    source for dfs
     * @param p:    vector storing the parent
     * @param d:    vector storing discovery time
     * @param l:    vector storing min time for any node within it's reach
     * @param vis:  visited array
     * @param ce:   vector storing cut edges
     *
     */
    void __getCutEdges(int & t, int s, vector<int> & p, vector<int> & d,
            vector<int> & l, vector<bool> & vis, vector<pair<int, int>> & ce) {
        l[s] = d[s] = t++;
        vis[s] = 1;
        int c = 0;

        for(int i : adjList[s]) {
            if(!vis[i]) {
                p[i] = s;
                __getCutEdges(t, i, p, d, l, vis, ce);
                c++;

                l[s] = min(l[s], l[i]);

                if(l[i] > d[s])
                    ce.push_back({ i, s });
            } else if(i != p[s])
                l[s] = min(l[s], d[i]);
        }
    }

public:

    /*
     * Constructor
     *
     * @param n:            number of vertices
     * @param isDirected:   is the graph directed?
     * @param isWeighted:   do the edge have weights associated with them?
     */
    Graph(int n, bool isDirected = false, bool isWeighted = false) {
        this->n = n;
        this->isDirected = isDirected;
        this->isWeighted = isWeighted;

        this->adjList.resize(n + 1);

        if(isWeighted)
            this->adjListWeighted.resize(n + 2);
    }

    /*
     * Does a DFS traversal on the graph
     *
     * @param s: The source vertex
     *
     * @return a vector containing starting and finishing timestamps
     */
    vector<pair<int, int>> DFS(int s = -1) {
        vector<pair<int, int>> timeStamp(n + 1, { INT_MAX, INT_MAX });
        vector<bool> vis(n + 1);
        int t = 0;

        if(s >= 0) {
            //  __DFS(vis, s, timeStamp, t);
            __DFSIterative(vis, s, timeStamp, t);
        } else {
            for(s = 1; s <= n; s++)
                if(!vis[s]) {
                    //  __DFS(vis, s, timeStamp, t);
                    __DFSIterative(vis, s, timeStamp, t);
                }
        }

        return timeStamp;
    }

    /*
     * Does a DFS traversal on the graph
     *
     * @param s: The source vertex
     *
     * @return a vector containing starting and finishing timestamps
     */
    vector<int> BFS(int s = -1) {
        vector<int> level(n + 1, INT_MAX);
        vector<bool> visited(n + 1);

        if(s >= 0) {
            __BFS(level, visited, s);
        } else {
            for(int i = 1; i <= n; i++)
                if(!visited[i])
                    __BFS(level, visited, i);
        }

        return level;
    }

    /*
     * Performs a topological sort on the graph [uses DFS]
     *
     * @return the topologically sorted list of vertices if graph has no cycles
     *          OR
     *         an empty vector
     */
    vector<int> topoSortDFS() {
        vector<char> vis(n + 1);
        stack<int> orderRev;
        bool cycle;
        int t = 0;
        vector<pair<int, int>> timeStamp(n + 1, { INT_MAX, INT_MAX });

        for(int s = 1; s <= n; s++)
            if(!vis[s])
                __topoSortDFS(vis, s, timeStamp, t, cycle, orderRev);

        vector<int> order(n, INT_MIN);

        if(!cycle) {
            int i = 0;
            while(!orderRev.empty()) {
                order[i++] = orderRev.top();
                orderRev.pop();
            }

            return order;
        }

        return vector<int>(0);
    }

    /*
     * Performs a topological sort on the graph [uses BFS]
     *
     * @return the topologically sorted list of vertices if graph has no cycles
     *          OR
     *         an empty vector
     */
    vector<int> topoSortBFS() {
        bool cycle = false;
        auto vec = __topoSortBFS(cycle);

        if(!cycle)
            return vec;

        return vector<int>(0);
    }

    /*
     * Detects if there is a cycle in the graph
     *
     * @return true if cycle found else false
     * TODO: Improve this
     */
    bool hasCycle() {
        bool cycle = false;
        __topoSortBFS(cycle);

        return cycle;
    }

    /*
     * Calculates the shortest path from source to all vertices
     * Bellman-Ford algorithm
     *
     * @param s:    source from which shrtest path has to be calculated
     *
     * @return vector with shortest path from given source to other vertices
     *         if no negative cycle was found
     *          OR
     *         an empty vector
     */
    vector<int> bellmanFord(int s) {
        bool cycle = false;
        auto sp = __bellmanFord(s, cycle);

        if(!cycle)
            return sp;

        return vector<int>(0);
    }

    /*
     * Calculates the transpose of the graph
     *
     * @return a new graph which is this graph's transpose
     */
    Graph transpose() {
        Graph g(n, isDirected, isWeighted);

        if(isWeighted) {
            for(int i = 1; i <= n; i++)
                for(auto j : adjListWeighted[i])
                    g.addEdge(j.first, i, j.second);
        } else {
            for(int i = 1; i <= n; i++)
                for(int j : adjList[i])
                    g.addEdge(j, i);
        }

        return g;
    }

    /*
     * Finds the strongly connected components
     *
     * @return a list of vectors containing strongly connected components
     */
    vector<vector<int>> stronglyConnected() {
        vector<vector<int>> scc;

        auto timeStamp = this->DFS();
        Graph g = this->transpose();

        vector<pair<int, pair<int, int>>> labelled(n + 1);
        for(int i = 1; i <= n; i++)
            labelled[i] = { i, timeStamp[i] };

        sort(labelled.begin(), labelled.end(),
                [](pair<int, pair<int, int>> a, pair<int, pair<int, int>> b) {
                    return a.second.second > b.second.second;
                });

        vector<bool> vis(n + 1);

        for(auto i : labelled) {
            vector<int> reach;

            if(!vis[i.first]) {
                g.__sccDFS(vis, i.first, reach);
                scc.push_back(reach);
            }
        }

        return scc;
    }

    /*
     * Uses dijkstra's algorithm to calculate the shortest path from
     * a given source to all other vertices.
     *
     * No checks are performed[ user must ensure that edge weights are >= 0 ]
     *
     * @param s:    The source vertex
     * @return      a vector containing shortest paths wrt given source.
     */
    vector<int> dijkstra(int s) {
        const int INF = 1e9;
        vector<int> sp(n + 1, INF);
        sp[s] = 0;

        set<int> completed;
        set<pair<int, int>> processing;

        for(int i = 1; i <= n; i++)
            processing.insert({ sp[i], i });

        while(!processing.empty()) {
            auto smallest = processing.begin();

            int v = (*smallest).second;
            processing.erase(processing.begin());

            completed.insert(v);

            for(auto i : adjListWeighted[v]) {
                int u = i.first, w = i.second;

                if(sp[v] + w < sp[u]) {
                    auto update = processing.find({ sp[u], u });
                    processing.erase(update);
                    processing.insert({ sp[v] + w, u });
                    sp[u] = sp[v] + w;
                }
            }
        }

        return sp;
    }

    /*
     * Get all the cut vertices [articulation points] in the graph
     *
     * @return list containing the said vertices
     */
    vector<int> getCutVertices() {
        vector<bool> isCV(n + 1);
        int t = 0;
        vector<int> p(n + 1, -1), d(n + 1), l(n + 1);
        vector<bool> vis(n + 1);

        for(int i = 1; i <= n; i++)
            if(!vis[i])
                __getCutVertices(t, i, p, d, l, vis, isCV);

        vector<int> cv;
        for(int i = 1; i <= n; i++)
            if(isCV[i])
                cv.push_back(i);

        return cv;
    }

    /*
     * Get all the cut edges [bridges] in the graph
     *
     * @return list containing the said edges
     */
    vector<pair<int, int>> getCutEdges() {
        vector<pair<int, int>> ce;
        int t = 0;
        vector<int> p(n + 1, -1), d(n + 1), l(n + 1);
        vector<bool> vis(n + 1);

        for(int i = 1; i <= n; i++)
            if(!vis[i])
                __getCutEdges(t, i, p, d, l, vis, ce);

        return ce;
    }

    /*
     * Get the minimum spanning tree for the current graph
     * Prim's algorithm
     *
     * @return a Graph object, which is actually a tree - the MST
     */
    Graph MSTPrims() {
        if(!isWeighted)
            return *(new Graph(0));

        Graph g(n, false, true);

        set<pair<int, pair<int, int>>> processing;
        set<int> completed;

        int v = 1, edgesInserted = 0;
        completed.insert(v);

        for(auto edges : adjListWeighted[v])
            processing.insert({ edges.second, { v, edges.first } });

        while(edgesInserted != n - 1) {
            auto least = processing.begin();

            int from = (*least).second.first,
                to = (*least).second.second,
                weight = (*least).first;

            bool canInsertFrom = completed.find(from) == completed.end(),
                 canInsertTo = completed.find(to) == completed.end();

            while(!canInsertFrom && !canInsertTo) {
                processing.erase(least);

                least = processing.begin();

                from = (*least).second.first;
                to = (*least).second.second;
                weight = (*least).first;

                canInsertFrom = completed.find(from) == completed.end();
                canInsertTo = completed.find(to) == completed.end();
            }

            if(canInsertFrom) {
                for(auto edges : adjListWeighted[from])
                    processing.insert({ edges.second, { from, edges.first } });
                completed.insert(from);
            }

            if(canInsertTo) {
                for(auto edges : adjListWeighted[to])
                    processing.insert({ edges.second, { to, edges.first } });
                completed.insert(to);
            }

            processing.erase(least);

            g.addEdge(from, to, weight);
            edgesInserted++;
        }

        return g;
    }

    /*
     * Adds an edge in the graph [not weighted]
     * Checks if the graph is directed or not
     *
     * @param x: edge from vertex x
     * @param y: edge to vertex y
     */
    void addEdge(int x, int y) {
        adjList[x].push_back(y);

        if(!isDirected)
            adjList[y].push_back(x);
    }

    /*
     * Adds an edge in the graph [weighted]
     * Checks if the graph is directed or not
     *
     * @param x: edge from vertex x
     * @param y: edge to vertex y
     * @param w: weight of edge from x to y
     */
    void addEdge(int x, int y, int w) {
        adjListWeighted[x].push_back({ y, w });

        if(!isDirected)
            adjListWeighted[y].push_back({ x, w });

        addEdge(x, y);
    }

    /*
     * Returns the adjacency list
     */
    vector<vector<int>> getEdges() {
        return adjList;
    }

    /*
     * Returns the adjacency list containing weights
     */
    vector<vector<pair<int, int>>> getEdgesWeighted() {
        return adjListWeighted;
    }
};

int main() {

    Graph g(6, false, true);

    g.addEdge(1, 2, 3);
    g.addEdge(2, 3, 5);
    g.addEdge(3, 4, 4);
    g.addEdge(4, 5, 2);
    g.addEdge(5, 1, 4);

    g.addEdge(1, 6, 7);
    g.addEdge(2, 6, 8);
    g.addEdge(3, 6, 6);
    g.addEdge(4, 6, 8);
    g.addEdge(5, 6, 5);

    auto mst = g.MSTPrims();
    auto edges = mst.getEdges();

    for(int i = 1; i <= 6; i++)
        for(auto edge : edges[i])
            cout << "Edge from " << i << " to " << edge << endl;

    return 0;
}

