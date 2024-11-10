#include "cmath"
#include "random"
#include "queue"

struct Node{
    int x, y;
    int g, h;
    Node* parent;

    int f() const{
        return g + h;
    }
};

struct Compare{
    bool operator()(const Node* a, const Node* b){
        return a->f() > b->f();
    }
};

int heuristic(int x1, int y1, int x2, int y2){
    return abs(x2 - x1) + abs(y2 - y1);
}

bool validSpace(int x, int y, int n, int m){
    return (x >= 0 && x < n && y >= 0 && y < m);
}

void show(int grid[5][5]){
    for (int i = 0; i < 5; i++){
        for (int j = 0; j < 5; j++){
            printf("%d ", grid[i][j]);
        }
        printf("\n");
    }
}

std::vector<std::pair<int, int>> search(){
    int n = 5;
    int m = 5;
    int grid[5][5] = {
        {0, 0, 0, 0, 0},
        {0, 1, 1, 1, 0},
        {0, 1, 0, 0, 0},
        {0, 0, 1, 0, 1},
        {1, 1, 0, 0, 0}
    };
    int visited[5][5] ={
        {0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0},
        {0, 0, 0, 0, 0}
    };
    int start[2] = {0, 0};
    int goal[2] = {n - 1, m - 1};
    int moves[4][2] = {{-1, 0}, {1,0}, {0, -1}, {0, 1}};

    //std::random_device rd;
    std::mt19937 gen;
    std::uniform_real_distribution<> dist(0.0, 1.0);
    for (int i = 0; i < n; i++){
        for (int j = 0; j < m; j++){
            bool blocked = dist(gen) < 0.3;
            grid[i][j] = blocked;
        }
    }
    grid[n - 1][m - 1] = 0;
    grid[0][0] = 0;
    show(grid);
    

    std::priority_queue<Node*, std::vector<Node*>, Compare> pq;

    Node* startNode = new Node{start[0], start[1], 0, heuristic(start[0], start[1], goal[0], goal[1]), nullptr}; 

    pq.push(startNode);

    while (!pq.empty()){
        Node* current = pq.top();
        pq.pop();

        if (current->x == goal[0] && current->y == goal[1]){
            // Recreate path to goal
            std::vector<std::pair<int, int>> path;
            while (current){
                path.push_back({current->x, current->y});
                current = current->parent;
            }
            std::reverse(path.begin(), path.end());
            return path;
        }
        for (int i = 0; i < 4; i++){
            int new_x = current->x + moves[i][0];
            int new_y = current->y + moves[i][1];

            if (validSpace(new_x, new_y, n, m) && !grid[new_x][new_y] && !visited[new_x][new_y]){
                visited[new_x][new_y]++;
                int g = current->g + 1;
                Node* neighbor = new Node{new_x, new_y, g, heuristic(new_x, new_y, goal[0], goal[1]), current};
                pq.push(neighbor);
            }
        }
    }
    return {};
}

int main(){
    std::vector<std::pair<int, int>> path = search();
    if (path.size() == 0){
        printf("No path found\n");
    }

    for (int i = 0; i < path.size(); i++){
        printf("[%d, %d]\n", path[i].first, path[i].second);
    }

    return 0;
}