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

void show(int** grid, int n, int m){
    for (int i = 0; i < n; i++){
        for (int j = 0; j < m; j++){
            printf("%d ", grid[i][j]);
        }
        printf("\n");
    }
}

std::vector<std::vector<int>> search(int** grid, int start[2], int goal[2], int n, int m){
    
    int moves[4][2] = {{-1, 0}, {1,0}, {0, -1}, {0, 1}};

    int** visited = new int*[n];
    for (int i = 0; i < n; i++){
        visited[i] = new int[m];
        for (int j = 0; j < m; j++){
            visited[i][j] = 0;
        }
    }
    
    std::priority_queue<Node*, std::vector<Node*>, Compare> pq;
    Node* startNode = new Node{start[0], start[1], 0, heuristic(start[0], start[1], goal[0], goal[1]), nullptr}; 

    pq.push(startNode);

    while (!pq.empty()){
        Node* current = pq.top();
        pq.pop();

        if (current->x == goal[0] && current->y == goal[1]){
            // Recreate path to goal
            std::vector<std::vector<int>> path;
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
    int n = 7;
    int m = 4;
    int start[2] = {0, 0};
    int goal[2] = {n - 1, m - 1};
    int** grid = new int*[n];
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dist(0.0, 1.0);
    for (int i = 0; i < n; i++){
        grid[i] = new int[m];
        for (int j = 0; j < m; j++){
            bool blocked = dist(gen) < 0.3; // 30% chance of space being blocked
            grid[i][j] = blocked;
        }
    }
    grid[start[0]][start[1]] = 0;
    grid[goal[0]][goal[1]] = 0;
    show(grid, n, m);


    std::vector<std::vector<int>> path = search(grid, start, goal, n, m);
    if (path.size() == 0){
        printf("No path found\n");
    }

    for (int i = 0; i < path.size(); i++){
        printf("[%d, %d]\n", path[i][0], path[i][1]);
    }

    return 0;
}