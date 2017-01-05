#include <iostream>
#include <fstream>
#include <vector>
#include <set>
#include <map>
#include <thread>
#include <queue>
#include <unordered_map>
#include <stack>
#include <mutex>
#include <sstream>
#include <atomic>
#include <algorithm>
#include "brandes.h"

unsigned int threads_count;
std::unordered_map<int, std::vector<int>> graph;
std::set<int> non_dead_end_vertices;
std::unordered_map<int, double> betweenness_centrality;
std::mutex vertices_queue_mutex;
std::mutex result_update_mutex;
std::queue<int> vertices_to_process;

int main(int argc, char *argv[]) {
    if (argc != 4) {
        std::cerr << "Incorrect number of arguments\n"
                  << "USAGE: ./brandes [number_of_threads] [input_file] [output_file]" << std::endl;
        return 0;
    }
    if (std::atoi(argv[1]) < 1) {
        std::cerr << "Incorrect value of number_number_of_threads. Must be greater than 0" << std::endl;
        return 0;
    }
    threads_count = (unsigned int) std::atoi(argv[1]); //we can safely cast now
    std::string input_file_path = argv[2];
    std::string output_file_path = argv[3];

    read_input(input_file_path);
    for (auto v : non_dead_end_vertices) {
        vertices_to_process.push(v);
    }

    std::vector<std::thread> threads;
    for (unsigned int i = 0; i < threads_count; i++) {
        threads.push_back(std::thread( [] { brandes_algorithm(); }));
    }

    for (auto& t : threads) {
        t.join();
    }

    write_results(output_file_path);

    return 0;
}

void brandes_algorithm() {
    std::unordered_map<int, double> betweenness_centrality_local;

    for (auto v : graph) {
        betweenness_centrality_local[v.first] = 0;
    }

    while (!vertices_to_process.empty()) {
        int current_vertex;
        {
            std::lock_guard<std::mutex> lock(vertices_queue_mutex);
            if (vertices_to_process.empty())
                break;

            current_vertex = vertices_to_process.front();
            vertices_to_process.pop();
        }
        perform_brandes_computing(current_vertex, &betweenness_centrality_local);
    }

    {
        std::lock_guard<std::mutex> lock(result_update_mutex);
        for (auto v : graph) {
            betweenness_centrality[v.first] += betweenness_centrality_local[v.first];
        }
    }
}

void perform_brandes_computing(int node_id, std::unordered_map<int, double> *betweenness_centrality_local) {
    std::stack<int>                             S;
    std::unordered_map<int, std::vector<int>>   predecessors;
    std::unordered_map<int, int>                number_of_shortest_paths;
    std::unordered_map<int, int>                distance_to;
    std::unordered_map<int, double>             dependency;

    for (auto v : graph) {
        predecessors[v.first] = std::vector<int>();
        number_of_shortest_paths[v.first] = 0;
        distance_to[v.first] = -1;
        dependency[v.first] = 0.0;
    }

    number_of_shortest_paths[node_id] = 1;
    distance_to[node_id] = 0;

    std::queue<int> Q;

    Q.push(node_id);

    while (!Q.empty()) {
        int current_node = Q.front();
        Q.pop();

        S.push(current_node);
        for (auto w : graph[current_node]) {
            if (distance_to[w] < 0) {
                Q.push(w);
                distance_to[w] = distance_to[current_node] + 1;
            }

            if (distance_to[w] == distance_to[current_node] + 1) {
                number_of_shortest_paths[w] += number_of_shortest_paths[current_node];
                predecessors[w].push_back(current_node);
            }
        }
    }

    while (!S.empty()) {
        int current_node = S.top();
        S.pop();
        for (auto v : predecessors[current_node]) {
            dependency[v] += (double(number_of_shortest_paths[v]) / number_of_shortest_paths[current_node])
                             * (1.0 + dependency[current_node]);
        }

        if (current_node != node_id) {
            (*betweenness_centrality_local)[current_node] += dependency[current_node];
        }
    }
}


void read_input(std::string file_name) {
    std::ifstream input_stream(file_name);
    int v1, v2;
    while (input_stream >> v1 >> v2) {
        if (non_dead_end_vertices.find(v1) == non_dead_end_vertices.end()) {
            non_dead_end_vertices.insert(v1);
        }

        graph[v1].push_back(v2);
        graph[v2];
    }

    input_stream.close();
}

void write_results(std::string file_name) {
    std::ofstream output_file;
    output_file.open(file_name);

    for (auto v : non_dead_end_vertices) {
        output_file << v << " " << betweenness_centrality[v] << std::endl;
    }

    output_file.close();
}