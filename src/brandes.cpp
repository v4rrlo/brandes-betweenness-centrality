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
std::set<int> non_dead_end_vertices;
std::vector<double> betweenness_centrality;
std::mutex vertices_queue_mutex;
std::mutex result_update_mutex;
std::queue<int> vertices_to_process;
std::unordered_map<int, int> remapped_vertices;
std::vector<std::vector<int>> graph;
std::vector<int> read_remapped;
int vertices_count = 0;

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

    for (int i = 0; i < graph.size(); i++) {
        betweenness_centrality.push_back(0.0);
    }

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
    //std::unordered_map<int, double> betweenness_centrality_local;
    std::vector<double> betweenness_centrality_local;

    for (int i = 0; i < graph.size(); i++) {
        betweenness_centrality_local[i] = 0;
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
        for (int i = 0; i < vertices_count; i++) {
            betweenness_centrality[i] += betweenness_centrality_local[i];
        }
    }
}

void perform_brandes_computing(int node_id, std::vector<double> *betweenness_centrality_local) {
    std::stack<int>                             S;
    std::vector<std::vector<int>> predecessors;
    std::vector<int> number_of_shortest_paths;
    std::vector<int> distance_to;
    std::vector<double> dependency;

    for (int i = 0; i < vertices_count; i++) {
        predecessors.emplace_back(std::vector<int>());
        number_of_shortest_paths.push_back(0);
        distance_to.push_back(-1);
        dependency.push_back(0.0);
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
            double result = ((double)(number_of_shortest_paths[v]) / number_of_shortest_paths[current_node])
                             * (1.0 + dependency[current_node]);
            dependency[v] += result;
        }

        if (current_node != node_id) {
            (*betweenness_centrality_local)[current_node] += dependency[current_node];
        }
    }
}


void read_input(std::string file_name) {


    std::ifstream input_stream(file_name);
    int v1, v2;

    int count = 0;
    while (input_stream >> v1 >> v2) {
        if (remapped_vertices.find(v1) == remapped_vertices.end()) {
            graph.emplace_back(std::vector<int>());
            read_remapped.push_back(v1);
            remapped_vertices[v1] = count++;
        }

        if (remapped_vertices.find(v2) == remapped_vertices.end()) {
            graph.emplace_back(std::vector<int>());
            read_remapped.push_back(v2);
            remapped_vertices[v2] = count++;
        }

        if (non_dead_end_vertices.find(remapped_vertices[v1]) == non_dead_end_vertices.end()) {
            non_dead_end_vertices.insert(remapped_vertices[v1]);
        }

        graph[remapped_vertices[v1]].push_back(remapped_vertices[v2]);
    }
    vertices_count = count;

    input_stream.close();
}

void write_results(std::string file_name) {
    std::ofstream output_file;
    output_file.open(file_name);

    std::vector<std::pair<int, double>> result_vertices;

    for (auto v : non_dead_end_vertices) {
        result_vertices.push_back(std::pair<int, double>(read_remapped[v], betweenness_centrality[v]));
    }

    sort(result_vertices.begin(), result_vertices.end());

    for (auto v : result_vertices) {
        output_file << v.first << " " << v.second << std::endl;
    }

    output_file.close();
}