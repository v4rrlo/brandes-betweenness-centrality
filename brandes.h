#ifndef PW_PROJEKT_2_BRANDES_H
#define PW_PROJEKT_2_BRANDES_H

void read_input(std::string file_name);
void perform_brandes_computing(int node_id, std::unordered_map<int, double> *betweenness_centrality_local);
void brandes_algorithm();
void write_results(std::string file_name);

#endif //PW_PROJEKT_2_BRANDES_H
