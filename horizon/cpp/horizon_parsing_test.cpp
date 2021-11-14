#include <yaml-cpp/yaml.h>
#include "src/ilqr.h"
#include <vector>

int main(int argc, char **argv)
{
    std::string path_to_yaml = argv[1];

    auto problem_yaml = YAML::LoadFile(path_to_yaml);

    std::vector<casadi::SX> state_vec, input_vec;
    std::map<std::string, casadi::SX> var_map;
    int N = problem_yaml["n_nodes"].as<int>();

    std::cout << "n_nodes = " << N << "\n";

    // retrieve state
    for(auto item : problem_yaml["state"])
    {
        auto name = item.first.as<std::string>();
        auto var_data = item.second;

        int size = var_data["size"].as<int>();
        auto lb = var_data["lb"].as<std::vector<double>>();
        auto ub = var_data["ub"].as<std::vector<double>>();

        auto lb_eig = Eigen::MatrixXd::Map(lb.data(), size, lb.size()/size);
        auto ub_eig = Eigen::MatrixXd::Map(ub.data(), size, ub.size()/size);

        if(N < 0)
        {
            N = lb_eig.cols();
        }

        std::cout << "state " << name << "[" << size << "]\n" <<
                     "=======\n";

        state_vec.push_back(casadi::SX::sym(name, size));
        var_map[name] = state_vec.back();
    }

    // retrieve input
    for(auto item : problem_yaml["input"])
    {
        auto name = item.first.as<std::string>();
        auto var_data = item.second;

        int size = var_data["size"].as<int>();
        auto lb = var_data["lb"].as<std::vector<double>>();
        auto ub = var_data["ub"].as<std::vector<double>>();

        auto lb_eig = Eigen::MatrixXd::Map(lb.data(), size, lb.size()/size);
        auto ub_eig = Eigen::MatrixXd::Map(ub.data(), size, ub.size()/size);

        std::cout << "input " << name << "[" << size << "]\n" <<
                     "=======\n";

        input_vec.push_back(casadi::SX::sym(name, size));
        var_map[name] = input_vec.back();
    }

    // dynamics
    auto dyn = casadi::Function::deserialize(problem_yaml["dynamics"].as<std::string>());
    auto x = casadi::SX::vertcat(state_vec);
    auto u = casadi::SX::vertcat(input_vec);
    std::cout << dyn << "\n";

    // solver options
    std::cout << problem_yaml["solver"]["opt"] << std::endl;

    horizon::IterativeLQR ilqr(dyn, N);

    // retrieve param
    for(auto item : problem_yaml["param"])
    {
        auto name = item.first.as<std::string>();
        auto var_data = item.second;

        int size = var_data["size"].as<int>();
        
        std::cout << "param " << name << "[" << size << "]\n" <<
                     "=======\n";

        var_map[name] = casadi::SX::sym(name, size);
    }

    // retrive costs
    for(auto item : problem_yaml["cost"])
    {
        auto name = item.first.as<std::string>();
        auto var_data = item.second;
        auto fun = casadi::Function::deserialize(var_data["function"].as<std::string>());
        
        std::vector<casadi::SX> fun_input;
        for(int i = 0; i < fun.n_in(); i++)
        {
            fun_input.push_back(var_map.at(fun.name_in(i)));
        }
        
        casadi::SX fn_value = fun(fun_input)[0];

        auto fun_xup = casadi::Function(fun.name(), 
            {x, u, p})

        std::cout << fn_value << std::endl;
    }

}
