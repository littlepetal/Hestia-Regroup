
#include <iostream>
#include <vector>
#include <fstream>
#include <string>
#include <ros/ros.h>
#include "yaml-cpp/yaml.h"
#include <ros/package.h>

class Bush {
public:
    int id;
    std::pair<float, float> position;
    bool onFire;
    int fireIntensity;

    Bush(int id, std::pair<float, float> pos, bool fire, int intensity) 
        : id(id), position(pos), onFire(fire), fireIntensity(intensity) {}
};

class Reservoir {
public:
    int id;
    std::pair<float, float> position;

    Reservoir(int id, std::pair<float, float> pos) 
        : id(id), position(pos) {}
};

class Map {
private:
    std::vector<Bush> bushes;
    std::vector<Reservoir> reservoirs;
    // std::string filename = "map_data.yaml";
    std::string filename = "src/Hestia/data/map_data.yaml";
    // std::string filename = ros::package::getPath("ui_package") + "/data/map_data.yaml";


public:
    void addBush(const Bush& bush) {
        bushes.push_back(bush);
    }

    void addReservoir(const Reservoir& res) {
        reservoirs.push_back(res);
    }

    void saveAndUpdate() {
        YAML::Node config;
        if (std::ifstream(filename)) {
            config = YAML::LoadFile(filename);
        }

        for (Bush& bush : bushes) {
            std::string bushKey = "bush" + std::to_string(bush.id);
            YAML::Node bush_position;
            bush_position.push_back(bush.position.first);
            bush_position.push_back(bush.position.second);

            if (config[bushKey]) {
                // Update only if bush has disappeared or fire intensity has increased
                if (!bush.onFire || bush.fireIntensity > config[bushKey]["fireIntensity"].as<int>()) {
                    config[bushKey]["position"] = bush_position;
                    config[bushKey]["onFire"] = bush.onFire;
                    config[bushKey]["fireIntensity"] = bush.fireIntensity;
                }
            } else {
                // Add new bush
                config[bushKey]["position"] = bush_position;
                config[bushKey]["onFire"] = bush.onFire;
                config[bushKey]["fireIntensity"] = bush.fireIntensity;
            }
        }

        for (Reservoir& res : reservoirs) {
            std::string resKey = "reservoir" + std::to_string(res.id);
            YAML::Node res_position;
            res_position.push_back(res.position.first);
            res_position.push_back(res.position.second);

            config[resKey]["position"] = res_position;
        }

        std::ofstream fout(filename);
        fout << config;
        fout.close();
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "map_test");
    Map map;

    // 添加Bush对象到Map
    map.addBush(Bush(1, {1.0, 2.0}, false, 0));
    map.addBush(Bush(2, {2.0, 3.0}, true, 5));

    // 添加Reservoir对象到Map
    map.addReservoir(Reservoir(1, {4.0, 5.0}));

    // 调用saveAndUpdate函数
    map.saveAndUpdate();

    // 打开并检查map_data.yaml文件，看看是否已经正确写入了数据
    return 0;
}