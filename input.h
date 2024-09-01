#pragma once
#include <string>
#include <variant>
#include <vector>
#include <nlohmann/json.hpp>


struct GeometryEngineInput
{
    struct Model
    {
        std::string id;       // Model ID
        std::string name;     // Model name
        std::string type;     // Model format type, like '.stl'
        std::string subType;  // Model sub type, like 'binary'
        std::string data;     // Model data
    };

    struct Operation
    {
        struct Marginline {
            std::string type;  // type of seed point, 'id' for vertex id, or 'coordinate' for coordinate. it must be "coordinate" now.
            std::vector<double> seed;  // seed point to generate margin line
            int num_samples; // number of samples
            double threshold_to_remove_last_point; // threshold to remove last point
        };

        std::string type;      // Operation data type, like 'marginline'
        Marginline marginline; // input data to generate initial margin line
    };

    Model model;
    Operation operation;
};


// serialize functions
void to_json(nlohmann::json& j, const GeometryEngineInput::Model& m);
void to_json(nlohmann::json& j, const GeometryEngineInput::Operation::Marginline& ml);
void to_json(nlohmann::json& j, const GeometryEngineInput::Operation& o);
void to_json(nlohmann::json& j, const GeometryEngineInput& gei);

// deserialize functions
void from_json(const nlohmann::json& j, GeometryEngineInput::Model& m);
void from_json(const nlohmann::json& j, GeometryEngineInput::Operation::Marginline& ml);
void from_json(const nlohmann::json& j, GeometryEngineInput::Operation& o);
void from_json(const nlohmann::json& j, GeometryEngineInput& gei);

// testing
void test_input_json_00();