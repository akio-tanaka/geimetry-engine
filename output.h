#pragma once
#include <string>
#include <vector>
#include <nlohmann/json.hpp>


struct GeometryEngineOutput {
    struct Result {
        struct Marginline {
            std::vector<std::vector<double>> points; // 3éüå≥ç¿ïWÇÃîzóÒ
        };

        std::string type;      // Operation data type, like 'marginline'
        Marginline marginline; // output data to generate initial margin line
    };

    int return_code;    // Return code
    Result result;      // Result data
};


// serialize functions
void to_json(nlohmann::json& j, const GeometryEngineOutput::Result::Marginline& ml);
void to_json(nlohmann::json& j, const GeometryEngineOutput::Result& r);
void to_json(nlohmann::json& j, const GeometryEngineOutput& geo);

// deserialize functions
void from_json(const nlohmann::json& j, GeometryEngineOutput::Result::Marginline& ml);
void from_json(const nlohmann::json& j, GeometryEngineOutput::Result& r);
void from_json(const nlohmann::json& j, GeometryEngineOutput& geo);

// testing
void test_output_json_00();