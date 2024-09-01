#include "input.h"
#include <variant>
#include <vector>
#include <string>
#include <fstream>
#include <iostream>


void to_json(nlohmann::json& j, const GeometryEngineInput::Model& m)
{
    j = nlohmann::json{ {"id", m.id}, {"name", m.name}, {"type", m.type}, {"subType", m.subType}, {"data", m.data} };
}


void to_json(nlohmann::json& j, const GeometryEngineInput::Operation::Marginline& ml)
{
    nlohmann::json seed_json;
    std::visit([&seed_json](auto&& arg) {
        seed_json = arg;
        }, ml.seed);

    j = nlohmann::json{ {"type", ml.type}, {"seed", seed_json} };
}


void to_json(nlohmann::json& j, const GeometryEngineInput::Operation& o)
{
    j = nlohmann::json{ {"type", o.type}, {"marginline", o.marginline} };
}


void to_json(nlohmann::json& j, const GeometryEngineInput& gei)
{
    j = nlohmann::json{ {"model", gei.model}, {"operation", gei.operation} };
}


void from_json(const nlohmann::json& j, GeometryEngineInput::Model& m)
{
    j.at("id").get_to(m.id);
    j.at("name").get_to(m.name);
    j.at("type").get_to(m.type);
    j.at("subType").get_to(m.subType);
    j.at("data").get_to(m.data);
}


void from_json(const nlohmann::json& j, GeometryEngineInput::Operation::Marginline& ml)
{
    j.at("type").get_to(ml.type);
    if (j.at("seed").is_number()) {
        ml.seed = j.at("seed").get<double>();
    }
    else {
        ml.seed = j.at("seed").get<std::vector<double>>();
    }
}


void from_json(const nlohmann::json& j, GeometryEngineInput::Operation& o)
{
    j.at("type").get_to(o.type);
    j.at("marginline").get_to(o.marginline);
}


void from_json(const nlohmann::json& j, GeometryEngineInput& gei)
{
    j.at("model").get_to(gei.model);
    j.at("operation").get_to(gei.operation);
}


void test_input_json_00()
{
    try
    {
        auto testing_json = "..\\tests\\input_00.json";
        auto testing_result_json = "..\\tests\\input_00_result.json";

        // serialization testing
        std::ifstream ifs(testing_json);
        if (!ifs.is_open())
        {
            throw std::runtime_error("Can't open file.");
        }
        auto j = nlohmann::json::parse(ifs);


        // deserialization testing
        auto gei = j.get<GeometryEngineInput>();
        nlohmann::json serialized_gei = gei;
        std::ofstream ofs(testing_result_json);
        ofs << serialized_gei.dump(4);
    }
    catch (const std::exception& e)
    {
		std::cerr << e.what() << std::endl;
	}
}