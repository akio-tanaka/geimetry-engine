#include "output.h"
#include <fstream>
#include <iostream>


void to_json(nlohmann::json& j, const GeometryEngineOutput::Result::Marginline& ml)
{
	j = nlohmann::json{ {"points", ml.points} };
}


void to_json(nlohmann::json& j, const GeometryEngineOutput::Result& r)
{
	j = nlohmann::json{ {"type", r.type}, {"marginline", r.marginline} };
}


void to_json(nlohmann::json& j, const GeometryEngineOutput& geo)
{
	j = nlohmann::json{ {"return_code", geo.return_code}, {"result", geo.result} };
}


void from_json(const nlohmann::json& j, GeometryEngineOutput::Result::Marginline& ml)
{
	j.at("points").get_to(ml.points);
}


void from_json(const nlohmann::json& j, GeometryEngineOutput::Result& r)
{
	j.at("type").get_to(r.type);
	j.at("marginline").get_to(r.marginline);
}


void from_json(const nlohmann::json& j, GeometryEngineOutput& geo)
{
	j.at("return_code").get_to(geo.return_code);
	j.at("result").get_to(geo.result);
}


void test_output_json_00()
{
    try
    {
        auto testing_json = "..\\tests\\output_00.json";
        auto testing_result_json = "..\\tests\\output_00_result.json";

        // serialization testing
        std::ifstream ifs(testing_json);
        if (!ifs.is_open())
        {
            throw std::runtime_error("Can't open file.");
        }
        auto j = nlohmann::json::parse(ifs);


        // deserialization testing
        auto gei = j.get<GeometryEngineOutput>();
        nlohmann::json serialized_gei = gei;
        std::ofstream ofs(testing_result_json);
        ofs << serialized_gei.dump(4);
    }
    catch (const std::exception& e)
    {
        std::cerr << e.what() << std::endl;
    }
}