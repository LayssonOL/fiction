//
// Created by Jan Drewniok on 09.06.23.
//

#include <catch2/catch_template_test_macros.hpp>

#include <fiction/algorithms/simulation/sidb/exhaustive_ground_state_simulation.hpp>
#include <fiction/io/write_location_and_ground_state.hpp>
#include <fiction/layouts/cell_level_layout.hpp>
#include <fiction/technology/cell_technologies.hpp>
#include <fiction/technology/charge_distribution_surface.hpp>

#include <algorithm>
#include <sstream>
#include <string>

using namespace fiction;

// Helper function to compare string output with expected string
bool compare_output(const std::string& output, const std::string& expected)
{
    std::string clean_output = output;
    clean_output.erase(std::remove_if(clean_output.begin(), clean_output.end(), ::isspace), clean_output.end());

    std::string clean_expected = expected;
    clean_expected.erase(std::remove_if(clean_expected.begin(), clean_expected.end(), ::isspace), clean_expected.end());

    return clean_output == clean_expected;
}

TEST_CASE("writes expected output", "[write_txt_sim_result]")
{
    sidb_cell_clk_lyt_siqad lyt{};

    SECTION("Output is written to ostream correctly, degenerated GS")
    {
        lyt.assign_cell_type({0, 0}, sidb_cell_clk_lyt_siqad::cell_type::NORMAL);
        lyt.assign_cell_type({3, 0}, sidb_cell_clk_lyt_siqad::cell_type::NORMAL);
        lyt.assign_cell_type({5, 0}, sidb_cell_clk_lyt_siqad::cell_type::NORMAL);
        lyt.assign_cell_type({8, 0}, sidb_cell_clk_lyt_siqad::cell_type::NORMAL);

        const sidb_simulation_parameters params{2, -0.32};
        const auto simulation_results = exhaustive_ground_state_simulation<sidb_cell_clk_lyt_siqad>(lyt, params);

        std::stringstream ss;
        write_location_and_ground_state(simulation_results, ss);

        const std::string expected_output = R"(x [nm];y [nm];GS_0;GS_1;
                                                0.000;0.000;-1;-1;
                                                1.152;0.000;0;-1;
                                                1.920;0.000;-1;0;
                                                3.072;0.000;-1;-1;)";

        const std::string expected_output_second = R"(x [nm];y [nm];GS_0;GS_1;
                                                0.000;0.000;-1;-1;
                                                1.152;0.000;-1;0;
                                                1.920;0.000;0;-1;
                                                3.072;0.000;-1;-1;)";

        REQUIRE((compare_output(ss.str(), expected_output) || compare_output(ss.str(), expected_output_second)));
    }

    SECTION("Output is written to ostream correctly, unique GS")
    {
        lyt.assign_cell_type({0, 0}, sidb_cell_clk_lyt_siqad::cell_type::NORMAL);
        lyt.assign_cell_type({3, 0}, sidb_cell_clk_lyt_siqad::cell_type::NORMAL);
        lyt.assign_cell_type({5, 0}, sidb_cell_clk_lyt_siqad::cell_type::NORMAL);

        const sidb_simulation_parameters params{2, -0.32};
        const auto simulation_results = exhaustive_ground_state_simulation<sidb_cell_clk_lyt_siqad>(lyt, params);

        std::stringstream ss;
        write_location_and_ground_state(simulation_results, ss);

        const std::string expected_output = R"(x [nm];y [nm];GS_0;
                                                0.000;0.000;-1;
                                                1.152;0.000;0;
                                                1.920;0.000;-1;)";

        REQUIRE(compare_output(ss.str(), expected_output));
    }
}
