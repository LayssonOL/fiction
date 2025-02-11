//
// Created by Jan Drewniok on 11.09.23.
//

#include <catch2/catch_template_test_macros.hpp>

#include <fiction/algorithms/simulation/sidb/is_operational.hpp>
#include <fiction/layouts/cell_level_layout.hpp>
#include <fiction/traits.hpp>
#include <fiction/utils/truth_table_utils.hpp>

using namespace fiction;

TEST_CASE("SiQAD's AND gate with input BDL pairs of different size", "[is-operational]")
{
    using layout = sidb_cell_clk_lyt_siqad;

    layout lyt{};

    lyt.assign_cell_type({0, 0, 1}, sidb_technology::cell_type::INPUT);
    lyt.assign_cell_type({2, 1, 1}, sidb_technology::cell_type::INPUT);

    lyt.assign_cell_type({20, 0, 1}, sidb_technology::cell_type::INPUT);
    lyt.assign_cell_type({19, 1, 1}, sidb_technology::cell_type::INPUT);

    lyt.assign_cell_type({4, 2, 1}, sidb_technology::cell_type::NORMAL);
    lyt.assign_cell_type({6, 3, 1}, sidb_technology::cell_type::NORMAL);

    lyt.assign_cell_type({14, 3, 1}, sidb_technology::cell_type::NORMAL);
    lyt.assign_cell_type({16, 2, 1}, sidb_technology::cell_type::NORMAL);

    lyt.assign_cell_type({10, 6, 0}, sidb_technology::cell_type::OUTPUT);
    lyt.assign_cell_type({10, 7, 0}, sidb_technology::cell_type::OUTPUT);

    lyt.assign_cell_type({10, 9, 1}, sidb_technology::cell_type::NORMAL);

    CHECK(is_operational(lyt, std::vector<tt>{create_and_tt()},
                         is_operational_params{sidb_simulation_parameters{2, -0.28}})
              .first == operational_status::OPERATIONAL);
    CHECK(is_operational(lyt, std::vector<tt>{create_and_tt()},
                         is_operational_params{sidb_simulation_parameters{2, -0.1}})
              .first == operational_status::NON_OPERATIONAL);
}

TEST_CASE("Bestagon FO2 gate", "[is-operational]")
{
    using layout = sidb_cell_clk_lyt_siqad;

    layout lyt{};

    lyt.assign_cell_type({0, 0, 0}, sidb_technology::cell_type::INPUT);
    lyt.assign_cell_type({2, 1, 0}, sidb_technology::cell_type::INPUT);

    lyt.assign_cell_type({21, 11, 1}, sidb_technology::cell_type::NORMAL);
    lyt.assign_cell_type({17, 11, 0}, sidb_technology::cell_type::NORMAL);
    lyt.assign_cell_type({12, 4, 0}, sidb_technology::cell_type::NORMAL);
    lyt.assign_cell_type({18, 13, 0}, sidb_technology::cell_type::NORMAL);
    lyt.assign_cell_type({6, 2, 0}, sidb_technology::cell_type::NORMAL);
    lyt.assign_cell_type({8, 3, 0}, sidb_technology::cell_type::NORMAL);
    lyt.assign_cell_type({19, 7, 0}, sidb_technology::cell_type::NORMAL);
    lyt.assign_cell_type({14, 5, 0}, sidb_technology::cell_type::NORMAL);

    lyt.assign_cell_type({18, 6, 0}, sidb_technology::cell_type::NORMAL);
    lyt.assign_cell_type({24, 15, 0}, sidb_technology::cell_type::NORMAL);
    lyt.assign_cell_type({26, 16, 0}, sidb_technology::cell_type::NORMAL);

    lyt.assign_cell_type({12, 16, 0}, sidb_technology::cell_type::NORMAL);
    lyt.assign_cell_type({14, 15, 0}, sidb_technology::cell_type::NORMAL);

    lyt.assign_cell_type({8, 17, 0}, sidb_technology::cell_type::OUTPUT);
    lyt.assign_cell_type({6, 18, 0}, sidb_technology::cell_type::OUTPUT);

    lyt.assign_cell_type({30, 17, 0}, sidb_technology::cell_type::OUTPUT);
    lyt.assign_cell_type({32, 18, 0}, sidb_technology::cell_type::OUTPUT);

    lyt.assign_cell_type({36, 19, 0}, sidb_technology::cell_type::NORMAL);
    lyt.assign_cell_type({2, 19, 0}, sidb_technology::cell_type::NORMAL);

    SECTION("using QuickExact")
    {
        CHECK(is_operational(
                  lyt, std::vector<tt>{create_fan_out_tt()},
                  is_operational_params{sidb_simulation_parameters{2, -0.32}, sidb_simulation_engine::QUICKEXACT})
                  .first == operational_status::OPERATIONAL);
        CHECK(is_operational(
                  lyt, std::vector<tt>{create_fan_out_tt()},
                  is_operational_params{sidb_simulation_parameters{2, -0.30}, sidb_simulation_engine::QUICKEXACT})
                  .first == operational_status::NON_OPERATIONAL);
    }

    SECTION("using QuickSim")
    {
        CHECK(is_operational(
                  lyt, std::vector<tt>{create_fan_out_tt()},
                  is_operational_params{sidb_simulation_parameters{2, -0.32}, sidb_simulation_engine::QUICKSIM})
                  .first == operational_status::OPERATIONAL);
        CHECK(is_operational(
                  lyt, std::vector<tt>{create_fan_out_tt()},
                  is_operational_params{sidb_simulation_parameters{2, -0.30}, sidb_simulation_engine::QUICKSIM})
                  .first == operational_status::NON_OPERATIONAL);
    }
}

TEST_CASE("Bestagon CROSSING gate", "[is-operational]")
{
    using layout = sidb_cell_clk_lyt_siqad;

    layout lyt{};

    lyt.assign_cell_type({36, 1, 0}, sidb_technology::cell_type::INPUT);
    lyt.assign_cell_type({2, 1, 0}, sidb_technology::cell_type::INPUT);

    lyt.assign_cell_type({0, 0, 0}, sidb_technology::cell_type::INPUT);
    lyt.assign_cell_type({38, 0, 0}, sidb_technology::cell_type::INPUT);

    lyt.assign_cell_type({6, 2, 0}, sidb_technology::cell_type::NORMAL);
    lyt.assign_cell_type({20, 12, 0}, sidb_technology::cell_type::NORMAL);
    lyt.assign_cell_type({8, 3, 0}, sidb_technology::cell_type::NORMAL);
    lyt.assign_cell_type({14, 5, 0}, sidb_technology::cell_type::NORMAL);
    lyt.assign_cell_type({14, 11, 1}, sidb_technology::cell_type::NORMAL);

    lyt.assign_cell_type({12, 4, 0}, sidb_technology::cell_type::NORMAL);
    lyt.assign_cell_type({14, 15, 0}, sidb_technology::cell_type::NORMAL);
    lyt.assign_cell_type({26, 4, 0}, sidb_technology::cell_type::NORMAL);

    lyt.assign_cell_type({14, 9, 0}, sidb_technology::cell_type::NORMAL);
    lyt.assign_cell_type({24, 15, 0}, sidb_technology::cell_type::NORMAL);
    lyt.assign_cell_type({12, 16, 0}, sidb_technology::cell_type::NORMAL);

    lyt.assign_cell_type({18, 9, 0}, sidb_technology::cell_type::NORMAL);
    lyt.assign_cell_type({26, 16, 0}, sidb_technology::cell_type::NORMAL);
    lyt.assign_cell_type({24, 13, 1}, sidb_technology::cell_type::NORMAL);

    lyt.assign_cell_type({24, 5, 0}, sidb_technology::cell_type::NORMAL);
    lyt.assign_cell_type({30, 3, 0}, sidb_technology::cell_type::NORMAL);
    lyt.assign_cell_type({16, 13, 1}, sidb_technology::cell_type::NORMAL);

    lyt.assign_cell_type({32, 2, 0}, sidb_technology::cell_type::NORMAL);
    lyt.assign_cell_type({20, 8, 0}, sidb_technology::cell_type::NORMAL);

    lyt.assign_cell_type({30, 17, 0}, sidb_technology::cell_type::OUTPUT);
    lyt.assign_cell_type({6, 18, 0}, sidb_technology::cell_type::OUTPUT);

    lyt.assign_cell_type({32, 18, 0}, sidb_technology::cell_type::OUTPUT);
    lyt.assign_cell_type({8, 17, 0}, sidb_technology::cell_type::OUTPUT);

    lyt.assign_cell_type({2, 19, 0}, sidb_technology::cell_type::NORMAL);
    lyt.assign_cell_type({36, 19, 0}, sidb_technology::cell_type::NORMAL);

    CHECK(lyt.num_cells() == 29);

    CHECK(
        is_operational(lyt, create_crossing_wire_tt(),
                       is_operational_params{sidb_simulation_parameters{2, -0.32}, sidb_simulation_engine::QUICKEXACT})
            .first == operational_status::OPERATIONAL);
    CHECK(
        is_operational(lyt, create_crossing_wire_tt(),
                       is_operational_params{sidb_simulation_parameters{2, -0.30}, sidb_simulation_engine::QUICKEXACT})
            .first == operational_status::NON_OPERATIONAL);
}

TEST_CASE("Bestagon AND gate", "[is-operational]")
{
    using layout = sidb_cell_clk_lyt_siqad;

    layout lyt{};

    lyt.assign_cell_type({36, 1, 0}, sidb_technology::cell_type::INPUT);
    lyt.assign_cell_type({2, 1, 0}, sidb_technology::cell_type::INPUT);

    lyt.assign_cell_type({38, 0, 0}, sidb_technology::cell_type::INPUT);
    lyt.assign_cell_type({0, 0, 0}, sidb_technology::cell_type::INPUT);

    lyt.assign_cell_type({23, 9, 0}, sidb_technology::cell_type::NORMAL);
    lyt.assign_cell_type({18, 11, 1}, sidb_technology::cell_type::NORMAL);
    lyt.assign_cell_type({18, 9, 0}, sidb_technology::cell_type::NORMAL);
    lyt.assign_cell_type({19, 8, 0}, sidb_technology::cell_type::NORMAL);

    lyt.assign_cell_type({20, 14, 0}, sidb_technology::cell_type::NORMAL);
    lyt.assign_cell_type({19, 13, 0}, sidb_technology::cell_type::NORMAL);
    lyt.assign_cell_type({26, 16, 0}, sidb_technology::cell_type::NORMAL);
    lyt.assign_cell_type({24, 15, 0}, sidb_technology::cell_type::NORMAL);
    lyt.assign_cell_type({32, 2, 0}, sidb_technology::cell_type::NORMAL);
    lyt.assign_cell_type({30, 3, 0}, sidb_technology::cell_type::NORMAL);
    lyt.assign_cell_type({26, 4, 0}, sidb_technology::cell_type::NORMAL);
    lyt.assign_cell_type({24, 5, 0}, sidb_technology::cell_type::NORMAL);
    lyt.assign_cell_type({12, 4, 0}, sidb_technology::cell_type::NORMAL);
    lyt.assign_cell_type({14, 5, 0}, sidb_technology::cell_type::NORMAL);
    lyt.assign_cell_type({6, 2, 0}, sidb_technology::cell_type::NORMAL);
    lyt.assign_cell_type({8, 3, 0}, sidb_technology::cell_type::NORMAL);

    lyt.assign_cell_type({32, 18, 0}, sidb_technology::cell_type::OUTPUT);
    lyt.assign_cell_type({30, 17, 0}, sidb_technology::cell_type::OUTPUT);

    lyt.assign_cell_type({36, 19, 0}, sidb_technology::cell_type::NORMAL);

    CHECK(lyt.num_cells() == 23);

    CHECK(
        is_operational(lyt, std::vector<tt>{create_and_tt()},
                       is_operational_params{sidb_simulation_parameters{2, -0.32}, sidb_simulation_engine::QUICKEXACT})
            .first == operational_status::OPERATIONAL);
    CHECK(
        is_operational(lyt, std::vector<tt>{create_and_tt()},
                       is_operational_params{sidb_simulation_parameters{2, -0.30}, sidb_simulation_engine::QUICKEXACT})
            .first == operational_status::NON_OPERATIONAL);
}

TEST_CASE("Not working diagonal Wire", "[is-operational]")
{
    using layout = sidb_cell_clk_lyt_siqad;

    layout lyt{};

    lyt.assign_cell_type({0, 0, 0}, sidb_technology::cell_type::INPUT);
    lyt.assign_cell_type({2, 1, 0}, sidb_technology::cell_type::INPUT);

    lyt.assign_cell_type({6, 2, 0}, sidb_technology::cell_type::NORMAL);
    lyt.assign_cell_type({8, 3, 0}, sidb_technology::cell_type::NORMAL);
    lyt.assign_cell_type({12, 4, 0}, sidb_technology::cell_type::NORMAL);
    lyt.assign_cell_type({14, 5, 0}, sidb_technology::cell_type::NORMAL);

    // canvas SiDB
    lyt.assign_cell_type({14, 6, 0}, sidb_technology::cell_type::NORMAL);

    lyt.assign_cell_type({24, 15, 0}, sidb_technology::cell_type::NORMAL);
    lyt.assign_cell_type({26, 16, 0}, sidb_technology::cell_type::NORMAL);

    lyt.assign_cell_type({30, 17, 0}, sidb_technology::cell_type::OUTPUT);
    lyt.assign_cell_type({32, 18, 0}, sidb_technology::cell_type::OUTPUT);

    lyt.assign_cell_type({36, 19, 0}, sidb_technology::cell_type::NORMAL);

    CHECK(
        is_operational(lyt, std::vector<tt>{create_id_tt()},
                       is_operational_params{sidb_simulation_parameters{2, -0.32}, sidb_simulation_engine::QUICKEXACT})
            .first == operational_status::NON_OPERATIONAL);
}
