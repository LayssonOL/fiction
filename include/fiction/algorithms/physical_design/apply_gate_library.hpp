//
// Created by marcel on 28.06.21.
//

#ifndef FICTION_APPLY_GATE_LIBRARY_HPP
#define FICTION_APPLY_GATE_LIBRARY_HPP

#include "fiction/io/print_layout.hpp"
#include "fiction/technology/cell_ports.hpp"
#include "fiction/technology/inml_topolinano_library.hpp"
#include "fiction/technology/nmlib_inml_library.hpp"
#include "fiction/technology/qca_one_library.hpp"
#include "fiction/technology/sidb_bestagon_library.hpp"
#include "fiction/traits.hpp"
#include "fiction/utils/layout_utils.hpp"

#include <mockturtle/traits.hpp>

#include <celaeno/graph/search/a-star.hpp>
#include <celaeno/heuristics/euclidian.hpp>

// #include <algorithm>
#include <array>
#include <cstddef>
// #include <cstdint>
#include <cstdint>
#include <ctime>
#include <iostream>
#include <map>
#include <ostream>
// #include <iterator>
// #include <map>
// #include <type_traits>
// #include <utility>

#if (PROGRESS_BARS)
#include <mockturtle/utils/progress_bar.hpp>
#endif

// data types cannot properly be converted to bit field types
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wuseless-cast"
#pragma GCC diagnostic ignored "-Wconversion"

const uint8_t TILE_HEIGHT = 5;
const uint8_t TILE_WIDTH  = 5;

namespace fiction
{

namespace detail
{

namespace ns_heuristics = celaeno::heuristics;
namespace a_star        = celaeno::graph::search::a_star;
namespace fc            = fiction;

using float64_t              = double;
using ClockingZonesTilesPair = std::map<port_position, int>;
using ClockingZonesMap       = std::map<int, ClockingZonesTilesPair>;

template <typename T>
using Coord = std::pair<T, T>;

template <typename CellLyt, typename GateLibrary, typename GateLyt>
class apply_gate_library_impl
{
  public:
    explicit apply_gate_library_impl(const GateLyt& lyt) :
            gate_lyt{lyt},
            cell_lyt{aspect_ratio<CellLyt>{((gate_lyt.x() + 1) * GateLibrary::gate_x_size()) - 1,
                                           ((gate_lyt.y() + 1) * GateLibrary::gate_y_size()) - 1, gate_lyt.z()},
                     gate_lyt.get_clocking_scheme(), "", GateLibrary::gate_x_size(), GateLibrary::gate_y_size()}
    {}

    CellLyt run()
    {
#if (PROGRESS_BARS)
        // initialize a progress bar
        mockturtle::progress_bar bar{static_cast<uint32_t>(gate_lyt.size()), "[i] applying gate library: |{0}|"};
#endif
        // Generate tiles pairs with respectives predecessors and successors

        std::map<fc::tile<GateLyt>, std::vector<fc::tile<GateLyt>>> tile_preds_map, tile_succs_map;
        fc::generate_tiles_pairs_from_gate_level_layout(this->gate_lyt, tile_preds_map, tile_succs_map);

        gate_lyt.foreach_node(
            [&, this](const auto& n, [[maybe_unused]] auto i)
            {
                // const auto tile
                if (!gate_lyt.is_constant(n))
                {
                    const auto t = gate_lyt.get_tile(n);

                    // retrieve the top-leftmost cell in tile t
                    const auto c =
                        relative_to_absolute_cell_position<GateLibrary::gate_x_size(), GateLibrary::gate_y_size(),
                                                           GateLyt, CellLyt>(gate_lyt, t, cell<CellLyt>{0, 0});

                    const auto tp = GateLibrary::set_up_gate(gate_lyt, t);

                    auto tile = std::get<0>(tp);
                    // auto pred_tile = std::get<1>(tp);
                    std::vector<fc::tile<GateLyt>> pred_tiles = tile_preds_map[tile];

                    auto portlist     = std::get<2>(tp);
                    auto pr           = std::get<3>(tp);
                    auto gate         = pr.first;
                    auto clock_scheme = pr.second;
                    tile_gate_cell_layout_map.emplace(std::make_pair(t, std::make_pair(portlist, clock_scheme)));

                    assign_gate(c, pred_tiles, tile, pr, n);
                }
#if (PROGRESS_BARS)
                // update progress
                bar(i);
#endif
            });

        // perform post-layout optimization if necessary
        if constexpr (has_post_layout_optimization_v<GateLibrary, CellLyt>)
        {
            GateLibrary::post_layout_optimization(cell_lyt);
        }
        // if available, recover layout name
        if constexpr (has_get_layout_name_v<GateLyt> && has_set_layout_name_v<CellLyt>)
        {
            cell_lyt.set_layout_name(gate_lyt.get_layout_name());
        }

        return cell_lyt;
    }

  private:
    GateLyt gate_lyt;
    CellLyt cell_lyt;
    std::map<tile<GateLyt>, std::pair<port_list<port_position>, typename GateLibrary::fcn_clk_sch>>
                     tile_gate_cell_layout_map;
    ClockingZonesMap node_outputs_clocking_zones_map;

    // template <typename T>
    // decltype(auto) path(Coord<T> p1, Coord<T> p2,
    //                     std::array<std::array<int, TILE_WIDTH>, TILE_HEIGHT>& clock_zones_map) noexcept
    // {
    //     auto neighbors = [&clock_zones_map](auto&& pair) constexpr -> std::vector<std::pair<int64_t, int64_t>>
    //     {
    //         std::vector<std::pair<int64_t, int64_t>> possible_neighbors = {{
    //             {pair.first - 1, pair.second - 1},
    //             {pair.first - 1, pair.second},
    //             {pair.first - 1, pair.second + 1},
    //             {pair.first, pair.second + 1},
    //             {pair.first, pair.second - 1},
    //             {pair.first + 1, pair.second - 1},
    //             {pair.first + 1, pair.second},
    //             {pair.first + 1, pair.second + 1},
    //         }};
    //         // std::vector<std::pair<int64_t, int64_t>> neighbors{};
    //         //
    //         // for (auto& p : possible_neighbors)
    //         // {
    //         //     if (p.first >= 0 && p.first < 5 && p.second >= 0 && p.second < 5 &&
    //         //         clock_zones_map[p.second][p.first] >= 0 && clock_zones_map[p.second][p.first] < 4)
    //         //     {
    //         //         neighbors.push_back(p);
    //         //     }
    //         // }
    //         // std::cout << fmt::format("neighbors of {} is {}", pair, neighbors) << std::endl;
    //         return possible_neighbors;
    //     };
    //
    //     return a_star::run(
    //         std::forward<Coord<T>>(p1), std::forward<Coord<T>>(p2),
    //         [&](Coord<T> src) -> std::vector<std::pair<int64_t, int64_t>> { return neighbors(src); },
    //         [&](Coord<T> dest) -> bool
    //         {
    //             bool position_with_clk_zone =
    //                 clock_zones_map[dest.second][dest.first] >= 0 && clock_zones_map[dest.second][dest.first] < 4;
    //             bool valid_clk_zone = (dest.first >= 0 && dest.second >= 0 && dest.first < NMLIB_TILE_WIDTH &&
    //                                    dest.second < NMLIB_TILE_HEIGHT) &&
    //                                   position_with_clk_zone;
    //
    //             // if (valid_clk_zone)
    //             // {
    //             //     // std::cout << fmt::format("\t\tDEST {} - CLK ZONE {} - VALID CLK ZN: {}", dest,
    //             //     //                          clock_zones_map[dest.second][dest.first], valid_clk_zone)
    //             //     //           << std::endl;
    //             // }
    //             return !valid_clk_zone;
    //         },
    //         [&](auto&& a, auto&& b) -> float64_t
    //         {
    //             // auto m{1};
    //             auto m{ns_heuristics::manhattan::run(a, b)};
    //             // std::cout << fmt::format("\t\t COST {}-{} = {}", a, b, m) << std::endl;
    //             // auto m{ns_heuristics::euclidian::run(a, b)};
    //             // return clock_zones_map[b.first][b.second];
    //             return m;
    //         });
    // }

    decltype(auto)
    get_tile_clk_zn_sequence(const std::array<std::array<int, TILE_WIDTH>, TILE_HEIGHT>& tile_clk_zn_map) noexcept
    {
        std::vector<std::pair<int, int>> clk_zn_sequence;
        for (int i = 0; i < TILE_HEIGHT; i++)
        {
            for (int j = 0; j < TILE_WIDTH; j++)
            {
                if (tile_clk_zn_map[i][j] >= 0 && tile_clk_zn_map[i][j] < 4)
                {
                    clk_zn_sequence.push_back({j, i});
                }
            }
        }
        return clk_zn_sequence;
    }

    void assign_gate(const cell<CellLyt>& c, const std::vector<tile<GateLyt>>& pred_tiles, const tile<GateLyt>& tile,
                     const typename GateLibrary::fcn_gate_clk_sch& gclk, const mockturtle::node<GateLyt>& n)
    {
        auto start_x = c.x;
        auto start_y = c.y;
        auto layer   = c.z;
        auto g       = gclk.first;
        auto clk     = gclk.second;

        std::cout << " ################# ASSIGN GATE BEGIN #################" << std::endl;
        bool is_gate = this->gate_lyt.is_gate(n) && !this->gate_lyt.is_wire(n);

        const auto tile_portlist = tile_gate_cell_layout_map.at(tile).first;

        // if (is_gate)
        // {
        fmt::print("\n NODE: {}", n);
        fmt::print("\n IS GATE: {}", is_gate);
        fmt::print("\n PRED TILES: {}", pred_tiles);
        fmt::print("\n TILE: {}", tile);
        fmt::print("\n TILE PORTS: {}", tile_portlist);
        fmt::print("\n TILE INPS: {}", tile_portlist.inp);
        fmt::print("\n TILE OUTS: {}", tile_portlist.out);
        // }

        auto checked_clk_zone = assign_clock_zones(pred_tiles, tile, gclk, is_gate);

        ClockingZonesTilesPair updated_clock_zones;

        fmt::print(" ## UPDATED TILE OUTS: {}", tile_portlist.out);
        for (auto out : tile_portlist.out)
        {
            std::cout << " OUT: " << fmt::format("{}", out)
                      << " CLOCK ZONE: " << fmt::format("{}", checked_clk_zone[out.y][out.x]) << "\n"
                      << std::endl;
            updated_clock_zones[out] = checked_clk_zone[out.y][out.x];
        }

        std::cout << fmt::format("INSERTION OF {}", n) << std::endl;
        this->node_outputs_clocking_zones_map[n] = updated_clock_zones;
        std::cout << "SIZE OF node_outputs_clocking_zones_map is now " << this->node_outputs_clocking_zones_map.size()
                  << std::endl;
        // if (GateLibrary::is_crosswire(gclk.first))
        // {}
        // else
        // {}

        for (auto y = 0ul; y < g.size(); ++y)
        {
            for (auto x = 0ul; x < g[y].size(); ++x)
            {
                const cell<CellLyt> pos{start_x + x, start_y + y, layer};
                const auto          type{g[y][x]};
                const auto          cellclk{checked_clk_zone[y][x]};

                if (!technology<CellLyt>::is_empty_cell(type))
                {
                    cell_lyt.assign_cell_type(pos, type);
                    cell_lyt.assign_custom_clock_number(pos, cellclk);
                }

                // set IO names
                if (technology<CellLyt>::is_input_cell(type) || technology<CellLyt>::is_output_cell(type))
                {
                    cell_lyt.assign_cell_name(pos, gate_lyt.get_name(n));
                }
            }
        }
        std::cout << " ################# ASSIGN GATE END ################# \n" << std::endl;
    }

    GateLibrary::fcn_clk_sch assign_clock_zones(const std::vector<tile<GateLyt>>& pred_tiles, const tile<GateLyt>& tile,
                                                const typename GateLibrary::fcn_gate_clk_sch& gpair, bool is_gate)
    {
        fmt::print("\n\t --------------------- ASSIGN CLOCK ZONES ---------------------\n");

        auto cell = gpair.first;
        auto gclk = gpair.second;
        FMTPRINT("CELL: ", cell);
        FMTPRINT("GCLK: ", gclk);
        FMTPRINT("PRED TILES: ", fmt::format("{}", pred_tiles));
        FMTPRINT("TILE: ", tile);

        // If the predecessor tile is an empty tile
        // returns the predefined clock scheme
        if (pred_tiles.size() == 1 && gate_lyt.is_empty_tile(pred_tiles[0]))
        {
            std::cout << " -- PRED IS EMPTY TILE: " << std::endl;
            if (is_gate)
            {
                std::cout << " -- " << fmt::format("Tile {} is gate {}", tile, is_gate) << std::endl;
                auto                       gate_clk_sch  = gclk;
                const auto                 tile_portlist = tile_gate_cell_layout_map.at(tile).first;
                std::vector<port_position> tile_inp(tile_portlist.inp.begin(), tile_portlist.inp.end());
                const auto                 first_inp = tile_inp[0];
                std::cout << " -- Tile INPS: " << fmt::format("{}", tile_inp) << std::endl;
                return gclk;
            }
            return gclk;
        }
        else if (is_gate)
        {
            std::cout << "\n -- Tile is GATE:  " << fmt::format("Tile {} has gate {}", tile, is_gate) << std::endl;

            auto       gate_clk_sch  = gclk;
            const auto tile_portlist = tile_gate_cell_layout_map.at(tile).first;

            std::cout << " -- Tile INPS: " << fmt::format("{}", tile_portlist) << "\n\n" << std::endl;

            std::vector<port_position> tile_inp(tile_portlist.inp.begin(), tile_portlist.inp.end());

            const auto first_inp = tile_inp[0];

            std::cout << " -- Tile FIRST INP: " << fmt::format("{}", first_inp) << "\n\n" << std::endl;
        }

        // std::cout << "\n ## Tile: " << fmt::format("{}", tile) << std::endl;
        auto                       gate_clk_sch  = gclk;
        const auto                 tile_portlist = tile_gate_cell_layout_map.at(tile).first;
        std::vector<port_position> tile_inps(tile_portlist.inp.begin(), tile_portlist.inp.end());
        std::vector<port_position> tile_outs(tile_portlist.out.begin(), tile_portlist.out.end());
        const auto                 first_inp = tile_inps[0];
        auto                       pred_biggest_clk_number{-1};

        fmt::print("\n\t TILE INP PORTS: {}", tile_inps);
        fmt::print("\n\t TILE OUT PORTS: {}", tile_outs);

        std::vector<int> tile_inps_clk_zone(tile_inps.size(), 0);

        // Get tile inps opposite ports
        std::vector<port_position> tile_inp_feeders;
        std::vector<port_position> tile_feeders;
        std::vector<int>           tile_inp_feeders_clk_zone;

        auto get_pred_node_by_port = [this](auto& tile, auto oppositePort) -> std::pair<int, std::pair<int, int>>
        {
            int                 node = -1;
            std::pair<int, int> tl{};
            if (oppositePort.x == 0)
            {
                tl   = std::make_pair(tile.x + 1, tile.y);
                node = this->gate_lyt.get_node({tile.x + 1, tile.y});
            }
            else if (oppositePort.x == 4)
            {
                tl   = std::make_pair(tile.x - 1, tile.y);
                node = this->gate_lyt.get_node({tile.x - 1, tile.y});
            }
            else if (oppositePort.y == 0)
            {
                tl   = std::make_pair(tile.x, tile.y + 1);
                node = this->gate_lyt.get_node({tile.x, tile.y + 1});
            }
            else if (oppositePort.y == 4)
            {
                tl   = std::make_pair(tile.x, tile.y - 1);
                node = this->gate_lyt.get_node({tile.x, tile.y - 1});
            }

            return std::make_pair(node, tl);
        };

        auto get_tile_inp_feeders_clk_zone = [this, &get_pred_node_by_port, &tile_inp_feeders_clk_zone, &tile,
                                              &gclk](auto tilePort, std::vector<port_position> tile_inp_feeders)
        {
            fmt::print("\n\t ======== SEARCH CLK ZN FOR PORT: {}", tilePort);

            auto predPair  = get_pred_node_by_port(tile, tilePort);
            auto predNode  = predPair.first;
            auto predTileY = predPair.second.first;
            auto predTileX = predPair.second.second;

            fmt::print("\n\t === PRED NODE: {} - X: {} - Y: {}", predNode, predTileX, predTileY);

            if (predNode == 0 || predNode == -1)
            {
                fmt::print("\n\t === PRED NODE DOESN'T EXIST");
                fmt::print("\n\t === THIS IS AN INPUT TILE");
                tile_inp_feeders_clk_zone.push_back(gclk[4][2]);
                return;
            }

            if (this->node_outputs_clocking_zones_map.find(predNode) == this->node_outputs_clocking_zones_map.end())
            {
                fmt::print("\n\t === OUTPUTS CLOCKING ZONES DOESN'T CONTAIN PRED NODE");
                tile_inp_feeders_clk_zone.push_back(0);
                return;
            }

            // std::cout << "\t -- NODE OUTPUTS CLK ZONES contains " << predNode << std::endl;

            auto predNodeOutputsClockingZones = this->node_outputs_clocking_zones_map.at(predNode);
            // std::cout << "\t -- OUTPUTS CLOCKING ZONES: " << fmt::format("{}", predNodeOutputsClockingZones)
            //           << std::endl;

            for (auto feeder_port : tile_inp_feeders)
            {
                // std::cout << fmt::format("\t == Feeder port X: {} - Y: {}", feeder_port.x, feeder_port.y) <<
                // std::endl;
                if (predNodeOutputsClockingZones.find(feeder_port) != predNodeOutputsClockingZones.end())
                {
                    auto clk_zone = predNodeOutputsClockingZones.at(feeder_port);
                    tile_inp_feeders_clk_zone.push_back(clk_zone);
                }
            }
        };

        auto get_tile_inps_clk_zone = [this, &tile_inp_feeders_clk_zone, &tile_inps, &tile_outs, &tile, &gate_clk_sch](
                                          auto tile_inp_feeders) -> std::vector<std::pair<port_position, int>>
        {
            // print a section separator
            std::cout << "\n\t ============ GET TILE INPS CLK ZONES ===============" << std::endl;
            std::cout << "\n\t\t TILE " << fmt::format("X: {} - Y: {}", tile.x, tile.y) << std::endl;

            std::cout << "\t\t TILE INPS: " << fmt::format("{}", tile_inps) << std::endl;
            std::cout << "\t\t TILE INP FEEDERS: " << fmt::format("{}", tile_inp_feeders) << std::endl;
            std::cout << "\t\t TILE INP FEEDERS CLK ZONES: " << fmt::format("{}", tile_inp_feeders_clk_zone)
                      << std::endl;

            std::vector<std::pair<port_position, int>> new_inps_clk_zones;

            bool different_clk_zones = false;
            if (tile_inp_feeders_clk_zone.size() == 0)
            {
                for (size_t i{0}; i < tile_inps.size(); i++)
                {
                    auto ty         = tile_inps[i].y;
                    auto tx         = tile_inps[i].x;
                    auto i_clk_zone = gate_clk_sch[ty][tx];
                    // fmt::print("\n\t\t TY: {} - TX: {} - I CLK ZONE: {}", ty, tx, i_clk_zone);
                    new_inps_clk_zones.push_back(std::pair(tile_inps[i], i_clk_zone));
                }
            }
            else if (tile_inp_feeders_clk_zone.size() == 1)
            {
                auto feeder_clk_zn = tile_inp_feeders_clk_zone.at(0);
                new_inps_clk_zones.push_back(std::pair(tile_inps.at(0), feeder_clk_zn + 1));
            }
            else
            {
                for (size_t i{0}; i < tile_inp_feeders_clk_zone.size(); ++i)
                {
                    if (i > 0 && tile_inp_feeders_clk_zone.at(i) != tile_inp_feeders_clk_zone.at(i - 1))
                    {
                        different_clk_zones = true;
                    }
                }
                // TODO: I need to run an A* algorithm to mount clock zone sequences from inputs to outputs
                if (!different_clk_zones)
                {
                    // Run routine to update the tile clock zone based on the value of one of the inputs
                    // Clock zone index
                    int clk_zone_idx = tile_inp_feeders_clk_zone.at(0);
                }
            }

            std::cout << "\n\t DIFFERENT CLK ZONES: " << fmt::format("{}", different_clk_zones) << std::endl;
            std::cout << "\t ============ GET TILE INPS CLK ZONES ===============" << std::endl;
            return new_inps_clk_zones;
        };

        auto update_tile_clock_zones_sequence = [this, &gclk, &tile_inps, &cell, &tile_outs, &tile, &gate_clk_sch,
                                                 &get_tile_inps_clk_zone](auto tile_inp_feeders)
        {
            auto tile_inp_clk_zones = get_tile_inps_clk_zone(tile_inp_feeders);
            // std::cout << fmt::format("\n\t TILE INP CLK ZONES: {}", tile_inp_clk_zones) << std::endl;
            // std::cout << "\n\t ============ UPDATE TILE CLK SEQUENCE ===============" << std::endl;

            // std::cout << "\t\t TILE INPS: " << fmt::format("{}", tile_inps) << std::endl;
            // std::cout << "\t\t TILE OUTS: " << fmt::format("{}", tile_outs) << std::endl;

            std::vector<std::pair<int, int>> clk_zone_sequence_to_update;
            for (size_t i{0}; i < tile_inps.size(); ++i)
            {
                for (size_t i{0}; i < tile_outs.size(); ++i)
                {
                    Coord<int64_t> src{std::make_pair(tile_inps.at(i).x, tile_inps.at(i).y)};
                    Coord<int64_t> dst{std::make_pair(tile_outs.at(i).x, tile_outs.at(i).y)};
                    // std::cout << fmt::format("\t\t SEARCH PATH FROM {} TO {}", src, dst) << std::endl;
                    // auto clk_zone_sequence_to_update{this->path(src, dst, gate_clk_sch)};
                    clk_zone_sequence_to_update = get_tile_clk_zn_sequence(gclk);

                    if (clk_zone_sequence_to_update.size() > 0)
                    {
                        fmt::print("\n\t\t Tile: {} - Src: {} - Dst: {} - A*: {}", tile, tile_inps.at(i),
                                   tile_outs.at(i), clk_zone_sequence_to_update);
                    }
                }
            }

            // fmt::print("\t\t Sequence to update: {}\n", clk_zone_sequence_to_update);
            std::vector<int> clk_zn_differences;
            clk_zn_differences.push_back(0);
            for (size_t i{1}; i < clk_zone_sequence_to_update.size(); ++i)
            {
                // fmt::print("\t\t Pairs: {} - {}", clk_zone_sequence_to_update.at(i),
                //            clk_zone_sequence_to_update.at(i - 1));
                auto curr = gclk[clk_zone_sequence_to_update.at(i).second][clk_zone_sequence_to_update.at(i).first];
                auto prev =
                    gclk[clk_zone_sequence_to_update.at(i - 1).second][clk_zone_sequence_to_update.at(i - 1).first];
                // fmt::print("\t\t Curr: {} - Prev: {}", curr, prev);
                clk_zn_differences.push_back(curr - prev);
            }

            // fmt::print("\n\t CLK ZONE DIFFERENCES: {}", clk_zn_differences);
            // fmt::print("\n\t TILE INP CLK ZONES: {}", tile_inp_clk_zones);

            for (size_t i{0}; i < tile_inp_clk_zones.size(); ++i)
            {
                auto inp          = tile_inp_clk_zones.at(i).first;
                auto inp_clk_zone = tile_inp_clk_zones.at(i).second;
                fmt::print("\n\t\tINP: {} - CLK ZONE: {}", inp, inp_clk_zone);

                // if (clk_zone_delta == 1)
                // {
                //     continue;
                // }

                gclk[inp.y][inp.x] = inp_clk_zone % 4;
                for (size_t j{1}; j < clk_zone_sequence_to_update.size(); ++j)
                {
                    auto prev_magnet = clk_zone_sequence_to_update.at(j - 1);
                    auto magnet      = clk_zone_sequence_to_update.at(j);
                    gclk[magnet.second][magnet.first] =
                        (gclk[prev_magnet.second][prev_magnet.first] + clk_zn_differences.at(j)) % 4;
                }
            }
            fmt::print("\n\t\t UPDATED TILE CLK ZONES: {}", gclk);
            std::cout << "\t ============ UPDATE TILE CLK SEQUENCE ===============" << std::endl;
        };

        for (size_t i{0}; i < tile_inps.size(); ++i)
        {
            auto oppositePort = GateLibrary::opposite(tile_inps[i]);
            tile_inp_feeders.push_back(oppositePort);
            get_tile_inp_feeders_clk_zone(oppositePort, tile_inp_feeders);
        }

        if (pred_tiles.size() > 0 && !GateLibrary::is_crosswire(cell))
        {
            update_tile_clock_zones_sequence(tile_inp_feeders);
        }

        // Iterate over inputs and update their clock zones based on the tile_inp_feeders_clk_zone or their default
        // clock zone

        auto gate_inp_clk_zone = gclk[first_inp.y][first_inp.x];
        std::cout << " -- Tile INP CLK SCH: " << fmt::format("{}", gate_inp_clk_zone) << std::endl;

        if (gate_inp_clk_zone != (pred_biggest_clk_number + 1))
        {
            auto clk_delta = (gate_inp_clk_zone - (pred_biggest_clk_number + 1)) * -1;

            if (GateLibrary::is_crosswire(cell))
            {
                gate_clk_sch = GateLibrary::get_crosswire_clock_scheme(pred_biggest_clk_number);
            }
            else
            {
                for (size_t y{0}; y < gate_clk_sch.size(); ++y)
                {
                    for (size_t x{0}; x < gate_clk_sch[y].size(); ++x)
                    {
                        if (gate_clk_sch[x][y] != -1)
                        {
                            gate_clk_sch[x][y] = (gate_clk_sch[x][y] + clk_delta) % 4;
                        }
                    }
                }
            }

            tile_gate_cell_layout_map[tile] = std::make_pair(tile_portlist, gate_clk_sch);
        }
        std::cout << " -------------------- ASSIGN CLOCK ZONES -------------------- \n\n" << std::endl;

        return gate_clk_sch;
    }
};

}  // namespace detail

/**
 * Applies a gate library to a given gate-level layout and, thereby, creates and returns a cell-level layout. The gate
 * library type should provide all functions specified in fcn_gate_library. It is, thus, easiest to extend
 * fcn_gate_library to implement a new gate library. Examples are `qca_one_library`, `inml_topolinano_library`,
 * `sidb_bestagon_library` and `nmlib_inml_library`.
 *
 * May pass through, and thereby throw, an `unsupported_gate_type_exception` or an
 * `unsupported_gate_orientation_exception`.
 *
 * @tparam CellLyt Type of the returned cell-level layout.
 * @tparam GateLibrary Type of the gate library to apply.
 * @tparam GateLyt Type of the gate-level layout to apply the library to.
 * @param lyt The gate-level layout.
 * @return A cell-level layout that implements `lyt`'s gate types with building blocks defined in `GateLibrary`.
 */
template <typename CellLyt, typename GateLibrary, typename GateLyt>
CellLyt apply_gate_library(const GateLyt& lyt)
{
    static_assert(is_cell_level_layout_v<CellLyt>, "CellLyt is not a cell-level layout");
    static_assert(!has_siqad_coord_v<CellLyt>, "CellLyt cannot have SiQAD coordinates");
    static_assert(is_gate_level_layout_v<GateLyt>, "GateLyt is not a gate-level layout");
    static_assert(mockturtle::has_is_constant_v<GateLyt>, "GateLyt does not implement the is_constant function");
    static_assert(mockturtle::has_foreach_node_v<GateLyt>, "GateLyt does not implement the foreach_node function");

    static_assert(std::is_same_v<technology<CellLyt>, technology<GateLibrary>>,
                  "CellLyt and GateLibrary must implement the same technology");

    detail::apply_gate_library_impl<CellLyt, GateLibrary, GateLyt> p{lyt};

    auto result = p.run();

    return result;
}

}  // namespace fiction

#pragma GCC diagnostic pop

#endif  // FICTION_APPLY_GATE_LIBRARY_HPP
