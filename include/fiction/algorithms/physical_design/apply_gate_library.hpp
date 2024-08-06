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

using float64_t             = double;
using ClockingZoneByPortMap = std::map<port_position, int>;
using ClockingZoneByNodeMap = std::map<int, ClockingZoneByPortMap>;

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

                    // GateLibrary::set_up_gate will return a tuple of tile, predecessor tiles, portlist
                    // and a pair of tile and its clocking zone sequence
                    const auto tp = GateLibrary::set_up_gate(gate_lyt, t);

                    auto tile = std::get<0>(tp);
                    // auto pred_tile = std::get<1>(tp);
                    std::vector<fc::tile<GateLyt>> pred_tiles = tile_preds_map[tile];

                    // The tile port list
                    auto portlist = std::get<2>(tp);

                    // Pair <tile, clocking zone>
                    auto pr           = std::get<3>(tp);
                    auto gate         = pr.first;
                    auto clock_scheme = pr.second;

                    // Store tile and a pair of its portlist and its clocking zone scheme
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
    ClockingZoneByNodeMap node_outputs_clocking_zones_map;

    template <typename T>
    decltype(auto) path(Coord<T> p1, Coord<T> p2,
                        std::array<std::array<int, TILE_WIDTH>, TILE_HEIGHT>& clock_zones_map) noexcept
    {
        auto neighbors = [&clock_zones_map](auto&& pair) constexpr -> std::vector<std::pair<int64_t, int64_t>>
        {
            std::vector<std::pair<int64_t, int64_t>> possible_neighbors = {{
                {pair.first - 1, pair.second - 1},
                {pair.first - 1, pair.second},
                {pair.first - 1, pair.second + 1},
                {pair.first, pair.second + 1},
                {pair.first, pair.second - 1},
                {pair.first + 1, pair.second - 1},
                {pair.first + 1, pair.second},
                {pair.first + 1, pair.second + 1},
            }};
            return possible_neighbors;
        };

        return a_star::clk_zn_run(
            std::forward<Coord<T>>(p1), std::forward<Coord<T>>(p2),
            [&](Coord<T> src) -> std::vector<std::pair<int64_t, int64_t>> { return neighbors(src); },
            [&](Coord<T> dest) -> bool
            {
                bool position_with_clk_zone =
                    clock_zones_map[dest.second][dest.first] >= 0 && clock_zones_map[dest.second][dest.first] < 4;
                bool valid_clk_zone = (dest.first >= 0 && dest.second >= 0 && dest.first < NMLIB_TILE_WIDTH &&
                                       dest.second < NMLIB_TILE_HEIGHT) &&
                                      position_with_clk_zone;

                // if (valid_clk_zone)
                // {
                //     // std::cout << fmt::format("\t\tDEST {} - CLK ZONE {} - VALID CLK ZN: {}", dest,
                //     //                          clock_zones_map[dest.second][dest.first], valid_clk_zone)
                //     //           << std::endl;
                // }
                return !valid_clk_zone;
            },
            [&](auto&& a, auto&& b) -> float64_t
            {
                // auto m{1};
                auto m{ns_heuristics::manhattan::run(a, b)};
                // std::cout << fmt::format("\t\t COST {}-{} = {}", a, b, m) << std::endl;
                // auto m{ns_heuristics::euclidian::run(a, b)};
                // return clock_zones_map[b.first][b.second];
                return m;
            });
    }

    decltype(auto)
    get_tile_clk_zn_sequences(const std::array<std::array<int, TILE_WIDTH>, TILE_HEIGHT>& tile_clk_zn_map) noexcept
    {
        // std::vector<std::vector<std::pair<int, int>>> clk_zn_sequences;
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

        ClockingZoneByPortMap updated_clock_zones;

        fmt::print("\n\t ## UPDATED TILE OUTS: {}\n", tile_portlist.out);
        for (auto out : tile_portlist.out)
        {
            fmt::print("\n\t OUT: {} - CLK ZONE: {} ", out, checked_clk_zone[out.y][out.x]);
            updated_clock_zones[out] = checked_clk_zone[out.y][out.x];
        }
        fmt::print("\n");

        this->node_outputs_clocking_zones_map[n] = updated_clock_zones;

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
        fmt::print("\n\t =============== ASSIGN CLOCK ZONES BEGIN =============== \n");

        auto cell = gpair.first;
        auto gclk = gpair.second;
        fmt::print("\n\t CELL: {}", cell);
        fmt::print("\n\t GCLK: {}", gclk);
        fmt::print("\n\t PRED TILES: {}", pred_tiles);
        fmt::print("\n\t TILE: {}", tile);

        // If the predecessor tile is an empty tile
        // returns the predefined clock scheme
        if (pred_tiles.size() == 0 || (pred_tiles.size() == 1 && gate_lyt.is_empty_tile(pred_tiles[0])))
        {
            return gclk;
        }

        // std::cout << "\n ## Tile: " << fmt::format("{}", tile) << std::endl;

        // Tile clock scheme
        auto gate_clk_sch = gclk;

        // Tile list of ports (input or output)
        const auto tile_portlist = tile_gate_cell_layout_map.at(tile).first;

        // Tile input ports list
        std::vector<port_position> tile_inps(tile_portlist.inp.begin(), tile_portlist.inp.end());

        // Tile output ports list
        std::vector<port_position> tile_outs(tile_portlist.out.begin(), tile_portlist.out.end());

        // Biggest clock zone index of the predecessor tile
        auto pred_biggest_clk_number{-1};

        fmt::print("\n\t TILE INP PORTS: {}", tile_inps);
        fmt::print("\n\t TILE OUT PORTS: {}", tile_outs);

        // Clock zones of the tile input ports
        std::vector<int> tile_inps_clk_zone(tile_inps.size(), 0);

        // Tile inps opposite ports
        std::vector<port_position> tile_inp_feeders;

        std::vector<int> tile_inp_feeders_clk_zone;

        std::map<port_position, int> tile_inp_feeders_clk_zone_map;

        // Get predecessor node by input port coordinates
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

        // Get predecessor tile output ports clock zones
        auto get_tile_inp_feeders_clk_zone = [this, &get_pred_node_by_port, &tile_inp_feeders_clk_zone,
                                              &tile_inp_feeders_clk_zone_map, &tile,
                                              &gclk](auto tilePort, std::vector<port_position> tile_inp_feeders) -> void
        {
            // fmt::print("\n\t ====== GET TILE INP FEEDERS CLK ZONE BEGIN ======= \n");
            // fmt::print("\n\t ======== SEARCH CLK ZN FOR TILE PORT: {}", tilePort);

            // Predecessor node
            auto     predPair  = get_pred_node_by_port(tile, tilePort);
            auto     predNode  = predPair.first;
            uint16_t predTileY = predPair.second.first;
            uint16_t predTileX = predPair.second.second;

            // fmt::print("\n\t === PRED NODE: {} - COORD: ({}, {})", predNode, predTileY, predTileX);

            if (predNode == 0 || predNode == -1)
            {
                // fmt::print("\n\t === PRED NODE DOESN'T EXIST");
                // fmt::print("\n\t === THIS IS AN INPUT TILE");
                tile_inp_feeders_clk_zone.push_back(0);
                tile_inp_feeders_clk_zone_map[tile_inp_feeders[0]] = 0;
                return;
            }

            if (this->node_outputs_clocking_zones_map.find(predNode) == this->node_outputs_clocking_zones_map.end())
            {
                // fmt::print("\n\t === OUTPUTS CLOCKING ZONES DOESN'T CONTAIN PRED NODE");
                tile_inp_feeders_clk_zone.push_back(0);
                tile_inp_feeders_clk_zone_map[tile_inp_feeders[0]] = 0;
                return;
            }

            // fmt::print("\n\t === PRED NODE EXISTS: {} - COORD: ({}, {})", predNode, predTileY, predTileX);
            auto predNodeOutputsClockingZones = this->node_outputs_clocking_zones_map.at(predNode);
            // fmt::print("\n\t === PRED NODE OUTPUTS CLOCKING ZONES: {}", predNodeOutputsClockingZones);
            // fmt::print("\n\t === PRED NODE CLOCKING ZONES: {}", gclk[predTileY][predTileX]);

            for (auto feeder_port : tile_inp_feeders)
            {
                if (predNodeOutputsClockingZones.find(feeder_port) != predNodeOutputsClockingZones.end())
                {
                    auto feeder_port_clk_zn = predNodeOutputsClockingZones.at(feeder_port);
                    tile_inp_feeders_clk_zone.push_back(feeder_port_clk_zn);
                    tile_inp_feeders_clk_zone_map[feeder_port] = feeder_port_clk_zn;
                }
            }
            // fmt::print("\n\t ====== GET TILE INP FEEDERS CLK ZONE END ======= \n");
        };

        // Get tile input ports clock zones
        auto get_tile_inps_clk_zone = [this, &tile_inp_feeders_clk_zone, &tile_inp_feeders_clk_zone_map, &tile_inps,
                                       &tile_outs, &tile, &pred_tiles, &cell,
                                       &gate_clk_sch](auto tile_inp_feeders) -> std::map<port_position, int>
        {
            // std::cout << "\n\t ============ GET TILE INPS CLK ZONES BEGIN ===============" << std::endl;
            // fmt::print("\n\t\t TILE X: {} - Y: {}", tile.x, tile.y);
            // fmt::print("\n\t\t TILE INPS: {}", tile_inps);
            // fmt::print("\n\t\t TILE INP FEEDERS: {}", tile_inp_feeders);
            // fmt::print("\n\t\t TILE INP FEEDERS CLK ZONES: {}\n", tile_inp_feeders_clk_zone);
            // fmt::print("\n\t\t TILE INP FEEDERS CLK ZONES MAP: {}\n", tile_inp_feeders_clk_zone_map);

            // Inputs new clock zones
            std::map<port_position, int> new_inps_clk_zones;

            // Boolean to store if the inputs clock zones are different between them
            bool different_clk_zones = false;

            // If the predecessor outputs clock zones are empty
            if (tile_inp_feeders_clk_zone.size() == 0)
            {
                // Visit each tile input port and store its clock zone
                for (size_t i{0}; i < tile_inps.size(); i++)
                {
                    auto ty                          = tile_inps[i].y;
                    auto tx                          = tile_inps[i].x;
                    auto i_clk_zone                  = gate_clk_sch[ty][tx];
                    new_inps_clk_zones[tile_inps[i]] = i_clk_zone;
                }
            }
            else if (tile_inp_feeders_clk_zone.size() == 1)
            {
                auto feeder_clk_zn = tile_inp_feeders_clk_zone.at(0);
                if (gate_lyt.is_pi_tile(pred_tiles[0]))
                {
                    new_inps_clk_zones[tile_inps.at(0)] = feeder_clk_zn;
                }
                else
                {
                    auto fclk_zone                      = ((feeder_clk_zn + 1) % 4);
                    new_inps_clk_zones[tile_inps.at(0)] = fclk_zone;
                }
            }
            else
            {
                if (GateLibrary::is_crosswire(cell))
                {
                    // fmt::print("\n\t\t UPDATING CROSSWIRE CLK ZN SCHEME");
                    auto hz_feeder_clk_zn = tile_inp_feeders_clk_zone_map[{4, 2}];
                    auto vt_feeder_clk_zn = tile_inp_feeders_clk_zone_map[{2, 4}];
                    // fmt::print("\n\t\t HZ FEEDER CLK ZN: {}", hz_feeder_clk_zn);
                    // fmt::print("\n\t\t VT FEEDER CLK ZN: {}", vt_feeder_clk_zn);
                    if ((hz_feeder_clk_zn == 1 || hz_feeder_clk_zn == 2) &&
                        (vt_feeder_clk_zn == 0 || vt_feeder_clk_zn == 3))
                    {
                        // set crosswire clk sch type II
                        new_inps_clk_zones[{0, 2}] = 2;
                        new_inps_clk_zones[{2, 0}] = 0;
                    }
                    else
                    {
                        // set crosswire clk sch type I
                        new_inps_clk_zones[{0, 2}] = 0;
                        new_inps_clk_zones[{2, 0}] = 2;
                    }
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

                    // std::cout << "\n\t\t DIFFERENT CLK ZONES: " << fmt::format("{}", different_clk_zones) <<
                    // std::endl;

                    // TODO: I need to run an A* algorithm to mount clock zone sequences from inputs to outputs
                    if (!different_clk_zones)
                    {
                        // TODO: Run routine to update the tile clock zone based on the value of one of the inputs
                        // Clock zone index
                        int clk_zone_idx = tile_inp_feeders_clk_zone.at(0);
                    }
                }
            }

            // std::cout << "\t ============ GET TILE INPS CLK ZONES END ===============" << std::endl;
            return new_inps_clk_zones;
        };

        auto update_tile_clock_zones_sequence = [this, &tile_inps, &cell, &is_gate, &tile_outs, &tile, &gate_clk_sch,
                                                 &get_tile_inps_clk_zone](auto tile_inp_feeders) -> void
        {
            std::cout << "\n\t ============ UPDATE TILE CLK SEQUENCE ===============" << std::endl;

            auto tile_inp_clk_zones = get_tile_inps_clk_zone(tile_inp_feeders);
            fmt::print("\n\t\t TILE INPS: {}\n", tile_inps);
            fmt::print("\n\t\t TILE OUTS: {}\n", tile_outs);
            fmt::print("\n\t\t TILE INP CLK ZONES: {}\n", tile_inp_clk_zones);

            if (GateLibrary::is_crosswire(cell))
            {
                fmt::print("\n\t\t UPDATING CROSSWIRE CLK ZN SCHEME");
                if (tile_inp_clk_zones[{0, 2}] == 2 && tile_inp_clk_zones[{2, 0}] == 0)
                {
                    // set crosswire clk sch type II
                    gate_clk_sch = GateLibrary::get_crosswire_clock_scheme(1);
                    fmt::print("\n\t\t UPDATED TILE CLK ZONES: {}", gate_clk_sch);
                    fmt::print("\n\t ============ UPDATE TILE CLK SEQUENCE =============== \n");
                    return;
                }
                gate_clk_sch = GateLibrary::get_crosswire_clock_scheme(0);
                fmt::print("\n\t\t UPDATED TILE CLK ZONES: {}", gate_clk_sch);
                fmt::print("\n\t ============ UPDATE TILE CLK SEQUENCE =============== \n");
                return;
            }

            std::vector<std::pair<int, int>> clk_zone_sequence_to_update;
            // Routine to update gates clock zone sequences
            if (is_gate)
            {
                fmt::print("\n\t\t ******** UPDATING GATE CLOCK ZONE SEQUENCES\n");
                // TODO: Calculate paths from each input to each output
                for (size_t i{0}; i < tile_inps.size(); ++i)
                {
                    Coord<int64_t> src{std::make_pair(tile_inps.at(i).x, tile_inps.at(i).y)};
                    fmt::print("\n\t\t SEARCH PATH FROM {} ", src);
                    for (size_t j{0}; j < tile_outs.size(); ++j)
                    {
                        Coord<int64_t> dst{std::make_pair(tile_outs.at(j).x, tile_outs.at(j).y)};
                        fmt::print(" TO {}", dst);
                        auto clk_zn_sequence_to_update{this->path(src, dst, gate_clk_sch)};

                        if (clk_zn_sequence_to_update)
                        {
                            fmt::print("\n\t\t GATE: {} - Src: {} - Dst: {} - A*: {}\n", tile, tile_inps.at(i),
                                       tile_outs.at(j), *clk_zn_sequence_to_update);
                        }

                        clk_zone_sequence_to_update.insert(clk_zone_sequence_to_update.end(),
                                                           (*clk_zn_sequence_to_update).begin(),
                                                           (*clk_zn_sequence_to_update).end());
                    }
                }
                // TODO: Check if the paths are synchronous
                // TODO: If they are not synchronous, update the clock zone sequences adjusting the first magnets clock
                // zones to make them synchronous If they are synchronous, update the clock zones sequences
                // synchronously
            }
            else
            {
                for (size_t i{0}; i < tile_inps.size(); ++i)
                {

                    Coord<int64_t> src{std::make_pair(tile_inps.at(i).x, tile_inps.at(i).y)};
                    std::cout << fmt::format("\n\t\t SEARCH PATH FROM {} ", src) << std::endl;
                    for (size_t j{0}; j < tile_outs.size(); ++j)
                    {
                        Coord<int64_t> dst{std::make_pair(tile_outs.at(j).x, tile_outs.at(j).y)};
                        std::cout << fmt::format(" TO {}", dst) << std::endl;
                        auto clk_zn_sequence_to_update{this->path(src, dst, gate_clk_sch)};
                        // clk_zone_sequence_to_update = get_tile_clk_zn_sequences(gate_clk_sch);

                        // if (clk_zone_sequence_to_update.size() > 0)
                        if (clk_zn_sequence_to_update)
                        {
                            fmt::print("\n\t\t Tile: {} - Src: {} - Dst: {} - A*: {}\n", tile, tile_inps.at(i),
                                       tile_outs.at(j), *clk_zn_sequence_to_update);
                        }

                        clk_zone_sequence_to_update.insert(clk_zone_sequence_to_update.end(),
                                                           (*clk_zn_sequence_to_update).begin(),
                                                           (*clk_zn_sequence_to_update).end());
                    }
                }
            }

            // Clock zone delta between adjacent magnets whithin a tile
            std::vector<int> clk_zn_differences;
            clk_zn_differences.push_back(0);

            for (size_t i{1}; i < clk_zone_sequence_to_update.size(); ++i)
            {
                auto curr =
                    gate_clk_sch[clk_zone_sequence_to_update.at(i).second][clk_zone_sequence_to_update.at(i).first];
                auto prev = gate_clk_sch[clk_zone_sequence_to_update.at(i - 1).second]
                                        [clk_zone_sequence_to_update.at(i - 1).first];
                clk_zn_differences.push_back(curr - prev);
            }

            fmt::print("\n\t\t CLK ZONE DIFFERENCES: {}", clk_zn_differences);

            // for (size_t i{0}; i < tile_inp_clk_zones.size(); ++i)
            for (const auto& [key, value] : tile_inp_clk_zones)
            {
                auto inp          = key;
                auto inp_clk_zone = value;
                fmt::print("\n\t\t\tINP: {} - CLK ZONE: {}", inp, inp_clk_zone);

                gate_clk_sch[inp.y][inp.x] = inp_clk_zone % 4;
                for (size_t j{1}; j < clk_zone_sequence_to_update.size(); ++j)
                {
                    auto prev_magnet = clk_zone_sequence_to_update.at(j - 1);
                    auto magnet      = clk_zone_sequence_to_update.at(j);
                    gate_clk_sch[magnet.second][magnet.first] =
                        (gate_clk_sch[prev_magnet.second][prev_magnet.first] + clk_zn_differences.at(j)) % 4;
                }
            }
            fmt::print("\n\t\t UPDATED TILE CLK ZONES: {}", gate_clk_sch);
            fmt::print("\n\t ============ UPDATE TILE CLK SEQUENCE =============== \n");
        };

        // Loop to search the feeders of all input ports of current tile
        for (size_t i{0}; i < tile_inps.size(); ++i)
        {
            auto oppositePort = GateLibrary::opposite(tile_inps[i]);
            tile_inp_feeders.push_back(oppositePort);
            get_tile_inp_feeders_clk_zone(oppositePort, tile_inp_feeders);
        }

        if (pred_tiles.size() > 0)
        {
            update_tile_clock_zones_sequence(tile_inp_feeders);
            tile_gate_cell_layout_map[tile] = std::make_pair(tile_portlist, gate_clk_sch);
        }

        // Iterate over inputs and update their clock zones based on the tile_inp_feeders_clk_zone or their default
        // clock zone
        fmt::print("\n\t ============= ASSIGN CLOCK ZONES ==============  \n");

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
