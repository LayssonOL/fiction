//
// Created by marcel on 28.06.21.
//

#ifndef FICTION_APPLY_GATE_LIBRARY_HPP
#define FICTION_APPLY_GATE_LIBRARY_HPP

#include "fiction/technology/cell_ports.hpp"
#include "fiction/technology/inml_topolinano_library.hpp"
#include "fiction/technology/nmlib_inml_library.hpp"
#include "fiction/technology/qca_one_library.hpp"
#include "fiction/technology/sidb_bestagon_library.hpp"
#include "fiction/traits.hpp"
#include "fiction/utils/layout_utils.hpp"

#include <mockturtle/traits.hpp>

#include <cstddef>
#include <cstdint>
#include <ctime>
#include <iostream>
#include <map>
#include <type_traits>
#include <utility>

#if (PROGRESS_BARS)
#include <mockturtle/utils/progress_bar.hpp>
#endif

// data types cannot properly be converted to bit field types
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wuseless-cast"
#pragma GCC diagnostic ignored "-Wconversion"

namespace fiction
{

namespace detail
{

using ClockingZonesTilesPair = std::pair<std::vector<port_position>, std::vector<int>>;
using ClockingZonesMap       = std::map<int, ClockingZonesTilesPair>;

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

                    ClockingZonesMap node_outputs_clocking_zones_map;

                    auto tile      = std::get<0>(tp);
                    auto pred_tile = std::get<1>(tp);

                    auto portlist     = std::get<2>(tp);
                    auto pr           = std::get<3>(tp);
                    auto gate         = pr.first;
                    auto clock_scheme = pr.second;
                    tile_gate_cell_layout_map.emplace(std::make_pair(t, std::make_pair(portlist, clock_scheme)));

                    assign_gate(c, pred_tile, tile, pr, n, node_outputs_clocking_zones_map);
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

    void assign_gate(const cell<CellLyt>& c, const tile<GateLyt>& pred_tile, const tile<GateLyt>& tile,
                     const typename GateLibrary::fcn_gate_clk_sch& gclk, const mockturtle::node<GateLyt>& n,
                     ClockingZonesMap& node_outputs_clocking_zones_map)
    {
        auto start_x = c.x;
        auto start_y = c.y;
        auto layer   = c.z;
        auto g       = gclk.first;
        auto clk     = gclk.second;

        std::cout << " ################# ASSIGN GATE BEGIN #################" << std::endl;
        bool is_gate = this->gate_lyt.is_gate(n) && !this->gate_lyt.is_wire(n);

        const auto tile_portlist = tile_gate_cell_layout_map.at(tile).first;

        if (is_gate)
        {
            std::cout << " \n NODE: " << fmt::format("{}", n) << std::endl;
            std::cout << " IS GATE: " << fmt::format("{}", is_gate) << std::endl;
            std::cout << " PRED TILE: " << fmt::format("{}", pred_tile) << std::endl;
            std::cout << " TILE: " << fmt::format("{}", tile) << std::endl;
            std::cout << " TILE PORTS: " << fmt::format("{}", tile_portlist) << "\n\n" << std::endl;
            std::cout << " TILE INPS: " << fmt::format("{}", tile_portlist.inp) << "\n\n" << std::endl;
            std::cout << " TILE OUTS: " << fmt::format("{}", tile_portlist.out) << "\n\n" << std::endl;
        }

        auto checked_clk_zone = assign_clock_zones(pred_tile, tile, gclk, is_gate,
                                                   std::forward<ClockingZonesMap&>(node_outputs_clocking_zones_map));

        ClockingZonesTilesPair updated_clock_zones;

        for (auto out : tile_portlist.out)
        {
            std::cout << " OUT: " << fmt::format("{}", out)
                      << " CLOCK ZONE: " << fmt::format("{}", checked_clk_zone[out.x][out.y]) << "\n\n"
                      << std::endl;
            updated_clock_zones.first.push_back(out);
            updated_clock_zones.second.push_back(checked_clk_zone[out.x][out.y]);
        }

        node_outputs_clocking_zones_map.insert(
            {std::stoi(fmt::format("{}", n)), std::make_pair(updated_clock_zones.first, updated_clock_zones.second)});

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
        std::cout << " ################# ASSIGN GATE END #################" << std::endl;
    }

    GateLibrary::fcn_clk_sch assign_clock_zones(const tile<GateLyt>& pred_tile, const tile<GateLyt>& tile,
                                                const typename GateLibrary::fcn_gate_clk_sch& gpair, bool is_gate,
                                                ClockingZonesMap& node_outputs_clocking_zones_map)
    {
        std::cout << " ==================== ASSIGN CLOCK ZONES ==================== \n" << std::endl;

        auto cell = gpair.first;
        auto gclk = gpair.second;

        // If the predecessor tile is an empty tile
        // returns the predefined clock scheme
        if (gate_lyt.is_empty_tile(pred_tile))
        {
            std::cout << "\n ## PRED IS EMPTY TILE: " << std::endl;
            if (is_gate)
            {
                std::cout << " ## " << fmt::format("Tile {} is gate {}", tile, is_gate) << std::endl;
                auto                       gate_clk_sch  = gclk;
                const auto                 tile_portlist = tile_gate_cell_layout_map.at(tile).first;
                std::vector<port_position> tile_inp(tile_portlist.inp.begin(), tile_portlist.inp.end());
                const auto                 first_inp = tile_inp[0];
                std::cout << " ## Tile INPS: " << fmt::format("{}", tile_inp) << std::endl;
                return gclk;
            }
            return gclk;
        }
        else if (is_gate)
        {
            std::cout << "\n ## Tile is GATE:  " << fmt::format("Tile {} has gate {}", tile, is_gate) << std::endl;

            auto       gate_clk_sch  = gclk;
            const auto tile_portlist = tile_gate_cell_layout_map.at(tile).first;

            std::cout << " ## Tile INPS: " << fmt::format("{}", tile_portlist) << "\n\n" << std::endl;

            std::vector<port_position> tile_inp(tile_portlist.inp.begin(), tile_portlist.inp.end());

            const auto first_inp = tile_inp[0];

            std::cout << " ## Tile INPS: " << fmt::format("{}", first_inp) << "\n\n" << std::endl;
        }

        // TODO: Use the pred_output_ports to define the clock zones sequence into the current tile
        auto pred_node         = gate_lyt.get_node(pred_tile);
        auto pred_output_ports = node_outputs_clocking_zones_map.at(pred_node);

        std::cout << "\n ## Pred Tile: " << fmt::format("{}", pred_tile) << std::endl;
        std::cout << " ## Tile: " << fmt::format("{}", tile) << std::endl;
        std::cout << " ## FCN GATE TYPE: " << fmt::format("{}", cell) << std::endl;
        std::cout << " ## FCN GATE CLK SCH TYPE: " << fmt::format("{}", typeid(gclk).name()) << std::endl;

        const auto                 pred_tile_pair      = tile_gate_cell_layout_map.at(pred_tile);
        const auto                 pred_tile_port_list = pred_tile_pair.first;
        const auto                 pred_tile_clk_sch   = pred_tile_pair.second;
        std::vector<port_position> vc(pred_tile_port_list.out.begin(), pred_tile_port_list.out.end());
        // std::cout << "## Pred OUT PORTS: " << fmt::format("{}", vc) << std::endl;

        // std::cout << "\n ## Tile: " << fmt::format("{}", tile) << std::endl;
        auto                       gate_clk_sch  = gclk;
        const auto                 tile_portlist = tile_gate_cell_layout_map.at(tile).first;
        std::vector<port_position> tile_inp(tile_portlist.inp.begin(), tile_portlist.inp.end());
        const auto                 first_inp = tile_inp[0];

        auto pred_biggest_clk_number{pred_tile_clk_sch[2][4]};

        // if (tile_inp.size() == 1)
        // {
        auto oppositePort       = GateLibrary::opposite(first_inp);
        pred_biggest_clk_number = pred_tile_clk_sch[oppositePort.y][oppositePort.x];
        // }

        // std::cout << " ## Pred Tile OUT CLK ZONES: " << fmt::format("{}", pred_tile_clk_sch) << std::endl;
        // std::cout << " ## Pred Biggest CLK NUMBER: " << fmt::format("{}", pred_biggest_clk_number) << std::endl;

        auto gate_inp_clk_zone = gclk[first_inp.y][first_inp.x];
        // std::cout << " ## Tile INP CLK SCH: " << fmt::format("{}", gate_inp_clk_zone) << std::endl;

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

            // std::cout << " ## BEFORE : " << fmt::format("{}", gclk) << std::endl;
            // std::cout << " ## AFTER : " << fmt::format("{}", gate_clk_sch) << std::endl;
            tile_gate_cell_layout_map[tile] = std::make_pair(tile_portlist, gate_clk_sch);
        }
        std::cout << " ==================== ASSIGN CLOCK ZONES ==================== \n\n" << std::endl;

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
