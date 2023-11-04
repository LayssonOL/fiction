//
// Created by marcel on 29.07.19.
//

// #ifndef FICTION_INML_TOPOLINANO_LIBRARY_HPP
#include "cell_ports.hpp"

#include <cstdint>
#include <iostream>
#include <utility>
#define FICTION_INML_TOPOLINANO_LIBRARY_HPP

#include "fiction/technology/cell_ports.hpp"
#include "fiction/technology/cell_technologies.hpp"
#include "fiction/technology/fcn_gate_library.hpp"
#include "fiction/traits.hpp"
#include "fiction/utils/array_utils.hpp"

#include <fmt/format.h>
#include <mockturtle/traits.hpp>
#include <phmap.h>

#include <algorithm>
#include <vector>

const uint8_t NMLIB_TILE_HEIGHT = 5;
const uint8_t NMLIB_TILE_WIDTH  = 5;

namespace fiction
{
/**
 * A concrete FCN gate library as used in \"NMLib\" for the iNML technology.
 */
class nmlib_inml_library : public fcn_gate_library<nmlib_inml_technology, NMLIB_TILE_HEIGHT, NMLIB_TILE_WIDTH>
{
  public:
    explicit nmlib_inml_library() = delete;
    /**
     * Overrides the corresponding function in fcn_gate_library. Given a tile `t`, this function takes all necessary
     * information from the stored grid into account to choose the correct fcn_gate representation for that tile.
     * May it be a gate or wires. Rotation and special marks like input and output, const cells etc. are computed
     * additionally.
     *
     * @tparam GateLyt Shifted Cartesian gate-level layout type.
     * @param lyt Layout that hosts tile `t`.
     * @param t Tile to be realized as a NMLib gate.
     * @return NMLib gate representation of `t` including I/Os, rotation, etc.
     */
    [[nodiscard]] static bool is_crosswire(const fcn_gate& cell)
    {
        for (size_t y{0}; y < NMLIB_TILE_HEIGHT; ++y)
        {
            for (size_t x{0}; x < NMLIB_TILE_WIDTH; ++x)
            {
                if (cell[y][x] != CROSSWIRE[y][x])
                    return false;
            }
        }
        return true;
    }

    [[nodiscard]] static fcn_clk_sch get_crosswire_clock_scheme(const uint16_t& pred_clock_number)
    {
        if (pred_clock_number == 1 || pred_clock_number == 2)
        {
            return CROSSWIRE_CLOCK_SCHEME_II;
        }
        else
        {
            return CROSSWIRE_CLOCK_SCHEME;
        }
    }

    template <typename GateLyt>
    [[nodiscard]] static std::tuple<tile<GateLyt>, tile<GateLyt>, port_list<port_position>, fcn_gate_clk_sch>
    set_up_gate(const GateLyt& lyt, const tile<GateLyt>& t)
    {
        static_assert(is_gate_level_layout_v<GateLyt>, "Lyt must be a gate-level layout");
        // static_assert(is_shifted_cartesian_layout_v<GateLyt>, "Lyt must be a shifted Cartesian layout");

        // crossing magnets are handled only in the ground layer
        if (lyt.is_crossing_layer(t))
        {
            port_list<port_position> pl = {{port_position(0, 0)}, {port_position(0, 0)}};
            return {t, {}, pl, std::make_pair(EMPTY_GATE, EMPTY_GATE_CLOCK_SCHEME)};
        }

        const auto n = lyt.get_node(t);

        if constexpr (fiction::has_is_fanout_v<GateLyt>)
        {
            if (lyt.is_fanout(n))
            {
                port_list<port_position> pl = {{port_position(0, 2)}, {port_position(4, 0), port_position(4, 4)}};
                return {t, {}, pl, std::make_pair(COUPLER, COUPLER_WIRE_CLOCK_SCHEME)};
            }
        }
        if constexpr (mockturtle::has_is_and_v<GateLyt>)
        {
            if (lyt.is_and(n))
            {
                port_list<port_position> pl = {{port_position(0, 2), port_position(2, 0)}, {port_position(4, 2)}};
                return {t, {}, pl, std::make_pair(CONJUNCTION, CONJUNCTION_CLOCK_SCHEME)};
            }
        }
        if constexpr (mockturtle::has_is_or_v<GateLyt>)
        {
            if (lyt.is_or(n))
            {
                port_list<port_position> pl = {{port_position(0, 2), port_position(2, 0)}, {port_position(4, 2)}};
                return {t, {}, pl, std::make_pair(DISJUNCTION, DISJUNCTION_CLOCK_SCHEME)};
            }
        }
        if constexpr (mockturtle::has_is_maj_v<GateLyt>)
        {
            if (lyt.is_maj(n))
            {
                port_list<port_position> pl = {{port_position(0, 2), port_position(2, 0), port_position(2, 4)},
                                               {port_position(4, 2)}};
                return {t, {}, pl, std::make_pair(MAJORITY, MAJORITY_CLOCK_SCHEME)};
            }
        }

        const auto pair      = determine_port_routing(lyt, t);
        const auto p         = pair.second;
        const auto pred_tile = pair.first;

        try
        {
            if constexpr (fiction::has_is_inv_v<GateLyt>)
            {
                if (lyt.is_inv(n))
                {
                    return {t, pred_tile, p, std::make_pair(INVERTER_MAP.at(p), INVERTER_CLOCK_SCHEME_MAP.at(p))};
                }
            }

            if constexpr (fiction::has_is_buf_v<GateLyt>)
            {
                if (lyt.is_buf(n))
                {
                    const auto a        = lyt.above(t);
                    const auto is_equal = t != a;

                    // crossing case
                    if (const auto a = lyt.above(t); t != a && lyt.is_wire_tile(a))
                    {
                        return {t, pred_tile, p, std::make_pair(CROSSWIRE, CROSSWIRE_CLOCK_SCHEME)};
                    }

                    auto wire            = WIRE_MAP.at(p);
                    auto wire_clk_scheme = WIRE_CLOCK_SCHEME_MAP.at(p);
                    auto wire_print      = fmt::format("WIRE: {} - INP: {} - OUT: {}", wire, p.inp, p.out);
                    std::cout << wire_print << std::endl;

                    if (lyt.is_pi(n))
                    {
                        const auto inp_mark_pos = p.inp.empty() ? opposite(*p.out.begin()) : *p.inp.begin();
                        wire = mark_cell(wire, inp_mark_pos, nmlib_inml_technology::cell_mark::INPUT);
                    }
                    if (lyt.is_po(n))
                    {
                        const auto out_mark_pos = p.out.empty() ? opposite(*p.inp.begin()) : *p.out.begin();
                        wire = mark_cell(wire, out_mark_pos, nmlib_inml_technology::cell_mark::OUTPUT);
                    }
                    return {t, pred_tile, p, std::make_pair(wire, wire_clk_scheme)};
                }
            }
        }
        catch (const std::out_of_range&)
        {
            throw unsupported_gate_orientation_exception(t, p);
        }

        throw unsupported_gate_type_exception(t);
    }

    /**
     * Post-layout optimization that straightens the wire segments to save cells.
     *
     * @tparam CellLyt Cell-level layout type.
     * @param lyt The cell-level layout that has been created via application of `set_up_gate`.
     */
    template <typename CellLyt>
    static void post_layout_optimization(CellLyt& lyt) noexcept
    {
        static_assert(is_cell_level_layout_v<CellLyt>, "CellLyt must be a cell-level layout");

        enum class status
        {
            SEARCH,
            COLLECT,
            SKIP
        };

        bool improvement_found{false};

        const auto handle = [&lyt, &improvement_found](const auto& hump)
        {
            // single cell collections are no humps, do not merge them.
            if (hump.size() > 1)
            {
                // if sequence starts with a PI or there are normal cells south of first and last hump cell, this is
                // an upper hump
                if (const auto inp = lyt.get_cell_type(hump.front()), out = lyt.get_cell_type(hump.back()),
                    fts = lyt.get_cell_type(lyt.south(hump.front())), bts = lyt.get_cell_type(lyt.south(hump.back()));
                    (inp == nmlib_inml_technology::cell_type::INPUT ||
                     fts == nmlib_inml_technology::cell_type::NORMAL) &&
                    (bts == nmlib_inml_technology::cell_type::NORMAL ||
                     out == nmlib_inml_technology::cell_type::OUTPUT ||
                     bts == nmlib_inml_technology::cell_type::INVERTER_MAGNET))
                {
                    // hump found, check if there is enough space below for merging
                    if (std::all_of(hump.begin() + 1, hump.end() - 2,
                                    [&lyt](const auto hc) { return lyt.is_empty_cell(lyt.south(lyt.south(hc))); }))
                    {
                        // merge it down
                        for (const auto& hc : hump)
                        {
                            const auto s = lyt.south(hc);
                            if (lyt.get_cell_type(s) != nmlib_inml_technology::cell_type::INVERTER_MAGNET)
                            {
                                lyt.assign_cell_type(s, lyt.get_cell_type(hc));
                                lyt.assign_cell_mode(s, lyt.get_cell_mode(hc));
                                lyt.assign_cell_name(s, lyt.get_cell_name(hc));
                            }
                            lyt.assign_cell_type(hc, nmlib_inml_technology::cell_type::EMPTY);
                            lyt.assign_cell_name(hc, "");
                        }

                        improvement_found = true;
                    }
                }
                // if there are normal cells north of first and last hump cell, this is a lower hump
                else if (const auto ftn = lyt.get_cell_type(lyt.north(hump.front())),
                         btn            = lyt.get_cell_type(lyt.north(hump.back()));
                         (inp == nmlib_inml_technology::cell_type::INPUT ||
                          ftn == nmlib_inml_technology::cell_type::NORMAL) &&
                         (btn == nmlib_inml_technology::cell_type::NORMAL ||
                          out == nmlib_inml_technology::cell_type::OUTPUT))
                {
                    // hump found, check if there is enough space above for merging
                    if (std::all_of(hump.begin() + 1, hump.end() - 2,
                                    [&lyt](const auto hc) { return lyt.is_empty_cell(lyt.north(lyt.north(hc))); }))
                    {
                        // merge it up
                        for (const auto& hc : hump)
                        {
                            const auto n = lyt.north(hc);
                            lyt.assign_cell_type(n, lyt.get_cell_type(hc));
                            lyt.assign_cell_mode(n, lyt.get_cell_mode(hc));
                            lyt.assign_cell_name(n, lyt.get_cell_name(hc));
                            lyt.assign_cell_type(hc, nmlib_inml_technology::cell_type::EMPTY);
                            lyt.assign_cell_name(hc, "");
                        }

                        improvement_found = true;
                    }
                }
            }
        };

        do {
            status st = status::SEARCH;

            improvement_found = false;

            for (decltype(lyt.y()) row = 0; row <= lyt.y(); ++row)
            {
                std::vector<cell<CellLyt>> hump{};

                for (decltype(lyt.x()) column = 0; column <= lyt.x(); ++column)
                {
                    // simple state machine for identifying humps and removing them
                    switch (const auto c = cell<CellLyt>{column, row}; st)
                    {
                        case status::SEARCH:
                        {
                            switch (const auto t = lyt.get_cell_type(c); t)
                            {
                                // encountering a normal, input, or inverter magnet triggers collecting hump cells
                                case nmlib_inml_technology::cell_type::NORMAL:
                                case nmlib_inml_technology::cell_type::INPUT:
                                case nmlib_inml_technology::cell_type::INVERTER_MAGNET:
                                {
                                    st = status::COLLECT;
                                    hump.push_back(c);
                                    break;
                                }
                                // remain searching
                                case nmlib_inml_technology::cell_type::EMPTY:
                                {
                                    break;
                                }
                                // everything else leads to skipping
                                default:
                                {
                                    st = status::SKIP;
                                    break;
                                }
                            }
                            break;
                        }
                        case status::COLLECT:
                        {
                            switch (const auto t = lyt.get_cell_type(c); t)
                            {
                                // collect cells
                                case nmlib_inml_technology::cell_type::NORMAL:
                                case nmlib_inml_technology::cell_type::INVERTER_MAGNET:
                                {
                                    hump.push_back(c);
                                    break;
                                }
                                // interesting branch: could be a hump
                                case nmlib_inml_technology::cell_type::EMPTY:
                                case nmlib_inml_technology::cell_type::OUTPUT:
                                {
                                    handle(hump);
                                    // discard hump cells and start searching again
                                    hump.clear();
                                    st = status::SEARCH;
                                    break;
                                }
                                // encountered anything else: cannot be a hump
                                default:
                                {
                                    hump.clear();
                                    st = status::SKIP;
                                    break;
                                }
                            }
                            break;
                        }
                        case status::SKIP:
                        {
                            if (const auto t = lyt.get_cell_type(c); t == nmlib_inml_technology::cell_type::EMPTY)
                            {
                                // skipping over, return to searching
                                st = status::SEARCH;
                            }
                            break;
                        }
                        break;  // silence compiler warning
                    }
                }
            }
        } while (improvement_found);
    }

  private:
    /**
     * Checks whether the given node has an AND, OR, or MAJ fanin node.
     *
     * @tparam Lyt Gate-level layout type.
     * @param lyt The layout to check.
     * @param n Node whose fanins are to be considered.
     * @return `true` iff `n` has an AND, OR, or MAJ fanin node.
     */
    template <typename Lyt>
    [[nodiscard]] static bool has_and_or_maj_fanin(const Lyt& lyt, const mockturtle::node<Lyt>& n) noexcept
    {
        auto pre_and_or_maj = false;

        lyt.foreach_fanin(n,
                          [&lyt, &pre_and_or_maj](const auto& fi)
                          {
                              const auto fin = lyt.get_node(fi);

                              if constexpr (mockturtle::has_is_and_v<Lyt>)
                              {
                                  if (lyt.is_and(fin))
                                  {
                                      pre_and_or_maj = true;
                                      return false;  // exit function
                                  }
                              }
                              if constexpr (mockturtle::has_is_or_v<Lyt>)
                              {
                                  if (lyt.is_or(fin))
                                  {
                                      pre_and_or_maj = true;
                                      return false;  // exit function
                                  }
                              }
                              if constexpr (mockturtle::has_is_maj_v<Lyt>)
                              {
                                  if (lyt.is_maj(fin))
                                  {
                                      pre_and_or_maj = true;
                                      return false;  // exit function
                                  }
                              }
                              return true;  // continue iteration
                          });

        return pre_and_or_maj;
    }
    /**
     * Checks whether the given node has an fanout node as fanout.
     *
     * @tparam Lyt Gate-level layout type.
     * @param lyt The layout to check.
     * @param n Node whose fanouts are to be considered.
     * @return `true` iff `n` has a fanout node as fanout.
     */
    template <typename Lyt>
    [[nodiscard]] static bool has_fanout_fanout(const Lyt& lyt, const mockturtle::node<Lyt>& n) noexcept
    {
        auto fanout_fanout = false;

        lyt.foreach_fanout(n,
                           [&lyt, &fanout_fanout](const auto& fon)
                           {
                               if constexpr (fiction::has_is_fanout_v<Lyt>)
                               {
                                   if (lyt.is_fanout(fon))
                                   {
                                       fanout_fanout = true;
                                       return false;  // exit function
                                   }
                               }
                               return true;  // continue iteration
                           });

        return fanout_fanout;
    }

    template <typename Lyt>
    [[nodiscard]] static std::pair<tile<Lyt>, port_list<port_position>>
    determine_port_routing(const Lyt& lyt, const tile<Lyt>& t) noexcept
    {
        static_assert(fiction::has_is_inv_v<Lyt>, "Lyt must implement the is_inv function");
        static_assert(fiction::has_is_po_v<Lyt>, "Lyt must implement the is_po function");
        static_assert(mockturtle::has_is_pi_v<Lyt>, "Lyt must implement the is_pi function");
        static_assert(mockturtle::has_is_and_v<Lyt>, "Lyt must implement the is_and function");
        static_assert(mockturtle::has_is_or_v<Lyt>, "Lyt must implement the is_or function");
        static_assert(mockturtle::has_is_maj_v<Lyt>, "Lyt must implement the is_maj function");

        tile<Lyt>                pred_tile = t;
        port_list<port_position> p{};

        const auto n = lyt.get_node(t);

        // NOLINTBEGIN(*-branch-clone)

        // wires within the circuit
        if (lyt.is_buf(n) && !lyt.is_pi(n) && !lyt.is_po(n))
        {
            // inputs
            if (lyt.has_northern_incoming_signal(t))
            {
                pred_tile.y = pred_tile.y - 1;
                p.inp.emplace(2u, 0u);
            }
            else if (lyt.has_southern_incoming_signal(t))
            {
                pred_tile.y = pred_tile.y + 1;
                p.inp.emplace(2u, 4u);
            }
            else if (lyt.has_south_western_incoming_signal(t))
            {

                pred_tile.y = pred_tile.y + 1;
                pred_tile.x = pred_tile.x - 1;
                // special case: if predecessor is AND, OR, MAJ, input port is at (0,3)
                if (has_and_or_maj_fanin(lyt, n))
                {
                    p.inp.emplace(0u, 4u);
                }
                else
                {
                    p.inp.emplace(0u, 2u);
                }
            }
            else
            {
                pred_tile.x = pred_tile.x - 1;
                p.inp.emplace(0u, 2u);
            }

            // outputs
            if (lyt.has_northern_outgoing_signal(t))
            {
                // special case: if northern tile is MAJ, output port is at (1,0)
                if (lyt.is_maj(lyt.get_node(lyt.north(t))))
                {
                    p.out.emplace(1u, 0u);
                }
                else
                {
                    p.out.emplace(4u, 0u);
                }
            }
            else if (lyt.has_southern_outgoing_signal(t))
            {
                p.out.emplace(2u, 4u);
            }
            else if (lyt.has_north_eastern_outgoing_signal(t))
            {
                p.out.emplace(4u, 0u);
            }
            else if (lyt.has_south_eastern_outgoing_signal(t))
            {
                // special case: if successor is a fanout, output port is at (3,3)
                if (has_fanout_fanout(lyt, n))
                {
                    p.out.emplace(4u, 4u);
                }
                else
                {
                    p.out.emplace(4u, 2u);
                }
            }
            else
            {
                p.out.emplace(4u, 2u);
            }
        }
        else  // PI, PO, etc.
        {
            // special case: PO determines output according to its predecessor
            if (lyt.is_po(n))
            {
                // if predecessor is an AND/OR/MAJ gate, input port is at (0,3) and output port at (3,3)
                if (has_and_or_maj_fanin(lyt, n))
                {

                    // but only if that gate is SW of the PO
                    if (lyt.has_south_western_incoming_signal(t))
                    {
                        pred_tile.x = pred_tile.x - 1;
                        p.inp.emplace(0u, 2u);
                        p.out.emplace(4u, 4u);

                        return std::make_pair(pred_tile, p);
                    }
                    if (lyt.has_western_incoming_signal(t))
                    {

                        pred_tile.x = pred_tile.x - 1;
                        p.inp.emplace(0u, 2u);
                        p.out.emplace(4u, 4u);

                        return std::make_pair(pred_tile, p);
                    }
                }
                // if input is north-western, output port is at (3,0)
                else if (lyt.has_north_western_incoming_signal(t))
                {

                    p.out.emplace(4u, 0u);
                }
                // if input is south-western, output port is at (3,2)
                else if (lyt.has_south_western_incoming_signal(t))
                {

                    p.out.emplace(4u, 4u);
                }
                else if (lyt.has_western_incoming_signal(t))
                {

                    p.out.emplace(4u, 4u);
                }
            }

            if (lyt.has_north_western_incoming_signal(t))
            {
                pred_tile.y = pred_tile.y - 1;
                p.inp.emplace(2u, 0u);
            }
            if (lyt.has_south_western_incoming_signal(t))
            {
                if (lyt.is_po(n) || lyt.is_inv(n))
                {
                    pred_tile.y = pred_tile.y - 1;
                    // special case: if predecessor is AND, OR, MAJ, input port is at (2,4)
                    if (has_and_or_maj_fanin(lyt, n))
                    {
                        p.inp.emplace(2u, 4u);
                    }
                    else
                    {
                        p.inp.emplace(0u, 4u);
                    }
                }
                else
                {
                    pred_tile.x = pred_tile.x - 1;
                    p.inp.emplace(0u, 2u);
                }
            }
            // special case: PI determines input according to successor
            if (lyt.is_pi(n))
            {
                // if successor is a fan-out, input port is at (0,3) and output port at (3,3)
                if (has_fanout_fanout(lyt, n))
                {
                    pred_tile.x = pred_tile.x - 1;
                    p.inp.emplace(0u, 3u);
                    p.out.emplace(3u, 3u);

                    return std::make_pair(pred_tile, p);
                }
                // if output is north-eastern, input port is at (0,0)
                if (lyt.has_north_eastern_outgoing_signal(t))
                {
                    pred_tile.y = pred_tile.y + 1;
                    p.inp.emplace(2u, 4u);
                }
                else if (lyt.has_eastern_outgoing_signal(t))
                {
                    pred_tile.x = pred_tile.x - 1;
                    p.inp.emplace(0u, 2u);
                }
                // if output is south-eastern, input port is at (0,2)
                else if (lyt.has_south_eastern_outgoing_signal(t))
                {
                    pred_tile.y = pred_tile.y - 1;
                    p.inp.emplace(2u, 0u);
                }
                else if (lyt.has_southern_outgoing_signal(t))
                {
                    pred_tile.y = pred_tile.y - 1;
                    p.inp.emplace(2u, 0u);
                }
            }
            // determine outgoing connector ports
            if (lyt.has_north_eastern_outgoing_signal(t))
            {

                // special case: output port of AND, OR, MAJ is fixed at pos (3,1)
                if (lyt.is_and(n) || lyt.is_or(n) || lyt.is_maj(n))
                {
                    p.out.emplace(3u, 1u);
                }
                else
                {
                    p.out.emplace(3u, 0u);
                }
            }
            if (lyt.has_south_eastern_outgoing_signal(t))
            {
                p.out.emplace(3u, 2u);
            }
        }

        // NOLINTEND(*-branch-clone)

        return std::make_pair(pred_tile, p);
    }

    static port_position opposite(const port_position& p)
    {
        using port_port_map = phmap::flat_hash_map<port_position, port_position>;

        static const port_port_map pp_map = {
            {port_position(0, 0), port_position(4, 0)}, {port_position(0, 1), port_position(4, 1)},
            {port_position(0, 2), port_position(4, 2)}, {port_position(0, 3), port_position(4, 3)},
            {port_position(0, 4), port_position(4, 4)}, {port_position(4, 0), port_position(0, 0)},
            {port_position(4, 1), port_position(0, 1)}, {port_position(4, 2), port_position(0, 2)},
            {port_position(4, 3), port_position(0, 3)}, {port_position(4, 4), port_position(0, 4)},
            {port_position(2, 0), port_position(2, 4)}, {port_position(2, 4), port_position(2, 0)}};

        return pp_map.at(p);
    }

    // clang-format off

    // ************************************************************
    // ************************** Gates ***************************
    // ************************************************************

    static constexpr const fcn_gate CONJUNCTION{cell_list_to_gate<char>(
    {{
        {' ', ' ', 'n', 'd', 'N'},
        {' ', ' ', ' ', ' ', 'n'},
        {'n', ' ', ' ', ' ', 'u'},
        {' ', 'N', ' ', ' ', 'n'},
        {' ', ' ', 'N', 'd', 'N'},
    }})};

    static constexpr const fcn_clk_sch CONJUNCTION_CLOCK_SCHEME{clock_list_to_clk_sch<int>(
    {{
        {-1, -1, 0, 1, 2},
        {-1, -1, -1, -1, 2},
        {0, -1, -1, -1, 3},
        {-1, 0, -1, -1, 2},
        {-1, -1, 1, 1, 2},
    }})};

    static constexpr const fcn_gate DISJUNCTION{cell_list_to_gate<char>(
    {{
        {'N', 'n', 'N', 'n', 'N'},
        {' ', ' ', ' ', ' ', 'n'},
        {' ', ' ', ' ', ' ', 'y'},
        {' ', ' ', ' ', ' ', 'n'},
        {'N', 'n', 'N', 'n', 'N'},
    }})};

    static constexpr const fcn_clk_sch DISJUNCTION_CLOCK_SCHEME{clock_list_to_clk_sch<int>(
    {{
        { 0,  0, 1, 1, 2},
        {-1,-1, -1,-1, 2},
        {-1,-1, -1,-1, 3},
        {-1,-1, -1,-1, 2},
        { 0,  0, 1, 1, 2},
    }})};

    static constexpr const fcn_gate MAJORITY{cell_list_to_gate<char>(
    {{
        {' ', ' ', 'N', ' ', ' '},
        {' ', ' ', 'n', ' ', ' '},
        {'n', 'd', 'n', 'c', 'n'},
        {' ', ' ', 'n', ' ', ' '},
        {' ', ' ', 'N', ' ', ' '},
    }})};

    static constexpr const fcn_clk_sch MAJORITY_CLOCK_SCHEME{clock_list_to_clk_sch<int>(
    {{
        {-1, -1, 0, -1, -1},
        {-1, -1, 0, -1, -1},
        {0, 0, 1, 2, 2},
        {-1, -1, 0, -1, -1},
        {-1, -1, 0, -1, -1},
    }})};

    static constexpr const fcn_gate LOWER_STRAIGHT_INVERTER{cell_list_to_gate<char>(
    {{
        {' ', ' ', ' ', ' ', ' '},
        {' ', ' ', ' ', ' ', ' '},
        {'n', ' ', ' ', ' ', ' '},
        {' ', 'n', ' ', ' ', ' '},
        {' ', ' ', 'N', 'n', 'n'},
    }})};

    static constexpr const fcn_clk_sch LOWER_STRAIGHT_INVERTER_CLOCK_SCHEME{clock_list_to_clk_sch<int>(
    {{
        {-1, -1, -1, -1, -1},
        {-1, -1, -1, -1, -1},
        { 0, -1, -1, -1, -1},
        {-1, 0, -1, -1, -1},
        {-1, -1, 1,  1,  2},
    }})};

    static constexpr const fcn_gate BOTTOM_LOWER_STRAIGHT_INVERTER{cell_list_to_gate<char>(
    {{
        {' ', ' ', ' ', ' ', ' '},
        {' ', ' ', ' ', ' ', ' '},
        {' ', ' ', ' ', ' ', ' '},
        {'N', 'n', ' ', ' ', ' '},
        {' ', ' ', 'N', 'n', 'n'},
    }})};

    static constexpr const fcn_clk_sch BOTTOM_LOWER_STRAIGHT_INVERTER_CLOCK_SCHEME{clock_list_to_clk_sch<int>(
    {{
        {-1, -1, -1, -1, -1},
        {-1, -1, -1, -1, -1},
        {-1, -1, -1, -1, -1},
        { 0,  0, -1, -1, -1},
        {-1,  -1, 1,  1,  2},
    }})};

    static constexpr const fcn_gate UPPER_STRAIGHT_INVERTER{cell_list_to_gate<char>(
    {{
        {' ', ' ', 'N', 'n', 'n'},
        {' ', 'n', ' ', ' ', ' '},
        {'n', ' ', ' ', ' ', ' '},
        {' ', ' ', ' ', ' ', ' '},
        {' ', ' ', ' ', ' ', ' '},
    }})};

    static constexpr const fcn_clk_sch UPPER_STRAIGHT_INVERTER_CLOCK_SCHEME{clock_list_to_clk_sch<int>(
    {{
        {-1, -1, 1, 1, 2},
        {-1, 0, -1, -1, -1},
        { 0, -1, -1, -1, -1},
        {-1, -1, -1, -1, -1},
        {-1, -1, -1, -1, -1},
    }})};

    static constexpr const fcn_gate TOP_DOWN_BENT_INVERTER{cell_list_to_gate<char>(
    {{
        {' ', 'N', ' ', ' ', ' '},
        {' ', 'n', ' ', ' ', ' '},
        {' ', 'N', '6', 'n', 'n'},
        {' ', ' ', ' ', ' ', ' '},
        {' ', ' ', ' ', ' ', ' '},
    }})};

    static constexpr const fcn_clk_sch TOP_DOWN_BENT_INVERTER_CLOCK_SCHEME{clock_list_to_clk_sch<int>(
    {{
        {-1, 0, -1, -1, -1},
        {-1, 0, -1, -1, -1},
        {-1, 1,  2,  3,  0},
        {-1, -1, -1, -1, -1},
        {-1, -1, -1, -1, -1},
    }})};

    static constexpr const fcn_gate BOTTOM_UP_BENT_INVERTER{cell_list_to_gate<char>(
    {{
        {' ', ' ', ' ', ' ', ' '},
        {' ', ' ', ' ', ' ', ' '},
        {' ', 'N', '6', 'n', 'l'},
        {' ', 'n', ' ', ' ', ' '},
        {' ', 'N', ' ', ' ', ' '},
    }})};

    static constexpr const fcn_clk_sch BOTTOM_UP_BENT_INVERTER_CLOCK_SCHEME{clock_list_to_clk_sch<int>(
    {{
        {-1, -1, -1, -1, -1},
        {-1, -1, -1, -1, -1},
        {-1,  1,  2,  3, 3},
        {-1,  0, -1, -1, -1},
        {-1,  0, -1, -1, -1},
    }})};

    static constexpr const fcn_gate BOTTOM_LOWER_UP_BENT_INVERTER{cell_list_to_gate<char>(
    {{
        {'N', '6', 'n', 'N', 'n'},
        {'n', ' ', ' ', ' ', ' '},
        {'N', ' ', ' ', ' ', ' '},
        {'n', ' ', ' ', ' ', ' '},
        {'N', ' ', ' ', ' ', ' '},
    }})};

    static constexpr const fcn_clk_sch BOTTOM_LOWER_UP_BENT_INVERTER_CLOCK_SCHEME{clock_list_to_clk_sch<char>(
    {{
        {2, 3, 0, 1, 1},
        {1, -1, -1, -1, -1},
        {1, -1, -1, -1, -1},
        {0, -1, -1, -1, -1},
        {0, -1, -1, -1, -1},
    }})};

    // ************************************************************
    // ************************** Wires ***************************
    // ************************************************************

    static constexpr const fcn_gate CROSSWIRE{cell_list_to_gate<char>(
    {{
        {' ', ' ', 'N', ' ', ' '},
        {' ', ' ', 'n', ' ', ' '},
        {'n', 'd', 'n', 'c', 'n'},
        {' ', ' ', 'n', ' ', ' '},
        {' ', ' ', 'N', ' ', ' '},
    }})};

    static constexpr const fcn_clk_sch CROSSWIRE_CLOCK_SCHEME{clock_list_to_clk_sch<int>(
    {{
        {-1, -1, 2, -1, -1},
        {-1, -1, 2, -1, -1},
        { 0, 0, 4, 1, 2},
        {-1, -1, 3, -1, -1},
        {-1, -1, 3, -1, -1},
    }})};

    static constexpr const fcn_clk_sch CROSSWIRE_CLOCK_SCHEME_II{clock_list_to_clk_sch<int>(
    {{
        {-1, -1, 0, -1, -1},
        {-1, -1, 0, -1, -1},
        { 2, 2, 4, 3, 0},
        {-1, -1, 1, -1, -1},
        {-1, -1, 1, -1, -1},
    }})};

    static constexpr const fcn_gate COUPLER{cell_list_to_gate<char>(
    {{
        {' ', ' ', 'N', '6', 'n'},
        {' ', ' ', 'n', ' ', ' '},
        {'N', 'n', 'N', ' ', ' '},
        {' ', ' ', 'n', ' ', ' '},
        {' ', ' ', 'N', '6', 'n'},
    }})};

    static constexpr const fcn_clk_sch COUPLER_CLOCK_SCHEME{clock_list_to_clk_sch<int>(
    {{
        {-1, -1, 3, 3, 0},
        {-1, -1, 2, -1, -1},
        { 0, 0, 1, -1, -1},
        {-1, -1, 2, -1, -1},
        {-1, -1, 3, 3, 0},
    }})};

    static constexpr const fcn_gate LOWER_WIRE{cell_list_to_gate<char>(
    {{
        {' ', ' ', ' ', ' ', ' '},
        {' ', ' ', ' ', ' ', ' '},
        {' ', ' ', ' ', ' ', ' '},
        {' ', ' ', ' ', ' ', ' '},
        {'N', 'n', 'N', 'n', 'n'},
    }})};

    static constexpr const fcn_clk_sch LOWER_WIRE_CLOCK_SCHEME{clock_list_to_clk_sch<int>(
    {{
        {-1, -1, -1, -1, -1},
        {-1, -1, -1, -1, -1},
        {-1, -1, -1, -1, -1},
        {-1, -1, -1, -1, -1},
        { 0,  0, 1, 1, 1},
    }})};

    static constexpr const fcn_gate MIDDLER_WIRE{cell_list_to_gate<char>(
    {{
        {' ', ' ', ' ', ' ', ' '},
        {' ', ' ', ' ', ' ', ' '},
        {'N', 'n', 'N', 'n', 'n'},
        {' ', ' ', ' ', ' ', ' '},
        {' ', ' ', ' ', ' ', ' '},
    }})};
    
    static constexpr const fcn_clk_sch MIDDLER_WIRE_CLOCK_SCHEME{clock_list_to_clk_sch<int>(
    {{
        {-1, -1, -1, -1, -1},
        {-1, -1, -1, -1, -1},
        { 0,  0,  1,  1,  1},
        {-1, -1, -1, -1, -1},
        {-1, -1, -1, -1, -1},
    }})};

    static constexpr const fcn_gate INPUT_MIDDLER_WIRE{cell_list_to_gate<char>(
    {{
        {' ', ' ', ' ', ' ', ' '},
        {' ', ' ', ' ', ' ', ' '},
        {' ', 'n', 'N', 'n', 'n'},
        {' ', ' ', ' ', ' ', ' '},
        {' ', ' ', ' ', ' ', ' '},
    }})};

    static constexpr const fcn_clk_sch INPUT_MIDDLER_WIRE_CLOCK_SCHEME{clock_list_to_clk_sch<int>(
    {{
        {-1, -1, -1, -1, -1},
        {-1, -1, -1, -1, -1},
        { 0,  0,  1,  1,  1},
        {-1, -1, -1, -1, -1},
        {-1, -1, -1, -1, -1},
    }})};

    static constexpr const fcn_gate TOP_DOWN_BENT_WIRE{cell_list_to_gate<char>(
    {{
        {'N', ' ', ' ', ' ', ' '},
        {'n', ' ', ' ', ' ', ' '},
        {'N', '6', 'n', '7', 'N'},
        {' ', ' ', ' ', ' ', 'n'},
        {' ', ' ', ' ', ' ', 'N'},
    }})};

    static constexpr const fcn_clk_sch TOP_DOWN_BENT_WIRE_CLOCK_SCHEME{clock_list_to_clk_sch<int>(
    {{
        {0, -1, -1, -1, -1},
        {0, -1, -1, -1, -1},
        {1,  2,  2,  3,  0},
        {-1, -1, -1, -1, 1},
        {-1, -1, -1, -1, 2},
    }})};

    static constexpr const fcn_gate TOP_RIGHT_BENT_WIRE{cell_list_to_gate<char>(
    {{
        {' ', ' ', 'N', ' ', ' '},
        {' ', ' ', 'n', ' ', ' '},
        {' ', ' ', 'N', '6', 'n'},
        {' ', ' ', ' ', ' ', ' '},
        {' ', ' ', ' ', ' ', ' '},
    }})};

    static constexpr const fcn_clk_sch TOP_RIGHT_BENT_WIRE_CLOCK_SCHEME{clock_list_to_clk_sch<int>(
    {{
        {-1, -1, 0, -1, -1},
        {-1, -1, 0, -1, -1},
        {-1, -1, 1, 2, 2},
        {-1, -1, -1, -1, -1},
        {-1, -1, -1, -1, -1},
    }})};

    static constexpr const fcn_gate BOTTOM_UP_BENT_WIRE{cell_list_to_gate<char>(
    {{
        {' ', ' ', ' ', ' ', 'N'},
        {' ', ' ', ' ', ' ', 'n'},
        {'N', '6', 'n', '7', 'N'},
        {'n', ' ', ' ', ' ', ' '},
        {'N', ' ', ' ', ' ', ' '},
    }})};

    static constexpr const fcn_clk_sch BOTTOM_UP_BENT_WIRE_CLOCK_SCHEME{clock_list_to_clk_sch<int>(
    {{
        {-1, -1, -1, -1, 1},
        {-1, -1, -1, -1, 0},
        { 1,  2,  2,  3, 0},
        {0, -1, -1, -1, -1},
        {0, -1, -1, -1, -1},
    }})};

    static constexpr const fcn_gate BOTTOM_RIGHT_BENT_WIRE{cell_list_to_gate<char>(
    {{
        {' ', ' ', ' ', ' ', ' '},
        {' ', ' ', ' ', ' ', ' '},
        {' ', ' ', 'N', 'c', 'n'},
        {' ', ' ', 'n', ' ', ' '},
        {' ', ' ', 'N', ' ', ' '}
    }})};

    static constexpr const fcn_clk_sch BOTTOM_RIGHT_BENT_WIRE_CLOCK_SCHEME{clock_list_to_clk_sch<int>(
    {{
        {-1, -1, -1, -1, -1},
        {-1, -1, -1, -1, -1},
        {-1, -1, 1, 2, 2},
        {-1, -1, 0, -1, -1},
        {-1, -1, 0, -1, -1}
    }})};

    static constexpr const fcn_gate TOP_DOWN_STAIRCASE_WIRE{cell_list_to_gate<char>(
    {{
        {'N', ' ', ' ', ' ', ' '},
        {'n', ' ', ' ', ' ', ' '},
        {'N', '6', 'N', ' ', ' '},
        {' ', ' ', 'n', ' ', ' '},
        {' ', ' ', 'N', '6', 'n'}
    }})};

    static constexpr const fcn_clk_sch TOP_DOWN_STAIRCASE_WIRE_CLOCK_SCHEME{clock_list_to_clk_sch<int>(
    {{
        {0, -1, -1, -1, -1},
        {0, -1, -1, -1, -1},
        {1, 1, 2, -1, -1},
        {-1, -1, 2, -1, -1},
        {-1, -1, 3, 0, 0}
    }})};

    static constexpr const fcn_gate BOTTOM_UP_STAIRCASE_WIRE{cell_list_to_gate<char>(
    {{
        {' ', ' ', 'N', '6', 'n'},
        {' ', ' ', 'n', ' ', ' '},
        {'N', '6', 'N', ' ', ' '},
        {'n', ' ', ' ', ' ', ' '},
        {'N', ' ', ' ', ' ', ' '}
    }})};

    static constexpr const fcn_clk_sch BOTTOM_UP_STAIRCASE_WIRE_CLOCK_SCHEME{clock_list_to_clk_sch<int>(
    {{
        {-1, -1, 3, 0, 1},
        {-1, -1, 2, -1, -1},
        {1, 1, 2, -1, -1},
        {0, -1, -1, -1, -1},
        {0, -1, -1, -1, -1}
    }})};

    static constexpr const fcn_gate BOTTOM_DOWN_BENT_WIRE{cell_list_to_gate<char>(
    {{
        {' ', ' ', ' ', ' ', ' '},
        {' ', ' ', ' ', ' ', ' '},
        {'n', '7', 'N', ' ', ' '},
        {' ', ' ', 'n', ' ', ' '},
        {' ', ' ', 'N', '6', 'n'}
    }})};

    static constexpr const fcn_clk_sch BOTTOM_DOWN_BENT_WIRE_CLOCK_SCHEME{clock_list_to_clk_sch<int>(
    {{
        {-1, -1, -1, -1, -1},
        {-1, -1, -1, -1, -1},
        {0, 0, 1, -1, -1},
        {-1, -1, 2, -1, -1},
        {-1, -1, 2, 3, 3}
    }})};

    static constexpr const fcn_gate LEFT_DOWN_BENT_WIRE{cell_list_to_gate<char>(
    {{
        {' ', ' ', ' ', ' ', ' '},
        {' ', ' ', ' ', ' ', ' '},
        {'n', '7', 'N', ' ', ' '},
        {' ', ' ', 'n', ' ', ' '},
        {' ', ' ', 'N', ' ', ' '}
    }})};

    static constexpr const fcn_clk_sch LEFT_DOWN_BENT_WIRE_CLOCK_SCHEME{clock_list_to_clk_sch<int>(
    {{
        {-1, -1, -1, -1, -1},
        {-1, -1, -1, -1, -1},
        {0, 0, 1, -1, -1},
        {-1, -1, 2, -1, -1},
        {-1, -1, 2, -1, -1}
    }})};

    static constexpr const fcn_gate LEFT_UP_BENT_WIRE{cell_list_to_gate<char>(
    {{
        {' ', ' ', 'N', ' ', ' '},
        {' ', ' ', 'n', ' ', ' '},
        {'n', '7', 'N', ' ', ' '},
        {' ', ' ', ' ', ' ', ' '},
        {' ', ' ', ' ', ' ', ' '}
    }})};
    
    static constexpr const fcn_clk_sch LEFT_UP_BENT_WIRE_CLOCK_SCHEME{clock_list_to_clk_sch<int>(
    {{
        {-1, -1, 2, -1, -1},
        {-1, -1, 1, -1, -1},
        {0, 0, 1, -1, -1},
        {-1, -1, -1, -1, -1},
        {-1, -1, -1, -1, -1}
    }})};


    static constexpr const fcn_gate MAJORITY_WIRE{cell_list_to_gate<char>(
    {{
        {'n', 'n', ' ', ' ', ' '},
        {' ', ' ', ' ', ' ', ' '},
        {' ', ' ', ' ', ' ', ' '},
        {' ', ' ', ' ', ' ', ' '},
        {' ', ' ', ' ', ' ', ' '}
    }})};

    static constexpr const fcn_clk_sch MAJORITY_WIRE_CLOCK_SCHEME{clock_list_to_clk_sch<int>(
    {{
        {0, 0, -1, -1, -1},
        {-1, -1, -1, -1, -1},
        {-1, -1, -1, -1, -1},
        {-1, -1, -1, -1, -1},
        {-1, -1, -1, -1, -1}
    }})};

    static constexpr const fcn_gate COUPLER_WIRE{cell_list_to_gate<char>(
    {{
        {' ', ' ', ' ', ' ', ' '},
        {' ', ' ', ' ', ' ', ' '},
        {'N', '6', 'n', 'N', 'n'},
        {'n', ' ', ' ', ' ', ' '},
        {'N', ' ', ' ', ' ', ' '}
    }})};

    static constexpr const fcn_clk_sch COUPLER_WIRE_CLOCK_SCHEME{clock_list_to_clk_sch<int>(
    {{
        {-1, -1, -1, -1, -1},
        {-1, -1, -1, -1, -1},
        {1, 2, 2, 3, 3},
        {0, -1, -1, -1, -1},
        {0, -1, -1, -1, -1}
    }})};

    // clang-format on

    using port_gate_map    = phmap::flat_hash_map<port_list<port_position>, fcn_gate>;
    using port_clk_sch_map = phmap::flat_hash_map<port_list<port_position>, fcn_clk_sch>;

    static inline const port_gate_map WIRE_MAP = {
        // straight wires
        {{{port_position(0, 2)}, {port_position(3, 2)}}, LOWER_WIRE},
        {{{}, {port_position(3, 2)}}, LOWER_WIRE},
        {{{port_position(0, 2)}, {}}, INPUT_MIDDLER_WIRE},
        {{{port_position(0, 0)}, {port_position(3, 0)}}, MIDDLER_WIRE},
        {{{}, {port_position(3, 0)}}, MIDDLER_WIRE},
        {{{port_position(0, 0)}, {}}, MIDDLER_WIRE},
        {{{port_position(0, 0)}, {port_position(3, 0)}}, rotate_180(LOWER_WIRE)},
        {{{}, {port_position(3, 3)}}, rotate_180(LOWER_WIRE)},
        {{{port_position(0, 3)}, {}}, rotate_180(LOWER_WIRE)},
        // bent wires
        {{{port_position(0, 0)}, {port_position(3, 2)}}, TOP_DOWN_BENT_WIRE},
        {{{port_position(0, 2)}, {port_position(3, 0)}}, BOTTOM_UP_BENT_WIRE},
        {{{port_position(0, 2)}, {port_position(3, 3)}}, BOTTOM_DOWN_BENT_WIRE},
        // staircase wires
        {{{port_position(0, 0)}, {port_position(3, 3)}}, TOP_DOWN_STAIRCASE_WIRE},
        {{{port_position(0, 3)}, {port_position(3, 0)}}, BOTTOM_UP_STAIRCASE_WIRE},
        // special wires
        {{{port_position(0, 0)}, {port_position(1, 0)}}, MAJORITY_WIRE},
        {{{port_position(0, 3)}, {port_position(3, 2)}}, COUPLER_WIRE},
        // NOTE more wires go here!

        // NMLIB Tile of 5x5
        // straight wires
        {{{port_position(0, 4)}, {port_position(4, 4)}}, LOWER_WIRE},
        {{{}, {port_position(4, 4)}}, LOWER_WIRE},
        {{{port_position(0, 4)}, {}}, LOWER_WIRE},
        {{{port_position(0, 2)}, {port_position(4, 2)}}, MIDDLER_WIRE},
        {{{}, {port_position(4, 2)}}, MIDDLER_WIRE},
        {{{port_position(0, 2)}, {}}, MIDDLER_WIRE},
        {{{port_position(0, 0)}, {port_position(4, 0)}}, rotate_180(LOWER_WIRE)},
        {{{}, {port_position(4, 0)}}, rotate_180(LOWER_WIRE)},
        {{{port_position(0, 0)}, {}}, rotate_180(LOWER_WIRE)},
        // bent wires
        {{{port_position(0, 0)}, {port_position(4, 2)}}, TOP_DOWN_BENT_WIRE},
        {{{port_position(2, 0)}, {port_position(4, 2)}}, TOP_RIGHT_BENT_WIRE},
        {{{port_position(0, 2)}, {port_position(4, 0)}}, BOTTOM_UP_BENT_WIRE},
        {{{port_position(0, 2)}, {port_position(4, 4)}}, BOTTOM_DOWN_BENT_WIRE},
        {{{port_position(0, 2)}, {port_position(2, 4)}}, LEFT_DOWN_BENT_WIRE},
        // staircase wires
        {{{port_position(0, 0)}, {port_position(4, 4)}}, TOP_DOWN_STAIRCASE_WIRE},
        {{{port_position(0, 4)}, {port_position(4, 0)}}, BOTTOM_UP_STAIRCASE_WIRE},
        // special wires
        {{{port_position(0, 0)}, {port_position(1, 0)}}, MAJORITY_WIRE},
        {{{port_position(0, 3)}, {port_position(4, 2)}}, COUPLER_WIRE}};

    static inline const port_clk_sch_map WIRE_CLOCK_SCHEME_MAP = {
        // straight wires
        {{{port_position(0, 2)}, {port_position(3, 2)}}, LOWER_WIRE_CLOCK_SCHEME},
        {{{}, {port_position(3, 2)}}, LOWER_WIRE_CLOCK_SCHEME},
        {{{port_position(0, 2)}, {}}, INPUT_MIDDLER_WIRE_CLOCK_SCHEME},
        {{{port_position(0, 0)}, {port_position(3, 0)}}, MIDDLER_WIRE_CLOCK_SCHEME},
        {{{}, {port_position(3, 0)}}, MIDDLER_WIRE_CLOCK_SCHEME},
        {{{port_position(0, 0)}, {}}, MIDDLER_WIRE_CLOCK_SCHEME},
        {{{port_position(0, 0)}, {port_position(3, 0)}}, rotate_180(LOWER_WIRE_CLOCK_SCHEME)},
        {{{}, {port_position(3, 3)}}, rotate_180(LOWER_WIRE_CLOCK_SCHEME)},
        {{{port_position(0, 3)}, {}}, rotate_180(LOWER_WIRE_CLOCK_SCHEME)},
        // bent wires
        {{{port_position(0, 0)}, {port_position(3, 2)}}, TOP_DOWN_BENT_WIRE_CLOCK_SCHEME},
        {{{port_position(0, 2)}, {port_position(3, 0)}}, BOTTOM_UP_BENT_WIRE_CLOCK_SCHEME},
        {{{port_position(0, 2)}, {port_position(3, 3)}}, BOTTOM_DOWN_BENT_WIRE_CLOCK_SCHEME},
        // staircase wires
        {{{port_position(0, 0)}, {port_position(3, 3)}}, TOP_DOWN_STAIRCASE_WIRE_CLOCK_SCHEME},
        {{{port_position(0, 3)}, {port_position(3, 0)}}, BOTTOM_UP_STAIRCASE_WIRE_CLOCK_SCHEME},
        // special wires
        {{{port_position(0, 0)}, {port_position(1, 0)}}, MAJORITY_WIRE_CLOCK_SCHEME},
        {{{port_position(0, 3)}, {port_position(3, 2)}}, COUPLER_WIRE_CLOCK_SCHEME},
        // NOTE more wires go here!

        // NMLIB Tile of 5x5
        // straight wires
        {{{port_position(0, 4)}, {port_position(4, 4)}}, LOWER_WIRE_CLOCK_SCHEME},
        {{{}, {port_position(4, 4)}}, LOWER_WIRE_CLOCK_SCHEME},
        {{{port_position(0, 4)}, {}}, LOWER_WIRE_CLOCK_SCHEME},
        {{{port_position(0, 2)}, {port_position(4, 2)}}, MIDDLER_WIRE_CLOCK_SCHEME},
        {{{}, {port_position(4, 2)}}, MIDDLER_WIRE_CLOCK_SCHEME},
        {{{port_position(0, 2)}, {}}, MIDDLER_WIRE_CLOCK_SCHEME},
        {{{port_position(0, 0)}, {port_position(4, 0)}}, rotate_180(LOWER_WIRE_CLOCK_SCHEME)},
        {{{}, {port_position(4, 0)}}, rotate_180(LOWER_WIRE_CLOCK_SCHEME)},
        {{{port_position(0, 0)}, {}}, rotate_180(LOWER_WIRE_CLOCK_SCHEME)},
        // bent wires
        {{{port_position(0, 0)}, {port_position(4, 2)}}, TOP_DOWN_BENT_WIRE_CLOCK_SCHEME},
        {{{port_position(2, 0)}, {port_position(4, 2)}}, TOP_RIGHT_BENT_WIRE_CLOCK_SCHEME},
        {{{port_position(0, 2)}, {port_position(4, 0)}}, BOTTOM_UP_BENT_WIRE_CLOCK_SCHEME},
        {{{port_position(0, 2)}, {port_position(4, 4)}}, BOTTOM_DOWN_BENT_WIRE_CLOCK_SCHEME},
        {{{port_position(0, 2)}, {port_position(2, 4)}}, LEFT_DOWN_BENT_WIRE_CLOCK_SCHEME},
        // staircase wires
        {{{port_position(0, 0)}, {port_position(4, 4)}}, TOP_DOWN_STAIRCASE_WIRE_CLOCK_SCHEME},
        {{{port_position(0, 4)}, {port_position(4, 0)}}, BOTTOM_UP_STAIRCASE_WIRE_CLOCK_SCHEME},
        // special wires
        {{{port_position(0, 0)}, {port_position(1, 0)}}, MAJORITY_WIRE_CLOCK_SCHEME},
        {{{port_position(0, 3)}, {port_position(4, 2)}}, COUPLER_WIRE_CLOCK_SCHEME}};
    /**
     * Lookup table for inverter rotations. Maps ports to corresponding inverters.
     */
    static inline const port_gate_map INVERTER_MAP = {
        // straight inverters
        {{{port_position(0, 2)}, {port_position(3, 2)}}, LOWER_STRAIGHT_INVERTER},
        {{{port_position(0, 3)}, {port_position(3, 2)}}, BOTTOM_LOWER_STRAIGHT_INVERTER},
        {{{port_position(0, 0)}, {port_position(3, 0)}}, UPPER_STRAIGHT_INVERTER},
        // without outputs
        {{{port_position(0, 2)}, {}}, LOWER_STRAIGHT_INVERTER},
        {{{port_position(0, 0)}, {}}, UPPER_STRAIGHT_INVERTER},
        // without inputs
        {{{}, {port_position(3, 2)}}, LOWER_STRAIGHT_INVERTER},
        {{{}, {port_position(3, 0)}}, UPPER_STRAIGHT_INVERTER},
        // bent inverters
        {{{port_position(0, 0)}, {port_position(3, 2)}}, TOP_DOWN_BENT_INVERTER},
        {{{port_position(0, 2)}, {port_position(3, 0)}}, BOTTOM_UP_BENT_INVERTER},
        {{{port_position(0, 3)}, {port_position(3, 0)}}, BOTTOM_LOWER_UP_BENT_INVERTER}};

    static inline const port_clk_sch_map INVERTER_CLOCK_SCHEME_MAP = {
        // straight inverters
        {{{port_position(0, 2)}, {port_position(3, 2)}}, LOWER_STRAIGHT_INVERTER_CLOCK_SCHEME},
        {{{port_position(0, 3)}, {port_position(3, 2)}}, BOTTOM_LOWER_STRAIGHT_INVERTER_CLOCK_SCHEME},
        {{{port_position(0, 0)}, {port_position(3, 0)}}, UPPER_STRAIGHT_INVERTER_CLOCK_SCHEME},
        // without outputs
        {{{port_position(0, 2)}, {}}, LOWER_STRAIGHT_INVERTER_CLOCK_SCHEME},
        {{{port_position(0, 0)}, {}}, UPPER_STRAIGHT_INVERTER_CLOCK_SCHEME},
        // without inputs
        {{{}, {port_position(3, 2)}}, LOWER_STRAIGHT_INVERTER_CLOCK_SCHEME},
        {{{}, {port_position(3, 0)}}, UPPER_STRAIGHT_INVERTER_CLOCK_SCHEME},
        // bent inverters
        {{{port_position(0, 0)}, {port_position(3, 2)}}, TOP_DOWN_BENT_INVERTER_CLOCK_SCHEME},
        {{{port_position(0, 2)}, {port_position(3, 0)}}, BOTTOM_UP_BENT_INVERTER_CLOCK_SCHEME},
        {{{port_position(0, 3)}, {port_position(3, 0)}}, BOTTOM_LOWER_UP_BENT_INVERTER_CLOCK_SCHEME}};
};
}  // namespace fiction

// #endif  // FICTION_INML_NMLIB_LIBRARY_HPP
