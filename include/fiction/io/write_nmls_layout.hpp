
//
// Created by layssonluz on 23.09.23.
//

#ifndef FICTION_WRITE_QCC_LAYOUT_HPP
#define FICTION_WRITE_QCC_LAYOUT_HPP

#include "fiction/layouts/bounding_box.hpp"
#include "fiction/technology/cell_ports.hpp"
#include "fiction/technology/cell_technologies.hpp"
#include "fiction/technology/nmlsim_magnet_count.hpp"
#include "fiction/traits.hpp"
#include "fiction/types.hpp"
#include "utils/version_info.hpp"

#include <fmt/format.h>

#include <algorithm>
#include <array>
#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <functional>
#include <iostream>
#include <ostream>
#include <stdexcept>
#include <string>
#include <string_view>
#include <unordered_map>
#include <unordered_set>
#include <utility>
#include <vector>

namespace fiction
{

/**
 * Parameters for writing NMLS layouts.
 */
struct write_nmls_layout_params
{
    /**
     * Use the given filename as the component name inside the QCC file.
     */
    bool use_filename_as_component_name = false;
    /**
     * Filename of the QCC file.
     */
    std::string_view filename{};
};

namespace detail
{

namespace nmls
{
inline constexpr const char* LIBRARY_NAME = "components_fiction";
// NMLS_HEADER - Simulation Params {{{
// 2 engines: LLG and Behaviour
inline constexpr const char* TECHNOLOGY = "LLG";

// There are 4 simulationMode: exaustive, direct, repetitive and verbose
inline constexpr const char* SIMULATION_MODE = "verbose";

// There are two possible different methods for the LLG engine, the RK4 and the RKW2 - Runge Kutta 4th order and Runge
// Kutta Weak 2nd order Be mindfull that the RK4 method disconsiders the temperature
inline constexpr const char* LLG_ENGINE_METHOD = "RK4";

// The number of simulations to be done in the repetitive mode
inline constexpr const char* SIMULATION_EXECUTIONS_QNTD = "100";

// The report time step. Starting at time 0, the program reports at each reportStep. Only used in verbose mode
inline constexpr const char* REPORT_STEP = "0.1";

// Gilbert damping factor
inline constexpr const char* ALPHA = "0.05";

// Saturation Magnetization in A/m
inline constexpr const char* SATURATION_MAGNETIZATION = "800000";

// Temperature in K. Be mindfull that the RK-Weak order 2.0 (RKW2) is used for T > 0 and the RK of fourth order (RK4)
// for T == 0
inline constexpr const char* TEMPERATURE = "300";

// Discretization of time in nanoseconds. It's highly recommended using very low timeStep
inline constexpr const char* TIME_STEP = "0.001";

// Simulation Duration in nanoseconds
inline constexpr const char* SIMULATION_TIME = "70";

// Properties for the heavy material for spin hall effect
inline constexpr const char* SPIN_ANGLE           = "0.4";
inline constexpr const char* SPIN_DIFUSION_LENGTH = "3.5";

// Thickness in nanometers
inline constexpr const char* THICKNESS = "5";

// Radius of the considered neighborhood. We recomend using values of 300 for more efficciency.
inline constexpr const char* NEIGHBORHOOD_RATIO = "300";

// Composition Sequence:
// TECHNOLOGY;SIMULATION_MODE; LLG_ENGINE_METHOD; SIMULATION_EXECUTIONS_QNTD; REPORT_STEP; ALPHA;
// SATURATION_MAGNETIZATION; TEMPERATURE; TIME_STEP; SIMULATION_TIME; SPIN_ANGLE; SPIN_DIFUSION_LENGTH; THICKNESS;
// NEIGHBORHOOD_RATIO
inline constexpr const char* NMLS_HEADER = "{};{};{};{};{};{};{};{};{};{};{};{};{};{};";
// }}}

// NMLS - Dimension Constants {{{

// Composition Sequence:
// SUBSTRATE_HEIGHT, SUBSTRATE_WIDTH, MIN_CELL_HEIGHT, MIN_CELL_WIDTH, MAGNET_CENTERS_X_DISTANCE,
// MAGNET_CENTERS_Y_DISTANCE
inline constexpr const char* DIMENSIONS = "{},{},{},{},{},{},";
// }}}

// PHASES_SECTION {{{
//
inline constexpr const char* PHASES_SECTION =
    "Phases\nreset;0,0,0,;300,0,0,;0,0,0,;0,0,0,;5\nrelax;300,0,0,;300,0,0,;0,0,0,;0,0,0,;5\nswitch;300,0,0,;0,0,0,;0,"
    "0,0,;0,0,0,;5\nhold;0,0,0,;0,0,0,;0,0,0,;0,0,0,;5";
// }}}

// ZONES SECTION {{{
//
inline constexpr const char* ZONES_SECTION =
    "Zones\n0;reset;relax;switch;hold;-16777216\n1;hold;reset;relax;switch;-30208\n2;switch;hold;reset;relax;-"
    "14757932\n3;relax;switch;hold;reset;-11206444\n4;relax;switch;relax;switch;-11522794";
// }}}

inline constexpr const char* MAGNETS_SECTION_HEADER = "Magnets\n{}";

// Magnet Specs {{{
// Magnet_{ID};magnet_type;clock_zone;magnetization_vector;fixed_magnetization;width;height;thickness;top_cut;bottom_cut;pos_x,pos_y;color_code;
inline constexpr const char* MAGNET_SPECS = "Magnet_{};{};{};{};{};{};{};{};{};{};{};{};";
// }}}

inline constexpr const char* LAYOUT_ITEM_PROPERTY = "\t\t\t<property name=\"{}\" value=\"{}\"/>\n";
inline constexpr const char* PROPERTY_PHASE       = "phase";
inline constexpr const char* PROPERTY_LENGTH      = "length";

inline constexpr const std::array<const char*, 6> COMPONENTS{"Magnet", "Coupler",  "Cross Wire",
                                                             "And",    "Inverter", "Or"};

}  // namespace nmls

template <typename Lyt>
class write_nmls_layout_impl
{
  public:
    write_nmls_layout_impl(const Lyt& src, const uint32_t critical_path_length, std::ostream& s,
                           write_nmls_layout_params p) :
            lyt{src},
            bb{lyt},
            sorted_pi_list{sorted_pis()},
            sorted_po_list{sorted_pos()},
            num_magnets{nmlsim_magnet_count(lyt)},
            critical_path_length{critical_path_length},
            os{s},
            ps{std::move(p)}
    {}

    void run()
    {
        if (!has_border_io_pins())
        {
            throw std::invalid_argument(
                "the iNML layout does not fulfill all requirements to be written as a NMLS file because it does not "
                "have designated I/O pins or they are not routed to the layout's borders");
        }

        write_header();
        write_dimensions();
        write_phases_section();
        write_zones_section();
        write_magnets_section();
    }

  private:
    const Lyt& lyt;

    const bounding_box_2d<Lyt> bb;

    std::vector<cell<Lyt>> sorted_pi_list, sorted_po_list;

    const uint64_t num_magnets;

    const uint32_t critical_path_length;

    std::ostream& os;

    const write_nmls_layout_params ps;

    [[nodiscard]] std::vector<cell<Lyt>> sorted_pis() const noexcept
    {
        std::vector<cell<Lyt>> pi_list{};
        lyt.foreach_pi([&pi_list](const auto& pi) { pi_list.push_back(pi); });
        std::sort(pi_list.begin(), pi_list.end(), [](const auto& c1, const auto& c2) { return c1.y < c2.y; });

        return pi_list;
    }

    [[nodiscard]] std::vector<cell<Lyt>> sorted_pos() const noexcept
    {
        std::vector<cell<Lyt>> po_list{};
        lyt.foreach_po([&po_list](const auto& po) { po_list.push_back(po); });
        std::sort(po_list.begin(), po_list.end(), [](const auto& c1, const auto& c2) { return c1.y < c2.y; });

        return po_list;
    }

    [[nodiscard]] auto bb_x(const cell<Lyt>& c) const noexcept
    {
        return static_cast<decltype(c.x)>(c.x - bb.get_min().x);
    }

    [[nodiscard]] auto bb_y(const cell<Lyt>& c) const noexcept
    {
        return static_cast<decltype(c.y)>(c.y - bb.get_min().y);
    }

    [[nodiscard]] bool has_border_io_pins() const noexcept
    {
        auto all_border_pins = true;
        // check PI border cells
        lyt.foreach_pi(
            [this, &all_border_pins](const auto& pi)
            {
                if (bb_x(pi) != 0)
                {
                    all_border_pins = false;
                    return false;  // break iteration
                }
                return true;  // keep iterating
            });
        // check PO border cells
        lyt.foreach_po(
            [this, &all_border_pins](const auto& po)
            {
                if (bb_x(po) != lyt.x())
                {
                    all_border_pins = false;
                    return false;  // break iteration
                }
                return true;  // keep iterating
            });

        return all_border_pins;
    }

    [[nodiscard]] std::vector<std::string> get_pin_data() const
    {
        std::vector<std::string> pin_data{};

        auto store_pin_data = [this, &pin_data](const auto& io)
        { pin_data.push_back(fmt::format("{}{}{}{}", lyt.get_cell_name(io), bb_x(io), bb_y(io), io.z)); };

        for (const auto& pi : sorted_pi_list)
        {
            store_pin_data(pi);
        }
        for (const auto& po : sorted_po_list)
        {
            store_pin_data(po);
        }
        std::sort(pin_data.begin(), pin_data.end());

        return pin_data;
    }

    [[nodiscard]] std::string generate_layout_id_hash() const
    {
        std::stringstream ss{};

        ss << lyt.get_layout_name() << nmls::LIBRARY_NAME << tech_impl_name<technology<Lyt>> << num_magnets
           << bb.get_x_size() << bb.get_y_size();

        const auto pin_data = get_pin_data();
        std::for_each(pin_data.cbegin(), pin_data.cend(),
                      [&ss](auto&& pdata) { ss << std::forward<decltype(pdata)>(pdata); });

        const auto hash_fragment = std::hash<std::string>()(ss.str());

        return fmt::format("{0:<020}13{0:<020}37{0:<020}", hash_fragment);
    }

    [[nodiscard]] std::tuple<std::string, std::string> get_cell_cuts(const auto& magnet_type) const
    {
        switch (magnet_type)
        {
            case nmlib_inml_technology::cell_type::EMPTY:
            case nmlib_inml_technology::cell_type::LITTLE:
            case nmlib_inml_technology::cell_type::NORMAL:
            case nmlib_inml_technology::cell_type::BIG:
            case nmlib_inml_technology::cell_type::LITTLE_INPUT:
            case nmlib_inml_technology::cell_type::INPUT:
            case nmlib_inml_technology::cell_type::BIG_INPUT:
            case nmlib_inml_technology::cell_type::LITTLE_OUTPUT:
            case nmlib_inml_technology::cell_type::BIG_OUTPUT:
            case nmlib_inml_technology::cell_type::OUTPUT: return {"0", "0"}; break;

            case nmlib_inml_technology::cell_type::LITTLE_SLANTED_EDGE_LEFT_UP_MAGNET:
            case nmlib_inml_technology::cell_type::SLANTED_EDGE_LEFT_UP_MAGNET:
            case nmlib_inml_technology::cell_type::BIG_SLANTED_EDGE_LEFT_UP_MAGNET: return {"-15", "0"}; break;

            case nmlib_inml_technology::cell_type::LITTLE_SLANTED_EDGE_RIGHT_UP_MAGNET:
            case nmlib_inml_technology::cell_type::SLANTED_EDGE_RIGHT_UP_MAGNET:
            case nmlib_inml_technology::cell_type::BIG_SLANTED_EDGE_RIGHT_UP_MAGNET: return {"15", "0"}; break;

            case nmlib_inml_technology::cell_type::LITTLE_SLANTED_EDGE_LEFT_DOWN_MAGNET:
            case nmlib_inml_technology::cell_type::SLANTED_EDGE_LEFT_DOWN_MAGNET:
            case nmlib_inml_technology::cell_type::BIG_SLANTED_EDGE_LEFT_DOWN_MAGNET: return {"0", "-15"}; break;

            case nmlib_inml_technology::cell_type::LITTLE_SLANTED_EDGE_RIGHT_DOWN_MAGNET:
            case nmlib_inml_technology::cell_type::SLANTED_EDGE_RIGHT_DOWN_MAGNET:
            case nmlib_inml_technology::cell_type::BIG_SLANTED_EDGE_RIGHT_DOWN_MAGNET: return {"0", "15"}; break;

            case nmlib_inml_technology::cell_type::LITTLE_SLANTED_EDGE_LEFT_UP_AND_DOWN_MAGNET:
            case nmlib_inml_technology::cell_type::SLANTED_EDGE_LEFT_UP_AND_DOWN_MAGNET:
            case nmlib_inml_technology::cell_type::BIG_SLANTED_EDGE_LEFT_UP_AND_DOWN_MAGNET:
                return {"-15", "-15"};
                break;

            case nmlib_inml_technology::cell_type::LITTLE_SLANTED_EDGE_RIGHT_UP_AND_DOWN_MAGNET:
            case nmlib_inml_technology::cell_type::SLANTED_EDGE_RIGHT_UP_AND_DOWN_MAGNET:
            case nmlib_inml_technology::cell_type::BIG_SLANTED_EDGE_RIGHT_UP_AND_DOWN_MAGNET:
                return {"15", "15"};
                break;

            case nmlib_inml_technology::cell_type::INVERTER_MAGNET: return {"0", "0"}; break;
            case nmlib_inml_technology::cell_type::CROSSWIRE_MAGNET: return {"0", "0"}; break;
            case nmlib_inml_technology::cell_type::FANOUT_COUPLER_MAGNET: return {"0", "0"}; break;
            default: return {"0", "0"}; break;
        }
    }

    [[nodiscard]] std::string get_magnet_type(const auto& cell) const
    {
        return lyt.is_pi(cell) ? "input" : lyt.is_po(cell) ? "output" : "regular";
    }

    [[nodiscard]] std::string get_cell_color(const auto& clock_zone) const
    {
        if (clock_zone == 0)
        {
            return "-16777216";
        }

        if (clock_zone == 1)
        {
            return "-30208";
        }

        if (clock_zone == 2)
        {
            return "-14757932";
        }

        return "-11206444";
    }

    [[nodiscard]] std::string get_cell_specs_str(const auto& cell, auto& idx) const
    {
        // Magnet_{ID};magnet_type;clock_zone;magnetization_vector;fixed_magnetization;width;height;thickness;top_cut;bottom_cut;pos_x,pos_y;color_code
        std::string                             cell_str{""};
        std::string                             magnet_type         = get_magnet_type(cell);
        std::string                             fixed_magnetization = magnet_type == "input" ? "true" : "false";
        const auto&                             clock_zone          = lyt.get_custom_clock_number(cell);
        const nmlib_inml_technology::cell_type& ctype               = lyt.get_cell_type(cell);

        const uint64_t cell_height = nmlib_inml_technology::get_cell_height(ctype);

        const auto [top_cut, bottom_cut] = get_cell_cuts(ctype);
        const uint32_t cell_abs_x =
            static_cast<float>(nmlib_inml_technology::LAYOUT_BASE_X +
                               (cell.x * (nmlib_inml_technology::CELL_HSPACE + nmlib_inml_technology::CELL_WIDTH)));
        const uint32_t cell_abs_y = static_cast<float>(
            nmlib_inml_technology::LAYOUT_BASE_Y +
            (cell.y * (nmlib_inml_technology::CELL_VSPACE + nmlib_inml_technology::BIG_CELL_HEIGHT)));
        std::string magnet_str = fmt::format(
            nmls::MAGNET_SPECS, idx, magnet_type, clock_zone, nmlib_inml_technology::DEFAULT_MAG, fixed_magnetization,
            nmlib_inml_technology::CELL_WIDTH, cell_height, nmlib_inml_technology::CELL_THICKNESS, top_cut, bottom_cut,
            fmt::format("{}.0,{}.0", cell_abs_x, cell_abs_y), get_cell_color(clock_zone));
        cell_str += fmt::format(magnet_str);
        idx++;
        return cell_str;
    }

    [[nodiscard]] std::string get_magnet_specs_str(const auto& pos) const
    {
        // Magnet_{ID};magnet_type;clock_zone;magnetization_vector;fixed_magnetization;width;height;thickness;top_cut;bottom_cut;pos_x,pos_y;color_code
        std::string                             cell_str{""};
        const nmlib_inml_technology::cell_type& ctype               = lyt.get_cell_type(pos);
        const uint64_t                          cell_height         = nmlib_inml_technology::get_cell_height(ctype);
        std::string                             fixed_magnetization = ctype == 'i' ? "true" : "false";
        // const auto&                             clock_zone  = lyt.get_custom_clock_number(pos);

        const auto [top_cut, bottom_cut] = get_cell_cuts(ctype);

        const uint32_t cell_abs_x =
            static_cast<float>(nmlib_inml_technology::LAYOUT_BASE_X +
                               (pos.first * (nmlib_inml_technology::CELL_HSPACE + nmlib_inml_technology::CELL_WIDTH)));

        const uint32_t cell_abs_y = static_cast<float>(
            nmlib_inml_technology::LAYOUT_BASE_Y +
            (pos.second * (nmlib_inml_technology::CELL_VSPACE + nmlib_inml_technology::BIG_CELL_HEIGHT)));

        LOG(fmt::format("\n  ## Magnet pos {}", pos));
        LOG(fmt::format("Magnet type {}", ctype));
        LOG(fmt::format("Magnet: height {}, top_cut {}, bottom_cut {}", cell_height, top_cut, bottom_cut));
        LOG(fmt::format("magnet_abs_x: {}, magnet_abs_y: {}", cell_abs_x, cell_abs_y));

        // std::string magnet_str = fmt::format(
        //     nmls::MAGNET_SPECS, idx, magnet_type, clock_zone, nmlib_inml_technology::DEFAULT_MAG,
        //     fixed_magnetization, nmlib_inml_technology::CELL_WIDTH, cell_height,
        //     nmlib_inml_technology::CELL_THICKNESS, top_cut, bottom_cut, fmt::format("{}.0,{}.0", cell_abs_x,
        //     cell_abs_y), get_cell_color(clock_zone));
        // cell_str += fmt::format(magnet_str);
        return "";
    }

    void write_header()
    {
        // To calculate the simulation time we need to calculate the critical path length and multiple it
        // by the quantity of clock zones (4) and the clock zone delay into nanoseconds (5)
        auto sim_time = 40 + (critical_path_length * 20);
        os << fmt::format(nmls::NMLS_HEADER, nmls::TECHNOLOGY, nmls::SIMULATION_MODE, nmls::LLG_ENGINE_METHOD,
                          nmls::SIMULATION_EXECUTIONS_QNTD, nmls::REPORT_STEP, nmls::ALPHA,
                          nmls::SATURATION_MAGNETIZATION, nmls::TEMPERATURE, nmls::TIME_STEP, sim_time,
                          nmls::SPIN_ANGLE, nmls::SPIN_DIFUSION_LENGTH, nmls::THICKNESS, nmls::NEIGHBORHOOD_RATIO);
    }

    void write_dimensions()
    {
        auto LAYOUT_WIDTH = 3 * (nmlib_inml_technology::CELL_WIDTH + nmlib_inml_technology::CELL_HSPACE) +
                            (nmlib_inml_technology::CELL_WIDTH + nmlib_inml_technology::CELL_HSPACE) * bb.get_max().x;

        auto LAYOUT_HEIGHT =
            3 * (nmlib_inml_technology::BIG_CELL_HEIGHT + nmlib_inml_technology::CELL_VSPACE) +
            (nmlib_inml_technology::BIG_CELL_HEIGHT + nmlib_inml_technology::CELL_VSPACE) * bb.get_max().y;

        os << "\n"
           << fmt::format(nmls::DIMENSIONS, LAYOUT_WIDTH, LAYOUT_HEIGHT, "10", "10",
                          (nmlib_inml_technology::CELL_HSPACE + nmlib_inml_technology::CELL_WIDTH),
                          (nmlib_inml_technology::CELL_VSPACE + nmlib_inml_technology::BIG_CELL_HEIGHT));
    }

    void write_phases_section()
    {
        os << "\n" << nmls::PHASES_SECTION;
    }

    void write_zones_section()
    {
        os << "\n" << nmls::ZONES_SECTION;
    }

    void write_magnets_section()
    {
        os << "\n" << fmt::format(nmls::MAGNETS_SECTION_HEADER, lyt.num_cells());
        std::string magnet_lines{""};
        size_t      idx{0};
        size_t      magnets_qnt{0};
        LOG(fmt::format("\n WRITE MAGNETS SECTION \n"));
        // This function replaces entire cells, it is necessary to iterate over each magnet
        // and calculate its position
        for (decltype(lyt.y()) y_pos = 0; y_pos <= lyt.y(); ++y_pos)
        {
            for (decltype(lyt.x()) x_pos = 0; x_pos <= lyt.x(); ++x_pos)
            {
                if (!lyt.is_empty_cell({x_pos, y_pos, 0}))
                {
                    cell<Lyt> c{x_pos, y_pos};
                    ++magnets_qnt;
                    get_magnet_specs_str(c);
                }
            }
        }
        LOG(fmt::format("\n magnets_qnt {}", magnets_qnt));
        lyt.foreach_cell([this, &magnet_lines, &idx](const auto& cell)
                         { magnet_lines += get_cell_specs_str(cell, idx) + "\n"; });
        os << "\n" << magnet_lines;
    }
};

}  // namespace detail

/**
 * Writes a cell-level iNML layout to a nmls file that is used by ToPoliNano & MagCAD (https://topolinano.polito.it/),
 * an EDA tool and a physical simulator for the iNML technology platform.
 *
 * This overload uses an output stream to write into.
 *
 * @tparam Lyt Cell-level iNML layout type.
 * @param lyt The layout to be written.
 * @param os The output stream to write into.
 * @param ps Parameters.
 */
template <typename Lyt>
void write_nmls_layout(const Lyt& lyt, const uint32_t critical_path_length, std::ostream& os,
                       write_nmls_layout_params ps = {})
{
    static_assert(is_cell_level_layout_v<Lyt>, "Lyt is not a cell-level layout");
    static_assert(has_nmlib_inml_technology_v<Lyt>, "Lyt must be an iNML layout");

    detail::write_nmls_layout_impl p{lyt, critical_path_length, os, ps};

    p.run();
}
/**
 * Writes a cell-level iNML layout to a nmls file that is used by ToPoliNano & MagCAD (https://topolinano.polito.it/),
 * an EDA tool and a physical simulator for the iNML technology platform.
 *
 * This overload uses a file name to create and write into.
 *
 * @tparam Lyt Cell-level iNML layout type.
 * @param lyt The layout to be written.
 * @param filename The file name to create and write into. Should preferably use the `.nmls` extension.
 * @param ps Parameters.
 */
template <typename Lyt>
void write_nmls_layout(const Lyt& lyt, const uint32_t critical_path_length, const std::string_view& filename,
                       write_nmls_layout_params ps = {})
{
    std::ofstream os{filename.data(), std::ofstream::out};

    if (!os.is_open())
    {
        throw std::ofstream::failure("could not open file");
    }

    ps.filename = filename;

    write_nmls_layout(lyt, critical_path_length, os, ps);
    os.close();
}

}  // namespace fiction

#endif  // FICTION_WRITE_QCC_LAYOUT_HPP
