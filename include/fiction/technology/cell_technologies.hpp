//
// Created by marcel on 23.06.21.
//

#ifndef FICTION_CELL_TECHNOLOGIES_HPP
#define FICTION_CELL_TECHNOLOGIES_HPP

#include <cstdint>

namespace fiction
{

/**
 * Quantum-dot Cellular Automata (QCA) technology implementation of the FCN concept.
 */
struct qca_technology
{
    /**
     * Possible types of QCA cells.
     */
    enum cell_type : uint8_t
    {
        /**
         * Symbol used for empty QCA cells.
         */
        EMPTY = ' ',
        /**
         * Symbol used for normal QCA cells.
         */
        NORMAL = 'x',
        /**
         * Symbol used for input QCA cells.
         */
        INPUT = 'i',
        /**
         * Symbol used for output QCA cells.
         */
        OUTPUT = 'o',
        /**
         * Symbol used for constant 0 input QCA cells.
         */
        CONST_0 = '0',
        /**
         * Symbol used for constant 1 input QCA cells.
         */
        CONST_1 = '1'
    };
    /**
     * Possible cell modes for QCA cells.
     */
    enum class cell_mode : uint8_t
    {
        NORMAL = 0u,
        ROTATED,
        VERTICAL,
        CROSSOVER
    };
    /**
     * Possible marks to be applied to a cell to change its type.
     */
    enum class cell_mark : uint8_t
    {
        EMPTY  = cell_type::EMPTY,
        INPUT  = cell_type::INPUT,
        OUTPUT = cell_type::OUTPUT
    };

    [[nodiscard]] static constexpr bool is_empty_cell(const cell_type& c) noexcept
    {
        return c == EMPTY;
    }

    [[nodiscard]] static constexpr bool is_normal_cell(const cell_type& c) noexcept
    {
        return c == NORMAL;
    }

    [[nodiscard]] static constexpr bool is_input_cell(const cell_type& c) noexcept
    {
        return c == INPUT;
    }

    [[nodiscard]] static constexpr bool is_output_cell(const cell_type& c) noexcept
    {
        return c == OUTPUT;
    }

    [[nodiscard]] static constexpr bool is_const_0_cell(const cell_type& c) noexcept
    {
        return c == CONST_0;
    }

    [[nodiscard]] static constexpr bool is_const_1_cell(const cell_type& c) noexcept
    {
        return c == CONST_1;
    }

    [[nodiscard]] static constexpr bool is_constant_cell(const cell_type& c) noexcept
    {
        return is_const_0_cell(c) || is_const_1_cell(c);
    }

    [[nodiscard]] static constexpr bool is_normal_cell_mode(const cell_mode& m) noexcept
    {
        return m == cell_mode::NORMAL;
    }

    [[nodiscard]] static constexpr bool is_rotated_cell_mode(const cell_mode& m) noexcept
    {
        return m == cell_mode::ROTATED;
    }

    [[nodiscard]] static constexpr bool is_vertical_cell_mode(const cell_mode& m) noexcept
    {
        return m == cell_mode::VERTICAL;
    }

    [[nodiscard]] static constexpr bool is_crossover_cell_mode(const cell_mode& m) noexcept
    {
        return m == cell_mode::CROSSOVER;
    }
    /**
     * Default width of a QCA cell in QCADesigner (https://waluslab.ece.ubc.ca/qcadesigner/).
     */
    static constexpr uint64_t CELL_WIDTH = 18ul;
    /**
     * Default height of a QCA cell in QCADesigner.
     */
    static constexpr uint64_t CELL_HEIGHT = 18ul;
    /**
     * Default horizontal spacing between two QCA cells in QCADesigner.
     */
    static constexpr uint64_t CELL_HSPACE = 2ul;
    /**
     * Default vertical spacing between two QCA cells in QCADesigner.
     */
    static constexpr uint64_t CELL_VSPACE = 2ul;
};

/**
 * in-plane Nanomagnet Logic (iNML) technology implementation of the FCN concept.
 */
struct inml_technology
{
    /**
     * Possible types of iNML cells.
     */
    enum cell_type : uint8_t
    {
        /**
         * Symbol used for empty iNML cells.
         */
        EMPTY = ' ',
        /**
         * Symbol used for normal iNML cells.
         */
        NORMAL = 'x',
        /**
         * Symbol used for input iNML cells.
         */
        INPUT = 'i',
        /**
         * Symbol used for output iNML cells.
         */
        OUTPUT = 'o',
        /**
         * Symbol used for upper slanted edge magnets.
         */
        SLANTED_EDGE_UP_MAGNET = 'u',
        /**
         * Symbol used for lower slanted edge magnets.
         */
        SLANTED_EDGE_DOWN_MAGNET = 'd',
        /**
         * Symbol used for inverter magnets.
         */
        INVERTER_MAGNET = 'n',
        /**
         * Symbol used for cross-wire magnets.
         */
        CROSSWIRE_MAGNET = 'c',
        /**
         * Symbol used for coupler (fan-out) magnets.
         */
        FANOUT_COUPLER_MAGNET = 'f'
    };

    /**
     * iNML cells do not have modes.
     */
    struct cell_mode
    {};
    /**
     * Possible marks to be applied to a cell to change its type.
     */
    enum class cell_mark : uint8_t
    {
        EMPTY  = cell_type::EMPTY,
        INPUT  = cell_type::INPUT,
        OUTPUT = cell_type::OUTPUT
    };

    [[nodiscard]] static constexpr bool is_empty_cell(const cell_type& c) noexcept
    {
        return c == EMPTY;
    }

    [[nodiscard]] static constexpr bool is_normal_cell(const cell_type& c) noexcept
    {
        return c == NORMAL;
    }

    [[nodiscard]] static constexpr bool is_input_cell(const cell_type& c) noexcept
    {
        return c == INPUT;
    }

    [[nodiscard]] static constexpr bool is_output_cell(const cell_type& c) noexcept
    {
        return c == OUTPUT;
    }

    [[nodiscard]] static constexpr bool is_slanted_edge_up_magnet(const cell_type& c) noexcept
    {
        return c == SLANTED_EDGE_UP_MAGNET;
    }

    [[nodiscard]] static constexpr bool is_slanted_edge_down_magnet(const cell_type& c) noexcept
    {
        return c == SLANTED_EDGE_DOWN_MAGNET;
    }

    [[nodiscard]] static constexpr bool is_slanted_edge_magnet(const cell_type& c) noexcept
    {
        return is_slanted_edge_up_magnet(c) || is_slanted_edge_down_magnet(c);
    }

    [[nodiscard]] static constexpr bool is_inverter_magnet(const cell_type& c) noexcept
    {
        return c == INVERTER_MAGNET;
    }

    [[nodiscard]] static constexpr bool is_crosswire_magnet(const cell_type& c) noexcept
    {
        return c == CROSSWIRE_MAGNET;
    }

    [[nodiscard]] static constexpr bool is_fanout_coupler_magnet(const cell_type& c) noexcept
    {
        return c == FANOUT_COUPLER_MAGNET;
    }

    [[nodiscard]] static constexpr bool is_normal_cell_mode([[maybe_unused]] const cell_mode& m) noexcept
    {
        return true;
    }

    /**
     * Default width of a iNML cell in NMLSim.
     */
    static constexpr uint64_t CELL_WIDTH = 50ul;
    /**
     * Default height of a iNML cell in NMLSim.
     */
    static constexpr uint64_t CELL_HEIGHT = 100ul;
    /**
     * Default horizontal spacing between two iNML cells in NMLSim.
     */
    static constexpr uint64_t CELL_HSPACE = 10ul;
    /**
     * Default vertical spacing between two iNML cells in NMLSim.
     */
    static constexpr uint64_t CELL_VSPACE = 25ul;
};

/**
 * in-plane Nanomagnet Logic (iNML) technology implementation of the FCN concept following the NMLib shape engineer
 * rules.
 */
struct nmlib_inml_technology
{
    /**
     * Possible types of iNML cells.
     */
    enum cell_type : uint8_t
    {
        /**
         * Symbol used for empty iNML cells.
         */
        EMPTY = ' ',
        /**
         * Symbol used for normal iNML cells.
         */
        LITTLE = 'l',
        NORMAL = 'n',
        BIG    = 'N',
        /**
         * Symbol used for input iNML cells.
         */
        LITTLE_INPUT = '1',
        INPUT        = 'i',
        BIG_INPUT    = 'I',
        /**
         * Symbol used for output iNML cells.
         */
        LITTLE_OUTPUT = '0',
        OUTPUT        = 'o',
        BIG_OUTPUT    = 'O',
        /**
         * Symbol used for upper right slanted edge magnets.
         */
        LITTLE_SLANTED_EDGE_RIGHT_UP_MAGNET = '2',
        SLANTED_EDGE_RIGHT_UP_MAGNET        = 'u',
        BIG_SLANTED_EDGE_RIGHT_UP_MAGNET    = 'U',
        /**
         * Symbol used for upper left slanted edge magnets.
         */
        LITTLE_SLANTED_EDGE_LEFT_UP_MAGNET = '3',
        SLANTED_EDGE_LEFT_UP_MAGNET        = 'y',
        BIG_SLANTED_EDGE_LEFT_UP_MAGNET    = 'Y',
        /**
         * Symbol used for left down slanted edge magnets.
         */
        LITTLE_SLANTED_EDGE_LEFT_DOWN_MAGNET = '4',
        SLANTED_EDGE_LEFT_DOWN_MAGNET        = 'p',
        BIG_SLANTED_EDGE_LEFT_DOWN_MAGNET    = 'P',
        /**
         * Symbol used for right down slanted edge magnets.
         */
        LITTLE_SLANTED_EDGE_RIGHT_DOWN_MAGNET = '5',
        SLANTED_EDGE_RIGHT_DOWN_MAGNET        = 'r',
        BIG_SLANTED_EDGE_RIGHT_DOWN_MAGNET    = 'R',
        /**
         * Symbol used for left up and down slanted edge magnets.
         */
        LITTLE_SLANTED_EDGE_LEFT_UP_AND_DOWN_MAGNET = '6',
        SLANTED_EDGE_LEFT_UP_AND_DOWN_MAGNET        = 'c',
        BIG_SLANTED_EDGE_LEFT_UP_AND_DOWN_MAGNET    = 'C',
        /**
         * Symbol used for lower slanted edge magnets.
         */
        LITTLE_SLANTED_EDGE_RIGHT_UP_AND_DOWN_MAGNET = '7',
        SLANTED_EDGE_RIGHT_UP_AND_DOWN_MAGNET        = 'd',
        BIG_SLANTED_EDGE_RIGHT_UP_AND_DOWN_MAGNET    = 'D',
        /**
         * Symbol used for inverter magnets.
         */
        INVERTER_MAGNET = 'v',
        /**
         * Symbol used for cross-wire magnets.
         */
        CROSSWIRE_MAGNET = 'x',
        /**
         * Symbol used for coupler (fan-out) magnets.
         */
        FANOUT_COUPLER_MAGNET = 'f'
    };

    /**
     * iNML cells do not have modes.
     */
    struct cell_mode
    {};
    /**
     * Possible marks to be applied to a cell to change its type.
     */
    enum class cell_mark : uint8_t
    {
        EMPTY  = cell_type::EMPTY,
        INPUT  = cell_type::INPUT,
        OUTPUT = cell_type::OUTPUT
    };

    [[nodiscard]] static constexpr bool is_empty_cell(const cell_type& c) noexcept
    {
        return c == EMPTY;
    }

    [[nodiscard]] static constexpr bool is_normal_cell(const cell_type& c) noexcept
    {
        return c == NORMAL;
    }

    [[nodiscard]] static constexpr bool is_input_cell(const cell_type& c) noexcept
    {
        return c == INPUT;
    }

    [[nodiscard]] static constexpr bool is_output_cell(const cell_type& c) noexcept
    {
        return c == OUTPUT;
    }

    [[nodiscard]] static constexpr bool is_slanted_edge_up_magnet(const cell_type& c) noexcept
    {
        return c == SLANTED_EDGE_LEFT_UP_MAGNET || c == SLANTED_EDGE_RIGHT_UP_MAGNET;
    }

    [[nodiscard]] static constexpr bool is_slanted_edge_down_magnet(const cell_type& c) noexcept
    {
        return c == SLANTED_EDGE_LEFT_DOWN_MAGNET || c == SLANTED_EDGE_RIGHT_DOWN_MAGNET;
    }

    [[nodiscard]] static constexpr bool is_slanted_edge_up_and_down_magnet(const cell_type& c) noexcept
    {
        return c == SLANTED_EDGE_LEFT_UP_AND_DOWN_MAGNET || c == SLANTED_EDGE_RIGHT_UP_AND_DOWN_MAGNET;
    }

    [[nodiscard]] static constexpr bool is_slanted_edge_magnet(const cell_type& c) noexcept
    {
        return is_slanted_edge_up_magnet(c) || is_slanted_edge_down_magnet(c) || is_slanted_edge_up_and_down_magnet(c);
    }

    [[nodiscard]] static constexpr bool is_inverter_magnet(const cell_type& c) noexcept
    {
        return c == INVERTER_MAGNET;
    }

    [[nodiscard]] static constexpr bool is_crosswire_magnet(const cell_type& c) noexcept
    {
        return c == CROSSWIRE_MAGNET;
    }

    [[nodiscard]] static constexpr bool is_fanout_coupler_magnet(const cell_type& c) noexcept
    {
        return c == FANOUT_COUPLER_MAGNET;
    }

    [[nodiscard]] static constexpr bool is_normal_cell_mode([[maybe_unused]] const cell_mode& m) noexcept
    {
        return true;
    }

    [[nodiscard]] static constexpr uint64_t get_cell_height([[maybe_unused]] const cell_type& c) noexcept
    {
        switch (c)
        {
            case cell_type::LITTLE:
            case cell_type::LITTLE_INPUT:
            case cell_type::LITTLE_OUTPUT:
            case cell_type::LITTLE_SLANTED_EDGE_RIGHT_UP_MAGNET:
            case cell_type::LITTLE_SLANTED_EDGE_RIGHT_DOWN_MAGNET:
            case cell_type::LITTLE_SLANTED_EDGE_LEFT_UP_MAGNET:
            case cell_type::LITTLE_SLANTED_EDGE_LEFT_DOWN_MAGNET:
            case cell_type::LITTLE_SLANTED_EDGE_LEFT_UP_AND_DOWN_MAGNET:
            case cell_type::LITTLE_SLANTED_EDGE_RIGHT_UP_AND_DOWN_MAGNET: return LITTLE_CELL_HEIGHT;

            case cell_type::BIG:
            case cell_type::BIG_INPUT:
            case cell_type::BIG_OUTPUT:
            case cell_type::BIG_SLANTED_EDGE_RIGHT_UP_MAGNET:
            case cell_type::BIG_SLANTED_EDGE_LEFT_UP_MAGNET:
            case cell_type::BIG_SLANTED_EDGE_LEFT_DOWN_MAGNET:
            case cell_type::BIG_SLANTED_EDGE_RIGHT_DOWN_MAGNET:
            case cell_type::BIG_SLANTED_EDGE_LEFT_UP_AND_DOWN_MAGNET:
            case cell_type::BIG_SLANTED_EDGE_RIGHT_UP_AND_DOWN_MAGNET: return BIG_CELL_HEIGHT;
            default: return CELL_HEIGHT;
        }
    }

    /**
     * Default width of a iNML cell in NMLib.
     */
    static constexpr uint64_t CELL_WIDTH = 50ul;
    /**
     * Default height of a iNML cell in NMLib.
     */
    static constexpr uint64_t CELL_HEIGHT = 130ul;
    /**
     * Height of a little iNML cell in NMLib.
     */
    static constexpr uint64_t LITTLE_CELL_HEIGHT = 110ul;
    /**
     * Height of a big iNML cell in NMLib.
     */
    static constexpr uint64_t BIG_CELL_HEIGHT = 150ul;
    /**
     * Default horizontal spacing between two iNML cells in NMLib.
     */
    static constexpr uint64_t CELL_HSPACE = 20ul;
    /**
     * Default vertical spacing between two iNML cells in NMLib.
     */
    static constexpr uint64_t CELL_VSPACE = 20ul;
    /**
     * Default cells thickness in NMLib.
     */
    static constexpr uint64_t CELL_THICKNESS = 15ul;
    /**
     * Default initial X and Y distances to put all magnets into visible layout space of NMLSim.
     */
    static constexpr uint64_t LAYOUT_BASE_X = 30ul;
    static constexpr uint64_t LAYOUT_BASE_Y = 80ul;
    /**
     * Default magnetization in NMLib.
     */
    static constexpr auto DEFAULT_MAG = "0.99,0.141,0,";
};

/**
 * Silicon Dangling Bond (SiDB) technology implementation of the FCN concept.
 */
struct sidb_technology
{
    /**
     * Possible types of SiDB cells.
     */
    enum cell_type : uint8_t
    { /**
       * Symbol used for empty SiDB cells.
       */
      EMPTY = ' ',
      /**
       * Symbol used for normal SiDB cells.
       */
      NORMAL = 'x',
      /**
       * Symbol used for input SiDB cells.
       */
      INPUT = 'i',
      /**
       * Symbol used for output SiDB cells.
       */
      OUTPUT = 'o'
    };

    /**
     * SiDB cells do not have modes.
     */
    struct cell_mode
    {};
    /**
     * Possible marks to be applied to a cell to change its type.
     */
    enum class cell_mark : uint8_t
    {
        EMPTY  = cell_type::EMPTY,
        INPUT  = cell_type::INPUT,
        OUTPUT = cell_type::OUTPUT
    };

    [[nodiscard]] static constexpr bool is_empty_cell(const cell_type& c) noexcept
    {
        return c == EMPTY;
    }

    [[nodiscard]] static constexpr bool is_normal_cell(const cell_type& c) noexcept
    {
        return c == NORMAL;
    }

    [[nodiscard]] static constexpr bool is_input_cell(const cell_type& c) noexcept
    {
        return c == INPUT;
    }

    [[nodiscard]] static constexpr bool is_output_cell(const cell_type& c) noexcept
    {
        return c == OUTPUT;
    }

    [[nodiscard]] static constexpr bool is_normal_cell_mode([[maybe_unused]] const cell_mode& m) noexcept
    {
        return true;
    }

    /**
     * Default width of a SiDB in SiQAD (https://github.com/siqad/siqad).
     * Dots are considered to be 0-dimensional entities for simulation purposes.
     */
    static constexpr double CELL_WIDTH = 0.0;
    /**
     * Default height of a SiDB in SiQAD.
     * Dots are considered to be 0-dimensional entities for simulation purposes.
     */
    static constexpr double CELL_HEIGHT = 0.0;
    /**
     * Default horizontal spacing between two SiDBs in SiQAD.
     */
    static constexpr double CELL_HSPACE = 0.384;
    /**
     * Default average vertical spacing between two SiDBs in SiQAD.
     * Depending on their lattice, they can be closer together or further apart.
     */
    static constexpr double CELL_VSPACE = 0.384;
};

}  // namespace fiction

#endif  // FICTION_CELL_TECHNOLOGIES_HPP
