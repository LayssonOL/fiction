//
// Created by Jan Drewniok on 24.11.22.
//

#ifndef FICTION_SIDB_SIMULATION_PARAMETERS_HPP
#define FICTION_SIDB_SIMULATION_PARAMETERS_HPP

#include "fiction/technology/physical_constants.hpp"

#include <cassert>
#include <cstdint>

namespace fiction
{

/**
 * This struct collects all physical parameters for physical SiDB simulations. It can be useful to adjust them,
 * especially when experiments create new insights. However, the default values are commonly used.
 */
struct sidb_simulation_parameters
{
    /**
     * Default constructor.
     *
     * @param base_number simulation can be conducted with 2 and 3 charge states. 2 = (Negative, Neutral), 3 =
     * (Negative, Neutral, Positive).
     * @param mu (µ-) is the energy transition level (0/-) in eV.
     * @param relative_permittivity it describes the electric field reduction due to polarization.
     * @param screening_distance also known as "Thomas-Fermi screening" and it describes the electric field screening
     * due to free charges in nm.
     * @param a lattice constant in Å (Ångström).
     * @param b lattice constant in Å.
     * @param c lattice constant in Å.
     */
    constexpr explicit sidb_simulation_parameters(const uint8_t base_number = 3, const double mu = -0.32,
                                                  const double relative_permittivity = 5.6,
                                                  const double screening_distance = 5.0, const double a = 3.84,
                                                  const double b = 7.68, const double c = 2.25) :
            lat_a{a},
            lat_b{b},
            lat_c{c},
            epsilon_r{relative_permittivity},
            lambda_tf{screening_distance},
            mu_minus{mu},
            base{base_number}

    {
        assert((base == 2 || base == 3) && "base must be 2 or 3");
    }

    /**
     * lat_a is the lattice vector in x-direction (unit: Å).
     */
    double lat_a;
    /**
     * lat_b is the lattice vector in y-direction (unit: Å).
     */
    double lat_b;
    /**
     * lat_c is the dimer pair separation (unit: Å).
     */
    double lat_c;
    /**
     * epsilon_r is the electric permittivity. It is a material specific number (unit-less).
     */
    double epsilon_r;
    /**
     * lambda_tf is the Thomas-Fermi screening distance (unit: nm).
     */
    double lambda_tf;
    /**
     * mu_minus (µ-) is the energy transition level (0/-) (unit: eV).
     */
    double mu_minus;
    /**
     * base can be either 2 or 3 and describes the assumed number of charge states of one SiDB.
     * It often makes sense to assume only negatively and neutrally charged SiDBs.
     */
    uint8_t base;
    /**
     * k is the Coulomb constant K_E divided by epsilon_r (unit: \f$ N \cdot m^{2}
     * \cdot C^{-2} \f$).
     */
    [[nodiscard]] double k() const noexcept
    {
        return physical_constants::K_E / epsilon_r;
    }
    /**
     * mu_plus (µ+) is the energy transition level (+/0) (unit: eV).
     */
    [[nodiscard]] double mu_plus() const noexcept
    {
        return mu_minus - 0.59;
    };
};

}  // namespace fiction

#endif  // FICTION_SIDB_SIMULATION_PARAMETERS_HPP
