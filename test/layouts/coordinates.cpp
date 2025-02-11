//
// Created by marcel on 15.09.21.
//

#include <catch2/catch_template_test_macros.hpp>
#include <catch2/catch_test_macros.hpp>

#include <fiction/layouts/cartesian_layout.hpp>
#include <fiction/layouts/coordinates.hpp>
#include <fiction/layouts/hexagonal_layout.hpp>
#include <fiction/traits.hpp>

#include <fmt/format.h>

#include <map>
#include <sstream>
#include <vector>

using namespace fiction;

#pragma GCC diagnostic push
#if defined(__GNUC__)
#pragma GCC diagnostic ignored "-Wuseless-cast"
#endif
#pragma GCC diagnostic ignored "-Wconversion"

TEST_CASE("Unsigned offset coordinates", "[coordinates]")
{
    using coordinate = offset::ucoord_t;

    auto td = coordinate{};
    CHECK(td.is_dead());

    auto t0 = coordinate{0, 0, 0};
    CHECK(!t0.is_dead());

    CHECK(t0 != td);

    auto t1 = coordinate{1, 2, 0};
    auto t2 = coordinate{1, 2};

    CHECK(t0 < t1);
    CHECK(t1 > t0);
    CHECK(t1 >= t0);
    CHECK(t0 <= t1);
    CHECK(t1 == t2);
    CHECK(t2 == t1);

    t1.z += uint64_t{4ul};

    CHECK(t1 == t2);

    t1.y += uint64_t{2147483648ul};

    CHECK(t1 == t2);

    t1.x += uint64_t{2147483648ul};

    CHECK(t1 == t2);

    t1.x++;

    CHECK(t1 != t2);
    CHECK(t1 > t2);
    CHECK(t1 >= t2);
    CHECK(t2 < t1);
    CHECK(t2 <= t1);

    auto t3 = coordinate{0, 0, 1};

    CHECK(t1 < t3);
    CHECK(t2 < t3);

    const std::map<uint64_t, coordinate> coordinate_repr{
        {0x8000000000000000, coordinate{}},        {0x0000000000000000, coordinate{0, 0, 0}},
        {0x4000000000000000, coordinate{0, 0, 1}}, {0x4000000080000001, coordinate{1, 1, 1}},
        {0x0000000000000002, coordinate{2, 0, 0}}, {0x3fffffffffffffff, coordinate{2147483647, 2147483647, 0}}};

    for (auto [repr, coord] : coordinate_repr)
    {
        CHECK(static_cast<coordinate>(repr) == coord);
        CHECK(repr == static_cast<uint64_t>(coord));
        CHECK(coordinate{repr} == coord);
        CHECK(coordinate{coord} == coord);
        CHECK(coordinate{static_cast<uint64_t>(coord)} == coord);
    }

    std::ostringstream os{};
    os << coordinate{3, 2, 1};
    CHECK(os.str() == "(3,2,1)");
}

TEST_CASE("SiQAD coordinate conversion", "[coordinates]")
{
    using coordinate         = siqad::coord_t;
    using coordinate_fiction = cube::coord_t;

    auto t = coordinate{};
    CHECK(t.is_dead());
    auto fiction_d = siqad::to_fiction_coord<coordinate_fiction>(t);
    CHECK(fiction_d.is_dead());

    auto t0        = coordinate{0, 0, 0};
    auto fiction_0 = siqad::to_fiction_coord<coordinate_fiction>(t0);
    CHECK(!fiction_0.is_dead());

    auto t1         = coordinate{1, 3, 1};
    auto t1_fiction = siqad::to_fiction_coord<coordinate_fiction>(t1);
    CHECK(t1_fiction.x == t1.x);
    CHECK(t1_fiction.y == 7);
    auto t2 = siqad::to_siqad_coord<coordinate_fiction>(t1_fiction);
    CHECK(t1 == t2);

    auto t3_fiction = coordinate_fiction{1, 2};
    auto t3_siqad   = siqad::to_siqad_coord<coordinate_fiction>(t3_fiction);
    CHECK(t3_siqad.x == t3_fiction.x);
    CHECK(t3_siqad.y == 1);
    CHECK(t3_siqad.z == 0);

    auto t4_fiction = coordinate_fiction{-1, -2};
    auto t4_siqad   = siqad::to_siqad_coord<coordinate_fiction>(t4_fiction);
    CHECK(t4_siqad.x == t4_fiction.x);
    CHECK(t4_siqad.y == -1);
    CHECK(t4_siqad.z == 0);

    auto t5_siqad   = coordinate{-1, -2, 1};
    auto t5_fiction = siqad::to_fiction_coord<coordinate_fiction>(t5_siqad);
    CHECK(t5_fiction.x == -1);
    CHECK(t5_fiction.y == -3);
}

TEMPLATE_TEST_CASE("Coordinate iteration", "[coordinates]", offset::ucoord_t, cube::coord_t, siqad::coord_t)
{
    using lyt_t = cartesian_layout<TestType>;

    std::vector<TestType> coord_vector{};
    coord_vector.reserve(7);

    const lyt_t lyt{{1, 1, 1}};

    const auto fill_coord_vector = [&v = coord_vector](const auto& c) { v.emplace_back(c); };

    SECTION("With bounds")
    {
        lyt.foreach_coordinate(fill_coord_vector, {1, 0, 0}, {1, 1, 1});

        CHECK(coord_vector.size() == 6);

        CHECK(coord_vector[0] == TestType{1, 0, 0});

        if constexpr (std::is_same_v<TestType, siqad::coord_t>)
        {
            CHECK(coord_vector[1] == TestType{0, 0, 1});
            CHECK(coord_vector[2] == TestType{1, 0, 1});
            CHECK(coord_vector[3] == TestType{0, 1, 0});
            CHECK(coord_vector[4] == TestType{1, 1, 0});
        }
        else
        {
            CHECK(coord_vector[1] == TestType{0, 1, 0});
            CHECK(coord_vector[2] == TestType{1, 1, 0});
            CHECK(coord_vector[3] == TestType{0, 0, 1});
            CHECK(coord_vector[4] == TestType{1, 0, 1});
        }

        CHECK(coord_vector[5] == TestType{0, 1, 1});
    }
    SECTION("Without bounds")
    {
        coord_vector.clear();
        coord_vector.reserve(8);

        lyt.foreach_coordinate(fill_coord_vector);

        CHECK(coord_vector.size() == 8);

        CHECK(coord_vector.front().str() == fmt::format("{}", TestType{0, 0, 0}));
        CHECK(coord_vector.back().str() == fmt::format("{}", TestType{1, 1, 1}));
    }
    SECTION("With non-dead out of bounds end bound")
    {
        std::vector<TestType> good_bound_coord_vector{};

        const auto fill_good_bound_coord_vector = [&v = good_bound_coord_vector](const auto& c) { v.emplace_back(c); };

        const auto test_bounds_equal = [&](const auto& c_lyt, const TestType& bad_bound, const TestType& good_bound)
        {
            coord_vector.clear();
            coord_vector.reserve(8);

            good_bound_coord_vector.clear();
            good_bound_coord_vector.reserve(8);

            c_lyt.foreach_coordinate(fill_coord_vector, {}, bad_bound);
            c_lyt.foreach_coordinate(fill_good_bound_coord_vector, {}, good_bound);

            CHECK(coord_vector.size() == good_bound_coord_vector.size());
            CHECK(coord_vector.back() == good_bound_coord_vector.back());
        };

        test_bounds_equal(lyt, {9, 9, 9}, {});
        test_bounds_equal(lyt, {0, 2, 1}, {});

        if constexpr (std::is_same_v<TestType, cube::coord_t>)
        {
            test_bounds_equal(lyt, {0, 0, 9}, {});
        }

        if constexpr (std::is_same_v<TestType, siqad::coord_t>)
        {
            test_bounds_equal(lyt, {2, 0, 0}, {0, 0, 1});
            test_bounds_equal(lyt, {2, 0, 1}, {0, 1, 0});
            test_bounds_equal(lyt, {2, 1, 0}, {0, 1, 1});
            test_bounds_equal(lyt, {0, 2, 0}, {});

            using h_lyt = hexagonal_layout<TestType, even_row_hex>;

            test_bounds_equal(h_lyt{aspect_ratio<h_lyt>{0, 1, 0}}, {0, 1, 1}, {});
        }
        else
        {
            test_bounds_equal(lyt, {2, 0, 0}, {0, 1, 0});
            test_bounds_equal(lyt, {2, 0, 1}, {0, 1, 1});
            test_bounds_equal(lyt, {2, 1, 0}, {0, 0, 1});
            test_bounds_equal(lyt, {0, 2, 0}, {0, 0, 1});

            test_bounds_equal(lyt_t{aspect_ratio<lyt_t>{0, 1, 0}}, {0, 1, 1}, {});
        }

        test_bounds_equal(lyt_t{aspect_ratio<lyt_t>{0, 0, 0}}, {9, 9, 9}, {});
    }
}

TEST_CASE("Computing area and volume of offset coordinates", "[coordinates]")
{
    CHECK(area(offset::ucoord_t{1, 1, 1}) == 4);
    CHECK(volume(offset::ucoord_t{1, 1, 1}) == 8);
}

TEST_CASE("Computing area and volume of cube coordinates", "[coordinates]")
{
    CHECK(area(cube::coord_t{1, 1, 1}) == 4);
    CHECK(area(cube::coord_t{-1, -1, -1}) == 4);

    CHECK(volume(cube::coord_t{-1, -1, -1}) == 8);
    CHECK(volume(cube::coord_t{1, 1, 1}) == 8);
}

TEST_CASE("Computing area and volume of SiQAD coordinates", "[coordinates]")
{
    CHECK(area(siqad::coord_t{1, 1, 1}) == 8);
    CHECK(area(siqad::coord_t{-1, -1, 1}) == 8);

    CHECK(volume(siqad::coord_t{1, 1, 1}) == 8);
    CHECK(volume(siqad::coord_t{-1, -1, 1}) == 8);
}

TEST_CASE("Addition / subtraction of SiQAD coordinates", "[coordinates]")
{
    using coord = siqad::coord_t;

    CHECK(coord{-4, 4, 1} + coord{1, -7, 1} == coord{-3, -2, 0});
    CHECK(coord{-4, 4, 1} + coord{1, -7, 0} == coord{-3, -3, 1});

    CHECK(coord{-4, 4, 0} - coord{1, -7, 1} == coord{-5, 10, 1});
    CHECK(coord{-4, 4, 1} - coord{1, -7, 1} == coord{-5, 11, 0});
}

TEST_CASE("Addition / subtraction of cube coordinates", "[coordinates]")
{
    using coord = cube::coord_t;

    CHECK(coord{-4, 4, -43} + coord{1, -7, 27} == coord{-3, -3, -16});
    CHECK(coord{-4, 4, 42} - coord{1, -7, 24} == coord{-5, 11, 18});
}

#if defined(__GNUC__)
#pragma GCC diagnostic pop
#endif
