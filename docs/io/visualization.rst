Visualization
-------------

Visualization of logic networks and FCN layouts.

Layout Printing
###############

**Header:** ``fiction/io/print_layout.hpp``

.. doxygenfunction:: fiction::print_gate_level_layout
.. doxygenfunction:: fiction::print_cell_level_layout
.. doxygenfunction:: fiction::print_sidb_layout
.. doxygenfunction:: fiction::print_layout

Graphviz (DOT) Drawers
######################

**Header:** ``fiction/io/dot_drawers.hpp``

.. doxygenclass:: fiction::technology_dot_drawer
.. doxygenclass:: fiction::color_view_drawer
.. doxygenclass:: fiction::edge_color_view_drawer
.. doxygenclass:: fiction::simple_gate_layout_tile_drawer
.. doxygenclass:: fiction::gate_layout_cartesian_drawer
.. doxygenclass:: fiction::gate_layout_shifted_cartesian_drawer
.. doxygenclass:: fiction::gate_layout_hexagonal_drawer

.. doxygenfunction:: fiction::write_dot_layout(const Lyt& lyt, std::ostream& os, const Drawer& drawer = {})
.. doxygenfunction:: fiction::write_dot_layout(const Lyt& lyt, const std::string_view& filename, const Drawer& drawer = {})

SVG Images
##########

**Header:** ``fiction/io/write_svg_layout.hpp``

.. doxygenstruct:: fiction::write_qca_layout_svg_params
   :members:

.. doxygenfunction:: fiction::write_qca_layout_svg(const Lyt& lyt, std::ostream& os, write_qca_layout_svg_params ps = {})
.. doxygenfunction:: fiction::write_qca_layout_svg(const Lyt& lyt, const std::string& filename, write_qca_layout_svg_params ps = {})

.. doxygenclass:: fiction::unsupported_cell_type_exception
