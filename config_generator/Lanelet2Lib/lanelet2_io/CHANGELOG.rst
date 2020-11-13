^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package lanelet2_io
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Improve warning if wrong decimal symbol is set, also report it when loading
* Contributors: Fabian Poggenhans

1.0.0 (2020-03-03)
------------------
* Bump version to 1.0
* Add a new class 'LaneletSubmap' that only contains parts of the map and is faster to construct
* IO: Implement warning for cases where the decimal point is overridden by a different locale
  resolves MRT/released/lanelet2#91
* Fix loading of polygons that have been written without an area tag
  resolves MRT/released/lanelet2#113
* Refactored osm parser so that parsed roles in relations keep their
  positions
* Improve c++17 support, supress warnings and clang-tidy 7 issues
* IO now complains when loading georeferenced maps with a default origin (resolves #71)
* Initial commit
* Contributors: Fabian Poggenhans, Maximilian Naumann
