^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package lanelet2_routing
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

Forthcoming
-----------
* Fix clang-tidy warnings
* Contributors: Fabian Poggenhans

1.0.0 (2020-03-03)
------------------
* Bump version to 1.0
* Routing: Add shortest path search based on areas
* Fix default values for lane changes in RoutingGraph
* RoutingGraph and Route now use the new LaneletSubmap to store the lanelets they are using
  their member functions laneletMap() and passableMap() are now deprecated and should be replaced by laneletSubmap() and passableSubmap() respectively. These functions have less overhead and are less likely to be misinterpreted as 'maps containing only the lanelets you need'
* Edges with nonfinite costs are no longer added to the graph to avoid overflows.
  If a cost function returns infinite costs, no edge will be added and the connection will not be available to the routing graph
* Introduce proper namespacing for internal objects
* Update documentation
* Routing graph and route object now support queries with a custom search function
  Routing graph and route object now both have a function forEachSuccessor (and more) for doing more advanced queries. Added doc tests and python bindings
* Update the shortest path algorithm to use the new dijktra search
* Extended and simplified the reachablePath/Set functions
  by using boost graphs implementation of dijkstras shortest paths
* Refactored the internal representation of the route. Cleaned up headers that are only supposed to be used internally
* Complete rewrite of the route builder using boost graph
* Removed the "diverging" and "merging" classification from the routing
  graph and update the doc
  They are now all represented with the "succeeding" relation. Information
  about merging or diverging can now be obtained simply by querying the
  number of following/preceding lanelets.
  As a consequence, the route object no longer caches queried "lanes". The
  responsible functions are now const.
* Fix compiler errors with gcc 5
* Fix image paths in routing doc
* Initial commit
* Contributors: Fabian Poggenhans, Johannes Janosovits, Maximilian Naumann
