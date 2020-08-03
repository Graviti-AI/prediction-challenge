//
// Created by syzhang on 8/15/19.
//

// --------------------------------- Read me: --------------------------------- //
// This File is used to read the roundabout map information from "../maps" folder,
// creat the map information structure and do some geometry computations.
//
// Package needed: lanelet2.
//
// Instructions of structures:
// map_dot:
// a dot on the map, consists of a coordinate (x, y).
// map_branch:
// branch of a segmented reference, consists of one in branch and one out branch.
// number >= 0 means the entrance number or the exit number, while
// number = -1 means that direction is connected to one circle reference.
// TrajCir:
// circle references.
// TrajSeg:
// segmented references.
// Map_ref:
// A combination of the segmented references and the circle references. If the map
// information is needed in other place, this structure is the best to be used.
//
// Instructions of definitions:
// Relations between path:
// REF_IRRELEVANT:
// The two references are irrelevant.
// REF_CONFLICT:
// The two references are crossed by each other.


#ifndef LANELET2_MAPREADER_H
#define LANELET2_MAPREADER_H

#include <vector>
#include <lanelet2_core/primitives/LineString.h>
#include "../../alglib/stdafx.h"
#include "../../alglib/interpolation.h"
#include <lanelet2_core/primitives/Lanelet.h>
#include <lanelet2_io/Io.h>
#include <lanelet2_projection/UTM.h>
#include <lanelet2_routing/Route.h>
#include <lanelet2_routing/RoutingCost.h>
#include <lanelet2_routing/RoutingGraph.h>
#include <lanelet2_routing/RoutingGraphContainer.h>
#include <lanelet2_traffic_rules/TrafficRulesFactory.h>

using namespace std;

#define PI 3.1415926535897932384626433832795

// Relations between path
#define REF_IRRELEVANT 0
#define REF_CONFLICT 1
#define REF_SUCCEEDING 2
#define REF_IDENTICAL 3

// Type of reference
#define REF_TYPE_SEGMENTED 0
#define REF_TYPE_CIRCLE 1

struct map_dot{
    double x;
    double y;
    map_dot(){
        x = 0.0;
        y = 0.0;
    }
    map_dot(double xx, double yy){
        x = xx;
        y = yy;
    }
};

struct map_branch{
    int in;
    int out;
    map_branch(){
        in = -2;
        out = -2;
    }
    map_branch(int tmp_in, int tmp_out){
        in = tmp_in;
        out = tmp_out;
    }
};

struct TrajCir{
    int pathID = -1;                    // Real id, unchanged after finishing map reading process.
    int id = -1;                        // If this reference is part of a complete reference,
                                        // this is its serial number. Otherwise, this remains to be -1.
    map_dot center;                     // Center of the reference, usually the same.
    double radius;                      // Radius of the reference, usually the same.
    bool ifMerge;                       // Whether the reference is succeeded by a merging segmented reference.
    double mergeAngle;                  // If (ifMerge == true), this is the angle of the tangent line of the merging reference at the merging point.
                                        // Range: -PI ~ PI
    vector<map_branch> mergeBranch;     // If (ifMerge == true), this is the branch of the merging reference.
    bool ifDemerge;                     // Whether the reference is succeeding to a merging segmented reference.
    double demergeAngle;                // If (ifDemerge == true), this is the angle of the tangent line of the demerging reference at the demerging point.
                                        // range: -PI ~ PI
    vector<map_branch> demergeBranch;   // If (ifDemerge == true), this is the branch of the demerging reference.
    double demergeTerminateLengths;     // If (ifDemerge == true), this is the length of the demerging reference.
    vector<map_dot> path;               // A series of dots on the reference.
    lanelet::LineString2d lane;         // The lanelet expression of the reference.
    vector<int> succeedingRefID_seg;    // The pathID of the segmented references that the reference succeeds to.
    vector<int> succeedingRefID_cir;    // The pathID of the circle references that the reference succeeds to.
    vector<int> conflictRefID;          // The pathID of the references that the reference conflicts with. Usually there are only segmented reference.
};

struct TrajSeg{
    int pathID = -1;                    // Real id, unchanged after finishing map reading process.
    int id = -1;                        // If this reference is part of a complete reference,
                                        // this is its serial number. Otherwise, this remains to be -1.
    map_branch branch;                  // Branch of the reference.
    vector<map_dot> path;               // A series of dots on the reference.
    bool ifMerge;                       // Whether the reference is succeeding to or succeed by a circle reference.
    map_dot mergePoint;                 // If (ifMerge == true), this is the point where this reference is connected to a circle reference.
    double mergeAngle;                  // If (ifMerge == true), this is the angle of the tangent line of this reference at the merging point.
                                        // range: -PI ~ PI
    lanelet::LineString2d lane;         // The lanelet expression of the reference.
    vector<int> succeedingRefID_seg;    // The pathID of the segmented references that the reference succeeds to.
    vector<int> succeedingRefID_cir;    // The pathID of the circle references that the reference succeeds to.
    vector<int> conflictRefID_seg;      // The pathID of the segmented references that the reference conflicts with.
    vector<int> conflictRefID_cir;      // The pathID of the circle references that the reference conflicts with.
};

struct Map_ref{
    int pathID = -1;                    // Real id, unchanged after finishing map reading process. This is consisted of 2 kinds of references.
                                        // number 0~20 is the segmented references, 21~25 is the circle references.
    int id = -1;                        // If this reference is part of a complete reference,
                                        // this is its serial number. Otherwise, this remains to be -1.
    vector<map_dot> path;               // A series of dots on the reference.
    lanelet::LineString2d lane;         // The lanelet expression of the reference.
    vector<int> succeedingRefID;        // The pathID of the references that the reference succeeds to. Reference type must be Map_ref.
    vector<int> conflictRefID;          // The pathID of the circle references that the reference conflicts with. Reference type must be Map_ref.
    int refType;                        // Type of the reference. 0: Segmented reference, 1: Circle reference.
    bool operator ==(const Map_ref &other)const {
        return pathID == other.pathID;
    }
};

class MapReader{
public:
    vector<TrajCir> allTraj_cir;        // Record of all circle references.
    vector<TrajSeg> allTraj_seg;        // Record of all segmented references.
    vector<Map_ref> all_ref;            // Record of all references. 0 ~ 20: segmented references, 21 ~ 25: circle references.

    void main_readMap();                // The main function.

    // Functions to get the relation between two references. Relationships are defined in instructions.
    static int getRelation(const TrajSeg& thisTraj, const TrajSeg& otherTraj);
    static int getRelation(const TrajCir& thisTraj, const TrajSeg& otherTraj);
    static int getRelation(const TrajSeg& thisTraj, const TrajCir& otherTraj);
    static int getRelation(const TrajCir& thisTraj, const TrajCir& otherTraj);
    static int getRelation(const Map_ref& thisRef, const Map_ref& otherRef);

    // Geometry functions
    static bool getCrossPoint(vector<map_dot>& line1, vector<map_dot>& line2, vector<map_dot>& crossPoint);
    static bool ifLineCross(vector<map_dot>& line1, vector<map_dot>& line2);

private:
    int num_cirTraj = 5;                // Number of circle references.
    int num_segTraj = 21;               // Number of segmented references.

    void readCir(int fileNum);          // Read a circle reference and save it to allTraj_cir.
    void readSeg(int fileNum);          // Read a segmented reference and save it to allTraj_cir.
    static map_dot mapTrans(map_dot dot);       // Transform the dot, in order to match with the information of obstacle cars.

    // Find all references that succeeded by the input reference "traj".
    // The reference found will be recorded in "trajSegRec" and "trajCirRec".
    bool findMatchRef(TrajCir traj, vector<TrajSeg>& trajSegRec, vector<TrajCir>& trajCirRec);
    bool findMatchRef(TrajSeg traj, vector<TrajSeg>& trajSegRec, vector<TrajCir>& trajCirRec);

    void establishRelation_succeeding();    // Establish the succeeding relationship.
    void establishRelation_conflict();      // Establish the conflict relationship.
};

#endif //LANELET2_MAPREADER_H