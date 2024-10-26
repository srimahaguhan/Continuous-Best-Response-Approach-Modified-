#pragma once

#include <iostream>
#include <cmath>
#include <boost/heap/fibonacci_heap.hpp>
#include <boost/algorithm/string.hpp>
#include <boost/algorithm/string/trim_all.hpp>
#include <cassert>
#include <fstream>
#include <string>
#include <limits>
#include <functional> // For std::hash

using namespace std;
using boost::heap::fibonacci_heap;
using boost::heap::compare;

class Node {
public:
    int loc;
    double g_val;
    double h_val = 0;
    Node* parent;
    int timestep = 0;
    int num_internal_conf = 0;  // used in ECBS
    bool in_openlist = false;

    ///////////////////////////////////////////////////////////////////////////
    // NOTE -- Normally, compare_node (lhs,rhs) suppose to return true if lhs<rhs.
    //         However, Heaps in STL and Boost are implemented as max-Heap.
    //         Hence, to achieve min-Head, we return true if lhs>rhs
    ///////////////////////////////////////////////////////////////////////////

    // Compare nodes in the OPEN list
    struct compare_node {
        // Returns true if n1 > n2 (note -- this gives us *min*-heap).
        bool operator()(const Node* n1, const Node* n2) const {
            if (n1->g_val + n1->h_val == n2->g_val + n2->h_val)
                return n1->g_val <= n2->g_val;  // Break ties towards larger g_vals
            return n1->g_val + n1->h_val >= n2->g_val + n2->h_val;
        }
    };

    // Compare nodes in the FOCAL list
    struct secondary_compare_node {
        // Returns true if n1 > n2
        bool operator()(const Node* n1, const Node* n2) const {
            if (n1->num_internal_conf == n2->num_internal_conf) {
                if (n1->g_val + n1->h_val == n2->g_val + n2->h_val)
                    return n1->g_val <= n2->g_val;  // Break ties towards larger g_vals
                return n1->g_val + n1->h_val >= n2->g_val + n2->h_val;  // Prefer shorter solutions
            }
            return n1->num_internal_conf >= n2->num_internal_conf;  // Prefer fewer conflicts
        }
    };

    // Define typedefs for heap handles
    typedef boost::heap::fibonacci_heap<Node*, compare<compare_node>>::handle_type open_handle_t;
    typedef boost::heap::fibonacci_heap<Node*, compare<secondary_compare_node>>::handle_type focal_handle_t;

    open_handle_t open_handle;
    focal_handle_t focal_handle;

    // Constructor and default initialization
    Node(int loc, double g_val, double h_val, Node* parent, int timestep, int num_internal_conf = 0, bool in_openlist = false)
        : loc(loc), g_val(g_val), h_val(h_val), parent(parent), timestep(timestep),
          num_internal_conf(num_internal_conf), in_openlist(in_openlist) {}
    Node() : loc(0), g_val(0), h_val(0), parent(nullptr), timestep(0), num_internal_conf(0), in_openlist(false) {}
    Node(const Node& other) = default;
    ~Node() = default;

    inline double getFVal() const { return g_val + h_val; }

    // Equality comparison for hashing
    struct eqnode {
        bool operator()(const Node* s1, const Node* s2) const {
            return (s1 == s2) || (s1 && s2 && s1->loc == s2->loc && s1->timestep == s2->timestep);
        }
    };

    // Hashing function for Node
    struct NodeHasher {
        std::size_t operator()(const Node* n) const {
            size_t id_hash = std::hash<int>()(n->loc);
            size_t timestep_hash = std::hash<int>()(n->timestep);
            return id_hash ^ (timestep_hash << 1);
        }
    };
};

// Stream output operator for Node
inline std::ostream& operator<<(std::ostream& os, const Node& n) {
    os << "Node(loc: " << n.loc << ", g_val: " << n.g_val << ", h_val: " << n.h_val
       << ", timestep: " << n.timestep << ")";
    return os;
}
